import machine
import utime
import math
import struct
from ulab import numpy as np

# Constants
MPU6050_ADDR = 0x68
# some MPU6050 Registers and their Address
PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
DLPF_CONFIG = 3

GYRO_CONFIG = 0x1B
GYRO_XOUT_H = 0x43
GYRO_FS = 2

ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_FS = 2

DEG_TO_RAD = math.pi/180;
# Initialize IMU I2C
i2c = machine.I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=400000)

def configureIMU():
    # 1. Set power:
    i2c.writeto_mem(MPU6050_ADDR, PWR_MGMT_1, b'\x00')
    # 2. Activate low-pass filter:
    i2c.writeto_mem(MPU6050_ADDR, CONFIG, bytes([DLPF_CONFIG]))  # good for airplane
    # 3. Configure full scale range: 0: +-250 LSB/°/s, 1: +-500 LSB/°/s, 2: +-1000 LSB/°/s, 3: +2000 LSB/°/s
    i2c.writeto_mem(MPU6050_ADDR, GYRO_CONFIG, bytes([GYRO_FS << 3]))
    # 4. Configure full scale range: 0: +-2g , 1: +-4g , 2: +-8g , 3: +-16g
    i2c.writeto_mem(MPU6050_ADDR, ACCEL_CONFIG, bytes([ACCEL_FS << 3]))

def readIMU():
    # read from Gyro
    gyroData = i2c.readfrom_mem(MPU6050_ADDR, GYRO_XOUT_H, 6)
    # Convert the data
    gx, gy, gz = struct.unpack('>hhh', gyroData)
    gScale = 131 / 2 ** GYRO_FS
    # convert to deg/s:
    gx /= gScale
    gy /= gScale
    gz /= gScale
    
    # convert to rad/s:
    gx *= DEG_TO_RAD 
    gy *= DEG_TO_RAD
    gz *= DEG_TO_RAD
    
    # read from Accel
    accelData = i2c.readfrom_mem(MPU6050_ADDR, ACCEL_XOUT_H, 6)
    # Convert the data
    ax, ay, az = struct.unpack('>hhh', accelData)
    aScale = 16384 / 2 ** ACCEL_FS
    # convert to g:
    ax /= aScale
    ay /= aScale
    az /= aScale

    return np.array([ax, ay, az]), np.array([gx, gy, gz])

def calibrateIMU(n):
    print("Satrting calibration, keep the IMU still ....")
    gyroBiases = np.array([0, 0, 0])
    gyroSecondOrderMoment = np.array([0, 0, 0])
    
    accelBiases = np.array([0, 0, 0])
    accelSecondOrderMoment = np.array([0, 0, 0])
    for i in range(n):
        a, g = readIMU()
        a[2] -= 1
        gyroBiases += g
        accelBiases += a
        
        gyroSecondOrderMoment  += (DEG_TO_RAD* g)**2
        accelSecondOrderMoment += a**2
        utime.sleep_ms(1)
    print("Calibration finished")
    return accelBiases / n, accelSecondOrderMoment - (accelBiases / n)**2 , gyroBiases / n, gyroSecondOrderMoment -  (gyroBiases / n)**2

class KalmanFilter:
    def __init__(self):
        self.A = np.eye(3)
        self.H = np.array([[1, 0, 0],[0, 1, 0]])
        self.Q = np.eye(3) * (1)**2
        self.R = np.eye(2) * (0.5)**2
        self.P = np.eye(3) * (2)**2
        self.x = np.array([1.4,1.4, 1])#np.zeros(3)

    def predict(self, u, dt):
        self.B = np.eye(3)*dt
        self.Q = np.dot(self.B, self.B.transpose()) * (2 *DEG_TO_RAD)**2
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.transpose()) + self.Q

    def update(self, accel):
        y = np.array([math.atan2(accel[1], math.sqrt(accel[0]**2 + accel[2]**2)), math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))])
        S = np.dot(np.dot(self.H, self.P), self.H.transpose()) + self.R
        K = np.dot(np.dot(self.P, self.H.transpose()), np.linalg.inv(S))
        self.x += np.dot(K, (y - np.dot(self.H, self.x)))
        self.P = np.dot((np.eye(3) - np.dot(K, self.H)), self.P)


# Main
configureIMU()
accelBiases, gyroBiases = calibrateIMU(2000)
kf = KalmanFilter()  # 80ms sampling rate
lastTime = utime.ticks_us()
while True:
    ax, ay, az = np.array(readIMU()[0]) - accelBiases
    gx, gy, gz = np.array(readIMU()[1]) - gyroBiases
    
    gyro = DEG_TO_RAD * np.array([gx, gy, gz])
    currentTime = utime.ticks_us()
    dt = (currentTime - lastTime)/1e6
    lastTime = currentTime
    kf.predict(gyro, dt)
    kf.update(np.array([ax, ay, az]))

    roll, pitch, yaw = kf.x

    print(f"Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}°, Yaw: {math.degrees(yaw):.2f}°")
    utime.sleep_ms(10)
