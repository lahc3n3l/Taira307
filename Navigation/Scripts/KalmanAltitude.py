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

    accelMean = accelBiases / n
    gyroMean = gyroBiases / n

    accelVariance = (accelSecondOrderMoment / n) - accelMean ** 2
    gyroVariance = (gyroSecondOrderMoment / n) - gyroMean ** 2
    return accelMean, accelVariance, gyroMean, gyroVariance

class KalmanFilter:
    def __init__(self, dt):
        self.A = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, dt],[0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0],[0, 1, 0],[0, 0, 1, 0]])
        self.Q = np.eye(4) * (1)**2
        self.R = np.array([[(0.2)**2, 0, 0],[0, (0.2)**2, 0],[0, 0, 1]])
        self.P = np.eye(4) * (10)**2
        self.x = np.zeros(4) # [theta, phi, h, Vz]

    def predict(self, u, dt):
        self.B = np.array([[dt, 0, 0], [0, dt, 0], [0, 0, 0.5 * dt**2], [0, 0, dt]])
        self.Q = np.dot(self.B, self.B.transpose()) * (8 *DEG_TO_RAD)**2
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.transpose()) + self.Q

    def update(self, accel, h):
        y = np.array([math.atan2(accel[1], math.sqrt(accel[0]**2 + accel[2]**2)), math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)), hm])
        S = np.dot(np.dot(self.H, self.P), self.H.transpose()) + self.R
        K = np.dot(np.dot(self.P, self.H.transpose()), np.linalg.inv(S))
        self.x += np.dot(K, (y - np.dot(self.H, self.x)))
        self.P = np.dot((np.eye(4) - np.dot(K, self.H)), self.P)





