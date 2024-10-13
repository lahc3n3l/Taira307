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
# Initialize IMU I2C, 
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

    # read from Accel
    accelData = i2c.readfrom_mem(MPU6050_ADDR, ACCEL_XOUT_H, 6)
    # Convert the data
    ax, ay, az = struct.unpack('>hhh', accelData)
    aScale = 16384 / 2 ** ACCEL_FS
    # convert to deg/s:
    ax /= aScale
    ay /= aScale
    az /= aScale

    return np.array([ax, ay, az]), np.array([gx, gy, gz])


def calibrateIMU(n):
    gyroBiases = np.array([0, 0, 0])
    accelBiases = np.array([0, 0, 0])
    for i in range(n):
        a, g = readIMU()
        a[2] -= 1
        gyroBiases += g
        accelBiases += a
        utime.sleep_ms(1)
    return accelBiases / n, gyroBiases / n



def calculate_accel_angles(ax, ay, az):
    roll = math.atan2(ay, az) * 180 / math.pi
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180 / math.pi
    return roll, pitch



# Main
configureIMU()
accelBiases, gyroBiases = calibrateIMU(2000)

gyro_angle_x = gyro_angle_y = gyro_angle_z = 0
last_time = utime.ticks_us()
alpha = 0.70

while True:
    ax, ay, az = np.array(readIMU()[0]) - accelBiases
    gx, gy, gz = np.array(readIMU()[1]) - gyroBiases

    current_time = utime.ticks_us()
    dt = (current_time - last_time) / 1000000  # Convert to seconds
    last_time = current_time

    accel_roll, accel_pitch = calculate_accel_angles(ax, ay, az)

    # Integrate gyro data
    gyro_angle_x += gx * dt
    gyro_angle_y += gy * dt
    gyro_angle_z += gz * dt

    roll = alpha * (gyro_angle_x) + (1 - alpha) * accel_roll
    pitch = alpha * (gyro_angle_y) + (1 - alpha) * accel_pitch
    yaw = gyro_angle_z
    
    gyro_angle_x = roll
    gyro_angle_y = pitch
    


    print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
    utime.sleep_ms(80)

