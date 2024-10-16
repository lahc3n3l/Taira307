import machine
import utime
import math
import struct
from ulab import numpy as np

# Constants
MPU6050_ADDR = 0x68
BMP280_ADDR = 0x76
# some MPU6050 Registers and their Address
PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
DLPF_CONFIG = 3
#Gyro configuration
GYRO_CONFIG = 0x1B
GYRO_XOUT_H = 0x43
GYRO_FS = 2
#Accelerometer configuration
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_FS = 2
# Barometer BMP280
PRESS_MSB = 0xF7

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


# Main
configureIMU()
accelBiases, gyroBiases = calibrateIMU(2000)
while True:
    ax, ay, az = np.array(readIMU()[0]) - accelBiases
    gx, gy, gz = np.array(readIMU()[1]) - gyroBiases
    print(ax, ay, az)
    utime.sleep_ms(80)
