import machine
import utime
import math
import struct
from ulab import numpy as np

# Constants
MPU6050_ADDR = 0x68
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
GYRO_XOUT_H  = 0x43
GYRO_FS 	 = 2
DLPF_CONFIG	 = 3

# Initialize IMU I2C, 
i2c = machine.I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=400000)

def configureIMU():
    # set power:
    i2c.writeto_mem(MPU6050_ADDR, PWR_MGMT_1, b'\x00')
    #Activate low-pass filter:
    i2c.writeto_mem(MPU6050_ADDR, CONFIG, bytes([DLPF_CONFIG])) # good for airplane
    #2. configure full scale range: 0: +-250 LSB/°/s, 1: +-500 LSB/°/s, 2: +-1000 LSB/°/s, 3: +2000 LSB/°/s
    i2c.writeto_mem(MPU6050_ADDR, GYRO_CONFIG, bytes([GYRO_FS<<3]))
    
def readGyro():
    #read from
    gyroData = i2c.readfrom_mem(MPU6050_ADDR,GYRO_XOUT_H, 6)
    # Convert the data
    gx, gy, gz = struct.unpack('>hhh', gyroData)
    scale = 131 /2**GYRO_FS
    # convert to deg/s:
    gx /= scale
    gy /= scale
    gz /= scale
    return gx,gy,gz

def calibrateGyro(n):
    gyroBiases = np.array([0,0,0]) 
    for i in range(n):
        gx,gy,gz = readGyro()
        gyroBiases += np.array([gx,gy,gz])
        utime.sleep_ms(1)
    return gyroBiases / n

# Main

configureIMU()
gyroBiases = calibrateGyro(2000)
lastTime = utime.ticks_us
theta = 0
while True:
    x, y, z = np.array(readGyro())- gyroBiases
    dt = utime.ticks_us
    theta += y *dt
    print(x, y, z)
    utime.sleep_ms(80)