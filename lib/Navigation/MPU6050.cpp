#include "MPU6050.h"

MPU6050::MPU6050(uint8_t address) : _address(address) {}

void MPU6050::init(uint8_t accel_scale, uint16_t gyro_scale) {
    Wire.begin();

    writeRegister(PWR_MGMT_1, 0x00); // Wake up

    uint8_t accel_cfg = (accel_scale == 2) ? 0x00 :
                        (accel_scale == 4) ? 0x08 :
                        (accel_scale == 8) ? 0x10 : 0x18;

    uint8_t gyro_cfg = (gyro_scale == 250) ? 0x00 :
                       (gyro_scale == 500) ? 0x08 :
                       (gyro_scale == 1000) ? 0x10 : 0x18;

    writeRegister(ACCEL_CONFIG, accel_cfg);
    writeRegister(GYRO_CONFIG, gyro_cfg);
    writeRegister(INT_PIN_CFG, 0x02); // I2C bypass

    _accel_scale = (accel_scale == 2) ? 16384.0f :
                   (accel_scale == 4) ? 8192.0f :
                   (accel_scale == 8) ? 4096.0f : 2048.0f;

    _gyro_scale = (gyro_scale == 250) ? 131.0f :
                  (gyro_scale == 500) ? 65.5f :
                  (gyro_scale == 1000) ? 32.8f : 16.4f;
}

void MPU6050::readAccel(float &ax, float &ay, float &az) {
    uint8_t buffer[6];
    readRegisters(ACCEL_XOUT_H, buffer, 6);

    int16_t x = (int16_t)(buffer[0] << 8 | buffer[1]);
    int16_t y = (int16_t)(buffer[2] << 8 | buffer[3]);
    int16_t z = (int16_t)(buffer[4] << 8 | buffer[5]);

    float g = 9.80665f;
    ax = ((x / _accel_scale) - _accel_offsets[0]) * g;
    ay = ((y / _accel_scale) - _accel_offsets[1]) * g;
    az = ((z / _accel_scale) - _accel_offsets[2]) * g;
}

void MPU6050::readGyro(float &gx, float &gy, float &gz) {
    uint8_t buffer[6];
    readRegisters(GYRO_XOUT_H, buffer, 6);

    int16_t x = (int16_t)(buffer[0] << 8 | buffer[1]);
    int16_t y = (int16_t)(buffer[2] << 8 | buffer[3]);
    int16_t z = (int16_t)(buffer[4] << 8 | buffer[5]);

    float deg_to_rad = 3.14159265f / 180.0f;
    gx = ((x / _gyro_scale) - _gyro_offsets[0]) * deg_to_rad;
    gy = ((y / _gyro_scale) - _gyro_offsets[1]) * deg_to_rad;
    gz = ((z / _gyro_scale) - _gyro_offsets[2]) * deg_to_rad;
}


void MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void MPU6050::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_address, length);
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
}
