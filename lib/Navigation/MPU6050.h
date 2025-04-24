#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>

#define MPU6050_ADDR    0x68
#define PWR_MGMT_1      0x6B
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define INT_PIN_CFG     0x37
#define ACCEL_XOUT_H    0x3B
#define GYRO_XOUT_H     0x43

class MPU6050 {
public:
    MPU6050(uint8_t address = MPU6050_ADDR);

    void init(uint8_t accel_scale = 2, uint16_t gyro_scale = 250);

    void readAccel(float &ax, float &ay, float &az);  // m/s²
    void readGyro(float &gx, float &gy, float &gz);   // rad/s
    void calibrateAccel();
    void calibrateGyro();

private:
    uint8_t _address;

    float _accel_scale;
    float _gyro_scale;

    float _accel_offsets[3] = {0.0f, 0.0f, 0.0f};
    float _gyro_offsets[3] = {0.0f, 0.0f, 0.0f};

    void readRawAccel(float &ax, float &ay, float &az);  // m/s²
    void readRawGyro(float &gx, float &gy, float &gz);   // rad/s
    void writeRegister(uint8_t reg, uint8_t value);
    void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
};

#endif
// MPU6050_H