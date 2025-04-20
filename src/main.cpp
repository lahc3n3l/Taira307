#include <Arduino.h>
// include wire.h for I2C communication
#include <Wire.h>
// include IMU library
#include <MPU6050.h>
#include <kf.h>
MPU6050 imu;

#define IMU_TEST


KalmanFilter kf;
using namespace BLA;

void setup() {

  Serial.begin(115200); // USB serial for debug
  delay(1000);
  #ifdef IMU_TEST
  Wire.begin();

  imu.init(2, 250);  // 2g accel, 250 deg/s gyro*
  Serial.println("Calibrating accelerometer...");
  imu.calibrateAccel(); // Calibrate accelerometer
  Serial.println("Calibrating gyroscope...");
  imu.calibrateGyro(); // Calibrate gyroscope
  float ax, ay, az;
  imu.readAccel(ax, ay, az); // Read accelerometer data


  delay(100);

  
    // ---------- End GPS Module Setup ----------
  }

void loop() {

  // ---------------- GY87 Sensor Test ----------------
  #ifdef IMU_TEST

  float ax, ay, az;
  float gx, gy, gz;
  
  imu.readAccel(ax, ay, az); // Read accelerometer data
  imu.readGyro(gx, gy, gz); // Read gyroscope data
  BLA::Matrix<3, 1> accel = {ax, ay, az};
  BLA::Matrix<3, 1> gyro =   {gx, gy, gz}; 
  gyro = gyro * (float) DEG_TO_RAD; // Convert to radians/s

  kf.predict(gyro, 0.01f); // Predict next state
  kf.update(accel); // Update Kalman filter with
  float dt = 0.01f;



delay(10);
  float roll, pitch, yaw;
  kf.getEulerAnglesDeg(roll, pitch, yaw); // Get Euler angles in degrees
  Serial.print("Roll:");
  Serial.print(roll);
  Serial.print(" Pitch:");
  Serial.print(pitch);
  Serial.print(" Yaw:");
  Serial.println(yaw);
  
  #endif

} // End of loop function

#endif



