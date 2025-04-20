// inclue the libraries
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <MPU6050.h>
#include <kf.h>

// define IMU instance
MPU6050 imu;

// define GPS instance
SFE_UBLOX_GNSS myGNSS;

//define Kalman filter instance
KalmanFilter kf;


using namespace BLA;

void setup() {
 // begin the serial communication
  Serial.begin(115200); // USB serial for debug
  delay(1000);
  // begin the I2C communication
  Wire.begin();
  }

void loop() {

  // define accelerometer and gyroscope variables
  float ax, ay, az;
  float gx, gy, gz;
  BLA::Matrix<3> eulerAngles = {0.0f, 0.0f, 0.0f}; // Initialize euler angles
  // read the IMU data
  imu.readAccel(ax, ay, az); // Read accelerometer data
  imu.readGyro(gx, gy, gz); // Read gyroscope data
  float dt = 0.01f; // Time step in seconds

  
  // define acceleration and gyroscope matrices
  BLA::Matrix<3, 1> accel = {ax, ay, az};
  BLA::Matrix<3, 1> gyro =   {gx, gy, gz}; 

  // convert gyroscope data to radians/s
  gyro = gyro * (float) DEG_TO_RAD; 

  
  //perform kalman filter prediction and update
  kf.predict(gyro, 0.01f); // Predict next state
  kf.update(accel); // Update Kalman filter with

  // Get Euler angles in degrees
  eulerAngles = kf.getEulerAnglesDeg(); 
  
  delay(10);
} // End of loop function




