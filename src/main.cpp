// inclue the libraries
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <MPU6050.h>
#include <kf.h>

// define IMU instance
MPU6050 imu;

// UART pins for GPS
#define GPS_RX_PIN 16  // ESP32 RX <- GPS TX
#define GPS_TX_PIN 17  // ESP32 TX -> GPS RX
#define GPS_BAUD 115200 // GPS baud rate
HardwareSerial gpsSerial(2); // UART2  

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
  // ---------------- GPS Module Setup ----------------
  Serial.println("Initializing GPS module...");
  // Begin UART communication with GPS
  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (myGNSS.begin(gpsSerial)) {
    Serial.println("GNSS module connected!");
  } else {
    Serial.println("Failed to connect to GNSS module.");
  }

  // Optional tuning
  myGNSS.setNavigationFrequency(10); // 1 Hz updates
  myGNSS.setAutoPVT(true); // Enable automatic PVT messages
}

void loop() {

  // define accelerometer and gyroscope variables
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw =0.0f; // Initialize euler angles
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
  kf.predict(gyro, dt); // Predict next state
  kf.update(accel); // Update Kalman filter with

  // Get Euler angles in degrees
  kf.getEulerAnglesDeg(roll, pitch, yaw);

  delay(10);

} // End of loop function




