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
#define GPS_GROUND_SPEED_THD 2 // m/s (example value, replace with actual accuracy)


//define Kalman filter instance
KalmanFilter kf;




using namespace BLA;

void setup() {

 // begin the serial communication
  Serial.begin(115200); // USB serial for debug
  delay(1000);

  // begin the I2C communication
  Wire.begin();

  // Initialize the IMU
  Serial.println("Initializing IMU...");
  imu.init(2, 250); // 2g accelerometer, 250°/s gyroscope
  imu.calibrateAccel(); // Calibrate accelerometer
  imu.calibrateGyro(); // Calibrate gyroscope
  Serial.println("IMU initialized!");
  delay(1000);

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

// Keep track of time across loop iterations
unsigned long lastTime = 0;

void loop() {

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  



  // define accelerometer and gyroscope variables
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw =0.0f; // Initialize euler angles

  // read the IMU data
  imu.readAccel(ax, ay, az); // Read accelerometer data
  imu.readGyro(gx, gy, gz); // Read gyroscope data

  
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

  if (myGNSS.getPVT(10)) { // Wait for GNSS data
    // Get GNSS data
    float headingGNSS = myGNSS.getHeading(10)/1e5;      // degrees
    float headingAcc = myGNSS.getHeadingAccEst(10) / 100000.0f; // degrees

    float gnssGroundSpeed = myGNSS.getGroundSpeed(10); // degrees (example value, replace with actual accuracy)

    // Update Kalman filter with GNSS heading
    if ((float) myGNSS.getGroundSpeed(10)/1000.0f > (float)GPS_GROUND_SPEED_THD) { // Check if speed is above threshold
      kf.updateWithGnss(headingGNSS, headingAcc); // Update Kalman filter with GNSS data
      Serial.println("[INFO] Kalman filter updated with GNSS data.");
    } else {
      Serial.println("[INFO] GNSS speed too low, skipping update.");
    }
  }
  
  delay(10);
  lastTime = currentTime;

} // End of loop function




