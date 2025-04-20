// inclue the libraries
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <MPU6050.h>
#include <flightController.h>
#include <PIDController.h>
#include <ServoController.h>
#include <CommandReader.h>
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

// SERVO pins for left, right, and pitch servos
#define LEFT_SERVO_PIN 12
#define RIGHT_SERVO_PIN 13
#define PITCH_SERVO_PIN 14

// Receiver pins for roll, pitch, and flaps
#define ROLL_PIN 2  // ESP32 GPIO 2
#define PITCH_PIN 4 // ESP32 GPIO 4
#define FLAPS_PIN 5   // ESP32 GPIO 5

FlightController flightController;
ServoController servoController(LEFT_SERVO_PIN, RIGHT_SERVO_PIN, PITCH_SERVO_PIN); 
CommandReader commandReader(ROLL_PIN, PITCH_PIN, FLAPS_PIN);

//define Kalman filter instance
KalmanFilter kf;

// Timing variables
unsigned long previousTime = 0;
float dt = 0.0f;

float currentRoll = 0.0f;
float currentPitch = 0.0f;
float flapsAngle = 0.0f; 



using namespace BLA;

void setup() {

 // begin the serial communication
  Serial.begin(115200); // USB serial for debugging
  delay(100);
  Serial.println("Starting up...");

  // begin the I2C communication
  Wire.begin();

  // 
  // IMU SETUP
  //    

  Serial.println("Initializing IMU...");
  imu.init(2, 250); // 2g accelerometer, 250°/s gyroscope
  imu.calibrateAccel(); // Calibrate accelerometer
  imu.calibrateGyro(); // Calibrate gyroscope
  Serial.println("IMU initialized!");
  delay(1000);

  //
  // GPS SETUP
  //  

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
  
  //
  // COMMAND READER SETUP
  //

  // Initialize the command reader (RC receiver interface)
  commandReader.begin();
  // Configure command reader parameters
  commandReader.setRollMaxDegrees(40.0f);   // Maximum roll angle in degrees
  commandReader.setPitchMaxDegrees(40.0f);  // Maximum pitch angle in degrees
  commandReader.setFlapsMaxDegrees(45.0f);  // Maximum flaps angle in degrees
  commandReader.setDeadband(2.0f);          // Deadband to prevent jitter

  servoController.begin(); // Initialize servos
  servoController.setMaxDeflection(60.0f);
  servoController.setLeftTrim(0.0f);
  servoController.setRightTrim(0.0f); 
  servoController.setPitchTrim(0.0f);
  // Set to neutral position
  servoController.setToNeutral();
  Serial.println("Servo controller initialized");

  //
  // FLIGHT CONTROLLER SETUP
  //

  // Initialize flight controller
  Serial.println("Initializing flight controller...");
  flightController.setRollPID(1.2f, 0.0f, 0.0f);
  flightController.setPitchPID(1.2f, 0.0f, 0.0f);
  Serial.println("Flight Controller Ready");
  
  // Initialize timing
  previousTime = millis();

}

// Keep track of time across loop iterations
unsigned long lastTime = 0;

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0f;  // Convert to seconds
  previousTime = currentTime;
  
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

    // print GNSS data
    Serial.print("GNSS Heading: ");
    Serial.print(headingGNSS); // degrees
    Serial.print(" | GNSS Heading Accuracy: ");
    Serial.print(headingAcc); // degrees
    // print lat, lon, alt, speed
    Serial.print(" | Latitude: ");
    Serial.print(myGNSS.getLatitude(10) / 1e7); // degrees
    Serial.print(" | Longitude: ");
    Serial.print(myGNSS.getLongitude(10) / 1e7); // degrees
    Serial.print(" | Altitude: ");
    Serial.println(myGNSS.getAltitude(10) / 1000.0f); // meters
   
  

    float gnssGroundSpeed = myGNSS.getGroundSpeed(10); // degrees (example value, replace with actual accuracy)

    // Update Kalman filter with GNSS heading
    if ((float) myGNSS.getGroundSpeed(10)/1000.0f > (float)GPS_GROUND_SPEED_THD) { // Check if speed is above threshold
      kf.updateWithGnss(headingGNSS, headingAcc); // Update Kalman filter with GNSS data
      Serial.println("[INFO] Kalman filter updated with GNSS data.");
    } else {
      Serial.println("[INFO] GNSS speed too low, skipping update.");
    }
  }
  

  //
  // GET ROLL AND PITCH FLAPS COMMANDS
  //
  
  float currentRoll = roll; // Get current roll from Kalman filter
  float currentPitch = pitch; // Get current pitch from Kalman filter

    // Update commands from receiver
    commandReader.update();
  
    // Get the roll, pitch, and flaps commands
    float rollCommand = commandReader.getRollCommand();
    float pitchCommand = commandReader.getPitchCommand();
    float flapsCommand = commandReader.getFlapsCommand();

    // Check for signal validity
    bool signalValid = commandReader.isSignalValid(); 
    if (signalValid) {
      // Update the flight controller with the commands
      flightController.setTargetRoll(rollCommand);
      flightController.setTargetPitch(pitchCommand);
    } 
  
    float right_servo_angle = 0.0f;
    float left_servo_angle = 0.0f;
    float pitch_servo_angle = 0.0f;
  
    flightController.updateControlSurfaces(
      currentRoll, 
      currentPitch, 
      flapsCommand, 
      dt, 
      right_servo_angle, 
      left_servo_angle, 
      pitch_servo_angle
    );
  
    // Update servos with calculated angles
    servoController.updateServos(right_servo_angle, left_servo_angle, pitch_servo_angle);

  lastTime = currentTime;

} // End of loop function

