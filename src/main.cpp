#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
// include wire.h for I2C communication
#include <Wire.h>
// include IMU library
#include <MPU6050.h>
#include <ekf.h>
#include <flightController.h>
#include <PIDController.h>
#include <ServoController.h>

FlightController flightController;

// SERVO pins for left, right, and pitch servos
#define LEFT_SERVO_PIN 9
#define RIGHT_SERVO_PIN 10
#define PITCH_SERVO_PIN 11

ServoController servoController(LEFT_SERVO_PIN, RIGHT_SERVO_PIN, PITCH_SERVO_PIN); 

// Timing variables
unsigned long previousTime = 0;
float dt = 0.0f;

float currentRoll = 0.0f;
float currentPitch = 0.0f;
float flapsAngle = 0.0f; 

void setup() {

  Serial.begin(115200); // USB serial for debug
  delay(100);
  Serial.println("Starting up...");

  servoController.setMaxDeflection(40.0f);
  servoController.setLeftTrim(0.0f);
  servoController.setRightTrim(0.0f); 
  servoController.setPitchTrim(0.0f);

  // Initialize flight controller
  Serial.println("Initializing flight controller...");
  flightController.setRollPID(1.2f, 0.1f, 0.05f);
  flightController.setPitchPID(1.0f, 0.0f, 0.02f);
  flightController.setRollTrim(0.0f);
  flightController.setPitchTrim(0.0f);

  // Set to neutral position
  servoController.setToNeutral();
  Serial.println("Servo controller initialized");
  
  // Initialize timing
  previousTime = millis();
  
  };

void loop() {

  unsigned long currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0f;  // Convert to seconds
  previousTime = currentTime;

  float right_servo_angle = 0.0f; // Placeholder for right servo angle
  float left_servo_angle = 0.0f; // Placeholder for left servo angle
  float pitch_servo_angle = 0.0f; // Placeholder for pitch servo angle

  flightController.updateControlSurfaces(
    currentRoll, 
    currentPitch, 
    flapsAngle, 
    dt, 
    right_servo_angle, 
    left_servo_angle, 
    pitch_servo_angle
  );

  // Update servos with calculated angles
  servoController.updateServos(right_servo_angle, left_servo_angle, pitch_servo_angle);
};
