#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ekf.h>
#include <flightController.h>
#include <PIDController.h>
#include <ServoController.h>
#include <CommandReader.h>



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

  // Initialize the command reader (RC receiver interface)
  commandReader.begin();
  // Configure command reader parameters
  commandReader.setRollMaxDegrees(40.0f);   // Maximum roll angle in degrees
  commandReader.setPitchMaxDegrees(40.0f);  // Maximum pitch angle in degrees
  commandReader.setFlapsMaxDegrees(45.0f);  // Maximum flaps angle in degrees
  commandReader.setDeadband(2.0f);          // Deadband to prevent jitter

  servoController.begin(); // Initialize servos
  servoController.setMaxDeflection(40.0f);
  servoController.setLeftTrim(0.0f);
  servoController.setRightTrim(0.0f); 
  servoController.setPitchTrim(0.0f);
  // Set to neutral position
  servoController.setToNeutral();
  Serial.println("Servo controller initialized");

  // Initialize flight controller
  Serial.println("Initializing flight controller...");
  flightController.setRollPID(1.2f, 0.0f, 0.05f);
  flightController.setPitchPID(1.2f, 0.0f, 0.02f);


  
  Serial.println("Flight Controller Ready");
  
  // Initialize timing
  previousTime = millis();
  
  };

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0f;  // Convert to seconds
  previousTime = currentTime;

  // Update commands from receiver
  commandReader.update();
  
  // Get the roll, pitch, and flaps commands
  float rollCommand = commandReader.getRollCommand();
  float pitchCommand = commandReader.getPitchCommand();
  float flapsCommand = commandReader.getFlapsCommand();
  // // print the commands for debugging
  // Serial.print("Roll Command: ");
  // Serial.print(rollCommand);
  // Serial.print(" Pitch Command: ");
  // Serial.print(pitchCommand);
  // Serial.print(" Flaps Command: ");
  // Serial.println(flapsCommand);

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
};
