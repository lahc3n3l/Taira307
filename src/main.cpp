#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ekf.h>
#include "CommandReader.h"



// Define receiver pins (using ESP32 GPIO pins with interrupt support)
#define ROLL_PIN 2  // ESP32 GPIO 2
#define PITCH_PIN 4 // ESP32 GPIO 4
#define FLAPS_PIN 5 // ESP32 GPIO 5

// Create CommandReader instance
CommandReader commandReader(ROLL_PIN, PITCH_PIN, FLAPS_PIN);



// Debug flag to enable/disable detailed debug output
bool enableDebug = true;

// Flag for calibration mode
bool calibrationMode = false;

// Timing variables
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 500; // 500ms = 2Hz display rate

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Flight Controller Starting...");
  
  // Allow some time for devices to initialize
  delay(1000);
  
  // Initialize the command reader (RC receiver interface)
  commandReader.begin();
  
  // Configure command reader parameters
  commandReader.setRollMaxDegrees(30.0f);   // Maximum roll angle in degrees
  commandReader.setPitchMaxDegrees(30.0f);  // Maximum pitch angle in degrees
  commandReader.setFlapsMaxDegrees(45.0f);  // Maximum flaps angle in degrees
  commandReader.setDeadband(2.0f);          // Deadband to prevent jitter
  
  
  Serial.println("Flight Controller Ready");
  Serial.println("Send 'c' to enter calibration mode");
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'c' || cmd == 'C') {
      calibrationMode = !calibrationMode;
      if (calibrationMode) {
        Serial.println("Calibration Mode ON");
        Serial.println("Move sticks to extremes and center");
      } else {
        Serial.println("Calibration Mode OFF");
        // Apply calibration if needed
      }
    } else if (cmd == 'd' || cmd == 'D') {
      enableDebug = !enableDebug;
      Serial.print("Debug output: ");
      Serial.println(enableDebug ? "ON" : "OFF");
    }
  }
  
  // Update commands from receiver
  commandReader.update();
  
  // Get the roll, pitch, and flaps commands
  float rollCommand = commandReader.getRollCommand();
  float pitchCommand = commandReader.getPitchCommand();
  float flapsCommand = commandReader.getFlapsCommand();
  
  // Check for signal validity
  bool signalValid = commandReader.isSignalValid();
  
  // Display information periodically to avoid flooding the serial port
  unsigned long currentTime = millis();
  if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL) {
    lastDisplayTime = currentTime;
    
    if (calibrationMode) {
      // Display raw values for calibration
      Serial.println("--- Calibration Data ---");
      Serial.print("Roll Raw: ");
      Serial.print(commandReader.getRawRoll());
      Serial.print(" | Pitch Raw: ");
      Serial.print(commandReader.getRawPitch());
      Serial.print(" | Flaps Raw: ");
      Serial.println(commandReader.getRawFlaps());
    }
    else if (enableDebug) {
      // Print debug information
      Serial.print("Signal: ");
      Serial.print(signalValid ? "VALID" : "LOST");
      Serial.print(" | Roll: ");
      Serial.print(rollCommand);
      Serial.print(" | Pitch: ");
      Serial.print(pitchCommand);
      Serial.print(" | Flaps: ");
      Serial.println(flapsCommand);
    }
  }
  
  // Handle signal loss
  if (!signalValid) {
    // Reset commands to neutral if signal is lost
    rollCommand = 0.0f;
    pitchCommand = 0.0f;
    flapsCommand = 0.0f;
    
    // Add any additional failsafe behavior here
  }
  
  // Use the commands for your flight control logic here
  // This is where you would integrate with your FlightController class
  
  // Don't make the loop too fast to avoid overwhelming the serial output
  // and to give the interrupts time to capture the RC signals properly
  delay(100); // 100Hz main loop rate
}