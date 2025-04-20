#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoController {
private:
    // Servo objects
    Servo leftServo;
    Servo rightServo;
    Servo pitchServo;
    
    // Pin definitions
    uint8_t leftServoPin;
    uint8_t rightServoPin;
    uint8_t pitchServoPin;
    
    // Servo configuration
    float servoNeutral;        // Neutral position (degrees)
    float leftTrim;            // Left servo trim (degrees)
    float rightTrim;           // Right servo trim (degrees)
    float pitchTrim;           // Pitch servo trim (degrees)
    float maxDeflection;       // Maximum deflection from neutral (degrees)
    
    // Current servo positions
    float leftPosition;
    float rightPosition;
    float pitchPosition;
    
    // Servo limiting and safety
    bool limitsEnabled;        // Enable servo movement limits
    uint32_t lastUpdateTime;   // For rate limiting
    float maxMoveRate;         // Maximum degrees per second
    
    // Helper method to apply limits to a value
    float applyLimits(float value, float min, float max);
    
    // Helper method to apply rate limiting
    float applyRateLimit(float current, float target, float dt);

public:
    // Constructor
    ServoController(uint8_t leftPin, uint8_t rightPin, uint8_t pitchPin);
    
    // Initialize servos
    bool begin();
    
    // Set servo trims
    void setLeftTrim(float trim);
    void setRightTrim(float trim);
    void setPitchTrim(float trim);
    
    // Set neutral position (default is 90 degrees)
    void setNeutral(float position);
    
    // Set maximum deflection from neutral
    void setMaxDeflection(float degrees);
    
    // Set maximum movement rate (degrees per second)
    void setMaxMoveRate(float rate);
    
    // Enable/disable servo limits
    void enableLimits(bool enable);
    
    // Update servo positions
    void updateServos(float rightAngle, float leftAngle, float pitchAngle);
    
    // Set all servos to neutral position
    void setToNeutral();
    
    // Get current servo positions
    float getLeftPosition() const;
    float getRightPosition() const;
    float getPitchPosition() const;
    
    // Check if servos are attached
    bool isAttached();
    
    // Detach servos (to disable output)
    void detach();
    
    // Reattach servos
    void reattach();
};

#endif // SERVO_CONTROLLER_H