#include "ServoController.h"

ServoController::ServoController(uint8_t leftPin, uint8_t rightPin, uint8_t pitchPin) {
    // Store pin assignments
    leftServoPin = leftPin;
    rightServoPin = rightPin;
    pitchServoPin = pitchPin;
    
    // Default configuration
    servoNeutral = 90.0f;      // 90 degrees is typically neutral for servos
    leftTrim = 0.0f;           // No trim by default
    rightTrim = 0.0f;          // No trim by default
    pitchTrim = 0.0f;          // No trim by default
    maxDeflection = 40.0f;     // 40 degrees max deflection
    
    // Default positions (neutral)
    leftPosition = servoNeutral;
    rightPosition = servoNeutral;
    pitchPosition = servoNeutral;
    
    // Safety defaults
    limitsEnabled = true;      // Enable limits by default
    lastUpdateTime = 0;        // Initialize timestamp
    maxMoveRate = 300.0f;      // 300 degrees/second max rate
}

bool ServoController::begin() {
    // Initialize ESP32's servo library
    
    ESP32PWM::allocateTimer(0);  // Use first timer for servos
    ESP32PWM::allocateTimer(1);  // Use second timer for servos
    ESP32PWM::allocateTimer(2);  // Use third timer for servos
    ESP32PWM::allocateTimer(3);  // Use fourth timer for servos
    
    // Configure servos with standard pulse widths (adjust if needed for your servos)
    // Arguments are: min pulse width, max pulse width, default position
    leftServo.setPeriodHertz(50);   // Standard 50Hz PWM for servos
    rightServo.setPeriodHertz(50);  // Standard 50Hz PWM for servos
    pitchServo.setPeriodHertz(50);  // Standard 50Hz PWM for servos

    leftServo.attach(leftServoPin, 1000, 2000);
    rightServo.attach(rightServoPin, 1000, 2000);
    pitchServo.attach(pitchServoPin, 1000, 2000);
    
    
    // Set servos to neutral position
    setToNeutral();
    
    // Initialize timing
    lastUpdateTime = millis();
    
    return true;  // All servos initialized successfully
}

void ServoController::setLeftTrim(float trim) {
    leftTrim = trim;
}

void ServoController::setRightTrim(float trim) {
    rightTrim = trim;
}

void ServoController::setPitchTrim(float trim) {
    pitchTrim = trim;
}

void ServoController::setNeutral(float position) {
    servoNeutral = position;
}

void ServoController::setMaxDeflection(float degrees) {
    maxDeflection = degrees;
}

void ServoController::setMaxMoveRate(float rate) {
    maxMoveRate = rate;
}

void ServoController::enableLimits(bool enable) {
    limitsEnabled = enable;
}

float ServoController::applyLimits(float value, float min, float max) {
    if (limitsEnabled) {
        if (value < min) return min;
        if (value > max) return max;
    }
    return value;
}

float ServoController::applyRateLimit(float current, float target, float dt) {
    if (!limitsEnabled || maxMoveRate <= 0.0f) {
        return target;  // No rate limiting
    }
    
    // Calculate maximum allowed change based on time and rate
    float maxChange = maxMoveRate * dt;
    
    // Calculate desired change
    float change = target - current;
    
    // Limit the change if needed
    if (change > maxChange) {
        return current + maxChange;
    } else if (change < -maxChange) {
        return current - maxChange;
    } else {
        return target;  // Change is within limits
    }
}

void ServoController::updateServos(float rightAngle, float leftAngle, float pitchAngle) {
    // Calculate time since last update for rate limiting
    uint32_t currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;  // Convert to seconds
    lastUpdateTime = currentTime;
    
    // Apply trim to input angles
    leftAngle += leftTrim;
    rightAngle += rightTrim;
    pitchAngle += pitchTrim;
    
    // Apply limits to ensure servos stay within safe range
    float leftMin = servoNeutral - maxDeflection;   // Extended range for left servo
    float leftMax = servoNeutral + maxDeflection ;   // Extended range for left servo
    float rightMin = servoNeutral - maxDeflection;  // Extended range for right servo
    float rightMax = servoNeutral + maxDeflection;  // Extended range for right servo
    float pitchMin = servoNeutral - maxDeflection;          // Standard range for pitch servo
    float pitchMax = servoNeutral + maxDeflection;          // Standard range for pitch servo
    
    // Apply position limits
    leftAngle = applyLimits(leftAngle, leftMin, leftMax);
    rightAngle = applyLimits(rightAngle, rightMin, rightMax);
    pitchAngle = applyLimits(pitchAngle, pitchMin, pitchMax);
    
    // Apply rate limits
    leftAngle = applyRateLimit(leftPosition, leftAngle, dt);
    rightAngle = applyRateLimit(rightPosition, rightAngle, dt);
    pitchAngle = applyRateLimit(pitchPosition, pitchAngle, dt);
    
    // Update current positions
    leftPosition = leftAngle;
    rightPosition = rightAngle;
    pitchPosition = pitchAngle;
    // print for debugging
    Serial.print("Left Servo Angle: ");
    Serial.print(leftAngle);
    Serial.print(" Right Servo Angle: ");
    Serial.print(rightAngle);
    Serial.print(" Pitch Servo Angle: ");
    Serial.println(pitchAngle);

    
    // Write angles to servos
    leftServo.write(round(leftAngle));
    rightServo.write(round(rightAngle));
    pitchServo.write(round(pitchAngle));
}

void ServoController::setToNeutral() {
    // Apply trim to neutral position
    float leftNeutral = servoNeutral + leftTrim;
    float rightNeutral = servoNeutral + rightTrim;
    float pitchNeutral = servoNeutral + pitchTrim;
    
    // Update servos with neutral positions
    updateServos(rightNeutral, leftNeutral, pitchNeutral);
}

float ServoController::getLeftPosition() const {
    return leftPosition;
}

float ServoController::getRightPosition() const {
    return rightPosition;
}

float ServoController::getPitchPosition() const {
    return pitchPosition;
}

bool ServoController::isAttached() {
    return leftServo.attached() && rightServo.attached() && pitchServo.attached();
}

void ServoController::detach() {
    leftServo.detach();
    rightServo.detach();
    pitchServo.detach();
}

void ServoController::reattach() {
    if (!leftServo.attached()) {
        leftServo.attach(leftServoPin, 1000, 2000);
    }
    
    if (!rightServo.attached()) {
        rightServo.attach(rightServoPin, 1000, 2000);
    }
    
    if (!pitchServo.attached()) {
        pitchServo.attach(pitchServoPin, 1000, 2000);
    }
    
    // Set servos to their last known positions
    leftServo.write(round(leftPosition));
    rightServo.write(round(rightPosition));
    pitchServo.write(round(pitchPosition));
}