#include "CommandReader.h"

// Global pointer to CommandReader instance for use in ISRs
CommandReader* commandReaderPtr = nullptr;

// ISR for roll channel
void IRAM_ATTR rollISR() {
    if (commandReaderPtr) {
        commandReaderPtr->handleRollInterrupt();
    }
}

// ISR for pitch channel
void IRAM_ATTR pitchISR() {
    if (commandReaderPtr) {
        commandReaderPtr->handlePitchInterrupt();
    }
}

// ISR for flaps channel
void IRAM_ATTR flapsISR() {
    if (commandReaderPtr) {
        commandReaderPtr->handleFlapsInterrupt();
    }
}

CommandReader::CommandReader(uint8_t rollPin, uint8_t pitchPin, uint8_t flapsPin) {
    this->rollPin = rollPin;
    this->pitchPin = pitchPin;
    this->flapsPin = flapsPin;
    
    // Default values
    rollPulseWidth = 1500;
    pitchPulseWidth = 1500;
    flapsPulseWidth = 1500;
    
    rollStartTime = 0;
    pitchStartTime = 0;
    flapsStartTime = 0;
    
    // Default calibration (standard PWM values)
    rollMin = 1000;
    rollCenter = 1500;
    rollMax = 2000;
    
    pitchMin = 1000;
    pitchCenter = 1500;
    pitchMax = 2000;
    
    flapsMin = 1000;
    flapsMax = 2000;
    
    // Default command ranges
    rollMaxDegrees = 30.0f;
    pitchMaxDegrees = 30.0f;
    flapsMaxDegrees = 45.0f;
    
    // Default deadband
    deadband = 2.0f;
    
    // Initial signal state
    signalValid = false;
    lastValidSignalTime = 0;
    
    // Set the global pointer for ISR access
    commandReaderPtr = this;
    
    // Initialize commands
    rollCommand = 0.0f;
    pitchCommand = 0.0f;
    flapsCommand = 0.0f;
}

void CommandReader::begin() {
    // Set up pins as inputs with pullups
    pinMode(rollPin, INPUT_PULLUP);
    pinMode(pitchPin, INPUT_PULLUP);
    pinMode(flapsPin, INPUT_PULLUP);
    
    // Attach interrupts on ESP32 - both rising and falling edges
    attachInterrupt(digitalPinToInterrupt(rollPin), rollISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pitchPin), pitchISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(flapsPin), flapsISR, CHANGE);
}

void CommandReader::handleRollInterrupt() {
    // Get current time
    uint32_t now = micros();
    
    // Check pin state
    bool state = digitalRead(rollPin);
    
    if (state) {
        // Rising edge
        rollStartTime = now;
    } else {
        // Falling edge - calculate pulse width
        if (rollStartTime != 0) {
            uint32_t pulseWidth = now - rollStartTime;
            
            // Validate pulse width (typical range for RC receivers)
            if (pulseWidth >= 900 && pulseWidth <= 2100) {
                rollPulseWidth = pulseWidth;
                lastValidSignalTime = millis();
                signalValid = true;
            }
        }
    }
}

void CommandReader::handlePitchInterrupt() {
    // Get current time
    uint32_t now = micros();
    
    // Check pin state
    bool state = digitalRead(pitchPin);
    
    if (state) {
        // Rising edge
        pitchStartTime = now;
    } else {
        // Falling edge - calculate pulse width
        if (pitchStartTime != 0) {
            uint32_t pulseWidth = now - pitchStartTime;
            
            // Validate pulse width (typical range for RC receivers)
            if (pulseWidth >= 900 && pulseWidth <= 2100) {
                pitchPulseWidth = pulseWidth;
                lastValidSignalTime = millis();
                signalValid = true;
            }
        }
    }
}

void CommandReader::handleFlapsInterrupt() {
    // Get current time
    uint32_t now = micros();
    
    // Check pin state
    bool state = digitalRead(flapsPin);
    
    if (state) {
        // Rising edge
        flapsStartTime = now;
    } else {
        // Falling edge - calculate pulse width
        if (flapsStartTime != 0) {
            uint32_t pulseWidth = now - flapsStartTime;
            
            // Validate pulse width (typical range for RC receivers)
            if (pulseWidth >= 900 && pulseWidth <= 2100) {
                flapsPulseWidth = pulseWidth;
                lastValidSignalTime = millis();
                signalValid = true;
            }
        }
    }
}

void CommandReader::calibrateRoll(uint16_t min, uint16_t center, uint16_t max) {
    rollMin = min;
    rollCenter = center;
    rollMax = max;
}

void CommandReader::calibratePitch(uint16_t min, uint16_t center, uint16_t max) {
    pitchMin = min;
    pitchCenter = center;
    pitchMax = max;
}

void CommandReader::calibrateFlaps(uint16_t min, uint16_t max) {
    flapsMin = min;
    flapsMax = max;
}

void CommandReader::setRollMaxDegrees(float degrees) {
    rollMaxDegrees = degrees;
}

void CommandReader::setPitchMaxDegrees(float degrees) {
    pitchMaxDegrees = degrees;
}

void CommandReader::setFlapsMaxDegrees(float degrees) {
    flapsMaxDegrees = degrees;
}

void CommandReader::setDeadband(float value) {
    deadband = value;
}

float CommandReader::scalePulseToCommand(uint16_t pulse, uint16_t min, uint16_t center, uint16_t max, float maxDegrees) {
    float command = 0.0f;
    
    // Scale based on which side of center we're on
    if (pulse < center) {
        // Map from min to center to -maxDegrees to 0
        command = map(pulse, min, center, -maxDegrees * 100, 0) / 100.0f;
    } else {
        // Map from center to max to 0 to maxDegrees
        command = map(pulse, center, max, 0, maxDegrees * 100) / 100.0f;
    }
    
    // Apply deadband
    if (abs(command) < deadband) {
        command = 0.0f;
    }
    
    return command;
}

float CommandReader::scaleFlapsCommand(uint16_t pulse, uint16_t min, uint16_t max, float maxDegrees) {
    // Map from min to max to 0 to maxDegrees
    return map(pulse, min, max, 0, maxDegrees * 100) / 100.0f;
}

void CommandReader::update() {
    // Check if signal is still valid
    unsigned long currentTime = millis();
    if (currentTime - lastValidSignalTime > failsafeTimeout) {
        signalValid = false;
        
        // When signal is invalid, set commands to neutral
        rollCommand = 0.0f;
        pitchCommand = 0.0f;
        flapsCommand = 0.0f;
        return;
    }
    
    // Scale the raw pulse widths to command values
    rollCommand = scalePulseToCommand(rollPulseWidth, rollMin, rollCenter, rollMax, rollMaxDegrees);
    pitchCommand = scalePulseToCommand(pitchPulseWidth, pitchMin, pitchCenter, pitchMax, pitchMaxDegrees);
    flapsCommand = scaleFlapsCommand(flapsPulseWidth, flapsMin, flapsMax, flapsMaxDegrees);
}

float CommandReader::getRollCommand() const {
    return rollCommand;
}

float CommandReader::getPitchCommand() const {
    return pitchCommand;
}

float CommandReader::getFlapsCommand() const {
    return flapsCommand;
}

bool CommandReader::isSignalValid() const {
    return signalValid;
}

uint16_t CommandReader::getRawRoll() const {
    return rollPulseWidth;
}

uint16_t CommandReader::getRawPitch() const {
    return pitchPulseWidth;
}

uint16_t CommandReader::getRawFlaps() const {
    return flapsPulseWidth;
}