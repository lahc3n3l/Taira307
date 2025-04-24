#ifndef COMMAND_READER_H
#define COMMAND_READER_H

#include <Arduino.h>

class CommandReader {
private:
    // Pin definitions for the FlySky receiver channels
    uint8_t rollPin;
    uint8_t pitchPin;
    uint8_t flapsPin;
    
    // Raw pulse width values from receiver (in microseconds)
    volatile uint16_t rollPulseWidth;
    volatile uint16_t pitchPulseWidth;
    volatile uint16_t flapsPulseWidth;
    
    // Timestamps for calculating pulse width
    volatile uint32_t rollStartTime;
    volatile uint32_t pitchStartTime;
    volatile uint32_t flapsStartTime;
    
    // Calibration values for each channel
    uint16_t rollMin;
    uint16_t rollMax;
    uint16_t rollCenter;
    
    uint16_t pitchMin;
    uint16_t pitchMax;
    uint16_t pitchCenter;
    
    uint16_t flapsMin;
    uint16_t flapsMax;
    
    // Scaled command values (in degrees)
    float rollCommand;
    float pitchCommand;
    float flapsCommand;
    
    // Maximum command values (in degrees)
    float rollMaxDegrees;
    float pitchMaxDegrees;
    float flapsMaxDegrees;
    
    // Deadband to prevent jitter around center position
    float deadband;
    
    // Status
    bool signalValid;
    unsigned long lastValidSignalTime;
    const unsigned long failsafeTimeout = 500; // milliseconds
    
    // Scale raw pulse width to degrees
    float scalePulseToCommand(uint16_t pulse, uint16_t min, uint16_t center, uint16_t max, float maxDegrees);
    float scaleFlapsCommand(uint16_t pulse, uint16_t min, uint16_t max, float maxDegrees);
    
public:
    CommandReader(uint8_t rollPin, uint8_t pitchPin, uint8_t flapsPin);
    
    // Initialize and set up pin interrupts
    void begin();
    
    // Set calibration values
    void calibrateRoll(uint16_t min, uint16_t center, uint16_t max);
    void calibratePitch(uint16_t min, uint16_t center, uint16_t max);
    void calibrateFlaps(uint16_t min, uint16_t max);
    
    // Set maximum command values in degrees
    void setRollMaxDegrees(float degrees);
    void setPitchMaxDegrees(float degrees);
    void setFlapsMaxDegrees(float degrees);
    
    // Set deadband value
    void setDeadband(float value);
    
    // Update the commands by reading the receiver
    void update();
    
    // Get scaled command values
    float getRollCommand() const;
    float getPitchCommand() const;
    float getFlapsCommand() const;
    
    // Check if signal is valid
    bool isSignalValid() const;
    
    // Functions to handle interrupts
    void handleRollInterrupt();
    void handlePitchInterrupt();
    void handleFlapsInterrupt();
    
    // Debug methods to get raw pulse width values
    uint16_t getRawRoll() const;
    uint16_t getRawPitch() const;
    uint16_t getRawFlaps() const;
};

// Declare global functions for interrupt handling
void IRAM_ATTR rollISR();
void IRAM_ATTR pitchISR();
void IRAM_ATTR flapsISR();

// Declare external pointer to CommandReader instance (for interrupt access)
extern CommandReader* commandReaderPtr;

#endif // COMMAND_READER_H