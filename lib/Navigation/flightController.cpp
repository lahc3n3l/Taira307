#include "FlightController.h"
#include <Arduino.h>

FlightController::FlightController() : 
    roll_pid(1.0f, 0.0f, 0.0f),
    pitch_pid(1.0f, 0.0f, 0.0f) {
    
    // Target angles (degrees)
    target_roll = 0.0f;
    target_pitch = 0.0f;
    
    // Trim settings (degrees)
    roll_trim = 0.0f;
    pitch_trim = 0.0f;
    
    // Base servo position (90 degrees is neutral)
    servo_neutral = 90.0f;
    
    // Maximum deflection from neutral (degrees)
    max_deflection = 40.0f;
}

void FlightController::updateControlSurfaces(
    float current_roll, 
    float current_pitch, 
    float flaps_angle, 
    float dt, 
    float &right_servo_angle, 
    float &left_servo_angle, 
    float &pitch_servo_angle
) {
    // Calculate errors (in degrees)
    float roll_error = target_roll - current_roll;
    float pitch_error = target_pitch - current_pitch;
    
    // Get PID outputs
    float roll_correction = roll_pid.compute(roll_error, dt);
    float pitch_correction = pitch_pid.compute(pitch_error, dt);
    
    // Calculate servo angles
    // For roll: servos move in opposite directions
    // Pitch servo handles pitch independently
    left_servo_angle = (servo_neutral + flaps_angle - 
                      roll_correction + 
                      roll_trim);
    
    right_servo_angle = (servo_neutral - flaps_angle - 
                       roll_correction - 
                       roll_trim);
    
    pitch_servo_angle = (servo_neutral + 
                       pitch_correction + 
                       pitch_trim);
    
    // Constrain servo angles
    left_servo_angle = max(servo_neutral - max_deflection ,
                          min(servo_neutral + max_deflection ,
                              left_servo_angle));
    
    right_servo_angle = max(servo_neutral - max_deflection ,
                           min(servo_neutral + max_deflection ,
                               right_servo_angle));
    
    pitch_servo_angle = max(servo_neutral - max_deflection,
                           min(servo_neutral + max_deflection,
                               pitch_servo_angle));
}

void FlightController::setTargetRoll(float roll) {
    target_roll = roll;
}

void FlightController::setTargetPitch(float pitch) {
    target_pitch = pitch;
}

void FlightController::setRollTrim(float trim) {
    roll_trim = trim;
}

void FlightController::setPitchTrim(float trim) {
    pitch_trim = trim;
}

float FlightController::getTargetRoll() const {
    return target_roll;
}

float FlightController::getTargetPitch() const {
    return target_pitch;
}

float FlightController::getRollTrim() const {
    return roll_trim;
}

float FlightController::getPitchTrim() const {
    return pitch_trim;
}

void FlightController::setServoNeutral(float neutral) {
    servo_neutral = neutral;
}

void FlightController::setMaxDeflection(float deflection) {
    max_deflection = deflection;
}

float FlightController::getServoNeutral() const {
    return servo_neutral;
}

float FlightController::getMaxDeflection() const {
    return max_deflection;
}

void FlightController::setRollPID(float kp, float ki, float kd) {
    roll_pid.setTunings(kp, ki, kd);
}

void FlightController::setPitchPID(float kp, float ki, float kd) {
    pitch_pid.setTunings(kp, ki, kd);
}

PIDController& FlightController::getRollPID() {
    return roll_pid;
}

PIDController& FlightController::getPitchPID() {
    return pitch_pid;
}