#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->previous_error = 0.0f;
    this->integral = 0.0f;
}

void PIDController::reset() {
    previous_error = 0.0f;
    integral = 0.0f;
}

float PIDController::compute(float error, float dt) {
    if (abs(error) < 0.5f) {
        return 0.0f;
    }
    
    // Proportional term
    float p_term = kp * error;
    
    // Integral term
    integral += error * dt;
    float i_term = ki * integral;
    
    // Derivative term
    float d_term = kd * (error - previous_error) / dt;
    previous_error = error;
    
    return p_term + i_term + d_term;
}

float PIDController::getKp() const {
    return kp;
}

float PIDController::getKi() const {
    return ki;
}

float PIDController::getKd() const {
    return kd;
}

void PIDController::setKp(float kp) {
    this->kp = kp;
}

void PIDController::setKi(float ki) {
    this->ki = ki;
}

void PIDController::setKd(float kd) {
    this->kd = kd;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}