#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "PIDController.h"

class FlightController {
private:
    PIDController roll_pid;
    PIDController pitch_pid;
    
    float target_roll;
    float target_pitch;
    
    float roll_trim;
    float pitch_trim;
    
    float servo_neutral;
    float max_deflection;

public:
    FlightController();
    
    // Update control surfaces based on current orientation
    void updateControlSurfaces(
        float current_roll, 
        float current_pitch, 
        float flaps_angle, 
        float dt, 
        float &right_servo_angle, 
        float &left_servo_angle, 
        float &pitch_servo_angle
    );
    
    // Setters for target angles and trim
    void setTargetRoll(float roll);
    void setTargetPitch(float pitch);
    void setRollTrim(float trim);
    void setPitchTrim(float trim);
    
    // Getters for target angles and trim
    float getTargetRoll() const;
    float getTargetPitch() const;
    float getRollTrim() const;
    float getPitchTrim() const;
    
    // Setters for servo parameters
    void setServoNeutral(float neutral);
    void setMaxDeflection(float max_deflection);
    
    // Getters for servo parameters
    float getServoNeutral() const;
    float getMaxDeflection() const;
    
    // PID tuning methods
    void setRollPID(float kp, float ki, float kd);
    void setPitchPID(float kp, float ki, float kd);
    
    // Get PID controllers (for advanced tuning)
    PIDController& getRollPID();
    PIDController& getPitchPID();
};

#endif // FLIGHT_CONTROLLER_H