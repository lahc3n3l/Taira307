#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    float kp;
    float ki;
    float kd;
    float previous_error;
    float integral;

public:
    PIDController(float kp, float ki, float kd);
    
    // Resets the PID controller's internal state
    void reset();
    
    // Computes the PID output
    float compute(float error, float dt);
    
    // Getters and setters for PID gains
    float getKp() const;
    float getKi() const;
    float getKd() const;
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void setTunings(float kp, float ki, float kd);
};

#endif // PID_CONTROLLER_H