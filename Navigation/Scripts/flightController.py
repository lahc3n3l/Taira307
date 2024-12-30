from machine import Pin, PWM, I2C
import time
from math import pi


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        
    def compute(self, error, dt):
        if abs(error) < 0.5:
            return 0
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.previous_error) / dt
        self.previous_error = error
        
        return p_term + i_term + d_term

class ServoController:
    def __init__(self, pin, min_duty=2000, max_duty=7800):
        self.servo = PWM(Pin(pin))
        self.servo.freq(50)
        self.min_duty = min_duty
        self.max_duty = max_duty
        self.current_angle = 90  # Track current angle
        
    def set_angle(self, target_angle):
        # Smooth the movement
        target_angle = max(0, min(180, target_angle))
        self.current_angle += (target_angle - self.current_angle) 
        
        # Convert to duty cycle
        duty = int(self.min_duty + (self.max_duty - self.min_duty) * self.current_angle / 180)
        self.servo.duty_u16(duty)

class FlightController:
    def __init__(self, left_servo_pin, right_servo_pin, pitch_servo_pin):
        # Initialize servos
        self.left_servo = ServoController(left_servo_pin)
        self.right_servo = ServoController(right_servo_pin)
        self.pitch_servo = ServoController(pitch_servo_pin)
        
        # Initialize PID controllers for roll and pitch
        self.roll_pid = PIDController(kp=0.5, ki=0.0, kd=0.0)
        self.pitch_pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        
        # Target angles (degrees)
        self.target_roll = 0
        self.target_pitch = 0
        
        # Trim settings (degrees)
        self.roll_trim = 0
        self.pitch_trim = 0
        
        # Base servo position (90 degrees is neutral)
        self.servo_neutral = 90
        
        # Maximum deflection from neutral (degrees)
        self.max_deflection = 20
        
    def update_control_surfaces(self, current_roll, current_pitch, dt):
        # Calculate errors (in degrees)
        roll_error = self.target_roll - current_roll
        pitch_error = self.target_pitch - current_pitch
        
        # Get PID outputs
        roll_correction = self.roll_pid.compute(roll_error, dt)
        pitch_correction = self.pitch_pid.compute(pitch_error, dt)
        
        # Calculate servo angles
        # For roll: servos move in opposite directions
        # Pitch servo handles pitch independently
        left_servo_angle = (self.servo_neutral - 
                          roll_correction + 
                          self.roll_trim)
        
        right_servo_angle = (self.servo_neutral + 
                           roll_correction - 
                           self.roll_trim)
        
        pitch_servo_angle = (self.servo_neutral + 
                           pitch_correction + 
                           self.pitch_trim)
        
        # Constrain servo angles
        left_servo_angle = max(self.servo_neutral - self.max_deflection,
                             min(self.servo_neutral + self.max_deflection,
                                 left_servo_angle))
        
        right_servo_angle = max(self.servo_neutral - self.max_deflection,
                              min(self.servo_neutral + self.max_deflection,
                                  right_servo_angle))
        
        pitch_servo_angle = max(self.servo_neutral - self.max_deflection,
                              min(self.servo_neutral + self.max_deflection,
                                  pitch_servo_angle))
        
        # Update servos
        self.left_servo.set_angle(left_servo_angle)
        self.right_servo.set_angle(right_servo_angle)
        self.pitch_servo.set_angle(pitch_servo_angle)
        print(f"Right Servo: {right_servo_angle}, Left Servo: {left_servo_angle}, Pitch Servo: {pitch_servo_angle}")
