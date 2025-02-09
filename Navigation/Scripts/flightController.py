from machine import UART, Pin
import math,time
from ulab import numpy as np

DEG_TO_RAD = math.pi/180

class SerialCommunicator:
    def __init__(self, uart_id=0, baud_rate=9600, tx_pin=0, rx_pin=1):
        self.uart = UART(uart_id, baudrate=baud_rate)
        self.roll = 0
        self.pitch = 0
        self.flaps = 0
        self.rightServo = 0
        self.leftServo = 0
        self.pitchServo = 0
    
    def validate_value(self, value, min_val=-50, max_val=50):
        """Validate that a value is within expected range"""
        try:
            val = float(value)
            if val < min_val or val > max_val:
                return None
            return val
        except ValueError:
            return None
    
    def receive_target_orientation(self):
        command = f"<rqt>\n"
        self.uart.write(command.encode())
        time.sleep(0.005)
        try:
            response = self.uart.readline()
            if response:
                response = response.decode().strip()
                
                if response.startswith("<t,") and response.endswith(">"):
                    #print(response)
                    _, roll, pitch, flaps = response.split(",")
                    roll = self.validate_value(roll)
                    pitch = self.validate_value(pitch)
                    flaps = self.validate_value(flaps[:-1])
                    if roll is not None and pitch is not None and flaps is not None:
                        self.roll = roll
                        self.pitch = pitch
                        self.flaps = flaps
                        return True
        except:
            return False

    
    def send_servo_commands(self):
        # Send format: "rightServo,leftServo,pitchServo\n"
        command = f"<m,{self.rightServo:03d},{self.leftServo:03d},{self.pitchServo:03d}>\n"
        print(command)
        self.uart.write(command.encode())
    
    def update_servo_values(self, rightServo, leftServo, pitchServo):
        self.rightServo = int(rightServo)
        self.leftServo = int(leftServo)
        self.pitchServo = int(pitchServo)
    
    def get_target_orientation(self):
        return (self.roll, self.pitch, self.flaps)


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

class FlightController:
    def __init__(self):
        
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
        self.max_deflection = 40
        
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
         
        #print(f"Right Servo: {right_servo_angle}, Left Servo: {left_servo_angle}, Pitch Servo: {pitch_servo_angle}")
        return right_servo_angle, left_servo_angle, pitch_servo_angle


