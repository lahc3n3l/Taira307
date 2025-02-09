import time, utime
from math import pi
from flightController import *
from KalmanPitchRoll import *
from dataLogger import *


# Blink at startup

StartUpLed = Pin(25, Pin.OUT)  # Built-in LED on Pico is on GPIO 25
DataLoggerLed = Pin(16, Pin.OUT)  # Data logger LED on Pico is on Gpio 16

StartUpLed.toggle()
time.sleep(0.2)
StartUpLed.toggle()
time.sleep(0.2)
StartUpLed.toggle()

# Initialize components
Scom = SerialCommunicator()
kf = KalmanFilter()
fController = FlightController()
configureIMU()
logger = DataLogger()

DataLoggerLed.toggle()
time.sleep(0.2)
DataLoggerLed.toggle()
time.sleep(0.2)
DataLoggerLed.toggle()

accelBiases, accelVariance, gyroBiases, gyroVariance = calibrateIMU(2000)

# Initial target attitude and throttle
target_Roll = 0
target_Pitch = 0
target_Throttle = 0

lastTime = utime.ticks_us()
try:
    while True:
            
        ax, ay, az = np.array(readIMU()[0]) - accelBiases
        gx, gy, gz = np.array(readIMU()[1]) - gyroBiases
        
        gyro = np.array([gx, gy, gz])
        currentTime = utime.ticks_us()
        
        dt = (currentTime - lastTime)/1e6
        
        kf.predict(gyro, dt)
        kf.update(np.array([ax, ay, az]))

        roll, pitch, yaw = kf.x
        # Get current roll and pitch from your IMU
        
        current_roll = (roll*180/pi)   
        current_pitch = (pitch*180/pi)
        
        # get target Pitch & Roll angles
        Scom.receive_target_orientation()
        target_Roll,target_Pitch, target_flaps = Scom.get_target_orientation()

        #print(f"target Roll: {target_Roll}, target Pitch: {target_Pitch}")
        # Update target values
        fController.target_pitch = target_Pitch
        fController.target_roll = target_Roll
        # Update control surfaces
        right_servo_angle, left_servo_angle, pitch_servo_angle = fController.update_control_surfaces(current_roll, current_pitch, dt)
        Scom.update_servo_values(right_servo_angle, left_servo_angle,pitch_servo_angle)
        Scom.send_servo_commands()
        
        # Log data
         # Log data
        logger.log_data({
                'timestamp': utime.ticks_ms(),
                'ax': ax, 'ay': ay, 'az': az,
                'gx': gx, 'gy': gy, 'gz': gz,
                'pitch': current_pitch,
                'roll': current_roll,
                'yaw': yaw*180/pi,
                'target_pitch': target_Pitch,
                'target_roll': target_Roll,
                'right_servo': right_servo_angle,
                'left_servo': left_servo_angle,
                'pitch_servo': pitch_servo_angle
        })
        
        # Update timing
        lastTime = currentTime
        
        # Add a small delay to prevent overwhelming the system
        # print(f"Roll: {roll*180/pi}, Pitch: {pitch*180/pi}")
        time.sleep_ms(40)  # 50Hz update rate
        
except KeyboardInterrupt:
    print("keyboard intrupt!")
    
    
    




    

