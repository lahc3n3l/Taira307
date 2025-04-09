import time, utime
from math import pi
from flightController import *
from KalmanPitchRoll import *
from GY87Module import *

# Constants
DEG_TO_RAD = math.pi/180
RAD_TO_DEG = 180/math.pi

# Blink at startup

StartUpLed = Pin(25, Pin.OUT)  # Built-in LED on Pico is on GPIO 25
DataLoggerLed = Pin(16, Pin.OUT)  # Data logger LED on Pico is on Gpio 16

StartUpLed.toggle()
time.sleep(0.2)
StartUpLed.toggle()
time.sleep(0.2)
StartUpLed.toggle()

# Initialize components
# Initialize I2C
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

# Initialize GY87
sensor = GY87(i2c)
sensor.calibrate_accel_gyro()

# Initialize Serial communication
Scom = SerialCommunicator()

# Initialize Kalman filter
kf = KalmanFilter()

# Initialize flight controller
fController = FlightController()

# Initialize data logger
timeCounter = 1
try:
    timeFile = open("timeFile.txt", "r")
    
    lastValue = timeFile.readline().strip()
    timeFile.close()
    timeCounter = int(lastValue) + 1
    print("time file found ! ")
except:
    lastValue = timeCounter
    print("time file not found ! ")
    
fileName  = f"FlighData_{lastValue}.csv"

timeFile = open("timeFile.txt", "w")
timeFile.write(str(timeCounter) +"\n")
timeFile.close()


header = [
            "timestamp",
            "ax", "ay", "az",
            "gx", "gy", "gz",
            "mx", "my", "mz",
            "pitch", "roll", "yaw",
            "target_pitch", "target_roll", "target_flaps",
            "right_servo", "left_servo", "pitch_servo"
        ]
file = open(fileName, "w")
file.write(",".join(header)+"\n")
file.close()

DataLoggerLed.toggle()
time.sleep(0.2)
DataLoggerLed.toggle()
time.sleep(0.2)
DataLoggerLed.toggle()
time.sleep(0.2)
DataLoggerLed.toggle()
time.sleep(0.2)


# Initial target attitude and throttle
target_Roll = 0
target_Pitch = 0
target_Throttle = 0

# Initialize smoothing parameters
alpha = 0.3  # Smoothing facto
smoothed_target_roll = 0
smoothed_target_pitch = 0
smoothed_target_flaps = 0

log_counter = 0

lastTime = utime.ticks_us()
try:
    while True:

        accel, gyro, mag = sensor.get_all_data()
        ax, ay, az = accel
        gx, gy, gz = gyro
        mx, my, mz = mag
        
        currentTime = utime.ticks_us()
        dt = (currentTime - lastTime)/1e6
        
        gyro = np.array([x * DEG_TO_RAD for x in gyro])
        kf.predict(gyro, dt)
        kf.update(accel, mag)

        # Get estimated angles
        roll, pitch, yaw = kf.get_euler_angles_d()
        
        # get target Pitch & Roll angles
        Scom.receive_target_orientation()
        target_Roll,target_Pitch, target_flaps = Scom.get_target_orientation()

        # Smooth the target angles
        smoothed_target_roll = alpha * target_Roll + (1 - alpha) * smoothed_target_roll
        smoothed_target_pitch = alpha * target_Pitch + (1 - alpha) * smoothed_target_pitch
        smoothed_target_flaps = alpha * target_flaps + (1 - alpha) * smoothed_target_flaps

        # Update target values
        fController.target_pitch = smoothed_target_pitch
        fController.target_roll = smoothed_target_roll

        # Update control surfaces
        right_servo_angle, left_servo_angle, pitch_servo_angle = fController.update_control_surfaces(roll, pitch,smoothed_target_flaps, dt)
        Scom.update_servo_values(right_servo_angle, left_servo_angle,pitch_servo_angle)
        Scom.send_servo_commands()
        
        # Log data
        if log_counter%10 == 0:
            try:
                # Log data
                data_dict = {
                    'timestamp': utime.ticks_ms()*0.001,
                    'ax': ax, 'ay': ay, 'az': az,
                    'gx': gx, 'gy': gy, 'gz': gz,
                    'mx': mx, 'my': my, 'mz': mz,
                    'pitch': pitch,
                    'roll': roll,
                    'yaw': yaw,
                    'target_pitch': smoothed_target_pitch,
                    'target_roll': smoothed_target_roll,
                    'target_flaps': smoothed_target_flaps,
                    'right_servo': right_servo_angle,
                    'left_servo': left_servo_angle,
                    'pitch_servo': pitch_servo_angle
                }
                
                file = open(fileName, "a")
                file.write(f"{data_dict['timestamp']},{data_dict['ax']},{data_dict['ay']},{data_dict['az']},{data_dict['gx']},{data_dict['gy']},{data_dict['gz']},{data_dict['mx']},{data_dict['my']},{data_dict['mz']},{data_dict['pitch']},{data_dict['roll']},{data_dict['yaw']},{data_dict['target_pitch']},{data_dict['target_roll']},{data_dict['target_flaps']},{data_dict['right_servo']},{data_dict['left_servo']},{data_dict['pitch_servo']}\n")
                file.close()
                DataLoggerLed.toggle() #blink for successful log
                
            except Exception as e:
                print(f"Error in main loop logging: {e}")
                DataLoggerLed.toggle()
            
        # Update timing
        lastTime = currentTime
        
        # Add a small delay to prevent overwhelming the system
        #print(f"Roll: {roll}, Pitch: {pitch}")
        #print(f"target Roll: {smoothed_target_roll}, target Pitch: {smoothed_target_pitch}")
        #print('target_flaps: ', smoothed_target_flaps)
        log_counter+=1
        time.sleep_ms(40)  # 50Hz update rate
        
        
except KeyboardInterrupt:
    print("keyboard intrupt!")
    
    
    




    




