from flightController import *
from KalmanPitchRoll import *
# from Ibus import IBus
from Ppm import PPMReader
# Main
# Blink at startup

StartUpLed = Pin(16, Pin.OUT)  # Built-in LED on Pico is on GPIO 25

StartUpLed.toggle()
time.sleep(0.2)
StartUpLed.toggle()
time.sleep(0.2)
StartUpLed.toggle()

configureIMU()

#ibus_in = IBus(1)
ppm = PPMReader(ppm_pin=5, num_channels=6)
accelBiases, accelVariance, gyroBiases, gyroVariance = calibrateIMU(2000)
kf = KalmanFilter()  # 80ms sampling rate

# Initialize flight controller (adjust pins as needed)
controller = FlightController(left_servo_pin=11, right_servo_pin=12,pitch_servo_pin=13)

lastTime = utime.ticks_us()
# Initial target attitude and throttle
target_Roll = 0
target_Pitch = 0
target_Throttle = 0
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
        #res = ibus_in.read()
        res = ppm.read_channels()


        target_Roll = res[0]*180/200
        target_Pitch = res[1]*180/200
        #target_Throttle =0# (res[2]+100)*180/200
        
        
        #print(f"target Roll: {target_Roll}, target Pitch: {target_Pitch}")
        # Update target values
        controller.target_pitch = target_Pitch
        controller.target_roll = target_Roll
        # Update control surfaces
        controller.update_control_surfaces(current_roll, current_pitch, dt)
        
        
        # Update timing
        lastTime = currentTime
        
        # Add a small delay to prevent overwhelming the system
        # print(f"Roll: {roll*180/pi}, Pitch: {pitch*180/pi}")
        time.sleep_ms(40)  # 50Hz update rate
        
except KeyboardInterrupt:
    # Clean up
    controller.left_servo.servo.deinit()
    controller.right_servo.servo.deinit()
    
    
    




    
