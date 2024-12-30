from flightController import *
from KalmanPitchRoll import *


# Main
configureIMU()
accelBiases, accelVariance, gyroBiases, gyroVariance = calibrateIMU(2000)
kf = KalmanFilter()  # 80ms sampling rate

# Initialize flight controller (adjust pins as needed)
controller = FlightController(left_servo_pin=11, right_servo_pin=12,pitch_servo_pin=13)

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