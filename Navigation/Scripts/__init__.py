from Kalman import *

# Main
configureIMU()
accelBiases, accelVariance, gyroBiases, gyroVariance = calibrateIMU(2000)
kf = KalmanFilter()  # 80ms sampling rate
lastTime = utime.ticks_us()
print(accelBiases,accelVariance, gyroBiases, gyroVariance)
while True:
    ax, ay, az = np.array(readIMU()[0]) - accelBiases
    gx, gy, gz = np.array(readIMU()[1]) - gyroBiases
    
    gyro = np.array([gx, gy, gz])
    currentTime = utime.ticks_us()
    dt = (currentTime - lastTime)/1e6
    lastTime = currentTime
    kf.predict(gyro, dt)
    kf.update(np.array([ax, ay, az]))

    roll, pitch, yaw = kf.x

    print(f"Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}°")#, Yaw: {math.degrees(yaw):.2f}°")
    utime.sleep_ms(20)