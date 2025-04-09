from machine import I2C, Pin
import struct
from time import sleep_ms


class GY87:
    # MPU6050 addresses
    MPU6050_ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_PIN_CFG = 0x37
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    
    # QMC5883L addresses
    QMC5883L_ADDR = 0x0D
    QMC_X_LSB = 0x00
    QMC_CONTROL1 = 0x09
    QMC_CONTROL2 = 0x0A
    QMC_PERIOD = 0x0B
    
    def __init__(self, i2c, accel_scale=2, gyro_scale=250):
        self.i2c = i2c
        
        # Calibration offsets
        self.accel_offsets = [0.0, 0.0, 0.0]
        self.gyro_offsets = [0.0, 0.0, 0.0]
        self.mag_offsets = [0.0, 0.0, 0.0]  # Hard iron correction
        self.mag_cal_scale = [1.0, 1.0, 1.0]    # Soft iron correction
        
        # Scale factors
        accel_scales = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
        gyro_scales = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}
        
        self.accel_scale = accel_scales.get(accel_scale, 16384.0)
        self.gyro_scale = gyro_scales.get(gyro_scale, 131.0)
        
        # QMC5883L scale (LSB/Gauss) for 2G range
        self.mag_scale = 12000.0
        
        # Initialize the sensors
        self._mpu_init(accel_scale, gyro_scale)
        sleep_ms(10)  # Wait for MPU6050 to settle
        self._magnetometer_init()
        sleep_ms(10)  # Wait for QMC5883L to settle
        
        
    def _mpu_init(self, accel_scale, gyro_scale):
        """Initialize MPU6050"""
        # Wake up the MPU6050
        self.i2c.writeto_mem(self.MPU6050_ADDR, self.PWR_MGMT_1, bytes([0x00]))
        
        # Configure accelerometer
        accel_config = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}
        self.i2c.writeto_mem(self.MPU6050_ADDR, self.ACCEL_CONFIG, 
                           bytes([accel_config.get(accel_scale, 0x00)]))
        
        # Configure gyroscope
        gyro_config = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}
        self.i2c.writeto_mem(self.MPU6050_ADDR, self.GYRO_CONFIG,
                           bytes([gyro_config.get(gyro_scale, 0x00)]))
        
        # Enable I2C bypass to access QMC5883L
        self.i2c.writeto_mem(self.MPU6050_ADDR, self.INT_PIN_CFG, bytes([0x02]))
    
    def _magnetometer_init(self):
        """Initialize QMC5883L magnetometer"""
        try:
            # Soft reset
            self.i2c.writeto_mem(self.QMC5883L_ADDR, self.QMC_CONTROL2, bytes([0x80]))
            sleep_ms(5)
            
            # Configure Control Register 1:
            # Mode = Continuous (0x01)
            # ODR = 200Hz (0x0C)
            # RNG = 2G (0x00)
            # OSR = 512 (0x40)
            self.i2c.writeto_mem(self.QMC5883L_ADDR, self.QMC_CONTROL1, bytes([0x4D]))
            
            # Set-Reset period
            self.i2c.writeto_mem(self.QMC5883L_ADDR, self.QMC_PERIOD, bytes([0x01]))
            
        except Exception as e:
            print("Error initializing QMC5883L:", e)
    
    def _read_sensor_raw(self, addr, reg, length):
        """Read raw sensor data"""
        try:
            data = self.i2c.readfrom_mem(addr, reg, length)
            if addr == self.QMC5883L_ADDR:
                # QMC5883L uses little-endian format
                return struct.unpack('<%dh' % (length // 2), data)
            else:
                # MPU6050 uses big-endian format
                return struct.unpack('>%dh' % (length // 2), data)
        except:
            return (0,) * (length // 2)
    
    def get_acceleration(self):
        """
        Read accelerometer data
        Returns: tuple of (x, y, z) acceleration in g's
        """
        raw_accel = self._read_sensor_raw(self.MPU6050_ADDR, self.ACCEL_XOUT_H, 6)
        return tuple(-x / self.accel_scale for x in raw_accel)
    
    def get_gyro(self):
        """
        Read gyroscope data
        Returns: tuple of (x, y, z) angular velocity in degrees/sec
        """
        raw_gyro = self._read_sensor_raw(self.MPU6050_ADDR, self.GYRO_XOUT_H, 6)
        return tuple(x / self.gyro_scale for x in raw_gyro)
    
    def get_magnetic(self):
        """
        Read magnetometer data
        Returns: tuple of (x, y, z) magnetic field in Gauss
        """
        raw_mag = self._read_sensor_raw(self.QMC5883L_ADDR, self.QMC_X_LSB, 6)
        # Convert to Gauss (raw value / scale)
        return tuple(raw_mag[i] / self.mag_scale for i in range(3))
    
    def calibrate_accel_gyro(self, samples=500, delay_ms=5):
        """
        Calibrate accelerometer and gyroscope.
        Keep the sensor still and level during calibration.
        """
        print("Calibrating accelerometer and gyroscope...")
        print("Keep the sensor still and level!")
        sleep_ms(2000)  # Give user time to prepare
        
        accel_sums = [0.0, 0.0, 0.0]
        gyro_sums = [0.0, 0.0, 0.0]
        
        # Collect samples
        for _ in range(samples):
            accel = self.get_acceleration()
            gyro = self.get_gyro()
            
            for i in range(3):
                accel_sums[i] += accel[i]
                gyro_sums[i] += gyro[i]
            
            sleep_ms(delay_ms)
        
        # Calculate averages
        self.accel_offsets = [
            -accel_sums[0]/samples,
            -accel_sums[1]/samples,
            -(accel_sums[2]/samples - 1.0)  # Account for gravity
        ]
        
        self.gyro_offsets = [
            -gyro_sums[0]/samples,
            -gyro_sums[1]/samples,
            -gyro_sums[2]/samples
        ]
        
        print("Calibration complete!")
        print("Accelerometer offsets:", [round(x, 3) for x in self.accel_offsets])
        print("Gyroscope offsets:", [round(x, 3) for x in self.gyro_offsets])
        
    def calibrate_mag(self, samples=1000, delay_ms=10):
        """
        Calibrate magnetometer. During calibration, rotate the sensor in all directions
        to capture minimum and maximum values on all axes.
        """
        print("Calibrating magnetometer...")
        print("Rotate the sensor in all directions!")
        sleep_ms(2000)  # Give user time to prepare
        
        mag_min = [float('inf')] * 3
        mag_max = [float('-inf')] * 3
        
        for _ in range(samples):
            mag = list(self.get_magnetic())
            
            for i in range(3):
                mag_min[i] = min(mag_min[i], mag[i])
                mag_max[i] = max(mag_max[i], mag[i])
            
            sleep_ms(delay_ms)
        
        # Calculate hard iron offsets (center of min/max)
        self.mag_offsets = [
            (mag_min[0] + mag_max[0]) / 2,
            (mag_min[1] + mag_max[1]) / 2,
            (mag_min[2] + mag_max[2]) / 2
        ]
        
        # Calculate soft iron scaling factors
        avg_delta = 2
        
        self.mag_cal_scale = [
            avg_delta / (mag_max[0] - mag_min[0]),
            avg_delta / (mag_max[1] - mag_min[1]),
            avg_delta / (mag_max[2] - mag_min[2])
        ]
        
        print("Magnetometer calibration complete!")
        print("Hard iron offsets:", [round(x, 3) for x in self.mag_offsets])
        print("Soft iron scale:", [round(x, 3) for x in self.mag_cal_scale])
    
    def get_calibrated_acceleration(self):
        """Get calibrated accelerometer readings"""
        accel = self.get_acceleration()
        return tuple(accel[i] + self.accel_offsets[i] for i in range(3))
    
    def get_calibrated_gyro(self):
        """Get calibrated gyroscope readings"""
        gyro = self.get_gyro()
        return tuple(gyro[i] + self.gyro_offsets[i] for i in range(3))
    
    def get_calibrated_magnetic(self):
        """Get calibrated magnetometer readings"""
        mag = self.get_magnetic()
        # Apply hard iron offset correction and soft iron scaling
        return tuple((mag[i] - self.mag_offsets[i]) * self.mag_cal_scale[i] for i in range(3))
    
    def get_all_data(self):
        """
        Read all calibrated sensor data at once
        Returns: tuple of (accel_xyz, gyro_xyz, mag_xyz)
        """
        return (self.get_calibrated_acceleration(),
                self.get_calibrated_gyro(),
                self.get_calibrated_magnetic())



