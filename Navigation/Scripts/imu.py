from time import sleep
from machine import Pin, I2C
from utime import sleep_ms, ticks_ms
from math import sqrt, degrees, acos, atan2
import utime
import math
from ulab import numpy as np


def default_wait():
    '''
    delay of 50 ms
    '''
    sleep_ms(50)


class Vector3d(object):
    '''
    Represents a vector in a 3D space using Cartesian coordinates.
    Internally uses sensor relative coordinates.
    Returns vehicle-relative x, y and z values.
    '''

    def __init__(self, transposition, scaling, update_function):
        self._vector = [0, 0, 0]
        self._ivector = [0, 0, 0]
        self.cal = (0, 0, 0)
        self.argcheck(transposition, "Transposition")
        self.argcheck(scaling, "Scaling")
        if set(transposition) != {0, 1, 2}:
            raise ValueError('Transpose indices must be unique and in range 0-2')
        self._scale = scaling
        self._transpose = transposition
        self.update = update_function

    def argcheck(self, arg, name):
        '''
        checks if arguments are of correct length
        '''
        if len(arg) != 3 or not (type(arg) is list or type(arg) is tuple):
            raise ValueError(name + ' must be a 3 element list or tuple')

    def calibrate(self, stopfunc, waitfunc=default_wait):
        '''
        calibration routine, sets cal
        '''
        self.update()
        maxvec = self._vector[:]  # Initialise max and min lists with current values
        minvec = self._vector[:]
        while not stopfunc():
            waitfunc()
            self.update()
            maxvec = list(map(max, maxvec, self._vector))
            minvec = list(map(min, minvec, self._vector))
        self.cal = tuple(map(lambda a, b: (a + b) / 2, maxvec, minvec))

    @property
    def _calvector(self):
        '''
        Vector adjusted for calibration offsets
        '''
        return list(map(lambda val, offset: val - offset, self._vector, self.cal))

    @property
    def x(self):  # Corrected, vehicle relative floating point values
        self.update()
        return self._calvector[self._transpose[0]] * self._scale[0]

    @property
    def y(self):
        self.update()
        return self._calvector[self._transpose[1]] * self._scale[1]

    @property
    def z(self):
        self.update()
        return self._calvector[self._transpose[2]] * self._scale[2]

    @property
    def xyz(self):
        self.update()
        return (self._calvector[self._transpose[0]] * self._scale[0],
                self._calvector[self._transpose[1]] * self._scale[1],
                self._calvector[self._transpose[2]] * self._scale[2])

    @property
    def magnitude(self):
        x, y, z = self.xyz  # All measurements must correspond to the same instant
        return sqrt(x ** 2 + y ** 2 + z ** 2)

    @property
    def inclination(self):
        x, y, z = self.xyz
        return degrees(acos(z / sqrt(x ** 2 + y ** 2 + z ** 2)))

    @property
    def elevation(self):
        return 90 - self.inclination

    @property
    def azimuth(self):
        x, y, z = self.xyz
        return degrees(atan2(y, x))

    # Raw uncorrected integer values from sensor
    @property
    def ix(self):
        return self._ivector[0]

    @property
    def iy(self):
        return self._ivector[1]

    @property
    def iz(self):
        return self._ivector[2]

    @property
    def ixyz(self):
        return self._ivector

    @property
    def transpose(self):
        return tuple(self._transpose)

    @property
    def scale(self):
        return tuple(self._scale)


class MPUException(OSError):
    '''
    Exception for MPU devices
    '''
    pass


def bytes_toint(msb, lsb):
    '''
    Convert two bytes to signed integer (big endian)
    for little endian reverse msb, lsb arguments
    Can be used in an interrupt handler
    '''
    if not msb & 0x80:
        return msb << 8 | lsb  # +ve
    return - (((msb ^ 255) << 8) | (lsb ^ 255) + 1)


class MPU6050(object):
    '''
    Module for InvenSense IMUs. Base class implements MPU6050 6DOF sensor, with
    features common to MPU9150 and MPU9250 9DOF sensors.
    '''

    _I2Cerror = "I2C failure when communicating with IMU"
    _mpu_addr = (104, 105)  # addresses of MPU9150/MPU6050. There can be two devices
    _chip_id = 104

    def __init__(self, side_str, device_addr=None, transposition=(0, 1, 2), scaling=(1, 1, 1)):

        self._accel = Vector3d(transposition, scaling, self._accel_callback)
        self._gyro = Vector3d(transposition, scaling, self._gyro_callback)
        self.buf1 = bytearray(1)  # Pre-allocated buffers for reads: allows reads to
        self.buf2 = bytearray(2)  # be done in interrupt handlers
        self.buf3 = bytearray(3)
        self.buf6 = bytearray(6)

        sleep_ms(200)  # Ensure PSU and device have settled
        if isinstance(side_str, str):  # Non-pyb targets may use other than X or Y
            self._mpu_i2c = I2C(side_str)
        elif hasattr(side_str, 'readfrom'):  # Soft or hard I2C instance. See issue #3097
            self._mpu_i2c = side_str
        else:
            raise ValueError("Invalid I2C instance")

        if device_addr is None:
            devices = set(self._mpu_i2c.scan())
            mpus = devices.intersection(set(self._mpu_addr))
            number_of_mpus = len(mpus)
            if number_of_mpus == 0:
                raise MPUException("No MPU's detected")
            elif number_of_mpus == 1:
                self.mpu_addr = mpus.pop()
            else:
                raise ValueError("Two MPU's detected: must specify a device address")
        else:
            if device_addr not in (0, 1):
                raise ValueError('Device address must be 0 or 1')
            self.mpu_addr = self._mpu_addr[device_addr]

        self.chip_id  # Test communication by reading chip_id: throws exception on error
        # Can communicate with chip. Set it up.
        self.wake()  # wake it up
        self.passthrough = True  # Enable mag access from main I2C bus
        self.accel_range = 0  # default to highest sensitivity
        self.gyro_range = 0  # Likewise for gyro

    # read from device
    def _read(self, buf, memaddr, addr):  # addr = I2C device address, memaddr = memory location within the I2C device
        '''
        Read bytes to pre-allocated buffer Caller traps OSError.
        '''
        self._mpu_i2c.readfrom_mem_into(addr, memaddr, buf)

    # write to device
    def _write(self, data, memaddr, addr):
        '''
        Perform a memory write. Caller should trap OSError.
        '''
        self.buf1[0] = data
        self._mpu_i2c.writeto_mem(addr, memaddr, self.buf1)

    # wake
    def wake(self):
        '''
        Wakes the device.
        '''
        try:
            self._write(0x01, 0x6B, self.mpu_addr)  # Use best clock source
        except OSError:
            raise MPUException(self._I2Cerror)
        return 'awake'

    # mode
    def sleep(self):
        '''
        Sets the device to sleep mode.
        '''
        try:
            self._write(0x40, 0x6B, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        return 'asleep'

    # chip_id
    @property
    def chip_id(self):
        '''
        Returns Chip ID
        '''
        try:
            self._read(self.buf1, 0x75, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        chip_id = int(self.buf1[0])
        if chip_id != self._chip_id:
            raise ValueError('Bad chip ID retrieved: MPU communication failure')
        return chip_id

    @property
    def sensors(self):
        '''
        returns sensor objects accel, gyro
        '''
        return self._accel, self._gyro

    # get temperature
    @property
    def temperature(self):
        '''
        Returns the temperature in degree C.
        '''
        try:
            self._read(self.buf2, 0x41, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        return bytes_toint(self.buf2[0], self.buf2[1]) / 340 + 35  # I think

    # passthrough
    @property
    def passthrough(self):
        '''
        Returns passthrough mode True or False
        '''
        try:
            self._read(self.buf1, 0x37, self.mpu_addr)
            return self.buf1[0] & 0x02 > 0
        except OSError:
            raise MPUException(self._I2Cerror)

    @passthrough.setter
    def passthrough(self, mode):
        '''
        Sets passthrough mode True or False
        '''
        if type(mode) is bool:
            val = 2 if mode else 0
            try:
                self._write(val, 0x37, self.mpu_addr)  # I think this is right.
                self._write(0x00, 0x6A, self.mpu_addr)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('pass either True or False')

    # sample rate. Not sure why you'd ever want to reduce this from the default.
    @property
    def sample_rate(self):
        '''
        Get sample rate as per Register Map document section 4.4
        SAMPLE_RATE= Internal_Sample_Rate / (1 + rate)
        default rate is zero i.e. sample at internal rate.
        '''
        try:
            self._read(self.buf1, 0x19, self.mpu_addr)
            return self.buf1[0]
        except OSError:
            raise MPUException(self._I2Cerror)

    @sample_rate.setter
    def sample_rate(self, rate):
        '''
        Set sample rate as per Register Map document section 4.4
        '''
        if rate < 0 or rate > 255:
            raise ValueError("Rate must be in range 0-255")
        try:
            self._write(rate, 0x19, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)

    # Low pass filters. Using the filter_range property of the MPU9250 is
    # harmless but gyro_filter_range is preferred and offers an extra setting.
    @property
    def filter_range(self):
        '''
        Returns the gyro and temperature sensor low pass filter cutoff frequency
        Pass:               0   1   2   3   4   5   6
        Cutoff (Hz):        250 184 92  41  20  10  5
        Sample rate (KHz):  8   1   1   1   1   1   1
        '''
        try:
            self._read(self.buf1, 0x1A, self.mpu_addr)
            res = self.buf1[0] & 7
        except OSError:
            raise MPUException(self._I2Cerror)
        return res

    @filter_range.setter
    def filter_range(self, filt):
        '''
        Sets the gyro and temperature sensor low pass filter cutoff frequency
        Pass:               0   1   2   3   4   5   6
        Cutoff (Hz):        250 184 92  41  20  10  5
        Sample rate (KHz):  8   1   1   1   1   1   1
        '''
        # set range
        if filt in range(7):
            try:
                self._write(filt, 0x1A, self.mpu_addr)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('Filter coefficient must be between 0 and 6')

    # accelerometer range
    @property
    def accel_range(self):
        '''
        Accelerometer range
        Value:              0   1   2   3
        for range +/-:      2   4   8   16  g
        '''
        try:
            self._read(self.buf1, 0x1C, self.mpu_addr)
            ari = self.buf1[0] // 8
        except OSError:
            raise MPUException(self._I2Cerror)
        return ari

    @accel_range.setter
    def accel_range(self, accel_range):
        '''
        Set accelerometer range
        Pass:               0   1   2   3
        for range +/-:      2   4   8   16  g
        '''
        ar_bytes = (0x00, 0x08, 0x10, 0x18)
        if accel_range in range(len(ar_bytes)):
            try:
                self._write(ar_bytes[accel_range], 0x1C, self.mpu_addr)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('accel_range can only be 0, 1, 2 or 3')

    # gyroscope range
    @property
    def gyro_range(self):
        '''
        Gyroscope range
        Value:              0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        '''
        # set range
        try:
            self._read(self.buf1, 0x1B, self.mpu_addr)
            gri = self.buf1[0] // 8
        except OSError:
            raise MPUException(self._I2Cerror)
        return gri

    @gyro_range.setter
    def gyro_range(self, gyro_range):
        '''
        Set gyroscope range
        Pass:               0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        '''
        gr_bytes = (0x00, 0x08, 0x10, 0x18)
        if gyro_range in range(len(gr_bytes)):
            try:
                self._write(gr_bytes[gyro_range], 0x1B, self.mpu_addr)  # Sets fchoice = b11 which enables filter
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('gyro_range can only be 0, 1, 2 or 3')

    # Accelerometer
    @property
    def accel(self):
        '''
        Acceleremoter object
        '''
        return self._accel

    def _accel_callback(self):
        '''
        Update accelerometer Vector3d object
        '''
        try:
            self._read(self.buf6, 0x3B, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        scale = (16384, 8192, 4096, 2048)
        self._accel._vector[0] = self._accel._ivector[0] / scale[self.accel_range]
        self._accel._vector[1] = self._accel._ivector[1] / scale[self.accel_range]
        self._accel._vector[2] = self._accel._ivector[2] / scale[self.accel_range]

    def get_accel_irq(self):
        '''
        For use in interrupt handlers. Sets self._accel._ivector[] to signed
        unscaled integer accelerometer values
        '''
        self._read(self.buf6, 0x3B, self.mpu_addr)
        self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])

    # Gyro
    @property
    def gyro(self):
        '''
        Gyroscope object
        '''
        return self._gyro

    def _gyro_callback(self):
        '''
        Update gyroscope Vector3d object
        '''
        try:
            self._read(self.buf6, 0x43, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        self._gyro._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._gyro._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._gyro._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        scale = (131, 65.5, 32.8, 16.4)
        self._gyro._vector[0] = self._gyro._ivector[0] / scale[self.gyro_range]
        self._gyro._vector[1] = self._gyro._ivector[1] / scale[self.gyro_range]
        self._gyro._vector[2] = self._gyro._ivector[2] / scale[self.gyro_range]

    def get_gyro_irq(self):
        '''
        For use in interrupt handlers. Sets self._gyro._ivector[] to signed
        unscaled integer gyro values. Error trapping disallowed.
        '''
        self._read(self.buf6, 0x43, self.mpu_addr)
        self._gyro._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._gyro._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._gyro._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])


def get_rotation(accel_x, accel_y, accel_z):
    roll = atan2(accel_y, sqrt(accel_x ** 2 + accel_z ** 2))
    pitch = atan2(-accel_x, sqrt(accel_y ** 2 + accel_z ** 2))
    return degrees(roll), degrees(pitch)


def collect_samples(num_samples=100):
    samples = []
    for _ in range(num_samples):
        ax = imu.accel.x
        ay = imu.accel.y
        az = imu.accel.z
        gx = imu.gyro.x
        gy = imu.gyro.y
        gz = imu.gyro.z
        samples.append((ax, ay, az, gx, gy, gz))
        sleep_ms(10)
    return samples


def multi_position_accel_calibration():
    positions = [
        "Place the IMU flat (+Z up)",
        "Rotate 180 degrees around X (-Z up)",
        "Rest on +Y edge (X up)",
        "Rest on -Y edge (X down)",
        "Rest on +X edge (Y up)",
        "Rest on -X edge (Y down)"
    ]
    all_samples = []

    for pos in positions:
        print(f"{pos}. Press Enter when ready...")
        input()
        samples = collect_samples(20)
        all_samples.extend(samples)
        print("Samples collected. Move to next position.")

    # Process samples (simplified for brevity)
    accel_samples = [s[:3] for s in all_samples]
    accel_bias = [sum(axis) / len(accel_samples) for axis in zip(*accel_samples)]
    accel_scale = [16384 / (max(axis) - min(axis)) for axis in zip(*accel_samples)]

    return accel_bias, accel_scale


def gyro_calibration_with_rotation():
    print("Keep the IMU still for initial gyro bias...")
    static_samples = collect_samples(20)
    gyro_bias = [sum(axis) / len(static_samples) for axis in zip(*[s[3:] for s in static_samples])]

    print("Now, slowly rotate the IMU around each axis.")
    for axis in ['X', 'Y', 'Z']:
        print(f"Rotate around {axis} axis. Press Enter to start, then again to stop...")
        input()
        start_time = ticks_ms()
        rotation_samples = collect_samples(20)
        duration = (ticks_ms() - start_time) / 1000  # in seconds
        input()

        # Calculate total rotation
        total_rotation = sum(s[3 + "XYZ".index(axis)] - gyro_bias["XYZ".index(axis)]
                             for s in rotation_samples) * 0.0175  # Convert to radians

        print(f"Estimated rotation: {degrees(total_rotation):.2f} degrees")
        print(f"Rotation rate: {degrees(total_rotation) / duration:.2f} deg/s")

    return gyro_bias


def temperature_compensation():
    print("We'll collect data at different temperatures.")
    temp_ranges = ["Room temperature", "After gentle heating", "After cooling"]
    temp_data = []

    for temp in temp_ranges:
        print(f"Prepare the IMU for {temp} readings. Press Enter when ready...")
        input()
        samples = collect_samples(20)
        temp_data.append((temp, samples))

    # Process temperature data (simplified)
    for temp, samples in temp_data:
        accel_avg = [sum(axis) / len(samples) for axis in zip(*[s[:3] for s in samples])]
        gyro_avg = [sum(axis) / len(samples) for axis in zip(*[s[3:] for s in samples])]
        print(f"{temp} averages:")
        print(f"Accel: {accel_avg}, Gyro: {gyro_avg}")

    # You would typically fit a curve to this data for temperature compensation


# Main calibration routine
def advanced_calibration():
    print("Starting advanced IMU calibration...")

    # accel_bias, accel_scale = multi_position_accel_calibration()
    # print(f"Accelerometer bias: {accel_bias}")
    # print(f"Accelerometer scale factors: {accel_scale}")

    # g yro_bias = gyro_calibration_with_rotation()
    # print(f"Gyroscope bias: {gyro_bias}")
    # print("Advanced calibration complete!")
    return [0.07534748306666673, -0.009919429263333359, 0.05217854646666667], [0.984142318001577, 0.9910474226597741,
                                                                               0.9716522402219719], [0, 0, 0]


class ExtendedKalmanFilter:
    def __init__(self, initial_state, initial_P, Q, R):
        self.x = np.array(initial_state)
        self.P = np.array(initial_P)
        self.Q = np.array(Q)
        self.R = np.array(R)
        self.n = len(initial_state)
        self.m = len(R)
        self.I = np.eye(self.n)
        self.g = 9.81  # acceleration due to gravity

    def convert_accel(self, accel_normalized):
        return [a * self.g for a in accel_normalized]

    def predict(self, u, dt):
        # State transition function

        # Compute Jacobian F
        F = np.eye(self.n)

        # Control matrix
        B = np.eye(self.n)
        # predict 
        self.x = np.dot(F, self.x) + np.dot(B, u)

        # Update covariance
        self.P = np.dot(np.dot(F, self.P), F.transpose()) + self.Q

    def update(self, z, dt):
        # Convert accelerometer readings from normalized to m/s²
        z_converted = np.array(self.convert_accel(z[:3]) + list(z[3:]))

        # Compute Jacobin H
        H = np.zeros((self.m, self.n))
        H[0, 0] = self.g * math.cos(self.x[0]) * math.sin(self.x[1])
        H[0, 1] = self.g * math.sin(self.x[0]) * math.cos(self.x[1])

        H[1, 0] = self.g * math.cos(self.x[0]) * math.cos(self.x[1])
        H[1, 1] = -self.g * math.sin(self.x[0]) * math.sin(self.x[1])

        H[2, 0] = -self.g * math.sin(self.x[0])

        # estimate measurement
        z_hat = np.dot(H, self.x)

        # Compute Kalman gain
        S = np.dot(np.dot(H, self.P), H.transpose()) + self.R
        S_inv = np.linalg.inv(S)
        K = np.dot(np.dot(self.P, H.transpose()), S_inv)
        # Update state
        y = z_converted - z_hat
        self.x = self.x + np.dot(K, y)

        # Update covariance
        self.P = np.dot(self.I - np.dot(K, H), self.P)

    def get_state(self):
        return self.x


i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)
# Run the calibration
accel_bias, accel_scale, gyro_bias = advanced_calibration()
# Initialize EKF
initial_state = np.array([0, 0, 0])
initial_P = np.eye(3)


R = np.eye(7) * 1
ekf = ExtendedKalmanFilter(initial_state, initial_P, Q, R)

# Main loop
last_time = utime.ticks_us()
while True:
    # Get sensor data (replace with actual sensor readings)
    ax = round((imu.accel.x - accel_bias[0]) * accel_scale[0], 2)
    ay = round((imu.accel.y - accel_bias[1]) * accel_scale[1], 2)
    az = round((imu.accel.z - accel_bias[2]) * accel_scale[2], 2)
    gx = round(imu.gyro.x - gyro_bias[0], 2)
    gy = round(imu.gyro.y - gyro_bias[1], 2)
    gz = round(imu.gyro.z - gyro_bias[2], 2)
    tem = round(imu.temperature, 2)

    accel = [ax, ay, az]
    gyro = [gx, gy, gz]
    altitude = 0
    z = np.array(accel + gyro + [altitude])

    # Compute dt
    current_time = utime.ticks_us()
    dt = utime.ticks_diff(current_time, last_time) / 1e6
    last_time = current_time

    B = np.eye(3) * dt
    Q = np.dot(B.transpose(), B) , gyr
    # EKF predict and update steps
    ekf.predict(gyro,dt)
    ekf.update(z, dt)

    # Get updated state
    state = ekf.get_state()
    print(f"ax: {ax} m/s², ay; {ay} m/s², az: {az} m/s²")
    print(
        f"Pitch: {math.degrees(state[0]):.2f}°, Roll: {math.degrees(state[1]):.2f}°, Yaw: {math.degrees(state[2]):.2f}°")
    print(f"Altitude: {state[3]:.2f}m, Vertical Velocity: {state[7]:.2f}m/s")

    utime.sleep_ms(100)  # Adjust as needed
