from machine import Pin, I2C
import time
import struct

# BMP280 default address
BMP280_I2C_ADDR = 0x76

# BMP280 registers
REG_CALIBRATION = 0x88
REG_ID = 0xD0
REG_RESET = 0xE0
REG_STATUS = 0xF3
REG_CTRL_MEAS = 0xF4
REG_CONFIG = 0xF5
REG_PRESSURE = 0xF7
REG_TEMP = 0xFA

class BMP280:
    def __init__(self, i2c, addr=BMP280_I2C_ADDR):
        self.i2c = i2c
        self.addr = addr
        self.calibration = {}
        self.init_sensor()
        self.read_calibration()

    def init_sensor(self):
        # Soft reset
        self.i2c.writeto_mem(self.addr, REG_RESET, b'\xB6')
        time.sleep_ms(100)
        
        # Configure sensor
        self.i2c.writeto_mem(self.addr, REG_CTRL_MEAS, b'\x27')  # Temp and pressure oversampling x1, normal mode
        self.i2c.writeto_mem(self.addr, REG_CONFIG, b'\x00')     # No filtering, no standby time

    def read_calibration(self):

        temp_cal = self.i2c.readfrom_mem(self.addr, REG_CALIBRATION, 6)
        self.calibration['T1'], self.calibration['T2'], self.calibration['T3'] = struct.unpack('<HhH', temp_cal)

        # Read pressure calibration data
        press_cal = self.i2c.readfrom_mem(self.addr, REG_CALIBRATION + 6, 18)
        self.calibration['P1'], self.calibration['P2'], self.calibration['P3'], \
        self.calibration['P4'], self.calibration['P5'], self.calibration['P6'], \
        self.calibration['P7'], self.calibration['P8'], self.calibration['P9'] = struct.unpack('<HhhHhhhhh', press_cal)


    def read_raw_data(self):
        data = self.i2c.readfrom_mem(self.addr, REG_PRESSURE, 6)
        raw_press = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        raw_temp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        return (raw_temp, raw_press)

    def read_temperature(self):
        raw_temp, _ = self.read_raw_data()
        var1 = ((raw_temp / 16384.0 - self.calibration['T1'] / 1024.0) * self.calibration['T2'])
        var2 = ((raw_temp / 131072.0 - self.calibration['T1'] / 8192.0) * (raw_temp / 131072.0 - self.calibration['T1'] / 8192.0) * self.calibration['T3'])
        self.t_fine = var1 + var2
        return self.t_fine / 5120.0

    def read_pressure(self):
        _, raw_press = self.read_raw_data()
        var1 = self.t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.calibration['P6'] / 32768.0
        var2 = var2 + var1 * self.calibration['P5'] * 2.0
        var2 = var2 / 4.0 + self.calibration['P4'] * 65536.0
        var1 = (self.calibration['P3'] * var1 * var1 / 524288.0 + self.calibration['P2'] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.calibration['P1']
        
        if var1 == 0:
            return 0
        
        pressure = 1048576.0 - raw_press
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = self.calibration['P9'] * pressure * pressure / 2147483648.0
        var2 = pressure * self.calibration['P8'] / 32768.0
        pressure = pressure + (var1 + var2 + self.calibration['P7']) / 16.0
        
        return pressure / 100.0

    def read_altitude(self, sea_level_pressure=1018.25):
        pressure = self.read_pressure()
        return 44330 * (1 - (pressure / sea_level_pressure) ** (1/5.255))

# Initialize I2C
i2c = I2C(0, sda=Pin(0), scl=Pin(1))

# Initialize BMP280
bmp = BMP280(i2c)

def main():
    while True:
        temperature = bmp.read_temperature()
        pressure = bmp.read_pressure()
        altitude = bmp.read_altitude()
        
        print(f"Temperature: {temperature:.2f} °C, Pressure: {pressure/1000:.2f} khPa, Altitude: {altitude:.2f} m")

        time.sleep(0.08)

if __name__ == "__main__":
    main()