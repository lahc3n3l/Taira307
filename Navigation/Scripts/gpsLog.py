from machine import Pin, UART
import utime, time

class GPSParser:
    def __init__(self, uart_id=1, baudrate=115200, tx_pin=4, rx_pin=5):
        self.gpsModule = UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
        # Configure UART with a larger buffer and timeout
        self.gpsModule.init(baudrate=baudrate, bits=8, parity=None, stop=1, 
                           tx=Pin(tx_pin), rx=Pin(rx_pin), timeout=1000)
        
        self.gps_data = {
            'latitude': "",
            'longitude': "",
            'satellites': "",
            'time': "",
            'altitude': "",
            'speed': "",
            'fix_quality': "",
            'fix_status': False,
            'timeout': False
        }
    
    def is_valid_nmea(self, sentence):
        """Check if the sentence is a valid NMEA sentence"""
        if not sentence.startswith("b'$"):
            return False
            
        # Clean the sentence
        try:
            # Remove b' prefix and trailing '
            cleaned = sentence.strip("b'").strip("'")
            # Check if it starts with $ and has basic NMEA format
            if not cleaned.startswith('$'):
                return False
            # Check if it has the basic parts (header, data, checksum)
            parts = cleaned.split('*')
            if len(parts) != 2:
                return False
            return True
        except:
            return False
    
    def clean_nmea_str(self, raw_str):
        """Clean the NMEA string from binary artifacts"""
        try:
            # Remove b' prefix and trailing '
            cleaned = raw_str.strip("b'").strip("'")
            # Remove any escaped characters
            cleaned = cleaned.replace('\\r', '').replace('\\n', '')
            return cleaned
        except:
            return ""
    
    def read_until_valid(self, timeout_ms=1000):
        """Read from UART until a valid NMEA sentence is found"""
        start_time = time.ticks_ms()
        
        while time.ticks_diff(time.ticks_ms(), start_time) < timeout_ms:
            if self.gpsModule.any():  # Check if there's data available
                try:
                    raw_line = str(self.gpsModule.readline())
                    if self.is_valid_nmea(raw_line):
                        clean_line = self.clean_nmea_str(raw_line)
                        return clean_line
                except:
                    pass
            utime.sleep_ms(10)
        return None

    def get_gps_data(self, timeout_seconds=8):
        """Get GPS data from any available valid NMEA sentence"""
        self.gps_data['fix_status'] = False
        self.gps_data['timeout'] = False
        timeout = time.time() + timeout_seconds
        
        while True:
            sentence = self.read_until_valid()
            if sentence:
                try:
                    if not sentence.startswith('$'):
                        continue
                        
                    parts = sentence.split(',')
                    sentence_type = parts[0][1:]  # Remove $
                    
                    # Debug print to see clean sentences
                    print(f"Processing NMEA: {sentence_type}")
                    
                    # Process different sentence types
                    if sentence_type in ['GPGGA', 'GNGGA']:
                        if len(parts) > 14 and parts[2] and parts[4]:
                            self.gps_data['time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}"
                            self.gps_data['latitude'] = self.convert_to_degree(parts[2])
                            if parts[3] == 'S':
                                self.gps_data['latitude'] = '-' + self.gps_data['latitude']
                            self.gps_data['longitude'] = self.convert_to_degree(parts[4])
                            if parts[5] == 'W':
                                self.gps_data['longitude'] = '-' + self.gps_data['longitude']
                            self.gps_data['fix_quality'] = parts[6]
                            self.gps_data['satellites'] = parts[7]
                            self.gps_data['altitude'] = parts[9]
                            self.gps_data['fix_status'] = True
                            break
                    
                    elif sentence_type in ['GPGLL', 'GNGLL']:
                        if len(parts) > 7 and parts[1] and parts[3]:
                            self.gps_data['latitude'] = self.convert_to_degree(parts[1])
                            if parts[2] == 'S':
                                self.gps_data['latitude'] = '-' + self.gps_data['latitude']
                            self.gps_data['longitude'] = self.convert_to_degree(parts[3])
                            if parts[4] == 'W':
                                self.gps_data['longitude'] = '-' + self.gps_data['longitude']
                            self.gps_data['time'] = f"{parts[5][0:2]}:{parts[5][2:4]}:{parts[5][4:6]}"
                            self.gps_data['fix_status'] = True
                            break
                            
                except Exception as e:
                    print(f"Error parsing sentence: {e}")
            
            if time.time() > timeout:
                self.gps_data['timeout'] = True
                break
            
            utime.sleep_ms(100)
        
        return self.gps_data
    
    def convert_to_degree(self, raw_degrees):
        try:
            raw_as_float = float(raw_degrees)
            firstdigits = int(raw_as_float/100)
            nexttwodigits = raw_as_float - float(firstdigits*100)
            converted = float(firstdigits + nexttwodigits/60.0)
            return '{0:.6f}'.format(converted)
        except Exception as e:
            print(f"Error converting degrees: {e}")
            return "0.0"

def log_gps_data(filename="PedestrianPoissyLast.txt", iterations=600):
    gps = GPSParser()
    
    try:
        with open(filename, 'w') as f:
            f.write("Logging data\n")
            
            for n in range(iterations):
                print(f"\nIteration {n+1}/{iterations}")
                gps_data = gps.get_gps_data()
                
                if gps_data['fix_status']:
                    print("Valid GPS data received:")
                    
                    # Log all available data
                    for key, value in gps_data.items():
                        if value and key not in ['fix_status', 'timeout']:
                            print(f"{key}: {value}")
                            f.write(f"{key}: {value}\n")
                    
                    print("----------------------")
                    f.write("--------------------\n")
                
                if gps_data['timeout']:
                    print("No valid GPS data found in timeout period.")
                    
                utime.sleep_ms(500)  # Add small delay between iterations
    
    except Exception as e:
        print(f"Error during execution: {e}")

# Run the logging
if __name__ == '__main__':
    log_gps_data()