from machine import Pin, UART
import utime, time

# Initialize UART - adjust pins according to your connection
gpsModule = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
# PMTK251,115200 command with checksum *1F
command = b'$PMTK251,115200*1F\r\n'

print("Sending baud rate change command...")
gpsModule.write(command)

# Global variables
TIMEOUT = False
FIX_STATUS = False
latitude = ""
longitude = ""
satellites = ""
GPStime = ""

def getGPS(gpsModule):
    global FIX_STATUS, TIMEOUT, latitude, longitude, satellites, GPStime
    
    timeout = time.time() + 8   # 8 seconds timeout
    while True:
        gpsModule.readline()  # Clear buffer
        
        try:
            raw_data = gpsModule.readline()
            if not raw_data:
                continue
            
            # Decode the data, ignoring errors
            buff = raw_data.decode('utf-8', errors='ignore').strip()
            parts = buff.split(',')
            
            # Check for GPGGA sentence (Global Positioning System Fix Data)
            if parts[0] == "$GNGGA" and len(parts) > 7:
                if parts[1] and parts[2] and parts[3] and parts[4] and parts[5] and parts[6] and parts[7]:
                    # Convert latitude and longitude
                    latitude = convertToDegree(parts[2])
                    if parts[3] == 'S':
                        latitude = '-' + latitude
                    longitude = convertToDegree(parts[4])
                    if parts[5] == 'W':
                        longitude = '-' + longitude
                        
                    satellites = parts[7]
                    # Format time from HHMMSS.ss to HH:MM:SS
                    GPStime = parts[1][0:2] + ":" + parts[1][2:4] + ":" + parts[1][4:6]
                    FIX_STATUS = True
                    break
                    
        except Exception as e:
            print("Error parsing GPS data:", e)
            
        if time.time() > timeout:
            TIMEOUT = True
            break
            

def convertToDegree(RawDegrees):
    try:
        RawAsFloat = float(RawDegrees)
        firstdigits = int(RawAsFloat / 100) 
        nexttwodigits = RawAsFloat - float(firstdigits * 100) 
        
        Converted = float(firstdigits + nexttwodigits / 60.0)
        Converted = '{0:.6f}'.format(Converted) 
        return str(Converted)
    except ValueError as e:
        print("Error converting degrees:", e)
        return "0.0"

# Open file for logging
try:
    with open("Positioning.txt", 'w') as f:
        f.write("Logging data \n")
        
        n = 0
        while n < 600:  # Run for 600 iterations
            getGPS(gpsModule)
            
            if FIX_STATUS:
                print("Printing GPS data...")
                
                print("Latitude: " + latitude)
                f.write("Latitude: " + latitude + "\n")
                
                print("Longitude: " + longitude)
                f.write("Longitude: " + longitude + "\n")
                
                print("Satellites: " + satellites)
                f.write("Satellites: " + satellites + "\n")
                
                print("Time: " + GPStime)
                f.write("Time: " + GPStime + "\n")
                
                print("----------------------")
                f.write("--------------------\n")
                
                FIX_STATUS = False
                
            if TIMEOUT:
                print("No GPS data is found.")
                TIMEOUT = False
                
            n += 1
        
except Exception as e:
    print("Error during execution:", e)
