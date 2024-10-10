from machine import Pin, UART

import utime, time


gpsModule = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
print(gpsModule)

buff = bytearray(255)

TIMEOUT = False
FIX_STATUS = False

latitude = ""
longitude = ""
satellites = ""
GPStime = ""

def getGPS(gpsModule):
    global FIX_STATUS, TIMEOUT, latitude, longitude, satellites, GPStime
    
    timeout = time.time() + 8 
    while True:
        gpsModule.readline()
        buff = str(gpsModule.readline())
        parts = buff.split(',')
        if (parts[0] == "b'$GPGGA" and len(parts) == 15):
            if(parts[1] and parts[2] and parts[3] and parts[4] and parts[5] and parts[6] and parts[7]):
                
                latitude = convertToDegree(parts[2])
                if (parts[3] == 'S'):
                    latitude = -latitude
                longitude = convertToDegree(parts[4])
                if (parts[5] == 'W'):
                    longitude = -longitude
                satellites = parts[7]
                GPStime = parts[1][0:2] + ":" + parts[1][2:4] + ":" + parts[1][4:6]
                FIX_STATUS = True
                break
                
        if (time.time() > timeout):
            TIMEOUT = True
            break
        utime.sleep_ms(500)
        
def convertToDegree(RawDegrees):

    RawAsFloat = float(RawDegrees)
    firstdigits = int(RawAsFloat/100) 
    nexttwodigits = RawAsFloat - float(firstdigits*100) 
    
    Converted = float(firstdigits + nexttwodigits/60.0)
    Converted = '{0:.6f}'.format(Converted) 
    return str(Converted)


f = open("Positioning.txt", 'w')
f.write("Logging data \n")
n = 0
while(n<600):
    
    getGPS(gpsModule)

    if(FIX_STATUS == True):
        print("Printing GPS data...")
        
        print("Latitude: "+latitude)
        f.write("Latitude: "+latitude+ "\n")
        
        print("longitude: "+longitude)
        f.write("longitude: "+longitude+ "\n")
        
        print("Satellites: " +satellites)
        f.write("Satellites: "+satellites+ "\n")
        
        print("Time: "+GPStime)
        f.write("Time: "+GPStime+ "\n")
         i
        print("----------------------")
        f.write("--------------------\n")
        
        FIX_STATUS = False
        
        
    if(TIMEOUT == True):
        print("No GPS data is found.")
        TIMEOUT = False
        
    n += 1
        
    
f.close()
    