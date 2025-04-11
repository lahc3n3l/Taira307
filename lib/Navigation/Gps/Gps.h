#ifndef GPSMODULE_H
#define GPSMODULE_H

#include <HardwareSerial.h>

class GpsModule {
    
public:
    GpsModule(int rxPin, int txPin, int serialPortNumber = 1, unsigned long baudRate = 115200);

    void begin();
    void update();  // Call this in loop()
    void end();
private:
    HardwareSerial* gpsSerial;
    int rxPin;
    int txPin;
    unsigned long baudRate;
    String gpsSentence;

    void parseSentence(const String& sentence);
};

class GpsData {
public:
    float getLongitude() const { return lng; }
    float getLatitude() const { return lat; }
    float getAltitude() const { return alt; }
    float getGroundSpeed() const { return groundSpeed; }
    float getCourse() const { return course; }
    float getHdop() const { return hdop; }
public:
    GpsData() = default;
    private:
    float lng=0.0; // longitude (°)
    float lat=0.0; // latitude  (°)
    float alt=0.0;  // altitude (m)
    float groundSpeed=0.0; // (knots)
    float course=0.0; // (°) 0-360
    float hdop=0.0; // horizontal dilution of precision (m)
}


#endif
// GPSMODULE_H