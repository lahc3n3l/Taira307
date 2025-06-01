// include/dataLogger.h
#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <Arduino.h>
#include <SD_MMC.h>

class DataLogger {
private:
    File dataFile;
    String fileName;
    unsigned long lastLogTime;
    unsigned long logInterval;
    bool isReady;
    
public:
    DataLogger();
    bool begin();
    void logData(float roll, float pitch, float yaw,
                 float ax, float ay, float az,
                 float gx, float gy, float gz,
                 float gpsLat, float gpsLon, float gpsAlt);
    void end();
};

#endif