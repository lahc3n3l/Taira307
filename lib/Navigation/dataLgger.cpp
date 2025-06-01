// src/dataLogger.cpp
#include "dataLogger.h"

DataLogger::DataLogger() {
    logInterval = 50;  // 50ms = 20Hz
    lastLogTime = 0;
    isReady = false;
}

bool DataLogger::begin() {
    // Initialize SD card for GOOUUU board
    SD_MMC.setPins(39, 38, 40);
    
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("SD Card failed!");
        return false;
    }
    
    // Create filename with timestamp
    fileName = "/flight_" + String(millis()) + ".csv";
    
    dataFile = SD_MMC.open(fileName, FILE_WRITE);
    if (!dataFile) {
        Serial.println("Failed to create file!");
        return false;
    }
    
    // Write CSV header
    dataFile.println("time,roll,pitch,yaw,ax,ay,az,gx,gy,gz,lat,lon,alt");
    dataFile.flush();
    
    isReady = true;
    Serial.println("DataLogger ready: " + fileName);
    return true;
}

void DataLogger::logData(float roll, float pitch, float yaw,
                        float ax, float ay, float az,
                        float gx, float gy, float gz,
                        float gpsLat, float gpsLon, float gpsAlt) {
    
    if (!isReady) return;
    
    unsigned long currentTime = millis();
    
    // Check if it's time to log
    if (currentTime - lastLogTime < logInterval) return;
    
    lastLogTime = currentTime;
    
    // Write data
    dataFile.print(currentTime);
    dataFile.print(",");
    dataFile.print(roll, 2);
    dataFile.print(",");
    dataFile.print(pitch, 2);
    dataFile.print(",");
    dataFile.print(yaw, 2);
    dataFile.print(",");
    dataFile.print(ax, 3);
    dataFile.print(",");
    dataFile.print(ay, 3);
    dataFile.print(",");
    dataFile.print(az, 3);
    dataFile.print(",");
    dataFile.print(gx, 2);
    dataFile.print(",");
    dataFile.print(gy, 2);
    dataFile.print(",");
    dataFile.print(gz, 2);
    dataFile.print(",");
    dataFile.print(gpsLat, 6);
    dataFile.print(",");
    dataFile.print(gpsLon, 6);
    dataFile.print(",");
    dataFile.println(gpsAlt, 1);
    
    // Flush every 20 entries (1 second)
    static int flushCounter = 0;
    if (++flushCounter >= 20) {
        dataFile.flush();
        flushCounter = 0;
    }
}

void DataLogger::end() {
    if (dataFile) {
        dataFile.close();
    }
    isReady = false;
}