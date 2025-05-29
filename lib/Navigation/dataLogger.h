// DataLogger.h
#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <SD.h>
#include "SD_MMC.h"

#include <SPI.h>
#include <FS.h>

class DataLogger {
private:
    // SD Card configuration
    int chipSelectPin;
    bool sdCardInitialized;
    String currentLogFile;
    
    // Logging parameters
    unsigned long logInterval;        // Minimum time between logs (ms)
    unsigned long lastLogTime;        // Last time data was logged
    bool loggingEnabled;
    
    // Buffer for efficient writing
    String logBuffer;
    size_t bufferMaxSize;
    size_t flushInterval;             // Number of entries before forced flush
    size_t currentBufferEntries;
    
    // File management
    String logDirectory;
    String filePrefix;
    int maxFileSize;                  // Maximum file size in bytes
    int currentFileNumber;
    
    // Helper methods
    String generateFileName();
    String formatTimestamp(unsigned long timestamp);
    bool checkAndRotateFile();
    void flushBuffer();
    String sanitizeValue(float value);
    
public:
    // Constructor - ESP32-S3 N16R8 has integrated SD card
    DataLogger(unsigned long interval = 100, size_t bufferSize = 2048);
    
    // Initialization
    bool begin();
    bool begin(String directory, String prefix = "flight_log");
    
    // Configuration methods
    void setLogInterval(unsigned long interval);
    void setMaxFileSize(int sizeInBytes);
    void setFlushInterval(size_t entries);
    void enableLogging(bool enable = true);
    bool isLoggingEnabled() const;
    
    // Data logging methods
    struct FlightData {
        unsigned long timestamp;
        
        // IMU data
        float roll;
        float pitch;
        float yaw;
        float accelX, accelY, accelZ;
        float gyroX, gyroY, gyroZ;
        
        // GPS data
        float latitude;
        float longitude;
        float altitude;
        float groundSpeed;
        float heading;
        int numSatellites;
        
        // Control data
        float rollCommand;
        float pitchCommand;
        float flapsCommand;
        float leftServo;
        float rightServo;
        float pitchServo;
        
        // Flight controller data
        float targetRoll;
        float targetPitch;
        
        // System data
        float batteryVoltage;
        float cpuTemperature;
        bool rcSignalValid;
    };
    
    // Log flight data
    bool logFlightData(const FlightData& data);
    
    // Log custom message
    bool logMessage(String message, String level = "INFO");
    
    // Log GPS data specifically
    bool logGPSData(float lat, float lon, float alt, float speed, float heading, int sats);
    
    // Log IMU data specifically
    bool logIMUData(float roll, float pitch, float yaw, float ax, float ay, float az, float gx, float gy, float gz);
    
    // Log control data specifically
    bool logControlData(float rollCmd, float pitchCmd, float flapsCmd, float leftServo, float rightServo, float pitchServo);
    
    // File management
    bool createNewLogFile();
    String getCurrentLogFile() const;
    long getFileSize();
    bool closeCurrentFile();
    
    // SD Card management
    bool checkSDCard();
    void listFiles();
    bool deleteOldFiles(int keepLastN = 10);
    
    // Utility methods
    void flush();
    void printStatus();
    
    // Destructor
    ~DataLogger();
};

#endif // DATALOGGER_H