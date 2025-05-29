// DataLogger.cpp
#include "DataLogger.h"

// ESP32-S3 N16R8 SD card pins (built-in SD card slot)
#define SD_CLK_PIN  12
#define SD_CMD_PIN  11  
#define SD_D0_PIN   13
#define SD_D1_PIN   14
#define SD_D2_PIN   9
#define SD_D3_PIN   10

DataLogger::DataLogger(unsigned long interval, size_t bufferSize) {
    // No chip select pin needed for integrated SD card
    chipSelectPin = -1;
    logInterval = interval;
    lastLogTime = 0;
    loggingEnabled = false;
    sdCardInitialized = false;
    
    bufferMaxSize = bufferSize;
    flushInterval = 20;  // Flush every 20 entries by default
    currentBufferEntries = 0;
    
    logDirectory = "/flight_logs";
    filePrefix = "flight_log";
    maxFileSize = 10 * 1024 * 1024;  // 10MB default
    currentFileNumber = 0;
}

bool DataLogger::begin() {
    return begin("/flight_logs", "flight_log");
}

bool DataLogger::begin(String directory, String prefix) {
    logDirectory = directory;
    filePrefix = prefix;
    
    Serial.println("Initializing integrated SD card...");
    
    // Initialize SD card with 4-bit mode for ESP32-S3 N16R8
    // Use SD_MMC for the integrated SD card slot
    if (!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT, 5)) {
        Serial.println("SD card initialization failed!");
        Serial.println("Make sure SD card is properly inserted");
        return false;
    }
    
    // Check card type
    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return false;
    }
    
    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    
    sdCardInitialized = true;
    Serial.println("Integrated SD card initialized successfully");
    
    // Create log directory if it doesn't exist
    if (!SD_MMC.exists(logDirectory)) {
        if (SD_MMC.mkdir(logDirectory)) {
            Serial.println("Created log directory: " + logDirectory);
        } else {
            Serial.println("Failed to create log directory");
            return false;
        }
    }
    
    // Create initial log file
    if (!createNewLogFile()) {
        Serial.println("Failed to create initial log file");
        return false;
    }
    
    loggingEnabled = true;
    Serial.println("DataLogger initialized successfully");
    Serial.println("Log file: " + currentLogFile);
    
    return true;
}

bool DataLogger::createNewLogFile() {
    if (!sdCardInitialized) return false;
    
    currentLogFile = generateFileName();
    
    // Create file and write header using SD_MMC
    File logFile = SD_MMC.open(currentLogFile, FILE_WRITE);
    if (!logFile) {
        Serial.println("Failed to create log file: " + currentLogFile);
        return false;
    }
    
    // Write CSV header
    String header = "timestamp,roll,pitch,yaw,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,";
    header += "latitude,longitude,altitude,groundSpeed,heading,numSatellites,";
    header += "rollCommand,pitchCommand,flapsCommand,leftServo,rightServo,pitchServo,";
    header += "targetRoll,targetPitch,batteryVoltage,cpuTemperature,rcSignalValid\n";
    
    logFile.print(header);
    logFile.close();
    
    Serial.println("Created new log file: " + currentLogFile);
    return true;
}

String DataLogger::generateFileName() {
    currentFileNumber++;
    
    // Get current time for filename (you might want to use RTC if available)
    unsigned long timestamp = millis();
    
    String fileName = logDirectory + "/" + filePrefix + "_";
    fileName += String(timestamp / 1000);  // Convert to seconds
    fileName += "_" + String(currentFileNumber);
    fileName += ".csv";
    
    return fileName;
}

bool DataLogger::logFlightData(const FlightData& data) {
    if (!loggingEnabled || !sdCardInitialized) return false;
    
    // Check if enough time has passed since last log
    if (millis() - lastLogTime < logInterval) return false;
    
    // Check if file needs rotation
    checkAndRotateFile();
    
    // Build CSV line
    String logLine = String(data.timestamp) + ",";
    logLine += sanitizeValue(data.roll) + ",";
    logLine += sanitizeValue(data.pitch) + ",";
    logLine += sanitizeValue(data.yaw) + ",";
    logLine += sanitizeValue(data.accelX) + ",";
    logLine += sanitizeValue(data.accelY) + ",";
    logLine += sanitizeValue(data.accelZ) + ",";
    logLine += sanitizeValue(data.gyroX) + ",";
    logLine += sanitizeValue(data.gyroY) + ",";
    logLine += sanitizeValue(data.gyroZ) + ",";
    logLine += sanitizeValue(data.latitude) + ",";
    logLine += sanitizeValue(data.longitude) + ",";
    logLine += sanitizeValue(data.altitude) + ",";
    logLine += sanitizeValue(data.groundSpeed) + ",";
    logLine += sanitizeValue(data.heading) + ",";
    logLine += String(data.numSatellites) + ",";
    logLine += sanitizeValue(data.rollCommand) + ",";
    logLine += sanitizeValue(data.pitchCommand) + ",";
    logLine += sanitizeValue(data.flapsCommand) + ",";
    logLine += sanitizeValue(data.leftServo) + ",";
    logLine += sanitizeValue(data.rightServo) + ",";
    logLine += sanitizeValue(data.pitchServo) + ",";
    logLine += sanitizeValue(data.targetRoll) + ",";
    logLine += sanitizeValue(data.targetPitch) + ",";
    logLine += sanitizeValue(data.batteryVoltage) + ",";
    logLine += sanitizeValue(data.cpuTemperature) + ",";
    logLine += String(data.rcSignalValid ? 1 : 0) + "\n";
    
    // Add to buffer
    logBuffer += logLine;
    currentBufferEntries++;
    lastLogTime = millis();
    
    // Check if buffer needs flushing
    if (currentBufferEntries >= flushInterval || logBuffer.length() >= bufferMaxSize) {
        flushBuffer();
    }
    
    return true;
}

bool DataLogger::logMessage(String message, String level) {
    if (!loggingEnabled || !sdCardInitialized) return false;
    
    String logLine = String(millis()) + "," + level + "," + message + "\n";
    
    // For messages, write directly to a separate message log using SD_MMC
    String msgFile = logDirectory + "/messages.txt";
    File messageFile = SD_MMC.open(msgFile, FILE_APPEND);
    if (messageFile) {
        messageFile.print(formatTimestamp(millis()) + " [" + level + "] " + message + "\n");
        messageFile.close();
        return true;
    }
    
    return false;
}

bool DataLogger::logGPSData(float lat, float lon, float alt, float speed, float heading, int sats) {
    if (!loggingEnabled || !sdCardInitialized) return false;
    
    String logLine = String(millis()) + ",GPS,";
    logLine += sanitizeValue(lat) + ",";
    logLine += sanitizeValue(lon) + ",";
    logLine += sanitizeValue(alt) + ",";
    logLine += sanitizeValue(speed) + ",";
    logLine += sanitizeValue(heading) + ",";
    logLine += String(sats) + "\n";
    
    String gpsFile = logDirectory + "/gps_only.csv";
    File gpsLog = SD.open(gpsFile, FILE_APPEND);
    if (gpsLog) {
        // Write header if file is new
        if (gpsLog.size() == 0) {
            gpsLog.println("timestamp,type,latitude,longitude,altitude,groundSpeed,heading,numSatellites");
        }
        gpsLog.print(logLine);
        gpsLog.close();
        return true;
    }
    
    return false;
}

bool DataLogger::logIMUData(float roll, float pitch, float yaw, float ax, float ay, float az, float gx, float gy, float gz) {
    if (!loggingEnabled || !sdCardInitialized) return false;
    
    String logLine = String(millis()) + ",IMU,";
    logLine += sanitizeValue(roll) + ",";
    logLine += sanitizeValue(pitch) + ",";
    logLine += sanitizeValue(yaw) + ",";
    logLine += sanitizeValue(ax) + ",";
    logLine += sanitizeValue(ay) + ",";
    logLine += sanitizeValue(az) + ",";
    logLine += sanitizeValue(gx) + ",";
    logLine += sanitizeValue(gy) + ",";
    logLine += sanitizeValue(gz) + "\n";
    
    String imuFile = logDirectory + "/imu_only.csv";
    File imuLog = SD.open(imuFile, FILE_APPEND);
    if (imuLog) {
        // Write header if file is new
        if (imuLog.size() == 0) {
            imuLog.println("timestamp,type,roll,pitch,yaw,accelX,accelY,accelZ,gyroX,gyroY,gyroZ");
        }
        imuLog.print(logLine);
        imuLog.close();
        return true;
    }
    
    return false;
}

bool DataLogger::logControlData(float rollCmd, float pitchCmd, float flapsCmd, float leftServo, float rightServo, float pitchServo) {
    if (!loggingEnabled || !sdCardInitialized) return false;
    
    String logLine = String(millis()) + ",CONTROL,";
    logLine += sanitizeValue(rollCmd) + ",";
    logLine += sanitizeValue(pitchCmd) + ",";
    logLine += sanitizeValue(flapsCmd) + ",";
    logLine += sanitizeValue(leftServo) + ",";
    logLine += sanitizeValue(rightServo) + ",";
    logLine += sanitizeValue(pitchServo) + "\n";
    
    String controlFile = logDirectory + "/control_only.csv";
    File controlLog = SD.open(controlFile, FILE_APPEND);
    if (controlLog) {
        // Write header if file is new
        if (controlLog.size() == 0) {
            controlLog.println("timestamp,type,rollCommand,pitchCommand,flapsCommand,leftServo,rightServo,pitchServo");
        }
        controlLog.print(logLine);
        controlLog.close();
        return true;
    }
    
    return false;
}

void DataLogger::flushBuffer() {
    if (logBuffer.length() == 0 || !sdCardInitialized) return;
    
    File logFile = SD_MMC.open(currentLogFile, FILE_APPEND);
    if (logFile) {
        logFile.print(logBuffer);
        logFile.close();
        
        // Clear buffer
        logBuffer = "";
        currentBufferEntries = 0;
    } else {
        Serial.println("Failed to open log file for writing");
    }
}

bool DataLogger::checkAndRotateFile() {
    if (getFileSize() > maxFileSize) {
        flushBuffer(); // Flush current buffer before rotating
        return createNewLogFile();
    }
    return true;
}

long DataLogger::getFileSize() {
    if (!sdCardInitialized || currentLogFile.length() == 0) return 0;
    
    File logFile = SD_MMC.open(currentLogFile, FILE_READ);
    if (logFile) {
        long size = logFile.size();
        logFile.close();
        return size;
    }
    return 0;
}

String DataLogger::sanitizeValue(float value) {
    if (isnan(value) || isinf(value)) {
        return "0.0";
    }
    return String(value, 6); // 6 decimal places
}

String DataLogger::formatTimestamp(unsigned long timestamp) {
    // Simple timestamp formatting (you might want to use RTC for real dates)
    unsigned long seconds = timestamp / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    
    seconds %= 60;
    minutes %= 60;
    hours %= 24;
    
    String formatted = "";
    if (hours < 10) formatted += "0";
    formatted += String(hours) + ":";
    if (minutes < 10) formatted += "0";
    formatted += String(minutes) + ":";
    if (seconds < 10) formatted += "0";
    formatted += String(seconds);
    
    return formatted;
}

void DataLogger::setLogInterval(unsigned long interval) {
    logInterval = interval;
}

void DataLogger::setMaxFileSize(int sizeInBytes) {
    maxFileSize = sizeInBytes;
}

void DataLogger::setFlushInterval(size_t entries) {
    flushInterval = entries;
}

void DataLogger::enableLogging(bool enable) {
    loggingEnabled = enable;
    if (enable) {
        Serial.println("Data logging enabled");
    } else {
        Serial.println("Data logging disabled");
    }
}

bool DataLogger::isLoggingEnabled() const {
    return loggingEnabled;
}

bool DataLogger::checkSDCard() {
    return sdCardInitialized && SD_MMC.exists("/");
}

void DataLogger::listFiles() {
    if (!sdCardInitialized) {
        Serial.println("SD card not initialized");
        return;
    }
    
    File dir = SD_MMC.open(logDirectory);
    if (!dir) {
        Serial.println("Failed to open directory");
        return;
    }
    
    Serial.println("Files in " + logDirectory + ":");
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) break;
        
        Serial.print("  ");
        Serial.print(entry.name());
        Serial.print("  (");
        Serial.print(entry.size());
        Serial.println(" bytes)");
        
        entry.close();
    }
    dir.close();
}

void DataLogger::flush() {
    flushBuffer();
}

void DataLogger::printStatus() {
    Serial.println("=== DataLogger Status ===");
    Serial.println("SD Card: " + String(sdCardInitialized ? "OK" : "FAILED"));
    Serial.println("Logging: " + String(loggingEnabled ? "ENABLED" : "DISABLED"));
    Serial.println("Current file: " + currentLogFile);
    Serial.println("File size: " + String(getFileSize()) + " bytes");
    Serial.println("Buffer entries: " + String(currentBufferEntries));
    Serial.println("Log interval: " + String(logInterval) + " ms");
    Serial.println("========================");
}

String DataLogger::getCurrentLogFile() const {
    return currentLogFile;
}

bool DataLogger::closeCurrentFile() {
    flushBuffer();
    return true;
}

bool DataLogger::deleteOldFiles(int keepLastN) {
    // Implementation for deleting old files to save space
    // This is a simplified version - you might want to implement proper file sorting by date
    return true;
}

DataLogger::~DataLogger() {
    if (loggingEnabled) {
        flushBuffer();
    }
}