// src/testSD_goouuu.cpp
#include <Arduino.h>
#include <SD_MMC.h>
void setup() {
    Serial.begin(115200);
    delay(3000);  // Wait for serial
    
    Serial.println("\nGOOUUU ESP32-S3-CAM SD Card Test");
    Serial.println("================================\n");
    
    // Set correct pins for GOOUUU board
    SD_MMC.setPins(39, 38, 40);  // CLK, CMD, D0
    
    // Initialize in 1-bit mode
    if (SD_MMC.begin("/sdcard", true)) {
        Serial.println("SUCCESS! SD card initialized!");
        
        uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
        Serial.printf("Card Size: %llu MB\n", cardSize);
        
        // Test write
        File file = SD_MMC.open("/test.txt", FILE_WRITE);
        if (file) {
            file.println("GOOUUU ESP32-S3-CAM SD test OK!");
            file.close();
            Serial.println("File written successfully!");
        }
    } else {
        Serial.println("SD Card initialization failed!");
        Serial.println("Check: SD card inserted and formatted as FAT32");
    }
}

void loop() {
    delay(5000);
}