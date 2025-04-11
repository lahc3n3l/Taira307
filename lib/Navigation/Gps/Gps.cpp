/*
This libray is used to enable GNSS-based navigation on the ESP32 platform.
Author: 2023, Mohammad Ghannam, Lahcen Elmosafir

License: MIT
*/

#include "Gps.h"

GpsModule::GpsModule(int rxPin, int txPin, int serialPortNumber, unsigned long baudRate)
    : rxPin(rxPin), txPin(txPin), baudRate(baudRate)
{
    gpsSerial = new HardwareSerial(serialPortNumber);
}

void GpsModule::begin() {
    gpsSerial->begin(baudRate, SERIAL_8N1, rxPin, txPin);
    Serial.println("GPS Module Initialized");
}

void GpsModule::parseSentence(const String& sentence) {
    // Check for a valid NMEA sentence
    if (!sentence.startsWith("$")) return;

    // Split sentence into comma-separated fields
    int fieldIndex = 0;
    int startIdx = 0;
    String fields[20];  // assuming max 20 fields per sentence

    for (int i = 0; i < sentence.length(); i++) {
        if (sentence[i] == ',' || sentence[i] == '*') {
            fields[fieldIndex++] = sentence.substring(startIdx, i);
            startIdx = i + 1;
        }
    }

    // Handle specific sentence types
    if (fields[0] == "$GPGGA" || fields[0] == "$GNGGA") {
        // Latitude
        if (fields[2].length() > 0 && fields[3].length() > 0) {
            float rawLat = fields[2].toFloat(); // e.g., 4807.038
            float deg = floor(rawLat / 100);
            float min = rawLat - (deg * 100);
            float lat = deg + (min / 60.0);
            if (fields[3] == "S") lat *= -1;
            gpsData.setLatitude(lat);
        }

        // Longitude
        if (fields[4].length() > 0 && fields[5].length() > 0) {
            float rawLng = fields[4].toFloat(); // e.g., 01131.000
            float deg = floor(rawLng / 100);
            float min = rawLng - (deg * 100);
            float lng = deg + (min / 60.0);
            if (fields[5] == "W") lng *= -1;
            gpsData.setLongitude(lng);
        }

        // Altitude
        if (fields[9].length() > 0) {
            gpsData.setAltitude(fields[9].toFloat());
        }

        // HDOP
        if (fields[8].length() > 0) {
            gpsData.setHdop(fields[8].toFloat());
        }
    }

    else if (fields[0] == "$GPRMC" || fields[0] == "$GNRMC") {
        // Speed in knots
        if (fields[7].length() > 0) {
            gpsData.setGroundSpeed(fields[7].toFloat());
        }

        // Course
        if (fields[8].length() > 0) {
            gpsData.setCourse(fields[8].toFloat());
        }
    }
}

void GpsModule::update() {
    while (gpsSerial->available()) {
        char c = gpsSerial->read();
        if (c == '\n') {
            parseSentence(gpsSentence);
            gpsSentence = ""; // Clear the sentence after parsing
        } else {
            gpsSentence += c; // Append character to the sentence
        }
    }

    // print the nmea sentence
    if (gpsSentence.length() > 0) {
        Serial.println(gpsSentence); // Print the NMEA sentence for debugging
    }
}