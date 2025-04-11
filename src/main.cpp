#include <Arduino.h>
#include "Gps.h"

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD_RATE 115200

GpsModule gps(GPS_RX_PIN, GPS_TX_PIN);

void setup() {
  Serial.begin(GPS_BAUD_RATE);
  gps.begin();
  Serial.println("GPS Location Parser Ready");
}

void loop() {
  Serial.println("Waiting for GPS data...");
  gps.update(); // Start the GPS module and begin reading data
  delay(100); // Wait for a second before the next read
}
