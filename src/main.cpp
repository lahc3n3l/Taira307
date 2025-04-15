#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
// include wire.h for I2C communication
#include <Wire.h>



// UART pins for GPS
#define GPS_RX_PIN 16  // ESP32 RX <- GPS TX
#define GPS_TX_PIN 17  // ESP32 TX -> GPS RX
#define GPS_BAUD 115200 // GPS baud rate
HardwareSerial gpsSerial(2); // UART2  
SFE_UBLOX_GNSS myGNSS;


void setup() {
  Serial.begin(115200); // USB serial for debug
  delay(1000);



  // ---------------- GPS Module Setup ----------------
  Serial.println("Initializing GPS module...");
  // Begin UART communication with GPS
  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (myGNSS.begin(gpsSerial)) {
    Serial.println("GNSS module connected!");
  } else {
    Serial.println("Failed to connect to GNSS module.");
  }

  // Optional tuning
  myGNSS.setNavigationFrequency(1); // 1 Hz updates
  myGNSS.setAutoPVT(true); // Enable automatic PVT messages

  // ---------- End GPS Module Setup ----------
}

void loop() {



  
  if (myGNSS.getPVT()) {
    // Position
    Serial.print("Lat: ");
    Serial.println(myGNSS.getLatitude() / 1e7, 7); // degrees
    Serial.print("Lon: ");
    Serial.println(myGNSS.getLongitude() / 1e7, 7);
    Serial.print("Alt: ");
    Serial.println(myGNSS.getAltitude() / 1000.0); // meters

    // Velocity
    Serial.print("Speed (3D): ");
    Serial.println(myGNSS.getGroundSpeed() / 1000.0); // m/s
    Serial.print("Heading: ");
    Serial.println(myGNSS.getHeading() / 1e5); // degrees

    // Time
    Serial.printf("UTC Time: %02d:%02d:%02d\n", 
      myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());

    Serial.printf("Date: %04d-%02d-%02d\n", 
      myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());

    // Fix info
    Serial.print("Fix Type: ");
    Serial.println(myGNSS.getFixType()); // 0 = none, 3 = 3D fix
    Serial.print("Satellites: ");
    Serial.println(myGNSS.getSIV());

    Serial.println("----------------------------");
  }

  delay(1000);
}
