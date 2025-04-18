#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
// include wire.h for I2C communication
#include <Wire.h>
// include IMU library
#include <MPU6050.h>
MPU6050 imu;

#define IMU_TEST
//#define GPS_TEST
// Uncomment the above line to enable GPS test

// UART pins for GY87
#define GPS_RX_PIN 16  // ESP32 RX <- GPS TX
#define GPS_TX_PIN 17  // ESP32 TX -> GPS RX
#define GPS_BAUD 115200 // GPS baud rate
HardwareSerial gpsSerial(2); // UART2  
SFE_UBLOX_GNSS myGNSS;

// IMU6050 pins
#define IMU_SDA 21 // ESP32 SDA pin
#define IMU_SCL 22 // ESP32 SCL pin

void setup() {

  Serial.begin(115200); // USB serial for debug
  delay(1000);
  #ifdef IMU_TEST
  Serial.begin(115200);
  Wire.begin();

  imu.init(2, 250);  // 2g accel, 250 deg/s gyro
  delay(100);
  #endif

  #ifdef GPS_TEST
  Serial.println("GY87 Sensor Test");
  
  Wire.begin();  // Initialize I2C

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

    Serial.println("Initializing GPS module...");
    // Begin UART communication with GPS
  
    if (myGNSS.begin(gpsSerial)) {
      Serial.println("GNSS module connected!");
    } else {
      Serial.println("Failed to connect to GNSS module.");
    }
  
    // Optional tuning
    myGNSS.setNavigationFrequency(1); // 1 Hz updates
    myGNSS.setAutoPVT(true); // Enable automatic PVT messages

    #endif
  
    // ---------- End GPS Module Setup ----------
  }

void loop() {

  // ---------------- GY87 Sensor Test ----------------
  #ifdef IMU_TEST

  float ax, ay, az;
  float gx, gy, gz;
  imu.readAccel(ax, ay, az); // Read accelerometer data
  imu.readGyro(gx, gy, gz); // Read gyroscope data
  Serial.print("Accelerometer: \n");
  Serial.print("ax: "); Serial.print(ax);Serial.print("\n");
  Serial.print(" ay: "); Serial.print(ay); Serial.print("\n");
  Serial.print(" az: "); Serial.println(az); Serial.print("\n");
  Serial.println("----------------------------");

  Serial.print("Gyroscope: ");
  Serial.print("gX: "); Serial.print(gx);Serial.print("\n");
  Serial.print(" gY: "); Serial.print(gy); Serial.print("\n");
  Serial.print(" gZ: "); Serial.println(gz); Serial.print("\n");
  delay(1000); // Delay for readability

  #endif

  #ifdef GPS_TEST
  
    
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
    

  delay(1000);
  #endif
};
