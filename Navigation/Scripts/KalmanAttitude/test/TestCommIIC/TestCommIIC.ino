#include <Wire.h>

#define MPU6050_ADDRESS     0x68
#define PWR_MGMT_1         0x6B
#define INT_PIN_CFG        0x37
#define USER_CTRL          0x6A
#define WHO_AM_I          0x75

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while(!Serial);
  
  Serial.println("Starting full MPU6050 initialization...");
  
  // Reset MPU6050
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x80);  // Set DEVICE_RESET bit
  Wire.endTransmission(true);
  
  delay(100);  // Wait for reset
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);  // Clear SLEEP bit
  Wire.endTransmission(true);
  
  delay(100);
  
  // Verify MPU6050 is responsive
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 1);
  if(Wire.available()) {
    byte whoAmI = Wire.read();
    Serial.print("WHO_AM_I register: 0x");
    Serial.println(whoAmI, HEX);
    if(whoAmI != 0x68) {
      Serial.println("Unexpected WHO_AM_I value!");
    }
  }
  
  // Disable I2C master mode
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(USER_CTRL);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  delay(100);
  
  // Enable bypass mode
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02);
  Wire.endTransmission(true);
  
  delay(100);
  
  // Now try to detect magnetometer at both possible addresses
  Wire.beginTransmission(0x1E);  // HMC5883L address
  byte error1 = Wire.endTransmission();
  
  Wire.beginTransmission(0x0D);  // QMC5883L address
  byte error2 = Wire.endTransmission();
  
  Serial.println("\nTesting magnetometer addresses directly:");
  Serial.print("HMC5883L (0x1E) response: ");
  Serial.println(error1 == 0 ? "Found!" : "Not found");
  Serial.print("QMC5883L (0x0D) response: ");
  Serial.println(error2 == 0 ? "Found!" : "Not found");
  
  // Full I2C scan
  Serial.println("\nFull I2C scan:");
  scanI2C();
}

void loop() {
  delay(5000);
  scanI2C();
}

void scanI2C() {
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  }
}
