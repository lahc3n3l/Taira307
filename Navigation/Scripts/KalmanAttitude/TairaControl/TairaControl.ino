/*
 * Airplane Stabilization System with Enhanced Kalman Filter
 * Hardware: Arduino Uno
 * Sensors: MPU6050
 * Inputs: PWM receiver (3 channels)
 * Outputs: 3 servos (left aileron, right aileron, elevator)
 */

#include <Wire.h>
#include <Servo.h>
#include <BasicLinearAlgebra.h>
#include <EnableInterrupt.h>

using namespace BLA;

// Pin definitions
#define LEFT_AILERON_PIN 5
#define RIGHT_AILERON_PIN 6
#define ELEVATOR_PIN 9
#define ROLL_INPUT_PIN 10    // PWM input for roll
#define PITCH_INPUT_PIN 3    // Changed to pin 3 for interrupt
#define THROTTLE_INPUT_PIN 12 // PWM input for throttle
#define LOGGER_TRIGGER_PIN 11    // PWM input for logger trigger

#define FLAP_INPUT_PIN 2     // New interrupt for flaps (CH6)

// Flap settings
#define MAX_FLAP_DEFLECTION 40  // Maximum flap deflection in degrees
#define FLAP_THRESHOLD 1000     // PWM value above which flaps deploy

// Volatile variables for interrupt-based readings
volatile unsigned long pitch_start = 0;
volatile unsigned long pitch_value = 1500;
volatile bool pitch_reading = false;

volatile unsigned long flap_start = 0;
volatile unsigned long flap_value = 1000;  // Default no flaps
volatile bool flap_reading = false;

// MPU6050 settings
#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_PWR_MGMT_1 0x6B

// Calibration values
struct CalibrationData {
    float gyro_bias[3];
    float accel_bias[3];
    float accel_scale[3];
} calibration;


// Create servo objects
Servo leftAileron;
Servo rightAileron;
Servo elevator;

// PWM input variables
unsigned int pwm_roll_value = 1500;    // Default center position
unsigned int pwm_pitch_value = 1500;   // Default center position
unsigned int pwm_throttle_value = 1000; // Default low throttle
unsigned int pwm_flap_value = 1000;    // Default no flaps
unsigned long previousPulseRead = 0;    // Timer for pulse readings
const unsigned long PULSE_READ_INTERVAL = 20; // Read pulses every 20ms
int switchAValue;   // swith A value for logger activation 


// Kalman filter matrices using BasicLinearAlgebra
Matrix<3, 3> A = {1, 0, 0,
                  0, 1, 0,
                  0, 0, 1};
                  
Matrix<2, 3> H = {1, 0, 0,
                  0, 1, 0};
                  
Matrix<3, 3> Q = {0.1, 0, 0,
                  0, 0.1, 0,
                  0, 0, 0.1};
                  
Matrix<2, 2> R = {0.4, 0,
                  0, 0.4};
                  
Matrix<3, 3> P = {100, 0, 0,
                  0, 100, 0,
                  0, 0, 100};
                  
Matrix<3, 2> K;
Matrix<3, 1> x = {0, 0, 0}; // State vector as column matrix

const float dt = 0.004; // Loop time in seconds

// PID variables
float prevRollError = 0;
float prevPitchError = 0;
float rollIntegral = 0;
float pitchIntegral = 0;

// PID gains
const float Kp_roll = 1.5;
const float Ki_roll = 0.0;
const float Kd_roll = 0.0;
const float Kp_pitch = 0.75;
const float Ki_pitch = 0.0;
const float Kd_pitch = 0.0;

// Interrupt Service Routine for pitch input
void pitchInterrupt() {
    unsigned long curr_time = micros();
    
    if (digitalRead(PITCH_INPUT_PIN) == HIGH) {
        // Rising edge
        pitch_start = curr_time;
        pitch_reading = true;
    } else if (pitch_reading) {
        // Falling edge
        unsigned long pulse_width = curr_time - pitch_start;
        
        // Validate pulse width (typical RC values are between 1000-2000µs)
        if (pulse_width >= 900 && pulse_width <= 2100) {
            pitch_value = pulse_width;
        }
        pitch_reading = false;
    }
}

// Interrupt Service Routine for flap input
void flapInterrupt() {
    unsigned long curr_time = micros();
    
    if (digitalRead(FLAP_INPUT_PIN) == HIGH) {
        flap_start = curr_time;
        flap_reading = true;
    } else if (flap_reading) {
        unsigned long pulse_width = curr_time - flap_start;
        if (pulse_width >= 900 && pulse_width <= 2100) {
            flap_value = pulse_width;
        }
        flap_reading = false;
    }
}

// Modified readPWMInputs function
void readPWMInputs() {
    if (millis() - previousPulseRead >= PULSE_READ_INTERVAL) {
        unsigned long timeout = 25000;
        
        // Read roll input
        unsigned long pulse = pulseIn(ROLL_INPUT_PIN, HIGH, timeout);
        if (pulse != 0) {
            pwm_roll_value = pulse;
        }
        
        // Update pitch value from interrupt-based reading
        noInterrupts();
        pwm_pitch_value = pitch_value;
        pwm_flap_value = flap_value;    // Read flap value safely
        interrupts();
        
        // Read throttle input
        pulse = pulseIn(THROTTLE_INPUT_PIN, HIGH, timeout);
        if (pulse != 0) {
            pwm_throttle_value = pulse;
        }
        
        previousPulseRead = millis();

        // Read Switch A input
        pulse = pulseIn(LOGGER_TRIGGER_PIN, HIGH, timeout);
        if (pulse != 0) {
            switchAValue = pulse;
        }
        
        previousPulseRead = millis();
    }
}

void setup() {
    Serial.begin(9600);
    //    
    // Initialize servos
    leftAileron.attach(LEFT_AILERON_PIN);
    rightAileron.attach(RIGHT_AILERON_PIN);
    elevator.attach(ELEVATOR_PIN);
    pinMode(FLAP_INPUT_PIN, INPUT);
    
    // Setup interrupts
    enableInterrupt(PITCH_INPUT_PIN, pitchInterrupt, CHANGE);
    enableInterrupt(FLAP_INPUT_PIN, flapInterrupt, CHANGE);

    
    // Set servos to center position
    leftAileron.write(90);
    rightAileron.write(90);
    elevator.write(90);
    
    // Initialize PWM inputs
    pinMode(ROLL_INPUT_PIN, INPUT);
    pinMode(PITCH_INPUT_PIN, INPUT);
    pinMode(THROTTLE_INPUT_PIN, INPUT);
    
    // Setup interrupt for pitch input
    enableInterrupt(PITCH_INPUT_PIN, pitchInterrupt, CHANGE);
    
    // Initialize I2C and MPU6050
    Wire.begin();
    Wire.setClock(400000);
    setupMPU6050();
    
    // Wait for throttle down
    while(pwm_throttle_value > 1050) {
        delay(10);
    }
    
    //Serial.println("System initialized");
}

void setupMPU6050() {
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    
    // Configure gyro (500deg/s)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_GYRO_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();
    
    // Configure accelerometer (4g)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();
    
    delay(100);
}

void readMPU6050(float gyro[3], float accel[3]) {
    // Read raw values
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
    
    int16_t accX = Wire.read() << 8 | Wire.read();
    int16_t accY = Wire.read() << 8 | Wire.read();
    int16_t accZ = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    int16_t gyrX = Wire.read() << 8 | Wire.read();
    int16_t gyrY = Wire.read() << 8 | Wire.read();
    int16_t gyrZ = Wire.read() << 8 | Wire.read();
    
    // Convert to float values and apply calibration
    accel[0] = accX / 8192.0+0.015 ;//- calibration.accel_bias[0]); //* calibration.accel_scale[0];
    accel[1] = accY / 8192.0 ;//- calibration.accel_bias[1]); //* calibration.accel_scale[1];
    accel[2] = accZ / 8192.0 - 0.05;//- calibration.accel_bias[2]); //* calibration.accel_scale[2];
    
    gyro[0] = gyrX / 65.5 +0.13+1.6; //- calibration.gyro_bias[0];
    gyro[1] = gyrY / 65.5 -0.03-0.5; //- calibration.gyro_bias[1];
    gyro[2] = gyrZ / 65.5 -0.02-0.35; //- calibration.gyro_bias[2];

    gyro[0] *= DEG_TO_RAD ;
    gyro[1] *= DEG_TO_RAD ;
    gyro[2] *= DEG_TO_RAD ;

    // Debug output
   // Serial.print("accelX:"); Serial.println(accel[0]);
    //Serial.print("accelX:"); Serial.print(accel[0]); Serial.print(", accelY:"); Serial.print(accel[1]); Serial.print(", accelZ:"); Serial.println(accel[2]);
    //Serial.print("gyrX:"); Serial.print(gyro[0]); Serial.print(", gyroY:"); Serial.print(gyro[1]); Serial.print(", gyroZ:"); Serial.println(gyro[2]);
    //Serial.print("Pitch:"); Serial.print(x[1]);
    // Serial.print(" ServosL/R/E: ");
    
}

void kalmanPredict(float gyro[3]) {
    // Create gyro measurement matrix
  
    Matrix<3, 1> gyro_m = {gyro[0], gyro[1], gyro[2]};
    
    // Predict state
    x = x + gyro_m * dt;
    
    // Update error covariance: P = APA' + Q
    P = A * P * ~A + Q;
}

void kalmanUpdate(float accel[3]) {
    // Calculate measurement
    Matrix<2, 1> y = {
        atan2(accel[1], sqrt(accel[0]*accel[0] + accel[2]*accel[2])),
        atan2(-accel[0], sqrt(accel[1]*accel[1] + accel[2]*accel[2]))
    };

    
    // Innovation covariance: S = HPH' + R
    Matrix<2, 2> S = H * P * ~H + R;
    
    // Kalman gain: K = PH'S^(-1)
    K = P * ~H * Inverse(S);
    
    
    // Update state: x = x + K(y - Hx)
    Matrix<2, 1> innovation = y - H * x;
    
    x = x + K * innovation;
    
    // Update error covariance: P = (I - KH)P
    Matrix<3, 3> I = {1, 0, 0,
                      0, 1, 0,
                      0, 0, 1};
    P = (I - K * H) * P;

}

void updateServos() {
    // Get desired angles from PWM (scale to ±45 degrees)
    float desiredRoll = map(pwm_roll_value, 1000, 2000, -45, 45);
    float desiredPitch = map(pwm_pitch_value, 1000, 2000, 45, -45);
    float desiredThrottle = map(pwm_throttle_value, 1000, 2000, 0, 180);

    // Calculate flap deflection
    float flapDeflection = 0;
    if (pwm_flap_value > FLAP_THRESHOLD) {
        flapDeflection = map(pwm_flap_value, FLAP_THRESHOLD, 2000, 20, MAX_FLAP_DEFLECTION);
    }
    
    float Roll = x(0) * RAD_TO_DEG;
    float Pitch = x(1) * RAD_TO_DEG;
    float Yaw = x(2) * RAD_TO_DEG;  

   // Serial.print("Roll:"); Serial.println(Roll);
   //Serial.print("Pitch:"); Serial.println(Pitch);
   //Serial.print("Yaw:"); Serial.println(Yaw);
    // Calculate errors
    float rollError = desiredRoll - Roll;
    float pitchError = desiredPitch - Pitch;
    // Calculate PID terms for roll
    float rollP = Kp_roll * rollError;
    rollIntegral += Ki_roll * rollError * dt;
    rollIntegral = constrain(rollIntegral, -20, 20); // Anti-windup
    float rollD = Kd_roll * (rollError - prevRollError) / dt;
    prevRollError = rollError;
    
    // Calculate PID terms for pitch
    float pitchP = Kp_pitch * pitchError;
    pitchIntegral += Ki_pitch * pitchError * dt;
    pitchIntegral = constrain(pitchIntegral, -20, 20); // Anti-windup
    float pitchD = Kd_pitch * (pitchError - prevPitchError) / dt;
    prevPitchError = pitchError;
    
    // Calculate final outputs
    float rollOutput = rollP + rollIntegral + rollD;
    float pitchOutput = pitchP + pitchIntegral + pitchD;
    
    // Convert to servo angles (90 is center position)
    // Add flap deflection to both ailerons
    int leftAileronAngle = 90 + rollOutput - flapDeflection;
    int rightAileronAngle = 90 + rollOutput + flapDeflection;  // Note the negative rollOutput
    int elevatorAngle = 90 - pitchOutput;
    
    // Constrain servo angles
    leftAileronAngle = constrain(leftAileronAngle, 10, 170);
    rightAileronAngle = constrain(rightAileronAngle, 10, 170);
    elevatorAngle = constrain(elevatorAngle, 45, 135);
    
    // Write to servos
    //  Serial.print("leftAileronAngle:"); Serial.println(leftAileronAngle);
    //  Serial.print("rightAileronAngle:"); Serial.println(rightAileronAngle);
    //  Serial.print("elevatorAngle:"); Serial.println(elevatorAngle);

    leftAileron.write(leftAileronAngle);
    rightAileron.write(rightAileronAngle);
    elevator.write(elevatorAngle);
}

void loop() {
    static uint32_t loopTimer = micros();
    
    // Read PWM inputs
    readPWMInputs();

    
    // Read sensors
    float gyro[3], accel[3];
    readMPU6050(gyro, accel);
    
    // Kalman filter steps
    kalmanPredict(gyro);
    kalmanUpdate(accel);
    
    // Update servo outputs
    updateServos();
    
    Serial.print("Switch A:");
    Serial.print(switchAValue);
    
    // Maintain loop timing
    while(micros() - loopTimer < 4000);
    loopTimer = micros();
    
}
