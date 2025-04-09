/*
 * Inputs: PWM receiver (3 channels)
 * Outputs: 3 servos (left aileron, right aileron, elevator)
 */

 #include <Servo.h>
 #include <EnableInterrupt.h>
 
 // Pin definitions
 #define LEFT_AILERON_PIN 5
 #define RIGHT_AILERON_PIN 6
 #define ELEVATOR_PIN 9
 #define ROLL_INPUT_PIN 10    // PWM input for roll
 #define PITCH_INPUT_PIN 3    // Changed to pin 3 for interrupt
 #define THROTTLE_INPUT_PIN 12 // PWM input for throttle
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
 
 
 // Create servo objects
 Servo leftAileron;
 Servo rightAileron;
 Servo elevator;
 int rightServo = 0;
 int lefServo = 0;
 int elevatorServo = 0;
 
 // PWM input variables
 unsigned int pwm_roll_value = 1500;    // Default center position
 unsigned int pwm_pitch_value = 1500;   // Default center position
 unsigned int pwm_throttle_value = 1000; // Default low throttle
 unsigned int pwm_flap_value = 1000;    // Default no flaps
 unsigned long previousPulseRead = 0;    // Timer for pulse readings
 const unsigned long PULSE_READ_INTERVAL = 20; // Read pulses every 20ms
 int switchAValue;   // swith A value for logger activation 
 
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
 
         
         previousPulseRead = millis();
     }
 }
 
 
 void setup() {
     Serial.begin(9600);
     Serial.setTimeout(50);  // Lower timeout for faster recovery from errors
     
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
     
     //Serial.println("System initialized");
 }
 
 void updateServos(int leftAileronAngle, int rightAileronAngle, int elevatorAngle) {
 
     leftAileron.write(leftAileronAngle);
     rightAileron.write(rightAileronAngle);
     elevator.write(elevatorAngle);
 }
 
 void loop() {
     static uint32_t loopTimer = micros();
     
     // Read PWM inputs
     readPWMInputs();
     float desiredRoll = map(pwm_roll_value, 1000, 2000, -45, 45);
     float desiredPitch = map(pwm_pitch_value, 1000, 2000, 45, -45);
     float desiredThrottle = map(pwm_throttle_value, 1000, 2000, 0, 180);
     // Calculate flap deflection
     float flapDeflection = 0;
     if (pwm_flap_value > FLAP_THRESHOLD) {
         flapDeflection = map(pwm_flap_value, FLAP_THRESHOLD, 2000, 20, MAX_FLAP_DEFLECTION);
     }
     
     // Receive servo commands
     if (Serial.available()) {
         String received = Serial.readStringUntil('\n');
         if (received == "<rqt>") {
             // Send orientation data
             String message = "<t," + String(desiredRoll) + "," + String(desiredPitch) + "," + String(flapDeflection) + ">\n";
             Serial.print(message);
         }
         if (received.startsWith("<m,") && received.endsWith(">")) {
             // Parse the received commands
             rightServo = received.substring(3, 6).toInt();  
             lefServo = received.substring(7, 10).toInt();
             elevatorServo = received.substring(11, 14).toInt();
 
             updateServos(rightServo, lefServo, elevatorServo);
         }
 
         
     }
     
     
     // Maintain loop timing
     while(micros() - loopTimer < 4000);
     loopTimer = micros();
     
 }