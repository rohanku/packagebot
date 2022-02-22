#include "SparkFunLSM6DSO.h"
#include "Wire.h"

// Servo constants
#define SERVO_1 26
#define SERVO_2 25
#define PULSE_MIN 500
#define PULSE_MAX 2500
#define RANGE 270

// Motor constants
#define WINDING_MOTOR_1 13
#define WINDING_MOTOR_2 12
#define WINDING_MOTOR_ENABLE 27
#define WINDING_MOTOR_PWM 0
#define PWM_FREQ 30000
#define PWM_RES 8


LSM6DSO myIMU;

void turnServo(int pin, double angle) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(PULSE_MIN + (PULSE_MAX - PULSE_MIN) * angle/RANGE);
  digitalWrite(pin, LOW);
}

void turnServos(double angle) {
  digitalWrite(SERVO_1, HIGH);
  digitalWrite(SERVO_2, HIGH);
  unsigned long pulse_1 = PULSE_MIN + (PULSE_MAX - PULSE_MIN) * angle/RANGE;
  unsigned long pulse_2 = PULSE_MIN + (PULSE_MAX - PULSE_MIN) * (270 - angle)/RANGE;
  if (pulse_1 < pulse_2) {
    delayMicroseconds(pulse_1);
    digitalWrite(SERVO_1, LOW);
    delayMicroseconds(pulse_2-pulse_1);
    digitalWrite(SERVO_2, LOW);
  } else {
    delayMicroseconds(pulse_2);
    digitalWrite(SERVO_2, LOW);
    delayMicroseconds(pulse_1-pulse_2);
    digitalWrite(SERVO_1, LOW);
  }
}


void imuTest() {
  Serial.println("Testing IMU...");
  
  if( myIMU.begin() ) {
    Serial.println("IMU Ready.");

    if( myIMU.initialize(BASIC_SETTINGS) )
      Serial.println("Loaded Settings.");

    Serial.println("Taking 3 measurements. Ensure that values are nonzero and changing.");
    for (int i = 0; i < 3; i++) {
      //Get all parameters
      Serial.print("\nAccelerometer:\n");
      Serial.print(" X = ");
      Serial.println(myIMU.readFloatAccelX(), 3);
      Serial.print(" Y = ");
      Serial.println(myIMU.readFloatAccelY(), 3);
      Serial.print(" Z = ");
      Serial.println(myIMU.readFloatAccelZ(), 3);
    
      Serial.print("\nGyroscope:\n");
      Serial.print(" X = ");
      Serial.println(myIMU.readFloatGyroX(), 3);
      Serial.print(" Y = ");
      Serial.println(myIMU.readFloatGyroY(), 3);
      Serial.print(" Z = ");
      Serial.println(myIMU.readFloatGyroZ(), 3);
    
      Serial.print("\nThermometer:\n");
      Serial.print(" Degrees F = ");
      Serial.println(myIMU.readTempF(), 3);
    
      delay(500);
    }
  }
  else
    Serial.println("Could not connect to IMU.");
}

void motorTest() {
  Serial.println("Testing winding motor...");

  // Set up motor pins
  pinMode(WINDING_MOTOR_1, OUTPUT);
  pinMode(WINDING_MOTOR_2, OUTPUT);
  pinMode(WINDING_MOTOR_ENABLE, OUTPUT);
  
  // Set up PWM channel
  ledcSetup(WINDING_MOTOR_PWM, PWM_FREQ, PWM_RES);
  
  // Attach PWM channel to GPIO
  ledcAttachPin(WINDING_MOTOR_ENABLE, WINDING_MOTOR_PWM);

  
  // Move motor forward with increasing speed
  digitalWrite(WINDING_MOTOR_1, HIGH);
  digitalWrite(WINDING_MOTOR_2, LOW);
  for (int dutyCycle = 160; dutyCycle <= 255; dutyCycle += 5){
    ledcWrite(WINDING_MOTOR_PWM, dutyCycle);   
    Serial.print("Turning forwards with duty cycle: ");
    Serial.print(dutyCycle);
    Serial.println(".");
    delay(250);
  }
  
  // Move motor forward with decreasing speed
  for (int dutyCycle = 255; dutyCycle >= 160; dutyCycle -= 5){
    ledcWrite(WINDING_MOTOR_PWM, dutyCycle);   
    Serial.print("Turning forwards with duty cycle: ");
    Serial.print(dutyCycle);
    Serial.println(".");
    delay(250);
  }

  // Move motor backwards with increasing speed
  digitalWrite(WINDING_MOTOR_1, LOW);
  digitalWrite(WINDING_MOTOR_2, HIGH);
  for (int dutyCycle = 160; dutyCycle <= 255; dutyCycle += 5){
    ledcWrite(WINDING_MOTOR_PWM, dutyCycle);   
    Serial.print("Turning backwards with duty cycle: ");
    Serial.print(dutyCycle);
    Serial.println(".");
    delay(250);
  }


  // Move motor backwards with decreasing speed
  for (int dutyCycle = 255; dutyCycle >= 160; dutyCycle -= 5){
    ledcWrite(WINDING_MOTOR_PWM, dutyCycle);   
    Serial.print("Turning backwards with duty cycle: ");
    Serial.print(dutyCycle);
    Serial.println(".");
    delay(250);
  }
  // Stop motor
  digitalWrite(WINDING_MOTOR_1, LOW);
  digitalWrite(WINDING_MOTOR_2, LOW);
  Serial.println("Motor stopped.");
}

void servoTest() {
  // Set up servo pins
  pinMode(SERVO_1, OUTPUT);
  pinMode(SERVO_2, OUTPUT);
  digitalWrite(SERVO_1, LOW);
  digitalWrite(SERVO_2, LOW);
  
  delay(500);
  
  Serial.println("Turning servo 1 180 degrees.");
  turnServo(SERVO_1, 0);
  delay(1000);
  turnServo(SERVO_1, 180);
  delay(1000);
  turnServo(SERVO_1, 0);
  
  Serial.println("Turning servo 2 180 degrees.");
  turnServo(SERVO_2, 270);
  delay(1000);
  turnServo(SERVO_2, 90);
  delay(1000);
  turnServo(SERVO_2, 270);

  delay(1000);

  Serial.println("Turning servos both servos 180 degrees.");
  turnServos(0);
  delay(1000);
  turnServos(180);
  delay(1000);
  turnServos(0);

  Serial.println("Tests complete!");
}

void setup() {
  Serial.begin(115200);
  delay(500); 

  // Setup I2C devices
  Wire.begin(23, 22);
  delay(10);

  imuTest();
  
  motorTest();
  
  servoTest();
  
}

void loop() {
  // Nothing to run in a loop
}
