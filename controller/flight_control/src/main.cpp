#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include <ESP32Servo.h>

// IMU constants
#define MEASUREMENT_INTERVAL 50
#define ARRAY_SIZE 1800

float rolldata[ARRAY_SIZE];
float pitchdata[ARRAY_SIZE];
float yawdata[ARRAY_SIZE];
float x_accel[ARRAY_SIZE]; //forward back
float y_accel[ARRAY_SIZE]; //left right
float z_accel[ARRAY_SIZE]; //up down

int counter = 0; //counter; when counter reaches falldata.length, stop adding values
unsigned long last_measurement = 0;
int timer = 0;

// Servo constants
#define NUM_SERVOS 2
const int servo_pin[] = {26, 25};
Servo servo[2];
const bool servo_clockwise[] = {true, false};
#define PULSE_MIN 700
#define PULSE_MAX 2500
#define SERVO_PWM_FREQ 50
#define RANGE 180

// Motor constants
#define WINDING_MOTOR_1 13
#define WINDING_MOTOR_2 12
#define WINDING_MOTOR_ENABLE 27
ESP32PWM enable;
#define MOTOR_PWM_FREQ 30000

// General PWM constants
#define PWM_RES 8
#define DUTY_CYCLE_MAX 255

// Controller constants
#define Kp 0.1
#define Ki 0.05

LSM6DSO IMU;
bool IMU_setup_success;
#define IMU_ZERO_VALUE 0.22710

unsigned long flight_start = 0;
#define FLIGHT_LENGTH 5500
#define CONTROL_TIME 4000
#define DROP_TIME 3000
#define START_TIME 8000
unsigned long prev_t = 0;
float theta_dot = 0;
float theta = 0;

bool setupIMU() {
  Serial.println("Setting up IMU.");
  bool success = IMU.begin();
  if (!IMU.initialize(BASIC_SETTINGS)) {
    Serial.println("Failed to load IMU settings");
  }
  return success;
}

void setupServos() {
  Serial.println("Setting up servos.");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servo[i].setPeriodHertz(SERVO_PWM_FREQ);
    servo[i].attach(servo_pin[i], PULSE_MIN, PULSE_MAX);
  }
}

void turnServo(int id, float desired_angle) {
  float true_angle = servo_clockwise[id] ? desired_angle : RANGE - desired_angle;
  servo[id].write(true_angle);
}

void setupWindingMotor() {
  Serial.println("Setting up winding motor.");
  pinMode(WINDING_MOTOR_1, OUTPUT);
  pinMode(WINDING_MOTOR_2, OUTPUT);
  digitalWrite(WINDING_MOTOR_1, LOW);
  digitalWrite(WINDING_MOTOR_2, LOW);
  
  // Set up PWM channel
  enable.attachPin(WINDING_MOTOR_ENABLE, MOTOR_PWM_FREQ, PWM_RES);
}

void setup() {
  Serial.begin(115200);
  delay(500); 
  
  Wire.begin(23, 22);
  delay(10);

  // Set up IMU
  IMU_setup_success = setupIMU();

  // Set up servos
  setupServos();

  turnServo(0, 90);
  turnServo(1, 110);

  flight_start = millis();
}

void loop() {
  if (IMU_setup_success) {
    unsigned long curr_t = micros();
    theta_dot = IMU.readFloatGyroZ() - IMU_ZERO_VALUE;
    theta += theta_dot * (curr_t - prev_t)/1e6;
    prev_t = curr_t;



    if (millis() - last_measurement > MEASUREMENT_INTERVAL && counter < ARRAY_SIZE) {
      //note: these values are all acceleration and not absolute 
      rolldata[counter] = IMU.readFloatGyroX();
      pitchdata[counter] = IMU.readFloatGyroY();
      yawdata[counter] = IMU.readFloatGyroZ();
      //accel data
      x_accel[counter] = IMU.readFloatAccelX();
      y_accel[counter] = IMU.readFloatAccelY();
      z_accel[counter] = IMU.readFloatAccelZ();
      counter += 1;
      last_measurement = millis();
    }
  }

  Serial.print("theta:");
  Serial.print(theta);
  Serial.print(",theta_dot:");
  Serial.print(theta_dot);

  unsigned long curr_t = millis();

  if (curr_t - flight_start < START_TIME) {

  }
  else if (curr_t - flight_start < START_TIME + CONTROL_TIME) {
    turnServo(0, 0);
    turnServo(1, 20);
  }
  else if (curr_t - flight_start < START_TIME + FLIGHT_LENGTH) {
    float u = Kp * theta_dot + Ki * theta;
    Serial.print(",u:");
    Serial.println(u);
    if (u > 0) {
      turnServo(0, u);
      turnServo(1, 0);
    } else {
      turnServo(0, 0);
      turnServo(1, -u + 20);
    }
  } else {
    turnServo(0, 150);
    turnServo(1, 170);
  }
}
