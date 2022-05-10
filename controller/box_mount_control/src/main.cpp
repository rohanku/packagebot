#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

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

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

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

#define IMU_CALIBRATION_CYCLES 2000
LSM6DSO IMU;
bool IMU_setup_success;
float gyro_z_zero = 0;
float gyro_x_zero = 0;

#define BRAKE_DELAY 2000
unsigned long flight_start = 0;
unsigned long prev_t = 0;
float theta = 0; // turning angle
float theta_dot = 0;

#define LOADING_ANGLE -20
#define TOLERANCE 8
#define WINDING_START_DELAY 2000
#define WINDING_DELAY 4000
float phi = 0; // elevation angle
float phi_dot = 0;
unsigned long delay_start = LONG_MAX;


#define JOLT_THRESHOLD 800
#define JOLT_TO_FLIGHT_DELAY 2000

/*
STATUS CODES:
0 - Waiting to load
1 - Winding
2 - Launching
3 - In flight
4 - Landing
*/
int stage = 0;

bool setupIMU() {
  Serial.println("Setting up IMU.");
  bool success = IMU.begin();
  if (!IMU.initialize(BASIC_SETTINGS)) {
    Serial.println("Failed to load IMU settings");
  }

  if (success) {
    float gyro_z_sum = 0;
    float gyro_x_sum = 0;
    for (int i = 0; i < IMU_CALIBRATION_CYCLES; i++) {
        gyro_z_sum += IMU.readFloatGyroZ();
        gyro_x_sum += IMU.readFloatGyroX();
    }
    gyro_z_zero = gyro_z_sum / IMU_CALIBRATION_CYCLES;
    gyro_x_zero = gyro_x_sum / IMU_CALIBRATION_CYCLES;
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

void setWindingMotor(int dir, int duty_cycle) {
  // 0 - Forward, 1 - Backward, 2 - Stop
  switch (dir) {
    case 0:
      digitalWrite(WINDING_MOTOR_1, HIGH);
      digitalWrite(WINDING_MOTOR_2, LOW);
      enable.write(duty_cycle);
      break;
    case 1:
      digitalWrite(WINDING_MOTOR_1, LOW);
      digitalWrite(WINDING_MOTOR_2, HIGH);
      enable.write(duty_cycle);
      break;
    case 2:
      digitalWrite(WINDING_MOTOR_1, LOW);
      digitalWrite(WINDING_MOTOR_2, LOW);
      break;
  }
}

void printall(float input[]) {
  for (int i = 0; i < counter; i++) {
    SerialBT.println(input[i], 3);
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");
  delay(500); 
  
  Wire.begin(23, 22);
  delay(10);

  // Set up IMU
  IMU_setup_success = setupIMU();

  setupWindingMotor();
  // Set up servos
  setupServos();

  turnServo(0, 0);
  turnServo(1, 0);
}

void loop() {
  if (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n')  {
      delay(100);
      SerialBT.println(counter);
      SerialBT.println("Roll");
      printall(rolldata);
      SerialBT.println("Pitch");
      printall(pitchdata);
      SerialBT.println("Yaw");
      printall(yawdata);
      SerialBT.println("XAccel");
      printall(x_accel);
      SerialBT.println("YAccel");
      printall(y_accel);
      SerialBT.println("ZAccel");
      printall(z_accel);
      SerialBT.println("restarting...");
    }
  }

  if (IMU_setup_success) {
    unsigned long curr_t = micros();
    if (stage == 3) {
      theta_dot = IMU.readFloatGyroZ() - gyro_z_zero;
      theta += theta_dot * (curr_t - prev_t)/1e6;
    }
    phi_dot = IMU.readFloatGyroX() - gyro_x_zero;
    phi += phi_dot * (curr_t - prev_t)/1e6;

    if (stage == 0) {
      if (phi > LOADING_ANGLE - TOLERANCE && phi < LOADING_ANGLE + TOLERANCE) {
        if (delay_start == LONG_MAX)
          delay_start = millis();
        if (millis() - delay_start > WINDING_START_DELAY) {
          Serial.println("Entering winding stage");
          stage = 1;
          setWindingMotor(0, 220);
          delay(WINDING_DELAY);
          setWindingMotor(2, 0);
          Serial.println("Entering launching stage");
          stage =  2;
        }
      } else {
        delay_start = LONG_MAX;
      }
    }

    if (stage == 2 && IMU.readFloatAccelZ() > JOLT_THRESHOLD) {
      delay(JOLT_TO_FLIGHT_DELAY);
      Serial.println("Entering flying stage");
      stage = 3;
      flight_start = millis();
    }

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
      counter++;
      last_measurement = millis();
    }
  }

  if (stage == 0) {
    Serial.print(",phi:");
    Serial.print(phi);
    Serial.print(",phi_dot:");
    Serial.println(phi_dot);
  }

  if (stage == 3) {
    Serial.print("theta:");
    Serial.print(theta);
    Serial.print(",theta_dot:");
    Serial.print(theta_dot);
    float u = Kp * theta_dot + Ki * theta;
    Serial.print(",u:");
    Serial.println(u);
    if (millis() - flight_start < BRAKE_DELAY) {
      if (u > 0) {
        turnServo(0, u);
        turnServo(1, 0);
      } else {
        turnServo(0, 0);
        turnServo(1, -u);
      }
    } else {
      Serial.println("Entering braking mode");
      stage = 4;
      turnServo(0, 180);
      turnServo(1, 180);
    }
  }
}
