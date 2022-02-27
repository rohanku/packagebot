#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "BluetoothSerial.h"
#include "ArduinoJson.h"

// IMU constants
#define MEASUREMENT_INTERVAL 2000

// Servo constants
#define NUM_SERVOS 2
const int servo_pin[] = {26, 25};
const bool servo_clockwise[] = {true, false};
#define PULSE_MIN 500
#define PULSE_MAX 2500
#define PULSE_PERIOD 20000
#define RANGE 270

// Motor constants
#define WINDING_MOTOR_1 13
#define WINDING_MOTOR_2 12
#define WINDING_MOTOR_ENABLE 27
#define WINDING_MOTOR_PWM 0
#define PWM_FREQ 30000
#define PWM_RES 8
#define DUTY_CYCLE_MAX 255

// Serialization constants
#define DELIMITER '\n'

LSM6DSO IMU;
bool IMU_setup_success;
unsigned long IMU_last_measurement = 0;


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

unsigned long servo_pulse_start[] = {0, 0};
unsigned long servo_pulse_length[] = {0, 0};

String command = "";
unsigned long command_index = 0;

/* OPCODES
 *  0 - Turn servo
 *  1 - Set winding motor
 */

/* TELEMETRY TYPES
 *  0 - IMU data
 */

void processReceivedValue(char b){
  if(b == DELIMITER){
    Serial.println(command);
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, command);
    int opcode = doc["command"];
    JsonArray arguments = doc["args"];
    if (arguments != NULL) {
      switch (opcode) {
        case 0:
          turnServo(arguments);
          break;
        case 1:
          setWindingMotor(arguments);
          break;
      }
    } else {
      Serial.println("Opcode or arguments not passed in");
    }
    command = "";
  }
  else {
    command.concat(b);
  }
 
  return;
}

bool setupIMU() {
  bool success = IMU.begin();
  if (!IMU.initialize(BASIC_SETTINGS)) {
    Serial.println("Failed to load IMU settings");
  }
  return success;
}

void sendIMUData() {
  unsigned long curr_t = millis();
  if (IMU_last_measurement + MEASUREMENT_INTERVAL > curr_t) return;
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(IMU.readFloatAccelX(), 3);
  Serial.print(" Y = ");
  Serial.println(IMU.readFloatAccelY(), 3);
  Serial.print(" Z = ");
  Serial.println(IMU.readFloatAccelZ(), 3);

  Serial.print("\nGyroscope:\n");
  Serial.print(" X = ");
  Serial.println(IMU.readFloatGyroX(), 3);
  Serial.print(" Y = ");
  Serial.println(IMU.readFloatGyroY(), 3);
  Serial.print(" Z = ");
  Serial.println(IMU.readFloatGyroZ(), 3);

  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees F = ");
  Serial.println(IMU.readTempF(), 3);
  DynamicJsonDocument doc(1024);
  doc["type"] = 0;
  doc["data"]["a_x"] = IMU.readFloatAccelX();
  doc["data"]["a_y"] = IMU.readFloatAccelY();
  doc["data"]["a_z"] = IMU.readFloatAccelZ();
  
  doc["data"]["g_x"] = IMU.readFloatGyroX();
  doc["data"]["g_y"] = IMU.readFloatGyroY();
  doc["data"]["g_z"] = IMU.readFloatGyroZ();
  
  doc["data"]["temp"] = IMU.readTempF();

  serializeJson(doc, Serial);
  Serial.println();
  serializeJson(doc, SerialBT);
  SerialBT.println();

  IMU_last_measurement = curr_t;
}

void setupServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    pinMode(servo_pin[i], OUTPUT);
    digitalWrite(servo_pin[i], LOW);
  }
}

void turnServo(JsonArray arguments) {
  if (arguments.size() != 2) {
    Serial.println("Incorrect number of arguments for turning servos");
    return;
  }
  int id = arguments[0];
  if (id < 0 || id >= NUM_SERVOS) {
    Serial.println("Invalid servo id");
  }
  int desired_angle = arguments[1];
  if (desired_angle < 0 || desired_angle >= RANGE) {
    Serial.println("Invalid servo angle");
  }
  int true_angle = servo_clockwise[id] ? desired_angle : 270 - desired_angle;
  char buffer[40];
  sprintf(buffer, "Turning servo %d: %d.", id, true_angle);
  Serial.println(buffer);
  unsigned long curr_t = micros();
  if (servo_pulse_start[id] + PULSE_PERIOD > curr_t) return;
  servo_pulse_start[id] = curr_t;
  servo_pulse_length[id] = PULSE_MIN + (PULSE_MAX - PULSE_MIN) * true_angle/RANGE;
  digitalWrite(servo_pin[id], HIGH);
}

void checkServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (micros() > servo_pulse_start[i] + servo_pulse_length[i]) {
      digitalWrite(servo_pin[i], LOW);
    }
  }
}

void setupWindingMotor() {
  pinMode(WINDING_MOTOR_1, OUTPUT);
  pinMode(WINDING_MOTOR_2, OUTPUT);
  pinMode(WINDING_MOTOR_ENABLE, OUTPUT);
  
  // Set up PWM channel
  ledcSetup(WINDING_MOTOR_PWM, PWM_FREQ, PWM_RES);
  
  // Attach PWM channel to GPIO
  ledcAttachPin(WINDING_MOTOR_ENABLE, WINDING_MOTOR_PWM);
}

void setWindingMotor(JsonArray arguments) {
  if (arguments.size() != 2) {
    Serial.println("Incorrect number of arguments for turning servos");
    return;
  }
  int dir = arguments[0]; // 0 - Forward, 1 - Backward, 2 - Stop
  if (dir < 0 || dir > 2) {
    Serial.println("Invalid motor state");
    return;
  }
  int duty_cycle = arguments[1];
  if (dir != 2 && (duty_cycle < 0 || duty_cycle > DUTY_CYCLE_MAX)) {
    Serial.println("Invalid duty cycle");
    return;
  }

  char buffer[70];
  sprintf(buffer, "Setting winding motor to state %d, duty cycle %d.", dir, duty_cycle);
  Serial.println(buffer);
  switch (dir) {
    case 0:
      digitalWrite(WINDING_MOTOR_1, HIGH);
      digitalWrite(WINDING_MOTOR_2, LOW);
      ledcWrite(WINDING_MOTOR_PWM, duty_cycle);   
    case 1:
      digitalWrite(WINDING_MOTOR_1, LOW);
      digitalWrite(WINDING_MOTOR_2, HIGH);
      ledcWrite(WINDING_MOTOR_PWM, duty_cycle); 
    case 2:
      digitalWrite(WINDING_MOTOR_1, LOW);
      digitalWrite(WINDING_MOTOR_2, LOW);
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

  // Set up servos
  setupServos();

  // Set up winding motor
  setupWindingMotor();
  
}

void loop() {
  checkServos();
  if (IMU_setup_success)
    sendIMUData();
  if (SerialBT.available()) {
    char c = SerialBT.read();
    Serial.print(c);
    processReceivedValue(c);
  }
}
