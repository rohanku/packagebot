#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "BluetoothSerial.h"
#include "ArduinoJson.h"

// IMU constants
#define MEASUREMENT_INTERVAL 2000

// Servo constants
#define NUM_SERVOS 2
const int servo_pin[] = {26, 25};
const int servo_pwm[] = {1, 2};
const bool servo_clockwise[] = {true, false};
#define PULSE_MIN 700
#define PULSE_MAX 2500
#define PULSE_PERIOD 20000
#define SERVO_PWM_FREQ 50
#define RANGE 180

// Motor constants
#define WINDING_MOTOR_1 13
#define WINDING_MOTOR_2 12
#define WINDING_MOTOR_ENABLE 27
#define WINDING_MOTOR_PWM 0
#define MOTOR_PWM_FREQ 30000
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
  Serial.println("Setting up IMU.");
  bool success = IMU.begin();
  if (!IMU.initialize(BASIC_SETTINGS)) {
    Serial.println("Failed to load IMU settings");
  }
  return success;
}

void sendIMUData() {
  unsigned long curr_t = millis();
  if (IMU_last_measurement + MEASUREMENT_INTERVAL > curr_t) return;
  DynamicJsonDocument doc(1024);
  doc["type"] = 0;
  doc["data"]["a_x"] = IMU.readFloatAccelX();
  doc["data"]["a_y"] = IMU.readFloatAccelY();
  doc["data"]["a_z"] = IMU.readFloatAccelZ();
  
  doc["data"]["g_x"] = IMU.readFloatGyroX();
  doc["data"]["g_y"] = IMU.readFloatGyroY();
  doc["data"]["g_z"] = IMU.readFloatGyroZ();
  
  doc["data"]["temp"] = IMU.readTempF();
  
  serializeJson(doc, SerialBT);
  SerialBT.println();

  IMU_last_measurement = curr_t;
}

void setupServos() {
  Serial.println("Setting up servos.");
  for (int i = 0; i < NUM_SERVOS; i++) {
    ledcSetup(servo_pwm[i], SERVO_PWM_FREQ, PWM_RES);
    ledcAttachPin(servo_pin[i], servo_pwm[i]);
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
  int true_angle = servo_clockwise[id] ? desired_angle : RANGE - desired_angle;
  int duty_cycle = (PULSE_MIN + true_angle * (PULSE_MAX - PULSE_MIN) / RANGE) * DUTY_CYCLE_MAX / PULSE_PERIOD;
  char buffer[40];
  sprintf(buffer, "Turning servo %d: %d %d.", id, true_angle, duty_cycle);
  Serial.println(buffer);
  ledcWrite(servo_pwm[id], duty_cycle);
}

void setupWindingMotor() {
  Serial.println("Setting up winding motor.");
  pinMode(WINDING_MOTOR_1, OUTPUT);
  pinMode(WINDING_MOTOR_2, OUTPUT);
  pinMode(WINDING_MOTOR_ENABLE, OUTPUT);
  digitalWrite(WINDING_MOTOR_1, LOW);
  digitalWrite(WINDING_MOTOR_2, LOW);
  
  // Set up PWM channel
  ledcSetup(WINDING_MOTOR_PWM, MOTOR_PWM_FREQ, PWM_RES);
  
  // Attach PWM channel to GPIO
  ledcAttachPin(WINDING_MOTOR_ENABLE, WINDING_MOTOR_PWM);
}

void setWindingMotor(JsonArray arguments) {
  if (arguments.size() != 2) {
    Serial.println("Incorrect number of arguments for turning winding motor");
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
      break;
    case 1:
      digitalWrite(WINDING_MOTOR_1, LOW);
      digitalWrite(WINDING_MOTOR_2, HIGH);
      ledcWrite(WINDING_MOTOR_PWM, duty_cycle);
      break;
    case 2:
      digitalWrite(WINDING_MOTOR_1, LOW);
      digitalWrite(WINDING_MOTOR_2, LOW);
      break;
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
  if (IMU_setup_success)
    sendIMUData();
  if (SerialBT.available()) {
    char c = SerialBT.read();
    Serial.print(c);
    processReceivedValue(c);
  }
}
