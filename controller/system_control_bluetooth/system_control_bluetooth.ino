#include "WiFi.h"

#define SERVO_1 25
#define PULSE_MIN 500
#define PULSE_MAX 2500
#define PULSE_PERIOD 20000
#define RANGE 270
#define DELIMITER ' '

//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

/*void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);
}*/

unsigned long pulseStart = 0;
unsigned long pulseLength;

String command = "";
unsigned long command_index = 0;

void processReceivedValue(char b){
  if(b == DELIMITER){
    int angle = command.toInt();
    String debug = String("Turning servo: ") + angle;
    Serial.println(debug);
    turnServo(SERVO_1, angle);
    command = "";
  }
  else if('0' <= b && b <= '9'){
    command.concat(b);
  } else {
    command = "";
  }
 
  return;
}

void turnServo(int pin, int angle) {
  unsigned long curr_t = micros();
  if (pulseStart + PULSE_PERIOD > curr_t) return;
  pulseStart = curr_t;
  pulseLength = PULSE_MIN + (PULSE_MAX - PULSE_MIN) * angle/RANGE;
  digitalWrite(pin, HIGH);
}

void checkServo(int pin) {
  if (micros() > pulseStart + pulseLength) {
    digitalWrite(pin, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin("ESP32");
  pinMode(SERVO_1, OUTPUT);
  digitalWrite(SERVO_1, LOW);
}

void loop() {
  checkServo(SERVO_1);
  if (SerialBT.available()) {
    char c = SerialBT.read();
    processReceivedValue(c);
    Serial.write(c);
  }
}
