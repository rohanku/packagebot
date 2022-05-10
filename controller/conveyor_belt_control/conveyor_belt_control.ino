#include "Adafruit_VL53L0X.h"
#include "Servo.h"

// Hardware constants
#define THRESHOLD 100
#define MOTOR 22
#define RELEASE_PIN 2
#define HOLD_ANGLE 85
#define RELEASE_ANGLE 0

// Timing constants
#define FALL_WAIT 10000
#define FALL_TO_WINDING_DELAY 2000
#define WINDING_WAIT 10000
#define WINDING_TO_LAUNCH_DELAY 5000

int launched = false;
Servo release_servo;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  pinMode(MOTOR, OUTPUT);
  digitalWrite(MOTOR, 1);
  release_servo.attach(RELEASE_PIN);
  release_servo.write(HOLD_ANGLE);  

  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while(1);
  }
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); 

  // Once box reaches distance sensor, start loading process
  if (!launched && measure.RangeMilliMeter < THRESHOLD) {
    Serial.println("Dropping mount");
    digitalWrite(MOTOR, 0);
    release_servo.write(RELEASE_ANGLE);
    delay(FALL_WAIT);
    Serial.println("Moving to winding position");
    release_servo.write(HOLD_ANGLE);
    digitalWrite(MOTOR, 1);
    delay(FALL_TO_WINDING_DELAY);
    Serial.println("Waiting for winding to complete");
    digitalWrite(MOTOR, 0);
    delay(WINDING_WAIT);
    Serial.println("Launching");
    digitalWrite(MOTOR, 1);
    delay(WINDING_TO_LAUNCH_DELAY);
    digitalWrite(MOTOR, 0);
    launched = true;
  }
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
}
