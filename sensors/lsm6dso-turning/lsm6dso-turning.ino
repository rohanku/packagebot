#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#define ARRAY_SIZE 12000
//#include "SPI.h"
#define DELAY 50 

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B
int windowStart = 0;
int windowLength = 20;
int threshold = 5;
int inputPin = 7; //pin number

float yawdata[ARRAY_SIZE];

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! please run `make menuconfig to enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");
  delay(500); 

  Wire.begin(23, 22);
  delay(10);

  if( myIMU.begin() )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");

}


int checkTurn(float input[]) {
  boolean lt = false;
  boolean rt = false;
  boolean straight = false;
  int turnVal;
  for (int i = windowStart; i < windowLength; i++) {
    for (int j = windowStart; j < 4; j++) {
      turnVal += yawdata[j];
    }
    if (turnVal*10 > threshold && !rt) {
      lt = true; //left
      rt = false;
    } else if (turnVal * 10 < threshold && !lt) {
      rt = true; //right
      lt = false;
    } else {
      straight = true;
    }
    turnVal = 0;
  }

  if (straight) {
    return 0; //going straight
  } else if (lt) {
     return 1; // turning left
  } else if (rt) {
     return 2; // turning right
  } else {
     return 0;
  }
}

void loop() {
  int turnStatus;
  yawdata[counter] = myIMU.readFloatGyroY();
  
  if (counter > 30) {
    int turnStatus = checkTurn(yawdata); 
    windowStart += 1; //increment the window
  }

  if (SerialBT.available()) {
     char c = SerialBT.read();
     if (c == '\n') {
        delay(100);
        if (turnStatus == 0) {
           SerialBT.println("Going Straight..."); 
           //pseudocode here
           //if (servos ! at base position) {
           //   reset servos
           //}
        } else if (turnStatus == 1) {
           SerialBT.println("Turning Left..."); 

        } else if (turnStatus == 2) {
           SerialBT.println("Turning Rihgt..."); 
        }
     }
  }
  counter += 1;
  delay(20);
}
