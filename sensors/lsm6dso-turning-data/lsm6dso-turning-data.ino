#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#define ARRAY_SIZE 1800 //this number divided by (1000/delay) is the number of seconds the program will run for :D
#define DELAY 50 //the lower the delay, the more frequent the sampling
//#include "SPI.h"

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B
float rolldata[ARRAY_SIZE];
float pitchdata[ARRAY_SIZE];
float yawdata[ARRAY_SIZE];
float x_accel[ARRAY_SIZE]; //forward back
float y_accel[ARRAY_SIZE]; //left right
float z_accel[ARRAY_SIZE]; //up down
int counter = 0; //counter; when counter reaches falldata.length, stop adding values
int timer = 0;



void setup() {

  
  Serial.begin(115200);
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


void printall(float input[]) {
  for (int i = 0; i < counter; i++) {
    Serial.println(input[i], 3);
  }
}

void loop()
{
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n')  {
      delay(100);
      Serial.println("Roll");
      printall(rolldata);
      Serial.println("Pitch");
      printall(pitchdata);
      Serial.println("Yaw");
      printall(yawdata);
      Serial.println("XAccel");
      printall(x_accel);
      Serial.println("YAccel");
      printall(y_accel);
      Serial.println("ZAccel");
      printall(z_accel);
      Serial.println("restarting...");
      counter = 0;
    }
  }

  if (counter < ARRAY_SIZE) {
    //note: these values are all acceleration and not absolute 
    rolldata[counter] = myIMU.readFloatGyroX();
    pitchdata[counter] = myIMU.readFloatGyroY();
    yawdata[counter] = myIMU.readFloatGyroZ();
    //accel data
    x_accel[counter] = myIMU.readFloatAccelX();
    y_accel[counter] = myIMU.readFloatAccelY();
    z_accel[counter] = myIMU.readFloatAccelZ();
    counter += 1;
  }
  delay(DELAY);
}
