#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#define ARRAY_SIZE 5500
//#include "SPI.h"

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B
float rolldata[ARRAY_SIZE];
float pitchdata[ARRAY_SIZE];
float yawdata[ARRAY_SIZE];
float x_accel[ARRAY_SIZE]; //forward back
float y_accel[ARRAY_SIZE]; //left right
float z_accel[ARRAY_SIZE]; //up down
int counter = 0; //counter; when counter reaches falldata.length, stop adding values
int inputPin = 7; //pin number



void setup() {

  
  Serial.begin(9600);
  delay(500); 

  Wire.begin();
  pinMode(inputPin, INPUT); //wait for input
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
  for (int i = 0; i < ARRAY_SIZE; i++) {
    Serial.println(input[i]);
  }
}

void loop()
{
  //if we get input from the pin, barf all the data up into serial.out
  if (digitalRead(inputPin) != LOW) {
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
    delay(10000);
  }

  if (counter < ARRAY_SIZE) {
    //note: these values are all acceleration and not absolute 
    rolldata[counter] = myIMU.readFloatGyroX();
    pitchdata[counter] = myIMU.readFloatGyroY();
    rolldata[counter] = myIMU.readFloatGyroZ();
    //accel data
    x_accel[counter] = myIMU.readFloatAccelX();
    y_accel[counter] = myIMU.readFloatAccelY();
    z_accel[counter] = myIMU.readFloatAccelZ();
    counter += 1;
  }
  
  delay(50);
}
