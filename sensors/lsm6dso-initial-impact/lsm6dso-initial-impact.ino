/******************************************************************************
Basic_Readings.ino

https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library

Description:
Most basic example of use.

Example using the LSM6DSO with basic settings.  This sketch collects Gyro and
Accelerometer data every second, then presents it on the serial monitor.

Development environment tested:
Arduino IDE 1.8.2

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#define ARRAY_SIZE 12000
//#include "SPI.h"

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B
long falldata[ARRAY_SIZE]; //array to store info
int counter = 0; //counter; when counter reaches falldata.length, stop adding values
int inputPin = 7; //pin number

void setup() {

  
  Serial.begin(115200);
  delay(500); 

  Wire.begin();
  pinMode(inputPin, INPUT); // wait for input
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


void loop()
{
  //if we get input from the pin, throw all the data up into serial.out
  if (digitalRead(inputPin) != LOW) {
    delay(100);
    //spit out all data
    for (int i = 0; i < ARRAY_SIZE; i++) {
      Serial.println(falldata[i]);
    }
    delay(5000);
  }
  //drop impact settings (initial box mount)
   
  //Serial.print(" Z = ");
  //Serial.println(myIMU.readFloatAccelZ()*10, 10);
  if (counter < ARRAY_SIZE) {
    if (myIMU.readFloatAccelZ()*10<-5 || myIMU.readFloatAccelZ()*10>15) {
      Serial.println("drop detected");
      delay(5000);
    }
  
    falldata[counter] = myIMU.readFloatAccelZ()*10;
    counter += 1;
  }
  
  delay(20);
}
