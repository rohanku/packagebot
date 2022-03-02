#include "Adafruit_VL53L0X.h"

#define threshold 45.0

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
unsigned long startMeasure = 0;

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(100);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  //Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    //Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    if (measure.RangeMilliMeter < threshold && startMeasure == 0) {
      Serial.println("Starting measurement...");
      startMeasure = millis();
    } else if (measure.RangeMilliMeter > threshold && startMeasure != 0) {
      Serial.print("Time (ms): "); Serial.println(millis() - startMeasure + 150);
      startMeasure = 0;
    }
  } else {
    Serial.println(" out of range ");
  }
    
  delay(50);
}
