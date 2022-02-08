// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// Tested on 17 September 2019
// ---------------------------------------------------------------- //

#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 // attach pin D3 Arduino to pin Trig of HC-SR04
#define threshold 15.0 // minimum distance to begin measurement
#define window 25

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
int stage = 0;
int stageStart;
unsigned long startMeasure = 0;
int measurements[window];

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
  stageStart = micros();
}
void loop() {
  // Clears the trigPin condition
  if (stage == 0 && micros() - stageStart > 10) {
    stage++;
    digitalWrite(trigPin, LOW);
    stageStart = micros();
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; 
    
    shift_insert(distance);
    double avg = window_distance();
//    Serial.print("Distance: ");
//    Serial.print(avg);
//    Serial.println(" cm");
    
    if (avg > threshold && startMeasure > 0) {
      int timeElapsed = millis() - startMeasure;
      Serial.print("Time Elapsed: ");
      Serial.print(timeElapsed - 200);
      Serial.println(" ms");
      startMeasure = 0;
    }
    if (avg < threshold && startMeasure <= 0) {
      startMeasure = millis();
      Serial.print(startMeasure);
      Serial.println("Start measurement");
    }
  }
  if (stage == 1 && micros() - stageStart > 2) {
    stage = 0;
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    stageStart = micros();
  }
  
}

double window_distance() {
  int tot = 0;
  for (int i = 0; i < window; i++) {
    tot += measurements[i];
  }
  return (double) tot / window;
}

void shift_insert(int distance) {
  for (int i = 0; i < window-1; i++) {
    measurements[i] = measurements[i+1];
  }
  measurements[window-1] = distance;
}
