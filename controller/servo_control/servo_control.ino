#include "WiFi.h"

#define SERVO_1 13
#define PULSE_MIN 500
#define PULSE_MAX 2500
#define RANGE 270

const char* ssid = "aurora2";
const char* password =  "b0r3@l1s";
IPAddress local_IP(192, 168, 1, 14);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);
WiFiServer wifiServer(80);
WiFiClient client;

unsigned long pulseStart = -1;
unsigned long pulseLength;

void processReceivedValue(char command){
 
  if(command == '1'){ turnServo(SERVO_1, 0); }
  else if(command == '0'){ turnServo(SERVO_1, 180);}
 
  return;
}

void turnServo(int pin, double angle) {
  if (pulseStart >= 0) return;
  pulseStart = micros();
  pulseLength = PULSE_MIN + (PULSE_MAX - PULSE_MIN) * angle/RANGE;
  digitalWrite(pin, HIGH);
}

void checkServo(int pin) {
  if (pulseStart < 0) return;
  if (micros() > pulseStart + pulseLength) {
    pulseStart = -1;
    digitalWrite(pin, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(SERVO_1, OUTPUT);
  digitalWrite(SERVO_1, LOW);

  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
 
  wifiServer.begin();
  client = wifiServer.available();
}

void loop() {
  checkServo(SERVO_1);
  if (client) {
    if (client.connected()) {
 
      if (client.available()>0) {
        char c = client.read();
        processReceivedValue(c);
        Serial.write(c);
      }

    } else {
      client.stop();
    }
  } else {
    client = wifiServer.available();
  }
}
