#include "WiFi.h"

#define LED 13

const char* ssid = "aurora2";
const char* password =  "b0r3@l1s";
WiFiServer wifiServer(80);

void processReceivedValue(char command){
 
  if(command == '1'){ digitalWrite(LED, HIGH); }
  else if(command == '0'){ digitalWrite(LED, LOW); }
 
  return;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
 
  wifiServer.begin();
  pinMode(LED, OUTPUT);
}

void loop() {
  WiFiClient client = wifiServer.available();
  if (client) {
    while (client.connected()) {
 
      while (client.available()>0) {
        char c = client.read();
        processReceivedValue(c);
        Serial.write(c);
      }
   
      delay(10);
    }
  }
  client.stop();
}
