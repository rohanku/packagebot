/*
   GPS Portal, part of Tardis Time Server by: M. Ray Burnette 20150912
   Create a private 10. network and capture all DNS requests to the Time Portal
   Respond to both UDP and HTML requests
   Arduino GUI 1.6.7 on Linux Mint 17.3 tested on 20160203
    Sketch uses 246,644 bytes (56%) of program storage space. Maximum is 434,160 bytes.
    Global variables use 43,748 bytes (53%) of dynamic memory, leaving 38,172 bytes for local variables. Maximum is 81,920 bytes.

   ESP8266 core: http://arduino.esp8266.com/staging/package_esp8266com_index.json
*/

#include <Streaming.h>                                          // \Documents\Arduino\libraries\Streaming (legacy)
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include "./Utility.h"                                          // See Notes tab for credits

const byte   DNS_PORT  =   53;                                  // Capture DNS requests on port 53
int          ack_Count =    0;
uint8_t      hour, minute, seconds;                             // hour, minure, seconds,
uint8_t      day, month, year;                                  // year, month, day;
unsigned int localPort = 8888;                                  // any unused port on LAN
IPAddress    apIP(10, 10, 10, 1);                               // Private network address: local & gateway

char         packetBuffer[UDP_TX_PACKET_MAX_SIZE];              // buffer to hold incoming packet,
char         ReplyBuffer[] = "acknowledged";                    // a 12 character string to send back

String       responseHTML = ""
                            "<!DOCTYPE html><html><head><title>CaptivePortal</title></head><body>"
                            "<h1>Tardis Time</h1><p>Coordinated Universal Time (UTC) "
                            "HH:MM:SS  YYYYMMDD </p></body></html>";

DNSServer         dnsServer;                                    // Create the DNS object
WiFiUDP           UDP;                                          // UDP protocol on STA interface, localPort
ESP8266WebServer  webServer(80);                                // HTTP web server on common port 80

boolean connectUDP()                                            // connect to UDP – returns true if successful or false if not
{
  boolean state = false;
  Serial << endl << (F("Connecting to UDP ===> ")) ;

  if (UDP.begin(localPort) == 1) {
    Serial << (F("Connection successful")) ;
    state = true;
  }
  else {
    Serial << (F("Connection failed")) ;
    delay(50);
  }
  return state;
}

void Listeners() {
  webServer.handleClient();
  delay(0);                                                   // allow RTOS to breath

  int packetSize = UDP.parsePacket();                         // if there’s data available, read a packet
  if (packetSize)
  {
    Serial << endl;
    Serial << (F("Received packet of size "));
    Serial << (packetSize);
    Serial << (F("From "));
    IPAddress remote = UDP.remoteIP();

    for (int k = 0; k < 4; k++)
    {
      Serial << (remote[k], DEC);
      if (k < 3)
      {
        Serial << (F("."));
      }
    }
    yield();
    Serial << (F(", port "));
    Serial << "UDPremotePort" << (UDP.remotePort()) << endl;

    UDP.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);           // read the packet into packetBufffer
    Serial << (F("Contents: "));
    int value = packetBuffer[0] * 256 + packetBuffer[1];
    Serial << (value) << endl;
    ++ack_Count;                                              // added mrb to give an ack serial-number
    Serial << (F("Ack counter: "));
    Serial << (ack_Count);                                    // ever increasing serialnumber
    intToStr(ack_Count, ReplyBuffer, 12);                     // 12 characters wide
    yield();
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());        // send a reply, to the IP address and port that sent us the packet we received
    UDP.write(ReplyBuffer);
    UDP.endPacket();

    digitalWrite(2, value);                                   // turn LED on or off depending on value received
  }                                                           // if(packetSize)
}

void GPSstuff(char c) {                                       // GPSbuffer[] is global
  static int i, j;                                            // persistent within function scope
  static char q;
  static bool flag = false;
  static char GPSbuffer[120];                                  // GPS serial line input buffer
  q = c;

  if ( q == 0x24 )                                             // '$'
  {
    i = 0;                                                     // brute force sync on '$' to GPSbuffer[0]
    // Serial << "Found $" << endl;
  }

  if ( i < 120) GPSbuffer[i++] = q;
  // Serial << "Index=" << (i -1) << "Input=" << q << endl;
  //if (i = 120) i = 119;                                      // array limit safety

  if (q == 0x0d) {
    flag = true;                                               // is the character a CR for eol
    i = 0;
  }

  if (flag) {                                                  // test for end of line and if the right GPSbuffer
    flag = false;                                              // reset for next time
    // Serial << "We are in the flag routine..." << GPSbuffer[3] << GPSbuffer[4] << GPSbuffer[5] << endl;
    // Serial << "Analyzing " << GPSbuffer[3] << GPSbuffer[4] << GPSbuffer[5] << endl;
    if ( (GPSbuffer[3] == 0x52) && (GPSbuffer[4] == 0x4d) && (GPSbuffer[5] == 0x43)) // 'R' && 'M' && 'C'
    {
      for (j = 0; j < 120 ; j++) {
        UDP.write(GPSbuffer[j]);
      }
      UDP.write("\r\n");                                         // terminate the line
      UDP.endPacket();                                           // clear UDP buffer
    }
  }
}


void setup()
{
  Serial.begin(9600);                                           // Initialise Serial connection
  Serial << (F("(c) 2015 M. Ray Burnette")) << endl;
  Serial << (F("Tardis Time Portal 0.20150915")) << endl << endl;

  WiFi.mode(WIFI_AP_STA);                                       // AP + STA
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));   // subnet FF FF FF 00
  WiFi.softAP("TardisTime");

  dnsServer.start(DNS_PORT, "*", apIP);                       // "*" creates a captive portal

  webServer.onNotFound([]() {                                 // replay to all requests with same HTML
    webServer.send(200, "text/html", responseHTML);
  });

  webServer.begin();                                          // Start HTTP services

  while (! connectUDP() ) {                                   // UDP protocol connected to local
    Serial << "+" ;
    yield();                                                  // Play nicely with RTOS (alias = delay(0))
  }

  // This will loop forever if UDP fails
  Serial << endl;
  Serial << (F("Setting pin#2 to Output mode")) << endl;
  pinMode(2, OUTPUT);                                         // initialise pin mode
}


void loop()
{
  dnsServer.processNextRequest();                             // TCP Address handler when requested
  delay(0);                                                   // yield for RTOS
  Listeners();                                                // UPD and HTTP
  delay(0);
  if (Serial.available() > 0) {                               // anything in the serial hw buffer?
      char c = Serial.read();                                 // if so, fetch the next character from FIFO
      GPSstuff(c);
  }
  delay(0);
}                                                             // loop repeats forever unless stack crashes or uC hangs
