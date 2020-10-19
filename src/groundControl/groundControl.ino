#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
//#include <SoftwareSerial.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "keys.h"


#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS    10   // Chip select pin
#define ESP32_RESETN   5   // Reset pin
#define SPIWIFI_ACK    7   // a.k.a BUSY or READY pin
#define ESP32_GPIO0    6


unsigned long delayTime = 200;

IPAddress remoteIp(192, 168, 1, 65);
unsigned int localPort = 8888;      // local port to listen on
WiFiUDP Udp;
boolean launchStatus;

//unsigned int remotePort = 2930;

char packetBuffer[255];
char ReplyBuffer[255];

char countdown5[10], countdown4[10], countdown3[10], countdown2[10], countdown1[10];

char networkName[] = NETWORK;
char password[] = PASSWORD;

char launch[100], ignition[100];


void setup() {
  Serial.begin(9600);
  delay(2000);  // time to get serial running
  Serial.println(F("Power on"));

  unsigned status;
  unsigned wStatus;

  launchStatus = false;
  
  char groundControlDetected[100], flightCPU[100];
  int j = 0;
  int sizeOf = 100;
  int offset = 0;
  String gStr = "{""\"status\":""\"Ground Control Detected.\"""}";
  String reply = "{""\"status\":""\"Connection established, vehicle will launch.\"""}";
  String go4launch = "{""\"status\":""\"Launch initiated.\"""}";
  String launchComplete = "{""\"status\":""\"Ignition\"""}";

  while ((j < sizeOf))
  {
    groundControlDetected[j + offset] = gStr[j];
    ReplyBuffer[j + offset] = reply[j];
    launch[j + offset] = go4launch[j];
    ignition[j + offset] = launchComplete[j];
    j++;
  }

  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
  while (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("AirLift Module not detected!"));
    delay(1000);
  }

  Serial.println(F("AirLift Module detected."));

  Serial.println(F("Attempting to connect to WiFi "));
  do {
    wStatus = WiFi.begin(networkName, password);
    delay(100);
  } while (wStatus != WL_CONNECTED);
  printWifiStatus();

  Udp.begin(localPort);

  Udp.beginPacket(remoteIp, 2931);
  Udp.write(groundControlDetected);
  Udp.endPacket();
  pinMode(A0, OUTPUT);


}

void loop() {
  analogWrite(A0, 0);
  // Check for packet data from flight cpu
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    IPAddress remote = Udp.remoteIP();
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.print("Received: ");
    Serial.println(packetBuffer);

    analogWrite(A0, 150);

    if (launchStatus == false) {
      beginLaunch();
    }

      /* In theory, I would want an indication on the ground that the chutes successfully deployed*/
//    if (packetBuffer == '{"status":"Deploy chutes!"}') {
//      // Confirmation of deployed chutes
//      Serial.println("CONFIRM");
//      analogWrite(A0, 0);
//      delay(500);
//      analogWrite(A0, 150);
//      delay(500);
//    }
  }

  delay(500);

  // Flight computer startup has completed at this point, the next step would be to begin the launch sequence,
  // and ignite the motor using analogWrite(). -- Find more features to add to ground control

}

void beginLaunch() {
  launchStatus = true;
  Udp.beginPacket(remoteIp, 2931);
  Udp.write(launch);
  Udp.endPacket();
  delay(500);

  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);
  analogWrite(A0, 150);
  delay(1000);
  analogWrite(A0, 0);
  delay(500);


  Udp.beginPacket(remoteIp, 2931);
  Udp.write(ignition);
  Udp.endPacket();
  delay(500);


}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print(F("signal strength (RSSI):"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));

}
