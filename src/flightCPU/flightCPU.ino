#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_LIS3DH.h>
#include <LiquidCrystal.h>
//#include <SoftwareSerial.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "keys.h"

#define LIS3DH_CLK 10
#define LIS3DH_MISO 9
#define LIS3DH_MOSI 6
// Used for hardware & software SPI
#define LIS3DH_CS 5

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

#define SEALEVELPRESSURE_HPA (1016.3)

#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS    13   // Chip select pin
#define ESP32_RESETN  12   // Reset pin
#define SPIWIFI_ACK   11   // a.k.a BUSY or READY pin
#define ESP32_GPIO0   -1

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

//LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

IPAddress remoteIp(192, 168, 1, 65);
IPAddress groundIp(192, 168, 1, 153);
unsigned int localPort = 2390;      // local port to listen on
WiFiUDP Udp;

int altitude, lastAlt, apogee;

char networkName[] = NETWORK;
char password[] = PASSWORD;

char packetBuffer[100];
char wait[80] = "Flight Computer waiting to establish connection to Ground Control";
char flightData[100] = "{""\"status\":""\"Flight CPU sent data to server\"""}";
char chutes[80] = "{""\"status\":""\"Deploy chutes!\"""}";
char liftoff[80] = "{""\"status\":""\"Liftoff!\"""}";
char bmeDetected[100] = "{""\"status\":""\"BME 280 Detected.\"""}";
char lisDetected[100] = "{""\"status\":""\"LIS3DH Detected.\"""}";
char airDetected[100] = "{""\"status\":""\"AirLift Featherwing Detected.\"""}";
char startupCompleted[100] = "{""\"status\":""\"Startup process complete.\"""}";
char flightCPU[100] = "{""\"status\":""\"Flight CPU ready\"""}";

void setup() {
  Serial.begin(9600);
  delay(2000);  // time to get serial running

  unsigned status;
  unsigned lStatus;
  unsigned wStatus;

  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);  
  while (WiFi.status() == WL_NO_MODULE) {
    delay(1000);
  }
  
  Serial.println(F("Connecting to WiFi "));
  do {
    wStatus = WiFi.begin(networkName, password);
    delay(100);
  } while (wStatus != WL_CONNECTED);
  printWifiStatus();
  Udp.begin(localPort);

  Udp.beginPacket(remoteIp, 2931);
  if (wStatus) {
    Udp.write(airDetected);
  }
  Udp.endPacket();
  delay(2000);

  // default settings
  status = bme.begin();
//  // You can also pass in a Wire library object like &Wire2
//  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"));
    while (1) delay(10);
  } else {
    Udp.beginPacket(remoteIp, 2931);
    if (status) {
      Udp.write(bmeDetected);
    }
    Udp.endPacket();
    delay(2000);
  }

  wStatus = WL_IDLE_STATUS;

  delayTime = 1000;
  lStatus = lis.begin(0x18);

  if (!lStatus) {   // change this to 0x19 for alternative i2c address
    Serial.println(F("Couldnt start"));
    while (1) yield();
  }

  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  
  Udp.beginPacket(remoteIp, 2931);
  if (lStatus) {
    Udp.write(lisDetected);
  }
  Udp.endPacket();
  delay(2000);

  pinMode(A0, OUTPUT);
  analogWrite(A0, 150);
   delay(200);
  
  // Attempting to send confirmation to remote server to show startup has completed, but it's not loading the rest of the code when I do that... hmmmm
  Udp.beginPacket(remoteIp, 2931);
  Udp.write(startupCompleted);
  Udp.endPacket();
  delay(2000);

  Udp.beginPacket(groundIp, 8888);
  Udp.write(flightCPU);
  Udp.endPacket();
  Serial.println("Sent flight data to ground control");
  delay(2000);
  analogWrite(A0, 150);
  
}

void loop() {
  lis.read();
  sensors_event_t event;
  lis.getEvent(&event);

  // Check rocket has not surpassed apogee
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  if (altitude - lastAlt  <= -1) {
    delay(150);
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    if (altitude - lastAlt <= -2) {
      delay(150);
      altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
      if (altitude - lastAlt <= -3) {
        apogee = lastAlt - 3;
        Udp.beginPacket(remoteIp, 2931);
        Udp.write(chutes); // THIS WORKS!! 
        Udp.endPacket();
        delay(100);
        Udp.beginPacket(groundIp, 8888);
        Udp.write(chutes);
        Udp.endPacket();
        // BEGIN DESCENT // 
        analogWrite(A0,150);
        delay(500);
        analogWrite(A0, 0);
        delay(500);
        analogWrite(A0,150);
        delay(500);
        analogWrite(A0, 0);
        delay(500);
      } else {
      lastAlt = altitude;
      }
    } else {
      lastAlt = altitude;
    } 
  } else {
    lastAlt = altitude;
  }

  if (altitude - lastAlt  >= 1) {
    delay(150);
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    if (altitude - lastAlt >= 2) {
      delay(150);
      altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
      if (altitude - lastAlt >= 3) {
        apogee = lastAlt + 3;
        Udp.beginPacket(remoteIp, 2931);
        Udp.write(liftoff); 
        Udp.endPacket();
        delay(100);
        Udp.beginPacket(groundIp, 8888);
        Udp.write(liftoff);
        Udp.endPacket();
        // BEGIN ASCENT // 
        analogWrite(A0,150);
        delay(500);
        analogWrite(A0, 0);
        delay(500);
        analogWrite(A0,150);
        delay(500);
        analogWrite(A0, 0);
        delay(500);
      } else {
      lastAlt = altitude;
      }
    } else {
      lastAlt = altitude;
    } 
  } else {
    lastAlt = altitude;
  }

  
  String ReplyBuffer = "{""\"Temperature\":";
  ReplyBuffer += bme.readTemperature();
  ReplyBuffer += ",""\"Pressure\":";
  ReplyBuffer += (bme.readPressure() / 100.0F);
  ReplyBuffer += ",""\"Altitude\":";
  ReplyBuffer += bme.readAltitude(SEALEVELPRESSURE_HPA);
  ReplyBuffer += ",""\"Humidity\":";
  ReplyBuffer += bme.readHumidity();
  ReplyBuffer += ",""\"X\":";
  ReplyBuffer += lis.x;
  ReplyBuffer += ",""\"Y\":";
  ReplyBuffer += lis.y;
  ReplyBuffer += ",""\"Z\":";
  ReplyBuffer += lis.z;
  ReplyBuffer += ",""\"∆X\":";
  ReplyBuffer += event.acceleration.x;
  ReplyBuffer += ",""\"∆Y\":";
  ReplyBuffer += event.acceleration.y;
  ReplyBuffer += ",""\"∆Z\":";
  ReplyBuffer += event.acceleration.z;
  ReplyBuffer += "}";
  
  char reply[300];
  int i = 0;
  int sizeOf = 180;
  int offset = 0;
  
  while((i<sizeOf))
  {
     reply[i+offset]=ReplyBuffer[i];  
     i++;
  }
  
  Udp.beginPacket(remoteIp, 2931);
  Udp.write(reply);
  Udp.endPacket();
  Serial.println(reply);
  delay(100);
  
  Udp.beginPacket(groundIp, 8888);
  Udp.write(flightData);
  Udp.endPacket();
  delay(100);
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
  Serial.print(F("signal strength:"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));

}
