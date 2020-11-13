#include "Wire.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
//#include "keys.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ------------ Globals

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 6

#define SEALEVELPRESSURE_HPA (1022.6)

Adafruit_BME280 bme; // I2C

int altitude, lastAlt, apogee;

unsigned int numTones = 6;
unsigned int tones[] = {523, 587, 659, 739, 830, 880};
////            upper    C    D    E    F#   G#   A

boolean chutesFired = false;
boolean landed = false;

char packetBuffer[100];
char flightData[60] = "{""\"status\":""\"Flight CPU sent data to server\"""}";
char chutes[50] = "{""\"status\":""\"Chutes Deployed!\"""}";
char liftoff[50] = "{""\"status\":""\"Liftoff!\"""}";
char bmeDetected[60] = "{""\"status\":""\"BME 280 Detected.\"""}";
char imuDetected[60] = "{""\"status\":""\"MPU 6050 Detected.\"""}";
char airDetected[70] = "{""\"status\":""\"AirLift Featherwing Detected.\"""}";
char startupCompleted[60] = "{""\"status\":""\"Startup process complete.\"""}";
char flightCPU[50] = "{""\"status\":""\"Flight CPU ready\"""}";
char vehicleLanded[50] = "{""\"status\":""\"Touchdown!\"""}";



// ------------------------ MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// ------------------------ orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ---------------------- IMU interruption detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ------------ Setup

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    
  Serial.begin(115200);
  for (unsigned int i = 0; i < numTones; i++)
  {
    tone(A1, tones[i]);
    delay(50);
  }
  noTone(A1);
  delay(2000);  // time to get serial running
  unsigned status;
 

  // default settings
  status = bme.begin();
//  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    while (1) delay(10);
  } else {
    if (status) {
      tone(A1, 523);
      delay(50);
      noTone(A1);
      delay(1000);
    }
  }
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  tone(A1, 523);
  delay(50);
  noTone(A1);
  delay(1000);

  devStatus = mpu.dmpInitialize();


  // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-1);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(-1);
    mpu.setZAccelOffset(16381);

  if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  analogWrite(A0, 150);
  delay(200);

  Serial.println("");

  tone(A1, 1760);
  delay(500);
  noTone(A1);

//  delay(1000);
//  analogWrite(A0, 150);

// Relay
//  pinMode(A2, OUTPUT);
//  digitalWrite(A2, LOW);
  
}

// ------------ Main loop

void loop() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  }
  
  // Check rocket has not surpassed apogee 
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  if (altitude - lastAlt <= -1) {
    delay(100);
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    if (altitude - lastAlt <= -2) {
      delay(100);
      altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
      if (altitude - lastAlt <= -3) {
        apogee = lastAlt - 3;
        delay(150);
        deploy();
      } else {
      lastAlt = altitude;
      }
    } else {
      lastAlt = altitude;
      } 
   } else {
    lastAlt = altitude;
   }


   if (chutesFired == true) {
    // BEGIN DESCENT // 
        analogWrite(A0,150);
        delay(100);
        analogWrite(A0, 0);
        delay(100);
        analogWrite(A0,150);
        delay(100);
        analogWrite(A0, 0);
        delay(100);
        touchdown();
   }
  

  // ------------ Convert collected data to JSON
  String Payload = "{""\"Temperature\":";
  Payload += bme.readTemperature();
  Payload += ",""\"Pressure\":";
  Payload += (bme.readPressure() / 100.0F);
  Payload += ",""\"Altitude\":";
  Payload += bme.readAltitude(SEALEVELPRESSURE_HPA);
  Payload += ",""\"Humidity\":";
  Payload += bme.readHumidity();
  Payload += ",""\"Yaw\":";
  Payload += ypr[0] * 180/M_PI;
  Payload += ",""\"Pitch\":";
  Payload += ypr[1] * 180/M_PI;
  Payload += ",""\"Roll\":";
  Payload += ypr[2] * 180/M_PI;
  Payload += ",""\"X\":";
  Payload += aaReal.x;
  Payload += ",""\"Y\":";
  Payload += aaReal.y;
  Payload += ",""\"Z\":";
  Payload += aaReal.z;
  Payload += "}";
  
  char packet[200];
  int i = 0;
  int sizeOf = 200;
  int offset = 0;
  
  while((i<sizeOf))
  {
     packet[i]=Payload[i];  
     i++;
  }
  
//  Udp.beginPacket(remoteIp, 2931);
//  Udp.write(packet);
//  Udp.endPacket();
  Serial.println(packet);
//  delay(100);
  
//  Udp.beginPacket(groundIp, 8888);
//  Udp.write(flightData);
//  Udp.endPacket();
  delay(100);
}


void deploy() {
  chutesFired = true;
  // Fire relay
  digitalWrite(A2, HIGH);
  delay(1000);
  digitalWrite(A2, LOW);
  delay(100);
}

void touchdown() {
  if (altitude - lastAlt == 0) {
    delay(100);
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    if (altitude - lastAlt == 0) {
      delay(100);
      altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
      if (altitude - lastAlt == 0) {
        landed = true;
        delay(150);
      } else {
        lastAlt = altitude;
      }
    } else {
      lastAlt = altitude;
    }
   } else {
    lastAlt = altitude;
   }
}
