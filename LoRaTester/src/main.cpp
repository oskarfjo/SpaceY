#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration.h>
#include "sensor.h"
#include <FS.h>
#include <SdFat.h>

// coms bool
bool launchSignalReceived = false;
bool armed = false;

// Sensor PARAMS //
float imuData[12]; // array for imu data ; [heading, pitch, roll][0,0,0][0,0,0][0,0,0]
float barData[3]; // array for barometer data
float gpsData[10]; // array for gps data


extern bool launchSignaled;
extern bool armSignaled;


//////////
// INIT //
//////////

void setup() {

  Serial.begin(115200);
  pinMode(29, OUTPUT); // LED
  initSensors();
  delay(2000);

  digitalWrite(29, HIGH);
  delay(100);
  digitalWrite(29, LOW);
  delay(100);
  digitalWrite(29, HIGH);
  delay(100);
  digitalWrite(29, LOW);
}


//////////
// LOOP //
//////////

void loop() {

  receiveLoRaMessage();

  //sendLoRaMessage("test");

  if (armSignaled && !armed) {
      armed = true;
    }

  if (armed) {
    digitalWrite(29, HIGH);
  }

  delay(1);

}
