#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration.h>
#include <FS.h>

#include "logger.h"
#include "sensor.h"
#include "SimulatorInterface.h"
#include "rocketUtils.h"
#include "regulator.h"
#include "actuator.h"
#include "flightData.h"

void readSensors();
void calibrateSensors();
void flightPhases();
void testPhases();
void gimbalTest();
void reciever(unsigned long interval);

// coms bool
int armedSound = 200;
unsigned long lastMessageTime = 0;

// GIMBAL PARAMS  in deg//
extern const int gimbalLim = 7; // +-

// Sensor PARAMS //
float gpsData[10]; // array for gps data

float altitudePrev = 0.0;

// store of values from last loop
unsigned long timePrev = 0;
unsigned long timeIgnite = 0;

// Logging //
bool logging = true;
const unsigned long logInterval = 100; // ms
unsigned long logTimePrev = 0;

// SIMULATOR //
float simRead[12];

// gimbal oscillator //
int pitchSign = 1;
int rollSign = 1;
int gimbalTestPass = 0;
bool gimbalTestMode = false;

//////////
// INIT //
//////////

void setup() {
  initActuator();

  if (logging) {
    initLog();
  }

  if (systemFlag.programMode == systemFlag.SIM) {
    initSimulatorinterface();
  } else {
    Serial.begin(115200);
    initSensors();
    delay(2000);
    calibrateSensors();
  }

  // init prev values
  timePrev = micros();
}


//////////
// LOOP //
//////////

void loop() {
  unsigned long timeCur = micros(); // time now in micros ; 10^{-6}s
  ctrlData.dt = (timeCur - timePrev) * 1e-6; // dt in seconds
  timePrev = timeCur; // updates timePrev
  // NB! averag dt = 0.006s, but when logData() is run the system slows to dt = 0.015s for that loop

  readSensors();
  

 
  if (false) { // true for gimbal mechanical test
    gimbalTest();
  } else if (systemFlag.programMode == systemFlag.LAB) {
      testPhases();
  } else {
      flightPhases();
  }



  if (systemFlag.flightPhase == systemFlag.LAUNCHED && (millis() - timeIgnite) >= 1000) {
    // resets ignition 1 second after launch
    resetIgnition();
  }

  if (systemFlag.armed && !systemFlag.launchSignaled) {
    // makes a distinct noise when armed
    buzzer(armedSound);
    if (armedSound < 500) {
      armedSound += 1;
    } else {
      armedSound = 200;
    }
  }

  if ((timeCur - logTimePrev >= logInterval) && logging) {
    logTimePrev = timeCur;
    logData();
  }
}


///////////////
// FUNCTIONS //
///////////////


void calibrateSensors() {

  int intAmount = 750;

  Serial.println(" "); Serial.println("--- CALIBRATION STARTING ---"); Serial.println(" ");
  for (int i = 0; i < 1500; i++) {
    //buzzer(100);
    readImu();
    readPressure();
    readGps(gpsData);
    if (i>=intAmount) {
      sensorData.initPressure += sensorData.pressure; // integrates vals while calibrating
    }
    Serial.print(F("pitch calibrating: ")); Serial.println(sensorData.pitch);
    Serial.print(F("roll calibrating: ")); Serial.println(sensorData.roll);
    Serial.print(F("pressure calibrating: ")); Serial.println(sensorData.pressure);
    Serial.println(" ");
  } Serial.println("--- CALIBRATION FINISHED ---"); Serial.println(" ");

  sensorData.initPressure = sensorData.initPressure/intAmount; // sets average pressure while calibrating as refference pressure

  Serial.print(F("init pressure: ")); Serial.println(sensorData.initPressure);
  relativeAltitude(sensorData.initPressure);
  sensorData.altitude = calculateAltitude();
}


void readSensors() {
  // updates the measurement variables to the latest imu reading from sensor.ccp
  altitudePrev = sensorData.altitude;
  
  if (systemFlag.programMode == systemFlag.SIM) {
    // fetches the latest sim data
    readSimulator(simRead);

    sensorData.pressure = simRead[5];
    sensorData.altitude = relativeAltitude(simRead[5]);

    sensorData.pitch = simRead[1];
    sensorData.roll = simRead[2];

    sensorData.gyroZ = simRead[4];
    sensorData.gyroX = -simRead[3];

  } else {
    // fetches the latest sensor data
    readImu();
    readPressure();
    readGps(gpsData);
  
    sensorData.altitude = calculateAltitude();
  }
  if (false) {
    Serial.print(F("pitchMeasured: ")); Serial.println(sensorData.pitch);
    Serial.print(F("rollMeasured: ")); Serial.println(sensorData.roll);
    Serial.print(F("pitch gyro: ")); Serial.println(sensorData.gyroZ);
    Serial.print(F("roll gyro: ")); Serial.println(sensorData.gyroX);
    Serial.print(F("pressure (Bar): ")); Serial.println(sensorData.pressure);
    Serial.print(F("altitude (meters): ")); Serial.println(sensorData.altitude);
    Serial.print(F("vertical velocity: ")); Serial.println(sensorData.verticalVelocity);
  }
}


void flightPhases() {

  if (systemFlag.flightPhase == systemFlag.LAUNCHED || systemFlag.flightPhase == systemFlag.FLIGHT) {
    // rocket is ascending
    if ((millis() - timeIgnite) <= 1000) { // small pidVals during the first second
      ctrl(0.15, 0.06, 0.07, 0.3, 0.0);
    } else {
      ctrl(0.45, 0.15, 0.13, 0.3, 0.0); 
    }
    updateServos();

  }

  if (systemFlag.flightPhase == systemFlag.PREEFLIGHT) {
    // rocket has not yet launched
    reciever(100); // listening on LoRa at 10Hz

    if (!systemFlag.armed) { // sets the gimbal to neutral position if not armed
      ctrlData.gimbalPitchAngle = 0.0;
      ctrlData.gimbalRollAngle = 0.0;

    } else { // ensures correct angle at launch without integral buildup if armed
      ctrl(0.15, 0.0, 0.07, 0.3, 0.0);
    }
    
    if (systemFlag.launchSignaled && systemFlag.armed) {
      ignite();
      timeIgnite = millis();
      systemFlag.flightPhase = systemFlag.LAUNCHED;

    } else if (systemFlag.armSignaled && !systemFlag.armed) {
      armIgnition();
    }

    updateServos();  

  } else if (systemFlag.flightPhase == systemFlag.LAUNCHED && sensorData.altitude >= 10.0) {
    // rocket is in stable flight
    systemFlag.flightPhase = systemFlag.FLIGHT;
  
  } else if (systemFlag.flightPhase == systemFlag.FLIGHT && sensorData.altitude >= sensorData.altitudeMax) {
    // rocket is ascending
    sensorData.altitudeMax = sensorData.altitude;

  } else if (systemFlag.flightPhase == systemFlag.FLIGHT && (sensorData.altitude + 0.5) < sensorData.altitudeMax) {
    // rocket has reached its apogee and starts freefall
    systemFlag.flightPhase = systemFlag.APOGEE;

  } else if (systemFlag.flightPhase == systemFlag.APOGEE && (sensorData.altitudeMax - sensorData.altitude) > 0.3) {
    // rocket has fallen a certain distance from apogee and deploys parachute
    deployParachute(true);
    systemFlag.flightPhase = systemFlag.DESCENT;

  } else if (systemFlag.flightPhase == systemFlag.DESCENT) {
    // rocket is decending with parachute
    double altitudeDerivative = (sensorData.altitude - altitudePrev)/ctrlData.dt;
    if (abs(altitudeDerivative) < 0.001) {
      systemFlag.flightPhase = systemFlag.GROUND;
    }
    deployParachute(true);

  } else if (systemFlag.flightPhase == systemFlag.GROUND && logging) {
    // rocket has landed; start shutdown
    logData();
    logging = false;
  }

  if (false) {
    Serial.print(F("flightPhase: ")); Serial.println(systemFlag.flightPhase);
  }
}

void testPhases() {
  if (systemFlag.flightPhase == systemFlag.PREEFLIGHT) {
    reciever(100);

    if (systemFlag.launchSignaled && systemFlag.armed) {
      ignite();
      timeIgnite = millis();
      systemFlag.flightPhase = systemFlag.LAUNCHED;
    } else if (systemFlag.armSignaled && !systemFlag.armed) {
      armIgnition();
    } else if (systemFlag.armed) {
      ctrl(0.15, 0.00, 0.07, 0.3, 0.0);
      updateServos();
    }

    if (!systemFlag.armed) {
      ctrlData.gimbalPitchAngle = 0.0;
      ctrlData.gimbalRollAngle = 0.0;
      updateServos();
    }

  } else if (systemFlag.flightPhase == systemFlag.LAUNCHED) {
    if ((millis() - timeIgnite) >= 30000) { // 30 seconds after ignition, the system shuts off
      logging = false;
      systemFlag.flightPhase = systemFlag.GROUND;
      ctrlData.gimbalPitchAngle = 0.0;
      ctrlData.gimbalRollAngle = 0.0;
      updateServos();

    } else if ((millis() - timeIgnite) <= 1000) { // small pidVals during the first second
      ctrl(0.15, 0.06, 0.07, 0.3, 0.0);
      updateServos();
    } else {
      ctrl(0.45, 0.15, 0.13, 0.3, 0.0); 
      updateServos();
    }
  }

  if (false) {
    Serial.print(F("flightPhase: ")); Serial.println(systemFlag.flightPhase);
  }

}

void gimbalTest() {
  double step = 0.05;

  if (gimbalTestMode) {
    if (abs(ctrlData.gimbalPitchAngle) >= gimbalLim) {
      pitchSign *= -1;
      gimbalTestPass += 1;
    }
    ctrlData.gimbalPitchAngle += step*pitchSign;
  } else {
    if (abs(ctrlData.gimbalRollAngle) >= gimbalLim) {
      rollSign *= -1;
      gimbalTestPass += 1;
    }
  ctrlData.gimbalRollAngle += step*rollSign;
  }

  if ((gimbalTestPass >= 2 && abs(ctrlData.gimbalPitchAngle) <= 0.1 && gimbalTestMode) ||
      (gimbalTestPass >= 2 && abs(ctrlData.gimbalRollAngle) <= 0.1 && !gimbalTestMode)) {
    gimbalTestMode = !gimbalTestMode;
    gimbalTestPass = 0;
    ctrlData.gimbalPitchAngle = 0.0;
    ctrlData.gimbalRollAngle = 0.0;
  }

  updateServos();
}


void reciever(unsigned long interval) {
  // fetches LoRa messages at a set interval in milliseconds
  unsigned long now = millis();
  if (now - lastMessageTime >= interval) {
    lastMessageTime = now;
    receiveLoRaMessage();
  }
}