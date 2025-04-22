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
void reciever(unsigned long interval);

enum FlightPhase {
  PREEFLIGHT = 0,
  LAUNCHED = 1,
  FLIGHT = 2,
  APOGEE = 3,
  DESCENT = 4,
  GROUND = 5,
};

enum ProgramMode {
  LIVE = 0,
  LAB = 1,
  SIM = 2,
};


FlightPhase flightPhase = PREEFLIGHT;
ProgramMode programMode = LAB;

// coms bool
int armedSound = 200;
unsigned long lastMessageTime = 0;

// GIMBAL PARAMS  in deg//
extern const int gimbalLim = 7; // +-

// Sensor PARAMS //
float gpsData[10]; // array for gps data

float initPressure = 0.0;
float altitudePrev = 0.0;

// store of values from last loop
unsigned long timePrev = 0;
unsigned long timeIgnite = 0;

// Logging //
bool logging = false;
const unsigned long logInterval = 100; // ms
unsigned long logTimePrev = 0;

// SIMULATOR //
float simRead[12];


//////////
// INIT //
//////////

void setup() {
  initActuator();

  if (logging) {
    initLog();
  }

  if (programMode == SIM) {
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

  if (true) {
    if (programMode == LAB) {
      testPhases();
    } else {
      flightPhases();
    }
  }

  if (flightPhase == LAUNCHED && (millis() - timeIgnite) >= 1000) {
    resetIgnition();
  }

  if (systemFlag.armed) {
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

  int pressureIntAmount = 750;

  Serial.println(" "); Serial.println("--- CALIBRATION STARTING ---"); Serial.println(" ");
  for (int i = 0; i < 1500; i++) {
    buzzer(100);
    readImu();
    readPressure();
    readGps(gpsData);
    if (i>=pressureIntAmount) {
      initPressure += sensorData.pressure; // integrates pressure vals while calibrating
    }
    Serial.print(F("pitch calibrating: ")); Serial.println(sensorData.pitch);
    Serial.print(F("roll calibrating: ")); Serial.println(sensorData.roll);
    Serial.print(F("pressure calibrating: ")); Serial.println(sensorData.pressure);
    Serial.println(" ");
  } Serial.println("--- CALIBRATION FINISHED ---"); Serial.println(" ");
  initPressure = initPressure/pressureIntAmount; // sets average pressure while calibrating as refference pressure
  Serial.print(F("init pressure: ")); Serial.println(initPressure);
  sensorData.altitude = relativeAltitude(initPressure);
}


void readSensors() {
  // updates the measurement variables to the latest imu reading from sensor.ccp
  altitudePrev = sensorData.altitude;
  
  if (programMode == SIM) {
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
    Serial.print(F("pressure (Bar): ")); Serial.println(sensorData.pressure);
    Serial.print(F("altitude (meters): ")); Serial.println(sensorData.altitude);
  }
}


void flightPhases() {

  if (flightPhase == LAUNCHED || flightPhase == FLIGHT) {
    // rocket is ascending
    ctrl(0.5, 0.35, 0.0, 0.3, 0.0);
    updateServos();
  }

  if (flightPhase == PREEFLIGHT) {
    // rocket has not yet launched
    if (systemFlag.armed) { // ensures correct angle at lauch
      ctrl(0.5, 0.0, 0.0, 0.0, 0.0);
      updateServos();  
    }
    
    reciever(100); // reading LoRa at 10 Hz
    
    if (systemFlag.launchSignaled && systemFlag.armed) {
      ignite();
      timeIgnite = millis();
      flightPhase = LAUNCHED;

    } else if (systemFlag.armSignaled && !systemFlag.armed) {
      armIgnition();
      systemFlag.armed = true;
    }

  } else if (flightPhase == LAUNCHED && sensorData.altitude >= 10.0) {
    // rocket is in stable flight
    flightPhase = FLIGHT;
  
  } else if (flightPhase == FLIGHT && sensorData.altitude >= sensorData.altitudeMax) {
    // rocket is ascending
    sensorData.altitudeMax = sensorData.altitude;

  } else if (flightPhase == FLIGHT && (sensorData.altitude + 0.5) < sensorData.altitudeMax) {
    // rocket has reached its apogee and starts freefall
    flightPhase = APOGEE;

  } else if (flightPhase == APOGEE && (sensorData.altitudeMax - sensorData.altitude) > 0.3) {
    // rocket has fallen a certain distance from apogee and deploys parachute
    deployParachute(true);
    flightPhase = DESCENT;

  } else if (flightPhase == DESCENT) {
    // rocket is decending with parachute
    double altitudeDerivative = (sensorData.altitude - altitudePrev)/ctrlData.dt; // should use imu data
    if (abs(altitudeDerivative) < 0.001) {
      flightPhase = GROUND;
    }
    deployParachute(true);

  } else if (flightPhase == GROUND && logging) {
    // rocket has landed; start shutdown
    logData();
    logging = false;
  }

  if (false) {
    Serial.print(F("flightPhase: ")); Serial.println(flightPhase);
  }
}

void testPhases() {
  if (flightPhase == PREEFLIGHT) {
    reciever(100);
    if (systemFlag.launchSignaled && systemFlag.armed) {
      ignite();
      timeIgnite = millis();
      flightPhase = LAUNCHED;
    
    } else if (systemFlag.armSignaled && !systemFlag.armed) {
      armIgnition();
      systemFlag.armed = true;
    
    }
  } else if (flightPhase == LAUNCHED || flightPhase == FLIGHT) {
    // flight phase bypass
    ctrl(0.5, 0.0, 0.0, 1.0, 0.0);
    updateServos();

  }
}


void reciever(unsigned long interval) {
  unsigned long now = millis();
  if (now - lastMessageTime >= interval) {
    lastMessageTime = now;
    receiveLoRaMessage();
  }
}