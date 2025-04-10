#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration.h>
#include "sensor.h"
#include "SimulatorInterface.h"
#include <FS.h>
#include <SdFat.h>
#include "rocketUtils.h"
#include "regulator.h"
#include "actuator.h"

SdFat SD;
FsFile storedData;

bool logging = false;
const unsigned long logInterval = 100; // ms
unsigned long logTimePrev = 0;

int logBufferCount = 0;
const int LOG_BUFFER_SIZE = 20;
char logBuffer[LOG_BUFFER_SIZE][128];

void logData();
void flushLogBuffer();
void readSensors();
void calibrateSensors();
void flightPhases();
void initLog();
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

enum DebugMode {
  OFF = 0,
  CTRL = 1,
  SENSORS = 2,
  LOGGING = 3,
};

enum ProgramMode {
  LIVE = 0,
  LAB = 1,
  SIM = 2,
};


FlightPhase flightPhase = PREEFLIGHT;

DebugMode debugMode = CTRL;
ProgramMode programMode = LAB;

// coms bool
bool launchSignalReceived = false;
bool armed = false;
int armedSound = 200;
unsigned long lastMessageTime = 0;

// GIMBAL PARAMS  in deg//
extern const int gimbalLim = 7; // +-

double servoPitchAngle = 0.0;
double gimbalPitchAngle = 0.0;
double servoRollAngle = 0.0;
double gimbalRollAngle = 0.0;
double servoPitchSet = 0.0;
double servoRollSet = 0.0;

// Sensor PARAMS //
float imuData[12]; // array for imu data ; [heading, pitch, roll][0,0,0][0,0,0][0,0,0]
float barData[3]; // array for barometer data
float gpsData[10]; // array for gps data

bool initAltitude = false;
float initPressure = 0.0;
float pressure = 0.0;
float altitude = 0.0;
float altitudeMax = 0.0;
float altitudePrev = 0.0;

extern bool launchSignaled;
extern bool armSignaled;

// orientation in deg
float pitchMeasured = 0.0;
float rollMeasured = 0.0;

// angular velocity in deg/s
float deltaRollMeasured = 0.0;
float deltaPitchMeasured = 0.0;


// CTRL //
double dt = 0.0; // dynamic dt value that addapts to system frequency

extern const double pitchSet = 0.0;
extern const double rollSet = 90.0;

// store of values from last loop
unsigned long timePrev = 0;
double pitchMeasuredPrev = 0.0;
double rollMeasuredPrev = 0.0;


// SIMULATOR PARAMS //
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
    debugMode = OFF;
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
  dt = (timeCur - timePrev) * 1e-6; // dt in seconds
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

  if (armed) {
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

void initLog() {
  while (!Serial) {}

  Serial.println("--- Initializing SD card ---");

  if (!SD.begin(10)) {
    Serial.println("SD initialization failed!");
    return;
  }

  Serial.println("SD initialized successfully");

  if (SD.exists("data_log.csv")) {
    Serial.println("Existing data_log.csv found - removing it");
    SD.remove("data_log.csv");
  }

  storedData = SD.open("data_log.csv", O_WRITE | O_CREAT | O_APPEND);

  if (!storedData) {
    Serial.println("Failed to open log file!");
    return;
  }
  if (storedData.fileSize() == 0) {
    storedData.println("Time, dt, altitude, pitchMeasured, rollMeasured, pitchGimbal, rollGimbal, pitchSet, rollSet, pitchError, rollError"); // headers at the top csv
    storedData.flush();
  }
}


void logData() {
  if (!storedData) {
    Serial.println("Log file not open!");
    return;
  }

  double pitchError = pitchSet - pitchMeasured;
  double rollError = rollSet - rollMeasured;

  snprintf(logBuffer[logBufferCount], 128,
           "%lu,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
           millis(), dt, altitude, pitchMeasured, rollMeasured, gimbalPitchAngle,
           gimbalRollAngle, pitchSet, rollSet, pitchError, rollError);

  logBufferCount++;

  if (logBufferCount >= LOG_BUFFER_SIZE) {
    flushLogBuffer();
  }
  if (debugMode == LOGGING) {
    Serial.println("data buffered");
  }
}

void flushLogBuffer() {
  if (!storedData) {
    Serial.println("Log file not open!");
    return;
  }
  for (int i = 0; i < logBufferCount; i++) {
    storedData.println(logBuffer[i]);
  }
  storedData.flush();
  if (debugMode == LOGGING) {
    Serial.println("buffer flushed to SD");
  }
  logBufferCount = 0;
}


void calibrateSensors() {
  
  Serial.println(" "); Serial.println("--- CALIBRATION STARTING ---"); Serial.println(" ");
  for (int i = 0; i < 1500; i++) {
    buzzer(100);
    readImu(imuData);
    readPressure(barData);
    readGps(gpsData);
    Serial.print(F("pitch calibrating: ")); Serial.println(imuData[1]);
    Serial.print(F("roll calibrating: ")); Serial.println(imuData[2]);
    Serial.print(F("pressure calibrating: ")); Serial.println(barData[0]);
    Serial.println(" ");
  } Serial.println("--- CALIBRATION FINISHED ---"); Serial.println(" ");

  pressure = barData[0];

  altitude = calculateAltitude(pressure, imuData);
  
  //pitchSet = pitchMeasured, rollSet = rollMeasured;
}


void readSensors() {
  // updates the measurement variables to the latest imu reading from sensor.ccp
  pitchMeasuredPrev = pitchMeasured;
  rollMeasuredPrev = rollMeasured;
  altitudePrev = altitude;
  
  if (programMode == SIM) {
    // fetches the latest sim data
    readSimulator(simRead);

    pressure = simRead[5];
    //altitude = relativeAltitude(pressure);

    pitchMeasured = simRead[1];
    rollMeasured = simRead[2];

    deltaPitchMeasured = simRead[4];
    deltaRollMeasured = -simRead[3];

  } else {
    // fetches the latest sensor data
    readImu(imuData);
    readPressure(barData);
    readGps(gpsData);
  
    pressure = barData[0];
    //altitude = relativeAltitude(pressure);

    pitchMeasured = imuData[1];
    rollMeasured = imuData[2];

    deltaPitchMeasured = -imuData[5];
    deltaRollMeasured = imuData[3];
  }
  if (debugMode == SENSORS) {
    Serial.print(F("pitchMeasured: ")); Serial.println(pitchMeasured);
    Serial.print(F("rollMeasured: ")); Serial.println(rollMeasured);
    Serial.print(F("pressure (Bar): ")); Serial.println(pressure);
    Serial.print(F("altitude (meters): ")); Serial.println(altitude);
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
    if (armed) { // ensures correct angle at lauch
      ctrl(0.5, 0.0, 0.0, 0.0, 0.0);
      updateServos();  
    }

    reciever(100); // reading LoRa at 10 Hz

    if (launchSignaled && armed) {
      ignite();
      flightPhase = LAUNCHED;
    } else if (armSignaled && !armed) {
      armIgnition();
      armed = true;
    }

  } else if (flightPhase == PREEFLIGHT && altitude >= 0.2 && launchSignaled) {
    // rocket has just launched
    flightPhase = LAUNCHED;
    resetIgnition();

  } else if (flightPhase == LAUNCHED && altitude >= 1.0) {
    // rocket is in stable flight
    flightPhase = FLIGHT;
  
  } else if (flightPhase == FLIGHT && altitude >= altitudeMax) {
    // rocket is ascending
    altitudeMax = altitude;

  } else if (flightPhase == FLIGHT && (altitude + 0.5) < altitudeMax) {
    // rocket has reached its apogee and starts freefall
    flightPhase = APOGEE;

  } else if (flightPhase == APOGEE && (altitudeMax - altitude) > 0.3) {
    // rocket has fallen a certain distance from apogee and deploys parachute
    deployParachute(true);
    flightPhase = DESCENT;

  } else if (flightPhase == DESCENT) {
    // rocket is decending with parachute
    double altitudeDerivative = (altitude - altitudePrev)/dt; // should use imu data
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
    if (launchSignaled && armed) {
      ignite();
      flightPhase = LAUNCHED;
    } else if (armSignaled && !armed) {
      armIgnition();
      armed = true;
    }
  } else if (flightPhase == LAUNCHED || flightPhase == FLIGHT) {
    // flight phase bypass
    ctrl(0.5, 0.35, 0.3, 0.3, 0.0);
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