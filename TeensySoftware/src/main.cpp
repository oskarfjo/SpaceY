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

SdFat SD;
FsFile storedData;

bool const logging = false;
const unsigned long logInterval = 100; // ms
unsigned long logTimePrev = 0;

int logBufferCount = 0;
const int LOG_BUFFER_SIZE = 20;
char logBuffer[LOG_BUFFER_SIZE][128];

void logData();
void flushLogBuffer();
void readSensors();
void ctrl();
void updateServos();
void debugPrint();
void calibrateSensors();

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

DebugMode debugMode = OFF;
ProgramMode programMode = SIM;

// GIMBAL PARAMS //
const int gimbalLim = 7; // +- deg

double servoPitchAngle = 0.0, gimbalPitchAngle = 0.0; // deg
double servoRollAngle = 0.0, gimbalRollAngle = 0.0; // deg
double servoPitchSet = 0.0, servoRollSet = 0.0; // deg

// servo specs
const double maxServoChange = 0.3 * 1/(0.04/60); // 0.04 s for 60 deg : 1500*0.3 = maxDeg pr. s

// gear geometry
const double servoPitchRad = 7.7, gimbalPitchRad = 63.0; // mm
const double servoRollRad = 7.7, gimbalRollRad = 63.0; // mm


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

float pitchMeasured = 0.0, rollMeasured = 0.0; // orientation in deg
float deltaRollMeasured = 0.0, deltaPitchMeasured = 0.0; // angular velocity in deg/s


// CTRL //
double dt = 0.0; // dynamic dt value that addapts to system frequency

const double pitchSet = 0.0, rollSet = 90.0;

double filteredPitchDotError = 0.0, filteredRollDotError = 0.0;
double pitchErrorInt = 0.0, rollErrorInt = 0.0;

const double MAX_P_CONTRIBUTION = gimbalLim;
const double MAX_I_CONTRIBUTION = 0.5*gimbalLim;
const double MAX_D_CONTRIBUTION = 0.8*gimbalLim;

// store of values from last loop
unsigned long timePrev = 0;
double servoPitchAnglePrev = 0.0, servoRollAnglePrev = 0.0;
double pitchMeasuredPrev = 0.0, rollMeasuredPrev = 0.0;

// Parachute //

bool parachuteRelease = false;

// SIMULATOR PARAMS //
float simRead[12];
float simPub[2];


//////////
// INIT //
//////////

void setup() {

  if (logging) {
    while (!Serial) {}

    Serial.println("--- Initializing SD card ---");

    if (!SD.begin(10)) {
      Serial.println("SD initialization failed!");
      return;
    }

    Serial.println("SD initialized successfully");

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

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  if (programMode == SIM) {
    debugMode = OFF;
    initSimulatorinterface();
    delay(2000);
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
  
  if (flightPhase == LAUNCHED || flightPhase == FLIGHT) {
    ctrl();
    updateServos();
  }

  if (flightPhase == PREEFLIGHT && altitude >= 0.3) {
    flightPhase = LAUNCHED;
  }

  if (flightPhase == LAUNCHED && altitude >= 15.0) {
    flightPhase = FLIGHT;
  }

  if (flightPhase == FLIGHT && altitude >= altitudeMax) {
    altitudeMax = altitude;
  } else if (flightPhase == FLIGHT && (altitude + 1.0) < altitudeMax) {
    flightPhase = APOGEE;
  }

  if (flightPhase == APOGEE && (altitudeMax - altitude) > 5.0) {
    parachuteRelease = true;
    flightPhase = DESCENT;
  }

  if (flightPhase == DESCENT) {
    double altitudeDerivative = (altitude - altitudePrev)/dt; // should use imu data
    if (abs(altitudeDerivative) < 0.001) {
      flightPhase = GROUND;
    }
  }

  if (timeCur - logTimePrev >= logInterval && logging) {
    logTimePrev = timeCur;
    logData();
  }

  //Serial.print(F("flightPhase: ")); Serial.println(flightPhase);
  //Serial.print(F("parachuteRelease: ")); Serial.println(parachuteRelease);
  debugPrint();
}


///////////////
// FUNCTIONS //
///////////////


float relativeAltitude(float pressureMeasured) {
  if (!initAltitude || initPressure == 0.0) { // sets the starting pressure after the calibration run
    initPressure = pressureMeasured * 100; // converts bar to hPa
    initAltitude = true;
    return 0.0;
  }

  // function for relative altitude adapted from the bar.readAltitude(); function previously used in sensor.cpp
  float currentPressureHPa = pressureMeasured * 100;
  
  float relAltitude = 44330 * (1.0 - pow((currentPressureHPa / initPressure), 0.1903));
  
  return relAltitude;
}


double requiredForceEstimator(double currentAngle, double currentVelocity, double desiredAngle) {
  // Simplified physical model for andreas' required-force-estimation (pat pending) //
  
  // define constants
  const double calcTime = 0.39; // s : dt for the calculation to be realistic/achievable
  const double thrustMomentArm = 0.3; // m : distance from thruster to CM
  const double thrust = 12; // N : thrust magnitude
  const double inertia = 0.2; // moment of inertia (ESTIMATED)
  
  // calculate required angular acceleration
  // OLD - Using derivative: ((desiredAngle - currentAngle)/calcTime - currentVelocity) / (calcTime);
  // Using BeVeGeLsEsLiKnInG: x = x_0 + v_0*t + (1/2)*a*t^2
  // -> a = 2 * (x - x_0 - v_0*t) / t^2
  double angularAccel = 2.0 * (desiredAngle - currentAngle - currentVelocity * calcTime) / (calcTime * calcTime);
  
  // calculate required gimbal angle (alpha)
  // NB! model from simple rocket sim //
  // F_tot = T*l_T*sin(alpha) + F_D*l_D*sin(theta) //
  // theta[n+1] = F_tot * ((dt^2)/I) + 2*theta[n] - theta[n-1] //
  // theta[n+1] = setpoint //
  double tauRequired = inertia * angularAccel * dt/calcTime;
  double tauThrust = thrust * thrustMomentArm;
  double tauDrag = 0.0; // drag is rounded to 0 because it is at most 0.05*thrust
  double ratio = constrain(tauRequired / (tauThrust + tauDrag), -1, 1); // asin gives nan for val outside [-1, 1]
  double requiredGimbalAngle = asin(ratio) * 180/PI;

  // constrain to gimbal limits
  requiredGimbalAngle = constrain(requiredGimbalAngle, -gimbalLim, gimbalLim);

  if (false) { // set to true for estimator debugging, NB! also choose to estimate only roll, or only pitch
    Serial.print(F("currentAngle: ")); Serial.println(currentAngle);
    Serial.print(F("desiredAngle: ")); Serial.println(desiredAngle);
    Serial.print(F("currentVelocity: ")); Serial.println(currentVelocity);
    Serial.print(F("dt: ")); Serial.println(dt, 5);
    Serial.print(F("angularAccel: ")); Serial.println(angularAccel);
    Serial.print(F("tauRequired: ")); Serial.println(tauRequired);
    Serial.print(F("Ratio (unconstrained): ")); Serial.println(tauRequired/tauThrust);
    Serial.print(F("result - requiredGimbalAngle: ")); Serial.println(requiredGimbalAngle);
    Serial.println(" ");
  }

  return requiredGimbalAngle;
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
  
  Serial.println(" ");
  Serial.println("--- CALIBRATION STARTING ---");
  Serial.println(" ");

  for (int i = 0; i < 1500; i++) {
    readImu(imuData);
    readPressure(barData);
    readGps(gpsData);
    Serial.println(("Calibrating..."));
    Serial.print(F("pitchMeasured: ")); Serial.println(pitchMeasured);
    Serial.print(F("rollMeasured")); Serial.println(rollMeasured);
    Serial.print(F("pressure (Bar): ")); Serial.println(pressure);
    Serial.println(" ");
  }
  
  Serial.println("--- CALIBRATION FINISHED ---");
  Serial.println(" ");

  pressure = barData[0];
  altitude = relativeAltitude(pressure);
  
  //pitchSet = pitchMeasured, rollSet = rollMeasured;

  ctrl();
  updateServos();
}


void readSensors() {
  // updates the measurement variables to the latest imu reading from sensor.ccp
  pitchMeasuredPrev = pitchMeasured;
  rollMeasuredPrev = rollMeasured;
  altitudePrev = altitude;
  
  if (programMode == SIM) {
    // fetches the latest sim data
    readSimulator(imuData);

    pressure = 0;
    altitude = relativeAltitude(pressure);

    pitchMeasured = 0;
    rollMeasured = 0;

    deltaPitchMeasured = 0;
    deltaRollMeasured = 0;

  } else {
    // fetches the latest sensor data
    readImu(imuData);
    readPressure(barData);
    readGps(gpsData);
  
    double altitudeGain = 1.0;
    pressure = barData[0];
    double altitudeMeasured = relativeAltitude(pressure);

    altitude = altitudeMeasured * altitudeGain + (1-altitudeGain) * altitude; 

    pitchMeasured = imuData[1];
    rollMeasured = imuData[2];

    deltaPitchMeasured = -imuData[5];
    deltaRollMeasured = imuData[3];
  }
}


void ctrl() {
  
  // constants //
  double kp = 0.5;
  double kd = 0.35;
  double ki = 0.3;
  double dGain = 0.3;
  double reqGain = 0.2;

  // P //
  double pitchError = pitchSet - pitchMeasured;
  double rollError = rollSet - rollMeasured;

  // D //
  double pitchDotError = -deltaPitchMeasured;
  double rollDotError = -deltaRollMeasured;

  // D filter (lowpass)
  filteredPitchDotError = dGain * pitchDotError + (1-dGain) * filteredPitchDotError;
  filteredPitchDotError = constrain(filteredPitchDotError, -gimbalLim/kd, gimbalLim/kd);
  filteredRollDotError = dGain * rollDotError + (1-dGain) * filteredRollDotError;
  filteredRollDotError = constrain(filteredRollDotError, -gimbalLim/kd, gimbalLim/kd);

  // I - with windup //
  if (!(gimbalPitchAngle >= gimbalLim && pitchError > 0) && !(gimbalPitchAngle <= -gimbalLim && pitchError < 0)) {
  pitchErrorInt += dt * constrain(pitchError, -gimbalLim, gimbalLim); // integrates error, constrained to prevent large jumps
  pitchErrorInt = constrain(pitchErrorInt, -0.8*gimbalLim/ki, 0.8*gimbalLim/ki); // constrains the resulting value to 80% of gimbal limit
  }
  if (!(gimbalRollAngle >= gimbalLim && rollError > 0) && !(gimbalRollAngle <= -gimbalLim && rollError < 0)) {
  rollErrorInt += dt * constrain(rollError, -gimbalLim, gimbalLim);
  rollErrorInt = constrain(rollErrorInt, -0.8*gimbalLim/ki, 0.8*gimbalLim/ki);
  }
  // integral decays close to setpoint
  if (abs(pitchError) < 0.01) pitchErrorInt *= 0.999;
  if (abs(rollError) < 0.01) rollErrorInt *= 0.999;

  double pP = constrain(pitchError*kp, -MAX_P_CONTRIBUTION, MAX_P_CONTRIBUTION), rP = constrain(rollError*kp, -MAX_P_CONTRIBUTION, MAX_P_CONTRIBUTION);
  double pI = constrain(pitchErrorInt*ki, -MAX_I_CONTRIBUTION, MAX_I_CONTRIBUTION), rI = constrain(rollErrorInt*ki, -MAX_I_CONTRIBUTION, MAX_I_CONTRIBUTION);
  double pD = constrain(filteredPitchDotError*kd, -MAX_D_CONTRIBUTION, MAX_D_CONTRIBUTION), rD = constrain(filteredRollDotError*kd, -MAX_D_CONTRIBUTION, MAX_D_CONTRIBUTION);

  if (abs(pD) < 0.1) pD = 0.0;
  if (abs(rD) < 0.1) rD = 0.0;
  
  double uPitchEstimate = requiredForceEstimator(pitchMeasured, deltaPitchMeasured, pitchSet);
  double uRollEstimate = requiredForceEstimator(rollMeasured, deltaRollMeasured, rollSet);
  
  // PID
  double uPitch = pP + pI + pD;
  double uRoll = rP + rI + rD;

  // combines weighted calculated and estimated ctrl-inputs
  double pCTRL = uPitchEstimate*reqGain + (1-reqGain)*uPitch;
  double rCTRL = uRollEstimate*reqGain + (1-reqGain)*uRoll;

  // constrains the desired control to the gimbal limit
  gimbalPitchAngle = constrain(pCTRL, -gimbalLim, gimbalLim);
  gimbalRollAngle = constrain(rCTRL, -gimbalLim, gimbalLim);
  
  if (debugMode == CTRL) {
    Serial.print("Pitch: ");
    Serial.print(F("\tpP: ")); Serial.print(pP);
    Serial.print(F("\tpI: ")); Serial.print(pI);
    Serial.print(F("\tpD: ")); Serial.println(pD);
    Serial.println(" ");
    Serial.print(F("PIDp: ")); Serial.print(uPitch, 4);
    Serial.print(F("\tpEstimate: ")); Serial.println(uPitchEstimate, 4);
    Serial.println(" ");
    Serial.print("Roll: ");
    Serial.print(F("\trP: ")); Serial.print(rP);
    Serial.print(F("\trI: ")); Serial.print(rI);
    Serial.print(F("\trD: ")); Serial.println(rD);
    Serial.println(" ");
    Serial.print(F("PIDr: ")); Serial.print(uRoll, 4);
    Serial.print(F("\trEstimate: ")); Serial.println(uRollEstimate, 4);
    Serial.println(" ");
  }
}


void gimbalToServo() {
  double maxChange = maxServoChange * dt;

  // calculates servo setpoints from desired gimbal angle using physical geometry
  servoPitchAngle = - gimbalPitchAngle * gimbalPitchRad / servoPitchRad;
  servoRollAngle = - gimbalRollAngle * gimbalRollRad / servoRollRad;

  servoPitchAngle = constrain(servoPitchAngle, servoPitchAnglePrev - maxChange, servoPitchAnglePrev + maxChange);
  servoRollAngle = constrain(servoRollAngle, servoRollAnglePrev - maxChange, servoRollAnglePrev + maxChange);

  servoPitchAnglePrev = servoPitchAngle;
  servoRollAnglePrev = servoRollAngle;
}


void updateServos() {
  if (programMode == SIM) {
    simPub[0] = gimbalPitchAngle;
    simPub[1] = gimbalRollAngle;
    publishSimulator(simPub);
  } else {
    gimbalToServo();
    
    // Translates the servo setpoints to PWM signals. Servos idle at 90deg
    int pitchPulse = map(constrain(90 + servoPitchAngle, 0, 180), 0, 180, 500, 2500);
    int rollPulse = map(constrain(90 + servoRollAngle, 0, 180), 0, 180, 500, 2500);

    // mannualy sends the PWM signal to the servos 🤡🤡🤡
    digitalWrite(2, HIGH);
    delayMicroseconds(pitchPulse);
    digitalWrite(2, LOW);
    
    digitalWrite(3, HIGH);
    delayMicroseconds(rollPulse);
    digitalWrite(3, LOW);
  }
}


void debugPrint() {
  switch (debugMode) {
    // setpoint calculation data
    case CTRL:
      Serial.println(" ");
      Serial.print(F("dt: ")); Serial.println(dt, 5);
      Serial.println(" ");
      Serial.print(F("IMU pitch: ")); Serial.print(pitchMeasured);
      Serial.print(F("\tIMU roll: ")); Serial.println(rollMeasured);
      Serial.println("");
      Serial.print(F("Pitch set: ")); Serial.print(pitchSet);
      Serial.print(F("\tRoll set: ")); Serial.println(rollSet);
      Serial.println("");
      Serial.print(F("Pitch gimbal: ")); Serial.print(gimbalPitchAngle);
      Serial.print(F("\tPitch servo: ")); Serial.print(servoPitchAngle);
      Serial.print(F("\tRoll gimbal: ")); Serial.print(gimbalRollAngle);
      Serial.print(F("\tRoll servo: ")); Serial.println(servoRollAngle);
      Serial.println("");
      break;
    
    
    // sensor data
    // NB! Wrong names for indexing
    case SENSORS:
      Serial.print(F("pressure (Bar): ")); Serial.println(pressure);
      Serial.print(F("Altitude (Meters): ")); Serial.println(altitude);
      Serial.println(" ");
      Serial.print("Accel X: "); Serial.println(imuData[0]);
      Serial.print("Accel Y: "); Serial.println(imuData[1]);
      Serial.print("Accel Z: "); Serial.println(imuData[2]);
      Serial.println(" ");
      Serial.print("Gyro X: "); Serial.println(imuData[3]);
      Serial.print("Gyro Y: "); Serial.println(imuData[4]);
      Serial.print("Gyro Z: "); Serial.println(imuData[5]);
      Serial.println(" ");
      Serial.print("Magneto X: "); Serial.println(imuData[6]);
      Serial.print("Magneto Y: "); Serial.println(imuData[7]);
      Serial.print("Magneto Z: "); Serial.println(imuData[8]);
      Serial.println(" ");
      Serial.print("Roll: "); Serial.println(imuData[9]);
      Serial.print("Pitch: "); Serial.println(imuData[10]);
      Serial.print("Yaw: "); Serial.println(imuData[11]);
      Serial.println(" ");
      Serial.print("Hour: "); Serial.println(gpsData[0]);
      Serial.print("Min: "); Serial.println(gpsData[1]);
      Serial.print("Sec: "); Serial.println(gpsData[2]);
      Serial.println(" ");
      Serial.print("Lat: "); Serial.println(gpsData[3], 6);
      Serial.print("Lat_D: "); Serial.println(gpsData[4], 6);
      Serial.print("Lon: "); Serial.println(gpsData[5], 6);
      Serial.print("Lon_D: "); Serial.println(gpsData[6], 6);
      Serial.println(" ");
      Serial.print("Alt: "); Serial.println(gpsData[7], 6);
      Serial.print("Speed: "); Serial.println(gpsData[8]);
      Serial.print("Sat: "); Serial.println(gpsData[9]);
      Serial.println(" ");
      break;


    case OFF:

    default:
      break;
  }
}
