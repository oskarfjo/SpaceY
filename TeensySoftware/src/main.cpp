#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration.h>
#include "sensor.h"

void readSensors();
void updateTime();
void ctrl();
void updateServos();
void debugPrint();

enum DebugMode {
  OFF = 0,
  CTRL = 1,
  SENSORS = 2,
};

DebugMode debugMode = OFF;

// GIMBAL PARAMS //
const int gimbalLim = 7; // +- deg
double servoPitchAngle = 0.0, gimbalPitchAngle = 0.0; // deg
double servoRollAngle = 0.0, gimbalRollAngle = 0.0; // deg
double servoPitchSet = 0.0, servoRollSet = 0.0; // deg

const double maxServoChange = 0.3 * 1/(0.04/60); // 0,04 s for 60 deg : 1500*0.3 = maxDeg pr. s

/*
// for arm connection
const double servoPitchArm = 8.0, gimbalPitchArm = 37; // mm
const double servoRollArm = 7.0, gimbalRollArm = 35; // mm
*/

// for gear connection
const double servoPitchRad = 7.7, gimbalPitchRad = 63.0; // mm
const double servoRollRad = 7.7, gimbalRollRad = 63.0; // mm


// IMU PARAMS //
float imuData[12]; // array for imu data ; [heading, pitch, roll][0,0,0][0,0,0][0,0,0]
float barData[3]; // array for barometer data
float gpsData[10]; // array for gps data

float pitchMeasured = 0.0, rollMeasured = 0.0;
float deltaRollMeasured = 0.0, deltaPitchMeasured = 0.0;


// CTRL //
double dt = 0.0; // dynamic dt value that addapts to system frequency

double kp = 0.9;
double kd = 0.4;
double ki = 0.1;

double pitchSet = 0.0, rollSet = 90.0;
double pitchDotError = 0.0, rollDotError = 0.0;

double alpha = 0.2;
double filteredPitchDotError = 0.0;
double filteredRollDotError = 0.0;

double pitchErrorPrev = 0.0, rollErrorPrev = 0.0;
double pitchErrorInt = 0.0, rollErrorInt = 0.0;

// store of values from last loop
unsigned long timePrev = 0;
double servoPitchAnglePrev = 0.0, servoRollAnglePrev = 0.0;

//////////
// INIT //
//////////

void setup() {
  // begin serial communication at a baudrate of 115200Hz
  Serial.begin(115200);
  
  // sets the pins for the pitch and roll servos
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  initSensors();

  // delay to let the imu initialize
  delay(1000);

  timePrev = micros();

  // defines the angle setpoints
  //readImu(imuData);
  //pitchSet = pitchMeasured, rollSet = rollMeasured;
}


//////////
// LOOP //
//////////

void loop() {
  updateTime();

  readSensors();
  ctrl();
  updateServos();

  debugPrint();
}


///////////////
// FUNCTIONS //
///////////////

void updateTime() {
  unsigned long timeCur = micros(); // time now in micros ; 10^{-6}s
  dt = (timeCur - timePrev) * 1e-6; // dt in seconds
  timePrev = timeCur; // updates timePrev
}


void readSensors() {
  // fetches the latest sensor data
  readImu(imuData);
  readPressure(barData);
  readGps(gpsData);
  
  // updates the measurement variables to the latest imu reading from sensor.ccp
  rollMeasured = imuData[2];
  pitchMeasured = imuData[1];

  deltaRollMeasured = imuData[5];
  deltaPitchMeasured = imuData[3];
}


void ctrl() {

  // P //
  // calculates the current error
  double pitchError = pitchSet - pitchMeasured;
  double rollError = rollSet - rollMeasured;
  if (abs(pitchError) < 0.05) pitchError = 0;
  if (abs(rollError) < 0.05) rollError = 0;

  // D //
  if (dt < 0.001) {
    pitchDotError = 0.0;
    rollDotError = 0.0;
  } else if (false) { // uses error
  pitchDotError = (pitchError - pitchErrorPrev)/dt;
  rollDotError = (rollError - rollErrorPrev)/dt;
  if (abs(pitchDotError) < 0.5) pitchDotError = 0.0;
  if (abs(rollDotError) < 0.5) rollDotError = 0.0;
  } else { // uses measurement UNTESTED
    pitchDotError = - (pitchMeasured - pitchErrorPrev)/dt;
    rollDotError = - (rollMeasured - rollErrorPrev)/dt;
    if (abs(pitchDotError) < 0.5) pitchDotError = 0.0;
    if (abs(rollDotError) < 0.5) rollDotError = 0.0;
    }
  pitchErrorPrev = pitchMeasured; 
  rollErrorPrev = rollMeasured;

  // D filter (lowpass)
  filteredPitchDotError = alpha * pitchDotError + (1-alpha) * filteredPitchDotError;
  filteredPitchDotError = constrain(filteredPitchDotError, -gimbalLim/kd, gimbalLim/kd);
  filteredRollDotError = alpha * rollDotError + (1-alpha) * filteredRollDotError;
  filteredRollDotError = constrain(filteredRollDotError, -gimbalLim/kd, gimbalLim/kd);

  // I //
  if (abs(pitchError) < 3) {
  pitchErrorInt += dt * constrain(pitchError, -gimbalLim, gimbalLim); // integrates error, constrained to prevent large jumps
  pitchErrorInt = constrain(pitchErrorInt, -0.6*gimbalLim/ki, 0.6*gimbalLim/ki); // constrains the resulting value to 80% of gimbal limit
  } else {
    pitchErrorInt = 0.0;
  }
  if (abs(rollError) < 3) {
  rollErrorInt += dt * constrain(rollError, -gimbalLim, gimbalLim);
  rollErrorInt = constrain(rollErrorInt, -0.6*gimbalLim/ki, 0.6*gimbalLim/ki);
  } else {
    rollErrorInt = 0.0;
  }

  double pP = constrain(pitchError*kp, -gimbalLim, gimbalLim), rP = constrain(rollError*kp, -gimbalLim, gimbalLim);
  double pI = pitchErrorInt*ki, rI = rollErrorInt*ki;
  double pD = filteredPitchDotError*kd, rD = filteredRollDotError*kd;

  if (abs(pD < 0.15)) pD = 0.0;
  if (abs(rD < 0.15)) rD = 0.0;
  // PID
  double uPitch = pP + pI + pD;
  double uRoll = rP + rI + rD;

  // constrains the desired control to the gimbal limit
  gimbalPitchAngle = constrain(uPitch, -gimbalLim, gimbalLim);
  gimbalRollAngle = constrain(uRoll, -gimbalLim, gimbalLim);

  Serial.print("Pitch: ");
  Serial.print(F("\tpP: ")); Serial.print(pP);
  Serial.print(F("\tpI: ")); Serial.print(pI);
  Serial.print(F("\tpD: ")); Serial.print(pD);
  Serial.print(F("\tPIDp: ")); Serial.println(uPitch);

  
  Serial.print("Roll: ");
  Serial.print(F("\trP: ")); Serial.print(rP);
  Serial.print(F("\trI: ")); Serial.print(rI);
  Serial.print(F("\trD: ")); Serial.print(rD);
  Serial.print(F("\tPIDr: ")); Serial.println(uRoll);

  Serial.println(" ");
}


void gimbalToServo() {
  // calculates servo setpoints from desired gimbal angle using physical geometry

  /*
    // FOR ARMS //
  servoPitchAngle = asin(constrain((gimbalPitchArm * sin(gimbalPitchAngle * PI/180)) / servoPitchArm, -1, 1)) * (180/PI); // translating deg2rad in the sine function, and translating the answer rad2deg
  servoRollAngle = -asin(constrain((gimbalRollArm * sin(gimbalRollAngle * PI/180)) / servoRollArm, -1, 1)) * (180/PI); // - prefix because the servo is upside down
  */
  
    // FOR GEARS //
  servoPitchAngle = - gimbalPitchAngle * gimbalPitchRad / servoPitchRad;
  servoRollAngle = - gimbalRollAngle * gimbalRollRad / servoRollRad;

  double maxChange = maxServoChange * dt;

  servoPitchAngle = constrain(servoPitchAngle, servoPitchAnglePrev - maxChange, servoPitchAnglePrev + maxChange);
  servoRollAngle = constrain(servoRollAngle, servoRollAnglePrev - maxChange, servoRollAnglePrev + maxChange);

  servoPitchAnglePrev = servoPitchAngle;
  servoRollAnglePrev = servoRollAngle;
}


void updateServos() {
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


void debugPrint() {
  switch (debugMode) {
    // setpoint calculation data
    case CTRL: 
      Serial.print(F("Pitch set: ")); Serial.print(pitchSet);
      Serial.print(F("\tRoll set: ")); Serial.print(rollSet);
      Serial.print(F("\tPitch measured: ")); Serial.print(pitchMeasured);
      Serial.print(F("\tPitch gimbal: ")); Serial.print(gimbalPitchAngle);
      Serial.print(F("\tPitch servo: ")); Serial.print(servoPitchAngle);
      Serial.print(F("\tRoll measured: ")); Serial.print(rollMeasured);
      Serial.print(F("\tRoll gimbal: ")); Serial.print(gimbalRollAngle);
      Serial.print(F("\tRoll servo: ")); Serial.println(servoRollAngle);
      //Serial.print(F("Delta-pitch: ")); Serial.print(deltaPitchMeasured);
      //Serial.print(F("\tDelta-roll: ")); Serial.print(deltaRollMeasured);
      break;
    
    
    // sensor data
    // NB! Wrong names for indexing
    case SENSORS:
      Serial.print("Pressure: "); Serial.println(barData[0]);
      Serial.print("Temperature: "); Serial.println(barData[1]);
      Serial.print("Accel X: "); Serial.println(imuData[0]);
      Serial.print("Accel Y: "); Serial.println(imuData[1]);
      Serial.print("Accel Z: "); Serial.println(imuData[2]);
      Serial.print("Gyro X: "); Serial.println(imuData[3]);
      Serial.print("Gyro Y: "); Serial.println(imuData[4]);
      Serial.print("Gyro Z: "); Serial.println(imuData[5]);
      Serial.print("Magneto X: "); Serial.println(imuData[6]);
      Serial.print("Magneto Y: "); Serial.println(imuData[7]);
      Serial.print("Magneto Z: "); Serial.println(imuData[8]);
      Serial.print("Roll: "); Serial.println(imuData[9]);
      Serial.print("Pitch: "); Serial.println(imuData[10]);
      Serial.print("Yaw: "); Serial.println(imuData[11]);
      Serial.print("Hour: "); Serial.println(gpsData[0]);
      Serial.print("Min: "); Serial.println(gpsData[1]);
      Serial.print("Sec: "); Serial.println(gpsData[2]);
      Serial.print("Lat: "); Serial.println(gpsData[3], 6);
      Serial.print("Lat_D: "); Serial.println(gpsData[4], 6);
      Serial.print("Lon: "); Serial.println(gpsData[5], 6);
      Serial.print("Lon_D: "); Serial.println(gpsData[6], 6);
      Serial.print("Alt: "); Serial.println(gpsData[7], 6);
      Serial.print("Speed: "); Serial.println(gpsData[8]);
      Serial.print("Sat: "); Serial.println(gpsData[9]);
      break;


    case OFF:

    default:
      break;
  }
}