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
  PWM = 3
};

DebugMode debugMode = OFF;

// GIMBAL PARAMS //
const int gimbalLim = 8; // +- deg
double servoPitchAngle = 0.0, gimbalPitchAngle = 0.0; // deg
double servoRollAngle = 0.0, gimbalRollAngle = 0.0; // deg
double servoPitchSet = 0.0, servoRollSet = 0.0; // deg

// for arm connection
const double servoPitchArm = 8.0, gimbalPitchArm = 37; // mm
const double servoRollArm = 7.0, gimbalRollArm = 35; // mm

/*
// for gear connection
const double servoPitchRad = 1.0, gimbalPitchRad = 1.0; // mm
const double servoRollRad = 1.0, gimbalRollRad = 1.0; // mm
*/

// IMU PARAMS //
float imuData[12]; // array for imu data ; [heading, pitch, roll][0,0,0][0,0,0][0,0,0]
float barData[3]; // array for barometer data
float gpsData[10]; // array for gps data

double pitchMeasured = 0.0, rollMeasured = 0.0;



// CTRL //
double dt = 0.0; // dynamic dt value that addapts to system frequency

double kp = 1.3;
double kd = 0.7;

const double pitchSet = 0.0, rollSet = 90.0;

// store of values from last loop
double pitchErrorPrev = 0.0;
double rollErrorPrev = 0.0;
unsigned long timePrev = 0;


// PWM GENERATOR //
const int freq = 50;


//////////
// INIT //
//////////

void setup() {
  // begin serial communication at a baudrate of 115200Hz
  Serial.begin(115200);
  
  // sets the pins for the pitch and roll servos
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  // delay to let the imu initialize
  delay(1000);

  // sets the prev values for the first loop
  readSensors();
  pitchErrorPrev = pitchSet - pitchMeasured;
  rollErrorPrev = rollSet - rollMeasured;
  timePrev = micros();

  /*
  // defines the angle setpoints
  readImu(imuData);
  pitchSet = imuData[1], rollSet = imuData[2];
  */
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
}


void ctrl() {
  // calculates the current error
  double rollError = rollSet - rollMeasured;
  double pitchError = pitchSet - pitchMeasured;

  // calculates the rate of change for the error
  // andreas should see if sensor data could be used directly
  double pitchDotError = (pitchError - pitchErrorPrev) / dt;
  double rollDotError = (rollError - rollErrorPrev) / dt;

  // PD controller for the gimbal. u = Ctrl input
  double uPitch = pitchError*kp + pitchDotError*kd;
  double uRoll = rollError*kp + rollDotError*kd;

  // constrains the desired control to the gimbal limit
  gimbalPitchAngle = constrain(uPitch, -gimbalLim, gimbalLim);
  gimbalRollAngle = constrain(uRoll, -gimbalLim, gimbalLim);

  // updates error prev
  pitchErrorPrev = pitchError;
  rollErrorPrev = rollError;
}


void gimbalToServo() {
  // calculates servo setpoints from desired gimbal angle using physical geometry

  // FOR ARMS //
  servoPitchAngle = asin(constrain((gimbalPitchArm * sin(gimbalPitchAngle * PI/180)) / servoPitchArm, -1, 1)) * (180/PI); // translating deg2rad in the sine function, and translating the answer rad2deg
  servoRollAngle = -asin(constrain((gimbalRollArm * sin(gimbalRollAngle * PI/180)) / servoRollArm, -1, 1)) * (180/PI); // - prefix because the servo is upside down

  /*
  // FOR GEARS //
  servoPitchAngle = - gimbalPitchAngle * gimbalPitchRad / servoPitchRad;
  servoRollAngle = - gimbalRollAngle * gimbalRollRad / servoRollRad;
  */
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
      break;
    
    
    // sensor data
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
    

    // pulse data
    case PWM: 
      int pitchPulse = map(constrain(90 + servoPitchAngle, 0, 180), 0, 180, 500, 2500);
      int rollPulse = map(constrain(90 + servoRollAngle, 0, 180), 0, 180, 500, 2500);  
      Serial.print(F("Pitch pulse: ")); Serial.print(pitchPulse);
      Serial.print(F("\tRoll pulse: ")); Serial.print(rollPulse);
      break;

    case OFF:

    default:
      break;
  }
}