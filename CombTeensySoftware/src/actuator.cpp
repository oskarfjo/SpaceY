#include <Arduino.h>
#include <wiring.h>
#include "actuator.h"
#include "SimulatorInterface.h"
#include "flightData.h"

extern float simRead[12];
float simPub[4];

double servoPitchAnglePrev = 0.0;
double servoRollAnglePrev = 0.0;

int parachutePin = 1;
int pitchPin = 2;
int rollPin = 3;

void initActuator(){
    pinMode(pitchPin, OUTPUT);
    pinMode(rollPin, OUTPUT);
    pinMode(parachutePin, OUTPUT);
    
    pinMode(24, OUTPUT); // Ignitor 1
    pinMode(25, OUTPUT); // Ignitor 2
    pinMode(26, OUTPUT); // Ignitor 3
    pinMode(27, OUTPUT); // Ignitor 4
    pinMode(28, OUTPUT); // Arming ignition
    digitalWrite(28, LOW);
    digitalWrite(24, LOW);
    digitalWrite(25, LOW);
    digitalWrite(26, LOW);
    digitalWrite(27, LOW);

    pinMode(29, OUTPUT); // Buzzer
}

// Funksjon for fallskjermsystem
void deployParachute(bool release){
if (simData.simMode) {
  simPub[2] = 1;
  publishSimulator(simPub, simRead);

  } else {
    double parachuteLockAngle = 90; // deg
    double parachuteOpenAngle = 180; // deg
    
    if (!release) {
        digitalWrite(parachutePin, HIGH);
        delayMicroseconds(map(parachuteLockAngle, 0, 180, 500, 2300));
        digitalWrite(parachutePin, LOW);
      } else {
        digitalWrite(parachutePin, HIGH);
        delayMicroseconds(map(parachuteOpenAngle, 0, 180, 500, 2300));
        digitalWrite(parachutePin, LOW);
      }
    
      if (false) {
        Serial.println(release);
    }
  }
}

// Funksjon for å arme ignition
void armIgnition(){
    digitalWrite(28, HIGH);
    systemFlag.armed = true;
}

// Funksjon for å antenne rakett motorer
void ignite(){ 
if (simData.simMode) {
  simPub[3] = 1;
  publishSimulator(simPub, simRead);

  } else {
    digitalWrite(24, HIGH);
    digitalWrite(25, HIGH);
    digitalWrite(26, HIGH);
    digitalWrite(27, HIGH);
  }
}

// Funksjon for å resete ignition system
void resetIgnition(){
if (simData.simMode) {
    simPub[3] = 0;
    publishSimulator(simPub, simRead);

  } else {
    digitalWrite(28, LOW);
    digitalWrite(24, LOW);
    digitalWrite(25, LOW);
    digitalWrite(26, LOW);
    digitalWrite(27, LOW);
  }
}

void disarmIgnition() {
  digitalWrite(28, LOW);
  systemFlag.armed = false;
}

// Funksjon for å lage buzzer lyder
void buzzer(int freq){
    digitalWrite(29, HIGH);
    delayMicroseconds(freq);
    digitalWrite(29, LOW);
}


void updateServos() {
    if (simData.simMode) {
      simPub[0] = ctrlData.gimbalPitchAngle;
      simPub[1] = ctrlData.gimbalRollAngle;
      publishSimulator(simPub, simRead);

    } else {
      // servo specs
      const double maxServoChange = 0.3 * (60/0.04); // 0.04 s for 60 deg : 1500*0.3 = maxDeg pr. s

      // gear geometry in mm
      const double servoPitchRad = 7.7;
      const double gimbalPitchRad = 63.0;
      const double servoRollRad = 7.7;
      const double gimbalRollRad = 63.0;

      double maxChange = maxServoChange * 0.006;

      // calculates servo setpoints from desired gimbal angle using physical geometry
      double servoPitchAngle = - ctrlData.gimbalPitchAngle * gimbalPitchRad / servoPitchRad;
      double servoRollAngle = - ctrlData.gimbalRollAngle * gimbalRollRad / servoRollRad;

      servoPitchAngle = constrain(servoPitchAngle, servoPitchAnglePrev - maxChange, servoPitchAnglePrev + maxChange);
      servoRollAngle = constrain(servoRollAngle, servoRollAnglePrev - maxChange, servoRollAnglePrev + maxChange);

      servoPitchAnglePrev = servoPitchAngle;
      servoRollAnglePrev = servoRollAngle;

      Serial.print(F("servo pitch: ")); Serial.println(servoPitchAngle);
      Serial.print(F("servo roll: ")); Serial.println(servoRollAngle);

      // Translates the servo setpoints to PWM signals. Servos idle at 90deg
      int pitchPulse = map(constrain(90 - servoPitchAngle, 0, 180), 0, 180, 500, 2500);
      int rollPulse = map(constrain(90 + servoRollAngle, 0, 180), 0, 180, 500, 2500);
  
      // mannualy sends the PWM signal to the servos
      digitalWrite(pitchPin, HIGH);
      delayMicroseconds(pitchPulse);
      digitalWrite(pitchPin, LOW);
      
      digitalWrite(rollPin, HIGH);
      delayMicroseconds(rollPulse);
      digitalWrite(rollPin, LOW);
    }
  }