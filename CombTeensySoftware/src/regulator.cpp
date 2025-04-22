#include "regulator.h"
#include "flightData.h"
#include <Arduino.h>
#include <wiring.h>

extern const int gimbalLim;

const double MAX_P_CONTRIBUTION = gimbalLim;
const double MAX_I_CONTRIBUTION = 0.5*gimbalLim;
const double MAX_D_CONTRIBUTION = 0.8*gimbalLim;

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
    double tauRequired = inertia * angularAccel * ctrlData.dt/calcTime;
    double tauThrust = thrust * thrustMomentArm;
    double tauDrag = 0.0; // i rounded drag to 0 because it is at most 0.05*thrust
    double ratio = constrain(tauRequired / (tauThrust + tauDrag), -1, 1); // asin gives nan for val outside [-1, 1]
    double requiredGimbalAngle = asin(ratio) * 180/PI;

    // constrain to gimbal limits
    requiredGimbalAngle = constrain(requiredGimbalAngle, -gimbalLim, gimbalLim);

    if (false) { // set to true for estimator debugging, NB! also choose to estimate only roll, or only pitch
        Serial.print(F("currentAngle: ")); Serial.println(currentAngle);
        Serial.print(F("desiredAngle: ")); Serial.println(desiredAngle);
        Serial.print(F("currentVelocity: ")); Serial.println(currentVelocity);
        Serial.print(F("dt: ")); Serial.println(ctrlData.dt, 5);
        Serial.print(F("angularAccel: ")); Serial.println(angularAccel);
        Serial.print(F("tauRequired: ")); Serial.println(tauRequired);
        Serial.print(F("Ratio (unconstrained): ")); Serial.println(tauRequired/tauThrust);
        Serial.print(F("result - requiredGimbalAngle: ")); Serial.println(requiredGimbalAngle);
        Serial.println(" ");
    }

    return requiredGimbalAngle;
}


void ctrl(double kp, double ki, double kd, double dGain, double reqGain) {
  
    // constants //
    /*
    ctrl(0.5, 0.35, 0.3, 0.3, 0.0);
    double kp = 2;//0.5;
    double kd = 0; //0.35;
    double ki = 0;//0.3;
    double dGain = 0.3;
    double reqGain = 0.0;
    */

    // P //
    double pitchError = ctrlData.pitchSet - sensorData.pitch;
    double rollError = ctrlData.rollSet - sensorData.roll;

    ctrlData.pitchError = pitchError;
    ctrlData.rollError = rollError;

    // D //
    double pitchDotError = -sensorData.gyroZ;
    double rollDotError = -sensorData.gyroX;
  
    // D filter (lowpass)
    ctrlData.filteredPitchDotError = dGain * pitchDotError + (1-dGain) * ctrlData.filteredPitchDotError;
    ctrlData.filteredPitchDotError = constrain(ctrlData.filteredPitchDotError, -gimbalLim/kd, gimbalLim/kd);
    ctrlData.filteredRollDotError = dGain * rollDotError + (1-dGain) * ctrlData.filteredRollDotError;
    ctrlData.filteredRollDotError = constrain(ctrlData.filteredRollDotError, -gimbalLim/kd, gimbalLim/kd);
  
    // I - with windup //
    if (!(ctrlData.gimbalPitchAngle >= gimbalLim && pitchError > 0) && !(ctrlData.gimbalPitchAngle <= -gimbalLim && pitchError < 0)) {
      ctrlData.pitchErrorInt += ctrlData.dt * constrain(pitchError, -gimbalLim, gimbalLim); // integrates error, constrained to prevent large jumps
      ctrlData.pitchErrorInt = constrain(ctrlData.pitchErrorInt, -0.8*gimbalLim/ki, 0.8*gimbalLim/ki); // constrains the resulting value to 80% of gimbal limit
    }
    if (!(ctrlData.gimbalRollAngle >= gimbalLim && rollError > 0) && !(ctrlData.gimbalRollAngle <= -gimbalLim && rollError < 0)) {
      ctrlData.rollErrorInt += ctrlData.dt * constrain(rollError, -gimbalLim, gimbalLim);
      ctrlData.rollErrorInt = constrain(ctrlData.rollErrorInt, -0.8*gimbalLim/ki, 0.8*gimbalLim/ki);
    }
    // integral decays close to setpoint
    if (abs(pitchError) < 0.01) ctrlData.pitchErrorInt *= 0.999;
    if (abs(rollError) < 0.01) ctrlData.rollErrorInt *= 0.999;
  
    double pP = constrain(pitchError*kp, -MAX_P_CONTRIBUTION, MAX_P_CONTRIBUTION), rP = constrain(rollError*kp, -MAX_P_CONTRIBUTION, MAX_P_CONTRIBUTION);
    double pI = constrain(ctrlData.pitchErrorInt*ki, -MAX_I_CONTRIBUTION, MAX_I_CONTRIBUTION), rI = constrain(ctrlData.rollErrorInt*ki, -MAX_I_CONTRIBUTION, MAX_I_CONTRIBUTION);
    double pD = constrain(ctrlData.filteredPitchDotError*kd, -MAX_D_CONTRIBUTION, MAX_D_CONTRIBUTION), rD = constrain(ctrlData.filteredRollDotError*kd, -MAX_D_CONTRIBUTION, MAX_D_CONTRIBUTION);
  
    if (abs(pD) < 0.1) pD = 0.0;
    if (abs(rD) < 0.1) rD = 0.0;
    
    double uPitchEstimate = requiredForceEstimator(sensorData.pitch, sensorData.gyroZ, ctrlData.pitchSet);
    double uRollEstimate = requiredForceEstimator(sensorData.roll, sensorData.gyroX, ctrlData.rollSet);
    
    // PID
    double uPitch = pP + pI + pD;
    double uRoll = rP + rI + rD;
  
    // combines weighted calculated and estimated ctrl-inputs
    double pCTRL = uPitchEstimate*reqGain + (1-reqGain)*uPitch;
    double rCTRL = uRollEstimate*reqGain + (1-reqGain)*uRoll;
  
    // constrains the desired control to the gimbal limit
    ctrlData.gimbalPitchAngle = constrain(pCTRL, -gimbalLim, gimbalLim);
    ctrlData.gimbalRollAngle = constrain(rCTRL, -gimbalLim, gimbalLim);
    
    if (true) {
      Serial.print("Pitch: ");
      Serial.print(F("pitch IMU: ")); Serial.println(sensorData.pitch);
      Serial.print(F("pitch error: ")); Serial.println(pitchError);
      Serial.print(F("\tpP: ")); Serial.print(pP);
      Serial.print(F("\tpI: ")); Serial.print(pI);
      Serial.print(F("\tpD: ")); Serial.println(pD);
      Serial.println(" ");
      Serial.print(F("PIDp: ")); Serial.print(uPitch, 4);
      Serial.print(F("\tpEstimate: ")); Serial.println(uPitchEstimate, 4);
      Serial.println(" ");
      Serial.print("Roll: ");
      Serial.print(F("roll IMU: ")); Serial.println(sensorData.roll);
      Serial.print(F("pitch error: ")); Serial.println(pitchError);
      Serial.print(F("\trP: ")); Serial.print(rP);
      Serial.print(F("\trI: ")); Serial.print(rI);
      Serial.print(F("\trD: ")); Serial.println(rD);
      Serial.println(" ");
      Serial.print(F("PIDr: ")); Serial.print(uRoll, 4);
      Serial.print(F("\trEstimate: ")); Serial.println(uRollEstimate, 4);
      Serial.println(" ");
    }
  }