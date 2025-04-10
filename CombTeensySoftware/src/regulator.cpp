#include "regulator.h"
#include <Arduino.h>
#include <wiring.h>

extern const int gimbalLim;

extern double pitchSet;
extern double rollSet;
extern float pitchMeasured;
extern float rollMeasured;
extern float deltaPitchMeasured;
extern float deltaRollMeasured;
extern double gimbalPitchAngle;
extern double gimbalRollAngle;
extern double dt;

const double MAX_P_CONTRIBUTION = gimbalLim;
const double MAX_I_CONTRIBUTION = 0.5*gimbalLim;
const double MAX_D_CONTRIBUTION = 0.8*gimbalLim;

double filteredPitchDotError = 0.0;
double filteredRollDotError = 0.0;
double pitchErrorInt = 0.0;
double rollErrorInt = 0.0;

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
    double tauDrag = 0.0; // i rounded drag to 0 because it is at most 0.05*thrust
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
    
    if (false) {
      Serial.print("Pitch: ");
      Serial.print(F("pitch IMU: ")); Serial.println(pitchMeasured);
      Serial.print(F("pitch error: ")); Serial.println(pitchError);
      Serial.print(F("\tpP: ")); Serial.print(pP);
      Serial.print(F("\tpI: ")); Serial.print(pI);
      Serial.print(F("\tpD: ")); Serial.println(pD);
      Serial.println(" ");
      Serial.print(F("PIDp: ")); Serial.print(uPitch, 4);
      Serial.print(F("\tpEstimate: ")); Serial.println(uPitchEstimate, 4);
      Serial.println(" ");
      Serial.print("Roll: ");
      Serial.print(F("roll IMU: ")); Serial.println(rollMeasured);
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