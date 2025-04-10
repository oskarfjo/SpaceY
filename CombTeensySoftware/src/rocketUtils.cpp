// RocketUtils.cpp - Implementation of utility functions
#include "rocketUtils.h"
#include <Arduino.h>

extern bool initAltitude;
extern float initPressure;
extern float altitude;

float fusedAltitude = 0;
float lastFusedAltitude = 0;
float verticalVelocity = 0;
unsigned long lastAltitudeTime = 0;


float relativeAltitude(float pressureMeasured) {
    if (!initAltitude || initPressure == 0.0) { // sets the starting pressure
      initPressure = pressureMeasured * 100; // converts bar to hPa
      initAltitude = true;
      return 0.0;
    }
  
    // function for relative altitude adapted from the bar.readAltitude(); function previously used in sensor.cpp
    float currentPressureHPa = pressureMeasured * 100;
    float relAltitude = 44330 * (1.0 - pow((currentPressureHPa / initPressure), 0.1903));
    return relAltitude;
  }


double calculateAltitude(float pressureMeasured, float imuData[12]) {
    
    unsigned long now = millis();
    unsigned long dt = now - lastAltitudeTime;

    // Get barometric altitude
    float baroAlt = relativeAltitude(pressureMeasured);

    // Get vertical acceleration (in the world frame)
    // We need to transform the acceleration from sensor frame to world frame using the orientation
    float sinRoll = sin(imuData[2] * PI / 180.0);
    float cosRoll = cos(imuData[2] * PI / 180.0);
    float sinPitch = sin(imuData[1] * PI / 180.0);
    float cosPitch = cos(imuData[1] * PI / 180.0);
    
    // Transform acceleration to world frame (simplified)
    float verticalAccel = imuData[8] * cosPitch * cosRoll - 
                          imuData[6] * cosPitch * sinRoll - 
                          imuData[7] * sinPitch;
    
    // Remove gravity
    //verticalAccel -= 9.81;
    
    // Integrate acceleration to get velocity change
    float velocityChange = verticalAccel * dt;
    verticalVelocity += velocityChange;
    
    // Integrate velocity to get position change
    float positionChange = verticalVelocity * dt;
    
    // Complementary filter
    // Higher alpha = more trust in barometer/GPS, lower alpha = more trust in IMU
    float alpha = 0.1; // Adjust based on testing
    

    fusedAltitude = alpha * baroAlt + (1 - alpha) * (lastFusedAltitude + positionChange);
    
    // Update values for next iteration
    lastFusedAltitude = fusedAltitude;

    unsigned long lastAltitudeTime = now;
    
    return fusedAltitude;
}
