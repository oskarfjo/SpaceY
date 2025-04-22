#include "rocketUtils.h"
#include "flightData.h"
#include <Arduino.h>

bool initAltitude = false;
float initPressureHPa = 0.0;

float fusedAltitude = 0;
float lastFusedAltitude = 0;
float verticalVelocity = 0;
unsigned long lastAltitudeTime = 0;


float relativeAltitude(float pressureMeasured) {
    if (!initAltitude) { // sets the starting pressure
      initPressureHPa = pressureMeasured * 100; // converts bar to hPa
      initAltitude = true;
      return 0.0;
    }
  
    // function for relative altitude adapted from the bar.readAltitude(); function previously used in sensor.cpp
    float currentPressureHPa = pressureMeasured * 100;
    float relAltitude = 44330 * (1.0 - pow((currentPressureHPa / initPressureHPa), 0.1903));
    return relAltitude;
  }


double calculateAltitude() {
    
    unsigned long now = millis();
    unsigned long dt = (now - lastAltitudeTime)/1000; // dt converted to seconds
    lastAltitudeTime = now;

    // Get barometric altitude
    float baroAlt = relativeAltitude(sensorData.pressure);

    // Get vertical acceleration (in the world frame)
    // We need to transform the acceleration from sensor frame to world frame using the orientation
    float sinRoll = sin(sensorData.roll * PI / 180.0);
    float cosRoll = cos(sensorData.roll * PI / 180.0);
    float sinPitch = sin(sensorData.pitch * PI / 180.0);
    float cosPitch = cos(sensorData.pitch * PI / 180.0);
    
    // Transform acceleration to world frame (simplified)
    float verticalAccel = sensorData.accelZ * cosPitch * cosRoll - 
                          sensorData.accelX * cosPitch * sinRoll - 
                          sensorData.accelY * sinPitch;
    
    // Remove gravity
    verticalAccel -= 9.81;
    
    // Integrate acceleration to get velocity change
    float velocityChange = verticalAccel * dt;
    verticalVelocity += velocityChange;
    
    // Integrate velocity to get position change
    float positionChange = verticalVelocity * dt;
    
    // Complementary filter
    // Higher alpha = more trust in barometer/GPS, lower alpha = more trust in IMU
    float alpha = 1.0; // Adjust based on testing

    fusedAltitude = alpha * baroAlt + (1 - alpha) * (lastFusedAltitude + positionChange);
    
    // Update values for next iteration
    lastFusedAltitude = fusedAltitude;

    return fusedAltitude;
}
