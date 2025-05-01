#include "rocketUtils.h"
#include "flightData.h"
#include <Arduino.h>

bool initAltitude = false;
float initPressureHPa = 0.0;

float lastAltitude = 0;
double dtAltitude = 0.0;

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
    
    unsigned long timeCur = micros(); // time now in micros ; 10^{-6}s
    dtAltitude = (timeCur - lastAltitudeTime) * 1e-6; // dt in seconds
    lastAltitudeTime = timeCur; // updates timePrev

    // Get barometric altitude
    float baroAlt = relativeAltitude(sensorData.pressure);

    /*
    // Get vertical acceleration (in the world frame)
    // We need to transform the acceleration from sensor frame to world frame using the orientation
    float cr = cos(sensorData.roll), sr = sin(sensorData.roll);
    float cp = cos(sensorData.pitch), sp = sin(sensorData.pitch);
    float cy = cos(sensorData.heading), sy = sin(sensorData.heading);

    float R[3][3];

    // ZYX rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;

    float ax_local = sensorData.accelX;
    float ay_local = sensorData.accelY;
    float az_local = sensorData.accelZ;

    //float ax_global = R[0][0]*ax_local + R[0][1]*ay_local + R[0][2]*az_local;
    //float ay_global = R[1][0]*ax_local + R[1][1]*ay_local + R[1][2]*az_local;
    float az_global = R[2][0]*ax_local + R[2][1]*ay_local + R[2][2]*az_local;
    

    // Integrate acceleration to get velocity change
    float velocityChange = az_global * dtAltitude;
    sensorData.verticalVelocity += velocityChange;
    
    // Integrate velocity to get position change
    float positionChange = sensorData.verticalVelocity * dtAltitude;
    */
    float currentVel = (baroAlt - lastAltitude)/dtAltitude;
    sensorData.verticalVelocity = sensorData.verticalVelocity*0.9+currentVel*0.1;

    // Update values for next iteration
    lastAltitude = baroAlt;


    if (false) {
      Serial.print(F("dt (s): ")); Serial.println(dtAltitude);
      Serial.print(F("barometer altitude: ")); Serial.println(baroAlt);
      Serial.print(F("vertical velocity: ")); Serial.println(sensorData.verticalVelocity);
    }

    return baroAlt;
}
