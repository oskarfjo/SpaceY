#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

void initSensors();
void readImu();
void readPressure();
void readGps(float gpsData[10]);
void sendLoRaMessage(String message);

#endif