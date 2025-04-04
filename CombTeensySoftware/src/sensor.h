#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

void initSensors();
void readImu(float imuData[12]);
void readPressure(float barData[3]);
void readGps(float gpsData[10]);
void sendLoRaMessage(String message);
void receiveLoRaMessage();

#endif