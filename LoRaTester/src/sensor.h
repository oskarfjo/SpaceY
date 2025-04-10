#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

void initSensors();
void sendLoRaMessage(String message);
void receiveLoRaMessage();

#endif