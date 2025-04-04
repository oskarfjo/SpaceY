#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Arduino.h>

void initSimulatorinterface();
void readSimulator(float simRead[12]);
void publishSimulator(float simPub[2], float simRead[12]);

#endif