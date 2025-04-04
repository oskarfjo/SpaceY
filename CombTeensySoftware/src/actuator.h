#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>

void initActuator();
void deployParachute(bool release);
void armIgnition();
void ignite();
void resetIgnition();
void buzzer();
void updateServos();

#endif