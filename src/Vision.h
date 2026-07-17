// Self Definition
#ifndef VISION_H
#define VISION_H


// Dependencies
#include <VL53L0X.h>

// Sensor Definition
inline VL53L0X distanceSensor;

// Vision Functions
void beginVision();

float readDistanceCm();

#endif