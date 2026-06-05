#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

// Dependencies
#include <Arduino.h>
#include <Alfredo_NoU3.h>

// Start Functions
void beginDrivetrain();

//Loop Functions
void updateDrivetrain(float gamepadX, float gamepadY, float gamepadRotation, float movement_speed);

#define MEASURED_ANGLE 27.451;
#define ANGULAR_SCALE (5.0 * 2.0 * PI) / MEASURED_ANGLE;


#endif