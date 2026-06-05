// Dependenices
#include <Alfredo_NoU3.h>
#include "Drivetrain.h"

// Drivetrain Motors Init
NoU_Motor frontLeftMotor(1); // [NOTE]: THIS IS CHANGEABLE!
NoU_Motor frontRightMotor(4); // [NOTE]: THIS IS CHANGEABLE!
NoU_Motor rearLeftMotor(8); // [NOTE]: THIS IS CHANGEABLE!
NoU_Motor rearRightMotor(5); // [NOTE]: THIS IS CHANGEABLE!

// Drivetrain Init
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

void beginDrivetrain() {
  // Allows for immediate robot movement from small movements
  drivetrain.setMinimumOutput(0.1);

  // Inversion Parts
  frontLeftMotor.setInverted(true);
  frontRightMotor.setInverted(true);
  rearLeftMotor.setInverted(true);
  rearRightMotor.setInverted(true);
}

void updateDrivetrain(float gamepadX, float gamepadY, float gamepadRotation, float movement_speed) {
  // Sets Axes
  float fieldPowerX = movement_speed * gamepadX;
  float fieldPowerY = -movement_speed * gamepadY;
  float rotationPower = -movement_speed * gamepadRotation;

  // Get robot heading (in radians) from the gyro
  float heading = NoU3.yaw * ANGULAR_SCALE;

  // Rotate joystick vector to be robot-centric
  float cosA = cos(heading);
  float sinA = sin(heading);

  // Derive robot power
  float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
  float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

  // Send to helper method
  drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);
}