// Dependenices
#include "Drivetrain.h"


void beginDrivetrain() {
  // Allows for immediate robot movement from small movements
  drivetrain.setMinimumOutput(0.1);

  // Inversion Parts
  frontLeftMotor.setInverted(frontLeftInversion);
  frontRightMotor.setInverted(frontRightInversion);
  rearLeftMotor.setInverted(rearLeftInversion);
  rearRightMotor.setInverted(rearRightInversion);
}


void updateDrivetrain(float gamepadX, float gamepadY, float gamepadRotation, float movement_speed) {
  // Sets Axes
  float fieldPowerX = xAxesInversion * movement_speed * gamepadX;
  float fieldPowerY = yAxesInversion * movement_speed * gamepadY;
  float rotationPower = rotationAxesInversion * movement_speed * gamepadRotation;

  // Get robot heading (in radians) from the gyro
  float heading = NoU3.roll * ANGULAR_SCALE;


  if (fabs(rotationPower) > DEADBAND) { // Drift Compensation Script
    // The driver is turning intentionally. Turn off heading lock.
    headingLocked = false;
  } else if (fabs(fieldPowerX) > DEADBAND || fabs(fieldPowerY) > DEADBAND) {
    if (!headingLocked) {
      targetHeading = heading;
      headingLocked = true;
    }

    // Calculate the heading error
    float headingError = targetHeading - heading;

    // Optional: Normalize angle error to stay between -PI and PI 
    // (Prevents the robot from spinning the long way around)
    while (headingError > PI)  headingError -= 2.0 * PI;
    while (headingError < -PI) headingError += 2.0 * PI;

    // Calculate corrective rotation power using Proportional control
    rotationPower = headingError * kP;

    // Constrain the correction so it doesn't overpower everything
    if (rotationPower > 0.5)  rotationPower = 0.5;
    if (rotationPower < -0.5) rotationPower = -0.5;
  } else {
    headingLocked = false;
    rotationPower = 0;
  }

  // Rotate joystick vector to be robot-centric
  float cosA = cos(heading);
  float sinA = sin(heading);

  // Derive robot power
  float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
  float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

  // Send to helper method
  drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);
}
