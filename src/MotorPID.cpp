#include "MotorPID.h"


// Constructor: Initializes the member variables and references
MotorPID::MotorPID(NoU_Motor& givenMotor) 
  : myMotor(givenMotor), targetAngle(0), lastError(0.0), 
    integralSum(0.0), lastPIDTime(0), PIDInterval(20) {
  // Any extra setup code can go here
}

void MotorPID::setZieglerNicholsConstants(float Ku, float Tu) {
  // Ziegler-Nichols PID formulas
  Kp = 0.6 * Ku;
  Ki = (1.2 * Ku) / Tu;
  Kd = 0.075 * Ku * Tu;
}

void MotorPID::updateMotor() {
  if (millis() - lastPIDTime >= PIDInterval) {
    // Read the current angle from your encoder
    float currentAngle = myMotor.getPosition();

    // ** Step 1: Calculate the Error **
    float error = targetAngle - currentAngle;
    Serial.println(currentAngle);
    Serial.println(targetAngle);

    // ** Step 2: Calculate the PID terms **
    float proportionalTerm = Kp * error;

    // Integral term: sum of all past errors
    integralSum += error;
    float integralTerm = Ki * integralSum;

    // Derivative term: rate of change of the error
    float derivativeTerm = Kd * (error - lastError);

    // ** Step 3: Calculate the total PID output **
    float motorSpeedOutput = proportionalTerm + integralTerm + derivativeTerm;

    // ** Step 4: Constrain and set the motor speed **
    motorSpeedOutput = constrain(motorSpeedOutput, -1, 1);
    myMotor.set(-motorSpeedOutput);

    // ** Step 5: Update variables for the next iteration **
    lastError = error;
    lastPIDTime = millis();
  }
}

void MotorPID::setAngle(float givenAngle) {
  targetAngle = givenAngle + 10;
}