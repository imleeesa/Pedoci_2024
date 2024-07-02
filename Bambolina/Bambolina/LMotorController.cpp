/*
   LMotorController Constructor: Initializes motor control pins and constants and sets the pins as outputs.
  setMotorA: Updates the motor constants.
  move (leftSpeed, rightSpeed, minAbsSpeed): Moves both motors with specified speeds, ensuring speeds are within valid PWM range and adjusting direction accordingly.
  move (speed, minAbsSpeed): Moves both motors at the same speed, ensuring speed is within valid range and adjusting direction accordingly.
  move (speed): Moves both motors at the same speed without minimum absolute speed constraint, ensuring speed is within valid range.
  turnLeft: Turns the robot left at a given speed, with an optional initial kick.
  turnRight: Turns the robot right at a given speed, with an optional initial kick.
  stopMoving: Stops all motor movement by setting control pins to LOW and enable pins to HIGH, and resets the current speed.
*/

#include "LMotorController.h"
#include "Arduino.h"

// Constructor for LMotorController class, initializing pins and constants for two motors
LMotorController::LMotorController(int ena, int in1, int in2, int enb, int in3, int in4, double motorAConst, double motorBConst)
{
  _motorAConst = motorAConst; // Store the motor A constant
  _motorBConst = motorBConst; // Store the motor B constant

  // Initialize motor control pins
  _ena = ena;
  _in1 = in1;
  _in2 = in2;
  _enb = enb;
  _in3 = in3;
  _in4 = in4;

  // Set the motor control pins as output
  pinMode(_ena, OUTPUT);
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(_enb, OUTPUT);
  pinMode(_in3, OUTPUT);
  pinMode(_in4, OUTPUT);
}

// Method to set the motor constants
void LMotorController::setMotorA(double motorAConst, double motorBConst) {
  _motorAConst = motorAConst; // Update motor A constant
  _motorBConst = motorBConst; // Update motor B constant
}

// Method to move both motors with specified speeds and a minimum absolute speed
void LMotorController::move(int leftSpeed, int rightSpeed, int minAbsSpeed)
{
  // Adjust right motor speed based on the minimum absolute speed and within valid PWM range
  if (rightSpeed < 0)
  {
    rightSpeed = min(rightSpeed, -1 * minAbsSpeed); // Ensure it's not less than -minAbsSpeed
    rightSpeed = max(rightSpeed, -255); // Ensure it doesn't exceed -255
  }
  else if (rightSpeed > 0)
  {
    rightSpeed = max(rightSpeed, minAbsSpeed); // Ensure it's at least minAbsSpeed
    rightSpeed = min(rightSpeed, 255); // Ensure it doesn't exceed 255
  }

  // Map the right motor speed to the valid PWM range
  int realRightSpeed = map(abs(rightSpeed), 0, 255, minAbsSpeed, 255);

  // Adjust left motor speed based on the minimum absolute speed and within valid PWM range
  if (leftSpeed < 0)
  {
    leftSpeed = min(leftSpeed, -1 * minAbsSpeed); // Ensure it's not less than -minAbsSpeed
    leftSpeed = max(leftSpeed, -255); // Ensure it doesn't exceed -255
  }
  else if (leftSpeed > 0)
  {
    leftSpeed = max(leftSpeed, minAbsSpeed); // Ensure it's at least minAbsSpeed
    leftSpeed = min(leftSpeed, 255); // Ensure it doesn't exceed 255
  }

  // Map the left motor speed to the valid PWM range
  int realLeftSpeed = map(abs(leftSpeed), 0, 255, minAbsSpeed, 255);

  // Set the direction of the right motor
  digitalWrite(_in3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(_in4, rightSpeed > 0 ? LOW : HIGH);
  // Set the direction of the left motor
  digitalWrite(_in1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(_in2, leftSpeed > 0 ? LOW : HIGH);
  // Set the speed of the right motor with motorAConst scaling
  analogWrite(_ena, realRightSpeed * _motorAConst);
  // Set the speed of the left motor with motorBConst scaling
  analogWrite(_enb, realLeftSpeed * _motorBConst);
}

// Method to move both motors at the same speed with a minimum absolute speed
void LMotorController::move(int speed, int minAbsSpeed)
{
  int direction = 1; // Initialize direction as forward

  // Adjust speed based on the minimum absolute speed and within valid PWM range
  if (speed < 0)
  {
    direction = -1; // Change direction to reverse

    speed = min(speed, -1 * minAbsSpeed); // Ensure it's not less than -minAbsSpeed
    speed = max(speed, -255); // Ensure it doesn't exceed -255
  }
  else
  {
    speed = max(speed, minAbsSpeed); // Ensure it's at least minAbsSpeed
    speed = min(speed, 255); // Ensure it doesn't exceed 255
  }

  // If the speed hasn't changed, do nothing
  if (speed == _currentSpeed) return;

  // Ensure the real speed is at least minAbsSpeed
  int realSpeed = max(minAbsSpeed, abs(speed));
  Serial.print("vel="); // Print the speed for debugging
  Serial.println(realSpeed);

  // Set the direction for both motors
  digitalWrite(_in1, speed > 0 ? HIGH : LOW);
  digitalWrite(_in2, speed > 0 ? LOW : HIGH);
  digitalWrite(_in3, speed > 0 ? HIGH : LOW);
  digitalWrite(_in4, speed > 0 ? LOW : HIGH);
  // Set the speed for both motors with respective constants
  analogWrite(_ena, realSpeed * _motorAConst);
  analogWrite(_enb, realSpeed * _motorBConst);

  _currentSpeed = direction * realSpeed; // Update the current speed
}

// Method to move both motors at the same speed without minimum absolute speed
void LMotorController::move(int speed)
{
  // If the speed hasn't changed, do nothing
  if (speed == _currentSpeed) return;

  // Clamp the speed to the valid PWM range
  if (speed > 255) speed = 255;
  else if (speed < -255) speed = -255;

  // Set the direction for both motors
  digitalWrite(_in1, speed > 0 ? HIGH : LOW);
  digitalWrite(_in2, speed > 0 ? LOW : HIGH);
  digitalWrite(_in3, speed > 0 ? HIGH : LOW);
  digitalWrite(_in4, speed > 0 ? LOW : HIGH);
  // Set the speed for both motors with respective constants
  analogWrite(_ena, abs(speed) * _motorAConst);
  analogWrite(_enb, abs(speed) * _motorBConst);

  _currentSpeed = speed; // Update the current speed
}

// Method to turn the robot left at a given speed, with an optional initial kick
void LMotorController::turnLeft(int speed, bool kick)
{
  // Set the direction for turning left
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, LOW);
  digitalWrite(_in3, LOW);
  digitalWrite(_in4, HIGH);

  if (kick) // If a kick is needed
  {
    analogWrite(_ena, 255); // Set both motors to maximum speed
    analogWrite(_enb, 255);
    delay(100); // Delay for a short time
  }

  // Set the speed for both motors with respective constants
  analogWrite(_ena, speed * _motorAConst);
  analogWrite(_enb, speed * _motorBConst);
}

// Method to turn the robot right at a given speed, with an optional initial kick
void LMotorController::turnRight(int speed, bool kick)
{
  // Set the direction for turning right
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, HIGH);
  digitalWrite(_in3, HIGH);
  digitalWrite(_in4, LOW);

  if (kick) // If a kick is needed
  {
    analogWrite(_ena, 255); // Set both motors to maximum speed
    analogWrite(_enb, 255);
    delay(100); // Delay for a short time
  }

  // Set the speed for both motors with respective constants
  analogWrite(_ena, speed * _motorAConst);
  analogWrite(_enb, speed * _motorBConst);
}

// Method to stop all motor movement
void LMotorController::stopMoving()
{
  // Set all motor control pins to LOW
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, LOW);
  digitalWrite(_in3, LOW);
  digitalWrite(_in4, LOW);
  // Set the enable pins to HIGH to stop the motors
  digitalWrite(_ena, HIGH);
  digitalWrite(_enb, HIGH);

  _currentSpeed = 0; // Reset current speed to 0
}
