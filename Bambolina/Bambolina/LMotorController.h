#ifndef LMotorController_h
#define LMotorController_h

// Include the Arduino core library for using Arduino-specific functions
#include "Arduino.h"

// Definition of the LMotorController class
class LMotorController
{
protected:
    // Motor pins for motor A (left) and motor B (right)
    int _ena, _in1, _in2, _enb, _in3, _in4;
    
    // Current speed of the motors
    int _currentSpeed;
    
    // Constants for motor speeds (used to adjust speeds for each motor)
    double _motorAConst, _motorBConst;

public:
    // Constructor that initializes the motor pins and constants
    LMotorController(int ena, int in1, int in2, int enb, int in3, int in4, double motorAConst, double motorBConst);

    // Methods for controlling the motors
    void move(int leftSpeed, int rightSpeed, int minAbsSpeed);
    void move(int speed);
    void move(int speed, int minAbsSpeed);
    void turnLeft(int speed, bool kick);
    void turnRight(int speed, bool kick);
    void stopMoving();

    // Method to set the motor constants (adjust motor speeds)
    void setMotorA(double motorAConst, double motorBConst);
};

#endif
