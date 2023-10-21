#pragma once

#include <Arduino.h>
#include <ARB.h>

// Your enumerations
enum Direction {CW, CCW}; 

class Motor {
private:
  int _dirPin;
  int _pwmPin;
  int _encPin;
  Direction _dir;
  volatile int _steps;
  int _speed;
  int _forwardMultiplier;  // Either 1 or -1

public:
  Motor(int dirPin, int pwmPin, int encPin, int forwardMultiplier);


  void setDirection(Direction dir);
  Direction getDirection() const;
  
  void setSpeed(int speed);
  int getSpeed() const;
  
  void updateEncoder();
  int getSteps() const;
  void resetEncoder();

  inline int getEncPin() const { return _encPin; }
};

// Global motor objects
extern Motor* leftMotor;
extern Motor* rightMotor;

extern int MOTOR_LEFT;
extern int MOTOR_RIGHT;

// ISR handlers for encoders
void ENCA_ISR();
void ENCB_ISR();

// Movement functions
void forward();
void backward();
void left();
void right();
void stopRobot();

// Setup function
void Motor_setup();

// Cleanup function (if necessary)
void cleanup();
