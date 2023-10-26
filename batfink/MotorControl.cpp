//MotorControl.cpp
#include "MotorControl.h"
#include <Arduino.h>
#include <ARB.h>

int MOTOR_LEFT = 0;
int MOTOR_RIGHT = 0;

// No redefinition of the Motor class here!

Motor::Motor(int dirPin, int pwmPin, int encPin, int forwardMultiplier)
    : _dirPin(dirPin), _pwmPin(pwmPin), _encPin(encPin), _dir(CW), _steps(0), _speed(0), _forwardMultiplier(forwardMultiplier) 
{
    pinMode(_dirPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    pinMode(_encPin, INPUT);
    digitalWrite(_dirPin, _dir);
    analogWrite(_pwmPin, 0);
}

void Motor::setDirection(Direction dir) {
    _dir = dir;
    digitalWrite(_dirPin, _dir);
}

Direction Motor::getDirection() const {
    return _dir;
}

void Motor::setSpeed(int speed) {
    _speed = speed;
    analogWrite(_pwmPin, _speed);
}

int Motor::getSpeed() const {
    return _speed;
}

void Motor::updateEncoder() {
    if((_dir == CW && _forwardMultiplier == 1) || (_dir == CCW && _forwardMultiplier == -1)) {
        _steps++;
    } else {
        _steps--;
    }
}


int Motor::getSteps() const {
    return _steps;
}

void Motor::resetEncoder() {
    _steps = 0;
}

// ISR handlers need to be global
Motor *leftMotor;
Motor *rightMotor;

void ENCA_ISR() {
  rightMotor->updateEncoder();
}

void ENCB_ISR() {
  leftMotor->updateEncoder();
}


void Motor_setup() {
    // For left motor, forward direction is CW, so forwardMultiplier is 1
    leftMotor = new Motor(MOTOR_DIRB, MOTOR_PWMB, MOTOR_ENCB, 1);

    // For right motor, forward direction is CCW, so forwardMultiplier is -1
    rightMotor = new Motor(MOTOR_DIRA, MOTOR_PWMA, MOTOR_ENCA, -1);

    attachInterrupt(digitalPinToInterrupt(leftMotor->getEncPin()), ENCB_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightMotor->getEncPin()), ENCA_ISR, CHANGE);
}


void forward() {
  Serial.println("Moving Forward");
  rightMotor->setDirection(CCW);
  leftMotor->setDirection(CW);
  rightMotor->resetEncoder();
  leftMotor->resetEncoder();
  rightMotor->setSpeed(MOTOR_RIGHT);
  leftMotor->setSpeed(MOTOR_LEFT);
}

void left() {
  Serial.println("Turning Left");
  rightMotor->setDirection(CCW);
  leftMotor->setDirection(CCW);
  rightMotor->resetEncoder();
  leftMotor->resetEncoder();
  rightMotor->setSpeed(127);  // Assuming half speed for turning, adjust as needed
  leftMotor->setSpeed(127);
}

void right() {
  Serial.println("Turning Right");
  rightMotor->setDirection(CW);
  leftMotor->setDirection(CW);
  rightMotor->resetEncoder();
  leftMotor->resetEncoder();
  rightMotor->setSpeed(127);  // Assuming half speed for turning, adjust as needed
  leftMotor->setSpeed(127);
}

void backward() {
  Serial.println("Moving Backward");
  rightMotor->setDirection(CW);
  leftMotor->setDirection(CCW);
  rightMotor->resetEncoder();
  leftMotor->resetEncoder();
  rightMotor->setSpeed(MOTOR_RIGHT);
  leftMotor->setSpeed(MOTOR_LEFT);
}

void stopRobot() {
  Serial.println("Stopping");
  rightMotor->setSpeed(0);
  leftMotor->setSpeed(0);
}


// Make sure to delete the motors in a cleanup function if needed
void cleanup() {
  delete leftMotor;
  delete rightMotor;
}
