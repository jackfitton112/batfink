/**
 * @file pid.cpp
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief PID controller source file
 * @version 0.1
 * @date 28-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "pid.h"
#include "../MotorControl/MotorControl.h"

//define left and right motor pid controllers
PID* leftMotorPID = new PID(KP, KI, KD, DT);
PID* rightMotorPID = new PID(KP, KI, KD, DT);

PID::PID(float kp, float ki, float kd, float dt) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
    this->integral = 0;
    this->derivative = 0;
    this->previousError = 0;
}

float PID::update(float error) {
    this->integral += error * this->dt;
    this->derivative = (error - this->previousError) / this->dt;
    this->previousError = error;
    return this->kp * error + this->ki * this->integral + this->kd * this->derivative;
}

void PID::setKp(float kp) {
    this->kp = kp;
}

void PID::setKi(float ki) {
    this->ki = ki;
}

void PID::setKd(float kd) {
    this->kd = kd;
}

void PID::setDt(float dt) {
    this->dt = dt;
}

float PID::getKp() {
    return this->kp;
}

float PID::getKi() {
    return this->ki;
}

float PID::getKd() {
    return this->kd;
}

float PID::getDt() {
    return this->dt;
}



//constant velocity forwards
//use pid to control velocity

//SOMETHING IS VERY BROKEN HERE
void goForwardPid(float distance, float targetVelocity) {
    // Constants
    const float WHEEL_DIAMETER = 40.0;  // mm
    const float COUNTS_PER_REVOLUTION = 298.0 * 12.0;
    const float MM_PER_COUNT = WHEEL_DIAMETER * 3.14159265358979323846 / COUNTS_PER_REVOLUTION;

    // Calculate the target counts based on the given distance
    float targetCounts = distance / MM_PER_COUNT;

    // Initialize variables
    float leftError, rightError;
    float leftVelocity, rightVelocity;

    // Start the motors
    leftMotor->setSpeed(127); //PWM number between 0-255
    rightMotor->setSpeed(127);

    // Main loop
    while (true) {
        // Calculate current counts for left and right motors
        float leftCounts = leftMotor->getSteps();
        float rightCounts = rightMotor->getSteps();

        // Calculate errors
        leftError = targetCounts - leftCounts;
        rightError = targetCounts - rightCounts;

        // Update PID controllers
        leftVelocity = leftMotorPID->update(leftError);
        rightVelocity = rightMotorPID->update(rightError);

        // Adjust the motors accordingly
        leftMotor->setSpeed(targetVelocity + leftVelocity);
        rightMotor->setSpeed(targetVelocity + rightVelocity);

        // Check if the target distance has been reached
        if (fabs(leftError) < 1.0 && fabs(rightError) < 1.0) {
            break;
        }

        //sleep for 5ms
        rtos::ThisThread::sleep_for(5);
    }

    // Stop the motors
    leftMotor->stop();
    rightMotor->stop();
}

