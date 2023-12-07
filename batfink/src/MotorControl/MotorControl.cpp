/**
 * @file MotorControl.cpp
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief  Motor Control source file
 * @version 0.1
 * @date 28-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 * @note Max velocity = 55 rev/min
 * @note Min velocity = 3 rev/min (without stalling)
 * 
 */
#include "MotorControl.h"
//#include <mbed.h>

/**
 * @brief Constructor for the Motor class.
 * 
 * @param pwmPin The pin used for PWM control.
 * @param dirPin The pin used for direction control.
 * @param encPin The pin used for encoder input.
 */
Motor::Motor(PinName pwmPin, PinName dirPin, PinName encPin, bool Fwd_dir):
    pwmPin(pwmPin), dirPin(dirPin), encPin(encPin)
    {
        //initialise variables
        encCount = 0;
        encCountPrev = 0;
        Velocity = 0;
        Position = 0;
        dir = Fwd_dir;
        TargetPos = 0;
        TargetVel = 0;
        driveType = true;

       

    }
void Motor::setup() {
     //initialise pins
        this->pwmPin.period_us(20); //set pwm period to 50us
        this->pwmPin.write(0); //set pwm to 0
        this->dirPin.write(dir); //set direction to CW
        this->encPin.rise(mbed::callback(this, &Motor::encISR)); //set encoder ISR for rise
        this->encPin.fall(mbed::callback(this, &Motor::encISR)); //set encoder ISR for rise

        this->fwd_dir = this->dir;

        //initialise ticker
        velTicker.attach(mbed::callback(this, &Motor::calcVel), VEL_TICKER_PERIOD); //attach ticker to calcVel function, 0.1s period
}

//setters and getters

//setters
void Motor::setTargetPos(int pos) {
    TargetPos = pos;
    driveType = POS;
}

void Motor::setTargetVel(float vel) {
    TargetVel = vel;
    driveType = VEL;
}

void Motor::setDir(bool dir) {
    this->dir = dir;
    this->dirPin.write(dir);
}

//getters

float Motor::getVel() {
    return Velocity;
}

int Motor::getPos() {
    return Position;
}

float Motor::getTargetVel() {
    return TargetVel;
}

int Motor::getTargetPos() {
    return TargetPos;
}

bool Motor::getDir() {
    return dir;
}

//private functions

void Motor::calcVel() {
    //calculate velocity
    Velocity = (encCount - encCountPrev) * 10 * 60 / 64; //calculate velocity in rev/min
    encCountPrev = encCount; //set previous encoder count to current encoder count
    if (driveType == VEL) {
        PID();
        setPWM(PIDpwm);
    }

}

void Motor::encISR() {
    //encoder ISR
    if (dir == fwd_dir) {
        encCount++;
    } else {
        encCount--;
    }
}

void Motor::setPWM(float pwm) {
    //set pwm of motor
    if (pwm > 1) {
        pwm = 1;
    } else if (pwm < -1) {
        pwm = -1;
    }
    //TODO: IF THE PWM IS LESS THAN 0, THE MOTOR DIRECTION NEEDS TO CHANGE 
    this->pwmPin.write(fabs(pwm));
}

void Motor::setDriveType(bool driveType) {
    //set drive type of motor
    this->driveType = driveType;
}

float Motor::PID() {
    //PID control
    error = TargetVel - Velocity; //calculate error
    integral += error * VEL_TICKER_PERIOD; //calculate integral
    derivative = (error - errorPrev) / VEL_TICKER_PERIOD; //calculate derivative
    errorPrev = error; //set previous error to current error
    PIDerror = error; //set PIDerror to current error

    //return a pwm value between -1 and 1
    if (dir == fwd_dir) {
        PIDpwm = (KP * error) + (KI * integral) + (KD * derivative);
    } else {
        PIDpwm = -((KP * error) + (KI * integral) + (KD * derivative));
    }

}



