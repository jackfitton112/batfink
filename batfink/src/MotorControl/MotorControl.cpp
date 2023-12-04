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
void Motor::setDir(bool dir){
    this->dir = dir;
    this->dirPin.write(dir);
}

void Motor::setPWM(float pwm){
    this->pwmPin.write(pwm);
}

bool Motor::getDir(){
    return this->dir;
}

float Motor::getPWM(){
    return this->pwmPin.read();
}

//ISR
void Motor::encISR(){
    //increment encoder count
    if(this->dir == CW){
        this->encCount++;
    }
    else{
        this->encCount--;
    }
}

//private functions
void Motor::setDriveType(bool driveType){
    this->driveType = driveType;
}

void Motor::calcVel(){
    float encps = (this->encCount - this->encCountPrev) / VEL_TICKER_PERIOD;  //This provides encoder counts per second
    this->Velocity = ((encps / ENC_CPR) * 60) * 2; //calculate velocity in rev/min
    this->encCountPrev = this->encCount; //set previous encoder count to current encoder count

    //PID (constant velocity)
    if(this->driveType == VEL){
        this->error = this->TargetVel - this->Velocity; //calculate error
        this->integral += this->error * VEL_TICKER_PERIOD; //calculate integral
        this->derivative = (this->error - this->errorPrev) / VEL_TICKER_PERIOD; //calculate derivative
        this->errorPrev = this->error; //set previous error to current error

        float pwm = (KP * this->error) + (KI * this->integral) + (KD * this->derivative); //calculate pwm


        this->setPWM(pwm); //set pwm
    }

}

void Motor::calcPos(){
    //calculate position
    this->Position = this->encCount / ENC_CPR; //calculate position in rev
}

void Motor::setPwm(float pwm){
    //set pwm

    //if pwm is negative, flip direction
    if (pwm < 0){
        this->dirPin.write(!this->fwd_dir);
        pwm = -pwm;
    }
    else{
        this->dirPin.write(this->fwd_dir);
    }

    this->pwmPin.write(pwm);
}

int Motor::getPos(){
    return this->Position;
}

float Motor::getVel(){
    return this->Velocity;
}


void Motor::setTargetVel(float vel){
    //This is in rev/min but needs converting to encoder counts per second
    float encps = (vel / 60) * ENC_CPR; //convert to encoder counts per second
    this->TargetVel = encps; //set target velocity
}