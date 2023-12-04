/**
 * @file MotorControl.h
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief  Motor Control header file
 * @version 0.1
 * @date 28-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <mbed.h>


#define CCW true
#define CW false
#define VEL true
#define POS false

#define ENC_CPR 3576 //encoder counts per revolution (6 counts per motor revolution * 298:1 gear ratio)
#define VEL_TICKER_PERIOD 0.1 //ticker period for velocity calculation

//pid constants
#define KP 0.0003
#define KI 0
#define KD 0.002


//MotorControl class (using mbed)
class Motor {

  public:

    //constructor
    Motor(PinName pwmPin, PinName dirPin, PinName encPin, bool Fwd_dir);

    //functions
    void setup(); //initialises motor
    void setVel(float vel); //sets velocity of motor in rev/min
    void setPos(int pos); //sets position of motor - eg X mm driven

    float getVel(); //gets velocity of motor in rev/min
    int getPos(); //gets position of motor - eg X mm driven
    void setPWM(float pwm); //sets pwm of motor
    void setTargetVel(float vel); //sets target velocity of motor in rev/min

  private:

    //pins
    mbed::PwmOut pwmPin; //sends pwm signal to motor 
    mbed::DigitalOut dirPin; //sets direction of motor
    mbed::InterruptIn encPin; //reads encoder input


    //variables
    int32_t encCount; //encoder count
    int32_t encCountPrev; //previous encoder count
    int16_t Velocity; //velocity of motor in rev/min
    int16_t Position; //position of motor in rev

    bool dir; //direction of motor
    bool driveType; //drive type of the motor, true = velocity, false = position
    bool fwd_dir; //forward direction of motor

    mbed::Ticker velTicker; //ticker for velocity calculation

    int TargetPos; //target position of motor
    float TargetVel; //target velocity of motor in rev/min

    //pid
    float error; //error
    float errorPrev; //previous error
    float integral; //integral
    float derivative; //derivative


    //functions
    //setters and getters
    void setDir(bool dir); //sets direction of motor
    //void setPWM(float pwm); //sets pwm of motor
    bool getDir(); //gets direction of motor
    float getPWM(); //gets pwm of motor

    //ISR
    void encISR(); //encoder interrupt service routine

    //private functions
    void setDriveType(bool driveType); //sets drive type of motor
    void calcVel(); //calculates velocity of motor
    void calcPos(); //calculates position of motor
    void setPwm(float pwm); //sets pwm of motor (private
    int pid(float target, float error); //pid controller (private
  };

#endif