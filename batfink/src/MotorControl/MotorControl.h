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
#define KP 0.001
#define KI 0.01
#define KD 0


//MotorControl class (using mbed)
class Motor {

  public:

    //constructor & setup
    Motor(PinName pwmPin, PinName dirPin, PinName encPin, bool Fwd_dir); //constructor
    void setup(); //initialises motor

    //setters and getters

    //setters
    void setTargetPos(int pos); //sets target position of motor in wheel revolutions
    void setTargetVel(float vel); //sets target velocity of motor in wheel revolutions per minute
    void setDir(bool dir); //sets direction of motor (CW or CCW)

    //getters

    float getVel(); //returns velocity of motor in wheel revolutions per minute
    int getPos(); //returns position of motor in wheel revolutions
    float getTargetVel(); //returns target velocity of motor in wheel revolutions per minute
    int getTargetPos(); //returns target position of motor in wheel revolutions
    bool getDir(); //returns direction of motor (CW or CCW)

    float PIDerror;
    float PIDpwm;


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


    //ISR
    void encISR(); //encoder interrupt service routine

    //private functions
    void setPWM(float pwm); //sets pwm of motor
    void setDriveType(bool driveType); //sets drive type of motor
    void calcVel();
    float PID(); //PID control

  };

#endif