
/**
 * @file motor.h
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Motor module for the batfink robot header file
 * @version 0.1
 * @date 26-12-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */



#ifndef MOTOR_H
#define MOTOR_H

#include <mbed/mbed.h>
#include <functional>

#include "error.h"

using namespace mbed;
using namespace rtos;

//MBED compliant motor pins
#define MBED_RIGHT_MOTOR_DIR P0_4
#define MBED_RIGHT_MOTOR_ENC P1_11
#define MBED_RIGHT_MOTOR_PWM P0_27

#define MBED_LEFT_MOTOR_DIR P0_5
#define MBED_LEFT_MOTOR_ENC P1_12
#define MBED_LEFT_MOTOR_PWM P1_2

//3576 / 2
#define ENC_CPR 1746.0f //encoder counts per revolution
#define WHEEL_DIAMETER 47.0f //wheel diameter in mm
#define WHEEL_CIRCUMFERENCE 147.0f //wheel circumference in mm
#define WHEEL_BASE 150.0f //wheel base in mm

#define VEL_TICKER_PERIOD 0.1 //ticker period for velocity calculation
#define PID_TICKER_PERIOD 0.1 //ticker period for PID calculation

#define PWM_PERIOD_US 20 //pwm period in microseconds

//pid constants
#define VELKP 0.01
#define VELKI 0.02
#define VELKD 0.00001

#define POSKP 0.025
#define POSKI 0.0002
#define POSKD 0.00001

#define TURNINGKP 0.045
#define TURNINGKI 0.0004
#define TURNINGKD 0.00001


//called in the constructor, used as only 1 of the encoder pins is connected therefore dir is required
enum forwardDirection {
    CW = true,
    CCW = false
};

enum driveDirection {
    FWD = true,
    BKWD = false
};

enum RobotMovementMode {
    STP,
    CONSTANT_VELOCITY,
    EXACT_POSITION,
    TURNING

};


class Motor {

    public:

        Motor(PinName pwmPin, PinName directionPin, PinName encoderPin, forwardDirection fwdDir);

        int setup();
        int stop();
        int setVelocity(float velocity); //RPMs
        int setPosition(int position); //encoder counts
        int getCurrentVelocity();
        int getCurrentPosition();

        using MotorCallback = std::function<void()>;
        void onStopped(const MotorCallback& callback);





    //private:

        void _encoderISR(); //
        void _PID(); //
        void _CalcVelocity(); //
        int _setPWM(double pwmVal);
        int _setDirection(driveDirection dir);
        int _setTargetVelocity(float velocity);
        int _setTargetPosition(int position);


       
        RobotMovementMode _movementMode; //the current movement mode of the motor

        volatile int _encoderCount; //the current encoder count
        volatile int _lastEncoderCount; //the encoder count from the last tick

        volatile float _currentVelocity; //the current velocity of the motor
        volatile float _TargetVelocity; //the target velocity of the motor
        volatile float _encoderDiff; //the difference between the encoder count from the last tick and the current tick

        volatile int _TargetPosition; //the target position of the motor

        Ticker _velocityTicker;
        Ticker _PIDTicker;
        MotorCallback stoppedCallback;


        volatile float _PIDerror; //the current error of the PID controller
        volatile float _PIDlastError; //the error of the PID controller from the last tick
        volatile float _PIDintegral; //the integral of the PID controller
        volatile float _PIDderivative; //the derivative of the PID controller
        volatile float _PIDoutput; //the output of the PID controller
        volatile float _torqueCompensation; //the torque compensation value for the motor


        int _ERR = OK; //the error code for the motor
        bool _fwdDir; //the direction the motor needs to go to move the robot forwards
        DigitalOut _directionPin; //the pin that controls the direction of the motor
        PwmOut _pwmPin; //the pin that controls the speed of the motor
        InterruptIn _encoderPin; //the pin that reads the encoder


        
};


extern Motor rightMotor;
extern Motor leftMotor;



#endif //MOTOR_H
