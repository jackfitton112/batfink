/**
 * @file motor.h
 * @brief This file contains the declaration of the Motor class and related enums.
 * 
 * The Motor class provides an interface to control the motors of the batfink robot.
 * It allows setting the drive mode, target velocity, target position, movement mode,
 * and enables or disables the PID control. The class also provides functions to start
 * and stop the motor.
 * 
 * The class uses MBED compliant motor pins for controlling the motors. It calculates
 * the current velocity of the motor using an encoder and provides PID control for
 * maintaining a target velocity or position.
 * 
 * @author Jack Fitton
 * @version 0.1
 * @date 26-12-2023
 * 
 * @note This code is subject to the copyright.
 */
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
#include "../error/error.h"

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
#define ENC_CPR 1788 //encoder counts per revolution
#define VEL_TICKER_PERIOD 0.1 //ticker period for velocity calculation
#define PID_TICKER_PERIOD 0.1 //ticker period for PID calculation

#define PWM_PERIOD_US 20 //pwm period in microseconds

//pid constants
#define KP 0.001
#define KI 0.1
#define KD 0


enum forwardDirection {
    CW = false,
    CCW = true
};

enum driveDirection {
    FORWARD = false,
    BACKWARD = true
};

enum RobotDriveMode {
    CONSTANT_VELOCITY = false,
    EXACT_POSITION = true
};

enum RobotMovementMode {
    STOP = false,
    START = true
};

class Motor {

    public:

        Motor(PinName pwmPin, PinName directionPin, PinName encoderPin, forwardDirection fwdDir);

        int setup();

        //Drive functions
        int setDriveMode(RobotDriveMode mode);
        int setTargetVelocity(float targetVelocity);
        int setTargetPosition(float targetPosition);
        int setMovementMode(RobotMovementMode mode);
        int enablePID();
        int disablePID();
        int startMotor();
        int stopMotor();


    private:

        int _ERR = OK; //the error code for the motor
        bool _fwdDir; //the direction the motor needs to go to move the robot forwards
        DigitalOut _directionPin; //the pin that controls the direction of the motor
        PwmOut _pwmPin; //the pin that controls the speed of the motor
        InterruptIn _encoderPin; //the pin that reads the encoder

        RobotMovementMode _movementMode; //the mode the motor is in
        RobotDriveMode _driveMode; //the mode the motor is in

        float _TargetVelocity; //the target velocity of the motor in rpm
        float _TargetPosition; //the target position of the motor in encoder counts
        float _currentVelocity; //the current velocity of the motor in rpm
        float _currentPosition; //the current position of the motor in encoder counts (relative to the start position)

        //PID variables
        bool _PIDenabled;
        Ticker _PIDTicker;
        float _PIDerror;
        float _PIDlastError;
        float _PIDintegral;
        float _PIDderivative;
        double _PIDoutput;

        //Ticker for velocity calculation
        Ticker _velocityTicker;
        void _calculateCurrentVelocity(); //calculates the current velocity of the motor (ISR)
  
        //Encoder variables
        int _encoderCount; //the current encoder count
        int _lastEncoderCount; //the encoder count from the last velocity calculation
        int _encoderDiff; //the difference between the current and last encoder count
        void _encoderISR(); //the interrupt service routine for the encoder


        int _setPWM(double pwm);
        int _setDirection(driveDirection dir);
        void _PID(); //Used in ISR context    
        
};


extern Motor rightMotor;
extern Motor leftMotor;
extern int motorInit();


#endif //MOTOR_H