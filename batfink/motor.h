
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
#define ENC_CPR 1746 //encoder counts per revolution
#define WHEEL_DIAMETER 47 //wheel diameter in mm
#define WHEEL_CIRCUMFERENCE 147 //wheel circumference in mm
#define WHEEL_BASE 150 //wheel base in mm

#define VEL_TICKER_PERIOD 0.1 //ticker period for velocity calculation
#define PID_TICKER_PERIOD 0.1 //ticker period for PID calculation

#define PWM_PERIOD_US 20 //pwm period in microseconds

//pid constants
#define KP 0.001
#define KI 0.1
#define KD 0

//called in the constructor, used as only 1 of the encoder pins is connected therefore dir is required
enum forwardDirection {
    CW = true,
    CCW = false
};

enum driveDirection {
    FORWARD = true,
    BACKWARD = false
};

enum RobotMovementMode {
    STOP,
    CONSTANT_VELOCITY,
    EXACT_POSITION

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

        volatile float _PIDerror; //the current error of the PID controller
        volatile float _PIDlastError; //the error of the PID controller from the last tick
        volatile float _PIDintegral; //the integral of the PID controller
        volatile float _PIDderivative; //the derivative of the PID controller
        volatile float _PIDoutput; //the output of the PID controller



        int _ERR = OK; //the error code for the motor
        bool _fwdDir; //the direction the motor needs to go to move the robot forwards
        DigitalOut _directionPin; //the pin that controls the direction of the motor
        PwmOut _pwmPin; //the pin that controls the speed of the motor
        InterruptIn _encoderPin; //the pin that reads the encoder


        
};


extern Motor rightMotor;
extern Motor leftMotor;
extern int motorInit();


#endif //MOTOR_H

/*
void Motor::tickVelocity() {
  //calculate encoder difference
  int diff = encoderCount-encoderCountLast;
  //update encoder difference
  encoderCountLast = encoderCount;
  //calculate speed
  velocity = (float)diff/VELOCITY_MEASURE_SPEED;
  //calculate control signal, for VELOCITY and or POSITION
  switch (target.demandType) {
    case NONE:
      // do not do PID things.
      settled = true;
      rawset(0, 0);
      previous_error = 0;
      error = 0;
      integral_error = 0;
      break;
    case VELOCITY:
      settled = false;
      error =  velocity - target.value;
      derivative_error = (float)(error-previous_error)/VELOCITY_MEASURE_SPEED;
      integral_error = integral_error + error * VELOCITY_MEASURE_SPEED;
      signal = velocityConfig.kp*error + velocityConfig.kd*derivative_error + velocityConfig.ki*integral_error;
      rawset(signal, velocityConfig.MIZeroOffset);
      previous_error = error;
      break;
    case POSITION:
    case POSITION_RELATIVE:
      // motor hasn't settled yet
      settled = false;
      error = target.value - encoderCount;
      derivative_error = (float)(error-previous_error)/VELOCITY_MEASURE_SPEED;
      integral_error = integral_error + error * VELOCITY_MEASURE_SPEED;
      signal = positionConfig.kp*error + positionConfig.kd*derivative_error + positionConfig.ki*integral_error;
      // If motor has settled
      if ((error*error+velocity*velocity)<=positionConfig.deadZone*positionConfig.deadZone) {
        target.demandType=NONE;
        settled = true;
        return;
      }
      // Motor hasn't settled, transmit signal to wheels
      rawset(-signal, positionConfig.MIZeroOffset);
      previous_error = error;
      break;
  }
}
*/