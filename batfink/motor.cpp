/**
 * @file motor.cpp
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Motor module for the batfink robot source file
 * @version 0.1
 * @date 26-12-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include "motor.h"

using namespace mbed;
using namespace rtos;

//define motors

Motor rightMotor(MBED_RIGHT_MOTOR_PWM, MBED_RIGHT_MOTOR_DIR, MBED_RIGHT_MOTOR_ENC, CW);
Motor leftMotor(MBED_LEFT_MOTOR_PWM, MBED_LEFT_MOTOR_DIR, MBED_LEFT_MOTOR_ENC, CCW);

const float VELOCITY_FACTOR = 60.0f / (VEL_TICKER_PERIOD * ENC_CPR);


/**
 * @brief Constructs a Motor object.
 * 
 * @param pwmPin The pin used for PWM control.
 * @param directionPin The pin used for direction control.
 * @param encoderPin The pin used for encoder input.
 * @param fwdDir The forward direction of the motor.
 */
Motor::Motor(PinName pwmPin, PinName directionPin, PinName encoderPin, forwardDirection fwdDir) :
 _pwmPin(pwmPin), _directionPin(directionPin), _encoderPin(encoderPin), _fwdDir(fwdDir) {
   
    //set up variables
    _encoderCount = 0;
    _lastEncoderCount = 0;
    _currentVelocity = 0;
    _TargetVelocity = 0;
    _TargetPosition = 0;
    _PIDerror = 0;
    _PIDlastError = 0;
    _PIDintegral = 0;
    _PIDderivative = 0;
    _PIDoutput = 0;
    _movementMode = STOP;

}

/**
 * @brief Sets up the motor by configuring PWM, direction pin, encoder interrupts, and velocity ticker.
 * 
 * @return int Returns OK if the setup is successful.
 */
int Motor::setup(){
    //set up pwm
    _pwmPin.period_us(PWM_PERIOD_US);
    _setPWM(0);

    //set up direction pin to forward (default)
    _directionPin.write(_fwdDir);

    //set up encoder interrupts
    _encoderPin.rise(callback(this, &Motor::_encoderISR));
    _encoderPin.fall(callback(this, &Motor::_encoderISR));

    //velocity ticker
    _velocityTicker.attach(callback(this, &Motor::_CalcVelocity), VEL_TICKER_PERIOD);

    //pid ticker
    _PIDTicker.attach(callback(this, &Motor::_PID), PID_TICKER_PERIOD);

    return OK;
}

int Motor::stop() {
    _movementMode = STOP;
    _setPWM(0);
    return OK;
}

int Motor::setVelocity(float velocity) {
    _setTargetVelocity(velocity); //Velocity in RPMs
    _movementMode = CONSTANT_VELOCITY;
    return OK;
}

int Motor::setPosition(int position) {
    _setTargetPosition(position);
    _movementMode = EXACT_POSITION;
    return OK;
}

int Motor::getCurrentVelocity() {
    return _currentVelocity;
}

int Motor::getCurrentPosition() {
    return _encoderCount;
}



/**
 * @brief Interrupt service routine for the motor encoder.
 * 
 * This function is called when an interrupt is triggered by the motor encoder.
 * It updates the encoder count based on the direction of rotation.
 * 
 * @note Forwards is relative to the robot, not the motor.
 *       The convention is that forwards is positive and backwards is negative.
 */
void Motor::_encoderISR() {
    //Forwards is relative to the robot, not the motor
    //convention is that forwards is positive and backwards is negative
    if (_directionPin.read() == _fwdDir) {
        _encoderCount++;
    } 
    else {
        _encoderCount--;
    }
}

/**
 * @brief PID control for the motor.
 * 
 * @note This function is called every PID_TICKER_PERIOD seconds.
 */
void Motor::_PID(){

    RobotMovementMode mode = _movementMode;

    switch (mode)
    {
        case STOP:
            _PIDoutput = 0;
            break;
        case CONSTANT_VELOCITY:
            _PIDerror = _TargetVelocity - _currentVelocity;
            _PIDintegral += _PIDerror * PID_TICKER_PERIOD;
            _PIDderivative = (_PIDerror - _PIDlastError) / PID_TICKER_PERIOD;
            _PIDoutput = (KP * _PIDerror) + (KI * _PIDintegral) + (KD * _PIDderivative);
            _PIDlastError = _PIDerror;
            break;
        case EXACT_POSITION:

            _PIDerror = _TargetPosition - _encoderCount;
            _PIDintegral += _PIDerror * PID_TICKER_PERIOD;
            _PIDderivative = (_PIDerror - _PIDlastError) / PID_TICKER_PERIOD;
            _PIDoutput = (KP * _PIDerror) + (KI * _PIDintegral) + (KD * _PIDderivative);
            _PIDlastError = _PIDerror;
            break;

        default:
            break;
    }


    //clamp pid output to +/- 0.4
    _PIDoutput = fmax(fmin(_PIDoutput, 0.7), -0.7);

    //set pwm
    _setPWM(_PIDoutput);



}


/**
 * @brief Calculates the current velocity of the motor.
 * 
 * @note This function is called every VEL_TICKER_PERIOD seconds.
 */
void Motor::_CalcVelocity() {
    //to be called on a ticker every VEL_TICKER_PERIOD seconds (currently `0.01` seconds)
    //calculate velocity
    _encoderDiff = _encoderCount - _lastEncoderCount;

    // divide by 10 as the ticker is called every 0.1 seconds
    _currentVelocity = _encoderDiff * VELOCITY_FACTOR;

    //update last encoder count
    _lastEncoderCount = _encoderCount;
    
}


/**
 * Sets the PWM value for the motor.
 * 
 * @param pwmVal The PWM value to set. Should be between -1 and 1.
 * @return Returns OK if the PWM value was set successfully, or ERR_DRIVE_PWM_FAIL if there was an error.
 */
int Motor::_setPWM(double pwmVal) {

    if (_movementMode == STOP) {
        pwmVal = 0;
        _pwmPin.write(STOP); //Actually set the PWM, spent 3 hours debugging this
        return OK;
    }

    //check pwm value is in range
    if (pwmVal > 1) {
        pwmVal = 1;
    }
    else if (pwmVal < -1) {
        pwmVal = -1;
    }

    //if pwm value is negative, set direction pin to backwards
    if (pwmVal < 0) {
        _directionPin.write(!_fwdDir);
    }
    else {
        _directionPin.write(_fwdDir);
    }

    _pwmPin.write(fabs(pwmVal));

    //check pwm value was wrote correctly
    if (_pwmPin.read() != pwmVal) {
        return ERR_DRIVE_PWM_FAIL;
    } else {
        return OK;
    }


}



int Motor::_setDirection(driveDirection dir) {
    if (dir == FORWARD) {
        _directionPin.write(_fwdDir);
    }
    else {
        _directionPin.write(!_fwdDir);
    }

    //check direction pin was wrote correctly
    if (_directionPin.read() != dir) {
        return ERR_DRIVE_DIRECTION_FAIL;
    } else {
        return OK;
    }

    return OK;
}

int Motor::_setTargetVelocity(float velocity) {

    //convert rpm to encoder counts per second
    _TargetVelocity = velocity * (ENC_CPR / 600.0f);
    return OK;
}

int Motor::_setTargetPosition(int position) {


    _TargetPosition = position;
    return OK;
}



int motorInit() {
    rightMotor.setup();
    leftMotor.setup();
}