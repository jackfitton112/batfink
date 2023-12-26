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
    _currentPosition = 0;
    _TargetVelocity = 0;
    _TargetPosition = 0;
    _PIDerror = 0;
    _PIDlastError = 0;
    _PIDintegral = 0;
    _PIDderivative = 0;
    _PIDoutput = 0;
    _PIDenabled = false;
    _movementMode = STOP;
    _driveMode = CONSTANT_VELOCITY;
    _ERR = OK;

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

    //set up velocity ticker
    _velocityTicker.attach(callback(this, &Motor::_calculateCurrentVelocity), VEL_TICKER_PERIOD);
    _PIDTicker.attach(callback(this, &Motor::_PID), PID_TICKER_PERIOD);

    return OK;
}

/**
 * Sets the drive mode of the motor.
 * 
 * @param driveMode The desired drive mode.
 * @return Returns ERR_DRIVE_GENERAL if the drive mode was not set correctly, otherwise returns OK.
 */
int Motor::setDriveMode(RobotDriveMode driveMode) {
    _driveMode = driveMode;

    //check drive mode was set correctly
    if (_driveMode != driveMode) {
        return ERR_DRIVE_GENERAL;
    } else {
        return OK;
    }
}

/**
 * @brief Sets the target velocity for the motor.
 * 
 * This function sets the target velocity for the motor. It updates the private member variable _TargetVelocity
 * with the provided value. It also performs a check to ensure that the target velocity was set correctly.
 * 
 * @param targetVelocity The desired target velocity for the motor.
 * @return Returns ERR_DRIVE_VELOCITY_FAIL if the target velocity was not set correctly, otherwise returns OK.
 */
int Motor::setTargetVelocity(float targetVelocity) {

    
    _TargetVelocity = targetVelocity;

    //check target velocity was set correctly
    if (_TargetVelocity != targetVelocity) {
        return ERR_DRIVE_VELOCITY_FAIL;
    } else {
        return OK;
    }

}

/**
 * Sets the target position for the motor.
 * 
 * @param targetPosition The desired target position in units of revolutions (1 = 1 rotatation of the wheel).
 * @return Returns ERR_DRIVE_POSITION_FAIL if the target position was not set correctly, otherwise returns OK.
 */
int Motor::setTargetPosition(float targetPosition) {
    _TargetPosition = targetPosition * ENC_CPR;

    //check target position was set correctly
    if (_TargetPosition != targetPosition) {
        return ERR_DRIVE_POSITION_FAIL;
    } else {
        return OK;
    }
}

/**
 * Sets the movement mode of the motor.
 * 
 * @param movementMode The desired movement mode.
 * @return Returns ERR_DRIVE_GENERAL if the movement mode was not set correctly, otherwise returns OK.
 */
int Motor::setMovementMode(RobotMovementMode movementMode) {
    _movementMode = movementMode;

    //check movement mode was set correctly
    if (_movementMode != movementMode) {
        return ERR_DRIVE_GENERAL;
    } else {
        return OK;
    }
}

/**
 * Enables the PID control for the motor.
 * 
 * @return OK if the PID control was enabled successfully, ERR_DRIVE_PID_FAIL otherwise.
 */
int Motor::enablePID() {
    _PIDenabled = true;

    //check PID was enabled correctly
    if (_PIDenabled == true) {
        return OK;
    } else {
        return ERR_DRIVE_PID_FAIL;
    }
}

/**
 * Disables the PID control for the motor.
 * 
 * @return OK if the PID control was disabled successfully, ERR_DRIVE_PID_FAIL otherwise.
 */
int Motor::disablePID() {
    _PIDenabled = false;

    //check PID was disabled correctly
    if (_PIDenabled == false) {
        return OK;
    } else {
        return ERR_DRIVE_PID_FAIL;
    }
}

/**
 * Starts the motor.
 * 
 * @return Returns an error code if the movement mode was not set correctly, otherwise returns OK.
 */
int Motor::startMotor() {
    _movementMode = START;

    //check movement mode was set correctly
    if (_movementMode != START) {
        return ERR_DRIVE_GENERAL;
    } else {
        return OK;
    }
}

/**
 * Stops the motor.
 * 
 * @return Returns OK if the motor is successfully stopped, otherwise returns ERR_DRIVE_GENERAL.
 * 
 */
int Motor::stopMotor() {
    _movementMode = STOP;

    //check movement mode was set correctly
    if (_movementMode != STOP) {
        return ERR_DRIVE_GENERAL;
    } else {
        return OK;
    }
}

/**
 * Sets the direction of the motor.
 * 
 * @param dir The desired drive direction (FORWARD or BACKWARD).
 * @return Returns OK if the direction pin was set correctly, otherwise returns ERR_DRIVE_GENERAL.
 */
int Motor::_setDirection(driveDirection dir) {
    if (dir == FORWARD) {
        _directionPin.write(_fwdDir);
    }
    else {
        _directionPin.write(!_fwdDir);
    }

    //check direction pin was set correctly
    if (_directionPin.read() != dir) {
        return ERR_DRIVE_GENERAL;
    } else {
        return OK;
    }
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
 * Sets the PWM value for the motor.
 * 
 * @param pwmVal The PWM value to set. Should be between -1 and 1.
 * @return Returns OK if the PWM value was set successfully, or ERR_DRIVE_PWM_FAIL if there was an error.
 */
int Motor::_setPWM(double pwmVal) {

    if (_movementMode == STOP) {
        pwmVal = 0;
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

/**
 * @brief Calculates the current velocity of the motor.
 * 
 * This function is called by the velocity ticker. It calculates the current velocity of the motor
 * based on the encoder count and the time between ticks.
 * 
 * @note This function is called every VEL_TICKER_PERIOD seconds.
 */
void Motor::_calculateCurrentVelocity() {

    //calculate velocity
    _encoderDiff = _encoderCount - _lastEncoderCount;
    _currentVelocity = (_encoderDiff / VEL_TICKER_PERIOD) * 60 / ENC_CPR;
    _lastEncoderCount = _encoderCount;
}

/**
 * @brief PID control for the motor.
 * 
 * This function is called by the PID ticker. It calculates the PID output for the motor
 * based on the current velocity and the target velocity.
 * 
 * @note This function is called every PID_TICKER_PERIOD seconds.
 */
void Motor::_PID(){

    if (_PIDenabled == false) {
        return;
    }

    else {

        if (_driveMode == CONSTANT_VELOCITY) {
            //Constant Velocity PID
            _PIDerror = _TargetVelocity - _currentVelocity;
            _PIDintegral += _PIDerror * VEL_TICKER_PERIOD;
            _PIDderivative = (_PIDerror - _PIDlastError) / VEL_TICKER_PERIOD;
            _PIDlastError = _PIDerror;
            _PIDoutput = (_PIDerror * KP) + (_PIDintegral * KI) + (_PIDderivative * KD);
        }

        else if (_driveMode == EXACT_POSITION) {
            //Exact Position PID
            _currentPosition = _encoderCount;
            _PIDerror = _TargetPosition - _currentPosition;
            _PIDintegral += _PIDerror * PID_TICKER_PERIOD;
            _PIDderivative = (_PIDerror - _PIDlastError) / PID_TICKER_PERIOD;
            _PIDlastError = _PIDerror;
            _PIDoutput = (_PIDerror * KP);
        }

        else {
            _ERR = ERR_DRIVE_WRNG_MODE;
            return;
        }

        _setPWM(_PIDoutput);

    }
}

/**
 * @brief Initialises the motors.
 * 
 * This function initialises the motors by calling the setup() and start() functions.
 * 
 * @return Returns OK if the motors were initialised successfully, otherwise returns ERR_DRIVE_INIT_FAIL.
 */
int motorInit() {
    //initialise motors
    rightMotor.setup();
    leftMotor.setup();

    //start motors
    rightMotor.start();
    leftMotor.start();

    return OK;
}