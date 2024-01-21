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
    _movementMode = STP;

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
    _movementMode = STP;
    _setPWM(0);
    if (stoppedCallback) {
        stoppedCallback();
    }
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

void Motor::onStopped(const MotorCallback& callback) {
        stoppedCallback = callback;
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
 * if the motor is in STP mode, the PID output is set to 0.
 * if the motor is in CONSTANT_VELOCITY mode, the PID output is set to the output of the PID controller.
 */
void Motor::_PID(){

    RobotMovementMode mode = _movementMode;

    switch (mode)
    {
        case STP:
            _PIDoutput = 0;
            break;
        case CONSTANT_VELOCITY:

            //if target position is set, and the motor is within 1/2 a rotation of the target position, switch to EXACT_POSITION mode
            if (_TargetPosition != 0 && abs(_TargetPosition - _encoderCount) < ENC_CPR / 2) {
                //reset integral and derivative as not to cause a spike in output
                _PIDintegral = 0;
                _PIDderivative = 0;

                //set last error to current target 0encoder count
                _PIDlastError = _TargetPosition - _encoderCount;
                
                
                _movementMode = EXACT_POSITION;
            }


            //set PID values for constant velocity, cap output at +/- 0.5
            _PIDerror = _TargetVelocity - _currentVelocity;
            _PIDintegral += _PIDerror * PID_TICKER_PERIOD;
            _PIDderivative = (_PIDerror - _PIDlastError) / PID_TICKER_PERIOD;
            _PIDoutput = _PIDerror * VELKP + _PIDintegral * VELKI + _PIDderivative * VELKD;

            //cap output at +/- 0.5
            _PIDoutput = fmax(fmin(_PIDoutput, 0.7), -0.7);

            //update last error
            _PIDlastError = _PIDerror;
            
            break;

        case EXACT_POSITION:

            //if target is more than 2 rotations of the wheel away, switch to CONSTANT_VELOCITY mode
            if (abs(_TargetPosition - _encoderCount) > ENC_CPR * 2) {
                //reset integral and derivative as not to cause a spike in output
                _PIDintegral = 0;
                _PIDderivative = 0;

                setVelocity(10);

                //set last error to current target 0encoder count
                _PIDlastError = _TargetVelocity - _currentVelocity;
                _movementMode = CONSTANT_VELOCITY;
            }

            //set PID values for exact position control, cap output at +/- 0.3
            _PIDerror = _TargetPosition - _encoderCount;
            _PIDintegral += _PIDerror * PID_TICKER_PERIOD;
            _PIDderivative = (_PIDerror - _PIDlastError) / PID_TICKER_PERIOD;
            _PIDoutput = _PIDerror * POSKP + _PIDintegral * POSKI + _PIDderivative * POSKD;

            //cap output at +/- 0.3 if the motor is within 1/2 a rotation of the target position
            if (abs(_TargetPosition - _encoderCount) < ENC_CPR / 2) {
                _PIDoutput = fmax(fmin(_PIDoutput, 0.2), -0.2);
            }
            //if its within 1/4 of a rotation, cap output at +/- 0.1
            else if (abs(_TargetPosition - _encoderCount) < ENC_CPR / 4) {
                //if error == previous error (not moving), increase torque compensation by 0.01
                if (_PIDerror == _PIDlastError) {
                  _torqueCompensation += 0.01;
                }

                _PIDoutput = fmax(fmin(_PIDoutput, _torqueCompensation), -_torqueCompensation);
            }

            else {
                _PIDoutput = fmax(fmin(_PIDoutput, 0.3), -0.3);
                }

            //update last error
            _PIDlastError = _PIDerror;



            break;


        case TURNING:

            //only used when the robot is turning, as it turns on the spot there is never a velocity demand#

            //set PID values for turning, cap output at +/- 0.5
            _PIDerror = _TargetPosition - _encoderCount;
            _PIDintegral += _PIDerror * PID_TICKER_PERIOD;
            _PIDderivative = (_PIDerror - _PIDlastError) / PID_TICKER_PERIOD;
            _PIDoutput = _PIDerror * TURNINGKP + _PIDintegral * TURNINGKI + _PIDderivative * TURNINGKD;
            if (abs(_TargetPosition - _encoderCount) < ENC_CPR / 2) {
                _PIDoutput = fmax(fmin(_PIDoutput, 0.2), -0.2);
            }
            else if (abs(_TargetPosition - _encoderCount) < ENC_CPR / 2) {
                //if error == previous error (not moving), increase torque compensation by 0.01
                if (_PIDerror == _PIDlastError) {
                  _torqueCompensation += 0.01;
                }

                _PIDoutput = fmax(fmin(_PIDoutput, _torqueCompensation), -_torqueCompensation);
            }

            else {
                _PIDoutput = fmax(fmin(_PIDoutput, 0.3), -0.3);
                }



            //update last error
            _PIDlastError = _PIDerror;

            break;


        default:
            break;

    }


    //if PID error is less than 10 encoder counts, switch to STP mode
    if (abs(_TargetPosition - _encoderCount) < 45 && _TargetPosition != 0) {
        _movementMode = STP;
        _PIDoutput = 0;
        stop();
    }

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

    //the convention is that forwards is positive and backwards is negative
    if (pwmVal > 0) {
        _setDirection(FWD);
    }
    else {
        _setDirection(BKWD);
    }

    //set pwm
    _pwmPin.write(abs(pwmVal));


}



int Motor::_setDirection(driveDirection dir) {
    if (dir == FWD) {
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




