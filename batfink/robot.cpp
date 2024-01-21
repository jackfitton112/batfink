/**
 * @file robot.cpp
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Robot module for the batfink robot source file
 * @version 0.1
 * @date 17-01-2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "robot.h"


//declare robot object
Batfink batfinkRobot(leftMotor, rightMotor, frontSensor, leftSensor, rightSensor);





/**
 * @brief Construct a new Robot:: Robot object
 * 
 * @param leftMotor 
 * @param rightMotor 
 * @param frontSensor 
 * @param leftSensor 
 * @param rightSensor 
 */
Batfink::Batfink(Motor& leftMotor, Motor& rightMotor, Sensor& frontSensor, Sensor& leftSensor, Sensor& rightSensor) : _leftMotor(leftMotor), _rightMotor(rightMotor), _frontSensor(frontSensor), _leftSensor(leftSensor), _rightSensor(rightSensor){
    _XPOS = 0;
    _YPOS = 0;
    _XPOS_TARGET = 0;
    _YPOS_TARGET = 0;
    _THETA_RAD = 0;

}

/**
 * @brief Setup the robot
 * 
 * @return int 
 */
int Batfink::setup(){

    


    //setup motors
    _leftMotor.setup();
    _rightMotor.setup();

    _leftMotor.onStopped(callback(this, &Batfink::_onStopped));
    _rightMotor.onStopped(callback(this, &Batfink::_onStopped));

    //setup sensors
    _frontSensor.setup();
    _leftSensor.setup();
    _rightSensor.setup();

    //setup XYT ticker
    _XYTTicker.attach(callback(this, &Batfink::_updateXYT), XYT_UPDATE_INTERVAL);

    //setup sensor ticker
    _sensorTicker.attach(callback(this, &Batfink::_updateSensorValues), 0.25);

    return OK;
}

void Batfink::_onStopped(){
    //if the robot is stopped, set the movement mode to stopped
    if (_leftMotor._movementMode == STP && _rightMotor._movementMode == STP){
        //robot has stopped, carry on with the next movement
        _direction = STOP;
    }
}


/**
 * @brief Drive the robot forward
 * 
 * @param distancemm 
 * @return int 
 */
int Batfink::driveForward(int distancemm){
    return _drive(distancemm, FORWARD);
}

/**
 * @brief Drive the robot backward
 * 
 * @param distancemm 
 * @return int 
 */
int Batfink::driveBackward(int distancemm){
    return _drive(distancemm, BACKWARD);
}

/**
 * @brief Turn the robot left
 * 
 * @param angle 
 * @return int 
 */
int Batfink::turnLeft(int angle){
    _direction = LEFT;
    return _turn(angle, LEFT);
}

/**
 * @brief Turn the robot right
 * 
 * @param angle 
 * @return int 
 */
int Batfink::turnRight(int angle){
    _direction = RIGHT;
    return _turn(angle, RIGHT);
}

/**
 * @brief Stop the robot
 * 
 * @return int 
 */
int Batfink::stop(){
    _direction = STOP;
    _leftMotor.stop();
    _rightMotor.stop();
    return OK;
}

int Batfink::move(int direction, int distance){
    switch(direction){
        case 0:
            return stop();
        case 1:
            return driveForward(distance);
        case 2:
            return driveBackward(distance);
        case 3:
            return turnLeft(distance);
        case 4:
            return turnRight(distance);
        default:
            return ERR_ROBOT_BLE_MOVE_FAIL;
    }
    return OK;
}



/**
 * @brief Drive the robot forward or backward
 * 
 * @param distancemm 
 * @param direction 
 * @return int 
 */
int Batfink::_drive(int distancemm, robotDriveDirection direction){



    float encoderCounts = _helper_CalculateEncoderCountFromDistance(distancemm);

    encoderCounts *= 1.05f; //1.05f is the offset to account for slippage

    //set direction
    _direction = direction;

    //set movement mode
    leftMotor._movementMode = EXACT_POSITION;
    rightMotor._movementMode = EXACT_POSITION;


    //set target velocity
    if (direction == FORWARD){
        _leftMotor._TargetPosition = _leftMotor._encoderCount + encoderCounts;
        _rightMotor._TargetPosition = _rightMotor._encoderCount + encoderCounts;
    } else {
        _leftMotor._TargetPosition = _leftMotor._encoderCount - encoderCounts;
        _rightMotor._TargetPosition = _rightMotor._encoderCount - encoderCounts;

    }

    while(_leftMotor._movementMode != STP && _rightMotor._movementMode != STP){
        ThisThread::sleep_for(100);
    }


    return OK;

}

/**
 * @brief Turn the robot left or right
 * 
 * @param angle //Degrees
 * @param direction 
 * @return int 
 */
int Batfink::_turn(int angle, robotDriveDirection direction){
    //calculate encoder counts
    

    float encoderCounts = _helper_CalculateEncoderCountsFromAngle(angle);

    //set direction
    _direction = direction;

    encoderCounts *= TURN_OFFSET; //1.2f is the offset to account for slippage

    leftMotor._movementMode = TURNING;
    rightMotor._movementMode = TURNING;


    //if direction is left, set left motor to reverse and right motor to forward
    //for turn left and right as it spins on the spot there is little need for a velocity demand
    if (direction == LEFT){
        _leftMotor._TargetPosition = _leftMotor._encoderCount - encoderCounts;
        _rightMotor._TargetPosition = _rightMotor._encoderCount + encoderCounts;
    } else {
        _leftMotor._TargetPosition = _leftMotor._encoderCount + encoderCounts;
        _rightMotor._TargetPosition = _rightMotor._encoderCount - encoderCounts;
    }



    while(_leftMotor._movementMode != STP && _rightMotor._movementMode != STP){
        ThisThread::sleep_for(100);
    }

    return OK;
}

/**
 * @brief Update the X,Y,Theta values of the robot
 * 
 */
void Batfink::_updateXYT(){



    //calculate encoder diff
    _leftEncoderDiff = _leftMotor._encoderCount - _leftPrevEncoderCount;
    _rightEncoderDiff = _rightMotor._encoderCount - _rightPrevEncoderCount;

    //update encoder counts
    _leftPrevEncoderCount = _leftMotor._encoderCount;
    _rightPrevEncoderCount = _rightMotor._encoderCount;


    float distanceTravelled = _helper_CalculateDistanceFromEncoder((_leftEncoderDiff + _rightEncoderDiff) / 2.0f);

    // Use the locally calculated differences
    double angleTravelled = _helper_CalculateAngleFromEncoder(_leftEncoderDiff, _rightEncoderDiff);
   // _DEBUGVAL = angleTravelled;

    //calculate X,Y,Theta without converting angle to degrees
    _XPOS += (distanceTravelled * cos(_THETA_RAD + (angleTravelled / 2))) * 1.08f; //1.05f is the offset to account for slippage
    _YPOS += distanceTravelled * sin(_THETA_RAD + (angleTravelled / 2)) * 1.08f; //is the offset to account for slippage
    _THETA_RAD += angleTravelled;

    // Normalize the _THETA_RAD to stay within -π to π
    while(_THETA_RAD > PI) _THETA_RAD -= 2 * PI;
    while(_THETA_RAD < -PI) _THETA_RAD += 2 * PI;


    //convert theta to degrees
    _angleDeg = _THETA_RAD * (180.0f / PI);



}

/**
 * @brief Calculate the distance travelled from the encoder count
 * 
 * @param encoderCount 
 * @return float 
 */
float Batfink::_helper_CalculateDistanceFromEncoder(float encoderCount){
    return ((encoderCount *1.0f) / ENC_CPR) * (PI * WHEEL_DIA);
}

/**
 * @brief Calculate the encoder count from the distance travelled
 * 
 * @param distancemm 
 * @return float 
 */
float Batfink::_helper_CalculateEncoderCountFromDistance(float distancemm){
    return ((float)distancemm  / (PI * WHEEL_DIA)) * ENC_CPR;
}

/**
 * @brief Calculate the angle travelled from the encoder count
 * 
 * @param leftEncoderCount 
 * @param rightEncoderCount 
 * @return float 
 */
double Batfink::_helper_CalculateAngleFromEncoder(int leftEncoderCount, int rightEncoderCount){
    _leftDistance = ((float)leftEncoderCount / ENC_CPR) * (PI * WHEEL_DIA);
    _rightDistance = ((float)rightEncoderCount / ENC_CPR) * (PI * WHEEL_DIA);
    _DEBUGVAL = ((float)_leftDistance - (float)_rightDistance) / TRACK_WIDTH; // This is in radians
    return (((float)_leftDistance - (float)_rightDistance) / TRACK_WIDTH) *1.0f; // This is in radians
}


/**
 * @brief Calculate the encoder count from the angle travelled
 * 
 * @param angle 
 * @return float 
 */
float Batfink::_helper_CalculateEncoderCountsFromAngle(float angle){
    // Convert angle to radians
    float angleRadians = angle * (PI / 180.0f);
    // Calculate the arc length that each wheel needs to travel
    float arcLengthPerWheel = angleRadians * (TRACK_WIDTH / 2.0f);
    // Convert this arc length into encoder counts
    return (arcLengthPerWheel / (PI * WHEEL_DIA)) * ENC_CPR;
}




void Batfink::_updateSensorValues(){
    _frontSensorDistancemm = _frontSensor.getDistance();
    _leftSensorDistancemm = _leftSensor.getDistance();
   _rightSensorDistancemm = _rightSensor.getDistance();
}

