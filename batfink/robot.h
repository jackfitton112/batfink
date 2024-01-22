/**
 * @file robot.h
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Robot module for the batfink robot header file
 * @version 0.1
 * @date 17-01-2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "sensor.h"
#include "motor.h"
#include "error.h"


#include <mbed/mbed.h>

#define WHEEL_DIA 50.7f //mm
#define TRACK_WIDTH 146.9f //mm
#define PI 3.14159265358979323846f
#define XYT_UPDATE_INTERVAL 0.1 //s
#define TURN_OFFSET 1.00f //0% error in turning


enum robotDriveDirection {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP,
};


class Batfink {


    //public functions
    public:
        Batfink(Motor &leftMotor, Motor &rightMotor, Sensor &frontSensor, Sensor &leftSensor, Sensor &rightSensor);
        int setup();
        int driveForward(int distancemm);
        int driveBackward(int distancemm);
        int turnLeft(int angle);
        int turnRight(int angle);
        int stop();
        int move(int direction, int distance);

    //private:

        //private functions
        int _drive(int distancemm, robotDriveDirection direction);
        int _turn(int angle, robotDriveDirection direction);

        float _helper_CalculateDistanceFromEncoder(float encoderCount);
        float _helper_CalculateEncoderCountFromDistance(float distancemm);
        float _helper_CalculateEncoderCountsFromAngle(float angle);
        double _helper_CalculateAngleFromEncoder(int leftEncoderCount, int rightEncoderCount);


        Ticker _XYTTicker; //X,Y,Theta ticker
        void _updateXYT();

        //private variables

        //X and Y pos, theta in radians
        int _XPOS;
        int _YPOS;
        int _XPOS_TARGET;
        int _YPOS_TARGET;
        int _XPOS_ERR;
        int _YPOS_ERR;
        double _THETA_RAD; //theta in radians
        double _AngleERR;
        robotDriveDirection _direction;


        

        //motor objects
        Motor &_leftMotor;
        Motor &_rightMotor;
        int _leftPrevEncoderCount;
        int _rightPrevEncoderCount;
        int _leftEncoderDiff;
        int _rightEncoderDiff;
        float _leftDistance;
        float _rightDistance;
        void _onStopped();
        

        //sensor objects
        Ticker _sensorTicker;
        Sensor &_frontSensor;
        Sensor &_leftSensor;
        Sensor &_rightSensor;
        float _frontSensorDistancemm;
        float _leftSensorDistancemm;
        float _rightSensorDistancemm;
        void _updateSensorValues();
    

        //error var
        int _ERR;
        int _DEBUGVAL;
        int _angleDeg;

   
};



extern Batfink batfinkRobot;





#endif // ROBOT_H