/**
 * @file batfink.ino
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Main file for the batfink robot
 * @version 0.1
 * @date 16-01-2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "batfink.h"

using namespace mbed;
using namespace rtos;

Thread robotDriveThread(osPriorityNormal, 4096);





void setup(){

    //wait for serial connection
    Serial.begin(115200);
    ThisThread::sleep_for(1000);

    


    Serial.println("Starting up...");
    int err;

    err = batfinkRobot.setup();

    if (err != 0){
        Serial.print("Error setting up batfink: ");
        Serial.println(err);
    }

    initMaze();

    Serial.println("Setup complete");

   robotDriveThread.start(drive);



}

int messureSensorDistance(sensorType sensor){

    int distance = 0;

    //take 3 readings
    for(int i = 0; i < 3; i++){

        switch (sensor)
        {
        case FRONTSENSOR:
            distance += batfinkRobot._frontSensorDistancemm;
            break;
        case LEFTSENSOR:
            distance += batfinkRobot._leftSensorDistancemm;
            break;
        case RIGHTSENSOR:
            distance += batfinkRobot._rightSensorDistancemm;
            break;

        default:
            break;
        }

        ThisThread::sleep_for(300);
    }

    return distance / 3;

}


void drive(){

    bool start = true;

    while(1){

        int front = messureSensorDistance(FRONTSENSOR);
        int left = messureSensorDistance(LEFTSENSOR);
        int right = messureSensorDistance(RIGHTSENSOR);

        //print sensor values
        Serial.print("Front: ");
        Serial.print(front);
        Serial.print(" Left: ");
        Serial.print(left);
        Serial.print(" Right: ");
        Serial.println(right);

        if (start){
            //drive forward
            batfinkRobot.driveForward(100);
            start = false;
        }

        //if front sensor is less than 15cm
        else if (front < 150){
            //if left sensor is less than right sensor
            if (left < right){
                //turn left
                Serial.println("Turning Right 90");
                batfinkRobot.turnRight(90);
            } else {
                //turn right
                Serial.println("Turning Left 90");
                batfinkRobot.turnLeft(90);
            }
        }

        else {
            //drive forward
            //print drive distance
            int dist = (front - 50)/2;
            Serial.print("Driving forward: ");
            Serial.println(dist);
    
            batfinkRobot.driveForward(dist);
        }

    }

}

void printMaze() {
    Serial.println("========================================");
    for (auto &row : maze) {
        for (auto &cell : row) {
            switch (cell) {
                case UNKNOWN:
                    Serial.print("?");
                    break;
                case EMPTY:
                    Serial.print(" ");
                    break;
                case WALL:
                    Serial.print("X");
                    break;
            }
        }
        Serial.println();
    }
    Serial.println("========================================");
}



void loop(){

    /*
    //print robot x and y position
    Serial.print("X: ");
    Serial.print(batfinkRobot._XPOS);
    Serial.print(" Y: ");
    Serial.print(batfinkRobot._YPOS);
    Serial.print(" Theta: ");
    Serial.print(batfinkRobot._angleDeg);
    Serial.print(" Angle Err: ");
    Serial.print(batfinkRobot._AngleERR);
    Serial.print(" Direction: ");
    Serial.println(batfinkRobot._direction);
    //x y error
    Serial.print("X Error: ");
    Serial.print(batfinkRobot._XPOS_ERR);
    Serial.print(" Y Error: ");
    Serial.println(batfinkRobot._YPOS_ERR);
    //TARGETS
    Serial.print("X Target: ");
    Serial.print(batfinkRobot._XPOS_TARGET);
    Serial.print(" Y Target: ");
    Serial.println(batfinkRobot._YPOS_TARGET);
    //left and right motors theta and target theta
    */

    /*
    //print left and right encoder counts and targets
    Serial.print("Left Encoder Count: ");
    Serial.print(batfinkRobot._leftMotor._encoderCount);
    Serial.print(" Target: ");
    Serial.print(batfinkRobot._leftMotor._TargetPosition);
    Serial.print(" Err: ");
    Serial.print(batfinkRobot._leftMotor._PIDerror);
    Serial.print(" Right Encoder Count: ");
    Serial.print(batfinkRobot._rightMotor._encoderCount);
    Serial.print(" Target: ");
    Serial.print(batfinkRobot._rightMotor._TargetPosition);
    Serial.print(" Err: ");
    Serial.println(batfinkRobot._rightMotor._PIDerror);

    //print left and right motor modes
    Serial.print("Left Motor Mode: ");
    Serial.print(batfinkRobot._leftMotor._movementMode);
    Serial.print(" Right Motor Mode: ");
    Serial.println(batfinkRobot._rightMotor._movementMode);

    //print left and right motor PID values
    Serial.print("Left Motor PID: ");
    Serial.print(batfinkRobot._leftMotor._PIDoutput);
    Serial.print(" Right Motor PID: ");
    Serial.println(batfinkRobot._rightMotor._PIDoutput);

    */

   printMaze();




    //crashing here
  
    //wait for 1 second 
    ThisThread::sleep_for(2000);


}