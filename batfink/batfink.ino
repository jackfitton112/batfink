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
Thread OverrideThread(osPriorityNormal, 4096); 

int EmergencyOverride = 0;
int SerialConnected = 0;



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
    OverrideThread.start(obsticleAvoidance);



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


void obsticleAvoidance(){

    while(1){

        while(SerialConnected == 1){
            ThisThread::sleep_for(100);
        }

        int front = batfinkRobot._frontSensorDistancemm;
        int left = batfinkRobot._leftSensorDistancemm;
        int right = batfinkRobot._rightSensorDistancemm;

        if (front < 100 || left < 50 || right < 50){
            EmergencyOverride = 1;
            if (front < 100){
               //check if left or right is less
                if (left < right){
                     //turn right
                     Serial.println("Turning Right 90");
                     batfinkRobot.turnRight(90);
                } else {
                     //turn left
                     Serial.println("Turning Left 90");
                     batfinkRobot.turnLeft(90);
                }

            } else if (left < 50){
                //turn right
                Serial.println("Turning Right 90");
                batfinkRobot.turnRight(90);
            } else if (right < 50){
                //turn left
                Serial.println("Turning Left 90");
                batfinkRobot.turnLeft(90);

            }
        } else {
            EmergencyOverride = 0;
        }

        ThisThread::sleep_for(100);
    }

}

void drive(){


    //wait 2s
    ThisThread::sleep_for(2000);


    while(1){

        while(EmergencyOverride == 1 || SerialConnected == 1){
            ThisThread::sleep_for(100);
        }

        int front = messureSensorDistance(FRONTSENSOR);
        //int left = messureSensorDistance(LEFTSENSOR);
        //int right = messureSensorDistance(RIGHTSENSOR);

        //print sensor values
        Serial.print("Front: ");
        Serial.print(front);



        //if front sensor is less than 15cm
        if (front < 100){
            //if left sensor is less than right sensor
            int left = batfinkRobot._leftSensorDistancemm;
            int right = batfinkRobot._rightSensorDistancemm;
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
            int dist = (front - 90); //drive just into the turning state
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

    //if serial is connected, stop the robot, stop the mapping and print the maze

    if (Serial.available() > 0){
        map_hold = 0;
        Serial.println("Serial connected, stopping robot");
        batfinkRobot.stop();
        Serial.println("Printing maze");
        printMaze();
        Serial.println("Stopping");
        while(Serial.available() > 0){
            ThisThread::sleep_for(100);\
        }
        map_hold = 1;
    }
  
    //wait for 1 second 
    ThisThread::sleep_for(500);


}