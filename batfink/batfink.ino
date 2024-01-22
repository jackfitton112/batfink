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

    err = bleSetup();

    if (err != 0){
        Serial.print("Error setting up bluetooth: ");
        Serial.println(err);
    }

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

    ThisThread::sleep_for(2000);

    while(1){

    

        while(SerialConnected == 1){
            ThisThread::sleep_for(100);
        }

        int front = batfinkRobot._frontSensorDistancemm;
        int left = batfinkRobot._leftSensorDistancemm;
        int right = batfinkRobot._rightSensorDistancemm;

        if (front < 30 || left < 30 || right < 30){
            EmergencyOverride = 1;
            if (front < 30){
               //check if left or right is less
                if (left < right){
                     //turn right
                     Serial.println("Turning Right 45");
                     batfinkRobot.turnRight(90);
                } else {
                     //turn left
                     Serial.println("Turning Left 45");
                     batfinkRobot.turnLeft(90);
                }

            } else if (left < 50){
                //turn right
                Serial.println("Turning Right 90");
                batfinkRobot.turnRight(45);
            } else if (right < 50){
                //turn left
                Serial.println("Turning Left 90");
                batfinkRobot.turnLeft(45);

            }
        } else {
            EmergencyOverride = 0;
        }

        ThisThread::sleep_for(100);
    }

}

void drive() {
    // Wait 2s
    ThisThread::sleep_for(2000);

    // Initialize the previous left distance
    int previousLeft = 0;

    while(1) {
        // Check for emergency override or serial connection
        while(EmergencyOverride == 1 || SerialConnected == 1){
            ThisThread::sleep_for(100);
        }

        int fronts = batfinkRobot._frontSensorDistancemm;
        int lefts = batfinkRobot._leftSensorDistancemm;
        
        // Print sensor values
        Serial.print("Front: ");
        Serial.print(fronts);
        Serial.print(" Left: ");
        Serial.println(lefts);

        // Desired distance from the left wall
        int desiredDistancemin = 70;
        int desiredDistancemax = 150;

        // Check if the robot has passed the edge of the wall
        if (previousLeft <= 100 && lefts > 200) {
            Serial.println("Passed Wall Edge - Turning Left 90");
            batfinkRobot.turnLeft(90);
            batfinkRobot.driveForward(150);
            batfinkRobot.turnLeft(90);
        } else if (fronts < 12) {
            // Avoid obstacle by turning right
            Serial.println("Obstacle Ahead - Turning Right 90");
            batfinkRobot.turnRight(90);
        } else {
            // Adjust the robot's position relative to the left wall
            if (lefts < desiredDistancemin) {
                // Turn left
                Serial.println("Turning Left 45");
                batfinkRobot.turnLeft(45);
            } else if (lefts > desiredDistancemax) {
                // Turn right
                Serial.println("Turning Right 45");
                batfinkRobot.turnRight(45);
            } else {
                // Continue driving forward
                Serial.println("Driving Forward");
                batfinkRobot.driveForward(100); // Adjust the driving distance as needed
            }
        }

        // Update the previous left distance
        previousLeft = lefts;
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