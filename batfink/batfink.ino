/**
 * @file batfink.ino
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Main file for batfink project
 * @version 0.1
 * @date 28-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "src/batfink/batfink.h"
#include "src/MotorControl/MotorControl.h"
#include "src/usonic/usonic.h"
//#include "motors.h"
//#include "drive.h"


void setup(){

    //setup motors
    Motor_setup();

    //setup usonic
    Usonic_setup();

    Serial.begin(9600);



}





void loop() {


    //MAIN CONTROL LOOP

    //take mutex and get the sensor data
    sensorMutex.lock();
    int front = sensorData->front;
    sensorMutex.unlock();

    if (front < 10){
        //stop the robot
        motorMutex.lock();
        drive_direction = 0;
        motorMutex.unlock();
  
    } else {

    //drive forward to 5cm away from wall
    driveDistance(front - 5);


    }

    //non blocking delay
    rtos::ThisThread::sleep_for(1000);
}