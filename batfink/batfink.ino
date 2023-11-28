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
    //print sensor data
    Serial.print("Front: ");
    Serial.print(sensorData->front);
    Serial.print(" Left: ");
    Serial.print(sensorData->left);
    Serial.print(" Right: ");
    Serial.println(sensorData->right);
    sensorMutex.unlock();


    //non blocking delay
    rtos::ThisThread::sleep_for(1000);
}