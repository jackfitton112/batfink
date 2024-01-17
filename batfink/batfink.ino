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



void setup(){

    //wait for serial connection
    Serial.begin(115200);
    //while(!Serial);


    //setup motors
    //motorInit();


    leftMotor.setup();
    rightMotor.setup();


    //set velocity
    leftMotor.setVelocity(10);
    rightMotor.setVelocity(10);
  

}
void loop(){

    
    //print target and velocity
    Serial.print("Target velocity: ");
    Serial.println(leftMotor._TargetVelocity);
    Serial.print("Current velocity: ");
    Serial.println(leftMotor._currentVelocity);

    //print PID output
    Serial.print("PID output: ");
    Serial.println(leftMotor._PIDoutput);

    //print pid values
    Serial.print("PID error: ");
    Serial.println(leftMotor._PIDerror);
    Serial.print("PID integral: ");
    Serial.println(leftMotor._PIDintegral);
    Serial.print("PID derivative: ");
    Serial.println(leftMotor._PIDderivative);

    //print encoder count values
    Serial.print("Encoder count: ");
    Serial.println(leftMotor._encoderCount);
    Serial.print("Last encoder count: ");
    Serial.println(leftMotor._lastEncoderCount);
    Serial.print("Encoder diff: ");
    Serial.println(leftMotor._encoderDiff);

    //print movement mode
    Serial.print("Movement mode: ");
    Serial.println(leftMotor._movementMode);




    rtos::ThisThread::sleep_for(500);



}