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
#include "mbed.h"
//#include "src/batfink/batfink.h"
#include "src/batfink/pins.h"
#include "src/MotorControl/MotorControl.h"
#include "src/usonic/usonic.h"
//#include "motors.h"
//#include "drive.h"

Motor leftMotor(MBED_MOTOR_PWMA, MBED_MOTOR_DIRA, MBED_MOTOR_ENCA, CCW);
Motor rightMotor(MBED_MOTOR_PWMB, MBED_MOTOR_DIRB, MBED_MOTOR_ENCB, CW);


void setup(){

    //setup motors
    //Motor_setup();
    leftMotor.setup();
    rightMotor.setup();
    //setup usonic
    //Usonic_setup();

    Serial.begin(9600);

    //set motortarget velocity in wheel rev/min
    leftMotor.setTargetVel(10);
    rightMotor.setTargetVel(10);

}





void loop() {

    //print motor velocity
    Serial.print("Motor Velocity: ");
    Serial.println(leftMotor.getVel());

    //non blocking delay
    rtos::ThisThread::sleep_for(100);
}