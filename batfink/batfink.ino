//includes 
#include <Arduino.h>
#include <mbed.h>
#include <ArduinoBLE.h>
#include "pins.h"
#include "MotorControl.h"
#include "usonic.h"
#include "motors.h"
#include "ble.h"

void setup(){

    //setup motors
    Motor_setup();

    //setup bluetooth
    setup_ble();

    //start the poll thread
    ble.start(ble_thread);
    ble_worker.start(ble_worker_thread);

    Serial.begin(9600);

    //start usonic thread
    frontusonicThread.start(frontusonicRead);
    leftusonicThread.start(leftusonicRead);
    rightusonicThread.start(rightusonicRead);

    //start motor thread
    motorThread.start(motorDrive);


}


void turn_left(){
        //turn left
        motorMutex.lock();
        drive_direction = 3;
        motorMutex.unlock();

        //wait for 1 second
        rtos::ThisThread::sleep_for(2000);

        //stop
        motorMutex.lock();
        drive_direction = 0;
        motorMutex.unlock();
}

void turn_right(){
        //turn right
        motorMutex.lock();
        drive_direction = 4;
        motorMutex.unlock();

        //wait for 1 second
        rtos::ThisThread::sleep_for(2000);

        //stop
        motorMutex.lock();
        drive_direction = 0;
        motorMutex.unlock();

}

void go_forward(){
        //go forward
        motorMutex.lock();
        drive_direction = 1;
        motorMutex.unlock();

}

void rave_mode(){
    //make the robot rave
    turn_left();
    turn_right();

    //wait for 0.5 seconds
    rtos::ThisThread::sleep_for(500);
    
}


void loop() {

    //check if override is 1, if it is, stop the robot
    int isOverride;
    overrideMutex.lock();
    isOverride = override;
    overrideMutex.unlock();

    if (isOverride == 0){

        //take mutexand check distance
        int* dist = readSensorData(); //dist[0] = front, dist[1] = left, dist[2] = right


        if (dist[0] > 15){
            //if distance is greater than 10cm, drive forward
            go_forward();

        } else {
            //if left distance is greater than right distance, turn left
            if (dist[1] > dist[2]){
                turn_left();
            } else {
                turn_right();
            }
        }

    }

    /*
    //if rave mode is on
    else if (rave == 1 && isOverride == 1)
    {
        rave_mode();
    }
    */
    


    //non blocking delay
    rtos::ThisThread::sleep_for(10);
}