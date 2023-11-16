//includes 
#include <Arduino.h>
#include <mbed.h>
#include "pins.h"
#include "MotorControl.h"
#include "usonic.h"
#include "motors.h"
//#include "BLE.h"


//defines (in pins.h)

//setup ble
//BLE ble;






void setup(){

    //setup motors
    Motor_setup();

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

void loop() {

    //take mutexand check distance
    int* dist = readSensorData(); //dist[0] = front, dist[1] = left, dist[2] = right

    //print distance 0
    Serial.print("Front distance: ");
    Serial.println(dist[0]);

    if (dist[0] > 15){
        //if distance is greater than 10cm, drive forward
        motorMutex.lock();
        drive_direction = 1;
        motorMutex.unlock();

    } else {
        //if left distance is greater than right distance, turn left
        if (dist[1] > dist[2]){
            turn_left();
        } else {
            turn_right();
        }
    }


    //non blocking delay
    rtos::ThisThread::sleep_for(10);
}