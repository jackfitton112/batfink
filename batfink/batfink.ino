//includes 
#include <Arduino.h>
#include <mbed.h>
#include <ArduinoBLE.h>
#include "pins.h"
#include "MotorControl.h"
#include "usonic.h"
#include "motors.h"
#include "ble.h"
<<<<<<< Updated upstream
#include "drive.h"
=======


>>>>>>> Stashed changes

void setup(){

    //setup motors
    Motor_setup();

<<<<<<< Updated upstream
    //setup bluetooth
    setup_ble();

    //start the poll thread
    ble.start(ble_thread);
    ble_worker.start(ble_worker_thread);

=======
>>>>>>> Stashed changes
    Serial.begin(9600);

    //start usonic thread
    frontusonicThread.start(frontusonicRead);
    leftusonicThread.start(leftusonicRead);
    rightusonicThread.start(rightusonicRead);

    //start motor thread
    motorThread.start(motorDrive);

    //setup ble
    ble_setup();


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

 

    //non blocking delay
    rtos::ThisThread::sleep_for(100);
}