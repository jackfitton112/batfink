/**
 * @file usonic.cpp
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief UltraSonic Sensor Functions
 * @version 0.1
 * @date 28-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "usonic.h"


mbed::DigitalInOut frontSensor(P0_23);
mbed::DigitalInOut rightSensor(P1_14);
mbed::DigitalInOut leftSensor(P1_13);


// Define sensor data struct
sensorDataStruct* sensorData = new sensorDataStruct;

// Define sensorMutex variable
rtos::Mutex sensorMutex;

// Define sensorThread variable
rtos::Thread sensorThread;


void Usonic_setup(){

    //setup structs
    sensorData->front = 0;
    sensorData->left = 0;
    sensorData->right = 0;

    //set up the sensor thread
    sensorThread.start(getSensorDataThread);


}


/**
 * @brief Reads the distance from an ultrasonic sensor.
 * 
 * This function sends a trigger signal to the ultrasonic sensor, measures the time it takes for the echo signal to return,
 * and calculates the distance based on the measured time.
 * 
 * @param sensor The digital input/output pin connected to the ultrasonic sensor.
 * @return The distance measured by the ultrasonic sensor in centimeters.
 */

int ReadDistance(mbed::DigitalInOut sensor){
    
    // USONIC READ DISTANCE FUNCTION


    sensor.output(); //Set pin as output

    sensor = 0; //Set to low

    wait_us(2); //Wait 2 microseconds

    sensor = 1; //Set to high

    wait_us(5); //Wait 5 microseconds

    sensor = 0; //Set to low

    wait_us(10); //Wait 10 microseconds

    sensor.input(); //Set pin as input

    while(sensor.read() == 0); //Wait until pin is high

    mbed::Timer t; //Create timer object
    t.start(); //Start timer

    while(sensor.read() == 1); //Wait until pin is low

    int time = t.read_us(); //Read time in microseconds

    int distance = time / 58; //Calculate distance from time


    t.stop(); //Stop timer
    t.reset(); //Reset timer

    return distance;
 
}


/**
 * @brief This function runs in a separate thread to continuously read sensor data.
 * 
 * @details This function runs in an infinite loop and performs the following steps:
 * 1. Reads the distance from the front, left, and right ultrasonic sensors.
 * 2. Takes a mutex lock to ensure exclusive access to the sensor data.
 * 3. Updates the sensor data struct with the new sensor values.
 * 4. Releases the mutex lock.
 * 5. Sleeps for 100 milliseconds before repeating the process.
 */
void getSensorDataThread(){
    while(1){

        //get the sensor values 
        int front = ReadDistance(frontSensor);
        int left = ReadDistance(leftSensor);
        int right = ReadDistance(rightSensor);

        //take mutex and set the sensor data
        sensorMutex.lock();
        sensorData->front = front;
        sensorData->left = left;
        sensorData->right = right;
        sensorMutex.unlock();

        //sleep for 50ms
        rtos::ThisThread::sleep_for(100);

    }
}