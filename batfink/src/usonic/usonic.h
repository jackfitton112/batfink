/**
 * @file usonic.h
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief UltraSonic Sensor header file
 * @version 0.1
 * @date 28-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef USONIC_H
#define USONIC_H

#include <mbed.h>

// Declare sensor data struct
struct sensorDatas {
    int front;
    int left;
    int right;
};

// Declare sensor data extern variable
extern sensorDatas *sensorData;

// Declare sensorMutex extern variable
extern rtos::Mutex sensorMutex;

// Declare sensorThread extern variable
extern rtos::Thread sensorThread;

// Function prototypes
void Usonic_setup();
int ReadDistance(mbed::DigitalInOut sensor);
void getSensorDataThread();

#endif
