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
