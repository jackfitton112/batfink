/**
 * @file sensor.h
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Sensor module for the batfink robot header file
 * @version 0.1
 * @date 24-12-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "error.h"
#include <mbed/mbed.h>
#include <chrono>

enum sensorType {
    FRONTSENSOR,
    LEFTSENSOR,
    RIGHTSENSOR,
};


using namespace mbed;
using namespace rtos;

// Ultrasonic pins (MBed pins)
#define MBED_USONIC1 P0_23 //FRONT
#define MBED_USONIC2 P1_14 //RIGHT
#define MBED_USONIC3 P1_13 //LEFT
#define MBED_USONIC4 P1_15 //NC

#define READ_INTERVAL 0.25 //read interval in seconds

// Sensor class to read from ultrasonic sensors
class Sensor {
public:
    Sensor(PinName pin);
    int getDistance();
    void read();
    void setup();

private:
    bool _pointCloud = false;
    DigitalInOut _pin;
    Timer _timer;
    Ticker _ticker;
    int _distance;


};


extern Sensor frontSensor;
extern Sensor leftSensor;
extern Sensor rightSensor;

extern void sensorInit();

#endif // SENSOR_H
