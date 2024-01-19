/**
 * @file sensor.cpp
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Sensor module for the batfink robot source file
 * @version 0.1
 * @date 24-12-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "sensor.h"



using namespace mbed;
using namespace rtos;


Sensor frontSensor(MBED_USONIC1);
Sensor leftSensor(MBED_USONIC3);
Sensor rightSensor(MBED_USONIC2);




/**
 * @brief Construct a new Sensor:: Sensor object
 * 
 * @param pin 
 */
Sensor::Sensor(PinName pin) : _pin(pin){
    _distance = 0;

}

void Sensor::setup(){
    _ticker.attach(callback(this, &Sensor::read), READ_INTERVAL); //attach ticker to read function
}

/**
 * @brief Get the distance from the sensor in cm
 * 
 * @return int 
 */
int Sensor::getDistance(){
    return _distance;
}


/**
 * @brief Reads the distance from the sensor.
 * 
 * This function triggers the sensor to measure the distance and calculates the distance based on the time it takes for the signal to travel.
 * The distance is stored in the private member variable _distance.
 * 
 * @note This function assumes that the pin connected to the sensor is configured as an output and input pin.
 * 
 * @note This function uses a deprecated function wait_us() to introduce delays. the docs say its okay to use but should be changed in the future.
 */
void Sensor::read(){


    _pin.output();
    _pin = 0; //set pin low

    //wait for 2us
    wait_us(2); //DEPRECATED but still works

    _pin = 1; //set pin high

    //wait for 5us
    wait_us(5);

    _pin = 0; //set pin low

    //set pin to input
    _pin.input();

    //wait for pin to go high
    while(_pin.read() == 0);

    //start timer
    _timer.start();

    //wait for pin to go low
    while(_pin.read() == 1);

    //stop timer
    _timer.stop();

    //calculate distance
    int read_dist = _timer.read_us() / 58; //58 is the conversion factor from time to distance
    
    //update distance
    _distance = read_dist * 10; //convert to mm

    //if distance is more than 2000mm. set it to zero as this is likely an error
    if (_distance > 2000){
        _distance = 0;
    }

    //reset timer
    _timer.reset();

}

