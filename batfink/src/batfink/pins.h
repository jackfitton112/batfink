/**
 * @file pins.h
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Pin definitions header file
 * @version 0.1
 * @date 28-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef __PINS_H__
#define __PINS_H__

//ultra sonic sensors, these pins are wired to both trig and echo
//will convert to mbed pins soon 
#define USONIC1 D7
#define USONIC2 D6
#define USONIC3 D5
#define USONIC4 D4


//motor pins
// motor A is the right Motor
// motor B is the left Motor
#define MBED_MOTOR_DIRA P0_4
#define MBED_MOTOR_ENCA P1_11
#define MBED_MOTOR_PWMA P0_27

#define MBED_MOTOR_DIRB P0_5
#define MBED_MOTOR_ENCB P1_12
#define MBED_MOTOR_PWMB P1_2

//Ultrasonic pins
#define MBED_USONIC1 P0_23
#define MBED_USONIC2 P1_14
#define MBED_USONIC3 P1_13
#define MBED_USONIC4 P1_15

#endif //__PINS_H__
