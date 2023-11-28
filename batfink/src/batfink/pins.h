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

//ultra sonic sensors, these pins are wired to both trig and echo
#define USONIC1 D7
#define USONIC2 D6
#define USONIC3 D5
#define USONIC4 D4


//motor pins
// motor A is the right Motor
// motor B is the left Motor
#define MOTOR_DIRA A0
#define MOTOR_DIRB A1
#define MOTOR_ENCB D3
#define MOTOR_ENCA D2
#define MOTOR_PWMB D10
#define MOTOR_PWMA D9