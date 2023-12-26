/**
 * @file error.h
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Error codes for the batfink robot
 * @version 0.1
 * @date 24-12-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef ERROR_H
#define ERROR_H

#define OK 0

/*
NOTE: Error codes are in the format:
    ERR_<MODULE>_<ERROR>
    where <MODULE> is the module that the error occured in
    and <ERROR> is the error code for that module

    Each section has been given a range of 50 error codes for future expansion without having to renumber all the error codes

*/

// Drive/Motors Errors
#define ERR_DRIVE_GENERAL 1
#define ERR_DRIVE_INIT_FAIL 2
#define ERR_DRIVE_MOTOR_FAIL 3
#define ERR_DRIVE_WRNG_MODE 4
#define ERR_DRIVE_PWM_FAIL 5
#define ERR_DRIVE_VELOCITY_FAIL 6
#define ERR_DRIVE_POSITION_FAIL 7
#define ERR_DRIVE_PID_FAIL 8
#define ERR_DRIVE_ENCODER_FAIL 9
#define ERR_DRIVE_ENCODER_INIT_FAIL 10
#define ERR_DRIVE_ENCODER_RESET_FAIL 11
#define ERR_DRIVE_ENCODER_READ_FAIL 12


// BLE Connection and Data Transmission Errors
#define ERR_BLE_GENERAL 51
#define ERR_BLE_CONNECT_FAIL 52
#define ERR_BLE_DATA_SEND_FAIL 53


// Ultrasonic Sensor Errors
#define ERR_USONIC_GENERAL 101
#define ERR_USONIC_INIT_FAIL 102
#define ERR_USONIC_READ_FAIL 103


// Maze Creation and Solving Errors
#define ERR_MAZE_GENERAL 151
#define ERR_MAZE_CREATION_FAIL 152
#define ERR_MAZE_SOLVING_FAIL 153


#endif // ERROR_H
