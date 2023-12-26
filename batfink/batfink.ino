/**
 * @file batfink.ino
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Main file for the batfink robot
 * @version 0.1
 * @date 24-12-2023
 * 
 * @copyright Copyright (c) 2023
 * 
  ___   _ _____ ___ ___ _  _ _  ___ 
 | _ ) /_\_   _| __|_ _| \| | |/ / |
 | _ \/ _ \| | | _| | || .` | ' <|_|
 |___/_/ \_\_| |_| |___|_|\_|_|\_(_)

 Batfink Super sonic sonar radar robot

 Stage 3 Robotic Deisgn and Contstruction
 University of York
 Jack Fitton
 2023/24

*/                                  


#include "batfink.h"


/**
 * @brief Initializes the setup of the program.
 * 
 * This function is called once at the beginning of the program.
 * It starts the serial communication, initializes the sensors,
 * and prepares the program for further initialization of motors,
 * maze, and communication.
 */
void setup(){
    //start serial comms
    Serial.begin(115200);

    //initialise sensors
   //sensorInit();

    //initialise motors
    motorInit();

    //initialise maze
    //mazeInit(); TODO

    //initialise comms
    //commsInit(); TODO



}



//main loop, will have logic in here soon TM
void loop(){

    ThisThread::sleep_for(1000);

}
