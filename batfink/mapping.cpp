/**
 * @file mapping.cpp
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Mapping functions for batfink
 * @version 0.1
 * @date 19-01-2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "mapping.h"



rtos::Thread MappingThread(osPriorityNormal, 4096);
int map_hold = 1;


// Initialize the maze array
std::array<std::array<CellState, MAZE_HEIGHT>, MAZE_WIDTH> maze;

// Initialize maze with unknown cells
void initMaze() {
    for (auto &row : maze) {
        row.fill(UNKNOWN);
    }

    //start mapping thread
    MappingThread.start(mapping);
}

// Update function based on sensor data
void updateMaze(int x, int y, CellState state) {
    if (x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT) {
        maze[x][y] = state;
    }
}

// Mapping thread
void mapping() {
    while (true) {

        if (map_hold == 1){

        //get robot x and y
        int x = batfinkRobot._XPOS;
        int y = batfinkRobot._YPOS;
        double theta = batfinkRobot._THETA_RAD;


        int fsensor = batfinkRobot._frontSensorDistancemm;
        int lsensor = batfinkRobot._leftSensorDistancemm;
        int rsensor = batfinkRobot._rightSensorDistancemm;

        //update maze
        updateMaze(x, y, EMPTY);

        //calulate distances to walls
        int fdist = (fsensor * cos(theta)) - 20;
        int ldist = (lsensor * cos(theta + 1.5708)) - 20;
        int rdist = (rsensor * cos(theta - 1.5708)) - 20;


        //update maze
        updateMaze(x + fdist, y, WALL);//Not sure if this is correct
        updateMaze(x, y + ldist, WALL); //This is not always going to be plus minus, it needs to be dynamic based off which way the robot faces
        updateMaze(x, y - rdist, WALL);

        } else {
            ThisThread::sleep_for(1000);
        }

        

    

    }
}





