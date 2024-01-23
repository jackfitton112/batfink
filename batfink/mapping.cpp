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


// Initialize the maze array
std::array<std::array<CellState, MAZE_HEIGHT>, MAZE_WIDTH> maze;

int map_hold = 0;

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

    x = x / 50;
    y = y / 50;

    if (x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT) {
        maze[x][y] = state;

    }
}

// Mapping thread
void mapping() {
    while (true) {
        if (map_hold == 1) {
            // Get robot position and orientation
            int x = batfinkRobot._XPOS;
            int y = batfinkRobot._YPOS;
            double theta = batfinkRobot._THETA_RAD;

            // Get sensor distances
            int fsensor = batfinkRobot._frontSensorDistancemm;
            int lsensor = batfinkRobot._leftSensorDistancemm;
            int rsensor = batfinkRobot._rightSensorDistancemm;

            // Update maze for robot's current position
            updateMaze(x, y, EMPTY);

            // Calculate wall positions based on sensors and orientation
            // Convert sensor readings from mm to maze cell units and adjust positions
            int fWallX = x + (fsensor * cos(theta)) / 5;
            int fWallY = y + (fsensor * sin(theta)) / 5;
            int lWallX = x + (lsensor * cos(theta + M_PI_2)) / 5;
            int lWallY = y + (lsensor * sin(theta + M_PI_2)) / 5;
            int rWallX = x + (rsensor * cos(theta - M_PI_2)) / 5;
            int rWallY = y + (rsensor * sin(theta - M_PI_2)) / 5;

            // Update maze with wall positions
            updateMaze(fWallX, fWallY, WALL);
            updateMaze(lWallX, lWallY, WALL);
            updateMaze(rWallX, rWallY, WALL);

            ThisThread::sleep_for(50);
        } else {
            ThisThread::sleep_for(1000);
        }
    }
}




