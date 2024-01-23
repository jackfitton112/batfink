/**
 * @file mapping.h
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Header file for mapping.cpp
 * @version 0.1
 * @date 19-01-2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MAPPING_H
#define MAPPING_H



#include <mbed/mbed.h>
#include <array>
#include "robot.h"
#include <math.h>


//1.5m x 2m maze with 5cm cells
#define MAZE_WIDTH 40
#define MAZE_HEIGHT 30


enum CellState { UNKNOWN, EMPTY, WALL };
extern std::array<std::array<CellState, MAZE_HEIGHT>, MAZE_WIDTH> maze;
extern void initMaze(); 
extern void updateMaze(int x, int y, CellState state);
extern void mapping();
extern void printMaze();


extern int map_hold;



#endif // MAPPING_H