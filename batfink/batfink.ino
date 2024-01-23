/**
 * @file batfink.ino
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief Main file for the batfink robot
 * @version 0.1
 * @date 16-01-2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "batfink.h"

using namespace mbed;
using namespace rtos;

Thread robotDriveThread(osPriorityNormal, 4096);
Thread OverrideThread(osPriorityNormal, 4096); 

int EmergencyOverride = 0;
int SerialConnected = 0;



void setup(){

    //wait for serial connection
    Serial.begin(115200);
    ThisThread::sleep_for(1000);

    


    Serial.println("Starting up...");
    int err;

    err = batfinkRobot.setup();

    if (err != 0){
        Serial.print("Error setting up batfink: ");
        Serial.println(err);
    }

    initMaze();

    err = bleSetup();

    if (err != 0){
        Serial.print("Error setting up bluetooth: ");
        Serial.println(err);
    }

    Serial.println("Setup complete");

    robotDriveThread.start(exploreMode);
    //OverrideThread.start(obsticleAvoidance);

    //turn left 360 degrees
    //batfinkRobot.turnLeft(360);
}

int messureSensorDistance(sensorType sensor){

    int distance = 0;

    //take 3 readings
    for(int i = 0; i < 3; i++){

        switch (sensor)
        {
        case FRONTSENSOR:
            distance += batfinkRobot._frontSensorDistancemm;
            break;
        case LEFTSENSOR:
            distance += batfinkRobot._leftSensorDistancemm;
            break;
        case RIGHTSENSOR:
            distance += batfinkRobot._rightSensorDistancemm;
            break;

        default:
            break;
        }

        ThisThread::sleep_for(300);
    }

    return distance / 3;

}


void obsticleAvoidance(){

    ThisThread::sleep_for(2000);

    while(1){

    

        while(SerialConnected == 1){
            ThisThread::sleep_for(100);
        }

        int front = batfinkRobot._frontSensorDistancemm;
        int left = batfinkRobot._leftSensorDistancemm;
        int right = batfinkRobot._rightSensorDistancemm;

        if (front < 30 || left < 30 || right < 30){
            EmergencyOverride = 1;
            if (front < 30){
               //check if left or right is less
                if (left < right){
                     //turn right
                     Serial.println("Turning Right 45");
                     batfinkRobot.turnRight(90);
                } else {
                     //turn left
                     Serial.println("Turning Left 45");
                     batfinkRobot.turnLeft(90);
                }

            } else if (left < 50){
                //turn right
                Serial.println("Turning Right 90");
                batfinkRobot.turnRight(45);
            } else if (right < 50){
                //turn left
                Serial.println("Turning Left 90");
                batfinkRobot.turnLeft(45);

            }
        } else {
            EmergencyOverride = 0;
        }

        ThisThread::sleep_for(100);
    }

}

void drive() {
    // Wait 2s
    ThisThread::sleep_for(2000);

    // Initialize the previous left distance
    int previousLeft = 0;

    while(1) {
        // Check for emergency override or serial connection
        while(EmergencyOverride == 1 || SerialConnected == 1){
            ThisThread::sleep_for(100);
        }

        int fronts = batfinkRobot._frontSensorDistancemm;
        int lefts = batfinkRobot._leftSensorDistancemm;
        
        // Print sensor values
        Serial.print("Front: ");
        Serial.print(fronts);
        Serial.print(" Left: ");
        Serial.println(lefts);

        // Desired distance from the left wall
        int desiredDistancemin = 70;
        int desiredDistancemax = 150;

        // Check if the robot has passed the edge of the wall
        if (previousLeft <= 100 && lefts > 200) {
            Serial.println("Passed Wall Edge - Turning Left 90");
            batfinkRobot.turnLeft(90);
            batfinkRobot.driveForward(150);
            batfinkRobot.turnLeft(90);
        } else if (fronts < 12) {
            // Avoid obstacle by turning right
            Serial.println("Obstacle Ahead - Turning Right 90");
            batfinkRobot.turnRight(90);
        } else {
            // Adjust the robot's position relative to the left wall
            if (lefts < desiredDistancemin) {
                // Turn left
                Serial.println("Turning Left 45");
                batfinkRobot.turnLeft(45);
            } else if (lefts > desiredDistancemax) {
                // Turn right
                Serial.println("Turning Right 45");
                batfinkRobot.turnRight(45);
            } else {
                // Continue driving forward
                Serial.println("Driving Forward");
                batfinkRobot.driveForward(100); // Adjust the driving distance as needed
            }
        }

        // Update the previous left distance
        previousLeft = lefts;
    }
}

int calculateTurnBias(int LeftDistance, int RightDistance){
    /*int CurrentAngle = batfinkRobot._angleDeg;
    int leftAngle = (CurrentAngle + 90) % 360;
    int rightAngle = (CurrentAngle + 270) % 360; // Adjusted to avoid negative values

    // Normalize angles to be in the range [0, 360)
    if (rightAngle >= 360) rightAngle -= 360;

    // Determine the smaller angular distance to 0 degrees
    int leftDistanceToZero = leftAngle; // Closer to 0 going counterclockwise
    int rightDistanceToZero = 360 - rightAngle; // Closer to 0 going clockwise

    if (leftDistanceToZero < rightDistanceToZero && LeftDistance > 100){
        //turn left
        return 1;
    } else if (rightDistanceToZero < leftDistanceToZero && RightDistance > 100){
        //turn right
        return 2;
    } else {*/
        //stalemate, turn whichever side has greater distance
        if (LeftDistance > RightDistance){
            //turn left
            return 1;
        } else {
            //turn right
            return 2;
        }
    //}
}


void exploreMode(){


    //wait 2s to allow sensors to settle
    ThisThread::sleep_for(2000);
    
    //Robot will be placed into the maze towards the middle facing south so it can align its self in the maze

    //0,0 is the bottom left of the maze

    //take the left and right sensor readings to calc the robots position within the maze
    //left is going to be right and right is going to be left as the robot will be backwards
    int right = 0;
    int left = messureSensorDistance(RIGHTSENSOR) + 120; //offest to the centre of the robot
    int front = messureSensorDistance(FRONTSENSOR) + 120; //offest to the centre of the robot

    batfinkRobot._XPOS = front;
    batfinkRobot._YPOS = left;

    //print new position
    Serial.print("X: ");
    Serial.print(batfinkRobot._XPOS);
    Serial.print(" Y: ");
    Serial.println(batfinkRobot._YPOS);

    //set both motor targets to 100 encoder counts to check theyre working
    batfinkRobot.driveForward(20);

    //if encoder values are 0, return error
    if (batfinkRobot._leftMotor._encoderCount == 0 || batfinkRobot._rightMotor._encoderCount == 0){
        Serial.println("Error: Encoder values are 0");
        return;
    }

    //drive backwards to reset encoder values
    batfinkRobot.driveBackward(20);

    //turn 180 degrees to face north
    batfinkRobot.turnRight(180);


    //print angle in radians and degrees
    Serial.print("Angle in radians: ");
    Serial.print(batfinkRobot._THETA_RAD);
    Serial.print(" Angle in degrees: ");
    Serial.println(batfinkRobot._angleDeg);

    //reset theta values to 0 but maintain whatever error there is
    batfinkRobot._THETA_RAD = PI + batfinkRobot._THETA_RAD;
    batfinkRobot._angleDeg = 180 + batfinkRobot._angleDeg;
    batfinkRobot._TheoreticalAngle = 0;

    int MUSTTURN = 0;
    int currentAngle;
    int theoreticalAngle;
    int angle;
    int i;

    //start exploring the maze
    while(1){

        i += 1;
        //go forward unless there is a wall in front, then turn left or right whichever is greater
        front = messureSensorDistance(FRONTSENSOR);


        if(front > 150 && MUSTTURN == 0){
            //go forward
            Serial.print("Going forward: ");
            Serial.println(front-100);
            batfinkRobot.driveForward(front - 100);
            //must now check surroundings
            /*batfinkRobot.turnLeft(15);
            int l15 = messureSensorDistance(LEFTSENSOR);
            batfinkRobot.turnRight(30);
            int r30 = messureSensorDistance(RIGHTSENSOR);

            currentAngle = batfinkRobot._angleDeg;
            theoreticalAngle = batfinkRobot._TheoreticalAngle;
            angle = theoreticalAngle - currentAngle;
            angle = 360 + angle; //add 360 to account for negative angles
            angle = angle % 360; //get the remainder of the angle

            if (15-angle < 0){
                Serial.println(15-angle+360);
                batfinkRobot.turnLeft(15, 360-angle);
            } else {
                Serial.println(15-angle);
                batfinkRobot.turnLeft(15, -angle);
            }

            //if either of the readings are less than 70, turn 90 degrees
            if (l15 < 70 || r30 < 70){
                MUSTTURN = 1;
            }
            */

        }
        /* else if (front > 60 && front < 150 && MUSTTURN == 0){
            //go forward
            Serial.println("Going forward 30");
            batfinkRobot.driveForward(30);
            //must turn
            MUSTTURN = 1;
                
        } */else {
            
            //drive back 50mm to avoid any walls that havent been detected
            Serial.println("Backing up 20");
            batfinkRobot.driveBackward(20);

            //turn left or right
            left = messureSensorDistance(LEFTSENSOR);
            right = messureSensorDistance(RIGHTSENSOR);

            //calculate which direction to turn
            int direction = calculateTurnBias(left, right);

            currentAngle = batfinkRobot._angleDeg;
            theoreticalAngle = batfinkRobot._TheoreticalAngle;
            angle = theoreticalAngle - currentAngle;
            angle = 360 + angle; //add 360 to account for negative angles
            angle = angle % 360; //get the remainder of the angle


            //if direction is 1, turn left
            if (direction == 1){
                Serial.print("Turning left: ");
                if (90-angle < 0){
                    Serial.println(90-angle+360);
                    batfinkRobot.turnLeft(90, 360-angle);
                } else {
                    Serial.println(90-angle);
                    batfinkRobot.turnLeft(90, -angle);
                }
            } else {
                Serial.print("Turning right: ");

                if (90+angle > 360){
                    Serial.println(90+angle-360);
                    batfinkRobot.turnRight(90, angle-360);
                } else {
                    Serial.println(90+angle);
                    batfinkRobot.turnRight(90, angle);
                }
            }

            //reset MUSTTURN
            MUSTTURN = 0;
        }


        //if x is more than 1300, check 0 degrees to see if there is a path
        /*
        if (batfinkRobot._XPOS > 1300){
            //turn to 0 degrees
            currentAngle = batfinkRobot._angleDeg;

            //if angle is more than 180, turn right
            if (currentAngle >= 180){
                Serial.println("Turning right");
                batfinkRobot.turnRight(currentAngle -180);
            } else {
                Serial.println("Turning left ");
                batfinkRobot.turnLeft(currentAngle);
            }

            //check if there is a path
            front = messureSensorDistance(FRONTSENSOR);
            if (front > 300){
                //there is a path
                //drive forward front - 100
                Serial.print("Driving forward: ");
                Serial.println(front-100);
                batfinkRobot.driveForward(front-100);

            } 
              
        }*/

        //if xpos is more than 1300, point to 0 degrees and check if there is a path
        if (batfinkRobot._XPOS > 1300 && i % 5 == 0){
            //turn to 0 degrees
            currentAngle = batfinkRobot._angleDeg;

            batfinkRobot.turnRight(360 - currentAngle);

            //check front sensor
            front = messureSensorDistance(FRONTSENSOR);

            if(front > 300){
                //there is a path
                //drive forward front - 100
                Serial.print("Driving forward: ");
                Serial.println(front-100);
                batfinkRobot.driveForward(front-100);
            } 

        
              
        }

        //print new position
        Serial.print("X: ");
        Serial.print(batfinkRobot._XPOS);
        Serial.print(" Y: ");
        Serial.println(batfinkRobot._YPOS);

        //if X value is more than 1900, the robot has reached the end of the maze
        if (batfinkRobot._XPOS > 1480){
            Serial.println("End of maze reached");
            batfinkRobot.stop();
            break;
        }

        ThisThread::sleep_for(100);

    }


    //wait for a serial connection
    while(!Serial.available()){
        ThisThread::sleep_for(100);
    }

    //print the maze
    printMaze();

    //wait for serial to be disconnected
    while(Serial.available()){
        ThisThread::sleep_for(100);
    }




    

    

    


}

void printMaze() {
    Serial.println("========================================");
    for (auto &row : maze) {
        for (auto &cell : row) {
            switch (cell) {
                case UNKNOWN:
                    Serial.print("?");
                    break;
                case EMPTY:
                    Serial.print(" ");
                    break;
                case WALL:
                    Serial.print("X");
                    break;
            }
        }
        Serial.println();
    }
    Serial.println("========================================");
}



void loop(){

    //if serial is connected, stop the robot, stop the mapping and print the maze

    /*if (Serial.available() > 0){
        map_hold = 0;
        Serial.println("Serial connected, stopping robot");
        batfinkRobot.stop();
        Serial.println("Printing maze");
        printMaze();
        Serial.println("Stopping");
        while(Serial.available() > 0){
            ThisThread::sleep_for(100);\
        }
        map_hold = 1;
    }
    */
    /*
   //print encoder count and target position
    Serial.print("Left encoder count: ");
    Serial.print(batfinkRobot._leftMotor._encoderCount);
    Serial.print(" Left target position: ");
    Serial.print(batfinkRobot._leftMotor._TargetPosition);
    Serial.print(" Right encoder count: ");
    Serial.print(batfinkRobot._rightMotor._encoderCount);
    Serial.print(" Right target position: ");
    Serial.println(batfinkRobot._rightMotor._TargetPosition);
    */


    //angle calibration, print real angle and theoretical angle
    Serial.print("Real angle: ");
    Serial.print(batfinkRobot._angleDeg);
    Serial.print(" Theoretical angle: ");
    Serial.print(batfinkRobot._TheoreticalAngle);

    //print delta angle
    int deltaAngle = batfinkRobot._TheoreticalAngle - batfinkRobot._angleDeg;
    Serial.print(" Delta angle: ");
    Serial.println(deltaAngle);
  
    //wait for 1 second 
    ThisThread::sleep_for(1000);


}