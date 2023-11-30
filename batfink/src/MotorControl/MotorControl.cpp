/**
 * @file MotorControl.cpp
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief  Motor Control source file
 * @version 0.1
 * @date 28-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "MotorControl.h"


//TODO: make these values be dynamic via PID
int MOTOR_LEFT = 200;
int MOTOR_RIGHT = 200;



/**
 * @brief Motor class constructor.
 * 
 * This constructor initializes the Motor object with the specified pins and parameters.
 * It sets the pin modes, initializes the member variables, and configures the initial state of the motor.
 * 
 * @param dirPin The pin number for the direction control.
 * @param pwmPin The pin number for the PWM control.
 * @param encPin The pin number for the encoder input.
 * @param forwardMultiplier The multiplier for forward movement.
 */
Motor::Motor(int dirPin, int pwmPin, int encPin, int forwardMultiplier)
    : _dirPin(dirPin), _pwmPin(pwmPin), _encPin(encPin), _dir(CW), _steps(0), _speed(0), _forwardMultiplier(forwardMultiplier) 
{
    pinMode(_dirPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    pinMode(_encPin, INPUT);
    digitalWrite(_dirPin, _dir);
    analogWrite(_pwmPin, 0);
}

/**
 * @brief Sets the direction of the motor.
 * 
 * @param dir The direction to set the motor to.
 */
void Motor::setDirection(Direction dir) {
    _dir = dir;
    digitalWrite(_dirPin, _dir);
}

/**
 * @brief Gets the direction of the motor.
 * 
 * @return Direction The direction of the motor.
 */
Direction Motor::getDirection() const {
    return _dir;
}

/**
 * @brief Sets the speed of the motor.
 * 
 * @param speed The speed to set the motor to.
 */
void Motor::setSpeed(int speed) {
    _speed = speed;
    analogWrite(_pwmPin, _speed);
}

/**
 * @brief Gets the speed of the motor.
 * 
 * @return int The speed of the motor.
 */
int Motor::getSpeed() const {
    return _speed;
}

/**
 * @brief Updates the encoder value of the motor.
 * 
 * This function is called when an interrupt is triggered by the encoder.
 * It updates the encoder value of the motor based on the direction of the motor.
 */
void Motor::updateEncoder() {
    if((_dir == CW && _forwardMultiplier == 1) || (_dir == CCW && _forwardMultiplier == -1)) {
        _steps++;
    } else {
        _steps--;
    }
}

/**
 * @brief Gets the current encoder value of the motor.
 * 
 * @return int The current encoder value of the motor.
 */
int Motor::getSteps() const {
    return _steps;
}

/**
 * @brief Resets the encoder value of the motor to 0.
 * 
 */
void Motor::resetEncoder() {
    _steps = 0;
}

/**
 * @brief Sets the target encoder value of the motor.
 * 
 * @param steps The target encoder value of the motor.
 */
void Motor::setTargetSteps(int steps) {
    _targetSteps = steps;
    _prevSteps = _steps; // This will allow us to calculate the number of steps taken since the last time we set the target
}


// ISR handlers need to be global
Motor *leftMotor;
Motor *rightMotor;

/**
 * @brief Interrupt service routine for ENCA.
 * This function is called when an interrupt is triggered by ENCA.
 * It updates the encoder value of the right motor.
 */
void ENCA_ISR() {
  rightMotor->updateEncoder();
}

/**
 * @brief Interrupt service routine for ENCB.
 * This function is called when an interrupt is triggered by ENCB.
 * It updates the encoder value of the left motor.
 */
void ENCB_ISR() {
  leftMotor->updateEncoder();
}

/**
 * @brief Sets up the motor control.
 * 
 * This function creates the leftMotor and rightMotor objects and attaches the encoder ISRs to the encoder pins.
 * It also starts the motor drive thread.
 */
void Motor_setup() {
    // For left motor, forward direction is CW, so forwardMultiplier is 1
    leftMotor = new Motor(MOTOR_DIRB, MOTOR_PWMB, MOTOR_ENCB, 1);

    // For right motor, forward direction is CCW, so forwardMultiplier is -1
    rightMotor = new Motor(MOTOR_DIRA, MOTOR_PWMA, MOTOR_ENCA, -1);

    attachInterrupt(digitalPinToInterrupt(leftMotor->getEncPin()), ENCB_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightMotor->getEncPin()), ENCA_ISR, CHANGE);

    //start motor drive thread
    motorThread.start(motorDriveThread);
}


/**
 * @brief Moves the robot forward by setting the direction of the motors and the speed.
 * 
 */
void forward() {
  //Serial.println("Moving Forward");
  rightMotor->setDirection(CCW);
  leftMotor->setDirection(CW);
  rightMotor->resetEncoder();
  leftMotor->resetEncoder();
  rightMotor->setSpeed(MOTOR_RIGHT);
  leftMotor->setSpeed(MOTOR_LEFT);
}

/**
 * @brief Moves the robot left by setting the direction of the motors and the speed.
 * 
 */
void left() {
  //Serial.println("Turning Left");
  rightMotor->setDirection(CCW);
  leftMotor->setDirection(CCW);
  rightMotor->resetEncoder();
  leftMotor->resetEncoder();
  rightMotor->setSpeed(50);  // Assuming half speed for turning, adjust as needed
  leftMotor->setSpeed(50);
}

/**
 * @brief Turns the robot right by setting the direction of the motors and the speed.
 * 
 */
void right() {
  //Serial.println("Turning Right");
  rightMotor->setDirection(CW);
  leftMotor->setDirection(CW);
  rightMotor->resetEncoder();
  leftMotor->resetEncoder();
  rightMotor->setSpeed(50);  // Assuming half speed for turning, adjust as needed
  leftMotor->setSpeed(50);
}

/**
 * @brief Moves the robot backward by setting the direction of the motors and the speed.
 * 
 */
void backward() {
  //Serial.println("Moving Backward");
  rightMotor->setDirection(CW);
  leftMotor->setDirection(CCW);
  rightMotor->resetEncoder();
  leftMotor->resetEncoder();
  rightMotor->setSpeed(MOTOR_RIGHT);
  leftMotor->setSpeed(MOTOR_LEFT);
}

/**
 * @brief Stops the robot by setting the speed of both motors to 0.
 * 
 */
void stopRobot() {
  //Serial.println("Stopping");
  rightMotor->setSpeed(0);
  leftMotor->setSpeed(0);
}



/**
 * @brief Cleans up the resources used by the motor control.
 * 
 * This function deletes the leftMotor and rightMotor objects, freeing up the memory they occupy.
 */
void cleanup() {
  delete leftMotor;
  delete rightMotor;
}



rtos::Thread motorThread;
rtos::Mutex motorMutex;
int drive_direction = 0; // 0 = stop, 1 = forward, 2 = backward, 3 = left, 4 = right

/**
 * @brief This function is a thread that controls the motor drive based on the drive direction.
 * 
 * It continuously checks the drive direction and drives the motor accordingly.
 * The drive direction is obtained from a shared variable and is protected by a mutex.
 * The motor is driven in the specified direction using different motor control functions.
 * The thread sleeps for 100ms after each iteration.
 */
void motorDriveThread(){
    int prev_dir = 0;
    while(1){

        //take mutex and check drive direction
        motorMutex.lock();
        int dir = drive_direction;
        motorMutex.unlock();

        //drive in the direction specified
        if (dir != prev_dir){
            prev_dir = dir;
            switch(dir){
                case 0:
                    stopRobot();
                    break;
                case 1:
                    forward();
                    break;
                case 2:
                    backward();
                    break;
                case 3:
                    left();
                    break;
                case 4:
                    right();
                    break;
                default:
                    stopRobot();
                    break;
            }
        }

        //sleep for 100ms
        rtos::ThisThread::sleep_for(100);

    }

}



void driveDistance(int distance){
    //Motors are 298:1 ratio, 6 counts per revolution
    // 1 revolution = 298 * 12 = 3576 counts
    // wheel is 40mm diameter, 125mm circumference
    // 1 revolution = 125mm
    // 1 count = 125/3576 = 0.035mm

    int target = distance / 0.035;

    //set target steps
    leftMotor->setTargetSteps(target);
    rightMotor->setTargetSteps(target);

    //reset the encoders

    //set drive direction to forward
    motorMutex.lock();
    drive_direction = 1;
    motorMutex.unlock();

    //wait for the motors to hit the target
    while(abs(leftMotor->getSteps()) < target && abs(rightMotor->getSteps() < target)){
        Serial.print(leftMotor->getSteps());
        Serial.print(",");
        Serial.println(rightMotor->getSteps());

        rtos::ThisThread::sleep_for(10);
    }

    //stop the robot
    motorMutex.lock();
    drive_direction = 0;
    motorMutex.unlock();



    

}

void turnAngle(int angle, int direction){
    //Motors are 298:1 ratio, 2 step per revolution
    //1 rev = 720

    //wheels are 150mm apart and in the centre of the robot

    //1140 is 90 degrees
    //5760 is 360 degrees
    int TARGET = 5760 * (angle / 360);
    leftMotor->setTargetSteps(TARGET);
    rightMotor->setTargetSteps(TARGET);

    //Reset the encoders
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();

    //set drive direction to left
    if (direction == 0){
        motorMutex.lock();
        drive_direction = 4;
        motorMutex.unlock();
    } else {
        motorMutex.lock();
        drive_direction = 3;
        motorMutex.unlock();
    }

    //wait for the motors to turn 1 revolution
    while(abs(leftMotor->getSteps()) < TARGET && abs(rightMotor->getSteps() < TARGET)){
        Serial.print(leftMotor->getSteps());
        Serial.print(",");
        Serial.println(rightMotor->getSteps());

        rtos::ThisThread::sleep_for(10);
    }

    //stop the robot
    motorMutex.lock();
    drive_direction = 0;
    motorMutex.unlock();




}

/**
 * @brief Turns the robot 90 degrees in the specified direction.
 * 
 * @param direction The direction to turn (0 for left, 1 for right).
 */
void turn90deg(int direction){
    //Motors are 298:1 ratio, 2 step per revolution
    //1 rev = 720

    //wheels are 150mm apart and in the centre of the robot

    //TESTING - TURN THE WHEELS 1 REVOLUTION - This does 255 degrees
    //each degree is 16 syeps 
    //set target steps
    int TARGET = 710*2;
    leftMotor->setTargetSteps(TARGET);
    rightMotor->setTargetSteps(TARGET);

    //Reset the encoders
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();

    //set drive direction to left
    if (direction == 0){
        motorMutex.lock();
        drive_direction = 4;
        motorMutex.unlock();
    } else {
        motorMutex.lock();
        drive_direction = 3;
        motorMutex.unlock();
    }

    //wait for the motors to turn 1 revolution
    while(abs(leftMotor->getSteps()) < TARGET && abs(rightMotor->getSteps() < TARGET)){
        Serial.print(leftMotor->getSteps());
        Serial.print(",");
        Serial.println(rightMotor->getSteps());

        rtos::ThisThread::sleep_for(10);
    }

    //stop the robot
    motorMutex.lock();
    drive_direction = 0;
    motorMutex.unlock();



}


void stop(){
        motorMutex.lock();
        drive_direction = 0;
        motorMutex.unlock();
}


void findGreatestPath(){


    int distances[4];
    int angles[4];

    //get turn 90 degreees and get the distances
    for(int i = 0; i < 4; i++){
        turn90deg(0);
        rtos::ThisThread::sleep_for(1000);
        distances[i] = sensorData->front;
        angles[i] = i * 90;
    }

    //find the greatest distance
    int greatest = 0;
    int greatest_angle = 0;

    for(int i = 0; i < 4; i++){
        if(distances[i] > greatest){
            greatest = distances[i];
            greatest_angle = angles[i];
        }
    }

    //turn to the greatest angle in lots of 90
    int amount_of_turns = greatest_angle / 90;

    for (int i = 0; i < amount_of_turns; i++){
        turn90deg(0);
        rtos::ThisThread::sleep_for(1000);
    }


}

