
//includes
#include <Arduino.h>
#include <ARB.h>
#include <ArduinoBLE.h>
#include "MotorControl.h"

//Both echo and trig are wired to the same pin per sensor
#define FRONT_SENSOR D7
#define RIGHT_SENSOR D6
#define LEFT_SENSOR D5

//ultrasonic vaialbles
int front_distance = 0;
int right_distance = 0;
int left_distance = 0;

int front_trig_time = 0;
int right_trig_time = 0;
int left_trig_time = 0;

int front_echo_time = 0;
int right_echo_time = 0;
int left_echo_time = 0;

int front_trig_state = 0;
int right_trig_state = 0;
int left_trig_state = 0;

int front_distance_cm = 0;
int right_distance_cm = 0;
int left_distance_cm = 0;

int interupt_counter = 0;



// BLE Service and Characteristic
BLEService robotService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharCharacteristic commandCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

/*
	pinMode(USONIC1, OUTPUT);
	digitalWrite(USONIC1, LOW);
	delayMicroseconds(2);
	digitalWrite(USONIC1, HIGH);
	delayMicroseconds(15);
	digitalWrite(USONIC1, LOW);

	// The same pin is used to read back the returning signal, so must be set back to input
	pinMode(USONIC1, INPUT);
	duration[0] = pulseIn(USONIC1, HIGH);
*/

//ultrasonic functions
int getDistance(int sensor){
  //set pinmode to output
  pinMode(sensor, OUTPUT);

  //set pin to low
  digitalWrite(sensor, LOW);

  //wait 2 microseconds
  delayMicroseconds(2);

  //set pin to high
  digitalWrite(sensor, HIGH);

  //wait 15 microseconds
  delayMicroseconds(15);

  //set pin to low
  digitalWrite(sensor, LOW);

  //set pinmode to input
  pinMode(sensor, INPUT);

  //get duration of pulse
  int duration = pulseIn(sensor, HIGH);

  //calculate distance in cm
  int distance = duration * 0.034 / 2;

  return distance;


}








// The received command
int receivedCommand = 0;
int direction = 0;  // Add this line

// The direction of travel
int direction_of_travel() {
  return direction;
}








// Motor_A is on the right, Motor_B is on the left
class PIDController {
private:
    float kp, ki, kd;
    float previousError;
    float integral;

public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), previousError(0.0), integral(0.0) {}

    int calculate(float error) {
        // Proportional term
        float p = kp * error;
        
        // Integral term
        integral += error;
        float i = ki * integral;

        // Derivative term
        float d = kd * (error - previousError);
        previousError = error;

        // Compute the output
        float output = p + i + d;

        return (int)output;
    } 

    void resetIntegral() {
        integral = 0.0;
    } 

    void updatePIDValues(float newKp, float newKi, float newKd) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
    //print new values
    Serial.print("New PID values: ");
    Serial.print(kp);
    Serial.print(",");
    Serial.print(ki);
    Serial.print(",");
    Serial.println(kd);
  }

    float getKp() const {
        return kp;
    }

    float getKi() const {
        return ki;
    }

    float getKd() const {
        return kd;
    }

};

PIDController leftMotorPID(1.5,0,0.1); 
PIDController rightMotorPID(2.2,0,0.05);


void setup() {
  /*
  //ARBSetup(); // Setup ARB functionallity


  Serial.begin(9600);

  //wait for serial
  while (!Serial);
  

   // Initialize BLE 
   if (!BLE.begin()) {   
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Add service and characteristic
  //BLE.setLocalName("Batfink");
  //BLE.setAdvertisedService(robotService);
  robotService.addCharacteristic(commandCharacteristic);


  BLE.addService(robotService);
  String bleAddress = BLE.address();
  Serial.print("BLE address: ");
  Serial.println(bleAddress);

  commandCharacteristic.writeValue(receivedCommand);

  // Start advertising
  BLE.advertise();

  Serial.println("Robot is waiting for commands...");

  */

  //setup ultrasonic
  //ultrasonic_init();

  //setup serial 
  Serial.begin(9600);
  Motor_setup(); // Setup motor functionallity

  
}

void loop() {
  
  
  //BLEDevice central = BLE.central();

  /*

  // If a device just connected
  if (central && !central.connected()) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
  }

  // If there's an active connection
  if (central.connected()) {
    if (commandCharacteristic.written()) {
      receivedCommand = commandCharacteristic.value();
      if (direction_of_travel() != receivedCommand) { //only call driveRobot if the direction has changed
          direction = receivedCommand;
          driveRobot(receivedCommand);
          resetSync();
      }
    }
 
  }

  //if robot is going backwards or forwards, use PID to control the motors
  if (direction_of_travel() == 1 || direction_of_travel() == 2) {
      controlMotorsToTarget();
  }

  */


  //ultrasonic
  int front_distance = getDistance(FRONT_SENSOR);
  //int right_distance = getDistance(RIGHT_SENSOR);
  //int left_distance = getDistance(LEFT_SENSOR);

  //if front distance is less than 10cm, stop robot
  if (front_distance < 10 && direction_of_travel() == 1) {
    stopRobot();
    direction = 0;
  }

  //if distance is more than 10cm and the robot is stopped, drive
  if (front_distance > 10 && direction_of_travel() == 0) {
    MOTOR_LEFT = 100;
    MOTOR_RIGHT = 100;
    driveRobot(1);
    direction = 1;
  }

  














}


void driveRobot(int command) {
  switch (command) {
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

void controlMotorsToTarget() {

    int TARGET = 1800;
    const int encoderResetThreshold = 1795; // Some value close to 600

    int leftTarget = TARGET;
    int rightTarget = TARGET;

    int leftEncoderValue = leftMotor->getSteps();
    int rightEncoderValue = rightMotor->getSteps();



    // Calculate error for each motor
    float leftError = leftTarget - leftEncoderValue;
    float rightError = rightTarget - rightEncoderValue;

    // Calculate PID for each motor
    int leftAdjustment = leftMotorPID.calculate(leftError);
    int rightAdjustment = rightMotorPID.calculate(rightError);



    // Adjust motor speeds
    MOTOR_LEFT = constrain(100 + leftAdjustment, -200, 170);
    MOTOR_RIGHT = constrain(100 + rightAdjustment, -200, 160);




    // if less than 70, stop the motors as they cant move any further due to torque issues
    if (MOTOR_LEFT < 70) {
        MOTOR_LEFT = 0;
    }

    if (MOTOR_RIGHT < 70) {
        MOTOR_RIGHT = 0;
    }

    // print motorLeftencoder, motorrightencoder, target
    Serial.print(leftEncoderValue);
    Serial.print(",");
    Serial.print(rightEncoderValue);
    Serial.print(",");
    Serial.print(TARGET);
    Serial.print(",");
    Serial.println(0);

    // Set motor speeds
    leftMotor->setSpeed(MOTOR_LEFT);
    rightMotor->setSpeed(MOTOR_RIGHT);

}

//reset all sync variables
void resetSync() {
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();
    leftMotorPID.resetIntegral();
    rightMotorPID.resetIntegral();
}
