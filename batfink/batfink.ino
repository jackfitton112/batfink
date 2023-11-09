
//includes
#include <Arduino.h>
#include <ARB.h>
#include <ArduinoBLE.h>
#include "MotorControl.h"

#define FRONT_SENSOR USONIC1
#define RIGHT_SENSOR USONIC2
#define LEFT_SENSOR USONIC3

// BLE Service and Characteristic
BLEService robotService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharCharacteristic commandCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);





// The received command
int receivedCommand = 0;
int direction = 0;  // Add this line

// The direction of travel
int direction_of_travel() {
  return direction;
}

//git distance from one of the 3 ultrasonic sensors
int get_distance(int sensor) {

	pinMode(sensor, OUTPUT);
	digitalWrite(sensor, LOW);
	delayMicroseconds(2);
	digitalWrite(sensor, HIGH);
	delayMicroseconds(15);
	digitalWrite(sensor, LOW);

	// The same pin is used to read back the returning signal, so must be set back to input
	pinMode(sensor, INPUT);
	int duration = pulseIn(sensor, HIGH);

  int distance =  uSecToCM(duration);


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
 
  ARBSetup(); // Setup ARB functionallity
  Motor_setup(); // Setup motor functionallity

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

  
}

void loop() {
  
  
  BLEDevice central = BLE.central();

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




  //get distance from front sensor
  int front_distance = get_distance(FRONT_SENSOR);

  //if front distance is less than 10cm and direction is forward, stop robot and turn left
  if (front_distance < 10 && direction_of_travel() == 1) {
      Serial.println("Obstacle detected, turning left");
      stopRobot();
      delay(1000);
      left();
      delay(1000);
      stopRobot();
      delay(1000);
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
