/**
 * @file bluetooth.cpp
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief 
 * @version 0.1
 * @date 22-01-2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bluetooth.h"


rtos::Thread BLEThread(osPriorityNormal, 4096);

BLEService batfinkService(BLE_SERVICE);


    
//set up characteristics
BLECharacteristic robotStateCharacteristic(BLE_CHARACTERISTIC_ROBOT_STATE_UUID, BLERead | BLENotify, sizeof(int));
BLECharacteristic robotXCharacteristic(BLE_CHARACTERISTIC_ROBOT_X_UUID, BLERead | BLENotify, sizeof(int));
BLECharacteristic robotYCharacteristic(BLE_CHARACTERISTIC_ROBOT_Y_UUID, BLERead | BLENotify, sizeof(int));
BLECharacteristic robotThetaCharacteristic(BLE_CHARACTERISTIC_ROBOT_THETA_UUID, BLERead | BLENotify, sizeof(int));
BLECharacteristic robotFSenCharacteristic(BLE_CHARACTERISTIC_ROBOT_FSEN_UUID, BLERead | BLENotify, sizeof(int));
BLECharacteristic robotLSenCharacteristic(BLE_CHARACTERISTIC_ROBOT_LSEN_UUID, BLERead | BLENotify, sizeof(int));
BLECharacteristic robotRSenCharacteristic(BLE_CHARACTERISTIC_ROBOT_RSEN_UUID, BLERead | BLENotify, sizeof(int));

BLECharacteristic controlDirCharacteristic(BLE_CHARACTERISTIC_CONTROL_DIR_UUID, BLEWrite, sizeof(int));
BLECharacteristic controlDistCharacteristic(BLE_CHARACTERISTIC_CONTROL_DIST_UUID, BLEWrite, sizeof(int));


void BLEThreadWorker(){

    bool hasConnected = false;

    //add characteristics to service
    batfinkService.addCharacteristic(robotStateCharacteristic);
    batfinkService.addCharacteristic(robotXCharacteristic);
    batfinkService.addCharacteristic(robotYCharacteristic);
    batfinkService.addCharacteristic(robotThetaCharacteristic);
    batfinkService.addCharacteristic(robotFSenCharacteristic);
    batfinkService.addCharacteristic(robotLSenCharacteristic);
    batfinkService.addCharacteristic(robotRSenCharacteristic);
    batfinkService.addCharacteristic(controlDirCharacteristic);
    batfinkService.addCharacteristic(controlDistCharacteristic);

    //add service to ble
    BLE.addService(batfinkService);

    BLE.setLocalName("Batfink");

    //start advertising
    BLE.advertise();

    
    while(1){
        BLE.poll();
        rtos::ThisThread::sleep_for(100);
    }
    

}


int bleSetup(){
    
        BLEThread.start(BLEThreadWorker);

        return OK;
}