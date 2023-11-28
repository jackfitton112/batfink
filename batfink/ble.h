

//setup bluetooth
BLEService batfinkService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

<<<<<<< Updated upstream
// charactertistic for sending data
// front, left and right distance
// left encoder, right encoder
BLEByteCharacteristic frontChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic leftChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic rightChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic leftEncoderChar("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic rightEncoderChar("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
=======
//sensor characteristic (read) - all sensor data is in a single characteristic as this stops frame issues
BLEByteCharacteristic sensorChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20);
>>>>>>> Stashed changes

//characteristic for receiving data
// override robot and make it manual
// start and stop robot
BLEByteCharacteristic overrideChar("19B10006-E8F2-537E-4F6C-D104768A1214", BLEWrite);
BLEByteCharacteristic startChar("19B10007-E8F2-537E-4F6C-D104768A1214", BLEWrite);


rtos::Thread ble;
rtos::Thread ble_worker;
rtos::Mutex overrideMutex;
int override = 1;
int rave = 0;

void setup_ble(){

<<<<<<< Updated upstream
    //setup the BLE and start the poll thread
    BLE.begin();
=======
/**
 * @brief Initializes and configures the BLE (Bluetooth Low Energy) setup.
 * 
 * This function sets up the BLE, including device name, service, characteristics, and advertising.
 * It also starts a polling thread for BLE communication.
 * 
 * @note This function assumes that the necessary BLE objects and variables have been declared and defined.
 * 
 * @return void
 */
void ble_setup(){
    //setup ble
    if (!BLE.begin()) {
        Serial.println("starting BLE failed!");
        while (1);
    }
>>>>>>> Stashed changes

    //set the local name peripheral advertises
    BLE.setLocalName("Batfink");

    //set the UUID for the service this peripheral advertises
    BLE.setAdvertisedService(batfinkService);

<<<<<<< Updated upstream
    //add the characteristics to the service
    batfinkService.addCharacteristic(frontChar);
    batfinkService.addCharacteristic(leftChar);
    batfinkService.addCharacteristic(rightChar);
    batfinkService.addCharacteristic(leftEncoderChar);
    batfinkService.addCharacteristic(rightEncoderChar);
    batfinkService.addCharacteristic(overrideChar);
    batfinkService.addCharacteristic(startChar);

    //add the service
    BLE.addService(batfinkService);

    //set the initial value for the charactertistic
    frontChar.writeValue(0);
    leftChar.writeValue(0);
    rightChar.writeValue(0);
    leftEncoderChar.writeValue(0);
    rightEncoderChar.writeValue(0);
    overrideChar.writeValue(0);
    startChar.writeValue(0);
=======
    //add characteristics
    batfinkService.addCharacteristic(controlChar);
    batfinkService.addCharacteristic(sensorChar);

>>>>>>> Stashed changes

    //start advertising
    BLE.advertise();


    while(!BLE.connected()){
        //non blocking delay
        rtos::ThisThread::sleep_for(10);
    }

    //print the address of the connected device
    Serial.print("Connected to: ");
    Serial.println(BLE.address());

<<<<<<< Updated upstream

}

void ble_thread(){
    //keep ble running
    while(1){
=======
    //start polling thread and events thread
    blePollThread.start(ble_loop);
    bleEventsThread.start(ble_events);


}


/**
 * @brief This function runs an infinite loop to continuously poll the BLE module.
 * 
 * @details The function calls the `poll()` function of the BLE module in a while loop to ensure continuous polling.
 * 
 * @note This function should be called after initializing the BLE module.
 */
void ble_loop(){
    while(1){

        //poll ble for events / changes
>>>>>>> Stashed changes
        BLE.poll();

        //sleep for 50ms
        rtos::ThisThread::sleep_for(50);

    }
}



/**
 * @brief Continuously reads sensor data and updates it over BLE.
 * 
 * This function runs in an infinite loop and performs the following steps:
 * 1. Reads sensor data from the sensors.
 * 2. Retrieves the current encoder steps from the left and right motors.
 * 3. Takes a mutex lock to ensure exclusive access to the BLE resources.
 * 4. Converts the sensor data into bytes and writes it to the BLE characteristic.
 * 5. Releases the mutex lock.
 * 6. Sleeps for 50 milliseconds before repeating the process.
 */
void ble_events(){
    while(1){
        //ignoreing control for now

        //read sensor data
        int* sensorData = readSensorData();
        int leftencoder = leftmotor->getSteps();
        int rightencoder = rightmotor->getSteps();

        //take mutex and update sensor data
        bleMutex.lock();
        uint8_t* sensorDataBytes = reinterpret_cast<uint8_t*>(sensorData);
        sensorChar.writeValue(sensorDataBytes, sizeof(int) * 3); // write sensor data as bytes
        bleMutex.unlock();

        //sleep for 50ms
        rtos::ThisThread::sleep_for(50);
    }
}

void ble_worker_thread(){

    int prevStartValue = 48; //ascii value for 0


    while (1){
        //check if the start char is 48 (0 in ascii) 
        if (startChar.value() == 49){
            //take mutex and set override to 0
            overrideMutex.lock();
            override = 0;
            rave = 0;
            overrideMutex.unlock();

            prevStartValue = 49;

        } 

        //check if the start char is 48 (0 in ascii)
        if (startChar.value() == 48){
            //take mutex and set override to 1
            overrideMutex.lock();
            override = 1;
            rave = 0;
            overrideMutex.unlock();

            //stop the robot
            motorMutex.lock();
            drive_direction = 0;
            motorMutex.unlock();

            prevStartValue = 48;

        }


        //check if start char is 50 (2 in ascii), if so, enter rave mode
        /*
        if (startChar.value() == 50){
            //take mutex and set override to 1
            overrideMutex.lock();
            override = 1;
            rave = 1;
            overrideMutex.unlock();

            //stop the robot
            motorMutex.lock();
            drive_direction = 0;
            motorMutex.unlock();

            prevStartValue = 50;

        }
        */
        // write the value of the sensor data to the characteristic

        //take mutexand check distance
        int* dist = readSensorData(); //dist[0] = front, dist[1] = left, dist[2] = right

        //write the values to the characteristics as bytes
        frontChar.writeValue((byte)dist[0]);
        leftChar.writeValue((byte)dist[1]);
        rightChar.writeValue((byte)dist[2]);



    }
    
}



