

//setup bluetooth
BLEService batfinkService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

// charactertistic for sending data
// front, left and right distance
// left encoder, right encoder
BLEByteCharacteristic frontChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic leftChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic rightChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic leftEncoderChar("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic rightEncoderChar("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

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

    //setup the BLE and start the poll thread
    BLE.begin();

    //set the local name peripheral advertises
    BLE.setLocalName("Batfink");

    //set the UUID for the service this peripheral advertises
    BLE.setAdvertisedService(batfinkService);

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

    //start advertising
    BLE.advertise();


    while(!BLE.connected()){
        //non blocking delay
        rtos::ThisThread::sleep_for(10);
    }

    //print the address of the connected device
    Serial.print("Connected to: ");
    Serial.println(BLE.address());


}

void ble_thread(){
    //keep ble running
    while(1){
        BLE.poll();
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



