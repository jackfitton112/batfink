rtos::Thread frontusonicThread;
rtos::Thread leftusonicThread;
rtos::Thread rightusonicThread;

rtos::Mutex frontusonicMutex;
rtos::Mutex leftusonicMutex;
rtos::Mutex rightusonicMutex;
rtos::Mutex globalSensorMutex;

int front_distance = 0;
int left_distance = 0;
int right_distance = 0;

mbed::DigitalInOut frontSensor(P0_23);
mbed::DigitalInOut rightSensor(P1_14);
mbed::DigitalInOut leftSensor(P1_13);



void frontusonicRead() {
    while (1) {
        
        //get global sensor mutex
        globalSensorMutex.lock();

        // Read distance from the front sensor
        frontSensor.output();
        frontSensor = 0;  // Set the pin low for 2 microseconds
        wait_us(2);
        frontSensor = 1;  // Set the pin high for 15 microseconds
        wait_us(15);
        frontSensor = 0;  // Set the pin low
        frontSensor.input();  // Set the pin to input
        wait_us(20);  // Wait for the pulse to start

        mbed::Timer timer;
        timer.start();

        // Wait for the pulse to end
        while (frontSensor.read() == 0 && timer.read_us() < 5000) {
        }

        timer.reset();

        // Measure the pulse duration
        while (frontSensor.read() == 1 && timer.read_us() < 5000) {
        }

        timer.stop();

        //release global sensor mutex
        globalSensorMutex.unlock();

        // Acquire the mutex to prevent conflicts with other threads
        frontusonicMutex.lock();

        // Calculate distance based on the pulse duration
        front_distance = static_cast<int>(timer.read_us() * 0.0343 / 2);

        // Release the mutex
        frontusonicMutex.unlock();



        // Sleep for a while to control the sampling rate
        rtos::ThisThread::sleep_for(100);  // Sleep for 100 milliseconds (adjust as needed)
    }
}

void leftusonicRead() {

        while (1) {


        //get global sensor mutex
        globalSensorMutex.lock();
        
        // Read distance from the left sensor
        leftSensor.output();
        leftSensor = 0;  // Set the pin low for 2 microseconds
        wait_us(2);
        leftSensor = 1;  // Set the pin high for 15 microseconds
        wait_us(15);
        leftSensor = 0;  // Set the pin low
        leftSensor.input();  // Set the pin to input
        wait_us(20);  // Wait for the pulse to start

        mbed::Timer timer;
        timer.start();

        // Wait for the pulse to end
        while (leftSensor.read() == 0 && timer.read_us() < 5000) {
        }

        timer.reset();

        // Measure the pulse duration
        while (leftSensor.read() == 1 && timer.read_us() < 5000) {
        }

        timer.stop();

        //release global sensor mutex
        globalSensorMutex.unlock();

        // Acquire the mutex to prevent conflicts with other threads
        leftusonicMutex.lock();

        // Calculate distance based on the pulse duration
        left_distance = static_cast<int>(timer.read_us() * 0.0343 / 2);

        // Release the mutex
        leftusonicMutex.unlock();



        // Sleep for a while to control the sampling rate
        rtos::ThisThread::sleep_for(100);  // Sleep for 100 milliseconds (adjust as needed)
    }

}

void rightusonicRead() {

        while (1) {

        
        //get global sensor mutex
        globalSensorMutex.lock();
        
        // Read distance from the right sensor
        rightSensor.output();
        rightSensor = 0;  // Set the pin low for 2 microseconds
        wait_us(2);
        rightSensor = 1;  // Set the pin high for 15 microseconds
        wait_us(15);
        rightSensor = 0;  // Set the pin low
        rightSensor.input();  // Set the pin to input
        wait_us(20);  // Wait for the pulse to start

        mbed::Timer timer;
        timer.start();

        // Wait for the pulse to end
        while (rightSensor.read() == 0 && timer.read_us() < 5000) {
        }

        timer.reset();

        // Measure the pulse duration
        while (rightSensor.read() == 1 && timer.read_us() < 5000) {
        }

        timer.stop();

        //release global sensor mutex
        globalSensorMutex.unlock();

        // Acquire the mutex to prevent conflicts with other threads
        rightusonicMutex.lock();

        // Calculate distance based on the pulse duration
        right_distance = static_cast<int>(timer.read_us() * 0.0343 / 2);

        // Release the mutex
        rightusonicMutex.unlock();



        // Sleep for a while to control the sampling rate
        rtos::ThisThread::sleep_for(100);  // Sleep for 100 milliseconds (adjust as needed)
    }

}

int* readSensorData() {
    // get all sensor values and put them in an array
    static int sensorData[3];  // static to ensure it persists outside the function

    // take mutexes and get sensor values
    frontusonicMutex.lock();
    sensorData[0] = front_distance;
    frontusonicMutex.unlock();

    leftusonicMutex.lock();
    sensorData[1] = left_distance;
    leftusonicMutex.unlock();

    rightusonicMutex.lock();
    sensorData[2] = right_distance;
    rightusonicMutex.unlock();

    return sensorData;
}