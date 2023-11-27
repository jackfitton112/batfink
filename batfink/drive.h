void turn_left(){
        //turn left
        motorMutex.lock();
        drive_direction = 3;
        motorMutex.unlock();

        //wait for 1 second
        rtos::ThisThread::sleep_for(2000);

        //stop
        motorMutex.lock();
        drive_direction = 0;
        motorMutex.unlock();
}

void turn_right(){
        //turn right
        motorMutex.lock();
        drive_direction = 4;
        motorMutex.unlock();

        //wait for 1 second
        rtos::ThisThread::sleep_for(2000);

        //stop
        motorMutex.lock();
        drive_direction = 0;
        motorMutex.unlock();

}

void go_forward(){
        //go forward
        motorMutex.lock();
        drive_direction = 1;
        motorMutex.unlock();

}

void rave_mode(){
    //make the robot rave
    turn_left();
    turn_right();

    //wait for 0.5 seconds
    rtos::ThisThread::sleep_for(500);
    
}