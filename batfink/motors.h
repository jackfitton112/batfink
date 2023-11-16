//motor thread, mutex and direction variables

rtos::Thread motorThread;
rtos::Mutex motorMutex;
int drive_direction = 0; // 0 = stop, 1 = forward, 2 = backward, 3 = left, 4 = right

void motorDrive(){
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

        //non blocking delay
        rtos::ThisThread::sleep_for(10);

    }

}