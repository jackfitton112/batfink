/**
 * @file pid.h
 * @author Jack Fitton (jf1595@york.ac.uk)
 * @brief PID controller header file
 * @version 0.1
 * @date 28-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef PID_H
#define PID_H


class PID {
    public:
        PID(float kp, float ki, float kd, float dt);
        float update(float error);
        void setKp(float kp);
        void setKi(float ki);
        void setKd(float kd);
        void setDt(float dt);
        float getKp();
        float getKi();
        float getKd();
        float getDt();
    private:
        float kp;
        float ki;
        float kd;
        float dt;
        float integral;
        float derivative;
        float previousError;
};

#define KP 0.1
#define KI 0.1
#define KD 0.1
#define DT 0.01

//motors pid
extern PID* leftMotorPID;
extern PID* rightMotorPID;

void goForwardPid(float distance, float targetVelocity);

#endif