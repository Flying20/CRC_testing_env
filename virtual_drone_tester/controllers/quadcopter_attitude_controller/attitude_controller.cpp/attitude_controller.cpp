//#include <webots/stdio.hpp>
//#include <webots/stdlib.hpp>
#include <iostream>
//#include <webots/String.hpp>
#include <cmath>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Keyboard.hpp>

using namespace webots;  
using namespace std;

#define north 0
#define south -pi
#define east -pi/2
#define west pi/2

#define TIME_STEP 32

//initializing objects
Robot *robot = new Robot();
Motor *motor_xpyp;
Motor *motor_xpym;
Motor *motor_xmym;
Motor *motor_xmyp;
Accelerometer *accelerometer;
GPS *gps;
Gyro *gyro;
InertialUnit *imu;
Keyboard kb;

//defining functions
//void pilot_control();

//defining constants and variables


int main() {

    motor_xpyp=robot->getMotor("motor_xpyp");
    motor_xpyp->setPosition(INFINITY);
    motor_xpyp->setVelocity(0.0);

    motor_xpym=robot->getMotor("motor_xpym");
    motor_xpym->setPosition(INFINITY);
    motor_xpym->setVelocity(0.0);

    motor_xmym=robot->getMotor("motor_xmym");
    motor_xmym->setPosition(INFINITY);
    motor_xmym->setVelocity(0.0);

    motor_xmyp=robot->getMotor("motor_xmyp");
    motor_xmyp->setPosition(INFINITY);
    motor_xmyp->setVelocity(0.0);

    gps=robot->getGPS("gps");
    gps->enable(TIME_STEP);

    accelerometer=robot->getAccelerometer("accelerometer");
    accelerometer->enable(TIME_STEP);

    gyro=robot->getGyro("gyro");
    gyro->enable(TIME_STEP);

    imu=robot->getInertialUnit("imu");
    imu->enable(TIME_STEP);
    
    
    kb.enable(TIME_STEP);
    

    //main loop
    while (robot->step(TIME_STEP) != -1) {
        //cout<<"Piloted-Mode"<<endl;
        //pilot_control();
        motor_xpyp->setVelocity(-300);
        motor_xpym->setVelocity(300);
        motor_xmym->setVelocity(-300);
        motor_xmyp->setVelocity(300);
    }

    delete robot;
    return 0;
}