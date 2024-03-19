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

#define TIME_STEP 4

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
double x;
double y;
double z;
double hover_gain;
const double base_gain = 5;
int takeoff_it = 0;
double velo = 6;

//defining functions
//function definitions
void takeoff(double target_h) {
    motor_xpyp->setVelocity(-velo); 
    motor_xpym->setVelocity(velo);
    motor_xmym->setVelocity(velo);
    motor_xmyp->setVelocity(velo);
    x = gps->getValues()[0];
    y = gps->getValues()[1];
    z = gps->getValues()[2];
    cout << z << endl;
        
    if (z < target_h && takeoff_it == 0) {
        motor_xpyp->setVelocity(-300); 
        motor_xpym->setVelocity(300);
        motor_xmym->setVelocity(-300);
        motor_xmyp->setVelocity(300);
        x = gps->getValues()[0];
        y = gps->getValues()[1];
        z = gps->getValues()[2];
            
    }

    if (z > target_h) {
        takeoff_it = 1;
        hover_gain = base_gain * abs(target_h - z);
        motor_xpyp->setVelocity(-velo + hover_gain); 
        motor_xpym->setVelocity(velo - hover_gain);
        motor_xmym->setVelocity(-velo + hover_gain);
        motor_xmyp->setVelocity(velo - hover_gain);
        x = gps->getValues()[0];
        y = gps->getValues()[1];
        z = gps->getValues()[2];
    }

    if (z < target_h && takeoff_it == 1) {
        hover_gain = base_gain * abs(target_h - z);
        motor_xpyp->setVelocity(-velo - hover_gain); 
        motor_xpym->setVelocity(velo + hover_gain);
        motor_xmym->setVelocity(-velo - hover_gain);
        motor_xmyp->setVelocity(velo + hover_gain);
        x = gps->getValues()[0];
        y = gps->getValues()[1];
        z = gps->getValues()[2];
    }
    
}


//void hover(hover_h);

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
        takeoff(10.0);
        
    }

    delete robot;
    return 0;
}


