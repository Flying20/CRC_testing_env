#include <stdio.h>
#include <math.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
// #include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Keyboard.hpp>
#include "pid.hpp"

using namespace webots;  
using namespace std;

#define TIME_STEP 32

// initialize objects

Robot *robot = new Robot();
Motor *motor_xpyp, *motor_xmyp, *motor_xmym, *motor_xpym;
// PositionSensor *encoder_xpyp, *encoder_xmyp, *encoder_xmym, *encoder_xpym;
Accelerometer *accelerometer;
GPS *gps;
Gyro *gyro;
InertialUnit *imu;
Keyboard *kb;

pid_ang ang_control{10, 1, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// declare functions

double normalize_angle(double);
void angular_measurements();
void stabilize();

// define constants and variables

constexpr double pi = 3.14159265358979323846;
double xpyp = 25, xpym = 25, xmym = 25, xmyp = 25;

int main() {

    motor_xpyp = robot->getMotor("motor_xpyp");
    motor_xpyp->setPosition(INFINITY);
    motor_xpyp->setVelocity(0.0);
    // encoder_xpyp = motor_xpyp->getPositionSensor();
    // encoder_xpyp->enable(TIME_STEP);

    motor_xmyp = robot->getMotor("motor_xmyp");
    motor_xmyp->setPosition(INFINITY);
    motor_xmyp->setVelocity(0.0);
    // encoder_xmyp = motor_xmyp->getPositionSensor();
    // encoder_xmyp->enable(TIME_STEP);

    motor_xmym = robot->getMotor("motor_xmym");
    motor_xmym->setPosition(INFINITY);
    motor_xmym->setVelocity(0.0);
    // encoder_xmym = motor_xmym->getPositionSensor();
    // encoder_xmym->enable(TIME_STEP);

    motor_xpym = robot->getMotor("motor_xpym");
    motor_xpym->setPosition(INFINITY);
    motor_xpym->setVelocity(0.0);
    // encoder_xpym = motor_xpym->getPositionSensor();
    // encoder_xpym->enable(TIME_STEP);

    gps = robot->getGPS("gps");
    gps->enable(TIME_STEP);

    accelerometer = robot->getAccelerometer("accelerometer");
    accelerometer->enable(TIME_STEP);

    gyro = robot->getGyro("gyro");
    gyro->enable(TIME_STEP);

    imu = robot->getInertialUnit("imu");
    imu->enable(TIME_STEP);
    
    kb = robot->getKeyboard();
    kb->enable(TIME_STEP);

    int key = -1, key_flag = true;

    // main loop
    while (robot->step(TIME_STEP) != -1) {
        
        stabilize();

        int input = kb->getKey();
        printf("%d\t%c\t", key_flag, input);
        if (key_flag) {
            key = input;
            if (input != -1)
                key_flag = false;
        } else {
            key = -1;
            if (input == -1)
                key_flag = true;
        }
        printf("%c\n", key);

        if (key == '1') { xpyp += .1; }
        if (key == '2') { xmyp += .1; }
        if (key == '3') { xmym += .1; }
        if (key == '4') { xpym += .1; }
        if (key == '5') { xpyp -= .1; }
        if (key == '6') { xmyp -= .1; }
        if (key == '7') { xmym -= .1; }
        if (key == '8') { xpym -= .1; }
        if (key == '9') { xpyp += 5; xmyp += 5; xmym += 5; xpym += 5; }
        if (key == '0') { xpyp -= 5; xmyp -= 5; xmym -= 5; xpym -= 5; }
        // set motor speeds
        motor_xpyp->setVelocity(-xpyp);
        motor_xmyp->setVelocity(xmyp);
        motor_xmym->setVelocity(-xmym);
        motor_xpym->setVelocity(xpym);

        printf("%6.3e\t%6.3e\t%6.3e\n", ang_control.x_angpos, ang_control.y_angpos, ang_control.z_angpos);
        printf("%6.3f\t%6.3f\t%6.3f\t%6.3f\n", xpyp, xmyp, xmym, xpym);
        // printf("%6.3f\t%6.3f\t%6.3f\t%6.3f\n", encoder_xpyp->getValue(), encoder_xmyp->getValue(), encoder_xmym->getValue(), encoder_xpym->getValue());
    }
    
    delete robot;
    return 0;
}

// define functions

/**
 * @brief This does what you think it does
 * @param angle Angle to normalize
 * @return Normalized angle in [-pi, pi)
*/
double normalize_angle(double angle) {
    angle = fmod(angle+pi, 2*pi);
    if (angle < 0)
        angle += 2*pi;
    return angle-pi;
}

/**
 * @brief Read angular measurements
*/
void angular_measurements() {
    // integrate angular velocity to find angular position and normalize to [-pi, pi}
    /*
    const double *temp = gyro->getValues();
    ang_control.x_angvel = temp[0];
    ang_control.y_angvel = temp[1];
    ang_control.z_angvel = temp[2];
    ang_control.x_angpos = normalize_angle(ang_control.x_angpos+ang_control.x_angvel*TIME_STEP/1000);
    ang_control.y_angpos = normalize_angle(ang_control.y_angpos+ang_control.y_angvel*TIME_STEP/1000);
    ang_control.z_angpos = normalize_angle(ang_control.z_angpos+ang_control.z_angvel*TIME_STEP/1000);
    */
    const double *temp = imu->getRollPitchYaw();
    ang_control.x_angpos = normalize_angle(temp[0]);
    ang_control.y_angpos = normalize_angle(temp[1]);
    ang_control.z_angpos = normalize_angle(temp[2]);
    printf("\t%6.3f\t%6.3f\t%6.3f\n", temp[0], temp[1], temp[2]);
}

/**
 * @brief PID control to stabilize attitude (angular position)
*/
void stabilize() {
    angular_measurements();
    // find error and integral of error
    ang_control.x_angposerr = ang_control.x_angposref-ang_control.x_angpos;
    ang_control.y_angposerr = ang_control.y_angposref-ang_control.y_angpos;
    ang_control.z_angposerr = ang_control.z_angposref-ang_control.z_angpos;
    ang_control.x_angposerrint += ang_control.x_angposerr*TIME_STEP/1000;
    ang_control.y_angposerrint += ang_control.y_angposerr*TIME_STEP/1000;
    ang_control.z_angposerrint += ang_control.z_angposerr*TIME_STEP/1000;
    ang_control.x_angposerrder = (ang_control.x_angposerr-ang_control.x_angposerrprev)/TIME_STEP*1000;
    ang_control.y_angposerrder = (ang_control.y_angposerr-ang_control.y_angposerrprev)/TIME_STEP*1000;
    ang_control.z_angposerrder = (ang_control.z_angposerr-ang_control.z_angposerrprev)/TIME_STEP*1000;

    // this applies near z = 0, increase means magntiude only btw, n means any
    // if positive x deviation, increase xnym and decrease xnyp
    // if positive y deviation, increase xpyn and decrease xmyn
    // if positive z deviation, increase xmyp and decrease xpyp
    xpyp += + ang_control.kp*ang_control.x_angposerr - ang_control.kp*ang_control.y_angposerr + ang_control.kp*ang_control.z_angposerr \
            + ang_control.ki*ang_control.x_angposerrint - ang_control.ki*ang_control.y_angposerrint + ang_control.ki*ang_control.z_angposerrint \
            + ang_control.ki*ang_control.x_angposerrder - ang_control.kd*ang_control.y_angposerrder + ang_control.kd*ang_control.z_angposerrder;
    xmyp += + ang_control.kp*ang_control.x_angposerr + ang_control.kp*ang_control.y_angposerr - ang_control.kp*ang_control.z_angposerr \
            + ang_control.ki*ang_control.x_angposerrint + ang_control.ki*ang_control.y_angposerrint - ang_control.ki*ang_control.z_angposerrint \
            + ang_control.kd*ang_control.x_angposerrder + ang_control.kd*ang_control.y_angposerrder - ang_control.kd*ang_control.z_angposerrder;
    xmym += - ang_control.kp*ang_control.x_angposerr + ang_control.kp*ang_control.y_angposerr + ang_control.kp*ang_control.z_angposerr \
            - ang_control.ki*ang_control.x_angposerrint + ang_control.ki*ang_control.y_angposerrint + ang_control.ki*ang_control.z_angposerrint \
            - ang_control.kd*ang_control.x_angposerrder + ang_control.kd*ang_control.y_angposerrder + ang_control.kd*ang_control.z_angposerrder;
    xpym += - ang_control.kp*ang_control.x_angposerr - ang_control.kp*ang_control.y_angposerr - ang_control.kp*ang_control.z_angposerr \
            - ang_control.ki*ang_control.x_angposerrint - ang_control.ki*ang_control.y_angposerrint - ang_control.ki*ang_control.z_angposerrint \
            - ang_control.kd*ang_control.x_angposerrder - ang_control.kd*ang_control.y_angposerrder - ang_control.kd*ang_control.z_angposerrder;
}
