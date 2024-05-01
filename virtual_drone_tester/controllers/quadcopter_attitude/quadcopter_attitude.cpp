#include <stdio.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Keyboard.hpp>
#include "pid.hpp"

#define TIME_STEP 32

using namespace webots;

// initialize objects

Robot *robot = new Robot();
Motor *motor_xpyp, *motor_xmyp, *motor_xmym, *motor_xpym;
Accelerometer *accelerometer;
GPS *gps;
Gyro *gyro;
InertialUnit *imu;
Keyboard *kb;

const double pi = 3.14159;

pid attitude{10, 0, 10, 0, 0, 0, 0, 0, 0, .1, .1, .1, 0, 0, 0, 0, 0, 0, 0, 0, 0, .1, .1, .1};

// declare functions

void setup();
void angular_measurements();
void stabilize();

// define constants and variables

double hover = 560;
double xpyp = 0, xpym = 0, xmym = 0, xmyp = 0;
double uuvv[6];

int main() {

    setup();

    int key = -1, key_flag = true; int i = 0;
    int file_flag = false, pid_flag = false;
    FILE *fptr = fopen("data.csv", "w");
    FILE *fptp = fopen("temp.csv", "w");

    // main loop
    while (robot->step(TIME_STEP) != -1) {

        // ++i; if (i >= 500) pid_flag = true;
        int input = kb->getKey();
        if (key_flag) {
            key = input;
            if (input != -1)
                key_flag = false;
        } else {
            key = -1;
            if (input == -1)
                key_flag = true;
        }

        if (input == 'P') { pid_flag = true; file_flag = true; }
        if (input == 'Q') { fclose(fptr); file_flag = false; }
        if (pid_flag) { angular_measurements(); stabilize(); }
        // set motor speeds
        motor_xpyp->setVelocity(-hover-xpyp);
        motor_xmyp->setVelocity(hover+xmyp);
        motor_xmym->setVelocity(-hover-xmym);
        motor_xpym->setVelocity(hover+xpym);
        const double *temp = gyro->getValues();
        fprintf(fptp, "%6.3e,%6.3e,%6.3e\n", temp[0], temp[1], temp[2]);
        printf("%6.3e\t%6.3e\t%6.3e\n", temp[0], temp[1], temp[2]);
        printf("%6.3e\t%6.3e\t%6.3e\n", uuvv[0], uuvv[1], uuvv[2]);
        
        printf("Posit: %6.3e\t%6.3e\t%6.3e\n", attitude.x_angpos, attitude.y_angpos, attitude.z_angpos);
        printf("Error: %6.3e\t%6.3e\t%6.3e\n", attitude.px, attitude.py, attitude.pz);
        printf("Intgr: %6.3e\t%6.3e\t%6.3e\n", attitude.ix, attitude.iy, attitude.iz);
        printf("Deriv: %6.3e\t%6.3e\t%6.3e\n", attitude.dx, attitude.dy, attitude.dz);
        printf("Commd: %6.3e\t%6.3e\t%6.3e\t%6.3e\n", xpyp, xmyp, xmym, xpym);
        if (file_flag)
            fprintf(fptr, "%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e,%6.3e\n", attitude.x_angpos, attitude.y_angpos, attitude.z_angpos, attitude.px, attitude.py, attitude.pz, attitude.ix, attitude.iy, attitude.iz, attitude.dx, attitude.dy, attitude.dz, xpyp, xmyp, xmym, xpym);
        // printf("%6.3f\t%6.3f\t%6.3f\t%6.3f\n", encoder_xpyp->getValue(), encoder_xmyp->getValue(), encoder_xmym->getValue(), encoder_xpym->getValue());
    }
    
    delete robot;
    return 0;
}

// define functions

/**
 * @brief This does what you think it does
*/
void setup() {
    motor_xpyp = robot->getMotor("motor_xpyp");
    motor_xpyp->setPosition(INFINITY);
    motor_xpyp->setVelocity(0.0);

    motor_xmyp = robot->getMotor("motor_xmyp");
    motor_xmyp->setPosition(INFINITY);
    motor_xmyp->setVelocity(0.0);

    motor_xmym = robot->getMotor("motor_xmym");
    motor_xmym->setPosition(INFINITY);
    motor_xmym->setVelocity(0.0);

    motor_xpym = robot->getMotor("motor_xpym");
    motor_xpym->setPosition(INFINITY);
    motor_xpym->setVelocity(0.0);

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
}

/**
 * @brief Read angular measurements
*/
void angular_measurements() {
    const double *temp = imu->getRollPitchYaw();
    attitude.x_angpos = temp[0];
    attitude.y_angpos = temp[1];
    attitude.z_angpos = temp[2];
}

/**
 * @brief PID control to stabilize attitude (angular position)
*/
void stabilize() {
    // find error and integral of error
    attitude.px = attitude.x_angposref-attitude.x_angpos;
    attitude.py = attitude.y_angposref-attitude.y_angpos;
    attitude.pz = attitude.z_angposref-attitude.z_angpos;
    attitude.ix += attitude.px*TIME_STEP/1000;
    attitude.iy += attitude.py*TIME_STEP/1000;
    attitude.iz += attitude.pz*TIME_STEP/1000;
    attitude.dx = (attitude.px-attitude.xprev)/TIME_STEP*1000;
    attitude.dy = (attitude.py-attitude.yprev)/TIME_STEP*1000;
    attitude.dz = (attitude.pz-attitude.zprev)/TIME_STEP*1000;
    attitude.xprev = attitude.px;
    attitude.yprev = attitude.py;
    attitude.zprev = attitude.pz;
    uuvv[0] = (uuvv[3]-attitude.x_angpos)/TIME_STEP*1000;
    uuvv[1] = (uuvv[4]-attitude.y_angpos)/TIME_STEP*1000;
    uuvv[2] = (uuvv[5]-attitude.z_angpos)/TIME_STEP*1000;
    uuvv[3] = attitude.x_angpos;
    uuvv[4] = attitude.y_angpos;
    uuvv[5] = attitude.z_angpos;

    // small angle approximation, assumes motors are oriented at 45 deg relative to axes
    // for positive x, +xpyp, +xmyp, -xmym, -xpym
    // for positive y, -xpyp, +xmyp, +xmym, -xpym
    // for positive z, +xpyp, -xmyp, +xmym, -xpym
    // rotation of axes:
    // for positive x, x' = <1, 0, 0>, y' = <0, cos x, sin x>, z' = <0, -sin x, cos x>
    // for posiitve y, x' = <cos y, 0, -sin y>, y' = <0, 1, 0>, z' = <sin y, 0, cos y>
    // for positive z, x' = <cos z, sin z, 0>, y' = <-sin z, cos z, 0>, z' = <0, 0, 1>
    xpyp = + attitude.kp*attitude.px - attitude.kp*attitude.py + attitude.kp*attitude.pz \
           + attitude.ki*attitude.ix - attitude.ki*attitude.iy + attitude.ki*attitude.iz \
           + attitude.kd*attitude.dx - attitude.kd*attitude.dy + attitude.kd*attitude.dz;
    xmyp = + attitude.kp*attitude.px + attitude.kp*attitude.py - attitude.kp*attitude.pz \
           + attitude.ki*attitude.ix + attitude.ki*attitude.iy - attitude.ki*attitude.iz \
           + attitude.kd*attitude.dx + attitude.kd*attitude.dy - attitude.kd*attitude.dz;
    xmym = - attitude.kp*attitude.px + attitude.kp*attitude.py + attitude.kp*attitude.pz \
           - attitude.ki*attitude.ix + attitude.ki*attitude.iy + attitude.ki*attitude.iz \
           - attitude.kd*attitude.dx + attitude.kd*attitude.dy + attitude.kd*attitude.dz;
    xpym = - attitude.kp*attitude.px - attitude.kp*attitude.py - attitude.kp*attitude.pz \
           - attitude.ki*attitude.ix - attitude.ki*attitude.iy - attitude.ki*attitude.iz \
           - attitude.kd*attitude.dx - attitude.kd*attitude.dy - attitude.kd*attitude.dz;
    printf("kp: %6.3e\t%6.3e\t%6.3e\n", attitude.kp*attitude.px, attitude.kp*attitude.py, attitude.kp*attitude.pz);
    printf("ki: %6.3e\t%6.3e\t%6.3e\n", attitude.ki*attitude.ix, attitude.ki*attitude.iy, attitude.ki*attitude.iz);
    printf("kd: %6.3e\t%6.3e\t%6.3e\n", attitude.kd*attitude.dx, attitude.kd*attitude.dy, attitude.kd*attitude.dz);
}
