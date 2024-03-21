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
#include "pid.hpp"

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
double x = 0;
double y = 0;
double z = 0;
double x_v = 0;
double y_v = 0;
double z_v = 0;
double x_a = 0;
double y_a = 0;
double z_a = 0;

//defining constants for p_pos and pid_velo
#define KP_POS  1
#define ERROR_X  0
#define ERROR_Y 0
#define ERROR_Z 0
#define X_PREV 0.169
#define Y_PREV 0
#define Z_PREV 0
#define X_OUT 0;
#define Y_OUT 0;
#define Z_OUT 0;

#define KP_VELO 1
#define KI_VELO 1
#define KD_VELO 1
//#define TAU_VELO 1
#define MAX_VELO 680
#define MIN_VELO -680
#define ERROR_X_V 0
#define ERROR_Y_V 0
#define ERROR_Z_V 0
#define X_V_PREV 0
#define Y_V_PREV 0
#define Z_V_PREV 0
#define X_V_INTEGRATOR 0
#define X_V_DIFFERENTIATOR 0
#define ERROR_X_V_PREV 0
#define Y_V_INTEGRATOR 0
#define Y_V_DIFFERENTIATOR 0
#define ERROR_Y_V_PREV 0
#define Z_V_INTEGRATOR 0
#define Z_V_DIFFERENTIATOR 0
#define ERROR_Z_V_PREV 0

//create controller objects
p_pos_controller pos_controller {
        KP_POS,
        ERROR_X,
        ERROR_Y,
        ERROR_Z,
        X_PREV,
        Y_PREV,
        Z_PREV,
        X_OUT,
        Y_OUT,
        Z_OUT
    };

pid_velo_controller velo_controller{
        KP_VELO,
        KI_VELO,
        KD_VELO,
        //TAU_VELO,
        MAX_VELO,
        MIN_VELO,
        ERROR_X_V,
        ERROR_Y_V,
        ERROR_Z_V,
        X_V_PREV,
        Y_V_PREV,
        Z_V_PREV,
        X_V_INTEGRATOR,
        X_V_DIFFERENTIATOR,
        ERROR_X_V_PREV,
        Y_V_INTEGRATOR,
        Y_V_DIFFERENTIATOR,
        ERROR_Y_V_PREV,
        Z_V_INTEGRATOR,
        Z_V_DIFFERENTIATOR,
        ERROR_Z_V_PREV,
        X_V_SP,
        Y_V_SP,
        Z_V_SP
    }; 

//function definitions
void position_measurements (double bias_x, double bias_y, double bias_z) {

    pos_controller.x_prev = x;
    pos_controller.y_prev = y;
    pos_controller.z_prev = z;
    x = gps[2];               
    y = gps[0];                 
    z = gps[1];  
    
    //computing position error term
    pos_controller.error_x = x - pos_controller.x_prev;
    pos_controller.error_y = y - pos_controller.z_prev;
    pos_controller.error_z = z - pos_controller.z_prev;

    //output = Kp*error + bias
    pos_controller.x_out = pos_controller.error_x*pos_controller.kp_pos + bias_x;
    pos_controller.y_out = pos_controller.error_y*pos_controller.kp_pos + bias_y;
    pos_controller.z_out = pos_controller.error_z*pos_controller.kp_pos + bias_z;

}

void velo_measurements () {

    pid_velo_controller.x_v_prev = x_v;
    pid_velo_controller.y_v_prev = y_v;
    pid_velo_controller.z_v_prev = z_v;

    //v = dx/dt
    x_v = (x - pos_controller.x_prev)/TIME_STEP;
    y_v = (y - pos_controller.y_prev)/TIME_STEP;
    z_v = (z - pos_controller.z_prev)/TIME_STEP;

    //computing velocity set values, via position proportional controller outputs
    pid_velo_controller.x_v_sp = pid_velo_controller.x_v_prev + (pos_controller.x_out)/TIME_STEP;
    pid_velo_controller.y_v_sp = pid_velo_controller.y_v_prev + (pos_controller.y_out)/TIME_STEP;
    pid_velo_controller.z_v_sp = pid_velo_controller.z_v_prev + (pos_controller.z_out)/TIME_STEP;

    //computing velocity error term
    pid_velo_controller.error_x_v = pid_velo_controller.x_v_sp - x_v;
    pid_velo_controller.error_y_v = pid_velo_controller.y_v_sp - y_v;
    pid_velo_controller.error_z_v = pid_velo_controller.z_v_sp - z_v;

    //
}

void hover(double target_x, double target_y, double target_z) {
    position_measurements(target_x, target_y, target_z);
                   
}


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
        hover(10.0, 0.0, 0.0);
        
    }

    delete robot;
    return 0;
}


