#ifndef PID
#define PID

class pid_velo {
    public:
        
        //position weight
        double kw_pos;

        //constants
        double kp_velo;
        double ki_velo;
        double kd_velo;

        //low pass filter
        //double tau_velo;

        //motor output limits
        double max_velo;
        double min_velo;

        //error term
        double error_x_v;
        double error_y_v;
        double error_z_v;

        //memory

        //velocity
        double x_v_prev;
        double y_v_prev;
        double z_v_prev;

        //velocity set values
        double x_v_sp;
        double y_x_sp;
        double z_v_sp;

        //pid terms
        double x_v_integrator;
        double x_v_differentiator;
        double error_x_v_prev;
        double y_v_integrator;
        double y_v_differentiator;
        double error_y_v_prev;
        double z_v_integrator;
        double z_v_differentiator;
        double error_z_v_prev;




} pid_velo_controller;

class p_pos {
    public:

        //gains
        double kp_pos;
        
        //error term
        double error_x;
        double error_y;
        double error_z;

        //memory - position
        double x_prev;
        double y_prev;
        double z_prev;

        //output
        double x_out;
        double y_out;
        double z_out;

} p_pos_controller;

class pid_angr {
    public:
} pid_angr;

class p_ang {
    public:
} p_ang;

#endif