#ifndef PID
#define PID

class pid_velo {
    public:
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

struct pid_ang {
    // constants
    double kp, ki, kd;

    // ang vel
    double x_angvel, y_angvel, z_angvel;

    // ang pos
    double x_angpos, y_angpos, z_angpos;

    // ang pos reference
    double x_angposref, y_angposref, z_angposref;

    // ang pos error
    double x_angposerr, y_angposerr, z_angposerr;

    // ang pos error integral
    double x_angposerrint, y_angposerrint, z_angposerrint;

    // ang pos error derivative
    double x_angposerrder, y_angposerrder, z_angposerrder;

    // ang pos prev
    double x_angposerrprev, y_angposerrprev, z_angposerrprev;
};

#endif