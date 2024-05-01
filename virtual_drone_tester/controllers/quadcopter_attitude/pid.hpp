#ifndef PID
#define PID

struct pid {
    // PID constants
    double kp, ki, kd;

    // ang vel
    double x_angvel, y_angvel, z_angvel;

    // ang pos
    double x_angpos, y_angpos, z_angpos;

    // ang pos reference
    double x_angposref, y_angposref, z_angposref;

    // proportional error
    double px, py, pz;

    // integral error
    double ix, iy, iz;

    // derivative error
    double dx, dy, dz;

    // ang pos prev
    double xprev, yprev, zprev;
};

#endif
