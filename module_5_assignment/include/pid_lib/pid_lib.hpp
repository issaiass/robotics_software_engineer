#ifndef PID_HPP
#define PID_HPP

#include <iostream>
using namespace std;

class PID {
  public:
    PID(double Kp, double Ki, double Kd, double cmd_min, double cmd_max);
    double stepPID(double measurement, double setpoint, double dt);
    ~PID();

    double _Kp;              // Proportional gain constant
    double _Ki;              // Integral gain constant
    double _Kd;              // Derivative gain constant
    double _err_prev;        // Previous error
    double _integral_err;    // Integral term
    double _deriv_err_prev;  // Previous derivative
    double _cmd_min;         // minimal clamp
    double _cmd_max;         // maximum clamp
};

#endif