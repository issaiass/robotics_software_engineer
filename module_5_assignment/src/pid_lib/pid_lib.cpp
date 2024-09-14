#include "pid_lib/pid_lib.hpp"
#include "rclcpp/rclcpp.hpp"

PID::PID(double Kp, double Ki, double Kd, double cmd_min, double cmd_max) : _Kp(Kp), _Ki(Ki), _Kd(Kd), 
_cmd_min(cmd_min), _cmd_max(cmd_max), _deriv_err_prev(0.0) {}

PID::~PID(){}

double PID::stepPID(double measurement, double setpoint, double dt)
{
    /* Error calculation */
    double err = setpoint - measurement;
    //err = max(_cmd_min, min(err, _cmd_max));
    _integral_err += err*dt;
    double _deriv_err = (err - _err_prev + _deriv_err_prev)/dt;
    if (std::abs(_deriv_err) > _cmd_max) {
        _deriv_err = 0.0;
    }
    // double max_deriv_change = 0.01; // Maximum allowable change in derivative
    // if (std::abs(_deriv_err - _deriv_err_prev) > max_deriv_change) {
    //     _deriv_err = _deriv_err_prev + max_deriv_change * ((_deriv_err > _deriv_err_prev) ? 1.0 : -1.0);
    // }

    /* Error remembering */ 
    _err_prev = err;
    _deriv_err_prev = _deriv_err;
    
    /* Return command */
    double command = _Kp*err + _Ki*_integral_err + _Kd*_deriv_err;
    command = max(_cmd_min, min(command, _cmd_max));
    return command;
}