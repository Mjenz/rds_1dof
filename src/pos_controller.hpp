#ifndef POS_CONTROLLER_HPP
#define POS_CONTROLLER_HPP

#include <Arduino.h>

class PositionController
{
public:
    PositionController(double Kp);
    PositionController(double Kp, double Ki);
    PositionController(double Kp, double Ki, double Kd);
    double pump_controller(double setpoint, double actual, float shaft_vel);

private:
    double Kp_;
    double Ki_;
    double Kd_;

    double err;
    double err_int;
    double err_der;
    double err_prev;
    double clamp_val_;

    bool Icntrl_;
    bool Dcntrl_;
};

#endif