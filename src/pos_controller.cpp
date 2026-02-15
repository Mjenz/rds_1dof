#include "pos_controller.hpp"


PositionController::PositionController(double Kp)
{
    Serial.println("Initializing P position controller...");
    Kp_ = Kp;
    Ki_ = 0.0;
    Kd_ = 0.0;

    Icntrl_ = false;
    Dcntrl_ = false;

    err = 0.0;
    err_int = 0.0;
    err_der = 0.0;
    err_prev = 0.0;
    clamp_val_ = 5.0;


}

PositionController::PositionController(double Kp, double Ki)
{
    Serial.println("Initializing PI position controller...");
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = 0.0;

    Icntrl_ = true;
    Dcntrl_ = false;

    err = 0.0;
    err_int = 0.0;
    err_der = 0.0;
    err_prev = 0.0;
    clamp_val_ = 5.0;

}

PositionController::PositionController(double Kp, double Ki, double Kd)
{
    Serial.println("Initializing PID position controller...");
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    Icntrl_ = true;
    Dcntrl_ = true;

    err = 0.0;
    err_int = 0.0;
    err_der = 0.0;
    err_prev = 0.0;
    clamp_val_ = 5.0;


}

double PositionController::pump_controller(double setpoint, double actual, float shaft_vel)
{
    err = setpoint - actual;
    if (Icntrl_) {err_int += err;}
    // plug - shaft velocity in for
    if (Dcntrl_) { err_der = (shaft_vel);} // divide by dt = 100us
    err_prev = err;

    // clamp err_int
    if (err_int > clamp_val_ && Icntrl_)
    {
        // Serial.println("Clamping integrated error to max.");
        err_int = clamp_val_;
    } 
    else if (err_int < -clamp_val_ && Icntrl_)
    {
        // Serial.println("Clamping integrated error to min.");
        err_int = -clamp_val_;
    }

    return Kp_ * err + Ki_ * err_int + Kd_ * err_der;
}