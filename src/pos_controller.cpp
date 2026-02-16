#include "pos_controller.hpp"

PositionController::PositionController(double Kp)
: Kp_(Kp),
  Ki_(0.0),
  Kd_(0.0),
  Kff_(0.001),
  Icntrl_ (false),
  Dcntrl_ (false),
  err_ (0.0),
  err_int_ (0.0),
  err_der_ (0.0),
  err_prev_ (0.0),
  clamp_val_ (5.0),
  feed_fwd_enable_ (true),
  ffwd_term_ (0.0),
  gvty_fwd_enable_ (true),
  gvty_term_ (0.0)
{
    Serial.println("Initializing P position controller...");
}

PositionController::PositionController(double Kp, double Ki)
: Kp_(Kp),
  Ki_(Ki),
  Kd_(0.0),
  Kff_(0.001),
  Icntrl_ (true),
  Dcntrl_ (false),
  err_ (0.0),
  err_int_ (0.0),
  err_der_ (0.0),
  err_prev_ (0.0),
  clamp_val_ (5.0),
  feed_fwd_enable_ (true),
  ffwd_term_ (0.0),
  gvty_fwd_enable_ (true),
  gvty_term_ (0.0)
{
    Serial.println("Initializing PI position controller...");
}

PositionController::PositionController(double Kp, double Ki, double Kd)
: Kp_(Kp),
  Ki_(Ki),
  Kd_(Kd),
  Kff_(0.001),
  Icntrl_ (true),
  Dcntrl_ (true),
  err_ (0.0),
  err_int_ (0.0),
  err_der_ (0.0),
  err_prev_ (0.0),
  clamp_val_ (5.0),
  feed_fwd_enable_ (true),
  ffwd_term_ (0.0),
  gvty_fwd_enable_ (true),
  gvty_term_ (0.0)
{
    Serial.println("Initializing PID position controller...");
}

void PositionController::set_ffwd_control(bool enable)
{
    feed_fwd_enable_ = enable;
}

void PositionController::set_gvty_compensation(bool enable)
{
    gvty_fwd_enable_ = enable;
}

void PositionController::set_clamp_val(double clamp_val)
{
    clamp_val_ = clamp_val;
}

double PositionController::pump_controller(double setpoint, double actual, double next_cmd, float shaft_vel)
{

    err_ = setpoint - actual;
    if (gvty_fwd_enable_) {gvty_term_ = sin(actual / GEAR_RATIO);}
    if (feed_fwd_enable_) {ffwd_term_ = next_cmd;}
    if (Icntrl_) {err_int_ += err_;}

    if (Dcntrl_) { err_der_ = -(shaft_vel);} 
    err_prev_ = err_;

    // clamp err_int
    if (err_int_ > clamp_val_ && Icntrl_)
    {
        err_int_ = clamp_val_;
    } 
    else if (err_int_ < -clamp_val_ && Icntrl_)
    {
        err_int_ = -clamp_val_;
    }

    return gvty_term_ + ffwd_term_ + Kp_ * err_ + Ki_ * err_int_ + Kd_ * err_der_;
}