#ifndef POS_CONTROLLER_HPP
#define POS_CONTROLLER_HPP

#include <Arduino.h>

/// \brief Abstracted position controller for motor control
class PositionController
{
public:
  /// \brief Initializer for a P controller using ffwd
  /// \param Kp - The proportional gain for a P controller
  PositionController(double Kp);

  /// \brief Initializer for a PI controller using ffwd
  /// \param Kp - The proportional gain for a PI controller
  /// \param Ki - The integral gain for a PI controller
  PositionController(double Kp, double Ki);

  /// \brief Initializer for a PID controller using ffwd
  /// \param Kp - The proportional gain for a PID controller
  /// \param Ki - The integral gain for a PID controller
  /// \param Ki - The derivative gain for a PID controller
  PositionController(double Kp, double Ki, double Kd);

  /// \brief Pump controller
  /// \param setpoint - The set point position in radians
  /// \param actual - The actual motor shaft position in radians
  /// \param shaft_vel - The motor shaft velocity used in derivative control
  double pump_controller(double setpoint, double actual, float shaft_vel);

  /// \brief Set the status of feed forward control in the class
  /// \param enable - Sets if feed forward term is used in control
  void set_ffwd_control(bool enable);

  /// \brief Set the value of the integral control error clamp
  /// \param clamp_val - The value that the integral error is clamped at
  void set_clamp_val(double clamp_val);

private:
  /// \brief P gain
  double Kp_;
  /// \brief I gain
  double Ki_;
  /// \brief D gain
  double Kd_;
  /// \brief Boolean enabling integral control
  bool Icntrl_;
  /// \brief Boolean enabling derivative control
  bool Dcntrl_;
  /// \brief Error
  double err_;
  /// \brief Integrated error
  double err_int_;
  /// \brief Derivative error
  double err_der_;
  /// \brief Previous error
  double err_prev_;
  /// \brief Integral error clamping value
  double clamp_val_;
  /// \brief Boolean enabling feed forward control
  bool feed_fwd_enable_;
  /// \brief Feed forward compensation term
  double ffwd_term_;
};

#endif
