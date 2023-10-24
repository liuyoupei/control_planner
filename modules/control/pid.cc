#include "pid.h"
namespace control {
PID::PID() {
  reset();
}
PID::PID(const float& kp, const float& ki, const float& kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PID::reset() {
  p_        = 0;
  i_        = 0;
  d_        = 0;
  err_      = 0;
  pre_err_  = 0;
  init_err_ = false;
}
void PID::set_param(const float& kp, const float& ki, const float& kd, const float& max_vel) {
  kp_      = kp;
  ki_      = ki;
  kd_      = kd;
  max_vel_ = max_vel;
}
void PID::update_pid() {
  p_ = err_;
  i_ += err_;
  d_ = err_ - pre_err_;
}
float PID::calc_pid(const float& err) {
  update_err(err);
  update_pid();
  float p   = kp_ * p_;
  float i   = ki_ * i_;
  float d   = kd_ * d_;
  float vel = p + i + d;
  if (vel >= max_vel_) {
    vel = max_vel_;
  }

  if (vel <= -max_vel_) {
    vel = -max_vel_;
  }
  return vel;
}
void PID::update_err(const float& err) {
  if (!init_err_) {
    pre_err_  = err;
    init_err_ = true;
  } else
    pre_err_ = err_;
  err_ = err;
}
float PID::p() {
  return p_;
}
float PID::d() {
  return d_;
}
float PID::i() {
  return i_;
}
}  // namespace control
