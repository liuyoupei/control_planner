#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include <iostream>
namespace control {
class PID {
public:
  PID();
  PID(const float& kp, const float& ki, const float& kd);
  void  reset();
  void  set_param(const float& kp, const float& ki, const float& kd, const float& max_vel_);
  void  update_pid();
  float calc_pid(const float& err);
  float p();
  float d();
  float i();
  void  update_err(const float& err);

private:
  float kp_, ki_, kd_;
  float p_, i_, d_;
  float err_, pre_err_;
  bool  init_err_;
  float max_vel_;
};

}  // namespace control

#endif