#ifndef CLEAN_ROBOT_P2P_H
#define CLEAN_ROBOT_P2P_H
#include "RateTimer.h"
#include "common/common_type.h"
#include "common/enum_type.h"
#include <gflags/gflags_declare.h>
#include <iostream>
DECLARE_string(config_path);
namespace control {
class P2P {
public:
  virtual ~P2P();
  P2P();
  void setStartPose(const common::Pose& pose);
  void setStartTime();
  bool goDistance(const common::Pose& pose, const float& run_distance, const common::ControlDir& dir,
                  common::Twist& twist);
  bool goTime(const int& run_time, const common::ControlDir& dir, common::Twist& twist);
  bool turnAngle(const common::Pose& pose, const float& run_angle, const common::ControlDir& dir, common::Twist& twist);
  bool turnTime(const int& run_time, const common::ControlDir& dir, common::Twist& twist);
  common::Twist stopMove();
  void          setId(const int& id) {
    id_ = id;
  }
  int getId() {
    return id_;
  };

private:
  RateTimer    rate_timer_;
  common::Pose start_pose_;
  float        go_linear_v_;
  float        go_angular_z_;
  float        turn_angle_accumulation_;
  int          remote_control_status_;
  int          id_;
};
}  // namespace control
#endif  // CLEAN_ROBOT_P2P_H
