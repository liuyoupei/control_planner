#include "p2p.h"
#include "glog/logging.h"
#include <yaml-cpp/yaml.h>
namespace control {
P2P::P2P() {
  YAML::Node config = YAML::LoadFile(FLAGS_config_path);
  go_linear_v_  = 0.3;
  go_angular_z_ = 1.3;
  LOG_INFO << " p2p_go_linear_v = " << go_linear_v_;
  LOG_INFO << " p2p_go_angular_z = " << go_angular_z_;

  turn_angle_accumulation_ = 0;
}
P2P::~P2P() {}
void P2P::setStartPose(const common::Pose& pose) {
  start_pose_ = pose;
}
void P2P::setStartTime() {
  rate_timer_.tic();
}
bool P2P::goDistance(const common::Pose& pose, const float& run_distance, const common::ControlDir& dir,
                     common::Twist& twist) {
  float distance = pose.distanceTo(start_pose_);
  if (distance >= fabs(run_distance)) {
    twist.angular_z = 0;
    twist.linear_x  = 0;
    return true;
  } else {
    twist.angular_z                                      = 0;
    dir == common::ControlDir::GO_FRONT ? twist.linear_x = go_linear_v_ : twist.linear_x = -go_linear_v_;
    return false;
  }
}

bool P2P::goTime(const int& run_time, const common::ControlDir& dir, common::Twist& twist) {
  long time = run_time;
  if (rate_timer_.tac() >= time) {
    twist.angular_z = 0;
    twist.linear_x  = 0;
    return true;
  } else {
    twist.angular_z                                      = 0;
    dir == common::ControlDir::GO_FRONT ? twist.linear_x = go_linear_v_ : twist.linear_x = -go_linear_v_;
    return false;
  }
}

bool P2P::turnAngle(const common::Pose& pose, const float& run_angle, const common::ControlDir& dir,
                    common::Twist& twist) {
  float diff_angle = pose.distancePhiTo(start_pose_);
  start_pose_      = pose;
  turn_angle_accumulation_ += diff_angle;
  if (turn_angle_accumulation_ > run_angle) {
    twist.angular_z          = 0;
    twist.linear_x           = 0;
    turn_angle_accumulation_ = 0;
    return true;
  } else {
    dir == common::ControlDir::TURN_LEFT ? twist.angular_z = go_angular_z_ : twist.angular_z = -go_angular_z_;
    twist.linear_x                                                                           = 0;
    return false;
  }
}

bool P2P::turnTime(const int& run_time, const common::ControlDir& dir, common::Twist& twist) {
  long time = run_time;
  if (rate_timer_.tac() >= time) {
    twist.angular_z = 0;
    twist.linear_x  = 0;
    return true;
  } else {
    twist.angular_z                                        = 0;
    dir == common::ControlDir::TURN_LEFT ? twist.angular_z = go_angular_z_ : twist.angular_z = -go_angular_z_;
    return false;
  }
}

common::Twist P2P::stopMove() {
  common::Twist twist;
  return twist;
}

}  // namespace control
