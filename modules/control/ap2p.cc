#include "ap2p.h"
#include "common/common.h"
#include "common/enum_type.h"
#include "thrid_party/glog/logging.h"
#include <yaml-cpp/yaml.h>
namespace control {
AP2P::AP2P() {
  YAML::Node config   = YAML::LoadFile(FLAGS_config_path);
  ap2p_pid_           = std::make_shared<PID>();
  near_goal_distance_ = 0.1;
  near_goal_angle_    = 0.3;
  ap2p_p_       = 2.0;
  ap2p_i_       = 0;
  ap2p_d_       = 0.08;
  arc_linear_x_ = 0.15;
  ap2p_max_vel_ = 2.4;
  LOG_INFO << " ap2p_p_ = " << ap2p_p_;
  LOG_INFO << " ap2p_i_ = " << ap2p_i_;
  LOG_INFO << " ap2p_d_ = " << ap2p_d_;
  LOG_INFO << " arc_linear_x_ = " << arc_linear_x_;
  LOG_INFO << " ap2p_max_vel_ = " << ap2p_max_vel_;

  arc_status_          = 0;
  go_forward_distance_ = 0.2;
  ap2p_pid_->set_param(ap2p_p_, ap2p_i_, ap2p_d_, ap2p_max_vel_);
}

AP2P::~AP2P() {}

void AP2P::reset() {
  arc_status_ = common::ControlStatus::CONTROL_INIT;
}

void AP2P::setAP2PGoal(const common::Pose& goal, const common::Pose& next_goal) {
  goal_      = goal;
  goal_next_ = next_goal;
}

bool AP2P::isNearGoal(const common::Pose& robot_pose) {
  float diff_angle = getAP2PRelativeAngle(robot_pose, goal_next_);
  if (robot_pose.distanceTo(goal_next_) <= near_goal_distance_ || fabs(diff_angle) <= near_goal_angle_) {
    LOG_INFO << "AP2P:"
             << "reach goal"
             << "distance " << robot_pose.distanceTo(goal_next_) << ", " <<", diff_angle: " << fabs(diff_angle);
    return true;
  } else {
    return false;
  }
}

float AP2P::getAP2PRelativeAngle(const common::Pose& current_pose, const common::Pose& relative_pose) {
  float diff_x         = 0;
  float diff_y         = 0;
  float relative_Angle = 0;
  diff_x               = relative_pose.x() - current_pose.x();
  diff_y               = relative_pose.y() - current_pose.y();
  if (diff_x < -go_forward_distance_) {
    diff_x = -go_forward_distance_;
  }
  if (diff_y < -go_forward_distance_) {
    diff_y = -go_forward_distance_;
  }
  if (diff_x > go_forward_distance_) {
    diff_x = go_forward_distance_;
  }
  if (diff_y > go_forward_distance_) {
    diff_y = go_forward_distance_;
  }
  if (diff_x == 0 || diff_y == 0) {
    return 0;
  } else {
    float angle;
    angle          = atan2(diff_y, diff_x) - (current_pose.phi());
    relative_Angle = common::Common::GetInstance()->fpwrappi(angle);
    return relative_Angle;
  }
}

bool AP2P::getAp2pTwist(const common::Pose& robot_pose, common::Twist& twist) {
  switch (arc_status_) {
    case common::ControlStatus::CONTROL_INIT: {
      arc_status_ = common::ControlStatus::CONTROL_PID;
    }
    case common::ControlStatus::CONTROL_PID: {
      float diff_angle = getAP2PRelativeAngle(robot_pose, goal_next_);
      if (isNearGoal(robot_pose) == true) {
        return true;
      }
      twist.angular_z = ap2p_pid_->calc_pid(diff_angle);
      twist.linear_x  = arc_linear_x_;
      return false;
    }
  }
}
}  // namespace control