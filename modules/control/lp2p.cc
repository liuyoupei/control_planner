#include "lp2p.h"
#include "common/common.h"
#include "glog/logging.h"
#include <yaml-cpp/yaml.h>
namespace control {
LP2P::LP2P() {
  YAML::Node config = YAML::LoadFile(FLAGS_config_path);
  lp2p_pid_         = std::make_shared<PID>();
  rotate_angular_ = 1.3;
  lp2p_pid_->set_param(5, 0.05, 5, rotate_angular_);
  slow_speed_min_distance_ = 0.22;
  slow_linear_x_           = 0.2;
  linear_x_                = 0.5;
  LOG_INFO << " lp2p_rotate_angular = " << rotate_angular_;
  LOG_INFO << " lp2p_slow_speed_min_distance = " << slow_speed_min_distance_;
  LOG_INFO << " lp2p_slow_linear_x = " << slow_linear_x_;
  LOG_INFO << " lp2p_linear_x = " << linear_x_;

  near_goal_distance_         = 0.05;
  lp2p_status_                = common::ControlStatus::CONTROL_INIT;
  start_turn_angle_threshold_ = 0.5;
  stop_turn_angle_threshold_  = 0.1;
}
LP2P::~LP2P() {}

void LP2P::reset() {
  LOG_INFO << "enter reset";
  lp2p_status_                = common::ControlStatus::CONTROL_INIT;
}

void LP2P::setGoalPose(const common::Pose& goal_pose) {
  goal_pose_ = goal_pose;
}

void LP2P::setNearGoalDistance(const float& disntace) {
  LOG_INFO << "enter setNearGoalDistance";
  LOG_INFO << "distance = " << disntace;
  near_goal_distance_ = disntace;
}

common::Twist LP2P::calculatePid(const float& error, const common::Pose& robot_pose,
                                 const common::LaserScan& laser_scan, const common::Bumper& bumper) {
  common::Twist twist;
  twist.angular_z = lp2p_pid_->calc_pid(error);
  twist.linear_x  = getLinearX(laser_scan, bumper, robot_pose);
  return twist;
}

int LP2P::getLp2pStatus() {
  return lp2p_status_;
}

bool LP2P::getLp2pTwsit(const common::Pose& robot_pose, const common::LaserScan& laser_scan,
                        const common::Bumper& bumper, common::Twist& twist) {
  switch (lp2p_status_) {
    case common::ControlStatus::CONTROL_INIT: {
      lp2p_status_ = common::ControlStatus::CONTROL_TURN;
    }
    case common::ControlStatus::CONTROL_PID: {
      float error = common::Common::GetInstance()->getDiffAngle(robot_pose, goal_pose_);
      twist       = calculatePid(error, robot_pose, laser_scan, bumper);
      if (fabs(error) >= start_turn_angle_threshold_) {
        lp2p_status_ = common::ControlStatus::CONTROL_TURN;
        setStartPose(robot_pose);
        return false;
      }
      if (arriveGoalDistance(robot_pose)) {
        return true;
      } else {
        return false;
      }
    }
    case common::ControlStatus::CONTROL_TURN: {
      float error = common::Common::GetInstance()->getDiffAngle(robot_pose, goal_pose_);
      if (fabs(error) <= stop_turn_angle_threshold_) {
        lp2p_status_ = common::ControlStatus::CONTROL_PID;
        twist        = stopMove();
        return false;
      }
      twist.angular_z = error > 0 ? rotate_angular_ : -rotate_angular_;
      twist.linear_x  = 0;
      return false;
    }
  }
  return false;
}

float LP2P::getLinearX(const common::LaserScan& laser_scan, const common::Bumper& bumper,
                       const common::Pose& robot_pose) {
  int   per_angle_index = laser_scan.ranges.size() / 360;
  int   min_angle       = per_angle_index * 120;
  int   max_angle       = per_angle_index * 240;
  float min_distance    = 1e4;
  for (int i = min_angle; i < max_angle; i++) {
    float distance = laser_scan.ranges[i];
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  if (true == bumper.right_bumper || true == bumper.left_bumper) {
    return 0;
  } else if (min_distance <= slow_speed_min_distance_) {
    return slow_linear_x_;
  } else {
    return linear_x_;
  }
}

bool LP2P::arriveGoalDistance(const common::Pose& robot_pose) {
  float distance = robot_pose.distanceTo(goal_pose_);
  if (distance <= near_goal_distance_) {
    return true;
  }
  return false;
}

}  // namespace control