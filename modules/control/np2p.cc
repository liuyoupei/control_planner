#include "np2p.h"
#include "common/common.h"
#include "common/common_type.h"
#include "thrid_party/glog/logging.h"
#include <assert.h>
#include <yaml-cpp/yaml.h>
namespace control {
NP2P::NP2P() {
  YAML::Node config = YAML::LoadFile(FLAGS_config_path);
  np2p_pid_         = std::make_shared<PID>();
  rotate_angular_ = 1.3;
  np2p_pid_->set_param(3, 0.05, 5, rotate_angular_);
  slow_speed_          = 0.2;
  max_linear_x_        = 0.4;
  avoid_go_back_time_  = 200;
  avoid_go_front_time_ = 200;
  LOG_INFO << " np2p_rotate_angular = " << rotate_angular_;
  LOG_INFO << " np2p_slow_speed = " << slow_speed_;
  LOG_INFO << " np2p_max_linear_x = " << max_linear_x_;
  LOG_INFO << " np2p_avoid_go_back_time = " << avoid_go_back_time_;
  LOG_INFO << " np2p_avoid_go_front_time = " << avoid_go_front_time_;

  near_goal_distance_         = 0.10;
  start_turn_angle_threshold_ = 1.5;
  stop_turn_angle_threshold_  = 0.1;
  robot_radius_               = 0.175;
  np2p_status_                = common::ControlStatus::CONTROL_INIT;
  reach_to_goal_angle_        = true;
}
NP2P::~NP2P() {}

void NP2P::reset() {
  np2p_status_       = common::ControlStatus::CONTROL_INIT;
  np2p_avoid_status_ = common::ControlStatus::CONTROL_INIT;
}

void NP2P::setData(const common::Np2pData& np2p_data) {
  LOG_INFO << "senter setData";
  LOG_INFO << "path size = " << np2p_data.path.size();
  LOG_INFO << "reach_to_goal_angle_ = " << reach_to_goal_angle_;
  assert(!np2p_data.path.empty());
  path_.clear();
  path_                = np2p_data.path;
  start_pose_          = path_.front();
  goal_pose_           = path_[path_.size() - 1];
  reach_to_goal_angle_ = np2p_data.reach_to_goal_angle;
  LOG_INFO << "path size = " << path_.size();
}

bool NP2P::arriveGoalDistance(const common::Pose& robot_pose, const common::Pose& goal) {
  float distance = robot_pose.distanceTo(goal);
  if (distance <= near_goal_distance_) {
    return true;
  } else {
    return false;
  }
}

float NP2P::getLinearSpeed(const common::Pose& robot_pose, const double& error_angle) {
  float robot_pose_to_start_pose_distance = start_pose_.distanceTo(robot_pose);
  float robot_pose_to_goal_pose_distance  = goal_pose_.distanceTo(robot_pose);
  if (robot_pose_to_goal_pose_distance < robot_radius_ || robot_pose_to_start_pose_distance < robot_radius_
      || fabs(error_angle) >= 1.0) {
    return slow_speed_;
  } else {
    return max_linear_x_;
  }
}

bool NP2P::getNp2pTwist(const common::Pose& robot_pose, common::Twist& twist) {
  switch (np2p_status_) {
    case common::ControlStatus::CONTROL_INIT: {
      LOG_INFO << "np2p init";
      LOG_INFO << "path_ size = " << path_.size();
      np2p_status_ = common::ControlStatus::CONTROL_PID;
      break;
    }
    case common::ControlStatus::CONTROL_PID: {
      common::Pose goal  = path_.front();
      float        error = common::Common::GetInstance()->getDiffAngle(robot_pose, goal);
      if (arriveGoalDistance(robot_pose, goal) == true) {
        if (path_.empty()) {
          if (true == reach_to_goal_angle_) {
            np2p_status_ = common::ControlStatus::CONTROL_TURN_GOAL;
            return false;
          } else {
            return true;
          }
        } else {
          if(path_.size() >= 4){
            path_.erase(path_.begin(), path_.begin() + 3);
          }else{
            path_.erase(path_.begin());
          }
        }
      }
      if (fabs(error) >= start_turn_angle_threshold_) {
        np2p_status_ = common::ControlStatus::CONTROL_TURN;
        twist        = stopMove();
      } else {
        twist.angular_z = np2p_pid_->calc_pid(error);
        twist.linear_x  = getLinearSpeed(robot_pose, error);
      }
      break;
    }
    case common::ControlStatus::CONTROL_TURN: {
      common::Pose goal  = path_.front();
      float        error = common::Common::GetInstance()->getDiffAngle(robot_pose, goal);
      if (fabs(error) <= stop_turn_angle_threshold_) {
        np2p_status_ = common::ControlStatus::CONTROL_PID;
        twist        = stopMove();
      }
      twist.angular_z = error > 0 ? rotate_angular_ : -rotate_angular_;
      twist.linear_x  = 0;
      break;
    }
    case common::ControlStatus::CONTROL_TURN_GOAL: {
      //      LOG_INFO << "CONTROL_TURN_GOAL";
      float error = robot_pose.distancePhiTo(goal_pose_);
      if (fabs(error) <= stop_turn_angle_threshold_) {
        twist = stopMove();
        return true;
      }
      twist.angular_z = error > 0 ? rotate_angular_ : -rotate_angular_;
      twist.linear_x  = 0;
      break;
    }
  }
  return false;
}


}  // namespace control