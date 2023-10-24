#include "followall.h"
#include "common/common.h"
#include "sensor/sensor.h"
#include "thrid_party/glog/logging.h"
#include <yaml-cpp/yaml.h>
namespace control {
Followall::Followall() {
  YAML::Node config = YAML::LoadFile(FLAGS_config_path);
  wall_distance_    = config["fw_distance"].as<float>();
  P_                  = 10;
  D_                  = 5;
  max_speed_linear_x_ = 0.4;
  min_speed_linear_x_ = 0.25;
  max_speed_angle_z_  = 1.5;
  slow_speed_angle_   = 1.75;
  go_back_time_       = 200;
  angle_coef_         = 1;
  laser_min_distance_ = 0.165;
  LOG_INFO << " fw_p = " << P_;
  LOG_INFO << " fw_p = " << D_;
  LOG_INFO << " fw_max_speed_linear_x = " << max_speed_linear_x_;
  LOG_INFO << " fw_min_speed_linear_x = " << min_speed_linear_x_;
  LOG_INFO << " fw_slow_speed_angle = " << slow_speed_angle_;
  LOG_INFO << " fw_go_back_time = " << go_back_time_;
  direction_        = -1;
  distance_error_   = 0;
  angle_min_        = 0;
  followall_status_ = common::FOllOWALL_FIND_WALL;
}

Followall::~Followall() {}

void Followall::init() {
  followall_status_ = common::FOllOWALL_FIND_WALL;
}

common::Twist Followall::calculatePid() {
  common::Twist twist;
  double        angle_error = angle_min_ - M_PI * direction_ / 2;
  twist.angular_z           = direction_ * (P_ * distance_error_ + D_ * diff_error_) + angle_coef_ * angle_error;
  if (dist_front_ < wall_distance_ * 1.5) {
    twist.linear_x = min_speed_linear_x_;
  } else if (fabs(angle_error) > 0.5) {
    twist.linear_x = min_speed_linear_x_;
  } else if (fabs(distance_error_) > 0.01) {
    twist.linear_x = min_speed_linear_x_;
  } else {
    twist.linear_x = max_speed_linear_x_;
  }

  if (twist.angular_z > max_speed_angle_z_) {
    twist.angular_z = max_speed_angle_z_;
  }
  if (twist.angular_z < -max_speed_angle_z_) {
    twist.angular_z = -max_speed_angle_z_;
  }
  /*  LOG_INFO << "dist_front_ = " << dist_front_ << " angle_error = " << angle_error
             << " distance_error_ = " << distance_error_;*/
  return twist;
}

double Followall::getLaserRangeMinDistance(const common::LaserScan& laser_scan) {
  int size     = laser_scan.ranges.size();
  int minIndex = size / 4;
  int maxIndex = size * 3 / 4;
  for (int i = minIndex; i < maxIndex; i++) {
    if (laser_scan.ranges[i] > 0.01 && laser_scan.ranges[i] < laser_scan.ranges[minIndex]) {
      minIndex = i;
    }
  }
  double min_distance = laser_scan.ranges[minIndex];
  return min_distance;
}

void Followall::calculateData(const common::Pose& robot_pose, const common::LaserScan& laser_scan) {
  int size     = laser_scan.ranges.size();
  int minIndex = size * (direction_ + 1) / 4;
  int maxIndex = size * (direction_ + 3) / 4;
  for (int i = minIndex; i < maxIndex; i++) {
    if (laser_scan.ranges[i] > 0.01 && laser_scan.ranges[i] < laser_scan.ranges[minIndex]) {
      minIndex = i;
    }
  }
  angle_min_ = (minIndex - size / 2) * laser_scan.angle_increment;

  right_min_distance_ = laser_scan.ranges[minIndex];
  dist_front_         = laser_scan.ranges[size / 2];
  diff_error_         = (right_min_distance_ - wall_distance_) - distance_error_;
  distance_error_     = right_min_distance_ - wall_distance_;
}

common::Twist Followall::getFollowallTwsit(const common::Pose& robot_pose, const common::LaserScan& laser_scan,
                                           const common::Bumper& bumper) {
  common::Twist twist;
  common::Pose  goal_pose;
  double        diff_angle;
  calculateData(robot_pose, laser_scan);
  switch (followall_status_) {
    case common::FOllOWALL_FIND_WALL: {
      followall_status_ = common::FOLLOWALL_PID;
      return twist;
    }
    case common::FOLLOWALL_PID: {
      if (dist_front_ <= wall_distance_) {
        turn_angle_ = M_PI_2;
        turn_dir_   = common::ControlDir::TURN_LEFT;
        setStartPose(robot_pose);
        followall_status_ = common::FOLLOWALL_TURN_ANGLE;
        return twist;
      } else if (bumper.left_bumper || bumper.right_bumper) {
        followall_status_ = common::FOLLOWALL_GO_BACK;
        setStartTime();
        return twist;
      } else if (getLaserRangeMinDistance(laser_scan) <= laser_min_distance_) {
        turn_angle_ = M_PI_4;
        turn_dir_   = common::ControlDir::TURN_LEFT;
        setStartPose(robot_pose);
        followall_status_ = common::FOLLOWALL_TURN_ANGLE;
        return twist;
      }
      return calculatePid();
    }
    case common::FOLLOWALL_GO_BACK: {
      if (goTime(go_back_time_, common::ControlDir::GO_BACK, twist)) {
        followall_status_ = common::FOLLOWALL_TURN_ANGLE;
        turn_angle_ = M_PI_4;
        turn_dir_   = common::ControlDir::TURN_LEFT;
        setStartPose(robot_pose);
        return twist;
      } else {
        return twist;
      }
    }
    case common::FOLLOWALL_TURN_ANGLE: {
      if (turnAngle(robot_pose, turn_angle_, turn_dir_, twist)) {
        followall_status_ = common::FOLLOWALL_PID;
        return twist;
      } else {
        return twist;
      }
    }
    case common::FOllOWALL_LINE_GOAL: {
      LOG_INFO << "right_min_distance_ = " << right_min_distance_;
      if (lp2P_.getLp2pTwsit(robot_pose, laser_scan, bumper, twist) || bumper.left_bumper || bumper.right_bumper) {
        followall_status_ = common::FOLLOWALL_PID;
        return twist;
      }
      return twist;
    }
  }
  return twist;
}
}  // namespace control
