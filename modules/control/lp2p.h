#ifndef CLEAN_ROBOT_LP2P_H
#define CLEAN_ROBOT_LP2P_H
#include "p2p.h"
#include "pid.h"
#include <iostream>
#include <memory>
namespace control {
class LP2P : public P2P {
public:
  LP2P();
  ~LP2P();
  bool getLp2pTwsit(const common::Pose& robot_pose, const common::LaserScan& laser_scan, const common::Bumper& bumper,
                    common::Twist& twist);
  void setGoalPose(const common::Pose& goal_pose);
  void setNearGoalDistance(const float& disntace);
  void setId(const int& id) {
    id_ = id;
  };
  int getId() {
    return id_;
  };
  int getLp2pStatus();
  void reset();
private:
  common::Twist calculatePid(const float& error, const common::Pose& robot_pose, const common::LaserScan& laser_scan,
                             const common::Bumper& bumper);
  float getLinearX(const common::LaserScan& laser_scan, const common::Bumper& bumper, const common::Pose& robot_pose);
  bool  arriveGoalDistance(const common::Pose& robot_pose);

private:
  std::shared_ptr<PID> lp2p_pid_;
  common::Pose         goal_pose_;
  float                slow_speed_min_distance_;
  float                slow_linear_x_;
  float                linear_x_;
  float                near_goal_distance_;
  int                  lp2p_status_;
  float                start_turn_angle_threshold_;
  float                stop_turn_angle_threshold_;
  float                rotate_angular_;
  int                  id_;
};
}  // namespace control
#endif  // CLEAN_ROBOT_LP2P_H
