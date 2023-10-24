#ifndef CLEAN_ROBOT_NP2P_H
#define CLEAN_ROBOT_NP2P_H
#include "common/common_type.h"
#include "navigation/path_search.h"
#include "p2p.h"
#include "pid.h"
#include <iostream>
#include <memory>
namespace control {
class NP2P : public P2P {
public:
  NP2P();
  ~NP2P();
  bool getNp2pTwist(const common::Pose& robot_pose, common::Twist& twist);
  void setId(const int& id) {
    id_ = id;
  };
  void setData(const common::Np2pData& np2p_data);
  int  getId() {
    return id_;
  };
  void reset();

private:
  bool  arriveGoalDistance(const common::Pose& robot_pose, const common::Pose& goal);
  float getLinearSpeed(const common::Pose& robot_pose, const double& error_angle);

private:
  std::vector<common::Pose> path_;
  int                       np2p_status_;
  int                       np2p_avoid_status_;
  std::shared_ptr<PID>      np2p_pid_;
  float                     rotate_angular_;
  float                     near_goal_distance_;
  float                     start_turn_angle_threshold_;
  float                     stop_turn_angle_threshold_;
  common::Pose              goal_pose_;
  common::Pose              start_pose_;
  float                     robot_radius_;
  float                     slow_speed_;
  float                     max_linear_x_;
  int                       id_;
  bool                      reach_to_goal_angle_;
  common::Bumper            last_bumper_;
  int                       avoid_go_back_time_;
  int                       avoid_go_front_time_;
};
}  // namespace control
#endif  // CLEAN_ROBOT_NP2P_H
