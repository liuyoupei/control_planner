#ifndef CLEAN_ROBOT_AP2P_H
#define CLEAN_ROBOT_AP2P_H
#include "p2p.h"
#include "pid.h"
#include <iostream>
#include <memory>
namespace control {
class AP2P : public P2P {
public:
  AP2P();
  ~AP2P();
  void reset();
  void setAP2PGoal(const common::Pose& goal, const common::Pose& next_goal);
  void setId(const int& id) {
    request_id_ = id;
  };
  int getId() {
    return request_id_;
  };
  bool getAp2pTwist(const common::Pose& robot_pose, common::Twist& twist);

private:
  bool  isNearGoal(const common::Pose& robot_pose);
  float getAP2PRelativeAngle(const common::Pose& current_pose, const common::Pose& relative_pose);

private:
  int                  arc_status_;
  float                arc_linear_x_;
  float                ap2p_max_vel_;
  float                near_goal_distance_;
  float                near_goal_angle_;
  common::Pose         goal_;
  common::Pose         goal_next_;
  int                  request_id_;
  std::shared_ptr<PID> ap2p_pid_;
  float                ap2p_p_;
  float                ap2p_i_;
  float                ap2p_d_;
  float                go_forward_distance_;
};
}  // namespace control
#endif  // CLEAN_ROBOT_AP2P_H
