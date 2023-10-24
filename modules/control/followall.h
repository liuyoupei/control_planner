#ifndef CLEAN_ROBOT_FOLLOWALL_H
#define CLEAN_ROBOT_FOLLOWALL_H
#include "common/enum_type.h"
#include "lp2p.h"
#include "p2p.h"
#include "pid.h"
#include <iostream>
#include <memory>
namespace control {
class Followall : public P2P {
public:
  Followall();
  ~Followall();
  common::Twist getFollowallTwsit(const common::Pose& robot_pose, const common::LaserScan& laser_scan,
                                  const common::Bumper& bumper);
  void          init();

private:
  common::Twist calculatePid();
  void          calculateData(const common::Pose& robot_pose, const common::LaserScan& laser_scan);
  double        getLaserRangeMinDistance(const common::LaserScan& laser_scan);
private:
  double                  wall_distance_;
  double                  distance_error_;
  double                  diff_error_;
  double                  max_speed_linear_x_;
  double                  min_speed_linear_x_;
  double                  max_speed_angle_z_;
  double                  P_;
  double                  D_;
  double                  angle_coef_;
  double                  angle_min_;
  double                  dist_front_;
  double                  right_min_distance_;
  int                     direction_;
  common::FollowallStatus followall_status_;
  double                  turn_angle_;
  common::ControlDir      turn_dir_;
  int                     go_back_time_;
  float                   slow_speed_angle_;
  float                   laser_min_distance_;
  LP2P                    lp2P_;
};
}  // namespace control
#endif
