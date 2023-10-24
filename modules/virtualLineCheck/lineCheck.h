#ifndef CLEAN_ROBOT_LINECHECK_H
#define CLEAN_ROBOT_LINECHECK_H
#include "common/common_type.h"
#include "memory"
#include <iostream>
#include <map>
#include <mutex>
class LineCheck {
public:
  LineCheck();
  ~LineCheck();
  void setRoomEdgeData(const std::vector<common::Edge>& edges);
  bool virtualLineCheck(const common::Pose& robot_pose, common::Edge& line_goal);
  void reset();
  void resetTrapping();
  void setFollowallMark(const common::Pose& robot_pose);
  void resetFollowall(const common::Pose& robot_pose);

private:
  void  checkAllLine(const common::Pose& robot_pose, std::vector<common::Edge>& candidate_room_lines);
  bool  chooseVirtualLine(const common::Pose& robot_pose, std::vector<common::Edge>& candidate_room_lines,common::Edge& best_goal);
  void  removeNearRobotGoal(const common::Pose& robot_pose, std::vector<common::Edge>& candidate_virtual_line);
  bool  getCounterClovkWise(const std::vector<common::Edge>& candidate_data, const common::Vector2D& base_vector2D,
                            std::map<float, common::Edge>& counter_clock_wise_line);
  bool  getClovkWise(const std::vector<common::Edge>& candidate_data, const common::Vector2D& base_vector2D,
                     std::map<float, common::Edge>& clock_wise_line, float& vector_angle);
  float getEdgeTypeAngle(const common::BlockEdgeType& type);

private:
  std::vector<common::Edge>          room_edges_;
  std::vector<common::Edge>          candidate_room_lines_;
  float                              check_line_distance_;
  float                              near_goal_distance_;
  std::tuple<bool, common::Vector2D> base_vector_;
  common::Edge                       last_line_goal_;
  bool                               is_followall_;
  common::Pose                       followall_mark_;
  float                              reset_followall_distance_;
  bool                               is_first_virtual_line_;
  common::RoomType                   room_type_;
  std::mutex                         update_room_edges_mutex_;
  bool                               bolck_start_followall_;
};
#endif  // CLEAN_ROBOT_LINECHECK_H
