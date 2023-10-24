#include "lineCheck.h"
#include "common/enum_type.h"
#include "sensor/sensor.h"
#include "thrid_party/glog/logging.h"
#include <common/common.h>
LineCheck::LineCheck() {
  last_line_goal_.edge_type = common::BlockEdgeType::NULL_EDGE;
  is_followall_             = false;
  reset_followall_distance_ = 0.17;
  is_first_virtual_line_    = false;
  near_goal_distance_       = 0.1;
  check_line_distance_      = 0.2;
  bolck_start_followall_    = false;
}
LineCheck::~LineCheck() {}
void LineCheck::setFollowallMark(const common::Pose& robot_pose) {
  LOG_INFO << "enter setFollowallMark";
  is_followall_             = true;
  followall_mark_           = robot_pose;
  bolck_start_followall_    = true;
  std::get<0>(base_vector_) = false;
}
void LineCheck::resetFollowall(const common::Pose& robot_pose) {
  if (robot_pose.distanceTo(followall_mark_) >= reset_followall_distance_) {
    is_followall_             = false;
    std::get<0>(base_vector_) = false;
    last_line_goal_.reset();
    LOG_INFO << "resetFollowall";
    LOG_INFO << "is_followall_ = " << is_followall_;
  }
}
void LineCheck::reset() {
  LOG_INFO << "enter reset";
  room_edges_.clear();
  candidate_room_lines_.clear();
  std::get<0>(base_vector_) = false;
  last_line_goal_.reset();
  is_first_virtual_line_ = false;
}

void LineCheck::resetTrapping() {
  is_first_virtual_line_ = false;
  last_line_goal_.reset();
  is_followall_ = false;
}

bool LineCheck::virtualLineCheck(const common::Pose& robot_pose, common::Edge& line_goal) {
  if (is_followall_) {
    resetFollowall(robot_pose);
    return false;
  }
  candidate_room_lines_.clear();
  checkAllLine(robot_pose, candidate_room_lines_);
  if (chooseVirtualLine(robot_pose, candidate_room_lines_
                        , line_goal)) {
    is_first_virtual_line_ = true;
    if (last_line_goal_ != line_goal) {
      LOG_INFO << "last_line_goal_ != line_goal";
      common::Pose foot_pose(line_goal.foot_pose);
      foot_pose.phi() = robot_pose.phi();
      switch (line_goal.room_type) {
        case common::RoomType::ROOM_DIVISION: {
          break;
        }
      }
      last_line_goal_             = line_goal;
      std::get<1>(base_vector_).x = line_goal.end_pose.x() - line_goal.start_pose.x();
      std::get<1>(base_vector_).y = line_goal.end_pose.y() - line_goal.start_pose.y();
      std::get<1>(base_vector_).unitVector();
      std::get<0>(base_vector_) = true;
      return true;
    } else if (last_line_goal_.edge_type == common::BlockEdgeType::NULL_EDGE) {
      LOG_INFO << "last_line_goal_.edge_type == common::LineType::NULL_TYPE";
      common::Pose foot_pose(line_goal.foot_pose);
      foot_pose.phi() = robot_pose.phi();
      switch (line_goal.room_type) {
        case common::RoomType::ROOM_DIVISION:{
          break;
        }
      }
      last_line_goal_             = line_goal;
      std::get<1>(base_vector_).x = line_goal.end_pose.x() - line_goal.start_pose.x();
      std::get<1>(base_vector_).y = line_goal.end_pose.y() - line_goal.start_pose.y();
      std::get<1>(base_vector_).unitVector();
      std::get<0>(base_vector_) = true;
      return true;
    }
  }
  return false;
}
void LineCheck::setRoomEdgeData(const std::vector<common::Edge>& edges) {
  LOG_INFO << "enter setRoomEdgeData";
  LOG_INFO << " edges size = " << edges.size();
  std::unique_lock<std::mutex> guard(update_room_edges_mutex_);
  room_edges_ = edges;
  room_type_  = common::RoomType::ROOM_DIVISION;
}

float LineCheck::getEdgeTypeAngle(const common::BlockEdgeType& type) {
  switch (type) {
    case common::BlockEdgeType::UP_EDGE: {
      return common::Common::GetInstance()->DEG2RAD(90);
    }
    case common::BlockEdgeType::LEFT_EDGE: {
      return common::Common::GetInstance()->DEG2RAD(180);
    }
    case common::BlockEdgeType::DOWN_EDGE: {
      return common::Common::GetInstance()->DEG2RAD(-90);
    }
    case common::BlockEdgeType::RIGHT_EDGE: {
      return common::Common::GetInstance()->DEG2RAD(0);
    }
  }
}

void LineCheck::checkAllLine(const common::Pose& robot_pose, std::vector<common::Edge>& candidate_room_lines) {
  if (room_type_ == common::RoomType::ROOM_DIVISION
     || bolck_start_followall_ == false) {
    std::unique_lock<std::mutex> guard(update_room_edges_mutex_);
    for (auto data : room_edges_) {
      common::Pose foot_pose;
      foot_pose       = common::Common::GetInstance()->findFootOfLine(robot_pose, data.start_pose, data.end_pose);
      foot_pose.phi() = getEdgeTypeAngle(data.edge_type);
      common::Vector2D startToFootVector, endToFootVector;
      startToFootVector.x = foot_pose.x() - data.start_pose.x();
      startToFootVector.y = foot_pose.y() - data.start_pose.y();
      startToFootVector.unitVector();
      endToFootVector.x = foot_pose.x() - data.end_pose.x();
      endToFootVector.y = foot_pose.y() - data.end_pose.y();
      endToFootVector.unitVector();
      double near_goal_distance = robot_pose.distanceTo(data.start_pose);
      double distance           = robot_pose.distanceTo(foot_pose);
      if ((distance <= check_line_distance_ && startToFootVector.dot(endToFootVector) < 0)
          || (near_goal_distance <= 0.1)) {
        data.foot_pose = foot_pose;
        candidate_room_lines.push_back(data);
      }
    }
  }
}

bool LineCheck::chooseVirtualLine(const common::Pose& robot_pose, std::vector<common::Edge>& candidate_room_lines,
 common::Edge& best_goal) {
  if (candidate_room_lines.empty()) {
    return false;
  }
  common::Vector2D base_vector;
  if (std::get<0>(base_vector_) == true) {
    base_vector = std::get<1>(base_vector_);
  } else {
    base_vector.x = cos(robot_pose.phi());
    base_vector.y = sin(robot_pose.phi());
  }
  base_vector.unitVector();
  std::map<float, common::Edge> clock_wise_candidate_line;
  std::map<float, common::Edge> counter_clock_wise_candidate_line;
  removeNearRobotGoal(robot_pose, candidate_room_lines);
  float vector_angle;
  if (candidate_room_lines.size() == 1) {
    if (candidate_room_lines.size() == 1) {
      getClovkWise(candidate_room_lines, base_vector, clock_wise_candidate_line, vector_angle);
      //      LOG_INFO << "vector_angle = " << vector_angle;
      if (is_first_virtual_line_ == true && fabs(fabs(vector_angle) - 180) <= 30) {
        return false;
      }
      if (is_first_virtual_line_ == false || clock_wise_candidate_line.empty()) {
        best_goal = candidate_room_lines.front();
        return true;
      }
      return false;
    }
  }
  getCounterClovkWise(candidate_room_lines, base_vector, counter_clock_wise_candidate_line);
  getClovkWise(candidate_room_lines, base_vector, clock_wise_candidate_line, vector_angle);
  if (!counter_clock_wise_candidate_line.empty()) {
    best_goal = (--(counter_clock_wise_candidate_line.end()))->second;
    return true;
  } else if (!clock_wise_candidate_line.empty()) {
    best_goal = clock_wise_candidate_line.begin()->second;
    return true;
  } else {
    return false;
  }
}
void LineCheck::removeNearRobotGoal(const common::Pose& robot_pose, std::vector<common::Edge>& candidate_virtual_line) {
  if (candidate_virtual_line.size() >= 2) {
    for (std::vector<common::Edge>::iterator iter = candidate_virtual_line.begin();
         iter != candidate_virtual_line.end();) {
      if (robot_pose.distanceTo(iter->end_pose) <= near_goal_distance_) {
        iter = candidate_virtual_line.erase(iter);
      } else {
        iter++;
      }
    }
  }
}

bool LineCheck::getCounterClovkWise(const std::vector<common::Edge>& candidate_data,
                                    const common::Vector2D&          base_vector2D,
                                    std::map<float, common::Edge>&   counter_clock_wise_line) {
  for (auto data : candidate_data) {
    common::Vector2D line_vector;
    line_vector.x = data.end_pose.x() - data.start_pose.x();
    line_vector.y = data.end_pose.y() - data.start_pose.y();
    line_vector.unitVector();
    float vector_angle;
    float crossed_product = base_vector2D.crossedProduct(line_vector);
    vector_angle          = base_vector2D.getAngle(line_vector);
    if (crossed_product > 0) {
      counter_clock_wise_line[vector_angle] = data;
    }
  }
  if (!counter_clock_wise_line.empty()) {
    return true;
  } else {
    return false;
  }
}

bool LineCheck::getClovkWise(const std::vector<common::Edge>& candidate_data, const common::Vector2D& base_vector2D,
                             std::map<float, common::Edge>& clock_wise_line, float& vector_angle) {
  for (auto data : candidate_data) {
    common::Vector2D line_vector;
    line_vector.x = data.end_pose.x() - data.start_pose.x();
    line_vector.y = data.end_pose.y() - data.start_pose.y();
    line_vector.unitVector();
    float crossed_product = base_vector2D.crossedProduct(line_vector);
    vector_angle          = base_vector2D.getAngle(line_vector);
    //    LOG_INFO << "vector_angle = " << vector_angle;
    if (crossed_product <= 0) {
      clock_wise_line[vector_angle] = data;
    }
  }
  if (!clock_wise_line.empty()) {
    return true;
  } else {
    return false;
  }
}


