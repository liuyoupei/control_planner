#include "cover_planner.h"
#include "Event.h"
#include "modules/sensor/sensor.h"
#include "common/common.h"
#include <yaml-cpp/yaml.h>
using namespace common;
using namespace std;
CoverPlanner::CoverPlanner() {
  YAML::Node config     = YAML::LoadFile(FLAGS_config_path);
  cover_offset_         = config["cover_radius"].as<float>();
  cover_offser_index_   = cover_offset_ / 0.05;
  near_goal_distance_   = 0.2;
  ap2p_max_angle_       = 140;
  ap2p_max_distance_    = 0.4;
  ap2p_min_distance_    = 0.2;
  map_resolution_       = 0.05;
  static_covert_offset_ = 0.25 / map_resolution_;
  polygon_              = nullptr;
  update_section_pose_history_.clear();
  LOG_INFO << " cover_radius = " << cover_offset_;
}

void CoverPlanner::init(const std::vector<common::Pose>& chains_points, const std::vector<common::Pose>& block_points,
                        const int& work_model) {
  LOG_INFO << "enter init";
  LOG_INFO << "block_points size = " << block_points.size();
  LOG_INFO << "chains_points size = " << chains_points.size();
  std::vector<common::Pose> chains_points_temp;
  chains_points_temp = chains_points;
  LOG_INFO << "chains_points_temp size = " << chains_points_temp.size();
  if (chains_points_temp.empty()) {
    return;
  }
  std::vector<common::Pose> polygon_chains_points;
  Common::GetInstance()->approxPolyDP(chains_points_temp, polygon_chains_points, 2, false);
  Sensor::GetInstance()->publishPolygon("polygon_chains_points", polygon_chains_points);
  if (polygon_) {
    LOG_INFO << "delete polygon_ ";
    delete polygon_;
    polygon_ = nullptr;
    LOG_INFO << "delete polygon_ success";
  }
  polygon_ = new Polygon(polygon_chains_points);
  intersects_section_.clear();
  last_tag_num_ = 0;
  tag_id_       = 0;
  section_id_   = 0;
}

void CoverPlanner::reset() {
  way_points_.clear();
  update_section_pose_.clear();
  update_section_pose_history_.clear();
}

std::vector<common::CoverPoint> CoverPlanner::pointToCoverPoint(std::vector<common::Pose>& inters_points) {
  std::vector<common::CoverPoint> intersects_path;
  for (int i = 0; i < inters_points.size(); i++) {
    common::CoverPoint cover_point(inters_points[i], LP2P);
    intersects_path.push_back(cover_point);
  }
  return intersects_path;
}

void CoverPlanner::setSectionId(int tag_num) {
  if (last_tag_num_ != tag_num) {
    section_id_ += (last_tag_num_ + 1);
    last_tag_num_ = tag_num;
    tag_id_++;
    LOG_INFO << "tag_num = " << tag_num;
  }
}

void CoverPlanner::getStaticCovertWithLineCoverX(const std::shared_ptr<costmap_2d::Costmap2D>& costmap,
                                                 const common::Rect<int>&                      polygon_rect) {
  last_tag_num_ = 0;
  std::vector<common::Pose> intersects_temp;
  int                       x = 0;
  for (x = polygon_rect.min_x + static_covert_offset_; x <= polygon_rect.max_x - static_covert_offset_;
       x += cover_offser_index_) {
    polygon_->intersectWithLineCoverX(costmap, x, intersects_temp);
    LOG_INFO << "intersects_temp size = " << intersects_temp.size();
    if (intersects_temp.empty()) {
      continue;
    }
    std::vector<common::CoverPoint> cover_point = pointToCoverPoint(intersects_temp);
    int                             tag_num     = intersects_temp.size() / 2;
    if (tag_num == 0) {
      continue;
    }
    setSectionId(tag_num);
    int section_id = getSenctionId();
    for (int i = 0; i < tag_num; i++) {
      intersects_section_[section_id].push_back(cover_point.at(i * 2));
      intersects_section_[section_id].push_back(cover_point.at(i * 2 + 1));
      section_id++;
    }
  }
}

void CoverPlanner::getStaticCovertWithLineCoverY(const std::shared_ptr<costmap_2d::Costmap2D>& costmap,
                                                 const common::Rect<int>&                      polygon_rect) {
  last_tag_num_ = 0;
  int y         = 0;
  for (y = polygon_rect.min_y + static_covert_offset_; y <= polygon_rect.max_y - static_covert_offset_;
       y += cover_offser_index_) {
    std::vector<common::Pose> intersects_temp;
    polygon_->intersectWithLineCoverY(costmap, y, intersects_temp);
    LOG_INFO << "intersects_temp size = " << intersects_temp.size();
    if (intersects_temp.empty()) {
      continue;
    }
    std::vector<common::CoverPoint> cover_point = pointToCoverPoint(intersects_temp);
    int                             tag_num     = intersects_temp.size() / 2;
    if (tag_num == 0) {
      continue;
    }
    setSectionId(tag_num);
    int section_id = getSenctionId();
    for (int i = 0; i < tag_num; i++) {
      intersects_section_[section_id].push_back(cover_point.at(i * 2));
      intersects_section_[section_id].push_back(cover_point.at(i * 2 + 1));
      section_id++;
    }
  }
}

bool CoverPlanner::generateStaticCoveragePlan(const std::shared_ptr<costmap_2d::Costmap2D>& costmap,
                                              const std::vector<common::Pose>&              chains_points,
                                              const std::vector<common::Pose>&              block_points,
                                              const common::Pose& robot_pose, const event::Type& work_model) {
  init(chains_points, block_points, work_model);
  if (polygon_ == nullptr) {
    return false;
  }
  common::Rect<int> polygon_rect;
  LOG_INFO << "enter generateStaticCoveragePlan";
  polygon_->getRectFromPolygon(polygon_rect);
  getCoverInfo(polygon_rect);
  switch (cover_dir_) {
    case common::CoverDirType::X_DIR: {
      getStaticCovertWithLineCoverX(costmap, polygon_rect);
      break;
    }
    case common::CoverDirType::Y_DIR: {
      getStaticCovertWithLineCoverY(costmap, polygon_rect);
      break;
    }
  }
  section_id_ += last_tag_num_;
  way_points_ = getBestOrder(robot_pose);
  getCoverPointType(way_points_);
  judgmentTypeValidity(way_points_);
  std::vector<common::Pose> cover_path;
  for (auto data : way_points_) {
    cover_path.push_back(data.pose);
  }
  if (way_points_.empty()) {
    return false;
  } else {
    return true;
  }
}

bool CoverPlanner::getNearSectionFromComparePose(const common::Pose&                    compare_pose,
                                                 const std::vector<common::CoverPoint>& intersects_section, int& id,
                                                 float& distance) {
  LOG_INFO << "enter getNearSectionFromComparePose intersects_section size = " << intersects_section.size();
  if (intersects_section.empty()) {
    LOG_INFO << "intersects_section empty";
    return false;
  }
  std::map<float, int> distance_map;
  int                  near_id = 0;
  common::Pose         first_line_start_pose;
  common::Pose         first_line_end_pose;
  first_line_start_pose                                        = intersects_section[0].pose;
  distance_map[first_line_start_pose.distanceTo(compare_pose)] = near_id;
  near_id++;
  first_line_end_pose                                        = intersects_section[1].pose;
  distance_map[first_line_end_pose.distanceTo(compare_pose)] = near_id;
  near_id++;
  if (intersects_section.size() > 2) {
    common::Pose end_line_start_pose;
    common::Pose end_line_end_pose;
    end_line_start_pose                                        = intersects_section[intersects_section.size() - 2].pose;
    distance_map[end_line_start_pose.distanceTo(compare_pose)] = near_id;
    near_id++;
    end_line_end_pose                                        = intersects_section[intersects_section.size() - 1].pose;
    distance_map[end_line_end_pose.distanceTo(compare_pose)] = near_id;
  }
  id       = distance_map.begin()->second;
  distance = distance_map.begin()->first;
  LOG_INFO << "id = " << id << " distance = " << distance;
  return true;
}

void CoverPlanner::changeOrder(int id, std::vector<common::CoverPoint>& intersects_section) {
  LOG_INFO << "enter changeOrder id = " << id;
  switch (id) {
    case FIRST_LINE_START: {
      break;
    }
    case FIRST_LINE_END: {
      for (int i = 0; i < intersects_section.size() / 2; i++) {
        int                index = i * 2;
        common::CoverPoint temp_point;
        temp_point                       = intersects_section.at(index);
        intersects_section.at(index)     = intersects_section.at(index + 1);
        intersects_section.at(index + 1) = temp_point;
      }
      break;
    }
    case END_LINE_START: {
      std::reverse(intersects_section.begin(), intersects_section.end());
      for (int i = 0; i < intersects_section.size() / 2; i++) {
        int                index = i * 2;
        common::CoverPoint temp_point;
        temp_point                       = intersects_section.at(index);
        intersects_section.at(index)     = intersects_section.at(index + 1);
        intersects_section.at(index + 1) = temp_point;
      }
      break;
    }
    case END_LINE_END: {
      std::reverse(intersects_section.begin(), intersects_section.end());
      break;
    }
  }
  intersects_section.front().type = NP2P;
}

int CoverPlanner::getSenctionId() {
  return section_id_;
}

bool CoverPlanner::getNextWayPoint(const common::Pose& robot_pose, common::CoverPoint& way_point) {
  LOG_INFO << "enter getNextWayPoint way_points_ size= " << way_points_.size();
  if (way_points_.empty()) {
    return false;
  } else {
    way_point = way_points_.front();
    Vector2D robot_vector, robot_to_goal;
    robot_vector.x = cos(robot_pose.phi());
    robot_vector.y = sin(robot_pose.phi());
    robot_vector.unitVector();
    robot_to_goal.x = way_point.pose.x() - robot_pose.x();
    robot_to_goal.y = way_point.pose.y() - robot_pose.y();
    robot_to_goal.unitVector();
    if (way_point.type == AP2P
        && (fabs(robot_vector.getAngle(robot_to_goal)) >= ap2p_max_angle_
            || robot_pose.distanceTo(way_point.pose) >= ap2p_max_distance_)) {
      way_point.type                = NP2P;
      way_point.near_goal_distance_ = near_goal_distance_ / 2.0;
    }
    return true;
  }
}

bool CoverPlanner::getNextNextWayPoint(common::CoverPoint& way_point) {
  if (way_points_.size() <= 1) {
    return false;
  } else {
    way_point = way_points_[1];
    return true;
  }
}

void CoverPlanner::getWayPoints(std::vector<common::CoverPoint>& inters_points) {
  LOG_INFO << "enter getWayPoints inters_points size = " << inters_points.size();
  for (int i = 1; i < inters_points.size() / 2; i++) {
    if (i % 2 != 0) {
      int                index = i * 2;
      common::CoverPoint temp_point;
      temp_point                  = inters_points.at(index);
      inters_points.at(index)     = inters_points.at(index + 1);
      inters_points.at(index + 1) = temp_point;
    }
  }
}

std::vector<common::CoverPoint> CoverPlanner::getBestOrder(const common::Pose& robot_pose) {
  LOG_INFO << "enter getBestOrder";
  std::vector<common::CoverPoint> way_points;
  common::Pose                    compare_pose;
  compare_pose = robot_pose;
  std::map<int, std::vector<common::CoverPoint>>::iterator iterator_erase;
  std::map<int, std::vector<common::CoverPoint>>           intersects_section;
  intersects_section = intersects_section_;
  for (int i = 0; i < intersects_section.size(); i++) {
    float min_distance = 1e6;
    int   near_id;
    for (std::map<int, std::vector<common::CoverPoint>>::iterator iter = intersects_section.begin();
         iter != intersects_section.end(); iter++) {
      int                             id;
      std::vector<common::CoverPoint> cover_points;
      float                           distance;
      getNearSectionFromComparePose(compare_pose, iter->second, id, distance);
      if (min_distance > distance) {
        min_distance   = distance;
        near_id        = id;
        iterator_erase = iter;
      }
    }
    LOG_INFO << "iterator_erase size = " << iterator_erase->second.size();
    changeOrder(near_id, iterator_erase->second);
    getWayPoints(iterator_erase->second);
    compare_pose = iterator_erase->second[iterator_erase->second.size() - 1].pose;
    LOG_INFO << "compare_pose x = " << compare_pose.x() << " y = " << compare_pose.y();
    way_points.insert(way_points.end(), iterator_erase->second.begin(), iterator_erase->second.end());
    intersects_section.erase(iterator_erase);
    i -= 1;
  }
  // getWayPoints(way_points);
  LOG_INFO << "way point size = " << way_points.size();
  return way_points;
}

void CoverPlanner::getCoverPointType(std::vector<common::CoverPoint>& intersects) {
  LOG_INFO << "enter getCoverPointType size = " << intersects.size();
  for (int i = 1; i < intersects.size(); i++) {
    if (intersects[i].type == NP2P) {
      continue;
    }
    intersects[i - 1].near_goal_distance_ = near_goal_distance_;
    intersects[i].near_goal_distance_     = near_goal_distance_;
    if (intersects[i - 1].type != LP2P) {
      intersects[i].type = LP2P;
    } else if (intersects[i - 1].type == LP2P) {
      intersects[i].type = AP2P;
    }
  }
}

void CoverPlanner::judgmentTypeValidity(std::vector<common::CoverPoint>& intersects) {
  if (intersects.size() <= 2) {
    return;
  }
  for (int i = 1; i < intersects.size() - 2; i++) {
    intersects[i - 1].near_goal_distance_ = near_goal_distance_;
    intersects[i].near_goal_distance_     = near_goal_distance_;
    float            distance             = intersects[i].pose.distanceTo(intersects[i - 1].pose);
    float            next_distance        = intersects[i].pose.distanceTo(intersects[i + 1].pose);
    common::Vector2D lastTOCurrentPointVector, currentToNextPointVector;
    lastTOCurrentPointVector.x = intersects[i].pose.x() - intersects[i - 1].pose.x();
    lastTOCurrentPointVector.y = intersects[i].pose.y() - intersects[i - 1].pose.y();
    lastTOCurrentPointVector.unitVector();
    currentToNextPointVector.x = intersects[i + 1].pose.x() - intersects[i].pose.x();
    currentToNextPointVector.y = intersects[i + 1].pose.y() - intersects[i].pose.y();
    currentToNextPointVector.unitVector();
    float angle = lastTOCurrentPointVector.getAngle(currentToNextPointVector);
    if (intersects[i].type == AP2P && (angle <= 40 || angle >= 130) && distance > ap2p_max_distance_
        && distance >= ap2p_max_distance_) {
      intersects[i].type = NP2P;
    } else if (intersects[i].type == AP2P && next_distance <= ap2p_min_distance_) {
      intersects[i].type = NP2P;
    }
  }
}

void CoverPlanner::getCoverInfo(common::Rect<int>& polygon_rect) {
  int length = polygon_rect.getLength();
  int width  = polygon_rect.getWidth();
  if (length > width) {
    cover_dir_ = common::CoverDirType::X_DIR;
  } else {
    cover_dir_ = common::CoverDirType::Y_DIR;
  }
  LOG_INFO << "cover_dir = " << cover_dir_;
}

void CoverPlanner::updateSection(const common::Pose& pose) {
  LOG_INFO << "enter updateSection update_section_pose_";
  LOG_INFO << "pose x = " << pose.x() << " y = " << pose.y();
  way_points_.erase(way_points_.begin());
  update_section_pose_.push_back(pose);
  LOG_INFO << "update_section_pose_ size = " << update_section_pose_.size();
  if (update_section_pose_.size() >= 2) {
    for (auto pose : update_section_pose_) {
      LOG_INFO << "pose x = " << pose.x() << " y = " << pose.y();
      for (std::map<int, std::vector<common::CoverPoint>>::iterator iter = intersects_section_.begin();
           iter != intersects_section_.end();) {
        for (std::vector<common::CoverPoint>::iterator point_iter = iter->second.begin();
             point_iter != iter->second.end();) {
          if (point_iter->pose == pose) {
            LOG_INFO << "erase section point x = " << point_iter->pose.x() << " y = " << point_iter->pose.y();
            point_iter = iter->second.erase(point_iter);
          } else {
            point_iter++;
          }
        }
        if (iter->second.empty()) {
          iter = intersects_section_.erase(iter);
          LOG_INFO << "erase section";
        } else {
          iter++;
        }
      }
    }
    update_section_pose_history_.insert(update_section_pose_history_.end(), update_section_pose_.begin(),
                                        update_section_pose_.end());
    update_section_pose_.clear();
  }
}

bool CoverPlanner::isCoverFinish() {
  if (way_points_.empty()) {
    return true;
  } else {
    return false;
  }
}
