//
// Created by lyp on 2022/11/29.
//
#include "path_search.h"
#include "glog/logging.h"
#include "sensor/sensor.h"
using namespace common;
namespace nav {
bool PathSearch::isInMap(const common::Point& point, int cols, int rows) {
  CHECK_GT(cols, 0);
  CHECK_GT(rows, 0);
  if (point.x() < 0 || point.x() >= cols || point.y() < 0 || point.y() >= rows) {
    return false;
  } else {
    return true;
  }
}
bool PathSearch::findReplacePose(const common::Pose& pose, common::Pose& replace_pose,
                                 const std::vector<std::vector<int>>& map, int cols, int rows) {
  LOG_INFO << "enter findReplacePose";
  int           x_dir[] = { 1, 0, -1, 0 };
  int           y_dir[] = { 0, 1, 0, -1 };
  common::Point start_point;
  Sensor::GetInstance()->worldToMap(pose.x(), pose.y(), start_point.x(), start_point.y());
  std::queue<common::Point> queue_bfs;
  queue_bfs.push(start_point);
  std::vector<std::vector<int>> local_map;
  std::vector<common::Pose>     temp_poses;
  local_map.clear();
  local_map      = map;
  int search_cnt = 0;
  while (!queue_bfs.empty()) {
    common::Point current_point;
    current_point = queue_bfs.front();
    queue_bfs.pop();
    search_cnt++;
    for (int i = 0; i < 4; i++) {
      common::Point temp_point;
      temp_point.x() = current_point.x() + x_dir[i];
      temp_point.y() = current_point.y() + y_dir[i];
      common::Pose mark_pose;
      Sensor::GetInstance()->mapToWorld(temp_point.x(), temp_point.y(), mark_pose.x(), mark_pose.y());
      temp_poses.push_back(mark_pose);
      if (!isInMap(temp_point, cols, rows) || local_map[temp_point.y()][temp_point.x()] == common::GRID_MAP_MARK) {
        continue;
      }
      if (local_map[temp_point.y()][temp_point.x()] == common::GRID_MAP_OBSTANCE) {
        queue_bfs.push(temp_point);
        local_map[temp_point.y()][temp_point.x()] = common::GRID_MAP_MARK;
        continue;
      }
      if (local_map[temp_point.y()][temp_point.x()] == common::GRID_MAP_FREE) {
        Sensor::GetInstance()->mapToWorld(temp_point.x(), temp_point.y(), replace_pose.x(), replace_pose.y());
        LOG_INFO << "find new goal success ";
        return true;
      }
    }
  }
  LOG_INFO << "find new goal fail search_cnt = " << search_cnt;
  return false;
}
}  // namespace nav