#include "jps_search.h"
#include "common/common.h"
#include "map/cost_values.h"
#include "sensor/sensor.h"
#include "thrid_party/glog/logging.h"
#include <common/enum_type.h>
namespace nav {

JpsSearch::JpsSearch() {
  cols_           = 0;
  rows_           = 0;
  inflate_radius_ = 0.3;
}
JpsSearch::~JpsSearch() {}
JpsNode* JpsSearch::findPath(const JpsNode& start_node, const JpsNode& end_node) {
  // create a new node with start point
  JpsNode* firstpt = new JpsNode(common::Point(start_node.pt.x(), start_node.pt.y()));
  openlist_.emplace_back(firstpt);
  while (!openlist_.empty()) {
    // find least F point, set it the current point
    auto cur_point = getLeastFpoint();

    // remove current point from openlist, add it to close list
    openlist_.remove(cur_point);
    closelist_.emplace_front(cur_point);

    // F=G+H
    int           d, f, g, h;
    common::Point jp;

    // get neighbour points
    std::vector<common::Point> nbps = getNeighbourPoints(cur_point);

    // find jump point in the neighbour point
    for (int i = 0; i < nbps.size(); i++) {
      jp = checkJumpPoint(nbps[i], cur_point->pt);
      // LOG_INFO<<"lyp:" << "jp:" << jp.x << "," << jp.y << endl;
      // find jump point
      if (jp.x() != -1) {
        JpsNode* now = new JpsNode(jp);
        // if jump point in closelist, continue to next
        if (isInList(closelist_, now))
          continue;

        // calc distance of jump point and current point
        d = abs(jp.x() - cur_point->pt.x()) + abs(jp.y() - cur_point->pt.y());
        // calc G-value of jump point
        g = cur_point->G + d;
        // calc H-value of jump point
        h = abs(jp.x() - finalpoint_.x()) + abs(jp.y() - finalpoint_.y());

        // if jump point is in openlist
        JpsNode* hasnow = isInList(openlist_, now);
        if (hasnow) {
          hasnow->G      = g;
          hasnow->H      = h;
          hasnow->F      = g + h;
          hasnow->parent = cur_point;
        } else {  // not in openlist, add it
          now->G      = g;
          now->H      = h;
          now->F      = g + h;
          now->parent = cur_point;
          openlist_.emplace_front(now);
        }

        // if end_node in the open list, return
        JpsNode* res_point = isInList(openlist_, &end_node);
        if (res_point)
          return res_point;
      }
    }
  }
  return NULL;
}

std::vector<common::Point> JpsSearch::getNeighbourPoints(JpsNode* point) const {
  std::vector<common::Point> nbps;
  common::Point              dir;
  if (point->parent == NULL) {
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {
        if (x == 0 && y == 0)
          continue;
        int dx = point->pt.x() + x;
        int dy = point->pt.y() + y;
        if (!isUnreachable(dx, dy)) {
          nbps.emplace_back(common::Point(dx, dy));
        }
      }
    }
  } else {
    dir = point->getDirection();
    if (dir.x() != 0 && dir.y() != 0) {
      if (!isUnreachable(point->pt.x(), point->pt.y() + dir.y()))
        nbps.emplace_back(common::Point(point->pt.x(), point->pt.y() + dir.y()));
      if (!isUnreachable(point->pt.x() + dir.x(), point->pt.y()))
        nbps.emplace_back(common::Point(point->pt.x() + dir.x(), point->pt.y()));
      if (!isUnreachable(point->pt.x() + dir.x(), point->pt.y() + dir.y()))
        nbps.emplace_back(common::Point(point->pt.x() + dir.x(), point->pt.y() + dir.y()));
      if (isUnreachable(point->pt.x() - dir.x(), point->pt.y())
          && !isUnreachable(point->pt.x() - dir.x(), point->pt.y() + dir.y()))
        nbps.emplace_back(common::Point(point->pt.x() - dir.x(), point->pt.y() + dir.y()));
      if (isUnreachable(point->pt.x(), point->pt.y() - dir.y())
          && !isUnreachable(point->pt.x() + dir.x(), point->pt.y() - dir.y()))
        nbps.emplace_back(common::Point(point->pt.x() + dir.x(), point->pt.y() - dir.y()));
    } else {
      if (dir.x() != 0) {
        if (!isUnreachable(point->pt.x() + dir.x(), point->pt.y()))
          nbps.emplace_back(common::Point(point->pt.x() + dir.x(), point->pt.y()));
        if (isUnreachable(point->pt.x(), point->pt.y() - 1)
            && !isUnreachable(point->pt.x() + dir.x(), point->pt.y() - 1))
          nbps.emplace_back(common::Point(point->pt.x() + dir.x(), point->pt.y() - 1));
        if (isUnreachable(point->pt.x(), point->pt.y() + 1)
            && !isUnreachable(point->pt.x() + dir.x(), point->pt.y() + 1))
          nbps.emplace_back(common::Point(point->pt.x() + dir.x(), point->pt.y() + 1));
      } else {
        if (!isUnreachable(point->pt.x(), point->pt.y() + dir.y()))
          nbps.emplace_back(common::Point(point->pt.x(), point->pt.y() + dir.y()));
        if (isUnreachable(point->pt.x() - 1, point->pt.y())
            && !isUnreachable(point->pt.x() - 1, point->pt.y() + dir.y()))
          nbps.emplace_back(common::Point(point->pt.x() - 1, point->pt.y() + dir.y()));
        if (isUnreachable(point->pt.x() + 1, point->pt.y())
            && !isUnreachable(point->pt.x() + 1, point->pt.y() + dir.y()))
          nbps.emplace_back(common::Point(point->pt.x() + 1, point->pt.y() + dir.y()));
      }
    }
  }
  return nbps;
}

common::Point JpsSearch::checkJumpPoint(const common::Point& targetpt, const common::Point& prept) {
  common::Point dir = targetpt - prept;
  common::Point tmp = common::Point(-1, -1);

  if (isUnreachable(targetpt.x(), targetpt.y())) {
    return tmp;
  }
  if (targetpt.x() == finalpoint_.x() && targetpt.y() == finalpoint_.y()) {
    return targetpt;
  }

  if (dir.x() != 0 && dir.y() != 0) {
    if ((!isUnreachable(targetpt.x() - dir.x(), targetpt.y() + dir.y())
         && isUnreachable(targetpt.x() - dir.x(), targetpt.y()))
        || (!isUnreachable(targetpt.x() + dir.x(), targetpt.y() - dir.y())
            && isUnreachable(targetpt.x(), targetpt.y() - dir.y()))) {
      return targetpt;
    }

  } else {
    if (dir.x() != 0) {
      if ((!isUnreachable(targetpt.x() + dir.x(), targetpt.y() + 1) && isUnreachable(targetpt.x(), targetpt.y() + 1))
          || (!isUnreachable(targetpt.x() + dir.x(), targetpt.y() - 1)
              && isUnreachable(targetpt.x(), targetpt.y() - 1))) {
        return targetpt;
      }

    } else {
      if ((!isUnreachable(targetpt.x() - 1, targetpt.y() + dir.y()) && isUnreachable(targetpt.x() - 1, targetpt.y()))
          || (!isUnreachable(targetpt.x() + 1, targetpt.y() + dir.y())
              && isUnreachable(targetpt.x() + 1, targetpt.y()))) {
        return targetpt;
      }
    }
  }

  if (dir.x() != 0 && dir.y() != 0) {
    tmp                = checkJumpPoint(common::Point(targetpt.x() + dir.x(), targetpt.y()), targetpt);
    common::Point tmp2 = checkJumpPoint(common::Point(targetpt.x(), targetpt.y() + dir.y()), targetpt);
    if (tmp.x() != -1 || tmp2.x() != -1)
      return targetpt;
  }
  if (!isUnreachable(targetpt.x() + dir.x(), targetpt.y()) || !isUnreachable(targetpt.x(), targetpt.y() + dir.y())) {
    tmp = checkJumpPoint(common::Point(targetpt.x() + dir.x(), targetpt.y() + dir.y()), targetpt);
    if (tmp.x() != -1)
      return tmp;
  }

  return tmp;
}

JpsNode* JpsSearch::isInList(const std::list<JpsNode*>& list, const JpsNode* point) const {
  for (auto p : list)
    if (p->pt.x() == point->pt.x() && p->pt.y() == point->pt.y())
      return p;
  return NULL;
}

bool JpsSearch::isUnreachable(const int x, int y) const {
  if (x < 0 || y < 0 || y >= map_.size() || x >= map_[0].size())
    return true;
  if (map_[y][x] == common::GridMapType::GRID_MAP_OBSTANCE) {
    return true;
  } else {
    return false;
  }
}

JpsNode* JpsSearch::getLeastFpoint() {
  if (!openlist_.empty()) {
    auto resPoint = openlist_.front();
    for (auto& point : openlist_)
      if (point->F < resPoint->F)
        resPoint = point;
    return resPoint;
  }
  return NULL;
}

void JpsSearch::setMap(const std::shared_ptr<costmap_2d::Costmap2D>& map) {
  costmap_ = map;
  map_cnt_++;
  map_.clear();
  cols_ = map->GetSizeInCellsX();
  rows_ = map->GetSizeInCellsY();

  for (size_t y = 0; y < rows_; y++) {
    std::vector<int> tmp;
    for (size_t x = 0; x < cols_; x++) {
      auto value = map->GetCost(x, y);
      if (value >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        tmp.emplace_back(common::GridMapType::GRID_MAP_OBSTANCE);
      } else {
        tmp.emplace_back(common::GridMapType::GRID_MAP_FREE);
      }
    }
    map_.emplace_back(tmp);
  }
  openlist_.clear();
  closelist_.clear();
}

bool JpsSearch::findPath(const common::Pose& start_pose, const common::Pose& end_pose, std::vector<common::Pose>& path,
                         bool find_replace) {
  LOG_INFO << "enter findPath end_pose x = " << end_pose.x() << " y = " << end_pose.y();
  LOG_INFO << "enter findPath start_pose x = " << start_pose.x() << " y = " << start_pose.y();
  LOG_INFO << " find_replace = " << find_replace;
  std::vector<common::Pose> path_local;
  openlist_.clear();
  closelist_.clear();
  path.clear();
  std::vector<common::Point> path_point;
  path_point.clear();
  common::Point start, end;
  common::Point new_end_point, new_start_point;
  Sensor::GetInstance()->worldToMap(start_pose.x(), start_pose.y(), start.x(), start.y());
  if (Sensor::GetInstance()->worldToMap(end_pose.x(), end_pose.y(), end.x(), end.y()) == false) {
    LOG_INFO << "end is out map";
    return false;
  }
  new_end_point   = end;
  new_start_point = start;
  int start_cost  = costmap_->GetCost(start.x(), start.y());
  int goal_cost   = costmap_->GetCost(end.x(), end.y());
  LOG_INFO << "start cost = " << start_cost << " goal_cost = " << goal_cost;
  if (start_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    common::Pose new_start_pose;
    assert(findReplacePose(start_pose, new_start_pose, map_, cols_, rows_));
    Sensor::GetInstance()->worldToMap(new_start_pose.x(), new_start_pose.y(), new_start_point.x(), new_start_point.y());
  }
  if (find_replace == true && goal_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    common::Pose new_goal_pose;
    assert(findReplacePose(end_pose, new_goal_pose, map_, cols_, rows_));
    Sensor::GetInstance()->worldToMap(new_goal_pose.x(), new_goal_pose.y(), new_end_point.x(), new_end_point.y());
  }
  JpsNode startpt = JpsNode(new_start_point);
  JpsNode endpt   = JpsNode(new_end_point);
  finalpoint_     = new_end_point;
  JpsNode* result = findPath(startpt, endpt);
  // get path, path need reverse
  while (result) {
    path_point.emplace_back(result->pt);
    result = result->parent;
  }
  if (!path_point.empty()) {
    std::reverse(path_point.begin(), path_point.end());
    for (auto point : path_point) {
      common::Pose pose;
      Sensor::GetInstance()->mapToWorld(point.x(), point.y(), pose.x(), pose.y());
      path_local.push_back(pose);
    }
  }
  Sensor::GetInstance()->publishPath("path_local", path_local);
  if (!path_local.empty()) {
    path                  = path_local;
    path[path.size() - 1] = end_pose;
    LOG_INFO << "path size = " << path.size();
    return true;
  }
  return false;
}

}  // namespace nav
