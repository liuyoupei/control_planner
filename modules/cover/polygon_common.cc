#include "polygon_common.h"
#include "sensor/sensor.h"
Polygon::Polygon(const std::vector<common::Pose>& points) : points_(points) {
  for (auto point : points) {
    common::Point chain_point;
    Sensor::GetInstance()->worldToMap(point.x(), point.y(), chain_point.x(), chain_point.y());
    polygon_rect_.update(chain_point.x(), chain_point.y());
  }
  polygon_rect_.expand(0.1 / 0.05);
  shrink_distance_add_        = 0.15;
  shrink_distance_sub_        = 0.2;
  remove_intersects_distance_ = 0.2;
  LOG_INFO << "points_ size = " << points_.size();
}

bool Polygon::getRectFromPolygon(common::Rect<int>& rect) {
  if (points_.empty()) {
    return false;
  }
  rect = polygon_rect_;
  return true;
}

int Polygon::intersectWithLineCoverX(const std::shared_ptr<costmap_2d::Costmap2D>& costmap, int x,
                                     std::vector<common::Pose>& intersects) {
  LOG_INFO << "enter intersectWithLineCoverX points_";
  intersects.clear();
  bool         lase_is_in_polygon = false;
  common::Pose pose;
  double       min_y, max_y;
  double       world_x;
  Sensor::GetInstance()->mapToWorld(x, polygon_rect_.min_y, world_x, min_y);
  Sensor::GetInstance()->mapToWorld(x, polygon_rect_.max_y, world_x, max_y);
  float offset_y = 0.05;
  for (double y = min_y; y < max_y; y += offset_y) {
    pose.x() = world_x;
    pose.y() = y;
    bool is_in_polygon = isInPolygon(points_, pose);
    if (y == polygon_rect_.min_y) {
      lase_is_in_polygon = is_in_polygon;
      LOG_INFO << "init lase_is_in_polygon = " << lase_is_in_polygon;
      continue;
    }
    if (lase_is_in_polygon != is_in_polygon) {
      intersects.push_back(pose);
      if (intersects.size() % 2 == 0) {
        if (intersects.at(intersects.size() - 1).distanceTo(intersects.at(intersects.size() - 2))
            <= remove_intersects_distance_) {
          LOG_INFO << "intersectWithLineCoverX distance = "
                   << intersects[intersects.size() - 1].distanceTo(intersects[intersects.size() - 2]);
          intersects.pop_back();
          intersects.pop_back();
        } else {
          if (intersects.at(intersects.size() - 1).y() > intersects.at(intersects.size() - 2).y()) {
            intersects.at(intersects.size() - 1).y() -= shrink_distance_sub_;
            intersects.at(intersects.size() - 2).y() += shrink_distance_add_;
          } else {
            intersects.at(intersects.size() - 1).y() += shrink_distance_add_;
            intersects.at(intersects.size() - 2).y() -= shrink_distance_sub_;
          }
          if (intersects.at(intersects.size() - 1).distanceTo(intersects.at(intersects.size() - 2)) <= 0.1) {
            intersects.pop_back();
            intersects.pop_back();
          }
        }
      }
    }
    lase_is_in_polygon = is_in_polygon;
  }
  return intersects.size();
}

int Polygon::intersectWithLineCoverY(const std::shared_ptr<costmap_2d::Costmap2D>& costmap, int y,
                                     std::vector<common::Pose>& intersects) {
  LOG_INFO << "enter intersectWithLineCoverY";
  intersects.clear();
  bool   lase_is_in_polygon = false;
  double min_x, max_x;
  double world_y;
  Sensor::GetInstance()->mapToWorld(polygon_rect_.min_x, y, min_x, world_y);
  Sensor::GetInstance()->mapToWorld(polygon_rect_.max_x, y, max_x, world_y);
  float offset_x = 0.05;
  for (double x = min_x; x < max_x; x += offset_x) {
    common::Pose pose;
    pose.x() = x;
    pose.y() = world_y;
    bool is_in_polygon = isInPolygon(points_, pose);
    if (x == polygon_rect_.min_x) {
      lase_is_in_polygon = is_in_polygon;
      LOG_INFO << "init lase_is_in_polygon = " << lase_is_in_polygon;
      continue;
    }
    if (lase_is_in_polygon != is_in_polygon) {
      intersects.push_back(pose);
      if (intersects.size() % 2 == 0) {
        if (intersects.at(intersects.size() - 1).distanceTo(intersects.at(intersects.size() - 2))
            <= remove_intersects_distance_) {
          LOG_INFO << "intersectWithLineCoverX distance = "
                   << intersects.at(intersects.size() - 1).distanceTo(intersects.at(intersects.size() - 2));
          intersects.pop_back();
          intersects.pop_back();
        } else {
          if (intersects.at(intersects.size() - 1).x() > intersects.at(intersects.size() - 2).x()) {
            intersects.at(intersects.size() - 1).x() -= shrink_distance_add_ * 1.3;
            intersects.at(intersects.size() - 2).x() += shrink_distance_add_;
          } else {
            intersects.at(intersects.size() - 1).x() += shrink_distance_add_;
            intersects.at(intersects.size() - 2).x() -= shrink_distance_add_ * 1.3;
          }
          if (intersects.at(intersects.size() - 1).distanceTo(intersects.at(intersects.size() - 2)) <= 0.1) {
            intersects.pop_back();
            intersects.pop_back();
          }
        }
      }
    }
    lase_is_in_polygon = is_in_polygon;
  }
  return intersects.size();
}

bool Polygon::isInPolygon(const std::vector<common::Pose>& poses, const common::Pose& pose) {
  int  i, j;
  bool in_polygon = false;
  for (i = 0, j = poses.size() - 1; i < poses.size(); j = i++) {
    if (((poses[i].y() > pose.y()) != (poses[j].y() > pose.y()))
        && (pose.x()
            < (poses[j].x() - poses[i].x()) * (pose.y() - poses[i].y()) / (poses[j].y() - poses[i].y()) + poses[i].x()))
      in_polygon = !in_polygon;
  }
  return in_polygon;
}
