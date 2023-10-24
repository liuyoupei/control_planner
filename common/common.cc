#include "common.h"
#include "navigation/jps/jps_search.h"
#include "navigation/path_search.h"
#include "sensor/sensor.h"
#include <fcntl.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <unistd.h>
using namespace std;
namespace common {
std::shared_ptr<Common> Common::Common_ptr_ = nullptr;
std::mutex              Common::Common_mutex_;
Common::Common() {
  go_forward_distance_ = 0.5;
}
Common::~Common() {}

float Common::fpwrappi(float angle) {
  while (angle > M_PI)
    angle -= (2 * M_PI);
  while (angle <= -M_PI)
    angle += (2 * M_PI);
  return angle;
}

std::vector<Pose> Common::raytrace(const Pose& start_pose, const Pose& end_pose) {
  int               x0, y0, x1, y1;
  std::vector<Pose> line_data;
  Sensor::GetInstance()->worldToMap(start_pose.x(), start_pose.y(), x0, y0);
  Sensor::GetInstance()->worldToMap(end_pose.x(), end_pose.y(), x1, y1);
  int   dx = abs(x1 - x0);
  int   dy = abs(y1 - y0);
  Point pt;
  pt.x()    = x0;
  pt.y()    = y0;
  int n     = 1 + dx + dy;
  int x_inc = (x1 > x0) ? 1 : -1;
  int y_inc = (y1 > y0) ? 1 : -1;
  int error = dx - dy;
  dx *= 2;
  dy *= 2;

  for (; n > 0; --n) {
    Pose pose;
    Sensor::GetInstance()->mapToWorld(pt.x(), pt.y(), pose.x(), pose.y());
    line_data.push_back(pose);
    if (error > 0) {
      pt.x() += x_inc;
      error -= dy;
    } else {
      pt.y() += y_inc;
      error += dx;
    }
  }
  return line_data;
}

Pose Common::findFootOfLine(const Pose& pt, const Pose& begin, const Pose& end) {
  Pose  foot_point;
  float A = end.y() - begin.y();
  float B = begin.x() - end.x();
  float C = end.x() * begin.y() - begin.x() * end.y();
  //判断A==0 && B==0
  if (1e-9 == fabs(A) && 1e-9 == fabs(B)) {
    foot_point.x() = -100.0;
    foot_point.y() = -100.0;
    return foot_point;
  }
  float x        = (B * B * pt.x() - A * B * pt.y() - A * C) / (A * A + B * B);
  float y        = (-A * B * pt.x() + A * A * pt.y() - B * C) / (A * A + B * B);
  foot_point.x() = x;
  foot_point.y() = y;
  return foot_point;
}

float Common::getDiffAngle(const common::Pose& current_pose, const common::Pose& goal_pose) {
  float diff_x     = 0;
  float diff_y     = 0;
  float diff_angle = 0;
  diff_x           = goal_pose.x() - current_pose.x();
  diff_y           = goal_pose.y() - current_pose.y();
  if (diff_x <= -go_forward_distance_) {
    diff_x = -go_forward_distance_;
  }
  if (diff_y <= -go_forward_distance_) {
    diff_y = -go_forward_distance_;
  }
  if (diff_x >= go_forward_distance_) {
    diff_x = go_forward_distance_;
  }
  if (diff_y >= go_forward_distance_) {
    diff_y = go_forward_distance_;
  }
  if (diff_x == 0 || diff_y == 0) {
    return 0;
  } else {
    float angle;
    angle      = atan2(diff_y, diff_x) - (current_pose.phi());
    diff_angle = common::Common::GetInstance()->fpwrappi(angle);
    return diff_angle;
  }
}

void Common::approxPolyDP(const std::vector<Pose>& curve, std::vector<Pose>& approxCurve, double epsilon, bool closed) {
  std::vector<cv::Point> curve_points;
  for (auto data : curve) {
    cv::Point point;
    Sensor::GetInstance()->worldToMap(data.x(), data.y(), point.x, point.y);
    curve_points.push_back(point);
  }
  std::vector<cv::Point> approxCurve_points;
  cv::approxPolyDP(curve_points, approxCurve_points, epsilon, closed);
  for (auto data : approxCurve_points) {
    Pose pose;
    Sensor::GetInstance()->mapToWorld(data.x, data.y, pose.x(), pose.y());
    approxCurve.push_back(pose);
  }
}

bool Common::isClockwise(const std::vector<common::Pose>& data) {  // todo
  LOG_INFO << "enter isClockwise";
  LOG_INFO << "lyp:"
           << "data size = " << data.size();
  float d    = 0;
  int   size = data.size();
  if (size < 3) {
    LOG_INFO << "counter clockwise";
    return false;
  }
  for (int i = 0; i < size - 1; i++) {
    d += -0.5 * (data[i + 1].y() + data[i].y()) * (data[i + 1].x() - data[i].x());
  }
  d += -0.5 * (data[0].y() + data[size - 1].y()) * (data[0].x() - data[size - 1].x());
  LOG_INFO << "d = " << d;
  if (d >= 0) {
    LOG_INFO << "counter clockwise";
    return false;
  } else {
    LOG_INFO << "clockwise";
    return true;
  }
}

float Common::getPathDistance(const std::shared_ptr<costmap_2d::Costmap2D>& costmap, const common::Pose& start,
                              const common::Pose& goal) {
  std::vector<common::Pose>        path;
  std::shared_ptr<nav::PathSearch> path_search = std::make_shared<nav::JpsSearch>();
  path_search->setMap(costmap);
  path_search->findPath(start, goal, path);
  if (path.empty()) {
    return 0;
  }
  int   size     = path.size();
  float distance = 0;
  for (int i = 1; i < size; i++) {
    distance += path[i - 1].distanceTo(path[i]);
  }
  return distance;
}

}  // namespace common