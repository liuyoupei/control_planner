#include "sensor.h"
#include "common/common.h"
#include "common/publish_node.h"
std::shared_ptr<Sensor> Sensor::sensor_ptr_ = nullptr;
std::mutex              Sensor::sensor_mutex_;
Sensor::~Sensor() {}
Sensor::Sensor() {
  cmd_vel_.linear_x        = 0;
  cmd_vel_.angular_z       = 0;
  get_pose_                = false;
  get_map_                 = false;
  bumper_msg_.left_bumper  = false;
  bumper_msg_.right_bumper = false;
}


bool Sensor::isReady() {
  if (true == get_map_ && true == get_pose_) {
    return true;
  } else {
    return false;
  }
}

bool Sensor::worldToMap(const double& wx, const double& wy, int& mx, int& my) {
  mx = int((wx - map_data_.info.origin.x()) / map_data_.info.resolution + 0.5);
  my = int((wy - map_data_.info.origin.y()) / map_data_.info.resolution + 0.5);
  if (mx < 0 || mx >= map_data_.info.width || my < 0 || my >= map_data_.info.height) {
    return false;
  }
  return true;
}

bool Sensor::mapToWorld(const int& mx, const int& my, double& wx, double& wy) {
  wx = (mx * map_data_.info.resolution) + map_data_.info.origin.x();
  wy = (my * map_data_.info.resolution) + map_data_.info.origin.y();
  if (mx < 0 || mx >= map_data_.info.width || my < 0 || my >= map_data_.info.height) {
    return false;
  }
  return true;
}
void Sensor::updateSlamPose(const common::Pose& robot_pose) {
  std::unique_lock<std::mutex> guard(slam_robot_pose_mutex_);
  slam_robot_pose_ = robot_pose;
  get_pose_        = true;
}


void Sensor::updateCostmap(const common::Map& map_data) {
  std::unique_lock<std::mutex> guard(costmap_mutex_);
  if (costmap_2d_ == nullptr) {
    costmap_2d_ =
      std::make_shared<costmap_2d::Costmap2D>(map_data.info.width, map_data.info.height, map_data.info.resolution,
                                              map_data.info.origin.x(), map_data.info.origin.y());
  } else {
    costmap_2d_->ResizeMap(map_data.info.width, map_data.info.height, map_data.info.resolution,
                           map_data.info.origin.x(), map_data.info.origin.y());
  }
  int x_size = map_data.info.width;
  int y_size = map_data.info.height;
  for (int y = 0; y < y_size; y++) {
    for (int x = 0; x < x_size; x++) {
      int index = y * x_size + x;
      if (map_data.data[index] == 100) {
        costmap_2d_->SetCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      } else if (map_data.data[index] == -1) {
        costmap_2d_->SetCost(x, y, costmap_2d::NO_INFORMATION);
      } else {
        costmap_2d_->SetCost(x, y, costmap_2d::FREE_SPACE);
      }
    }
  }
  inflater_.inflate(costmap_2d_);
}


void Sensor::updateMapData(const common::Map& map_data) {
  std::unique_lock<std::mutex> guard(map_mutex_);
  map_data_ = map_data;
  get_map_  = true;
}

void Sensor::updateBumper(const common::Bumper& bumper) {
  std::unique_lock<std::mutex> guard(bumper_mutex_);
  bumper_msg_ = bumper;
}


common::Pose Sensor::getSlamPose() {
  std::unique_lock<std::mutex> guard(slam_robot_pose_mutex_);
  return slam_robot_pose_;
}
void Sensor::updateLaserScan(const common::LaserScan& laser_scan) {
  std::unique_lock<std::mutex> guard(laser_scan_mutex_);
  laser_scan_ = laser_scan;
}

common::LaserScan Sensor::getLaserScan() {
  std::unique_lock<std::mutex> guard(laser_scan_mutex_);
  return laser_scan_;
}

common::Map Sensor::getMapData() {
  std::unique_lock<std::mutex> guard(map_mutex_);
  return map_data_;
}


common::Bumper Sensor::getBumper() {
  std::unique_lock<std::mutex> guard(bumper_mutex_);
  return bumper_msg_;
}

costmap_2d::Costmap2D Sensor::getCostmap2d() {
  std::unique_lock<std::mutex> guard(costmap_mutex_);
  return *costmap_2d_;
}


void Sensor::publishCmdVel(const common::Twist& twist) {
  cmd_vel_ = twist;
  RosPublishNode::GetInstance()->pubCmdVel(twist);
}

void Sensor::publishPath(const char* name, const std::vector<common::Pose>& path) {
  RosPublishNode::GetInstance()->pubPath(name, path);
}

void Sensor::publishPolygon(const char* name, const std::vector<common::Pose>& polygon) {
  RosPublishNode::GetInstance()->pubPolygon(name, polygon);
}

void Sensor::pubCleanMap(const common::Pose& robot_pose) {
  RosPublishNode::GetInstance()->pubCleanMap(robot_pose, map_data_);
}

void Sensor::pubRoom(const std::unordered_map<int, room::RegionData>& room_data) {
  RosPublishNode::GetInstance()->pubRoom(room_data, map_data_);
}

