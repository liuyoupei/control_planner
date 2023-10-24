#ifndef CLEAN_ROBOT_SENSOR_H
#define CLEAN_ROBOT_SENSOR_H
#include "RateTimer.h"
#include "common/common_type.h"
#include "common/enum_type.h"
#include "common/region_type.h"
#include "map/costmap_2d.h"
#include "map/inflater.h"
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
class Sensor {
public:
  ~Sensor();
  Sensor();
  static std::shared_ptr<Sensor> GetInstance() {
    if (sensor_ptr_ == nullptr) {
      std::lock_guard<std::mutex> lk(sensor_mutex_);
      if (sensor_ptr_ == nullptr) {
        sensor_ptr_ = std::shared_ptr<Sensor>(new Sensor());
      }
    }
    return sensor_ptr_;
  }
  /****publish data****/
  void publishCmdVel(const common::Twist& twist);
  void publishPath(const char* name, const std::vector<common::Pose>& path);
  void pubCleanMap(const common::Pose& robot_pose);
  void pubRoom(const std::unordered_map<int, room::RegionData>& room_data);
  void publishPolygon(const char* name, const std::vector<common::Pose>& polygon);
  /***update data***/
  void updateSlamPose(const common::Pose& robot_pose);
  void updateMapData(const common::Map& map_data);
  void updateBumper(const common::Bumper& bumper);
  void updateCostmap(const common::Map& map_data);
  void updateLaserScan(const common::LaserScan& laser_scan);
  /***update slam status***/
  /***get data***/
  common::Pose             getSlamPose();
  common::LaserScan        getLaserScan();
  common::Map              getMapData();
  common::Bumper           getBumper();
  bool                     worldToMap(const double& wx, const double& wy, int& mx, int& my);
  bool                     mapToWorld(const int& mx, const int& my, double& wx, double& wy);
  bool                     isReady();
  costmap_2d::Costmap2D    getCostmap2d();
private:
  static std::shared_ptr<Sensor>         sensor_ptr_;
  static std::mutex                      sensor_mutex_;
  std::mutex                             slam_robot_pose_mutex_;
  std::mutex                             laser_scan_mutex_;
  std::mutex                             map_mutex_;
  std::mutex                             bumper_mutex_;
  std::mutex                             costmap_mutex_;
  common::Pose                           slam_robot_pose_;
  common::LaserScan                      laser_scan_;
  common::Map                            map_data_;
  common::Twist                          cmd_vel_;
  common::Bumper                         bumper_msg_;
  bool                                   get_pose_;
  bool                                   get_map_;
  std::shared_ptr<costmap_2d::Costmap2D> costmap_2d_;
  costmap_2d::Inflater                   inflater_;
};
#endif  // CLEAN_ROBOT_SENSOR_H
