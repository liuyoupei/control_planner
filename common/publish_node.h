#ifndef CLEAN_ROBOT_ROS_PUBLISH_NODE_H_
#define CLEAN_ROBOT_ROS_PUBLISH_NODE_H_

#include "common/common_type.h"
#include "common/region_type.h"
#include "map/costmap_2d.h"
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
#include <visualization_msgs/MarkerArray.h>
class RosPublishNode {
public:
  RosPublishNode();
  ~RosPublishNode();
  static std::shared_ptr<RosPublishNode> GetInstance() {
    if (ros_publish_ptr_ == nullptr) {
      std::lock_guard<std::mutex> lk(ros_publish_mutex_);
      if (ros_publish_ptr_ == nullptr) {
        ros_publish_ptr_ = std::shared_ptr<RosPublishNode>(new RosPublishNode());
      }
    }
    return ros_publish_ptr_;
  }
  void pubPath();
  void pubPath(const char* name, const std::vector<common::Pose>& path_data);
  void pubCmdVel(const common::Twist& twist);
  void pubCleanMap(const common::Pose& robot_pose, const common::Map& map);
  void pubMark();
  void pubRoom(const std::unordered_map<int, room::RegionData>& room_data, const common::Map& map);
  void pubRoom();
  void pubPolygon(const char* name, const std::vector<common::Pose>& path_polygon);
  void pubPolygon();
  bool worldToMap(const double& wx, const double& wy, int& mx, int& my, const common::Map& map);

private:
  std::thread                                         m_rateRunner_;
  static std::shared_ptr<RosPublishNode>              ros_publish_ptr_;
  static std::mutex                                   ros_publish_mutex_;
  std::mutex                                          ros_publish_mark_mutex_;
  std::mutex                                          ros_publish_path_mutex_;
  std::mutex                                          ros_publish_room_mutex_;
  std::mutex                                          ros_publish_polygon_mutex_;
  std::map<ros::Publisher, std::vector<common::Pose>> path_publisher_;
  std::map<ros::Publisher, std::vector<common::Pose>> polygon_publisher_;
  std::map<std::string, ros::Publisher>               publisher_buff_;
  std::map<std::string, ros::Publisher>               publisher_polygon_buff_;
  std::vector<visualization_msgs::Marker>             mark_buff_;
  /* data */
  ros::Publisher                          pub_mark_;
  ros::Publisher                          pub_room_ids_;
  ros::Publisher                          pub_room_points_;
  ros::Publisher                          pub_door_lines_;
  ros::Publisher                          pub_clean_map_;
  ros::Publisher                          pub_cmd_vel_;
  nav_msgs::OccupancyGrid                 clean_map_;
  std::vector<visualization_msgs::Marker> room_id_msg_;
  visualization_msgs::Marker              door_marker_msg_;
  visualization_msgs::Marker              room_marker_msg_;
  common::Pose last_pose_;
};

#endif
