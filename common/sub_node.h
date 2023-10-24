#ifndef CLEAN_ROBOT_ROS_SUBSCRIBE_NODE_H
#define CLEAN_ROBOT_ROS_SUBSCRIBE_NODE_H

#include "common/common_type.h"
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <thread>
class RosSubscribeNode {
public:
  RosSubscribeNode();

private:
  void odomCallBack(const geometry_msgs::PoseStamped& msg);
  void laserCallBack(const sensor_msgs::LaserScan& msg);
  void bumperCallBack(const std_msgs::ByteMultiArray& msg);
  void mapCallBack(const nav_msgs::OccupancyGrid& msg);

private:
  std::thread     m_rateRunner_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_laser_;
  ros::Subscriber sub_bumper_;
  ros::Subscriber sub_map_;
};

#endif  // CLEAN_ROBOT_ROS_SENSOR_NODE_H
