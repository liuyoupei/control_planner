#include "sub_node.h"
#include "RateTimer.h"
#include "sensor/sensor.h"
RosSubscribeNode::RosSubscribeNode() {
  ros::NodeHandle n;
  sub_odom_           = n.subscribe("/tf_odom", 1, &RosSubscribeNode::odomCallBack, this);
  sub_map_            = n.subscribe("/map", 1, &RosSubscribeNode::mapCallBack, this);
  sub_laser_          = n.subscribe("/front_scan", 1, &RosSubscribeNode::laserCallBack, this);
  sub_bumper_         = n.subscribe("/bumper", 1, &RosSubscribeNode::bumperCallBack, this);
  m_rateRunner_       = move(std::thread{ [this] {
    std::string name = "sub_node";
    pthread_setname_np(pthread_self(), name.c_str());
    RateTimer rateTimer{ 10 };
    while (1) {
      ros::spinOnce();
      rateTimer.sleep();
    }
  } });
}

void RosSubscribeNode::odomCallBack(const geometry_msgs::PoseStamped& msg) {
  common::Pose robot_pose;
  robot_pose.x()   = msg.pose.position.x;
  robot_pose.y()   = msg.pose.position.y;
  robot_pose.phi() = tf::getYaw(msg.pose.orientation);
  Sensor::GetInstance()->updateSlamPose(robot_pose);
}


void RosSubscribeNode::mapCallBack(const nav_msgs::OccupancyGrid& msg) {
  common::Map map_data;
  map_data.info.width        = msg.info.width;
  map_data.info.height       = msg.info.height;
  map_data.info.resolution   = msg.info.resolution;
  map_data.info.origin.x()   = msg.info.origin.position.x;
  map_data.info.origin.y()   = msg.info.origin.position.y;
  map_data.info.origin.phi() = tf::getYaw(msg.info.origin.orientation);
  int size                   = msg.data.size();
  for (int i = 0; i < size; i++) {
    map_data.data.push_back(msg.data[i]);
  }
  map_data.header.frame_id = msg.header.frame_id;
  Sensor::GetInstance()->updateMapData(map_data);
}

void RosSubscribeNode::laserCallBack(const sensor_msgs::LaserScan& msg) {
  common::LaserScan laser_scan;
  laser_scan.angle_min       = msg.angle_min;
  laser_scan.angle_max       = msg.angle_max;
  laser_scan.angle_increment = msg.angle_increment;
  laser_scan.range_max       = msg.range_max;
  laser_scan.range_min       = msg.range_min;
  laser_scan.scan_time       = msg.scan_time;
  laser_scan.ranges          = msg.ranges;
  int    size                = msg.ranges.size();
  float  angle               = 0;
  double angle_increment     = msg.angle_increment;
  for (int i = 0; i < size; i++) {
    angle -= angle_increment;
    if (angle <= -M_PI) {
      laser_scan.angles.push_back(angle + M_PI * 2);
    } else {
      laser_scan.angles.push_back(angle);
    }
  }
  Sensor::GetInstance()->updateLaserScan(laser_scan);
}

void RosSubscribeNode::bumperCallBack(const std_msgs::ByteMultiArray& msg) {
  common::Bumper bumper;
  bumper.right_bumper = msg.data[0];
  bumper.left_bumper  = msg.data[1];
  Sensor::GetInstance()->updateBumper(bumper);
}

