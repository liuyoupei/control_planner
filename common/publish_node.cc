#include "publish_node.h"
#include "RateTimer.h"
#include "common/common.h"
#include "thrid_party/glog/logging.h"
std::shared_ptr<RosPublishNode> RosPublishNode::ros_publish_ptr_ = nullptr;
std::mutex                      RosPublishNode::ros_publish_mutex_;
RosPublishNode::RosPublishNode() {
  ros::NodeHandle n;
  pub_cmd_vel_     = n.advertise<geometry_msgs::Twist>("/robot/controller/cmd_vel", 1);
  pub_mark_        = n.advertise<visualization_msgs::Marker>("/robot/mark", 1);
  pub_room_ids_    = n.advertise<visualization_msgs::Marker>("/room_ids", 1);
  pub_room_points_ = n.advertise<visualization_msgs::Marker>("/room_points", 1);
  pub_door_lines_  = n.advertise<visualization_msgs::Marker>("/door_lines", 1);
  pub_clean_map_   = n.advertise<nav_msgs::OccupancyGrid>("/clean_map", 1);
  m_rateRunner_ = move(std::thread{ [this] {
    pthread_setname_np(pthread_self(), "pub_node");
    RateTimer rateTimer{ 5 };
    while (1) {
      pubPath();
      pubRoom();
      pubMark();
      pubPolygon();
      rateTimer.sleep();
    }
  } });
}

RosPublishNode::~RosPublishNode() {

}

bool RosPublishNode::worldToMap(const double& wx, const double& wy, int& mx, int& my, const common::Map& map) {
  mx = int((wx - map.info.origin.x()) / map.info.resolution + 0.5);
  my = int((wy - map.info.origin.y()) / map.info.resolution + 0.5);
  if (mx < 0 || mx >= map.info.width || my < 0 || my >= map.info.height) {
    return false;
  }
  return true;
}

void RosPublishNode::pubRoom() {
  std::unique_lock<std::mutex> guard(ros_publish_room_mutex_);
  if (!door_marker_msg_.points.empty()) {
    pub_door_lines_.publish(door_marker_msg_);
  }
  if (!room_marker_msg_.points.empty()) {
    pub_room_points_.publish(room_marker_msg_);
  }
  if (!room_id_msg_.empty()) {
    for (auto data : room_id_msg_)
      pub_room_ids_.publish(data);
  }
}

void RosPublishNode::pubCmdVel(const common::Twist& twist) {
  geometry_msgs::Twist twist_data;
  twist_data.linear.x  = twist.linear_x;
  twist_data.angular.z = twist.angular_z;
  pub_cmd_vel_.publish(twist_data);
}

void RosPublishNode::pubPath() {
  if (path_publisher_.empty()) {
    return;
  }
  std::unique_lock<std::mutex> guard(ros_publish_path_mutex_);
  for (auto path : path_publisher_) {
    nav_msgs::Path             path_msg;
    geometry_msgs::PoseStamped poseStamped;
    path_msg.header.frame_id    = "map";
    poseStamped.header.frame_id = "map";
    for (auto data : path.second) {
      poseStamped.pose.position.x    = data.x();
      poseStamped.pose.position.y    = data.y();
      poseStamped.pose.position.z    = 0.05;
      poseStamped.pose.orientation.w = 1;
      path_msg.poses.push_back(poseStamped);
    }
    path.first.publish(path_msg);
  }
}

void RosPublishNode::pubPath(const char* name, const std::vector<common::Pose>& path_data) {
  ros::NodeHandle              n;
  ros::Publisher               publisher_name;
  bool                         is_new_name = true;
  std::unique_lock<std::mutex> guard(ros_publish_path_mutex_);
  for (auto publisher : path_publisher_) {
    std::string str_name(name);
    if (publisher.first.getTopic() == "/" + str_name) {
      is_new_name = false;
      break;
    }
  }
  if (true == is_new_name) {
    publisher_name        = n.advertise<nav_msgs::Path>(name, 1);
    publisher_buff_[name] = publisher_name;
  }
  path_publisher_[publisher_buff_[name]] = path_data;
}

void RosPublishNode::pubPolygon(const char* name, const std::vector<common::Pose>& path_polygon) {
  ros::NodeHandle              n;
  ros::Publisher               publisher_name;
  bool                         is_new_name = true;
  std::unique_lock<std::mutex> guard(ros_publish_polygon_mutex_);
  for (auto publisher : polygon_publisher_) {
    std::string str_name(name);
    if (publisher.first.getTopic() == "/" + str_name) {
      is_new_name = false;
      break;
    }
  }
  if (true == is_new_name) {
    publisher_name                = n.advertise<geometry_msgs::PolygonStamped>(name, 1);
    publisher_polygon_buff_[name] = publisher_name;
  }
  polygon_publisher_[publisher_polygon_buff_[name]] = path_polygon;
}

void RosPublishNode::pubPolygon() {
  if (polygon_publisher_.empty()) {
    return;
  }
  std::unique_lock<std::mutex> guard(ros_publish_path_mutex_);
  for (auto polygon : polygon_publisher_) {
    geometry_msgs::PolygonStamped polygon_msg;
    geometry_msgs::Point32        poseStamped;
    polygon_msg.header.stamp    = ros::Time::now();
    polygon_msg.header.frame_id = "map";
    for (auto data : polygon.second) {
      poseStamped.x = data.x();
      poseStamped.y = data.y();
      poseStamped.z = 0.15;
      polygon_msg.polygon.points.push_back(poseStamped);
    }
    polygon.first.publish(polygon_msg);
  }
}

void RosPublishNode::pubCleanMap(const common::Pose& robot_pose, const common::Map& map) {
  if (robot_pose.distanceTo(last_pose_) >= 0.1) {
    last_pose_ = robot_pose;
  } else {
    return;
  }
  float min_x                          = robot_pose.x() - 0.2;
  float max_x                          = robot_pose.x() + 0.2;
  float min_y                          = robot_pose.y() - 0.2;
  float max_y                          = robot_pose.y() + 0.2;
  clean_map_.info.origin.position.x    = map.info.origin.x();
  clean_map_.info.origin.position.y    = map.info.origin.y();
  clean_map_.info.origin.orientation.w = 1;
  clean_map_.header.frame_id           = "map";
  clean_map_.info.width                = map.info.width;
  clean_map_.info.height               = map.info.height;
  clean_map_.info.resolution           = map.info.resolution;
  clean_map_.data.resize(map.info.height * map.info.width);
  if (clean_map_.data.size() != clean_map_.info.width * clean_map_.info.height) {
    clean_map_.data.clear();
    clean_map_.data.resize(clean_map_.info.width * clean_map_.info.height);
  }
  common::Rect<int> rect;
  worldToMap(min_x, min_y, rect.min_x, rect.min_y, map);
  worldToMap(max_x, max_y, rect.max_x, rect.max_y, map);
  for (int y = rect.min_y; y < rect.max_y; y++) {
    for (int x = rect.min_x; x < rect.max_x; x++) {
      int index = y * clean_map_.info.width + x;
      if (clean_map_.data[index] == 0) {
        clean_map_.data[index] = -1;
      }
    }
  }
  pub_clean_map_.publish(clean_map_);
}

void RosPublishNode::pubMark() {
  if (mark_buff_.empty()) {
    return;
  }
  std::unique_lock<std::mutex> guard(ros_publish_mark_mutex_);
  for (const auto& data : mark_buff_) {
    pub_mark_.publish(data);
  }
}

void RosPublishNode::pubRoom(const std::unordered_map<int, room::RegionData>& room_data, const common::Map& map) {
  std::unique_lock<std::mutex> guard(ros_publish_room_mutex_);
  visualization_msgs::Marker   room_marker_msg;
  room_marker_msg.ns      = "room_points";
  room_marker_msg.id      = 0;
  room_marker_msg.type    = visualization_msgs::Marker::LINE_LIST;
  room_marker_msg.scale.x = map.info.resolution;
  room_marker_msg.scale.y = map.info.resolution;
  room_marker_msg.color.r = 1.0;
  room_marker_msg.color.g = 1.0;
  room_marker_msg.color.a = 1.0;

  visualization_msgs::Marker room_id_msg;
  room_id_msg                    = room_marker_msg;
  room_marker_msg.ns             = "room_ids";
  room_id_msg.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
  room_id_msg.action             = visualization_msgs::Marker::ADD;
  room_id_msg.header.frame_id    = "map";
  room_id_msg.pose.orientation.w = 1.0;
  room_id_msg.scale.x            = 0.5;
  room_id_msg.scale.y            = 0.5;
  room_id_msg.scale.z            = 0.5;

  visualization_msgs::Marker door_marker_msg;
  door_marker_msg.ns      = "door_lines";
  door_marker_msg.id      = 0;
  door_marker_msg.type    = visualization_msgs::Marker::LINE_LIST;
  door_marker_msg.scale.x = map.info.resolution;
  door_marker_msg.scale.y = map.info.resolution;
  door_marker_msg.color.g = 1.0;
  door_marker_msg.color.a = 1.0;

  for (auto data : room_data) {
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    int rooms_size  = data.second.rooms.size();
    for (int j = 0; j < rooms_size - 1; j++) {
      geometry_msgs::Point p;
      p.x = data.second.rooms[j].x;
      p.y = data.second.rooms[j].y;
      room_marker_msg.points.push_back(p);
      pose.position.x += p.x;
      pose.position.y += p.y;
      p.x = data.second.rooms[j + 1].x;
      p.y = data.second.rooms[j + 1].y;
      room_marker_msg.points.push_back(p);
    }
    geometry_msgs::Point p;
    p.x = data.second.rooms[data.second.rooms.size() - 1].x;
    p.y = data.second.rooms[data.second.rooms.size() - 1].y;
    room_marker_msg.points.push_back(p);
    pose.position.x += p.x;
    pose.position.y += p.y;
    p.x = data.second.rooms.front().x;
    p.y = data.second.rooms.front().y;
    room_marker_msg.points.push_back(p);
    pose.position.x /= rooms_size;
    pose.position.y /= rooms_size;
    std::stringstream ss;
    ss << data.first;
    room_id_msg.text         = ss.str();
    room_id_msg.pose         = pose;
    room_id_msg.id           = data.first;
    room_id_msg.header.stamp = ros::Time::now();
    room_id_msg_.push_back(room_id_msg);
  }

  for (auto rooms : room_data) {
    for (auto door : rooms.second.doors) {
      geometry_msgs::Point p;
      p.x = door.vertex_i.x;
      p.y = door.vertex_i.y;
      p.z = 0.1;
      door_marker_msg.points.push_back(p);
      p.x = door.vertex_j.x;
      p.y = door.vertex_j.y;
      p.z = 0.1;
      door_marker_msg.points.push_back(p);
    }
  }

  room_marker_msg.header.frame_id    = "map";
  room_marker_msg.header.stamp       = ros::Time::now();
  room_marker_msg.pose.orientation.w = 1.0;

  door_marker_msg.header.frame_id    = "map";
  door_marker_msg.header.stamp       = ros::Time::now();
  door_marker_msg.pose.orientation.w = 1.0;
  room_marker_msg_                   = room_marker_msg;
  door_marker_msg_                   = door_marker_msg;
}

