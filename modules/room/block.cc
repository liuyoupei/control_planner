#include "block.h"
#include "common/common.h"
#include "common/enum_type.h"
#include "common/region_type.h"
#include "map/costmap_2d.h"
#include "sensor/sensor.h"
#include <memory>
#include <yaml-cpp/yaml.h>
using namespace common;
namespace block {
Block::Block() {
  YAML::Node config = YAML::LoadFile(FLAGS_config_path);
  room_datas_.clear();
}
Block::~Block() {}
void Block::reset() {
  LOG_INFO << "enter reset";
  room_datas_.clear();
  active_segment_points_.clear();
  active_segment_pose_map_.clear();
  all_room_data_.clear();
}
void Block::initRoomData(const std::shared_ptr<costmap_2d::Costmap2D>& costmap, const common::Map& map) {
  active_segment_points_.clear();
  room_datas_.clear();
  auto map_local = std::make_shared<common::Map>(map);
  room_region_division_.setMap(map_local);
  room_region_division_.process();
  room_region_division_.getRegionData(region_datas_);
  regionDataToRoomData(costmap, region_datas_, room_datas_);
  Sensor::GetInstance()->pubRoom(region_datas_);
}

bool Block::getNextRoom(const common::Pose& robot_pose, const std::shared_ptr<costmap_2d::Costmap2D>& costmap,
                        const common::Map& map, common::RoomData& room_data) {
  LOG_INFO << "enter getNextRoom";
  active_segment_points_.clear();
  if (room_datas_.empty()) {
    return false;
  }
  bool success = getNextRoomData(costmap, robot_pose, room_data);
  if (success == true) {
    current_runing_rooom_  = room_data;
    active_segment_points_ = room_data.active_segment;
    sortActiveSegmentPoints(robot_pose, active_segment_points_);
  }
  return success;
}

bool Block::getNextRoomData(const std::shared_ptr<costmap_2d::Costmap2D>& costmap, const common::Pose& robot_pose,
                            common::RoomData& room_data) {
  LOG_INFO << "enter getNextRoomData";
  LOG_INFO << "room datas size = " << room_datas_.size();
  std::vector<common::RoomData>::iterator near_iter;
  if (room_datas_.empty()) {
    return false;
  }
  if (room_datas_.size() == 1) {
    near_iter = room_datas_.begin();
  } else {
    float min_distance = 1e5;
    for (std::vector<common::RoomData>::iterator iter = room_datas_.begin(); iter != room_datas_.end(); iter++) {
      for (auto edge_pose : iter->edges) {
        float temp_distance =
          common::Common::GetInstance()->getPathDistance(costmap, robot_pose, edge_pose.getMiddlePose());
        if (temp_distance < min_distance) {
          min_distance = temp_distance;
          near_iter    = iter;
        }
      }
    }
  }
  room_data = *near_iter;
  near_iter->times--;
  LOG_INFO << " times = " << near_iter->times;
  LOG_INFO << "room datas = " << room_datas_.size();
  if (near_iter->times <= 0) {
    room_datas_.erase(near_iter);
    LOG_INFO << "erase near_iter";
  }
  return true;
}

void Block::regionDataToRoomData(const std::shared_ptr<costmap_2d::Costmap2D>&    costmap,
                                 const std::unordered_map<int, room::RegionData>& region_data,
                                 std::vector<common::RoomData>&                   room_datas) {
  LOG_INFO << "enter regionDataToRoomData";
  common::Point point_coor;
  for (auto data : region_data) {
    common::RoomData room_data;
    room_data.id = data.first;
    point_coor.x()++;
    point_coor.y()++;
    for (auto pose : data.second.rooms) {
      room_data.outline.push_back(Pose(pose.x, pose.y));
    }
    for (auto door : data.second.doors) {
      common::Edge edge;
      edge.start_pose.x() = door.vertex_i.x;
      edge.start_pose.y() = door.vertex_i.y;
      edge.end_pose.x()   = door.vertex_j.x;
      edge.end_pose.y()   = door.vertex_j.y;
      edge.edge_type      = common::BlockEdgeType::ROOM_DOOR;
      edge.room_type      = common::RoomType::ROOM_DIVISION;
      room_data.edges.push_back(edge);
      std::vector<common::Pose> raytrace_data;
      raytrace_data = common::Common::GetInstance()->raytrace(edge.start_pose, edge.end_pose);
      int size      = raytrace_data.size();
      for (int i = 0; i < size; i++) {
        Point point;
        Pose  pose;
        pose = raytrace_data[i];
        Sensor::GetInstance()->worldToMap(pose.x(), pose.y(), point.x(), point.y());
        LOG_INFO << " x = " << costmap->GetSizeInCellsX() << " y = " << costmap->GetSizeInCellsY();
        if (costmap->GetCost(point.x(), point.y()) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          room_data.active_segment.push_back(pose);
        }
      }
    }
    room_data.times      = 1;
    room_data.block_coor = point_coor;
    room_datas.push_back(room_data);
  }
  LOG_INFO << "room_datas size = " << room_datas.size();
}

bool Block::getActiveSegmentPoints(const common::Pose& robot_pose, const common::Map& map,
                                   std::vector<common::Pose>& active_segment_points) {
  LOG_INFO << "enter getActiveSegmentPoints";
  LOG_INFO << "active_segment_points_ size = " << active_segment_points_.size();
  if (!active_segment_points_.empty()) {
    active_segment_points = active_segment_points_;
    return true;
  } else {
    return false;
  }
}

common::RoomData Block::getCurrentRoomData() {
  return current_runing_rooom_;
}


void Block::sortActiveSegmentPoints(const common::Pose& robot_pose, std::vector<common::Pose>& points) {
  std::map<float, common::Pose> points_temp;
  for (auto data : points) {
    float distance        = data.distanceTo(robot_pose);
    points_temp[distance] = data;
  }
  points.clear();
  for (auto data : points_temp) {
    points.push_back(data.second);
  }
}

}  // namespace block