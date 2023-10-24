#ifndef CLEAN_ROBOT_BLOCK_H
#define CLEAN_ROBOT_BLOCK_H
#include "Event.h"
#include "common/common_type.h"
#include "common/region_type.h"
#include "map/costmap_2d.h"
#include "room_region_division.h"
#include <iostream>
namespace block {
class Block {
public:
  Block();
  ~Block();
  void reset();
  void             initRoomData(const std::shared_ptr<costmap_2d::Costmap2D>& costmap, const common::Map& map);
  bool             getNextRoom(const common::Pose& robot_pose, const std::shared_ptr<costmap_2d::Costmap2D>& costmap,
                               const common::Map& map, common::RoomData& room_data);
  bool             getActiveSegmentPoints(const common::Pose& robot_pose, const common::Map& map,
                                          std::vector<common::Pose>& active_segment_points);
  common::RoomData getCurrentRoomData();
private:
  void regionDataToRoomData(const std::shared_ptr<costmap_2d::Costmap2D>&    costmap,
                            const std::unordered_map<int, room::RegionData>& region_data,
                            std::vector<common::RoomData>&                   room_datas);
  bool getNextRoomData(const std::shared_ptr<costmap_2d::Costmap2D>& costmap, const common::Pose& robot_pose,
                       common::RoomData& room_data);
  void          sortActiveSegmentPoints(const common::Pose& robot_pose, std::vector<common::Pose>& points);
private:
  std::string                                                                         debug_image_path_;
  room::RoomRegionDivision                                                            room_region_division_;
  std::vector<common::RoomData>                                                       room_datas_;
  std::unordered_map<int, room::RegionData>                                           region_datas_;
  common::RoomData                                                                    current_runing_rooom_;
  std::vector<common::Pose>                                                           active_segment_points_;
  std::map<common::Point, std::map<common::BlockEdgeType, std::vector<common::Pose>>> active_segment_pose_map_;
  std::map<common::Point, common::RoomData>                                           all_room_data_;
};
}  // namespace block
#endif  // CLEAN_ROBOT_BLOCK_H
