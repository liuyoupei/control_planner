#include "cleanActiveApi.h"
#include "sensor/sensor.h"
#include "common/common.h"
namespace active_api {
CleanActiveApi::CleanActiveApi() {
  not_cover_    = false;
}
CleanActiveApi::~CleanActiveApi() {}
void CleanActiveApi::roomIntRoomData(const common::Map& map) {
  LOG_INFO << "enter roomIntRoomData";
  Sensor::GetInstance()->updateCostmap(Sensor::GetInstance()->getMapData());
  auto costmap = Sensor::GetInstance()->getCostmap2d();
  block_.initRoomData(std::make_shared<costmap_2d::Costmap2D>(costmap), map);
}


void CleanActiveApi::reset() {
  LOG_INFO << "enter reset";
  chains_.reset();
  cover_planner_.reset();
  block_.reset();
  block_close_chains_.clear();
}

bool CleanActiveApi::roomGetNextRoom(const common::Pose&                           robot_pose,
                                     const std::shared_ptr<costmap_2d::Costmap2D>& costmap, const common::Map& map,
                                     common::RoomData& room_data) {
  LOG_INFO << "enter roomGetNextRoom";
  if (block_.getNextRoom(robot_pose, costmap, map, room_data)) {
    current_room_data_ = room_data;
    return true;
  } else {
    return false;
  }
}

bool CleanActiveApi::roomGetActiveSegmentPoints(common::Pose& active_segment_point) {
  LOG_INFO << "enter roomGetActiveSegmentPoints";
  std::vector<common::Pose> active_segment_points;
  auto                      map        = Sensor::GetInstance()->getMapData();
  auto                      robot_pose = Sensor::GetInstance()->getSlamPose();
  if (block_.getActiveSegmentPoints(robot_pose, map, active_segment_points)) {
    active_segment_point = active_segment_points.front();
    LOG_INFO << "active_segment_point x = " << active_segment_point.x() << " y = " << active_segment_point.y();
    return true;
  } else {
    return false;
  }
}

event::Type CleanActiveApi::getVirtualLineType() {
  if (work_model_ == event::Type::CLEAN_GLOBAL_CLEAN_ROOM || work_model_ == event::Type::CLEAN_SELECT_ROOM) {
    return event::Type::VIRTUAL_ROOM_LINE;
  }
}

bool CleanActiveApi::chainsIsClose(const common::Pose& robot_pose, std::vector<common::Pose>& close_chains,
                                   common::ChainsCLoseResult& result, bool check_history_chains) {
  chains_.updateChains(robot_pose);
  if (chains_.isCloseChains(robot_pose, close_chains, result, check_history_chains)) {
    if (result == common::ChainsCLoseResult::NORMAL_CHAINS_CLOSE) {
      std::vector<common::Pose> approxCurve;
      common::Rect<float>       close_chains_rect;
      close_chains_rect.update(close_chains);
      float area = close_chains_rect.getArea();
      LOG_INFO << "area = " << area;
      if (area <= 1.5) {
        close_chains_rect.expand(0.2);
        block_close_chains_[block_.getCurrentRoomData().block_coor].push_back(
          close_chains_rect.getAntiClockwiseVertices());
      } else {
        common::Common::GetInstance()->approxPolyDP(close_chains, approxCurve, 3, true);
        block_close_chains_[block_.getCurrentRoomData().block_coor].push_back(approxCurve);
      }
    }
    return true;
  } else {
    return false;
  }
}

void CleanActiveApi::chainsSetMode(const common::ChainsMode& mode) {
  chains_.setMode(mode);
}




bool CleanActiveApi::generateStaticCoveragePlan(const event::Type& work_model) {
  Sensor::GetInstance()->updateCostmap(Sensor::GetInstance()->getMapData());
  auto costmap    = Sensor::GetInstance()->getCostmap2d();
  auto robot_pose = Sensor::GetInstance()->getSlamPose();
  return cover_planner_.generateStaticCoveragePlan(std::make_shared<costmap_2d::Costmap2D>(costmap),
                                                   chains_.getClosechains(), block_.getCurrentRoomData().getVertices(),
                                                   robot_pose, work_model);
}

bool CleanActiveApi::getNextWayPoint(common::CoverPoint& way_point) {
  auto robot_pose = Sensor::GetInstance()->getSlamPose();
  return cover_planner_.getNextWayPoint(robot_pose, way_point);
}

bool CleanActiveApi::getNextNextWayPoint(common::CoverPoint& way_point) {
  return cover_planner_.getNextNextWayPoint(way_point);
}

void CleanActiveApi::updateSection() {
  common::CoverPoint way_point;
  getNextWayPoint(way_point);
  cover_planner_.updateSection(way_point.pose);
}




void CleanActiveApi::coverFinish() {
  chains_.resetChainsCoverFinish();
}

bool CleanActiveApi::isCoverFinish() {
  return cover_planner_.isCoverFinish();
}


bool CleanActiveApi::haveRemnantChains(std::vector<common::Pose>& remnant_chains) {
  remnant_chains = chains_.getRemnantChains(common::ChainsMode::COVER_MODE);
  if (remnant_chains.size() >= 15) {
    chains_.remnantChainsTrapping();
    return true;
  } else {
    return false;
  }
}

}  // namespace active_api