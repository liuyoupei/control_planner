#ifndef CLEAN_ROBOT_CLEANACTIVEAPI_H
#define CLEAN_ROBOT_CLEANACTIVEAPI_H
#include "Event.h"
#include "map/costmap_2d.h"
#include "modules/chains/chains.h"
#include "modules/cover/cover_planner.h"
#include "modules/room/block.h"
#include "navigation/jps/jps_search.h"
#include "navigation/path_search.h"
#include "thrid_party/qp/qhsm.h"
#include <iostream>
namespace active_api {
class CleanActiveApi {
public:
  CleanActiveApi();
  ~CleanActiveApi();
  void reset();
  /***room**/
  void roomIntRoomData(const common::Map& map);
  bool roomGetNextRoom(const common::Pose& robot_pose, const std::shared_ptr<costmap_2d::Costmap2D>& costmap,
                       const common::Map& map, common::RoomData& room_data);
  bool roomGetActiveSegmentPoints(common::Pose& active_segment_point);
  common::RoomData roomGetCurrentRoomData() {
    return current_room_data_;
  };
  event::Type getVirtualLineType();
  /**chains**/
  bool                      chainsIsClose(const common::Pose& robot_pose, std::vector<common::Pose>& close_chains,
                                          common::ChainsCLoseResult& result, bool check_history_chains = false);
  void                      chainsSetMode(const common::ChainsMode& mode);
  bool                      haveRemnantChains(std::vector<common::Pose>& remnant_chains);
  /**cover**/
  bool generateStaticCoveragePlan(const event::Type& work_model);
  bool getNextWayPoint(common::CoverPoint& way_point);
  bool getNextNextWayPoint(common::CoverPoint& way_point);
  void updateSection();
  void setNextCoverPoint(const common::CoverPoint& next_cover_point) {
    next_cover_point_ = next_cover_point;
  };
  void coverFinish();
  bool isCoverFinish();
  event::Type getWorkModel() {
    return work_model_;
  };
  /**exporation*/
  bool getNotCover() {
    return not_cover_;
  };

private:
  block::Block                                                    block_;
  common::RoomData                                                current_room_data_;
  Chains                                                          chains_;
  CoverPlanner                                                    cover_planner_;
  event::Type                                                     work_model_;
  common::CoverPoint                                              next_cover_point_;
  std::map<common::Point, std::vector<std::vector<common::Pose>>> block_close_chains_;
  bool                                                            not_cover_;
};
}  // namespace active_api
#endif  // CLEAN_ROBOT_CLEANACTIVEAPI_H
