#ifndef CLEAN_COVER_PLANNER_H
#define CLEAN_COVER_PLANNER_H

#include "Event.h"
#include "navigation/path_search.h"
#include "polygon_common.h"
#include <gflags/gflags_declare.h>
#include <iostream>
#include <memory>
DECLARE_string(config_path);
typedef enum Order { FIRST_LINE_START = 0, FIRST_LINE_END, END_LINE_START, END_LINE_END } Order;
class CoverPlanner {
public:
  CoverPlanner();
  ~CoverPlanner() {
    if (polygon_) {
      delete polygon_;
      polygon_ = nullptr;
    }
  }
  bool generateStaticCoveragePlan(const std::shared_ptr<costmap_2d::Costmap2D>& costmap,
                                  const std::vector<common::Pose>&              chains_points,
                                  const std::vector<common::Pose>& block_points, const common::Pose& robot_pose,
                                  const event::Type& work_model);
  void reset();
  bool getNextWayPoint(const common::Pose& robot_pose, common::CoverPoint& way_point);
  bool getNextNextWayPoint(common::CoverPoint& way_point);
  void updateSection(const common::Pose& pose);
  bool isCoverFinish();

private:
  void init(const std::vector<common::Pose>& chains_points, const std::vector<common::Pose>& block_points,
            const int& work_model);
  std::vector<common::CoverPoint> getBestOrder(const common::Pose& robot_pose);
  void                            getWayPoints(std::vector<common::CoverPoint>& inters_points);
  std::vector<common::CoverPoint> pointToCoverPoint(std::vector<common::Pose>& inters_points);
  void                            getCoverPointType(std::vector<common::CoverPoint>& intersects);
  void                            judgmentTypeValidity(std::vector<common::CoverPoint>& intersects);
  void                            getCoverInfo(common::Rect<int>& polygon_rect);
  void                            setSectionId(int tag_num);
  int                             getSenctionId();
  bool                            getNearSectionFromComparePose(const common::Pose&                    compare_pose,
                                                                const std::vector<common::CoverPoint>& intersects_section, int& id,
                                                                float& distsance);
  void                            changeOrder(int id, std::vector<common::CoverPoint>& intersects_section);
  void                            getStaticCovertWithLineCoverX(const std::shared_ptr<costmap_2d::Costmap2D>& costmap,
                                                                const common::Rect<int>&                      polygon_rect);
  void                            getStaticCovertWithLineCoverY(const std::shared_ptr<costmap_2d::Costmap2D>& costmap,
                                                                const common::Rect<int>&                      polygon_rect);
  Polygon*                        polygon_;
  float                           cover_offset_;
  float                           map_resolution_;
  int                             cover_offser_index_;
  int                             section_id_;
  int                             last_tag_num_;
  int                             tag_id_;
  std::map<int, std::vector<common::CoverPoint>> intersects_section_;
  int                                            cover_dir_;
  float                                          near_goal_distance_;
  std::vector<common::CoverPoint>                way_points_;
  float                                          ap2p_max_angle_;
  float                                          ap2p_max_distance_;
  float                                          ap2p_min_distance_;
  int                                            static_covert_offset_;
  std::vector<common::Pose>                      update_section_pose_;
  std::vector<common::Pose>                      update_section_pose_history_;
};
#endif
