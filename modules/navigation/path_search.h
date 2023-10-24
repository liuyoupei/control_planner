#ifndef NAVIGATION_PATH_SEARCH_H_
#define NAVIGATION_PATH_SEARCH_H_

#include "common/common_type.h"
#include "map/costmap_2d.h"
#include <iostream>
namespace nav {

class PathSearch {
public:
  PathSearch(){};

   ~PathSearch(){};

  virtual void setMap(const std::shared_ptr<costmap_2d::Costmap2D>& map) = 0;
  virtual bool findPath(const common::Pose& start_pose, const common::Pose& end_pose, std::vector<common::Pose>& path,
                        bool find_replace = false)                       = 0;
  virtual void setId(const int& id)                                      = 0;
  virtual int  getId()                                                   = 0;

protected:
  bool findReplacePose(const common::Pose& pose, common::Pose& replace_pose, const std::vector<std::vector<int>>& map,
                       int cols, int rows);
  bool isInMap(const common::Point& point, int cols, int rows);
};
}  // namespace nav

#endif