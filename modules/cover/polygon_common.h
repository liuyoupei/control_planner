#ifndef CLEAN_POLYGON_COMMON_H
#define CLEAN_POLYGON_COMMON_H
#include "common/common_type.h"
#include "map/costmap_2d.h"
#include <algorithm>
#include <iostream>
#include <map>
#include <vector>
class Polygon {
public:
  Polygon(const std::vector<common::Pose>& points);
  bool getRectFromPolygon(common::Rect<int>& rect);
  int  intersectWithLineCoverX(const std::shared_ptr<costmap_2d::Costmap2D>& costmap, int x,
                               std::vector<common::Pose>& intersects);
  int  intersectWithLineCoverY(const std::shared_ptr<costmap_2d::Costmap2D>& costmap, int y,
                               std::vector<common::Pose>& intersects);
  bool isInPolygon(const std::vector<common::Pose>& poses, const common::Pose& pose);

private:
  std::vector<common::Pose> points_;
  common::Rect<int>         polygon_rect_;
  float                     shrink_distance_sub_;
  float                     shrink_distance_add_;
  float                     remove_intersects_distance_;
};
#endif  // UNTITLED_POLYGON_COMMON_H
