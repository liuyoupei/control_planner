#ifndef NAVIGATION_JPS_SEARCH_H_
#define NAVIGATION_JPS_SEARCH_H_

#include <algorithm>
#include <list>
#include "navigation/path_search.h"
namespace nav {
struct JpsNode {
  common::Point pt;
  int           F, G, H;  // F=G+H
  JpsNode*      parent;   // parent
  JpsNode(common::Point _pt) : pt(_pt), F(0), G(0), H(0), parent(NULL) {}

  common::Point getDirection() {
    if (parent == NULL)
      return common::Point(0, 0);
    // get direction from parent to current node
    int x = (pt.x() - parent->pt.x()) / std::max(abs(pt.x() - parent->pt.x()), 1);
    int y = (pt.y() - parent->pt.y()) / std::max(abs(pt.y() - parent->pt.y()), 1);
    return common::Point(x, y);
  }
};

class JpsSearch : public PathSearch {
public:
  JpsSearch();
   ~JpsSearch();
  void setMap(const std::shared_ptr<costmap_2d::Costmap2D>& map);
  bool findPath(const common::Pose& start_pose, const common::Pose& end_pose, std::vector<common::Pose>& path,
                bool find_replace = false);
  void setId(const int& id) {
    id_ = id;
  };
  int getId() {
    return id_;
  };

private:
  JpsNode* findPath(const JpsNode& startPoint, const JpsNode& endPoint);

  std::vector<common::Point> getNeighbourPoints(JpsNode* point) const;

  common::Point checkJumpPoint(const common::Point& targetpt, const common::Point& prept);

  JpsNode* isInList(const std::list<JpsNode*>& list, const JpsNode* point) const;

  bool isUnreachable(const int x, int y) const;

  JpsNode* getLeastFpoint();

  std::vector<std::vector<int>>          map_;
  int                                    cols_;
  int                                    rows_;
  common::Point                          finalpoint_;
  std::list<JpsNode*>                    openlist_;
  std::list<JpsNode*>                    closelist_;
  float                                  inflate_radius_;
  int                                    map_cnt_;
  std::shared_ptr<costmap_2d::Costmap2D> costmap_;
  int                                    id_;
};
}  // namespace nav
#endif
