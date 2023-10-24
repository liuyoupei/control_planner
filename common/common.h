#ifndef CLEAN_ROBOT_COMMON_H
#define CLEAN_ROBOT_COMMON_H
#include "common_type.h"
#include "map/costmap_2d.h"
#include <iostream>
#include <memory>
#include <mutex>
namespace common {
class Common {
public:
  Common();
  ~Common();
  static std::shared_ptr<Common> GetInstance() {
    if (Common_ptr_ == nullptr) {
      std::lock_guard<std::mutex> lk(Common_mutex_);
      if (Common_ptr_ == nullptr) {
        Common_ptr_ = std::shared_ptr<Common>(new Common());
      }
    }
    return Common_ptr_;
  }
  float  fpwrappi(float angle);
  double DEG2RAD(const double x) {
    return x * M_PI / 180.0;
  }
  std::vector<Pose> raytrace(const Pose& start_pose, const Pose& end_pose);
  Pose              findFootOfLine(const Pose& pt, const Pose& begin, const Pose& end);
  float             getDiffAngle(const common::Pose& current_pose, const common::Pose& goal_pose);
  void   approxPolyDP(const std::vector<Pose>& curve, std::vector<Pose>& approxCurve, double epsilon, bool closed);
  bool   isClockwise(const std::vector<common::Pose>& data);
  float getPathDistance(const std::shared_ptr<costmap_2d::Costmap2D>& costmap, const common::Pose& start,
                        const common::Pose& goal);

private:
  static std::shared_ptr<Common> Common_ptr_;
  static std::mutex              Common_mutex_;
  float                          go_forward_distance_;
};
}  // namespace common
#endif  // CLEAN_ROBOT_COMMON_H
