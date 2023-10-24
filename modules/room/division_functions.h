#ifndef CLEAN_DIVISION_FUNCTIONS_H
#define CLEAN_DIVISION_FUNCTIONS_H

#include <string>

#include "opencv2/opencv.hpp"

#include "common/common_type.h"
#include "common/enum_type.h"
#include "common/region_type.h"
#include "glog/logging.h"
#include <Eigen/Eigen>
namespace room {
class DivisionFunction {
public:
  DivisionFunction();
  ~DivisionFunction();
  void secondaryProcess(cv::Mat& mat);
  void searchKeyPoints(const cv::Mat& main_region, std::vector<cv::Point>& key_points);
  void stretchKeyPointsAlongAxis(const cv::Mat& main_region, const std::vector<cv::Point>& key_points,
                                 const DoorConfig& door_config, std::vector<Door>& candidate_doors);
  void checkCandidateDoors(const cv::Mat& main_region, const std::vector<Door>& input_doors,
                           const DoorConfig& door_config, std::vector<Door>& output_doors);
  void filterDoors(const DoorConfig& door_config, std::vector<Door>& doors);
  void reviseDoorVertex(const cv::Mat& main_region, const DoorConfig& door_config, std::vector<Door>& all_doors);
  void reviseSingleDoorVertex(const cv::Mat& main_region, const DoorConfig& door_config, Door& door);
  void addToDoor(const std::vector<cv::Point>& double_contours, const Door& door, std::vector<Door>& door_data);
  bool isSameOrderWithContour(const std::vector<cv::Point>& double_contours, const Door& door);
  void addByOrder(const Door& door, std::vector<Door>& door_data, bool by_order);
  std::pair<int, int> getRoomIdRange(const cv::Mat& mat, std::vector<int>& unnormal_room_ids);
};

}  // namespace room

#endif