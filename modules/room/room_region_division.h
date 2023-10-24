#ifndef REGION_DIVISION_H
#define REGION_DIVISION_H

#include <string>

#include "common/common_type.h"
#include "common/enum_type.h"
#include "common/region_type.h"
#include "division_functions.h"
#include <Eigen/Eigen>
DECLARE_string(config_path);
namespace room {

class RoomRegionDivision {
public:
  RoomRegionDivision();
  ~RoomRegionDivision();

  bool setMap(const std::shared_ptr<common::Map>& map);
  void setMapResolution(const double& resolution);
  int  process();
  void getRegionData(std::unordered_map<int, RegionData>& region_datas);
  void                         resetState();

private:
  DivisionFunction division_function_;
  bool             room_is_divided_;
  cv::Mat          origin_data_;

  cv::Mat origin_roi_;
  cv::Mat origin_revised_;
  cv::Mat main_contour_;
  cv::Mat divided_region_;

  int ratio_;
  int room_id_offset_;
  double resolution_;
  DoorConfig                                    door_config_;
  std::vector<Door>                             doors_;
  std::vector<Door>                             world_doors_;
  std::unordered_map<int, RegionData>           region_datas_;
  std::unordered_map<int, RegionData>           world_region_datas_;
  std::unordered_map<common::PixelMapType, int> pixel_map_;
  std::vector<RoomId>                           room_ids_;
  std::pair<int, int>                           room_id_range_;
  std::vector<int>                              unnormal_room_ids_;

  std::shared_ptr<common::Map> map_;
  std::shared_ptr<common::Map> room_map_;

private:
  void                       getROI();
  void                       preProcessing(bool secondary_process);
  void                       generateRegion();
  void                       computeRegionData();
  void                       generateCandidateDoors(const cv::Mat& main_region, std::vector<cv::Point>& key_points);
  void                       makeRegions(const cv::Mat& main_contour);
  void                       convertToWorld(const std::shared_ptr<common::Map>& map);
  void                       generateColoredMap();
  std::shared_ptr<cv::Mat1b> convertToCVMat(const std::shared_ptr<common::Map>& map);
  cv::Point2f                imagePtToMap(const cv::Point& image_pt, const common::Map_info& map_info);
};
}  // namespace room
#endif
