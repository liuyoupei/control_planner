#ifndef REGION_DIVISION_REGION_TYPE_H_
#define REGION_DIVISION_REGION_TYPE_H_

#include <vector>

#include "opencv2/opencv.hpp"

namespace room {

struct Door {
  cv::Point2f vertex_i;
  cv::Point2f vertex_j;
  int         id;
  bool        valid;
};

struct RegionData {
  std::vector<cv::Point2f> rooms;
  std::vector<Door>        doors;
};

struct RoomId {
  int    room_id;
  double room_area;
};

struct DoorConfig {
  float min_length;
  float max_length;
  float angle_delta;
  float cluster_distance;
  float min_area;
};
}  // namespace room

#endif