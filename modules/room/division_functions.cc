#include "division_functions.h"
#include "thrid_party/glog/logging.h"
namespace room {
DivisionFunction::DivisionFunction() {}
DivisionFunction::~DivisionFunction() {}

void DivisionFunction::secondaryProcess(cv::Mat& mat) {
  cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::erode(mat, mat, element_erode);
  cv::Mat element_dialate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(mat, mat, element_dialate);
  cv::GaussianBlur(mat, mat, cv::Size(3, 3), 0, 0);
  cv::threshold(mat, mat, 80, 255, cv::THRESH_BINARY);
}

void DivisionFunction::searchKeyPoints(const cv::Mat& main_region, std::vector<cv::Point>& key_points) {
  int    max_corners   = 200;
  double quality_level = 0.3;
  double min_distance  = 3.0;
  int    block_size    = 3;
  bool   use_harris    = true;
  cv::goodFeaturesToTrack(main_region, key_points, max_corners, quality_level, min_distance, cv::Mat(), block_size,
                          use_harris);
}

void DivisionFunction::stretchKeyPointsAlongAxis(const cv::Mat& main_region, const std::vector<cv::Point>& key_points,
                                                 const DoorConfig& door_config, std::vector<Door>& candidate_doors) {
  for (auto corner : key_points) {
    cv::Point check_point;
    check_point = corner;
    for (int x_shift = -door_config.min_length; x_shift >= -door_config.max_length; x_shift--) {
      check_point = { corner.x + x_shift, corner.y };
      if (check_point.x < 0 || check_point.x >= main_region.cols || check_point.y < 0
          || check_point.y >= main_region.rows)
        break;
      if (main_region.at<uchar>(check_point) == 0)
        break;
    }
    if (std::abs(check_point.x - corner.x) > door_config.min_length
        && std::abs(check_point.x - corner.x) < door_config.max_length) {
      Door door;
      door.vertex_i = corner;
      door.vertex_j = check_point;
      door.valid    = true;
      candidate_doors.push_back(door);
    }
    check_point = corner;
    for (int x_shift = door_config.min_length; x_shift <= door_config.max_length; x_shift++) {
      check_point = { corner.x + x_shift, corner.y };
      if (check_point.x < 0 || check_point.x >= main_region.cols || check_point.y < 0
          || check_point.y >= main_region.rows)
        break;
      if (main_region.at<uchar>(check_point) == 0)
        break;
    }
    if (std::abs(check_point.x - corner.x) > door_config.min_length
        && std::abs(check_point.x - corner.x) < door_config.max_length) {
      Door door;
      door.vertex_i = corner;
      door.vertex_j = check_point;
      door.valid    = true;
      candidate_doors.push_back(door);
    }
    check_point = corner;
    for (int y_shift = -door_config.min_length; y_shift >= -door_config.max_length; y_shift--) {
      check_point = { corner.x, corner.y + y_shift };
      if (check_point.x < 0 || check_point.x >= main_region.cols || check_point.y < 0
          || check_point.y >= main_region.rows)
        break;
      if (main_region.at<uchar>(check_point) == 0)
        break;
    }
    if (std::abs(check_point.y - corner.y) > door_config.min_length
        && std::abs(check_point.y - corner.y) < door_config.max_length) {
      Door door;
      door.vertex_i = corner;
      door.vertex_j = check_point;
      door.valid    = true;
      candidate_doors.push_back(door);
    }
    check_point = corner;
    for (int y_shift = door_config.min_length; y_shift <= door_config.max_length; y_shift++) {
      check_point = { corner.x, corner.y + y_shift };
      if (check_point.x < 0 || check_point.x >= main_region.cols || check_point.y < 0
          || check_point.y >= main_region.rows)
        break;
      if (main_region.at<uchar>(check_point) == 0)
        break;
    }
    if (std::abs(check_point.y - corner.y) > door_config.min_length
        && std::abs(check_point.y - corner.y) < door_config.max_length) {
      Door door;
      door.vertex_i = corner;
      door.vertex_j = check_point;
      door.valid    = true;
      candidate_doors.push_back(door);
    }
  }
}

void DivisionFunction::checkCandidateDoors(const cv::Mat& main_region, const std::vector<Door>& input_doors,
                                           const DoorConfig& door_config, std::vector<Door>& output_doors) {
  int check_window      = 3;
  int invalid_threshold = 3;
  for (auto& door : input_doors) {
    // angle check
    double angle       = std::atan2(door.vertex_i.y - door.vertex_j.y, door.vertex_i.x - door.vertex_j.x);
    double check_angle = std::min(std::abs(angle), std::abs(angle - M_PI_2));
    if (check_angle > door_config.angle_delta) {
      continue;
    }
    double length = cv::norm(door.vertex_i - door.vertex_j);
    if (length < door_config.min_length || length > door_config.max_length) {
      continue;
    }
    int       invalid_count = 0;
    cv::Point door_center   = static_cast<cv::Point>((door.vertex_i + door.vertex_j) / 2);
    for (int x_shift = -check_window; x_shift <= check_window; x_shift++) {
      for (int y_shift = -check_window; y_shift <= check_window; y_shift++) {
        cv::Point check_point = { door_center.x + x_shift, door_center.y + y_shift };
        if (check_point.x < 0 || check_point.x >= main_region.cols || check_point.y < 0
            || check_point.y >= main_region.rows) {
          continue;
        }
        if (main_region.at<uchar>(check_point) == 0) {
          invalid_count++;
        }
      }
    }
    if (invalid_count > invalid_threshold) {
      continue;
    }

    output_doors.push_back(door);
  }
}

void DivisionFunction::filterDoors(const DoorConfig& door_config, std::vector<Door>& doors) {
  std::vector<std::vector<int>> door_clusters;
  std::vector<bool>             marked(doors.size(), false);
  for (int i = 0; i < doors.size(); i++) {
    if (marked[i]) {
      continue;
    }
    std::vector<int> index;
    index.push_back(i);
    marked[i] = true;
    for (int j = i + 1; j < doors.size(); j++) {
      if (marked[j]) {
        continue;
      }
      auto center_i = (doors[i].vertex_i + doors[i].vertex_j) / 2;
      auto center_j = (doors[j].vertex_i + doors[j].vertex_j) / 2;
      auto distance = cv::norm(center_i - center_j);
      if (distance < door_config.cluster_distance) {
        index.push_back(j);
        marked[j] = true;
      }
    }
    door_clusters.push_back(index);
  }
  for (const auto& cluster : door_clusters) {
    if (cluster.size() == 1) {
      doors[cluster.back()].valid = true;
      continue;
    }
    double min_length = 100 / 0.05;
    int    min_id     = -1;
    for (const auto& i : cluster) {
      doors[i].valid = false;
      auto distance  = cv::norm(doors[i].vertex_i - doors[i].vertex_j);
      if (distance < min_length) {
        min_length = distance;
        min_id     = i;
      }
    }
    if (min_id >= 0) {
      doors[min_id].valid = true;
    }
  }
}

void DivisionFunction::reviseDoorVertex(const cv::Mat& main_region, const DoorConfig& door_config,
                                        std::vector<Door>& all_doors) {
  std::vector<Door> doors;
  doors.swap(all_doors);
  int door_id = 0;
  for (auto door : doors) {
    if (door.valid == false) {
      continue;
    }

    reviseSingleDoorVertex(main_region, door_config, door);

    all_doors.push_back(door);
    all_doors.back().id = door_id;
    door_id++;
  }
}

void DivisionFunction::reviseSingleDoorVertex(const cv::Mat& main_region, const DoorConfig& door_config, Door& door) {
  cv::Point center = static_cast<cv::Point>((door.vertex_i + door.vertex_j) / 2);
  double    angle  = std::atan2(door.vertex_i.y - door.vertex_j.y, door.vertex_i.x - door.vertex_j.x);
  for (int length = door_config.min_length / 2; length < door_config.max_length / 2; length++) {
    cv::Point vertex_i = { center.x + static_cast<int>(length * std::cos(angle)),
                           center.y + static_cast<int>(length * std::sin(angle)) };
    if (main_region.at<uchar>(vertex_i) == 0 || main_region.at<uchar>(vertex_i) == 1) {
      door.vertex_i = vertex_i;
      break;
    }
  }
  for (int length = -door_config.min_length / 2; length > -door_config.max_length / 2; length--) {
    cv::Point vertex_j = { center.x + static_cast<int>(length * std::cos(angle)),
                           center.y + static_cast<int>(length * std::sin(angle)) };
    if (main_region.at<uchar>(vertex_j) == 0 || main_region.at<uchar>(vertex_j) == 1) {
      door.vertex_j = vertex_j;
      break;
    }
  }
}

void DivisionFunction::addToDoor(const std::vector<cv::Point>& double_contours, const Door& door,
                                 std::vector<Door>& door_data) {
  if (isSameOrderWithContour(double_contours, door)) {
    addByOrder(door, door_data, false);
  } else {
    addByOrder(door, door_data, true);
  }
}

bool DivisionFunction::isSameOrderWithContour(const std::vector<cv::Point>& double_contours, const Door& door) {
  int       v1_min_idx = -1;
  int       v2_min_idx = -1;
  double    min_dis    = 200.0;
  cv::Point v1         = static_cast<cv::Point>(door.vertex_i);
  cv::Point v2         = static_cast<cv::Point>(door.vertex_j);
  for (int i = 0; i < double_contours.size(); ++i) {
    auto dis = cv::norm(double_contours[i] - v1);
    if (dis < min_dis) {
      v1_min_idx = i;
      min_dis    = dis;
    }
  }
  if (v1_min_idx < 0) {
    return false;
  }

  min_dis = 200.0;
  for (int i = v1_min_idx; i < double_contours.size(); i++) {
    auto dis = cv::norm(double_contours[i] - v2);
    if (dis < min_dis) {
      v2_min_idx = i;
      min_dis    = dis;
    }
  }

  auto delta_idx = v2_min_idx - v1_min_idx;
  return delta_idx < (double_contours.size() / 4);
}

void DivisionFunction::addByOrder(const Door& door, std::vector<Door>& door_data, bool by_order) {
  if (by_order) {
    door_data.emplace_back(door);
    return;
  }
  door_data.emplace_back(Door{ door.vertex_j, door.vertex_i, door.id, door.valid });
}

std::pair<int, int> DivisionFunction::getRoomIdRange(const cv::Mat& mat, std::vector<int>& unnormal_room_ids) {
  double                       max_value = 0;
  double                       min_value = 256;
  int                          size      = mat.rows * mat.cols;
  std::unordered_map<int, int> room_size_map;
  for (int i = 0; i < size; i++) {
    auto value = mat.at<uchar>(i);
    if (value == 255 || value == 1) {
      continue;
    }
    room_size_map[value]++;
    if (value > max_value) {
      max_value = value;
    }
    if (value < min_value) {
      min_value = value;
    }
  }
  for (const auto& m : room_size_map) {
    if (m.second < 5) {
      unnormal_room_ids.push_back(m.first);
    }
  }
  return { min_value, max_value };
}

}  // namespace room