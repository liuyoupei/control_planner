#include "room_region_division.h"
#include "sensor/sensor.h"
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
using namespace cv;
namespace room {

RoomRegionDivision::RoomRegionDivision() {
  YAML::Node config        = YAML::LoadFile(FLAGS_config_path);
  room_is_divided_ = config["room_is_divided"].as<bool>();
  ratio_           = config["ratio"].as<float>();
  resolution_      = config["resolution"].as<float>();

  door_config_.min_length =
    config["room_door_min_length"].as<float>() / resolution_;
  door_config_.max_length =
    config["room_door_max_length"].as<float>() / resolution_;
  door_config_.angle_delta      = M_PI / config["room_door_angle_delta"].as<float>();
  door_config_.cluster_distance = config["room_door_cluster_distance"].as<float>() / resolution_;
  door_config_.min_area         = config["room_door_min_area"].as<float>() / (resolution_ * resolution_);

  map_      = nullptr;
  room_map_ = nullptr;

  room_id_offset_ = 1;
  pixel_map_[common::PixelMapType::OBSTACLE] = 0;
  pixel_map_[common::PixelMapType::UNKNOWN]  = 64;
  pixel_map_[common::PixelMapType::FREE]     = 128;
}

RoomRegionDivision::~RoomRegionDivision() {}

int RoomRegionDivision::process() {
  LOG_INFO << "enter process ";
  resetState();
  generateRegion();
  computeRegionData();
  convertToWorld(map_);
  generateColoredMap();
  room_is_divided_ = true;
  return 0;
}

bool RoomRegionDivision::setMap(const std::shared_ptr<common::Map>& map) {
  if (map == nullptr) {
    return false;
  }
  map_           = std::make_shared<common::Map>(*map);
  auto image_map = convertToCVMat(map_);
  if (image_map == nullptr) {
    return false;
  }
  setMapResolution(map_->info.resolution);
  origin_data_ = *image_map;

  getROI();
  preProcessing(true);

  return true;
}

void RoomRegionDivision::setMapResolution(const double& resolution) {
  if (abs(resolution - resolution_) > 1e-6) {
    door_config_.min_length       = door_config_.min_length * resolution_ / resolution;
    door_config_.max_length       = door_config_.max_length * resolution_ / resolution;
    door_config_.cluster_distance = door_config_.cluster_distance * resolution_ / resolution;
    door_config_.min_area         = door_config_.min_area * (resolution_ * resolution_) / (resolution * resolution);

    resolution_ = resolution;
  }
}

void RoomRegionDivision::getROI() {
  origin_roi_ = origin_data_.clone();
}

void RoomRegionDivision::preProcessing(bool secondary_process) {
  cv::Mat binary_first;
  cv::threshold(origin_roi_, binary_first, common::PixelMapType::UNKNOWN * ratio_, 255, cv::THRESH_BINARY);

  if (secondary_process) {
    division_function_.secondaryProcess(binary_first);
  }

  origin_revised_ = binary_first.clone();
}

void RoomRegionDivision::generateRegion() {
  LOG_INFO << "enter generateRegion ";
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(origin_revised_, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  LOG_INFO << "contours size = " << contours.size();
  double       area_max           = std::numeric_limits<double>::min();
  unsigned int main_contour_index = 0;
  for (unsigned int i = 0; i < contours.size(); i++) {
    double area = cv::contourArea(contours[i]);
    if (area > area_max) {
      area_max           = area;
      main_contour_index = i;
    }
  }
  cv::Mat main_region = cv::Mat::zeros(origin_revised_.size(), origin_revised_.depth());

  cv::drawContours(main_region, contours, main_contour_index, common::PixelMapType::FREE * ratio_, cv::FILLED);
  std::vector<cv::Point> key_points;
  division_function_.searchKeyPoints(main_region, key_points);
  generateCandidateDoors(main_region, key_points);
  main_contour_ = cv::Mat::zeros(origin_revised_.size(), origin_revised_.depth());
  cv::drawContours(main_contour_, contours, main_contour_index, 255);

  makeRegions(main_contour_);
  if (!unnormal_room_ids_.empty()) {
    for (const auto& id : unnormal_room_ids_) {
      for (int i = 0; i < divided_region_.size().area(); i++) {
        if (divided_region_.at<uchar>(i) == id) {
          main_contour_.at<uchar>(i) = 255;
        }
      }
    }
    makeRegions(main_contour_);
  }
}

void RoomRegionDivision::generateCandidateDoors(const cv::Mat& main_region, std::vector<cv::Point>& key_points) {
  LOG_INFO << "enter generateCandidateDoors";
  doors_.clear();
  std::vector<Door> candidate_doors;
  for (unsigned int i = 0; i < key_points.size(); i++) {
    for (unsigned int j = i + 1; j < key_points.size(); j++) {
      Door door;
      door.vertex_i = key_points.at(i);
      door.vertex_j = key_points.at(j);
      door.valid    = true;
      candidate_doors.push_back(door);
    }
  }
  LOG_INFO << "candidate_doors size = " << candidate_doors.size();
  division_function_.stretchKeyPointsAlongAxis(main_region, key_points, door_config_, candidate_doors);
  division_function_.checkCandidateDoors(main_region, candidate_doors, door_config_, doors_);
  division_function_.filterDoors(door_config_, doors_);
  division_function_.reviseDoorVertex(main_region, door_config_, doors_);
  LOG_INFO << "door size: " << doors_.size();
}

void RoomRegionDivision::makeRegions(const cv::Mat& original_contour) {
  cv::Mat main_contour = original_contour.clone();
  for (auto door : doors_) {
    cv::line(main_contour, door.vertex_i, door.vertex_j, 255);
  }
  LOG_INFO << "door size: " << doors_.size();

  int fill_value = 0;
  while (true) {
    cv::Point seed = { -1, -1 };
    for (int row = 0; row < main_contour.rows; row++) {
      for (int col = 0; col < main_contour.cols; col++) {
        if (main_contour.at<uchar>(row, col) == 0) {
          seed.y = row;
          seed.x = col;
          break;
        }
      }
      if (seed.x != -1 && seed.y != -1)
        break;
    }
    if (seed.x != -1 && seed.y != -1) {
      cv::floodFill(main_contour, seed, fill_value);
      fill_value++;
    } else {
      break;
    }
  }
  unnormal_room_ids_.clear();
  room_id_range_ = division_function_.getRoomIdRange(main_contour, unnormal_room_ids_);
  room_id_range_.first += room_id_offset_;
  room_id_range_.second += room_id_offset_;
  divided_region_ = main_contour.clone();
}

void RoomRegionDivision::resetState() {
  LOG_INFO << "enter resetState";
  room_is_divided_ = false;
  region_datas_.clear();
}

void RoomRegionDivision::computeRegionData() {
  for (int room_id = room_id_range_.first; room_id <= room_id_range_.second; ++room_id) {
    auto    tmp_img_expr = divided_region_ == room_id - room_id_offset_;
    cv::Mat tmp_img      = tmp_img_expr;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(tmp_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
      continue;
    }
    if (contours[0].empty()) {
      continue;
    }
    std::vector<cv::Point> double_contours;
    double_contours.insert(double_contours.end(), contours[0].begin(), contours[0].end());
    double_contours.insert(double_contours.end(), contours[0].begin(), contours[0].end());
    std::vector<Door> door_data;
    for (const auto& door : doors_) {
      auto door_mid = (door.vertex_i + door.vertex_j) / 2;
      auto d        = cv::pointPolygonTest(contours[0], door_mid, true);
      if (abs(d) < 4.0) {
        division_function_.addToDoor(double_contours, door, door_data);
      }
    }

    if (door_data.empty()) {
      continue;
    }

    std::vector<cv::Point2f> room_data;
    room_data.insert(room_data.end(), contours[0].begin(), contours[0].end());
    if (!door_data.empty() && !room_data.empty()) {
      auto area              = cv::contourArea(room_data, false);
      region_datas_[room_id] = RegionData{ room_data, door_data };
      room_ids_.push_back({ room_id, area });
    }
  }
  std::sort(room_ids_.begin(), room_ids_.end(),
            [](const RoomId& a, const RoomId& b) { return a.room_area > b.room_area; });
  LOG_INFO << "region_datas_ size = " << region_datas_.size();
  if (region_datas_.empty()) {
    LOG_INFO << "No regions with doors found, maybe only one room!";

    for (int room_id = room_id_range_.first; room_id <= room_id_range_.second; ++room_id) {
      auto    tmp_img_expr = divided_region_ == room_id - room_id_offset_;
      cv::Mat tmp_img      = tmp_img_expr;

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(tmp_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      if (contours.empty()) {
        continue;
      }
      if (contours[0].empty()) {
        continue;
      }

      std::vector<cv::Point2f> room_data;
      room_data.insert(room_data.end(), contours[0].begin(), contours[0].end());
      if (!room_data.empty()) {
        auto area = cv::contourArea(room_data, false);
        room_ids_.push_back({ room_id, area });
      }
    }

    std::sort(room_ids_.begin(), room_ids_.end(),
              [](const RoomId& a, const RoomId& b) { return a.room_area > b.room_area; });

    auto                                tmp_img_expr = divided_region_ == room_ids_[0].room_id - room_id_offset_;
    cv::Mat                             tmp_img      = tmp_img_expr;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(tmp_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
      return;
    }
    if (contours[0].empty()) {
      LOG_INFO << "cannot find countours, something is wrong!!!";
      return;
    }
    std::vector<cv::Point2f> room_data;
    std::vector<Door>        doors;
    room_data.insert(room_data.end(), contours[0].begin(), contours[0].end());
    region_datas_[room_ids_[0].room_id] = RegionData{ room_data, doors };
    doors_.resize(0);
  }
}

void RoomRegionDivision::generateColoredMap() {
  if (map_ == nullptr) {
    return;
  }

  room_map_ = std::make_shared<common::Map>();
  room_map_->data.resize(map_->data.size());
  room_map_->header.frame_id = "room map";
  room_map_->info            = map_->info;

  cv::Mat region_mat = cv::Mat::zeros(divided_region_.size(), CV_8UC1);

  for (int i = 0; i < divided_region_.size().area(); i++) {
    if (divided_region_.at<uchar>(i) != 255) {
      region_mat.at<uchar>(i) = divided_region_.at<uchar>(i);
    }
  }

  cv::Mat element_dialate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(region_mat, region_mat, element_dialate);
  for (int i = 0; i < region_mat.size().area(); i++) {
    if (region_mat.at<uchar>(i) >= 2) {
      continue;
    }
    if (origin_data_.at<uchar>(i) == pixel_map_[common::PixelMapType::OBSTACLE]) {
      region_mat.at<uchar>(i) = 255;
    } else if (origin_data_.at<uchar>(i) == pixel_map_[common::PixelMapType::UNKNOWN]) {
      region_mat.at<uchar>(i) = 1;
    }
  }

  for (int i = 0; i < room_map_->data.size(); i++) {
    switch (region_mat.at<uchar>(i)) {
      case 1:
        room_map_->data[i] = 0;
        break;
      case 255:
        room_map_->data[i] = 1;
        break;
      default:
        room_map_->data[i] = region_mat.at<uchar>(i) + room_id_offset_;
        break;
    }
  }
}

void RoomRegionDivision::convertToWorld(const std::shared_ptr<common::Map>& map) {
  LOG_INFO << "enter convertToWorld";
  if (map == nullptr) {
    LOG_INFO << "map is null";
    return;
  }
  world_region_datas_.clear();
  for (const auto& region_data : region_datas_) {
    LOG_INFO << "region_data id = " << region_data.first;
    world_region_datas_[region_data.first] = RegionData();
    for (const auto& room : region_data.second.rooms) {
      world_region_datas_[region_data.first].rooms.emplace_back(imagePtToMap(room, map->info));
    }
    LOG_INFO << "door size = " << region_data.second.doors.size();
    for (const auto& door : region_data.second.doors) {
      world_region_datas_[region_data.first].doors.emplace_back(
        Door{ imagePtToMap(door.vertex_i, map->info), imagePtToMap(door.vertex_j, map->info), door.id, door.valid });
    }
  }

  world_doors_.clear();
  for (const auto& door : doors_) {
    world_doors_.emplace_back(Door());
    world_doors_.back().vertex_i = imagePtToMap(door.vertex_i, map->info);
    world_doors_.back().vertex_j = imagePtToMap(door.vertex_j, map->info);
    world_doors_.back().id       = door.id;
    world_doors_.back().valid    = door.valid;
  }
}

void RoomRegionDivision::getRegionData(std::unordered_map<int, RegionData>& region_datas) {
  region_datas.clear();
  if (!room_is_divided_) {
    return;
  }
  region_datas = world_region_datas_;
}

std::shared_ptr<cv::Mat1b> RoomRegionDivision::convertToCVMat(const std::shared_ptr<common::Map>& map) {
  if (map == nullptr) {
    return nullptr;
  }
  int width  = map->info.width;
  int height = map->info.height;

  if (width == 0 || height == 0) {
    return nullptr;
  }
  auto opencv_map = std::make_shared<cv::Mat1b>(height, width);
  CHECK(width * height == map->data.size());
  for (int i = 0; i < map->data.size(); ++i) {
    int value = map->data[i];
    if (value == -1) {
      opencv_map->at<uchar>(i) = 64;
    } else if (value <= 40) {
      opencv_map->at<uchar>(i) = 128;
    } else {
      opencv_map->at<uchar>(i) = 0;
    }
  }
  return opencv_map;
}

cv::Point2f RoomRegionDivision::imagePtToMap(const cv::Point& image_pt, const common::Map_info& map_info) {
  cv::Point2f point;
  point.x = image_pt.x * map_info.resolution + map_info.origin.x();
  point.y = image_pt.y * map_info.resolution + map_info.origin.y();
  return point;
}
}  // namespace room
