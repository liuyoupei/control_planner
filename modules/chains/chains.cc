#include "chains.h"
#include "common/common.h"
#include "sensor/sensor.h"
#include "thrid_party/glog/logging.h"
#include <yaml-cpp/yaml.h>
Chains::Chains() {
  YAML::Node config       = YAML::LoadFile(FLAGS_config_path);
  chains_update_distance_ = config["chains_update_distance"].as<float>();
  chains_close_distance_  = config["chains_close_distance"].as<float>();
  LOG_INFO << " chains_update_distance_ = " << chains_update_distance_;
  LOG_INFO << " chains_close_distance_ = " << chains_close_distance_;
  history_chians_num_ = 0;
}

Chains::~Chains() {}

void Chains::reset() {
  LOG_INFO << "enter reset";
  current_chains_.clear();
  close_chains_.clear();
  remnant_chains_.clear();
  history_chains_.clear();
  history_isolated_island_chians_.clear();
  history_chians_num_ = 0;
}

void Chains::resetChainsCoverFinish() {
  LOG_INFO << "enter resetChainsCoverFinish";
  close_chains_.clear();
  current_chains_.clear();
  remnant_chains_.clear();
  history_chains_.clear();
  history_chians_num_ = 0;
}

void Chains::remnantChainsTrapping() {
  LOG_INFO << "enter remnantChainsTrapping";
  history_chains_.clear();
  history_chians_num_ = 0;
  close_chains_.clear();
  current_chains_ = remnant_chains_[common::ChainsMode::COVER_MODE];
  LOG_INFO << "current_chains_ size = " << current_chains_.size();
}

void Chains::setMode(const common::ChainsMode& mode) {
  LOG_INFO << "enter setMode";
  chains_mode_ = mode;
  remnant_chains_[mode].clear();
}

bool Chains::isCloseChains(const common::Pose& robot_pose, std::vector<common::Pose>& close_chains,
                           common::ChainsCLoseResult& result, bool check_history_chains) {
  if (current_chains_.size() <= 5) {
    return false;
  }
  if (check_history_chains == true && isHistoryChains(robot_pose)) {
    LOG_INFO << "is history chains";
    close_chains = current_chains_;
    LOG_INFO << "close chains size = " << close_chains.size();
    if (close_chains.size() <= 10) {
      common::Rect<float> close_chains_rect;
      close_chains_rect.update(close_chains);
      close_chains_rect.expand(0.2);
      close_chains = close_chains_rect.getClockwiseVertices();
    }
    result = common::ChainsCLoseResult::HISTORY_CHAINS;
    return true;
  }
  for (int i = 0; i < current_chains_.size() - 5; i++) {
    float distance = current_chains_[i].distanceTo(robot_pose);
    float angle    = current_chains_[i].distancePhiTo(robot_pose);
    if (distance < chains_close_distance_ && angle <= M_PI_4) {
      LOG_INFO << "chains is close";
      LOG_INFO << "distance = " << distance << " angle = " << angle
               << " current_chains_[i] phi = " << current_chains_[i].phi() << " robot phi = " << robot_pose.phi();
      LOG_INFO << "current size = " << current_chains_.size();
      LOG_INFO << " close index = " << i;
      close_chains_.clear();
      remnant_chains_[chains_mode_].assign(current_chains_.begin(), current_chains_.begin() + i);
      close_chains_.assign(current_chains_.begin() + i, current_chains_.end());
      current_chains_.clear();
      LOG_INFO << "remnant_chains_ size = " << remnant_chains_[chains_mode_].size();
      if (remnant_chains_[chains_mode_].size() <= 5) {
        remnant_chains_[chains_mode_].clear();
      }
      close_chains = close_chains_;
      setHistoryChains(close_chains_);
      common::Rect<float> close_chains_rect;
      close_chains_rect.update(close_chains_);
      bool is_anti_clockwise = !common::Common::GetInstance()->isClockwise(close_chains);
      if (is_anti_clockwise == true) {
        float area = close_chains_rect.getArea();
        LOG_INFO << "area = " << area;
        if (area <= 0.01) {
          current_chains_ = remnant_chains_[chains_mode_];
          remnant_chains_[chains_mode_].clear();
          return false;
        } else {
          result = common::ChainsCLoseResult::NORMAL_CHAINS_CLOSE;
        }
        setHistoryIsolatedIslandCloseChains(close_chains);
      } else {
        if (close_chains_.size() <= 15) {
          current_chains_ = remnant_chains_[chains_mode_];
          remnant_chains_[chains_mode_].clear();
          return false;
        } else if (remnant_chains_.size() >= 10) {
          result = common::ChainsCLoseResult::SMALL_CHAINS_CLOSE;
        } else {
          result = common::ChainsCLoseResult::ISOLATED_ISLAND_CHAINS;
          return true;
        }
      }
      LOG_INFO << " is_anti_clockwise = " << is_anti_clockwise;
      LOG_INFO << "close_chains_ size = " << close_chains_.size();
      LOG_INFO << "erase current_chains_ size = " << current_chains_.size();
      LOG_INFO << " result = " << result;
      return true;
    }
  }
  return false;
}

void Chains::updateChains(const common::Pose& robot_pose) {
  if (robot_pose.distanceTo(last_pose_) > chains_update_distance_) {
    last_pose_ = robot_pose;
    current_chains_.push_back(robot_pose);
  }
}

std::vector<common::Pose> Chains::getClosechains() {
  LOG_INFO << "enter getClosechains";
  return close_chains_;
}


std::vector<common::Pose> Chains::getRemnantChains(const common::ChainsMode& mode) {
  LOG_INFO << "enter getRemnantChains";
  LOG_INFO << "mdoe = " << mode << " size = " << remnant_chains_[mode].size();
  return remnant_chains_[mode];
}

bool Chains::isHistoryChains(const common::Pose& robot_pose) {
  if (history_chains_.empty()) {
    return false;
  }
  if (find_history_chains_last_pose_.distanceTo(robot_pose) <= 0.05) {
    return false;
  }
  find_history_chains_last_pose_ = robot_pose;
  for (auto chains : history_chains_) {
    if (chains.distanceTo(robot_pose) <= 0.1
        && chains.distancePhiTo(robot_pose) <= common::Common::GetInstance()->DEG2RAD(40)) {
      LOG_INFO << "is history chains";
      history_chians_num_++;
      if (history_chians_num_ >= 3) {
        history_chians_num_ = 0;
        return true;
      } else {
        return false;
      }
    }
  }
  history_chians_num_ = 0;
  return false;
}

void Chains::setHistoryChains(const std::vector<common::Pose>& close_chians) {
  LOG_INFO << "enter setHistoryChains";
  LOG_INFO << "history_chains_ size = " << history_chains_.size();
  history_chains_.insert(history_chains_.end(), close_chians.begin(), close_chians.end());
  LOG_INFO << "history_chains_ size = " << history_chains_.size();
}

void Chains::setHistoryIsolatedIslandCloseChains(const std::vector<common::Pose>& close_chians) {
  LOG_INFO << "enter setHistoryIsolatedIslandCloseChains";
  LOG_INFO << "history_isolated_island_chians_ size = " << history_isolated_island_chians_.size();
  std::vector<common::Pose> polygon_chains_points;
  common::Common::GetInstance()->approxPolyDP(close_chians, polygon_chains_points, 1, false);
  history_isolated_island_chians_.push_back(polygon_chains_points);
}
