#ifndef CLEAN_ROBOT_CHAINS_H
#define CLEAN_ROBOT_CHAINS_H
#include "common/common_type.h"
#include "common/enum_type.h"
#include "map/costmap_2d.h"
#include <gflags/gflags_declare.h>
#include <iostream>
#include <map>
DECLARE_string(config_path);
class Chains {
public:
  Chains();
  ~Chains();
  void                      updateChains(const common::Pose& robot_pose);
  bool                      isCloseChains(const common::Pose& robot_pose, std::vector<common::Pose>& close_chains,
                                          common::ChainsCLoseResult& result, bool check_history_chains = false);
  void                      reset();
  void                      resetChainsCoverFinish();
  void                      remnantChainsTrapping();
  void                      setMode(const common::ChainsMode& mode);
  std::vector<common::Pose> getClosechains();
  std::vector<common::Pose> getRemnantChains(const common::ChainsMode& mode);
  bool                      isHistoryChains(const common::Pose& robot_pose);
private:
  void setHistoryChains(const std::vector<common::Pose>& close_chians);
  void setHistoryIsolatedIslandCloseChains(const std::vector<common::Pose>& close_chians);

private:
  std::vector<common::Pose>                               current_chains_;
  std::vector<common::Pose>                               close_chains_;
  std::map<common::ChainsMode, std::vector<common::Pose>> remnant_chains_;
  common::Pose                                            last_pose_;
  float                                                   chains_update_distance_;
  float                                                   chains_close_distance_;
  common::ChainsMode                                      chains_mode_;
  std::vector<common::Pose>                               history_chains_;
  int                                                     history_chians_num_;
  common::Pose                                            find_history_chains_last_pose_;
  std::vector<std::vector<common::Pose>>                  history_isolated_island_chians_;
};
#endif  // CLEAN_ROBOT_CHAINS_H
