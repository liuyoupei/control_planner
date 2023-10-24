#include "cleanActive.h"
#include "navigation/path_search.h"
#include "sensor/sensor.h"
#include "thrid_party/glog/logging.h"
#include <common/common_type.h>
#include <common/common.h>
#include <thread>
using namespace event;
using namespace qp;
using namespace std;
using QState = qp::QHsm::QState;
std::shared_ptr<CleanActive> CleanActive::create(std::shared_ptr<qp::Qf> qf) {
  return std::make_shared<CleanActive>(qf);
}
void CleanActive::initial(std::shared_ptr<qp::QEvent> e) {
  LOG_INFO << " CleanActive init";
  subscribe(Type::CLEAN_GLOBAL_CLEAN_ROOM);
  subscribe(Type::VIRTUAL_ROOM_LP2P_GOAL);
  subscribe(Type::CONTROL_NP2P_FINISH);
  subscribe(Type::CONTROL_LP2P_FINISH);
  subscribe(Type::CONTROL_AP2P_FINISH);
  subscribe(Type::CONTROL_STOP_FINISH);
  subscribe(Type::CONTROL_FIND_PATH_FINSIH);
  subscribe(Type::TERMINATE);
  sync_event_response_handler_ = SyncEventResponseHandler::create();
  Q_INIT(&CleanActive::running);
}

qp::QSTATE CleanActive::running(std::shared_ptr<qp::QEvent> e) {
  if (handle(e)) {
    return nullptr;
  }
  switch (e->sig) {
    case Q_ENTRY_SIG: {
      LOG_INFO << "enter  running";
      pushBack(make_shared<Event>(Type::STEP));
      return nullptr;
    }
    case Q_EXIT_SIG: {
      LOG_INFO << "exit running";
      return nullptr;
    }
    case Type::STEP: {
      pushBack(make_shared<Event>(Type::STEP));
      if(Sensor::GetInstance()->isReady()){
        pushBack(make_shared<Event>(Type::CLEAN_GLOBAL_CLEAN_ROOM));
      }
      return nullptr;
    }
    case Type::CLEAN_GLOBAL_CLEAN_ROOM: {
      pushBack(make_shared<Event>(Type::CLEAN_INIT_AREA));
      publishFront(make_shared<Event>(Type::VIRTUAL_RESET));
      Q_TRAN((qp::QSTATE)&CleanActive::clean);
      return nullptr;
    }
    default: {
      return (QSTATE)&CleanActive::top;
    }
  }
}

qp::QSTATE CleanActive::clean(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Q_ENTRY_SIG: {
      LOG_INFO << "enter  clean";
      return nullptr;
    }
    case Q_EXIT_SIG: {
      LOG_INFO << "exit clean";
      clean_active_api_.reset();
      return nullptr;
    }
    case Type::STEP: {
      return nullptr;
    }
    case Type::CLEAN_INIT_AREA: {
      LOG_INFO << "CLEAN_INIT_AREA";
      clean_active_api_.roomIntRoomData(Sensor::GetInstance()->getMapData());
      Q_TRAN((qp::QSTATE)&CleanActive::clean_area);
      return nullptr;
    }
    default: {
      return (QSTATE)&CleanActive::running;
    }
  }
}

qp::QSTATE CleanActive::clean_area(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Q_ENTRY_SIG: {
      LOG_INFO << "enter  clean_area";
      pushBack(make_shared<Event>(Type::CLEAN_AREA));
      publishFront(make_shared<Event>(Type::VIRTUAL_RESET));
      return nullptr;
    }
    case Q_EXIT_SIG: {
      LOG_INFO << "exit clean_area";
      return nullptr;
    }
    case Type::STEP: {
      return nullptr;
    }
    case Type::CLEAN_AREA: {
      LOG_INFO << "CLEAN_AREA";
      common::Pose robot_pose;
      robot_pose = Sensor::GetInstance()->getSlamPose();
      Sensor::GetInstance()->updateCostmap(Sensor::GetInstance()->getMapData());
      auto             costmap = Sensor::GetInstance()->getCostmap2d();
      auto             map     = Sensor::GetInstance()->getMapData();
      common::RoomData room_data;
      if (true
          == clean_active_api_.roomGetNextRoom(robot_pose, std::make_shared<costmap_2d::Costmap2D>(costmap), map,
                                               room_data)) {
        Q_TRAN((qp::QSTATE)&CleanActive::clean_area_line);
      } else {
        LOG_INFO << "clean finish";
        Q_TRAN((qp::QSTATE)&CleanActive::running);
      }
      return nullptr;
    }
    default: {
      return (QSTATE)&CleanActive::clean;
    }
  }
}

qp::QSTATE CleanActive::clean_area_line(std::shared_ptr<qp::QEvent> e) {
  if (handle(e)) {
    return nullptr;
  }
  switch (e->sig) {
    case Q_ENTRY_SIG: {
      LOG_INFO << "enter  clean_area_line";
      publish(make_shared<EvenData<std::vector<common::Edge>>>(event::Type::VIRTUAL_ROOM_LINE,
                                                               clean_active_api_.roomGetCurrentRoomData().edges));
      pushBack(make_shared<Event>(Type::CLEAN_AREA_LINE_NP2P));
      publish(make_shared<Event>(Type::VIRTUAL_STOP_CHECK_LINE));
      return nullptr;
    }
    case Q_EXIT_SIG: {
      LOG_INFO << "exit clean_area_line";
      return nullptr;
    }
    case Type::STEP: {
      return nullptr;
    }
    case Type::CLEAN_AREA_LINE_NP2P: {
      LOG_INFO << "VIRTUAL_DOOR_LP2P_GOAL";
      std::vector<common::Pose> path;
      common::Pose              robot_pose = Sensor::GetInstance()->getSlamPose();
      common::Pose              active_segment_point;
      if (clean_active_api_.roomGetActiveSegmentPoints(active_segment_point)) {
        common::FindPathData find_path_data(robot_pose, active_segment_point, false);
        request(make_shared<SyncEventWithData<common::FindPathData>>(Type::CONTROL_FIND_PATH, find_path_data),
                Type::CONTROL_FIND_PATH_FINSIH, [this, robot_pose](std::shared_ptr<SyncEvent> e) {
                  LOG_INFO << "respend CONTROL_FIND_PATH_FINSIH";
                  auto resp = static_pointer_cast<SyncEventWithData<std::vector<common::Pose>>>(e);
                  if (!resp->data.empty()) {
                    LOG_INFO << "get path success";
                    common::Np2pData np2p_data(resp->data, false);
                    request(make_shared<SyncEventWithData<common::Np2pData>>(Type::CONTROL_NP2P, np2p_data),
                            Type::CONTROL_NP2P_FINISH, [this](std::shared_ptr<SyncEvent> e) {
                              LOG_INFO << "respend CONTROL_NP2P_FINISH";
                              auto resp = static_pointer_cast<SyncEventWithData<int>>(e);
                              if (resp->data == common::ControlFinishResult::NP2P_SUCCESS) {
                                LOG_INFO << "CONTROL_NP2P SUCCESS";
                                Q_TRAN((qp::QSTATE)&CleanActive::clean_chains);
                              } else if (resp->data == common::ControlFinishResult::NP2P_FAIL) {
                                LOG_INFO << "CONTROL_NP2P FAIL";
                              }
                              return nullptr;
                            });
                  } else {
                    LOG_INFO << "get path fail";
                    pushBack(make_shared<Event>(Type::CLEAN_AREA_LINE_NP2P));
                  }
                  return nullptr;
                });
      } else {
        LOG_INFO << "roomGetActiveSegmentPoints fail";
        Q_TRAN((qp::QSTATE)&CleanActive::clean_area);
      }
      return nullptr;
    }
    default: {
      return (QSTATE)&CleanActive::clean;
    }
  }
}

qp::QSTATE CleanActive::clean_chains(std::shared_ptr<qp::QEvent> e) {
  if (handle(e)) {
    return nullptr;
  }
  switch (e->sig) {
    case Q_ENTRY_SIG: {
      LOG_INFO << "enter  clean_chains";
      publish(make_shared<Event>(Type::VIRTUAL_START_CHECK_LINE));
      clean_active_api_.chainsSetMode(common::ChainsMode::COVER_MODE);
      pushBack(make_shared<Event>(Type::STEP));
      return nullptr;
    }
    case Q_EXIT_SIG: {
      LOG_INFO << "exit clean_chains";
      return nullptr;
    }
    case Type::STEP: {
      pushBack(make_shared<Event>(Type::STEP));
      common::Pose robot_pose;
      robot_pose = Sensor::GetInstance()->getSlamPose();
      Sensor::GetInstance()->pubCleanMap(robot_pose);
      std::vector<common::Pose> close_chains_poses;
      common::ChainsCLoseResult result;
      if (clean_active_api_.chainsIsClose(robot_pose, close_chains_poses, result, false)) {
        LOG_INFO << "NORMAL_CHAINS_CLOSE";
        std::vector<common::Pose> polygon_chains;
        common::Common::GetInstance()->approxPolyDP(close_chains_poses, polygon_chains, 3, false);
        Sensor::GetInstance()->publishPolygon("close_chains", polygon_chains);
        Q_TRAN((qp::QSTATE)&CleanActive::clean_cover);
        return nullptr;
      }
      return nullptr;
    }
    case Type::CLEAN_CHAINS_FOLLOWWALL: {
      LOG_INFO << "CLEAN_CHAINS_FOLLOWWALL";
      publish(make_shared<Event>(Type::CONTROL_FOLLOWALL));
      return nullptr;
    }
    case Type::VIRTUAL_ROOM_LP2P_GOAL: {
      LOG_INFO << "VIRTUAL_ROOM_LP2P_GOAL";
      auto             req = static_pointer_cast<EvenData<common::Edge>>(e);
      common::Lp2pData lp2p_data(req->data.end_pose, 0.05);
      sync_event_response_handler_->clear(toString((qp::QSTATE)mySource));
      request(make_shared<SyncEventWithData<common::Lp2pData>>(Type::CONTROL_LP2P, lp2p_data),
              Type::CONTROL_LP2P_FINISH, [this](std::shared_ptr<SyncEvent> e) {
                LOG_INFO << "respend CONTROL_LP2P_FINISH";
                auto resp = static_pointer_cast<SyncEventWithData<int>>(e);
                if (resp->data == common::ControlFinishResult::LP2P_SUCCESS) {
                  LOG_INFO << "CONTROL_LP2P SUCCESS";
                } else if (resp->data == common::ControlFinishResult::LP2P_FAIL) {
                  LOG_INFO << "CONTROL_LP2P FAIL";
                  publish(make_shared<Event>(Type::CONTROL_FOLLOWALL));
                }
                return nullptr;
              });
      return nullptr;
    }
    default: {
      return (QSTATE)&CleanActive::clean;
    }
  }
}


qp::QSTATE CleanActive::clean_cover(std::shared_ptr<qp::QEvent> e) {
  if (handle(e)) {
    return nullptr;
  }
  switch (e->sig) {
    case Q_ENTRY_SIG: {
      LOG_INFO << "enter  clean_cover";
      clean_active_api_.generateStaticCoveragePlan(clean_active_api_.getWorkModel());
      pushBack(make_shared<Event>(Type::CLEAN_COVER_PLANNER));
      pushBack(make_shared<Event>(Type::STEP));
      return nullptr;
    }
    case Q_EXIT_SIG: {
      LOG_INFO << "exit clean_cover";
      publish(make_shared<Event>(Type::VIRTUAL_STOP_CHECK_LINE));
      return nullptr;
    }
    case Type::STEP: {
      pushBack(make_shared<Event>(Type::STEP));
      if (!clean_active_api_.isCoverFinish()) {
        common::Pose   robot_pose;
        robot_pose = Sensor::GetInstance()->getSlamPose();
        Sensor::GetInstance()->pubCleanMap(robot_pose);
      }
      return nullptr;
    }
    case Type::CLEAN_COVER_PLANNER: {
      LOG_INFO << "CLEAN_COVER_PLANNER";
      if (clean_active_api_.getNotCover()) {
        std::vector<common::Pose> remnant_chains;
        if (clean_active_api_.haveRemnantChains(remnant_chains)) {
          common::FindPathData find_path_data(Sensor::GetInstance()->getSlamPose(),
                                              remnant_chains[remnant_chains.size() - 3], true);
          request(make_shared<SyncEventWithData<common::FindPathData>>(Type::CONTROL_FIND_PATH, find_path_data),
                  Type::CONTROL_FIND_PATH_FINSIH, [this](std::shared_ptr<SyncEvent> e) {
                    LOG_INFO << "respend CONTROL_FIND_PATH_FINSIH";
                    auto resp = static_pointer_cast<SyncEventWithData<std::vector<common::Pose>>>(e);
                    if (!resp->data.empty()) {
                      LOG_INFO << "get path success";
                      common::Np2pData np2p_data(resp->data, true);
                      request(make_shared<SyncEventWithData<common::Np2pData>>(Type::CONTROL_NP2P, np2p_data),
                              Type::CONTROL_NP2P_FINISH, [this](std::shared_ptr<SyncEvent> e) {
                                LOG_INFO << "respend CONTROL_NP2P_FINISH";
                                auto resp = static_pointer_cast<SyncEventWithData<int>>(e);
                                if (resp->data == common::ControlFinishResult::NP2P_SUCCESS) {
                                  LOG_INFO << "CONTROL_NP2P SUCCESS";
                                  Q_TRAN((qp::QSTATE)&CleanActive::clean_chains);
                                } else if (resp->data == common::ControlFinishResult::NP2P_FAIL) {
                                  LOG_INFO << "CONTROL_NP2P FAIL";
                                }
                                return nullptr;
                              });
                    } else {
                      LOG_INFO << "get path fail";
                    }
                    return nullptr;
                  });
          return nullptr;
        } else {
          clean_active_api_.coverFinish();
          Q_TRAN((qp::QSTATE)&CleanActive::clean_area);
        }
        return nullptr;
      }
      common::CoverPoint next_cover_point;
      if (clean_active_api_.getNextWayPoint(next_cover_point)) {
        clean_active_api_.setNextCoverPoint(next_cover_point);
        switch (next_cover_point.type) {
          case common::CoverPointType::LP2P: {
            LOG_INFO << " lp2p ";
            common::Lp2pData lp2p_data(next_cover_point.pose, 0.05);
            request(make_shared<SyncEventWithData<common::Lp2pData>>(Type::CONTROL_LP2P, lp2p_data),
                    Type::CONTROL_LP2P_FINISH, [this](std::shared_ptr<SyncEvent> e) {
                      LOG_INFO << "respend CONTROL_LP2P_FINISH";
                      auto resp = static_pointer_cast<SyncEventWithData<int>>(e);
                      if (resp->data == common::ControlFinishResult::LP2P_SUCCESS) {
                        LOG_INFO << "CONTROL_LP2P SUCCESS";
                        clean_active_api_.updateSection();
                        pushBack(make_shared<Event>(Type::CLEAN_COVER_PLANNER));
                      } else if (resp->data == common::ControlFinishResult::LP2P_FAIL) {
                        LOG_INFO << "CONTROL_LP2P FAIL";
                        LOG_INFO << " to cover followall";
                      }
                      return nullptr;
                    });
            return nullptr;
          }
          case common::CoverPointType::AP2P: {
            LOG_INFO << " ap2p ";
            common::CoverPoint next_next_point;
            if (clean_active_api_.getNextNextWayPoint(next_next_point) == true) {
              common::Ap2pData ap2p_data(next_cover_point.pose, next_next_point.pose);
              request(make_shared<SyncEventWithData<common::Ap2pData>>(Type::CONTROL_AP2P, ap2p_data),
                      Type::CONTROL_AP2P_FINISH, [this](std::shared_ptr<SyncEvent> e) {
                        LOG_INFO << "respend CONTROL_AP2P_FINISH";
                        auto resp = static_pointer_cast<SyncEventWithData<int>>(e);
                        if (resp->data == common::ControlFinishResult::AP2P_SUCCESS) {
                          LOG_INFO << "CONTROL_AP2P SUCCESS";
                          clean_active_api_.updateSection();
                          pushBack(make_shared<Event>(Type::CLEAN_COVER_PLANNER));
                        } else if (resp->data == common::ControlFinishResult::AP2P_FAIL) {
                          LOG_INFO << "CONTROL_AP2P FAIL";
                          clean_active_api_.updateSection();
                          pushBack(make_shared<Event>(Type::CLEAN_COVER_PLANNER));
                        }
                        return nullptr;
                      });
            }
            return nullptr;
          }
          case common::CoverPointType::NP2P: {
            LOG_INFO << " np2p ";
            common::FindPathData find_path_data(Sensor::GetInstance()->getSlamPose(), next_cover_point.pose, true);
            request(make_shared<SyncEventWithData<common::FindPathData>>(Type::CONTROL_FIND_PATH, find_path_data),
                    Type::CONTROL_FIND_PATH_FINSIH, [this](std::shared_ptr<SyncEvent> e) {
                      LOG_INFO << "respend CONTROL_FIND_PATH_FINSIH";
                      auto resp = static_pointer_cast<SyncEventWithData<std::vector<common::Pose>>>(e);
                      if (!resp->data.empty()) {
                        LOG_INFO << "get path success";
                        common::Np2pData np2p_data(resp->data, false);
                        request(make_shared<SyncEventWithData<common::Np2pData>>(Type::CONTROL_NP2P, np2p_data),
                                Type::CONTROL_NP2P_FINISH, [this](std::shared_ptr<SyncEvent> e) {
                                  LOG_INFO << "respend CONTROL_NP2P_FINISH";
                                  auto resp = static_pointer_cast<SyncEventWithData<int>>(e);
                                  if (resp->data == common::ControlFinishResult::NP2P_SUCCESS) {
                                    LOG_INFO << "CONTROL_NP2P SUCCESS";
                                    clean_active_api_.updateSection();
                                    pushBack(make_shared<Event>(Type::CLEAN_COVER_PLANNER));
                                  } else if (resp->data == common::ControlFinishResult::NP2P_FAIL) {
                                    LOG_INFO << "CONTROL_NP2P FAIL";
                                  }
                                  return nullptr;
                                });
                      } else {
                        LOG_INFO << "get path fail";
                      }
                      return nullptr;
                    });
            return nullptr;
          }
        }
      } else {
        LOG_INFO << "cover finish";
        std::vector<common::Pose> remnant_chains;
        if (clean_active_api_.haveRemnantChains(remnant_chains)) {
          common::FindPathData find_path_data(Sensor::GetInstance()->getSlamPose(),
                                              remnant_chains[remnant_chains.size() - 3], true);
          request(make_shared<SyncEventWithData<common::FindPathData>>(Type::CONTROL_FIND_PATH, find_path_data),
                  Type::CONTROL_FIND_PATH_FINSIH, [this](std::shared_ptr<SyncEvent> e) {
                    LOG_INFO << "respend CONTROL_FIND_PATH_FINSIH";
                    auto resp = static_pointer_cast<SyncEventWithData<std::vector<common::Pose>>>(e);
                    if (!resp->data.empty()) {
                      LOG_INFO << "get path success";
                      common::Np2pData np2p_data(resp->data, true);
                      request(make_shared<SyncEventWithData<common::Np2pData>>(Type::CONTROL_NP2P, np2p_data),
                              Type::CONTROL_NP2P_FINISH, [this](std::shared_ptr<SyncEvent> e) {
                                LOG_INFO << "respend CONTROL_NP2P_FINISH";
                                auto resp = static_pointer_cast<SyncEventWithData<int>>(e);
                                if (resp->data == common::ControlFinishResult::NP2P_SUCCESS) {
                                  LOG_INFO << "CONTROL_NP2P SUCCESS";
                                  Q_TRAN((qp::QSTATE)&CleanActive::clean_chains);
                                } else if (resp->data == common::ControlFinishResult::NP2P_FAIL) {
                                  LOG_INFO << "CONTROL_NP2P FAIL";
                                }
                                return nullptr;
                              });
                    } else {
                      LOG_INFO << "get path fail";
                    }
                    return nullptr;
                  });
        } else {
          clean_active_api_.coverFinish();
          Q_TRAN((qp::QSTATE)&CleanActive::clean_area);
        }
      }
      return nullptr;
    }
    default: {
      return (QSTATE)&CleanActive::clean;
    }
  }
}

bool CleanActive::handle(std::shared_ptr<qp::QEvent> e) {
  return sync_event_response_handler_->handle(toString((qp::QSTATE)mySource), e);
}

string CleanActive::toString(qp::QSTATE state) {
  if (state == (qp::QSTATE)&CleanActive::initial) {
    return "initial";
  } else if (state == (qp::QSTATE)&CleanActive::running) {
    return "running";
  } else if (state == (qp::QSTATE)&CleanActive::clean) {
    return "clean";
  } else if (state == (qp::QSTATE)&CleanActive::clean_area_line) {
    return "clean_area_line";
  } else if (state == (qp::QSTATE)&CleanActive::clean_area) {
    return "clean_area";
  } else if (state == (qp::QSTATE)&CleanActive::clean_chains) {
    return "clean_chains";
  }  else {
    return ("non-existent state");
  }
}

template <typename T>
void CleanActive::request(std::shared_ptr<event::SyncEventWithData<T>> request, event::Type responsetype,
                          std::function<void(std::shared_ptr<event::SyncEvent>)> callbackMap) {
  publish(request);
  sync_event_response_handler_->add(toString((qp::QSTATE)mySource), responsetype, request->id, callbackMap);
}
