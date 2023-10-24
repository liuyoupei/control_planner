
#pragma once
#include "common/common_type.h"
#include "thrid_party/qp/qevent.h"
#include "thrid_party/qp/qhsm.h"
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
namespace event {
enum Type : unsigned short {
  EMPTY,
  INIT,
  ENTRY,
  EXIT,
  TERMINATE = qp::Q_USER_SIG,
  STEP,
  CLEAN_GLOBAL_CLEAN_ROOM,
  CLEAN_SELECT_ROOM,
  CLEAN_INIT_AREA,
  CLEAN_AREA,
  CLEAN_AREA_LINE_NP2P,
  CLEAN_CHAINS_FOLLOWWALL,
  CLEAN_COVER_PLANNER,
  CONTROL_STOP_FINISH,
  CONTROL_LP2P,
  CONTROL_LP2P_FINISH,
  CONTROL_FOLLOWALL,
  CONTROL_GO_BACK_FORWARD,
  CONTROL_GO_BACK_FORWARD_FINISH,
  CONTROL_NP2P,
  CONTROL_NP2P_FINISH,
  CONTROL_AP2P,
  CONTROL_AP2P_FINISH,
  CONTROL_FIND_PATH,
  CONTROL_FIND_PATH_FINSIH,
  CONTROL_START_FIND_PATH,
  CONTROL_TROUBLE_RECOVERY,
  VIRTUAL_ROOM_LINE,
  VIRTUAL_START_CHECK_LINE,
  VIRTUAL_STOP_CHECK_LINE,
  VIRTUAL_ROOM_LP2P_GOAL,
  VIRTUAL_RESET,
  ROBOT_START_CLEAN,
  MESSAGE_SIZE,
  MAX
};
int nextSno();
using Event = qp::QEvent;
struct SyncEvent : public qp::QEvent {
  explicit SyncEvent(int id, Type type) : Event(static_cast<unsigned short>(type)), id(id) {}
  SyncEvent(Type type) : Event(static_cast<unsigned short>(type)), id(nextSno()) {}
  const int id;
};
template <typename T> struct SyncEventWithData : public SyncEvent {
  explicit SyncEventWithData(int id, Type type, const T& data) : SyncEvent(id, type), data(data) {}
  SyncEventWithData(Type type, const T& data) : SyncEvent(type), data(data) {}
  const T data;
};

template <typename T> struct EvenData : public qp::QEvent {
  EvenData(Type type, T data) : Event(static_cast<unsigned short>(type)), data(data) {}
  const T data;
};
}  // namespace event
