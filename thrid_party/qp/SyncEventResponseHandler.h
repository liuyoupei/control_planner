#ifndef CLEAN_ROBOT_SYNC_EVENT_RESPONSE_H
#define CLEAN_ROBOT_SYNC_EVENT_RESPONSE_H
#pragma once
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <tuple>

#include "Event.h"
#include "active.h"
#include <map>
namespace event {
class SyncEventResponseHandler {
public:
  static std::shared_ptr<SyncEventResponseHandler> create();
  void                                             add(std::string state, event::Type responseType, int sno,
                                                       std::function<void(std::shared_ptr<event::SyncEvent>)> handler);
  bool                                             handle(std::string state, std::shared_ptr<qp::QEvent> e);
  void                                             clear(std::string state);
  void                                             clear();

private:
  std::map<std::string, std::map<std::tuple<event::Type, int>, std::function<void(std::shared_ptr<event::SyncEvent>)>>>
    handlerMap;
};
}  // namespace event
#endif
