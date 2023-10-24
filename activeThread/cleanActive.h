#ifndef QP_CLEAN_ACTIVE_H
#define QP_CLEAN_ACTIVE_H

#include "RateTimer.h"
#include "SyncEventResponseHandler.h"
#include "active.h"
#include "cleanActiveApi.h"
#include "navigation/jps/jps_search.h"
#include "navigation/path_search.h"
#include "qassert.h"
#include "qhsm.h"
#include <Event.h>
#include <assert.h>
#include <iostream>
#include <memory>
#include <thread>
class CleanActive : public qp::Active {
public:
  CleanActive(std::shared_ptr<qp::Qf> qf) : Active((qp::QHsm::QPseudoState)&CleanActive::initial, qf) {}
  static std::shared_ptr<CleanActive> create(std::shared_ptr<qp::Qf> qf);

protected:
  void       initial(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE running(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE clean(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE clean_area(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE clean_area_line(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE clean_chains(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE clean_cover(std::shared_ptr<qp::QEvent> e);
private:
  bool        handle(std::shared_ptr<qp::QEvent> e);
  std::string toString(qp::QSTATE state);
  template <typename T>
  void request(std::shared_ptr<event::SyncEventWithData<T>> request, event::Type responsetype,
               std::function<void(std::shared_ptr<event::SyncEvent>)> callbackMap);
  std::shared_ptr<event::SyncEventResponseHandler> sync_event_response_handler_;
  active_api::CleanActiveApi                       clean_active_api_;
};
#endif  // QP_ACTIVE_TEST1_H
