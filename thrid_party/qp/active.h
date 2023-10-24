
#ifndef QP_ACTIVE_H
#define QP_ACTIVE_H

#include <functional>
#include <future>
#include <memory>
#include <thread>

#include "qhsm.h"
#include "queue.h"

namespace qp {
class Qf;
class Active : public qp::QHsm {
protected:
  Active(QPseudoState initial, std::shared_ptr<qp::Qf> qf) : qp::QHsm(initial), qf(qf) {}

public:
  virtual int start();

  virtual void pushBack(std::shared_ptr<qp::QEvent> e);

  virtual void pushFront(std::shared_ptr<qp::QEvent> e);

  virtual void subscribe(qp::QSignal event);
  virtual void unsubscribe(qp::QSignal event);

  virtual void publish(std::shared_ptr<qp::QEvent> event);
  virtual void publishFront(std::shared_ptr<qp::QEvent> event);
  int          getMessageSize() {
    return queue.size();
  }

private:
  std::string getName();

private:
  //    std::thread runner;
  std::future<void>                      runner;
  qp::Queue<std::shared_ptr<qp::QEvent>> queue;
  std::shared_ptr<qp::Qf>                qf;
};
}  // namespace qp

#endif  // QP_ACTIVE_H
