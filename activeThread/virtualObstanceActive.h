#ifndef CLEAN_ROBOT_VIRTUALOBSTANCEACTIVE_H
#define CLEAN_ROBOT_VIRTUALOBSTANCEACTIVE_H
#include "active.h"
#include "modules/virtualLineCheck/lineCheck.h"
#include "qassert.h"
#include "qhsm.h"
#include <Event.h>
#include <assert.h>
#include <iostream>
#include <memory>
#include <thread>
class VirtualObstanceActive : public qp::Active {
public:
  VirtualObstanceActive(std::shared_ptr<qp::Qf> qf)
    : Active((qp::QHsm::QPseudoState)&VirtualObstanceActive::initial, qf) {}
  static std::shared_ptr<VirtualObstanceActive> create(std::shared_ptr<qp::Qf> qf);

protected:
  void       initial(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE running(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE checkLine(std::shared_ptr<qp::QEvent> e);
private:
  LineCheck   line_check_;
};
#endif  // CLEAN_ROBOT_VIRTUALOBSTANCEACTIVE_H
