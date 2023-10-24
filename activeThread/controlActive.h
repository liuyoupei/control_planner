#ifndef QP_CONTROL_ACTIVE_H
#define QP_CONTROL_ACTIVE_H

#include <Event.h>
#include <assert.h>

#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "RateTimer.h"
#include "active.h"
#include "control/ap2p.h"
#include "control/followall.h"
#include "control/lp2p.h"
#include "control/np2p.h"
#include "control/p2p.h"
#include "navigation/jps/jps_search.h"
#include "navigation/path_search.h"
#include "qassert.h"
#include "qhsm.h"
class ControlActive : public qp::Active {
public:
  ControlActive(std::shared_ptr<qp::Qf> qf) : Active((qp::QHsm::QPseudoState)&ControlActive::initial, qf) {}
  static std::shared_ptr<ControlActive> create(std::shared_ptr<qp::Qf> qf);

protected:
  void       initial(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE running(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE followWallControl(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE goBackForwardControl(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE lp2pControl(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE ap2pControl(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE np2pControl(std::shared_ptr<qp::QEvent> e);
  qp::QSTATE findPath(std::shared_ptr<qp::QEvent> e);

private:
  std::shared_ptr<control::P2P>       p2p_;
  std::shared_ptr<control::Followall> followall_;
  std::shared_ptr<control::LP2P>      lp2p_;
  std::shared_ptr<control::NP2P>      np2p_;
  std::shared_ptr<control::AP2P>      ap2p_;
  std::shared_ptr<nav::PathSearch>    path_search_;
  common::GoBackData                  go_back_data_;
  common::ArcData                     arc_data_;
  common::FindPathData                find_path_data_;
  RateTimer rateTimer{ 20 };
};
#endif  // QP_ACTIVE_TEST1_H
