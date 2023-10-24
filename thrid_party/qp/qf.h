/*****************************************************************************
 * Product:  qf/C++
 * Version:  2.6
 * Released: Dec 27 2003
 * Updated:  Dec 20 2004
 *
 * Copyright (C) 2002-2004 Quantum Leaps. All rights reserved.
 *
 * This software may be distributed and modified under the terms of the GNU
 * General Public License version 2 (GPL) as published by the Free Software
 * Foundation and appearing in the file GPL.TXT included in the packaging of
 * this file. Please note that GPL Section 2[b] requires that all works based
 * on this software must also be made publicly available under the terms of the
 * GPL ("Copyleft").
 *
 * Alternatively, this software may be distributed and modified under the terms
 * of Quantum Leaps commercial licenses, which are designed for users who want
 * to retain proprietary status of their code. This "dual-licensing" model is
 * possible because Quantum Leaps owns the copyright to this source code and as
 * such can license its intelectual property any number of times. The users who
 * license this software under one of Quantum Leaps commercial licenses do not
 * use this software under the GPL and therefore are not subject to any of its
 * terms.
 *
 * Contact information:
 * Quantum Leaps Web site:  http://www.quantum-leaps.com
 * Quantum Leaps licensing: http://www.quantum-leaps.com/licensing/overview.htm
 * e-mail:                  sales@quatnum-leaps.com
 *
 *****************************************************************************/
#ifndef QP_QF_H
#define QP_QF_H

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "active.h"
#include "qevent.h"

namespace qp {
class Qf {
public:
  void subscribe(Active* active, qp::QSignal event);

  void unsubscribe(Active* active, qp::QSignal event);

  void publish(std::shared_ptr<qp::QEvent> event);

  void publishFront(std::shared_ptr<qp::QEvent> event);

  void registerActor(std::shared_ptr<qp::Active> actor);

  void start();

private:
  std::function<void(std::shared_ptr<qp::QEvent>, std::string)> logFunction;
  std::mutex                                                    m_mutex;
  std::map<qp::QSignal, std::set<Active*>>                      subscribeMap;
  std::mutex                                                    m_actorMutex;
  std::vector<std::shared_ptr<qp::Active>>                      actorList;
};
}  // namespace qp

#endif  // QP_QF_H
