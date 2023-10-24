/*****************************************************************************
 * Product:  qhsm/C++
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
#ifndef qhsm_h
#define qhsm_h

#include <memory>
#ifndef qevent_h
#include "qevent.h"
#endif

namespace qp {

class QHsm {  // Quantum Hierarchical State Machine
public:
  typedef void (QHsm::*QPseudoState)(std::shared_ptr<QEvent>);    // pseudostate
  typedef QPseudoState (QHsm::*QState)(std::shared_ptr<QEvent>);  // state

  void               init(std::shared_ptr<QEvent> e = 0);  // execute initial transition
  virtual void       dispatch(std::shared_ptr<QEvent> e);  // dispatch event
  int                isIn(QState state);                   // "is-in-state" query
  bool               isSub(QState child, QState parent);
  static char const* getVersion();
  QState             getState() const {
    return myState;
  }
  QPseudoState getParentState(QState child);

protected:
  struct Tran {  // protected inner class Tran
    QState       myChain[16];
    unsigned int myActions;  // action mask (2-bits for action)
  };
  QHsm(QPseudoState initial);                 // Ctor
  virtual ~QHsm();                            // virtual Xtor
  QPseudoState top(std::shared_ptr<QEvent>);  // the "top" state

  void tran(QState target);               // dynamic state transition
  void tranStat(Tran* t, QState target);  // static state transition
  void init_(QState target) {
    myState = target;
  }
#define Q_INIT(target_) init_((QState)(target_))
#define Q_TRAN(target_)               \
  do {                                \
    static Tran t_;                   \
    tranStat(&t_, (QState)(target_)); \
  } while (0)
#define Q_TRAN_DYN(target_) tran((QState)(target_))

private:
  void tranSetup(Tran* t, QState target);

public:
  QState myState;   // the active state
  QState mySource;  // source state during a transition
};

// state-handler return type
typedef QHsm::QPseudoState QSTATE;

}  // namespace qp

// helper macro to calculate static dimension of a 1-dim array
#define DIM(array_) (sizeof(array_) / sizeof(*(array_)))

#endif  // qhsm_h
