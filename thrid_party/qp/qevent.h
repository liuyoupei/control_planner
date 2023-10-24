#ifndef qevent_h
#define qevent_h

#include <memory>

namespace qp {

typedef unsigned short QSignal;

struct QEvent {
  explicit QEvent(QSignal sig) : sig(sig) {}
  QSignal sig;  // signal of the event instance
  virtual ~QEvent() {}
};

enum {  // standard signals
  Q_INIT_SIG = 1,
  Q_ENTRY_SIG,
  Q_EXIT_SIG,
  Q_USER_SIG
};
}  // namespace qp

#endif  // qevent_h
