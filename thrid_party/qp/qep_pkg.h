#ifndef qep_pkg_h
#define qep_pkg_h

#include <memory>

using namespace qp;

enum { Q_EMPTY_SIG = 0 };
extern std::shared_ptr<QEvent> pkgStdEvt[];  // preallocated standard events

#endif
