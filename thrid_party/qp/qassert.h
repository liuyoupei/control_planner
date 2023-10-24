/*****************************************************************************
 * Product:  QEP/C++
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
#ifndef qassert_h
#define qassert_h

#include <memory>

/* NASSERT macro disables all contract validations
 * (assertions, preconditions, postconditions, and invariants).
 */
#ifdef NASSERT /* NASSERT defined--Design By Contract disabled */

#define DEFINE_THIS_FILE
#define ASSERT(ignore_) ((void)0)
#define ALLEGE(test_) ((void)(test_))

#else /* NASSERT not defined--Design By Contract enabled */

#ifdef __cplusplus
extern "C" {
#endif
/* callback invoked in case of assertion failure */
void onAssert__(char const* file, int line);

#ifdef __cplusplus
}
#endif

#define DEFINE_THIS_FILE static char const this_file__[] = __FILE__

#define ASSERT(test_) ((test_) ? (void)0 : onAssert__(this_file__, __LINE__))

#define ALLEGE(test_) ASSERT(test_)

#endif /* NASSERT */

#define REQUIRE(test_) ASSERT(test_)
#define ENSURE(test_) ASSERT(test_)
#define INVARIANT(test_) ASSERT(test_)

/* the compile-time assertion exploits the fact that in C/C++ a dimension of
 * an array must be non-zero. The following declaration causes a compilation
 * error if the compile-time expression (test_) is not true. The assertion
 * has no runtime side effects.
 */
#define ASSERT_COMPILE(test_) extern char assert_compile__[(test_) != 0]

#endif /* qassert_h */
