/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/common/tcas_common.h
 * Module      : Definitions shared by every C module of the project
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS (HAL and drivers)
 * Requirements: REQ-004 (placeholder ID)
 *
 * PURPOSE
 *   Until now every driver and HAL header defined the same compile-time
 *   assertion macro on its own, six copies with identical text. C allows
 *   that, but it is the kind of duplication where one copy drifts. This
 *   header is the single home of that macro, and of the two checks that the
 *   whole C side depends on.
 *
 *   Every header of the project includes this file instead of defining the
 *   macro itself:
 *     src/c/drivers/arinc429/tcas_a429_drv.h
 *     src/c/drivers/modes_s/tcas_modes_drv.h
 *     src/c/drivers/display/tcas_display_drv.h
 *     src/c/hal/watchdog/tcas_wdg.h
 *     src/c/hal/timer/tcas_timer.h
 *     src/c/hal/mpu/tcas_mpu.h
 *   The files that include those headers get the macro through them.
 *
 * WHAT IT PROVIDES
 *   TCAS_STATIC_ASSERT (cond, msg)
 *     a compile-time assertion. A false condition stops the build and
 *     prints msg. Used for every size, offset and constant that is shared
 *     with the Ada side, so a mismatch cannot reach a running system.
 *
 * WHAT IT CHECKS BY ITSELF (the build stops with a message if it fails)
 *   1 The compiler is C11 or later. _Static_assert and _Generic (used by
 *     the layout cross-checks) are C11 features.
 *   2 A byte is exactly 8 bits (CHAR_BIT). Every layout shared with Ada is
 *     written in bytes and the Ada types are Unsigned_8, Unsigned_16 and
 *     so on. On a processor with a wider char, every frame-size assert
 *     would fail with a much less helpful message.
 *
 * DESIGN RULES
 *   R1  Macros and compile-time checks only. No functions, no variables,
 *       no types: including this file adds nothing to the object code.
 *   R2  No other header is needed to use it, and it does not include any
 *       project header, so it can be included from every module without a
 *       cycle.
 *   R3  The include guard makes it safe to include more than once, which
 *       happens whenever a file includes two project headers (for example
 *       tests/unit/c/check_timer_consts.c).
 *
 * MISRA NOTES
 *   * _Static_assert needs C11. Allowed under MISRA C:2012 Amendment 3.
 *   * The macro wraps a single declaration, and its arguments are
 *     parenthesized (Rule 20.7).
 *   * #error is used only for a violated build requirement (Directive
 *     4.12 does not apply: there is no dynamic memory here).
 *   * No deviations are requested by this header.
 *
 * INCLUDE PATH
 *   Every compile command now needs  -I src/c/common  so that the other
 *   headers can find this one.
*/

#ifndef TCAS_COMMON_H
#define TCAS_COMMON_H

#include <limits.h>


/* check 1: the compiler must be C11 or later */
#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 201112L)
#error "TCAS needs a C11 compiler: use -std=c11 (or later)"
#endif


/* compile-time assertion helper, the single definition for the project */
#define TCAS_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)


/* check 2: a byte is 8 bits, so every layout in bytes matches the Ada side */
TCAS_STATIC_ASSERT(CHAR_BIT == 8,
                   "a byte must be 8 bits: the frame layouts shared with Ada "
                   "are written in bytes");

#endif