/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/hal/watchdog/tcas_wdg.h
 * Module      : Watchdog supervision of the cycle, public interface
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS (HAL)
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   The architecture document (section 4C) says that the watchdog is kicked
 *   "only after all frame completion flags verify true". This module is
 *   that rule. The cyclic executive marks each of the five phases of the
 *   1 000 ms cycle as done, in order, and at the end of the cycle asks for
 *   a kick. The kick reaches the hardware only when every phase is marked.
 *   If a phase was skipped or is stuck, the kick is refused, the hardware
 *   watchdog expires, and the unit resets: the intended safe state.
 *
 *   The five phases are the five windows of the schedule
 *   (Tcas.Config.Window_*):
 *     0 ownship ingestion      3 RA generation and coordination
 *     1 Mode S interrogation   4 display, audio and built-in test
 *     2 tracks and threats
 *
 *   WHAT THIS MODULE DOES NOT DO
 *   A mark means "this phase RAN", not "this phase gave the right result".
 *   The watchdog catches skipped, stuck and out-of-order phases. Wrong
 *   results are the job of the proofs, the tests and the built-in test.
 *
 * DESIGN RULES (each one maps to a MISRA or DO-178C argument)
 *   R1  No dynamic memory. All state lives in a structure that the CALLER
 *       owns (MISRA Rule 21.3).
 *   R2  No hidden state. The module has no static variables; everything it
 *       remembers is in tcas_wdg_state_t.
 *   R3  Bounded execution. There are no loops at all: every check is mask
 *       arithmetic on one 32-bit word. No recursion (MISRA Rule 17.2).
 *   R4  Every pointer argument is checked for NULL and the failure is
 *       reported, never ignored.
 *   R5  The hardware register is written in ONE place, tcas_wdg_kick,
 *       after every check passed: exactly two writes, key 1 and then key 2,
 *       in that order. It is never read, and never read-modified-written.
 *   R6  FAIL CLOSED. Any doubt means no kick: a missing phase, an out of
 *       order phase, an uninitialized state, no cycle begun. A refused
 *       kick leaves the hardware untouched.
 *   R7  Phases complete IN ORDER. A phase can be marked only when every
 *       earlier phase is marked, so a skipped phase cannot be hidden by
 *       marking it later.
 *   R8  ONE kick per cycle. A second kick in the same cycle is refused, so
 *       a runaway loop that calls kick repeatedly cannot keep the unit
 *       alive.
 *   R9  Counters saturate at UINT32_MAX and never wrap, so a long-running
 *       unit cannot turn "many faults" into "zero faults".
 *
 * ORDER OF CHECKS
 *   tcas_wdg_phase_done
 *     1 argument non-NULL                      ERR_NULL
 *     2 state initialized                      ERR_CONFIG
 *     3 a cycle has begun                      ERR_NO_CYCLE
 *     4 phase is 0 .. 4                        ERR_PHASE
 *     5 phase not already marked this cycle    ERR_DUPLICATE
 *     6 every earlier phase is marked          ERR_ORDER
 *   tcas_wdg_kick
 *     1 arguments non-NULL                     ERR_NULL
 *     2 state initialized                      ERR_CONFIG
 *     3 a cycle has begun                      ERR_NO_CYCLE
 *     4 no kick yet in this cycle              ERR_REPEAT
 *     5 all five phases marked                 ERR_INCOMPLETE
 *   The first failure wins, so the reported cause is stable.
 *
 * TIMING BUDGET
 *   One mark: a few instructions. One kick: two register writes, about 1
 *   microsecond. The kick belongs to the last slot of the cycle (900 to
 *   1 000 ms). [VERIFY] the hardware timeout: it must be longer than one
 *   cycle with margin, and short enough that a stalled cycle is caught
 *   before it matters.
 *
 * USAGE (the caller, the cyclic executive, once per cycle)
 *   tcas_wdg_init(&wdg);                                   once
 *   for (;;) {
 *       tcas_wdg_begin_cycle(&wdg);
 *       ... ownship ingestion ...
 *       tcas_wdg_phase_done(&wdg, TCAS_WDG_PHASE_OWNSHIP);
 *       ... interrogation ...
 *       tcas_wdg_phase_done(&wdg, TCAS_WDG_PHASE_INTERROGATE);
 *       ... threats, advisory, output ...
 *       tcas_wdg_phase_done(&wdg, TCAS_WDG_PHASE_OUTPUT);
 *       tcas_wdg_kick(&wdg, wdg_hw_ptr);
 *   }
 *   The caller must use every return value (MISRA Rule 17.7). An error
 *   from phase_done is a programming fault, not a run-time condition.
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * Status codes are uint8_t constants, not enum values, so the layout
 *     is fixed and no signed/unsigned mixing occurs (Rule 10.4).
 *   * _Static_assert needs C11. Allowed under MISRA C:2012 Amendment 3.
 *   * No deviations are requested by this header.
*/

#ifndef TCAS_WDG_H
#define TCAS_WDG_H

#include <stdint.h>


// compile-time assertion helper 
#define TCAS_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)

// phases of the cycle (the five windows of the schedule, in order)

typedef uint8_t tcas_wdg_phase_t;

#define TCAS_WDG_PHASE_OWNSHIP      ((tcas_wdg_phase_t)0U)  /*   0 ..  150 ms */
#define TCAS_WDG_PHASE_INTERROGATE  ((tcas_wdg_phase_t)1U)  /* 150 ..  450 ms */
#define TCAS_WDG_PHASE_THREAT       ((tcas_wdg_phase_t)2U)  /* 450 ..  750 ms */
#define TCAS_WDG_PHASE_ADVISORY     ((tcas_wdg_phase_t)3U)  /* 750 ..  900 ms */
#define TCAS_WDG_PHASE_OUTPUT       ((tcas_wdg_phase_t)4U)  /* 900 .. 1000 ms */

#define TCAS_WDG_PHASE_COUNT        (5U)

// the mask with one bit per phase, bit n for phase n 
#define TCAS_WDG_ALL_DONE           UINT32_C(0x0000001F)

TCAS_STATIC_ASSERT(((UINT32_C(1) << TCAS_WDG_PHASE_COUNT) - UINT32_C(1))
                       == TCAS_WDG_ALL_DONE,
                   "ALL_DONE must have exactly one bit per phase");

// result codes

typedef uint8_t tcas_wdg_status_t;

#define TCAS_WDG_OK              ((tcas_wdg_status_t)0U)
#define TCAS_WDG_ERR_NULL        ((tcas_wdg_status_t)1U)  /* NULL argument       */
#define TCAS_WDG_ERR_CONFIG      ((tcas_wdg_status_t)2U)  /* state not init      */
#define TCAS_WDG_ERR_NO_CYCLE    ((tcas_wdg_status_t)3U)  /* no cycle begun      */
#define TCAS_WDG_ERR_PHASE       ((tcas_wdg_status_t)4U)  /* phase not 0 .. 4    */
#define TCAS_WDG_ERR_DUPLICATE   ((tcas_wdg_status_t)5U)  /* already marked      */
#define TCAS_WDG_ERR_ORDER       ((tcas_wdg_status_t)6U)  /* earlier phase open  */
#define TCAS_WDG_ERR_INCOMPLETE  ((tcas_wdg_status_t)7U)  /* kick: phase missing */
#define TCAS_WDG_ERR_REPEAT      ((tcas_wdg_status_t)8U)  /* kick: already kicked*/


/* ---------------------------------------------------------------------------
 * hardware watchdog (one volatile register, 4 bytes)
 * placeholder layout until the SoC is chosen
 * [VERIFY] register offset, the two keys and their order against the SoC
 * manual. The watchdog accepts a kick only when KEY_1 and then KEY_2 are
 * written to the register, in that order, with no other write between.
 * ------------------------------------------------------------------------ */

#define TCAS_WDG_KEY_1   UINT32_C(0xA5A5A5A5)
#define TCAS_WDG_KEY_2   UINT32_C(0x5A5A5A5A)

typedef struct
{
    volatile uint32_t kick;     /* offset 0, write-only                       */
} tcas_wdg_hw_t;

TCAS_STATIC_ASSERT(sizeof(tcas_wdg_hw_t) == 4U, "watchdog must be 4 bytes");


/* ---------------------------------------------------------------------------
 * supervision state (owned by the caller, one per supervised cycle)
 * initialize with tcas_wdg_init before the first cycle; a zero-filled
 * structure is NOT a valid state: use the init function.
 * all counters saturate at UINT32_MAX and never wrap
 * ------------------------------------------------------------------------ */

typedef struct
{
    uint32_t done_mask;         /* bit n set when phase n is done this cycle  */
    uint32_t in_cycle;          /* 1U between begin_cycle and the next begin  */
    uint32_t kicked;            /* 1U once kicked in this cycle               */
    uint32_t cycle;             /* cycles begun since init                    */
    uint32_t kicks_total;       /* kicks that reached the hardware            */
    uint32_t incomplete_total;  /* kicks refused: a phase was missing         */
    uint32_t repeat_total;      /* kicks refused: second kick in one cycle    */
    uint32_t order_total;       /* phases refused: out of order or repeated   */
    uint32_t missed_total;      /* cycles that ended without a kick           */
    uint32_t initialized;       /* 1U once tcas_wdg_init succeeded            */
} tcas_wdg_state_t;

// interface

/*
 * tcas_wdg_init
 *   prepares one supervision state: no cycle begun, every counter zero.
 *   Returns TCAS_WDG_OK or TCAS_WDG_ERR_NULL; on NULL nothing is written
 */
tcas_wdg_status_t tcas_wdg_init(tcas_wdg_state_t * state);

/*
 * tcas_wdg_begin_cycle
 *   starts a new cycle: clears the phase marks and the kicked flag and
 *   counts the cycle. If the PREVIOUS cycle began but was never kicked, it
 *   is counted in missed_total (the hardware has been running without a
 *   kick for a whole cycle). Returns TCAS_WDG_OK, TCAS_WDG_ERR_NULL or
 *   TCAS_WDG_ERR_CONFIG; on an error the state is not changed
 */
tcas_wdg_status_t tcas_wdg_begin_cycle(tcas_wdg_state_t * state);

/*
 * tcas_wdg_phase_done
 *   marks one phase of the current cycle as done. The phases must be marked
 *   in order (rule R7). Returns the first failing check of ORDER OF CHECKS
 *   or TCAS_WDG_OK; on an error the marks are not changed
 */
tcas_wdg_status_t tcas_wdg_phase_done(tcas_wdg_state_t * state,
                                      tcas_wdg_phase_t phase);

/*
 * tcas_wdg_kick
 *   kicks the hardware watchdog, only when all five phases of the current
 *   cycle are marked and no kick was made yet in this cycle (rules R5 to
 *   R8). On any error the hardware register is NOT touched. Returns the
 *   first failing check of ORDER OF CHECKS or TCAS_WDG_OK
 */
tcas_wdg_status_t tcas_wdg_kick(tcas_wdg_state_t * state,
                                tcas_wdg_hw_t * hw);

/*
 * tcas_wdg_missing
 *   returns the phases that are NOT yet marked in the current cycle, one
 *   bit per phase (bit n for phase n); pure, no side effects; for
 *   diagnostics. A NULL, uninitialized or not-begun state reports every
 *   phase as missing (TCAS_WDG_ALL_DONE)
 */
uint32_t tcas_wdg_missing(const tcas_wdg_state_t * state);

#endif