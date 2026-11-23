/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/hal/timer/tcas_timer.h
 * Module      : Cycle clock and deadline supervision, public interface
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS (HAL)
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   The cyclic executive needs four answers from a clock, and this module
 *   gives them from ONE free-running hardware counter:
 *     1 How much time passed since the previous cycle began? The scheduler
 *       (Tcas.Scheduler.Run_Cycle) needs it as Elapsed, in milliseconds, to
 *       age the tracks.
 *     2 Is this phase still inside its deadline? The five deadlines are the
 *       ends of the five windows of the schedule (Tcas.Config.Window_*):
 *       150, 450, 750, 900 and 1 000 ms after the cycle began. A phase that
 *       runs past its end is an overrun (RISK-001 of the architecture
 *       document).
 *     3 Was the previous cycle late? The period is 1 000 ms, with a jitter
 *       tolerance.
 *     4 Is it too early to kick the watchdog? A kick is allowed only from
 *       the start of the last window (900 ms). This is the minimum-time
 *       check that tcas_wdg.h leaves to this module.
 *
 *   WHAT THIS MODULE DOES NOT DO
 *   It does not decide what to do about an overrun: it reports it and
 *   counts it. It does not refuse a LATE kick: the hardware watchdog has
 *   its own timeout, and refusing a late kick in software would reset a
 *   unit that is only slow. It does not program the hardware counter.
 *
 * COUNTER AND UNITS
 *   One tick is one microsecond (1 MHz). The counter is 32 bits wide and
 *   wraps about every 71.6 minutes. Every difference of two readings is an
 *   UNSIGNED subtraction, which is correct across a wrap as long as the
 *   real difference is below 2 ** 32 ticks. One cycle is about 10 ** 6
 *   ticks, so there are over 4 000 cycles of margin. [VERIFY] the tick
 *   rate against the SoC clock tree.
 *
 * DESIGN RULES (each one maps to a MISRA or DO-178C argument)
 *   R1  No dynamic memory. All state lives in a structure that the CALLER
 *       owns (MISRA Rule 21.3).
 *   R2  No hidden state. The module has no variables with static storage
 *       and no constant tables; everything it remembers is in
 *       tcas_timer_state_t. The deadline of a phase is a pure function.
 *   R3  Bounded execution. There are no loops at all: every answer is a
 *       subtraction and a comparison. No recursion (MISRA Rule 17.2).
 *   R4  Every pointer argument is checked for NULL and the failure is
 *       reported, never ignored.
 *   R5  The hardware counter is volatile. Every public function reads it
 *       exactly ONCE, into a local, and works on the local. The counter
 *       is never written.
 *   R6  FAIL CLOSED. Any doubt gives no time and an error: an
 *       uninitialized state, no cycle begun, a bad phase. A function that
 *       fails writes 0 to its output, never a stale or partial value.
 *   R7  Wrap-safe. Time is always (now - start) in unsigned 32-bit
 *       arithmetic. No function compares two raw readings.
 *   R8  The elapsed time given to the scheduler saturates at
 *       TCAS_TIMER_MAX_ELAPSED_MS (Tcas.Types.Time_Ms'Last), so it always
 *       fits the Ada type.
 *   R9  Counters saturate at UINT32_MAX and never wrap, so a long-running
 *       unit cannot turn "many faults" into "zero faults".
 *
 * ORDER OF CHECKS
 *   tcas_timer_begin_cycle
 *     1 arguments non-NULL                       ERR_NULL
 *     2 state initialized                        ERR_CONFIG
 *     3 the previous cycle was late              ERR_OVERRUN (the cycle
 *                                                 STILL begins and the
 *                                                 elapsed time is still
 *                                                 delivered)
 *   tcas_timer_elapsed_us
 *     1 arguments non-NULL                       ERR_NULL
 *     2 state initialized                        ERR_CONFIG
 *     3 a cycle has begun                        ERR_NO_CYCLE
 *   tcas_timer_check_phase
 *     1 arguments non-NULL                       ERR_NULL
 *     2 state initialized                        ERR_CONFIG
 *     3 a cycle has begun                        ERR_NO_CYCLE
 *     4 phase is 0 .. 4                          ERR_PHASE
 *     5 elapsed time within the phase deadline   ERR_OVERRUN
 *   tcas_timer_kick_allowed
 *     1 arguments non-NULL                       ERR_NULL
 *     2 state initialized                        ERR_CONFIG
 *     3 a cycle has begun                        ERR_NO_CYCLE
 *     4 elapsed time at or after TCAS_TIMER_KICK_EARLIEST_US
 *                                                ERR_EARLY
 *   The first failure wins, so the reported cause is stable.
 *
 * TIMING BUDGET
 *   One call: one register read and a few instructions, about 1 microsecond.
 *
 * USAGE (the caller, the cyclic executive, once per cycle)
 *   tcas_timer_init(&clk);                                    once
 *   for (;;) {
 *       status = tcas_timer_begin_cycle(&clk, clk_hw, &elapsed_ms);
 *       ... ownship ingestion ...
 *       status = tcas_timer_check_phase(&clk, clk_hw, TCAS_TIMER_PHASE_OWNSHIP);
 *       ... the other phases ...
 *       status = tcas_timer_kick_allowed(&clk, clk_hw);
 *       if (status == TCAS_TIMER_OK) { tcas_wdg_kick(&wdg, wdg_hw); }
 *   }
 *   The caller must use every return value (MISRA Rule 17.7).
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * Status codes are uint8_t constants, not enum values, so the layout
 *     is fixed and no signed/unsigned mixing occurs (Rule 10.4).
 *   * _Static_assert needs C11. Allowed under MISRA C:2012 Amendment 3.
 *   * No deviations are requested by this header.
*/

#ifndef TCAS_TIMER_H
#define TCAS_TIMER_H

#include <stdint.h>


// compile-time assertion helper 
#define TCAS_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)

// units, in ticks of one microsecond
#define TCAS_TIMER_TICKS_PER_MS  UINT32_C(1000)

// the cycle period: Tcas.Types.Cycle_Ms = 1 000 ms 
#define TCAS_TIMER_PERIOD_US     UINT32_C(1000000)

// a cycle that begins later than the period plus this tolerance is late [VERIFY] the tolerance against the timing analysis (RISK-001)
#define TCAS_TIMER_JITTER_US     UINT32_C(50000)

// largest elapsed time given to the scheduler, in milliseconds: must equal Tcas.Types.Time_Ms'Last 
#define TCAS_TIMER_MAX_ELAPSED_MS UINT32_C(3600000)

/*
 * phases of the cycle and their deadlines: the cumulative ends of the five
 * windows of Tcas.Config (150, 300, 300, 150 and 100 ms). The phase numbers
 * are the same as in tcas_wdg.h.
*/

typedef uint8_t tcas_timer_phase_t;

#define TCAS_TIMER_PHASE_OWNSHIP      ((tcas_timer_phase_t)0U)
#define TCAS_TIMER_PHASE_INTERROGATE  ((tcas_timer_phase_t)1U)
#define TCAS_TIMER_PHASE_THREAT       ((tcas_timer_phase_t)2U)
#define TCAS_TIMER_PHASE_ADVISORY     ((tcas_timer_phase_t)3U)
#define TCAS_TIMER_PHASE_OUTPUT       ((tcas_timer_phase_t)4U)

#define TCAS_TIMER_PHASE_COUNT        (5U)

#define TCAS_TIMER_DEADLINE_0_US  UINT32_C(150000)    /* ownship ingestion  */
#define TCAS_TIMER_DEADLINE_1_US  UINT32_C(450000)    /* interrogation      */
#define TCAS_TIMER_DEADLINE_2_US  UINT32_C(750000)    /* tracks and threats */
#define TCAS_TIMER_DEADLINE_3_US  UINT32_C(900000)    /* advisory           */
#define TCAS_TIMER_DEADLINE_4_US  UINT32_C(1000000)   /* output and BIT     */

// the watchdog may be kicked from the start of the last window 
#define TCAS_TIMER_KICK_EARLIEST_US  TCAS_TIMER_DEADLINE_3_US

// the deadlines grow, and the last one is the cycle period 
TCAS_STATIC_ASSERT((TCAS_TIMER_DEADLINE_0_US < TCAS_TIMER_DEADLINE_1_US) &&
                   (TCAS_TIMER_DEADLINE_1_US < TCAS_TIMER_DEADLINE_2_US) &&
                   (TCAS_TIMER_DEADLINE_2_US < TCAS_TIMER_DEADLINE_3_US) &&
                   (TCAS_TIMER_DEADLINE_3_US < TCAS_TIMER_DEADLINE_4_US),
                   "phase deadlines must be in increasing order");
TCAS_STATIC_ASSERT(TCAS_TIMER_DEADLINE_4_US == TCAS_TIMER_PERIOD_US,
                   "the last deadline must be the cycle period");

// the largest elapsed time, in ticks, still fits 32 bits: 3 600 000 ms is 3.6E9 ticks, below 2 ** 32 (4.29E9) 
TCAS_STATIC_ASSERT((TCAS_TIMER_MAX_ELAPSED_MS * TCAS_TIMER_TICKS_PER_MS) <
                       UINT32_MAX,
                   "the largest elapsed time must fit 32 bits of ticks");
// result codes

typedef uint8_t tcas_timer_status_t;

#define TCAS_TIMER_OK            ((tcas_timer_status_t)0U)
#define TCAS_TIMER_ERR_NULL      ((tcas_timer_status_t)1U)  /* NULL argument     */
#define TCAS_TIMER_ERR_CONFIG    ((tcas_timer_status_t)2U)  /* state not init    */
#define TCAS_TIMER_ERR_NO_CYCLE  ((tcas_timer_status_t)3U)  /* no cycle begun    */
#define TCAS_TIMER_ERR_PHASE     ((tcas_timer_status_t)4U)  /* phase not 0 .. 4  */
#define TCAS_TIMER_ERR_OVERRUN   ((tcas_timer_status_t)5U)  /* deadline or
                                                               period missed     */
#define TCAS_TIMER_ERR_EARLY     ((tcas_timer_status_t)6U)  /* kick too early    */


/* ---------------------------------------------------------------------------
 * hardware counter (one volatile register, 4 bytes, read-only)
 * placeholder layout until the SoC is chosen
 * [VERIFY] register offset, width and tick rate against the SoC manual
 * ------------------------------------------------------------------------ */

typedef struct
{
    volatile uint32_t count;    /* offset 0, free-running, 1 tick = 1 us      */
} tcas_timer_hw_t;

TCAS_STATIC_ASSERT(sizeof(tcas_timer_hw_t) == 4U, "timer must be 4 bytes");


/* ---------------------------------------------------------------------------
 * clock state (owned by the caller, one per supervised cycle)
 * initialize with tcas_timer_init before the first cycle; a zero-filled
 * structure is NOT a valid state: use the init function.
 * all counters saturate at UINT32_MAX and never wrap
 * ------------------------------------------------------------------------ */

typedef struct
{
    uint32_t start_ticks;           /* counter reading when the cycle began   */
    uint32_t in_cycle;              /* 1U once a cycle has begun              */
    uint32_t cycle;                 /* cycles begun since init                */
    uint32_t late_cycle_total;      /* cycles that began after the period
                                       plus the jitter tolerance              */
    uint32_t phase_overrun_total;   /* phase checks that found a missed
                                       deadline                               */
    uint32_t early_kick_total;      /* kick requests that came too early      */
    uint32_t initialized;           /* 1U once tcas_timer_init succeeded      */
} tcas_timer_state_t;

// interface

/*
 * tcas_timer_init
 *   prepares one clock state: no cycle begun, every counter zero.
 *   Returns TCAS_TIMER_OK or TCAS_TIMER_ERR_NULL; on NULL nothing is
 *   written
 */
tcas_timer_status_t tcas_timer_init(tcas_timer_state_t * state);

/*
 * tcas_timer_begin_cycle
 *   starts a new cycle at the current counter reading and delivers, in
 *   *elapsed_ms, the time since the PREVIOUS cycle began, in milliseconds,
 *   truncated and saturated at TCAS_TIMER_MAX_ELAPSED_MS (rule R8). On the
 *   first cycle it delivers 0. When the previous cycle began more than the
 *   period plus the jitter tolerance ago, the call returns
 *   TCAS_TIMER_ERR_OVERRUN, counts a late cycle, and STILL begins the cycle
 *   and delivers the elapsed time: the scheduler needs that number to age
 *   its tracks correctly. On NULL and CONFIG errors *elapsed_ms is 0 (when
 *   the pointer is usable) and the state is not changed
 */
tcas_timer_status_t tcas_timer_begin_cycle(tcas_timer_state_t * state,
                                           const tcas_timer_hw_t * hw,
                                           uint32_t * elapsed_ms);

/*
 * tcas_timer_elapsed_us
 *   delivers, in *elapsed_us, the time since the current cycle began, in
 *   microseconds. On any error *elapsed_us is 0 (when the pointer is
 *   usable). Does not change the state
 */
tcas_timer_status_t tcas_timer_elapsed_us(const tcas_timer_state_t * state,
                                          const tcas_timer_hw_t * hw,
                                          uint32_t * elapsed_us);

/*
 * tcas_timer_check_phase
 *   returns TCAS_TIMER_OK when the time since the cycle began is at or
 *   below the deadline of this phase, TCAS_TIMER_ERR_OVERRUN otherwise (and
 *   counts it). Call it when a phase finishes. Returns the first failing
 *   check of ORDER OF CHECKS
 */
tcas_timer_status_t tcas_timer_check_phase(tcas_timer_state_t * state,
                                           const tcas_timer_hw_t * hw,
                                           tcas_timer_phase_t phase);

/*
 * tcas_timer_kick_allowed
 *   returns TCAS_TIMER_OK when at least TCAS_TIMER_KICK_EARLIEST_US have
 *   passed since the cycle began, TCAS_TIMER_ERR_EARLY otherwise (and
 *   counts it). A kick that is late is NOT refused here. Returns the first
 *   failing check of ORDER OF CHECKS
 */
tcas_timer_status_t tcas_timer_kick_allowed(tcas_timer_state_t * state,
                                            const tcas_timer_hw_t * hw);

/*
 * tcas_timer_deadline_us
 *   the deadline of a phase, in microseconds after the cycle began; 0 for
 *   a phase that does not exist; pure function, no side effects
 */
uint32_t tcas_timer_deadline_us(tcas_timer_phase_t phase);

#endif