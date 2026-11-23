/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/hal/timer/tcas_timer.c
 * Module      : Cycle clock and deadline supervision, implementation
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS (HAL)
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Implements the interface declared in tcas_timer.h. The header holds the
 *   contract (units, the five phase deadlines, design rules, order of
 *   checks, timing budget); this file holds the code that makes it true.
 *   Read the header first.
 *
 * DESIGN RULES (as in the header, with the way each is met here)
 *   R1  No dynamic memory: no allocation call, no library call at all.
 *       This file includes only <stddef.h> (NULL) and the module header.
 *   R2  No hidden state: there are no variables with static storage and no
 *       constant tables. The static functions below are helpers without
 *       state, and the deadline of a phase is a flat if / else-if chain.
 *   R3  Bounded execution: there are no loops. Every answer is one
 *       subtraction and one or two comparisons.
 *   R4  Every pointer argument is checked for NULL before anything is read
 *       or written. An output pointer that is usable is set to 0 BEFORE the
 *       other checks, so every failing path leaves 0 in it (rule R6).
 *   R5  The hardware counter is read exactly ONCE per call, into a local
 *       (const uint32_t now), and only in the branch where every earlier
 *       check has passed. It is never written.
 *   R6  Fail closed: a failing call writes 0 to its output and does not
 *       change the state (except the counters that count the failure).
 *   R7  Wrap-safe: every time is (now - start) in unsigned 32-bit
 *       arithmetic, which is correct across a wrap of the counter. No
 *       function compares two raw readings.
 *   R8  The elapsed time given to the scheduler is divided by 1 000 and
 *       then saturated at TCAS_TIMER_MAX_ELAPSED_MS, so it always fits
 *       Tcas.Types.Time_Ms.
 *   R9  Counters go through sat_inc, which saturates at UINT32_MAX.
 *
 * BOUNDARIES (all comparisons are strict, and are tested at the edge)
 *   late cycle    : (now - start) >  PERIOD + JITTER   (1 050 000 us)
 *   phase overrun : (now - start) >  deadline of the phase
 *   kick too early: (now - start) <  KICK_EARLIEST     (900 000 us)
 *   So a cycle of exactly 1 050 000 us is on time, a phase that finishes
 *   exactly at its deadline is on time, and a kick at exactly 900 000 us is
 *   allowed.
 *
 * STRUCTURE OF THE CODE
 *   Every public function has ONE exit (a single return at the end) and
 *   builds its result in a local status variable. The decision chains are
 *   flat if / else-if sequences, one branch per check, in the order of the
 *   header, so each condition is a separate, testable branch for MC/DC
 *   coverage (DO-178C Table A-7).
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * No pointer arithmetic, no casts that change the value (Rule 11.x).
 *   * Every if / else-if chain ends in an else (Rule 15.7).
 *   * The volatile counter is read through a const pointer, once, into a
 *     local (Rule 13.2).
 *   * No deviations are requested by this file.
*/

#include <stddef.h>

#include "tcas_timer.h"

// helpers (static, no state)

// adds 1 to a counter and stops at UINT32_MAX; the pointer is never NULL because every caller passes the address of a member of 
// a checked state 
static void sat_inc(uint32_t * counter)
{
    if (*counter < UINT32_MAX)
    {
        *counter = *counter + 1U;
    }
    else
    {
        /* already saturated: nothing to do */
    }
}

// sets every field to zero: no cycle begun, not initialized 
static void state_clear(tcas_timer_state_t * state)
{
    state->start_ticks         = 0U;
    state->in_cycle            = 0U;
    state->cycle               = 0U;
    state->late_cycle_total    = 0U;
    state->phase_overrun_total = 0U;
    state->early_kick_total    = 0U;
    state->initialized         = 0U;
}

// public interface
tcas_timer_status_t tcas_timer_init(tcas_timer_state_t * state)
{
    tcas_timer_status_t status = TCAS_TIMER_OK;

    if (state == NULL)
    {
        status = TCAS_TIMER_ERR_NULL;
    }
    else
    {
        state_clear(state);
        state->initialized = 1U;
        status = TCAS_TIMER_OK;
    }

    return status;
}

uint32_t tcas_timer_deadline_us(tcas_timer_phase_t phase)
{
    uint32_t deadline = 0U;

    if (phase == TCAS_TIMER_PHASE_OWNSHIP)
    {
        deadline = TCAS_TIMER_DEADLINE_0_US;
    }
    else if (phase == TCAS_TIMER_PHASE_INTERROGATE)
    {
        deadline = TCAS_TIMER_DEADLINE_1_US;
    }
    else if (phase == TCAS_TIMER_PHASE_THREAT)
    {
        deadline = TCAS_TIMER_DEADLINE_2_US;
    }
    else if (phase == TCAS_TIMER_PHASE_ADVISORY)
    {
        deadline = TCAS_TIMER_DEADLINE_3_US;
    }
    else if (phase == TCAS_TIMER_PHASE_OUTPUT)
    {
        deadline = TCAS_TIMER_DEADLINE_4_US;
    }
    else
    {
        /* no such phase: 0 */
    }

    return deadline;
}

tcas_timer_status_t tcas_timer_begin_cycle(tcas_timer_state_t * state,
                                           const tcas_timer_hw_t * hw,
                                           uint32_t * elapsed_ms)
{
    tcas_timer_status_t status = TCAS_TIMER_OK;

    /* the output is 0 on every failing path (rule R6): set it first */
    if (elapsed_ms != NULL)
    {
        *elapsed_ms = 0U;
    }
    else
    {
        /* reported below as ERR_NULL */
    }

    /* check 1: arguments; check 2: state */
    if ((state == NULL) || (hw == NULL) || (elapsed_ms == NULL))
    {
        status = TCAS_TIMER_ERR_NULL;
    }
    else if (state->initialized != 1U)
    {
        status = TCAS_TIMER_ERR_CONFIG;
    }
    else
    {
        const uint32_t now  = hw->count;              /* one read (rule R5) */
        uint32_t       ms   = 0U;
        uint32_t       late = 0U;

        if (state->in_cycle == 1U)
        {
            /* time since the previous cycle began, wrap-safe (rule R7) */
            const uint32_t diff = now - state->start_ticks;

            ms = diff / TCAS_TIMER_TICKS_PER_MS;

            /* saturate so the value always fits Tcas.Types.Time_Ms (R8) */
            if (ms > TCAS_TIMER_MAX_ELAPSED_MS)
            {
                ms = TCAS_TIMER_MAX_ELAPSED_MS;
            }
            else
            {
                /* already within the range */
            }

            /* check 3: the previous cycle was late */
            if (diff > (TCAS_TIMER_PERIOD_US + TCAS_TIMER_JITTER_US))
            {
                late = 1U;
            }
            else
            {
                /* on time */
            }
        }
        else
        {
            /* the first cycle: nothing before it, 0 ms, never late */
        }

        /* the cycle begins, late or not */
        state->start_ticks = now;
        state->in_cycle    = 1U;
        sat_inc(&state->cycle);
        *elapsed_ms = ms;

        if (late == 1U)
        {
            sat_inc(&state->late_cycle_total);
            status = TCAS_TIMER_ERR_OVERRUN;
        }
        else
        {
            status = TCAS_TIMER_OK;
        }
    }

    return status;
}

tcas_timer_status_t tcas_timer_elapsed_us(const tcas_timer_state_t * state,
                                          const tcas_timer_hw_t * hw,
                                          uint32_t * elapsed_us)
{
    tcas_timer_status_t status = TCAS_TIMER_OK;

    /* the output is 0 on every failing path (rule R6): set it first */
    if (elapsed_us != NULL)
    {
        *elapsed_us = 0U;
    }
    else
    {
        /* reported below as ERR_NULL */
    }

    /* check 1: arguments; check 2: state; check 3: a cycle has begun */
    if ((state == NULL) || (hw == NULL) || (elapsed_us == NULL))
    {
        status = TCAS_TIMER_ERR_NULL;
    }
    else if (state->initialized != 1U)
    {
        status = TCAS_TIMER_ERR_CONFIG;
    }
    else if (state->in_cycle != 1U)
    {
        status = TCAS_TIMER_ERR_NO_CYCLE;
    }
    else
    {
        const uint32_t now = hw->count;               /* one read (rule R5) */

        *elapsed_us = now - state->start_ticks;       /* wrap-safe (R7)     */
        status = TCAS_TIMER_OK;
    }

    return status;
}

tcas_timer_status_t tcas_timer_check_phase(tcas_timer_state_t * state,
                                           const tcas_timer_hw_t * hw,
                                           tcas_timer_phase_t phase)
{
    tcas_timer_status_t status = TCAS_TIMER_OK;

    /* check 1: arguments; check 2: state; check 3: a cycle has begun;
     * check 4: the phase exists (before the counter is read) */
    if ((state == NULL) || (hw == NULL))
    {
        status = TCAS_TIMER_ERR_NULL;
    }
    else if (state->initialized != 1U)
    {
        status = TCAS_TIMER_ERR_CONFIG;
    }
    else if (state->in_cycle != 1U)
    {
        status = TCAS_TIMER_ERR_NO_CYCLE;
    }
    else if (phase >= TCAS_TIMER_PHASE_COUNT)
    {
        status = TCAS_TIMER_ERR_PHASE;
    }
    else
    {
        const uint32_t now     = hw->count;           /* one read (rule R5) */
        const uint32_t elapsed = now - state->start_ticks;   /* (rule R7)   */

        /* check 5: within the deadline of this phase (strict) */
        if (elapsed > tcas_timer_deadline_us(phase))
        {
            sat_inc(&state->phase_overrun_total);
            status = TCAS_TIMER_ERR_OVERRUN;
        }
        else
        {
            status = TCAS_TIMER_OK;
        }
    }

    return status;
}

tcas_timer_status_t tcas_timer_kick_allowed(tcas_timer_state_t * state,
                                            const tcas_timer_hw_t * hw)
{
    tcas_timer_status_t status = TCAS_TIMER_OK;

    /* check 1: arguments; check 2: state; check 3: a cycle has begun */
    if ((state == NULL) || (hw == NULL))
    {
        status = TCAS_TIMER_ERR_NULL;
    }
    else if (state->initialized != 1U)
    {
        status = TCAS_TIMER_ERR_CONFIG;
    }
    else if (state->in_cycle != 1U)
    {
        status = TCAS_TIMER_ERR_NO_CYCLE;
    }
    else
    {
        const uint32_t now     = hw->count;           /* one read (rule R5) */
        const uint32_t elapsed = now - state->start_ticks;   /* (rule R7)   */

        /* check 4: not before the start of the last window (strict) */
        if (elapsed < TCAS_TIMER_KICK_EARLIEST_US)
        {
            sat_inc(&state->early_kick_total);
            status = TCAS_TIMER_ERR_EARLY;
        }
        else
        {
            /* a kick that is LATE is not refused here (see the header) */
            status = TCAS_TIMER_OK;
        }
    }

    return status;
}