/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/hal/watchdog/tcas_wdg.c
 * Module      : Watchdog supervision of the cycle, implementation
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS (HAL)
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Implements the interface declared in tcas_wdg.h. The header holds the
 *   contract (the five phases, design rules, order of checks, timing
 *   budget); this file holds the code that makes it true. Read the header
 *   first.
 *
 * DESIGN RULES (as in the header, with the way each is met here)
 *   R1  No dynamic memory: no allocation call, no library call at all.
 *       This file includes only <stddef.h> (NULL) and the module header.
 *   R2  No hidden state: there are no variables with static storage. The
 *       static functions below are helpers without state.
 *   R3  Bounded execution: there are no loops. Every check is mask
 *       arithmetic on done_mask, one 32-bit word.
 *   R4  Every pointer argument is checked for NULL before anything is read
 *       or written.
 *   R5  The hardware register is written in ONE place: tcas_wdg_kick,
 *       after every check passed. Exactly two writes, TCAS_WDG_KEY_1 and
 *       then TCAS_WDG_KEY_2, with no read in between and no other write.
 *   R6  Fail closed: every refusal returns before the hardware is reached,
 *       and a refused kick leaves "kicked" unchanged. Only a kick that
 *       wrote both keys sets "kicked".
 *   R7  Phases complete in order: phase n can be marked only when the bits
 *       of phases 0 .. n-1 are all set.
 *   R8  One kick per cycle: "kicked" is checked before the phase marks, so
 *       the reported cause of a repeated kick is ERR_REPEAT, whatever the
 *       marks say.
 *   R9  Counters go through sat_inc, which saturates at UINT32_MAX.
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
 *   * Every shift is on an unsigned 32-bit value, and the shift amount is
 *     always 0 .. 4, because the phase is checked against the phase count
 *     BEFORE the shift (Rule 12.2).
 *   * Every if / else-if chain ends in an else (Rule 15.7).
 *   * The two writes to the volatile register are separate statements, in
 *     the order the hardware needs (Rule 13.2 does not apply: no volatile
 *     object is read).
 *   * No deviations are requested by this file.
*/

#include <stddef.h>

#include "tcas_wdg.h"



// helpers (static, no state)

// adds 1 to a counter and stops at UINT32_MAX; the pointer is never NULL because every caller passes the address of a member of a 
// checked state 
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

// sets every field to zero: no cycle begun, nothing marked, not initialized 
static void state_clear(tcas_wdg_state_t * state)
{
    state->done_mask        = 0U;
    state->in_cycle         = 0U;
    state->kicked           = 0U;
    state->cycle            = 0U;
    state->kicks_total      = 0U;
    state->incomplete_total = 0U;
    state->repeat_total     = 0U;
    state->order_total      = 0U;
    state->missed_total     = 0U;
    state->initialized      = 0U;
}

// public interface

tcas_wdg_status_t tcas_wdg_init(tcas_wdg_state_t * state)
{
    tcas_wdg_status_t status = TCAS_WDG_OK;

    if (state == NULL)
    {
        status = TCAS_WDG_ERR_NULL;
    }
    else
    {
        state_clear(state);
        state->initialized = 1U;
        status = TCAS_WDG_OK;
    }

    return status;
}

tcas_wdg_status_t tcas_wdg_begin_cycle(tcas_wdg_state_t * state)
{
    tcas_wdg_status_t status = TCAS_WDG_OK;

    /* check 1: argument; check 2: state */
    if (state == NULL)
    {
        status = TCAS_WDG_ERR_NULL;
    }
    else if (state->initialized != 1U)
    {
        status = TCAS_WDG_ERR_CONFIG;
    }
    else
    {
        /* the previous cycle began but never reached a kick: the hardware
         * has run a whole cycle without one */
        if ((state->in_cycle == 1U) && (state->kicked == 0U))
        {
            sat_inc(&state->missed_total);
        }
        else
        {
            /* first cycle, or the previous one was kicked: nothing missed */
        }

        state->done_mask = 0U;
        state->kicked    = 0U;
        state->in_cycle  = 1U;
        sat_inc(&state->cycle);
        status = TCAS_WDG_OK;
    }

    return status;
}

tcas_wdg_status_t tcas_wdg_phase_done(tcas_wdg_state_t * state,
                                      tcas_wdg_phase_t phase)
{
    tcas_wdg_status_t status = TCAS_WDG_OK;

    /* check 1: argument; check 2: state; check 3: a cycle has begun;
     * check 4: the phase exists (BEFORE any shift, so the shift amount is
     * always 0 .. 4) */
    if (state == NULL)
    {
        status = TCAS_WDG_ERR_NULL;
    }
    else if (state->initialized != 1U)
    {
        status = TCAS_WDG_ERR_CONFIG;
    }
    else if (state->in_cycle != 1U)
    {
        status = TCAS_WDG_ERR_NO_CYCLE;
    }
    else if (phase >= TCAS_WDG_PHASE_COUNT)
    {
        status = TCAS_WDG_ERR_PHASE;
    }
    else
    {
        const uint32_t bit     = UINT32_C(1) << phase;
        const uint32_t earlier = bit - UINT32_C(1);

        /* check 5: not already marked in this cycle */
        if ((state->done_mask & bit) != 0U)
        {
            sat_inc(&state->order_total);
            status = TCAS_WDG_ERR_DUPLICATE;
        }
        /* check 6: every earlier phase is marked (rule R7) */
        else if ((state->done_mask & earlier) != earlier)
        {
            sat_inc(&state->order_total);
            status = TCAS_WDG_ERR_ORDER;
        }
        else
        {
            state->done_mask = state->done_mask | bit;
            status = TCAS_WDG_OK;
        }
    }

    return status;
}

tcas_wdg_status_t tcas_wdg_kick(tcas_wdg_state_t * state,
                                tcas_wdg_hw_t * hw)
{
    tcas_wdg_status_t status = TCAS_WDG_OK;

    /* check 1: arguments; check 2: state; check 3: a cycle has begun */
    if ((state == NULL) || (hw == NULL))
    {
        status = TCAS_WDG_ERR_NULL;
    }
    else if (state->initialized != 1U)
    {
        status = TCAS_WDG_ERR_CONFIG;
    }
    else if (state->in_cycle != 1U)
    {
        status = TCAS_WDG_ERR_NO_CYCLE;
    }
    /* check 4: one kick per cycle (rule R8) */
    else if (state->kicked == 1U)
    {
        sat_inc(&state->repeat_total);
        status = TCAS_WDG_ERR_REPEAT;
    }
    /* check 5: all five phases are marked */
    else if (state->done_mask != TCAS_WDG_ALL_DONE)
    {
        sat_inc(&state->incomplete_total);
        status = TCAS_WDG_ERR_INCOMPLETE;
    }
    else
    {
        /* every check passed: the ONLY place that writes the hardware
         * (rule R5), two writes, in this order, no read */
        hw->kick = TCAS_WDG_KEY_1;
        hw->kick = TCAS_WDG_KEY_2;

        state->kicked = 1U;
        sat_inc(&state->kicks_total);
        status = TCAS_WDG_OK;
    }

    return status;
}

uint32_t tcas_wdg_missing(const tcas_wdg_state_t * state)
{
    uint32_t missing = TCAS_WDG_ALL_DONE;

    if (state == NULL)
    {
        /* nothing known: every phase counts as missing */
    }
    else if (state->initialized != 1U)
    {
        /* not initialized: every phase counts as missing */
    }
    else if (state->in_cycle != 1U)
    {
        /* no cycle begun: every phase counts as missing */
    }
    else
    {
        missing = (~state->done_mask) & TCAS_WDG_ALL_DONE;
    }

    return missing;
}