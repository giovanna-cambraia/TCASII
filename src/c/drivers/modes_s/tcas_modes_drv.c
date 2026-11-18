/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/drivers/modes_s/tcas_modes_drv.c
 * Module      : Mode S reply receive driver, implementation
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Implements the interface declared in tcas_modes_drv.h. The header holds
 *   the contract (frame layout, design rules, order of checks, timing
 *   budget); this file holds the code that makes it true. Read the header
 *   first.
 *
 * DESIGN RULES (as in the header, with the way each is met here)
 *   R1  No dynamic memory: no allocation call, no library call at all.
 *       This file includes only <stddef.h> (NULL) and the driver header.
 *   R2  No hidden state: there are no variables with static storage. The
 *       static functions below are helpers without state.
 *   R3  Bounded execution: the only loops run 14 times (the snapshot and
 *       the clear) or at most 14 times (the copy). No recursion.
 *   R4  Every pointer argument is checked for NULL before anything is
 *       read or written. A NULL out pointer is the one case where nothing
 *       can be reported through out; the return value still says ERR_NULL.
 *   R5  The mailbox is read ONCE: the status register first and alone
 *       (an empty mailbox costs one read), then, only when a reply is
 *       ready, every other register and every data byte, each exactly
 *       once, into local variables. The checks run on the locals. The
 *       mailbox is acknowledged once, at the END of the call, after the
 *       output frame is written, and only when a reply was taken.
 *   R6  Fail closed: the output frame is first cleared completely (length
 *       0, round trip 0, bearing 0, every byte 0, valid 0, the status
 *       set). Only the success branch copies data into it, and only the
 *       bytes up to the length, so a valid frame has 0 after its length.
 *   R7  A reply that cannot be used is still taken and acknowledged: the
 *       acknowledge depends only on "a reply was ready", never on the
 *       result of the checks.
 *
 * STRUCTURE OF THE CODE
 *   The public function has ONE exit (a single return at the end) and
 *   builds its result in a local status variable. The decision chains are
 *   flat if / else-if sequences, one branch per check, in the order of the
 *   header, so each condition is a separate, testable branch for MC/DC
 *   coverage (DO-178C Table A-7).
 *
 *   Counters saturate (sat_inc) and never wrap, so a long-running unit can
 *   not turn "many faults" into "zero faults".
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * No pointer arithmetic: arrays are indexed (Rule 18.4).
 *   * Every if / else-if chain ends in an else (Rule 15.7).
 *   * Loop counters are not modified in the loop body (Rule 14.2).
 *   * The volatile mailbox is read through the pointer into locals; the
 *     checks never use the volatile object twice (Rule 13.2).
 *   * No deviations are requested by this file.

*/

#include <stddef.h>

#include "tcas_modes_drv.h"


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

// sets every counter to zero and marks the state as not initialized 
static void state_clear(tcas_modes_state_t * state)
{
    state->replies_total      = 0U;
    state->valid_total        = 0U;
    state->overrun_total      = 0U;
    state->measure_fail_total = 0U;
    state->length_fail_total  = 0U;
    state->initialized        = 0U;
}

// an invalid frame: carries nothing but the status (rule R6) 
static void frame_clear(tcas_modes_frame_t * frame, tcas_modes_status_t status)
{
    uint32_t i = 0U;

    for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
    {
        frame->data[i] = 0U;
    }

    frame->length        = 0U;
    frame->status        = status;
    frame->round_trip_ns = 0U;
    frame->bearing       = 0U;
    frame->valid         = 0U;
    frame->reserved      = 0U;
}

// public interface

tcas_modes_status_t tcas_modes_init(tcas_modes_state_t * state)
{
    tcas_modes_status_t status = TCAS_MODES_OK;

    if (state == NULL)
    {
        status = TCAS_MODES_ERR_NULL;
    }
    else
    {
        state_clear(state);
        state->initialized = 1U;
        status = TCAS_MODES_OK;
    }

    return status;
}

tcas_modes_status_t tcas_modes_receive(tcas_modes_state_t * state,
                                       tcas_modes_hw_mailbox_t * hw,
                                       tcas_modes_frame_t * out)
{
    tcas_modes_status_t status = TCAS_MODES_OK;

    /* the snapshot of the mailbox (rule R5): filled only when a reply is
     * ready, and then every register and every byte is read exactly once */
    uint32_t hw_status   = 0U;
    uint32_t hw_length   = 0U;
    uint32_t hw_rtt      = 0U;
    uint32_t hw_bearing  = 0U;
    uint8_t  snap[TCAS_MODES_LONG_LENGTH] = { 0U };
    uint32_t i           = 0U;

    /* 1U once a reply was taken from the mailbox: it must be acknowledged
     * (rule R7) */
    uint8_t  taken       = 0U;

    /* check 1: arguments; check 2: state */
    if ((state == NULL) || (hw == NULL) || (out == NULL))
    {
        status = TCAS_MODES_ERR_NULL;
    }
    else if (state->initialized != 1U)
    {
        status = TCAS_MODES_ERR_CONFIG;
    }
    else
    {
        hw_status = hw->status;                          /* one read, alone */

        /* check 3: is there a reply? */
        if ((hw_status & TCAS_MODES_HW_READY) == 0U)
        {
            status = TCAS_MODES_ERR_NO_REPLY;            /* nothing taken   */
        }
        else
        {
            /* a reply is ready: snapshot everything, each read once */
            hw_length  = hw->length;
            hw_rtt     = hw->round_trip_ns;
            hw_bearing = hw->bearing_cdeg;

            for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
            {
                snap[i] = hw->data[i];
            }

            taken = 1U;
            sat_inc(&state->replies_total);

            /* check 4: overrun, the reply is discarded */
            if ((hw_status & TCAS_MODES_HW_OVERRUN) != 0U)
            {
                sat_inc(&state->overrun_total);
                status = TCAS_MODES_ERR_OVERRUN;
            }
            /* check 5: the hardware measured the time and the bearing */
            else if (((hw_status & TCAS_MODES_HW_MEAS_OK) == 0U) ||
                     (hw_bearing > TCAS_MODES_MAX_BEARING))
            {
                sat_inc(&state->measure_fail_total);
                status = TCAS_MODES_ERR_TIMESTAMP;
            }
            /* check 6: a Mode S message is 7 or 14 bytes */
            else if ((hw_length != TCAS_MODES_SHORT_LENGTH) &&
                     (hw_length != TCAS_MODES_LONG_LENGTH))
            {
                sat_inc(&state->length_fail_total);
                status = TCAS_MODES_ERR_LENGTH;
            }
            else
            {
                sat_inc(&state->valid_total);
                status = TCAS_MODES_OK;
            }
        }
    }

    /* the output frame is written on EVERY call (when there is one to
     * write): first cleared (rule R6), then filled by the success branch
     * only, and only up to the length */
    if (out != NULL)
    {
        frame_clear(out, status);

        if (status == TCAS_MODES_OK)
        {
            for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
            {
                if (i < hw_length)
                {
                    out->data[i] = snap[i];
                }
                else
                {
                    /* after the length: stays 0 from frame_clear */
                }
            }

            out->length        = (uint8_t)hw_length;
            out->round_trip_ns = hw_rtt;
            out->bearing       = (uint16_t)hw_bearing;
            out->valid         = 1U;
        }
        else
        {
            /* the frame stays empty: it carries only its status */
        }
    }
    else
    {
        /* nothing to write to */
    }

    /* acknowledge the mailbox once, after the frame is written, for every
     * reply taken, whatever the checks said (rule R7). hw is not NULL
     * whenever taken is 1U */
    if (taken == 1U)
    {
        hw->control = TCAS_MODES_HW_ACK;
    }
    else
    {
        /* no reply was taken: nothing to acknowledge */
    }

    return status;
}