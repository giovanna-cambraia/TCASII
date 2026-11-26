/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/drivers/display/tcas_display_drv.c
 * Module      : Display and audio output driver, implementation
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Implements the interface declared in tcas_display_drv.h. The header
 *   holds the contract (the frame layout, design rules, order of checks,
 *   transfer order, timing budget); this file holds the code that makes it
 *   true. Read the header first.
 *
 * DESIGN RULES (as in the header, with the way each is met here)
 *   R1  No dynamic memory: no allocation call, no library call at all.
 *       This file includes only <stddef.h> (NULL) and the driver header.
 *   R2  No hidden state: there are no variables with static storage and no
 *       tables. The static functions below are helpers without state.
 *   R3  Bounded execution: there are no loops. The frame check is a chain
 *       of comparisons and the transfer is a fixed list of writes.
 *   R4  Every pointer argument is checked for NULL before anything is read
 *       or written.
 *   R5  Fail closed: every refusal returns before the hardware is reached.
 *       The frame is checked BEFORE the status register is read, so a bad
 *       frame costs no register access at all.
 *   R6  The status register is read ONCE per call, into a local, and only
 *       in the branch where every earlier check passed. The seven data
 *       writes come first and the commit write is the LAST statement that
 *       touches the hardware.
 *   R7  Signed fields: rel_alt_ft is written as the 32-bit pattern of the
 *       value. target_rate_fpm is first converted to uint16_t and then
 *       widened, so a negative rate can never set the upper 16 bits of its
 *       register.
 *   R8  The frame check is the rule set of Display_Port.Is_Well_Formed.
 *       The four helpers (codes_ok, intruder_ok, rate_ok, audio_ok) follow
 *       the order of the rules in the header.
 *   R9  Counters go through sat_inc, which saturates at UINT32_MAX.
 *
 * STRUCTURE OF THE CODE
 *   Every public function has ONE exit (a single return at the end) and
 *   builds its result in a local variable. The decision chains are flat
 *   if / else-if sequences, one branch per check, in the order of the
 *   header, so each condition is a separate, testable branch for MC/DC
 *   coverage (DO-178C Table A-7). In the frame check each branch tests a
 *   BAD condition and the final else is the good case.
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * Every cast is explicit and value-preserving: int32_t to uint32_t is a
 *     bit pattern (rule R7), uint8_t and uint16_t widen to uint32_t
 *     (Rule 10.8).
 *   * Every if / else-if chain ends in an else (Rule 15.7).
 *   * The volatile registers are written through the pointer, one
 *     statement each, in the transfer order (Rule 13.2 does not apply: only
 *     one register is read, once, into a local).
 *   * No deviations are requested by this file.
*/

#include <stddef.h>

#include "tcas_display_drv.h"

// helpers (static, no state)

// adds 1 to a counter and stops at UINT32_MAX; the pointer is never NULL because every caller passes the address of a member of a checked state 
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
static void state_clear(tcas_display_state_t * state)
{
    state->shown_total    = 0U;
    state->rejected_total = 0U;
    state->busy_total     = 0U;
    state->fail_total     = 0U;
    state->initialized    = 0U;
}

// 1U when the reserved fields are 0 and every code is in its range (the first rules of the header) 
static uint8_t codes_ok(const tcas_display_frame_t * f)
{
    uint8_t ok = 0U;

    if ((f->reserved8 != 0U) || (f->reserved16 != 0U))
    {
        ok = 0U;
    }
    else if (f->mode > TCAS_DISPLAY_MODE_TA_RA)
    {
        ok = 0U;
    }
    else if (f->advisory > TCAS_DISPLAY_ADV_DESCEND)
    {
        ok = 0U;
    }
    else if (f->audio > TCAS_DISPLAY_AUDIO_CLEAR)
    {
        ok = 0U;
    }
    else if (f->intruder_class > TCAS_DISPLAY_CLASS_RESOLUTION)
    {
        ok = 0U;
    }
    else if (f->intruder_valid > 1U)
    {
        ok = 0U;
    }
    else
    {
        ok = 1U;
    }

    return ok;
}

// 1U when the intruder fields agree with intruder_valid: inside their limits with an intruder, all zero (and class "other") without one (rule R8 of the header, fail closed)
static uint8_t intruder_ok(const tcas_display_frame_t * f)
{
    uint8_t ok = 0U;

    if (f->intruder_valid == 1U)
    {
        if (f->slant_ft > TCAS_DISPLAY_SLANT_MAX)
        {
            ok = 0U;
        }
        else if (f->bearing_cdeg > TCAS_DISPLAY_BEARING_MAX)
        {
            ok = 0U;
        }
        else if ((f->rel_alt_ft < -TCAS_DISPLAY_ALT_DIFF_MAX) ||
                 (f->rel_alt_ft > TCAS_DISPLAY_ALT_DIFF_MAX))
        {
            ok = 0U;
        }
        else
        {
            ok = 1U;
        }
    }
    else if ((f->slant_ft != 0U) || (f->rel_alt_ft != 0) ||
             (f->bearing_cdeg != 0U) ||
             (f->intruder_class != TCAS_DISPLAY_CLASS_OTHER))
    {
        ok = 0U;                         /* no intruder: nothing about one */
    }
    else
    {
        ok = 1U;
    }

    return ok;
}

// 1U when the rate is inside its limit and agrees with the advisory, and an advisory comes with the TA and RA mode 
static uint8_t rate_ok(const tcas_display_frame_t * f)
{
    uint8_t ok = 0U;

    if ((f->target_rate_fpm < -TCAS_DISPLAY_RATE_MAX) ||
        (f->target_rate_fpm > TCAS_DISPLAY_RATE_MAX))
    {
        ok = 0U;
    }
    else if ((f->advisory == TCAS_DISPLAY_ADV_NONE) &&
             (f->target_rate_fpm != 0))
    {
        ok = 0U;                         /* no advisory: no rate */
    }
    else if ((f->advisory == TCAS_DISPLAY_ADV_CLIMB) &&
             (f->target_rate_fpm <= 0))
    {
        ok = 0U;                         /* a climb needs a positive rate */
    }
    else if ((f->advisory == TCAS_DISPLAY_ADV_DESCEND) &&
             (f->target_rate_fpm >= 0))
    {
        ok = 0U;                         /* a descend needs a negative rate */
    }
    else if ((f->advisory != TCAS_DISPLAY_ADV_NONE) &&
             (f->mode != TCAS_DISPLAY_MODE_TA_RA))
    {
        ok = 0U;                         /* an advisory needs TA and RA mode */
    }
    else
    {
        ok = 1U;
    }

    return ok;
}

// 1U when the audio phrase agrees with the advisory and the intruder: what the pilot hears never contradicts what the pilot sees (rule R7 of the Ada port)
static uint8_t audio_ok(const tcas_display_frame_t * f)
{
    uint8_t ok = 0U;

    if ((f->audio == TCAS_DISPLAY_AUDIO_TRAFFIC) &&
        ((f->intruder_valid != 1U) ||
         (f->intruder_class != TCAS_DISPLAY_CLASS_TRAFFIC)))
    {
        ok = 0U;                         /* "Traffic" needs a traffic advisory */
    }
    else if (((f->audio == TCAS_DISPLAY_AUDIO_CLIMB) ||
              (f->audio == TCAS_DISPLAY_AUDIO_CLIMB_NOW)) &&
             (f->advisory != TCAS_DISPLAY_ADV_CLIMB))
    {
        ok = 0U;                         /* "Climb" needs a climb */
    }
    else if (((f->audio == TCAS_DISPLAY_AUDIO_DESCEND) ||
              (f->audio == TCAS_DISPLAY_AUDIO_DESCEND_NOW)) &&
             (f->advisory != TCAS_DISPLAY_ADV_DESCEND))
    {
        ok = 0U;                         /* "Descend" needs a descend */
    }
    else if ((f->audio == TCAS_DISPLAY_AUDIO_CLEAR) &&
             (f->advisory != TCAS_DISPLAY_ADV_NONE))
    {
        ok = 0U;                         /* "Clear of conflict" needs none */
    }
    else
    {
        ok = 1U;
    }

    return ok;
}

// public interface

tcas_display_status_t tcas_display_init(tcas_display_state_t * state)
{
    tcas_display_status_t status = TCAS_DISPLAY_OK;

    if (state == NULL)
    {
        status = TCAS_DISPLAY_ERR_NULL;
    }
    else
    {
        state_clear(state);
        state->initialized = 1U;
        status = TCAS_DISPLAY_OK;
    }

    return status;
}

uint8_t tcas_display_frame_ok(const tcas_display_frame_t * frame)
{
    uint8_t ok = 0U;

    if (frame == NULL)
    {
        ok = 0U;
    }
    else if (codes_ok(frame) == 0U)
    {
        ok = 0U;
    }
    else if (intruder_ok(frame) == 0U)
    {
        ok = 0U;
    }
    else if (rate_ok(frame) == 0U)
    {
        ok = 0U;
    }
    else if (audio_ok(frame) == 0U)
    {
        ok = 0U;
    }
    else
    {
        ok = 1U;
    }

    return ok;
}

tcas_display_status_t tcas_display_show(tcas_display_state_t * state,
                                        tcas_display_hw_t * hw,
                                        const tcas_display_frame_t * frame)
{
    tcas_display_status_t status = TCAS_DISPLAY_OK;

    /* check 1: arguments; check 2: state; check 3: the frame, BEFORE any
     * register is read (rule R5) */
    if ((state == NULL) || (hw == NULL) || (frame == NULL))
    {
        status = TCAS_DISPLAY_ERR_NULL;
    }
    else if (state->initialized != 1U)
    {
        status = TCAS_DISPLAY_ERR_CONFIG;
    }
    else if (tcas_display_frame_ok(frame) == 0U)
    {
        sat_inc(&state->rejected_total);
        status = TCAS_DISPLAY_ERR_FRAME;
    }
    else
    {
        const uint32_t hw_status = hw->status;          /* one read (rule R6) */

        /* check 4: the unit has consumed the previous frame */
        if ((hw_status & TCAS_DISPLAY_HW_READY) == 0U)
        {
            sat_inc(&state->busy_total);
            status = TCAS_DISPLAY_ERR_BUSY;
        }
        else
        {
            /* every check passed: the ONLY place that writes the hardware
             * for a frame. Seven data writes, in this order, then the
             * commit, which is the last statement that touches the unit */
            hw->slant   = frame->slant_ft;
            hw->rel_alt = (uint32_t)frame->rel_alt_ft;           /* bit pattern */
            hw->bearing = (uint32_t)frame->bearing_cdeg;
            hw->rate    = (uint32_t)(uint16_t)frame->target_rate_fpm;
            hw->flags   = (uint32_t)frame->mode
                        | ((uint32_t)frame->advisory
                              << TCAS_DISPLAY_FLAGS_ADV_SHIFT)
                        | ((uint32_t)frame->intruder_class
                              << TCAS_DISPLAY_FLAGS_CLASS_SHIFT)
                        | ((uint32_t)frame->intruder_valid
                              << TCAS_DISPLAY_FLAGS_VALID_SHIFT);
            hw->audio   = (uint32_t)frame->audio;
            hw->control = TCAS_DISPLAY_CTRL_COMMIT;              /* LAST */

            sat_inc(&state->shown_total);
            status = TCAS_DISPLAY_OK;
        }
    }

    return status;
}

tcas_display_status_t tcas_display_fail(tcas_display_state_t * state,
                                        tcas_display_hw_t * hw)
{
    tcas_display_status_t status = TCAS_DISPLAY_OK;

    /* check 1: arguments; check 2: state */
    if ((state == NULL) || (hw == NULL))
    {
        status = TCAS_DISPLAY_ERR_NULL;
    }
    else if (state->initialized != 1U)
    {
        status = TCAS_DISPLAY_ERR_CONFIG;
    }
    else
    {
        const uint32_t hw_status = hw->status;          /* one read (rule R6) */

        /* check 3: the unit has consumed the previous frame */
        if ((hw_status & TCAS_DISPLAY_HW_READY) == 0U)
        {
            sat_inc(&state->busy_total);
            status = TCAS_DISPLAY_ERR_BUSY;
        }
        else
        {
            /* silence first, then the annunciation; the data registers are
             * not touched, the unit ignores them while FAIL is set */
            hw->audio   = 0U;
            hw->control = TCAS_DISPLAY_CTRL_COMMIT | TCAS_DISPLAY_CTRL_FAIL;

            sat_inc(&state->fail_total);
            status = TCAS_DISPLAY_OK;
        }
    }

    return status;
}