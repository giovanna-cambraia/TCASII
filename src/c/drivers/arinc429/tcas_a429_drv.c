/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/drivers/arinc429/tcas_a429_drv.c
 * Module      : ARINC 429 receive driver, implementation
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Implements the interface declared in tcas_a429_drv.h. The header holds
 *   the contract (word layout, design rules, order of checks, timing
 *   budget); this file holds the code that makes it true. Read the header
 *   first.
 *
 * DESIGN RULES (as in the header, with the way each is met here)
 *   R1  No dynamic memory: no allocation call, no library call at all.
 *       This file includes only <stddef.h> (NULL) and the driver header.
 *   R2  No hidden state: there are no variables with static storage. The
 *       static functions below are helpers without state.
 *   R3  Bounded execution: the only loops are the label filter and the
 *       duplicate scan (both bounded by TCAS_A429_MAX_LABELS) and the
 *       drain loop (bounded by cfg->max_words, at most
 *       TCAS_A429_MAX_WORDS_PER_CYCLE). No recursion.
 *   R4  Every pointer argument is checked for NULL. A NULL out pointer is
 *       the one case where nothing can be reported through out; the
 *       return value still says ERR_NULL.
 *   R5  The FIFO "data" register is read exactly ONCE per word, into a
 *       local, because the read pops the word in hardware. The "status"
 *       register is read once per loop pass and the overrun bit is
 *       accumulated, because the hardware clears it on every status read.
 *   R6  Fail closed: only SSM = Normal Operation is accepted.
 *   R7  On any failure data = 0 and valid = 0. After a parity failure
 *       label, sdi and ssm are 0 too (nothing in the word is trusted).
 *       After a label or SSM failure label, sdi and ssm are delivered
 *       for diagnostics; data is still 0.
 *   R8  The drain takes at most cfg->max_words words per call.
 *
 * STRUCTURE OF THE CODE
 *   Every public function has ONE exit (a single return at the end) and
 *   builds its result in a local status variable. This keeps the decision
 *   structure simple for MC/DC coverage (DO-178C Table A-7).
 *
 *   Counters saturate (sat_inc) and never wrap, so a long-running unit can
 *   not turn "many faults" into "zero faults".
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * No pointer arithmetic: the output array is indexed (Rule 18.4).
 *   * Every if / else-if chain ends in an else (Rule 15.7).
 *   * Loop counters are not modified in the loop body (Rule 14.2).
 *   * The result of tcas_a429_decode_word is deliberately discarded inside
 *     the drain loop with a (void) cast: the per-word status is stored in
 *     the output entry itself (Rule 17.7).
 *   * No deviations are requested by this file.
 * ===========================================================================
 */

#include <stddef.h>

#include "tcas_a429_drv.h"



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
        // already saturated: nothing to do 
    }
}

// returns 1U when any bit of mask is set in value, else 0U 
static uint8_t hw_flag(uint32_t value, uint32_t mask)
{
    uint8_t flag = 0U;

    if ((value & mask) != 0U)
    {
        flag = 1U;
    }
    else
    {
        flag = 0U;
    }

    return flag;
}

// bounds of a configuration: label count and word limit are in range; this is the part that makes the loops over cfg safe
static uint8_t cfg_bounds_ok(const tcas_a429_cfg_t * cfg)
{
    uint8_t ok = 0U;

    if ((cfg->num_labels >= 1U) &&
        (cfg->num_labels <= TCAS_A429_MAX_LABELS) &&
        (cfg->max_words >= 1U) &&
        (cfg->max_words <= TCAS_A429_MAX_WORDS_PER_CYCLE))
    {
        ok = 1U;
    }
    else
    {
        ok = 0U;
    }

    return ok;
}

// full configuration check used once, by init: bounds plus "no label is isted twice" (the nested loops are bounded by 
// TCAS_A429_MAX_LABELS) 
static uint8_t cfg_valid(const tcas_a429_cfg_t * cfg)
{
    uint8_t  ok = cfg_bounds_ok(cfg);
    uint32_t i  = 0U;
    uint32_t j  = 0U;

    if (ok == 1U)
    {
        for (i = 0U; i < (uint32_t)cfg->num_labels; i++)
        {
            for (j = i + 1U; j < (uint32_t)cfg->num_labels; j++)
            {
                if (cfg->labels[i] == cfg->labels[j])
                {
                    ok = 0U;
                }
                else
                {
                    // different labels: fine 
                }
            }
        }
    }
    else
    {
        // bounds already failed: do not touch the array 
    }

    return ok;
}

// cheap check done on every word and every drain: the driver was initialized and the configuration is within bounds 
static uint8_t cfg_state_ok(const tcas_a429_state_t * state,
                            const tcas_a429_cfg_t * cfg)
{
    uint8_t ok = 0U;

    if ((state->initialized == 1U) && (cfg_bounds_ok(cfg) == 1U))
    {
        ok = 1U;
    }
    else
    {
        ok = 0U;
    }

    return ok;
}

// returns 1U when label is in the accepted list; num_labels is already known to be within bounds (cfg_state_ok), so the loop is bounded
static uint8_t label_accepted(const tcas_a429_cfg_t * cfg, uint8_t label)
{
    uint8_t  found = 0U;
    uint32_t i     = 0U;

    for (i = 0U; i < (uint32_t)cfg->num_labels; i++)
    {
        if (cfg->labels[i] == label)
        {
            found = 1U;
        }
        else
        {
            // not this entry 
        }
    }

    return found;
}

// maps an SSM value to a status code; the SSM is two bits, so the four cases cover every value 
static tcas_a429_status_t ssm_to_status(uint8_t ssm)
{
    tcas_a429_status_t status = TCAS_A429_OK;

    if (ssm == TCAS_A429_SSM_NORMAL)
    {
        status = TCAS_A429_OK;
    }
    else if (ssm == TCAS_A429_SSM_FAILURE_WARNING)
    {
        status = TCAS_A429_ERR_SSM_FW;
    }
    else if (ssm == TCAS_A429_SSM_NO_COMPUTED_DATA)
    {
        status = TCAS_A429_ERR_SSM_NCD;
    }
    else
    {
        status = TCAS_A429_ERR_SSM_FT;
    }

    return status;
}

// sets every counter to zero and marks the state as not initialized 
static void state_clear(tcas_a429_state_t * state)
{
    state->words_total        = 0U;
    state->parity_fail_total  = 0U;
    state->label_reject_total = 0U;
    state->ssm_reject_total   = 0U;
    state->overrun_total      = 0U;
    state->initialized        = 0U;
}



// public interface
tcas_a429_status_t tcas_a429_init(tcas_a429_state_t * state,
                                  const tcas_a429_cfg_t * cfg)
{
    tcas_a429_status_t status = TCAS_A429_OK;

    if (state == NULL)
    {
        status = TCAS_A429_ERR_NULL;
    }
    else
    {
        // safe empty value first: any failure below leaves it in place 
        state_clear(state);

        if (cfg == NULL)
        {
            status = TCAS_A429_ERR_NULL;
        }
        else if (cfg_valid(cfg) == 0U)
        {
            status = TCAS_A429_ERR_CONFIG;
        }
        else
        {
            state->initialized = 1U;
            status = TCAS_A429_OK;
        }
    }

    return status;
}

uint8_t tcas_a429_parity_ok(uint32_t word)
{
    uint32_t v = word;

    // fold all 32 bits into bit 0: it becomes the XOR of every bit 
    v ^= (v >> 16U);
    v ^= (v >> 8U);
    v ^= (v >> 4U);
    v ^= (v >> 2U);
    v ^= (v >> 1U);

    // odd parity: the XOR of all 32 bits (parity bit included) is 1 
    return (uint8_t)(v & 1U);
}

uint8_t tcas_a429_label_value(uint32_t word)
{
    uint32_t b = word & TCAS_A429_LABEL_MASK;

    // reverse the order of the 8 bits: swap nibbles, pairs, then singles 
    b = ((b & UINT32_C(0xF0)) >> 4U) | ((b & UINT32_C(0x0F)) << 4U);
    b = ((b & UINT32_C(0xCC)) >> 2U) | ((b & UINT32_C(0x33)) << 2U);
    b = ((b & UINT32_C(0xAA)) >> 1U) | ((b & UINT32_C(0x55)) << 1U);

    return (uint8_t)b;
}

tcas_a429_status_t tcas_a429_decode_word(tcas_a429_state_t * state,
                                         const tcas_a429_cfg_t * cfg,
                                         uint32_t word,
                                         uint32_t frame,
                                         tcas_a429_word_t * out)
{
    tcas_a429_status_t status = TCAS_A429_OK;
    uint32_t           data   = 0U;
    uint8_t            label  = 0U;
    uint8_t            sdi    = 0U;
    uint8_t            ssm    = 0U;

    // check 1: arguments; check 2: configuration 
    if (out == NULL)
    {
        status = TCAS_A429_ERR_NULL;
    }
    else if ((state == NULL) || (cfg == NULL))
    {
        status = TCAS_A429_ERR_NULL;
    }
    else if (cfg_state_ok(state, cfg) == 0U)
    {
        status = TCAS_A429_ERR_CONFIG;
    }
    else
    {
        sat_inc(&state->words_total);

        // check 3: odd parity of the whole word 
        if (tcas_a429_parity_ok(word) == 0U)
        {
            sat_inc(&state->parity_fail_total);
            status = TCAS_A429_ERR_PARITY;
            // nothing in the word is trusted: label, sdi, ssm stay 0 
        }
        else
        {
            label = tcas_a429_label_value(word);
            sdi   = (uint8_t)((word & TCAS_A429_SDI_MASK) >> TCAS_A429_SDI_SHIFT);
            ssm   = (uint8_t)((word & TCAS_A429_SSM_MASK) >> TCAS_A429_SSM_SHIFT);

            // check 4: label filter 
            if (label_accepted(cfg, label) == 0U)
            {
                sat_inc(&state->label_reject_total);
                status = TCAS_A429_ERR_LABEL;
            }
            else
            {
                // check 5: sign/status matrix 
                status = ssm_to_status(ssm);

                if (status == TCAS_A429_OK)
                {
                    data = (word & TCAS_A429_DATA_MASK) >> TCAS_A429_DATA_SHIFT;
                }
                else
                {
                    sat_inc(&state->ssm_reject_total);
                }
            }
        }
    }

    // the output is written on EVERY call (when there is one to write); data is non-zero only when the status is OK (rule R7) 
    if (out != NULL)
    {
        out->data       = data;
        out->frame      = frame;
        out->label      = label;
        out->sdi        = sdi;
        out->ssm        = ssm;
        out->status     = status;
        out->valid      = (status == TCAS_A429_OK) ? 1U : 0U;
        out->reserved8  = 0U;
        out->reserved16 = 0U;
    }
    else
    {
        // nothing to write to 
    }

    return status;
}

tcas_a429_status_t tcas_a429_drain(tcas_a429_state_t * state,
                                   const tcas_a429_cfg_t * cfg,
                                   const tcas_a429_hw_fifo_t * fifo,
                                   uint32_t frame,
                                   tcas_a429_word_t * out,
                                   uint32_t capacity,
                                   uint32_t * count)
{
    tcas_a429_status_t status = TCAS_A429_OK;
    uint32_t           i      = 0U;

    // *count is 0 on the two argument errors (see the header) 
    if (count != NULL)
    {
        *count = 0U;
    }
    else
    {
        // reported below as ERR_NULL 
    }

    // check 1: arguments; check 2: configuration and capacity 
    if ((state == NULL) || (cfg == NULL) || (fifo == NULL) ||
        (out == NULL) || (count == NULL))
    {
        status = TCAS_A429_ERR_NULL;
    }
    else if ((cfg_state_ok(state, cfg) == 0U) ||
             (capacity < (uint32_t)cfg->max_words))
    {
        status = TCAS_A429_ERR_CONFIG;
    }
    else
    {
        const uint32_t limit     = (uint32_t)cfg->max_words;
        uint32_t       hw        = fifo->status;           // one read 
        uint8_t        not_empty = hw_flag(hw, TCAS_A429_HW_NOT_EMPTY);
        uint8_t        overrun   = hw_flag(hw, TCAS_A429_HW_OVERRUN);

        // at most "limit" words per call (rule R8) 
        for (i = 0U; (i < limit) && (not_empty == 1U); i++)
        {
            const uint32_t word = fifo->data;              // pops one word 

            (void)tcas_a429_decode_word(state, cfg, word, frame, &out[i]);

            hw        = fifo->status;                      // one read 
            not_empty = hw_flag(hw, TCAS_A429_HW_NOT_EMPTY);

            if (hw_flag(hw, TCAS_A429_HW_OVERRUN) == 1U)
            {
                overrun = 1U;      // the hardware clears it on each read 
            }
            else
            {
                // no new overrun in this pass 
            }
        }

        *count = i;

        // check 3 and 4: overrun wins over backlog 
        if (overrun == 1U)
        {
            sat_inc(&state->overrun_total);
            status = TCAS_A429_ERR_OVERRUN;
        }
        else if (not_empty == 1U)
        {
            status = TCAS_A429_ERR_BACKLOG;
        }
        else
        {
            status = TCAS_A429_OK;
        }
    }

    return status;
}