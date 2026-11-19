/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : tests/unit/c/test_wdg.c
 * Module      : Unit tests of the watchdog supervision (Unity)
 * Language    : C11, Unity test framework
 * Assurance   : DO-178C Software Level B (target), verification artifact
 * Tests       : src/c/hal/watchdog/tcas_wdg.c and .h
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Checks every behavior promised in tcas_wdg.h: the five phases of the
 *   cycle in order, the refusal of a kick unless every phase is marked,
 *   the single kick per cycle, the missed-cycle count and the saturating
 *   counters. Each test checks one thing and has a name that says what it
 *   is.
 *
 * DESIGN RULES OF THE TESTS
 *   T1  The hardware register is plain memory (see LIMITS below), so the
 *       tests need no hardware, no operating system and no timing.
 *   T2  Expected values are written as literals taken from the header
 *       (status codes, keys, masks), not computed with the driver's code.
 *       The sweep has its own oracle, written from the ORDER OF CHECKS of
 *       the header in a DIFFERENT form: a counter of "phases marked so
 *       far" instead of the driver's bit masks.
 *   T3  Every test builds its own state and register (setUp does it), so
 *       tests do not depend on each other or on their order.
 *   T4  The register starts at a sentinel pattern. A refused kick must
 *       leave it untouched (it still holds the sentinel); a good kick
 *       leaves KEY_2 in it.
 *   T5  Fail-closed cases are tested with FORCED states (a mask with extra
 *       bits, an "initialized" flag of 2, a "kicked" flag with no marks),
 *       because the driver must refuse anything that is not exactly the
 *       state it created itself.
 *   T6  Some phase numbers are SHIFT TRAPS: 32, 33 and 255 would be an
 *       undefined shift if the driver shifted BEFORE it checked the range.
 *       They must be refused as ERR_PHASE.
 *
 * TESTS BY GROUP
 *   init        : NULL, valid, dirty state
 *   begin       : NULL, uninitialized, flag, first cycle, clearing, missed
 *                 cycles, saturation
 *   phase       : NULL, state checks, range, order, duplicates, precedence,
 *                 saturation
 *   kick        : NULL, state checks, incomplete, good, repeat, extra mask
 *                 bits, next cycle, saturation
 *   missing     : every state, partial counts, no side effects
 *   scenario    : a stuck cycle between two good ones
 *   sweep       : 7 776 mark sequences against the oracle
 *
 * LIMITS (to be listed as an open item for v2)
 *   The fake register does not record writes: it only holds the last value.
 *   The tests therefore cannot show that KEY_1 was written BEFORE KEY_2, or
 *   that there were exactly two writes (rule R5). They show that nothing is
 *   written on a refusal (the sentinel survives) and that the final value
 *   after a good kick is KEY_2. A check of the write order needs a write
 *   hook in the driver or a link-time mock of the register access, the same
 *   item as the read hooks in the two receive-driver tests.
 *
 * USAGE (from the project root; UNITY_DIR is the folder that holds
 * unity.c and unity.h, for example the one used by the MIL1553 project)
 *   UNITY_DIR=../MIL1553/tests/unity   (adjust to your path)
 *   gcc -std=c11 -Wall -Wextra -Wpedantic \
 *       -I src/c/hal/watchdog -I $UNITY_DIR \
 *       tests/unit/c/test_wdg.c \
 *       src/c/hal/watchdog/tcas_wdg.c \
 *       $UNITY_DIR/unity.c -o build/bin/test_wdg.exe
 *   build/bin/test_wdg.exe
 *
 * MISRA NOTES
 *   * The rules of MISRA C apply to the driver, not to its tests. The
 *     tests use <string.h> and <stdio.h> and plain loops for clarity.
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "tcas_wdg.h"

// test fixtures

#define HW_SENTINEL  UINT32_C(0x5555AAAA)   /* "the register was not written" */

static tcas_wdg_state_t state;
static tcas_wdg_hw_t    hw;

// helpers

static void reset_hw(void)
{
    hw.kick = HW_SENTINEL;
}

// begins a cycle and asserts that it worked 
static void begin(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_begin_cycle(&state));
}

// marks the first n phases (0 .. n-1) in order and asserts that each worked 
static void mark_first(uint32_t n)
{
    uint32_t i = 0U;

    for (i = 0U; i < n; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(
            TCAS_WDG_OK, tcas_wdg_phase_done(&state, (tcas_wdg_phase_t)i));
    }
}

static void check_hw_untouched(void)
{
    TEST_ASSERT_EQUAL_HEX32(HW_SENTINEL, hw.kick);
}

static void check_counters(uint32_t cycle, uint32_t kicks,
                           uint32_t incomplete, uint32_t repeat,
                           uint32_t order, uint32_t missed)
{
    TEST_ASSERT_EQUAL_UINT32(cycle,      state.cycle);
    TEST_ASSERT_EQUAL_UINT32(kicks,      state.kicks_total);
    TEST_ASSERT_EQUAL_UINT32(incomplete, state.incomplete_total);
    TEST_ASSERT_EQUAL_UINT32(repeat,     state.repeat_total);
    TEST_ASSERT_EQUAL_UINT32(order,      state.order_total);
    TEST_ASSERT_EQUAL_UINT32(missed,     state.missed_total);
}

// the state is exactly what it was (a refusal changes nothing) 
static void check_unchanged(const tcas_wdg_state_t * before)
{
    TEST_ASSERT_EQUAL_INT(0, memcmp(before, &state, sizeof(state)));
}

void setUp(void)
{
    (void)memset(&state, 0, sizeof(state));
    reset_hw();
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_init(&state));
}

void tearDown(void)
{
}

// group 1: init

void test_init_null(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NULL, tcas_wdg_init(NULL));
}

void test_init_valid(void)
{
    (void)memset(&state, 0xFF, sizeof(state));

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_init(&state));
    TEST_ASSERT_EQUAL_UINT32(1U, state.initialized);
    TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(0U, state.in_cycle);
    TEST_ASSERT_EQUAL_UINT32(0U, state.kicked);
    check_counters(0U, 0U, 0U, 0U, 0U, 0U);
}

void test_init_wipes_a_dirty_state(void)
{
    state.done_mask        = 99U;
    state.in_cycle         = 98U;
    state.kicked           = 97U;
    state.cycle            = 96U;
    state.kicks_total      = 95U;
    state.incomplete_total = 94U;
    state.repeat_total     = 93U;
    state.order_total      = 92U;
    state.missed_total     = 91U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_init(&state));
    TEST_ASSERT_EQUAL_UINT32(1U, state.initialized);
    TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(0U, state.in_cycle);
    TEST_ASSERT_EQUAL_UINT32(0U, state.kicked);
    check_counters(0U, 0U, 0U, 0U, 0U, 0U);
}

// group 2: begin_cycle

void test_begin_null(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NULL, tcas_wdg_begin_cycle(NULL));
}

void test_begin_uninitialized_is_a_config_error(void)
{
    tcas_wdg_state_t before;

    (void)memset(&state, 0, sizeof(state));         /* zero-filled: NOT valid */
    (void)memcpy(&before, &state, sizeof(before));

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_CONFIG, tcas_wdg_begin_cycle(&state));
    check_unchanged(&before);
}

void test_begin_initialized_flag_must_be_exactly_one(void)
{
    state.initialized = 2U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_CONFIG, tcas_wdg_begin_cycle(&state));
    TEST_ASSERT_EQUAL_UINT32(0U, state.in_cycle);
}

void test_begin_first_cycle(void)
{
    begin();

    TEST_ASSERT_EQUAL_UINT32(1U, state.in_cycle);
    TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(0U, state.kicked);
    check_counters(1U, 0U, 0U, 0U, 0U, 0U);       /* nothing missed yet */
}

void test_begin_clears_marks_and_kicked(void)
{
    /* a finished and kicked cycle, then a new one begins */
    state.in_cycle  = 1U;
    state.kicked    = 1U;
    state.done_mask = 0x1FU;
    state.cycle     = 7U;

    begin();

    TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(0U, state.kicked);
    TEST_ASSERT_EQUAL_UINT32(1U, state.in_cycle);
    check_counters(8U, 0U, 0U, 0U, 0U, 0U);       /* it was kicked: not missed */
}

void test_begin_after_a_kicked_cycle_is_not_missed(void)
{
    begin();
    mark_first(5U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_kick(&state, &hw));

    begin();
    check_counters(2U, 1U, 0U, 0U, 0U, 0U);
}

void test_begin_after_an_unkicked_cycle_counts_a_missed_cycle(void)
{
    begin();
    mark_first(2U);                                /* the cycle stalls here */

    begin();
    check_counters(2U, 0U, 0U, 0U, 0U, 1U);
    TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask); /* marks are cleared */
}

void test_begin_missed_cycles_accumulate(void)
{
    begin();     /* cycle 1: nothing before it */
    begin();     /* cycle 2: cycle 1 was not kicked */
    begin();     /* cycle 3: cycle 2 was not kicked */
    begin();     /* cycle 4: cycle 3 was not kicked */

    check_counters(4U, 0U, 0U, 0U, 0U, 3U);
}

void test_begin_counters_saturate(void)
{
    state.in_cycle     = 1U;
    state.kicked       = 0U;
    state.cycle        = UINT32_MAX;
    state.missed_total = UINT32_MAX;

    begin();

    check_counters(UINT32_MAX, 0U, 0U, 0U, 0U, UINT32_MAX);
}

// group 3: phase_done

void test_phase_null(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NULL, tcas_wdg_phase_done(NULL, 0U));
}

void test_phase_uninitialized_is_a_config_error(void)
{
    tcas_wdg_state_t before;

    (void)memset(&state, 0, sizeof(state));
    (void)memcpy(&before, &state, sizeof(before));

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_CONFIG,
                            tcas_wdg_phase_done(&state, 0U));
    check_unchanged(&before);
}

void test_phase_no_cycle_begun(void)
{
    /* setUp initialized the state but no cycle has begun */
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NO_CYCLE,
                            tcas_wdg_phase_done(&state, 0U));
    TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask);
}

void test_phase_in_cycle_flag_must_be_exactly_one(void)
{
    begin();
    state.in_cycle = 2U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NO_CYCLE,
                            tcas_wdg_phase_done(&state, 0U));
    TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask);
}

void test_phase_all_five_in_order(void)
{
    static const uint32_t masks[5] =
        { 0x01U, 0x03U, 0x07U, 0x0FU, 0x1FU };
    uint32_t i = 0U;

    begin();

    for (i = 0U; i < 5U; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(
            TCAS_WDG_OK, tcas_wdg_phase_done(&state, (tcas_wdg_phase_t)i));
        TEST_ASSERT_EQUAL_HEX32(masks[i], state.done_mask);
    }

    check_counters(1U, 0U, 0U, 0U, 0U, 0U);
}

void test_phase_out_of_range_and_shift_traps(void)
{
    /* 32, 33 and 255 would be an undefined shift if the driver shifted
     * before it checked the range (rule T6) */
    static const uint8_t bad[6] = { 5U, 6U, 31U, 32U, 33U, 255U };
    uint32_t i = 0U;

    begin();

    for (i = 0U; i < 6U; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_PHASE,
                                tcas_wdg_phase_done(&state, bad[i]));
        TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask);
    }

    check_counters(1U, 0U, 0U, 0U, 0U, 0U);       /* not counted as an order fault */
}

void test_phase_duplicate(void)
{
    begin();

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_phase_done(&state, 0U));
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_DUPLICATE,
                            tcas_wdg_phase_done(&state, 0U));
    TEST_ASSERT_EQUAL_HEX32(0x01U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(1U, state.order_total);

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_phase_done(&state, 1U));
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_DUPLICATE,
                            tcas_wdg_phase_done(&state, 1U));
    TEST_ASSERT_EQUAL_HEX32(0x03U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(2U, state.order_total);
}

void test_phase_order_first_phase_skipped(void)
{
    begin();

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_ORDER,
                            tcas_wdg_phase_done(&state, 1U));
    TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(1U, state.order_total);

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_ORDER,
                            tcas_wdg_phase_done(&state, 4U));
    TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(2U, state.order_total);
}

void test_phase_order_gap(void)
{
    begin();
    mark_first(2U);                                /* phases 0 and 1 */

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_ORDER,
                            tcas_wdg_phase_done(&state, 3U));
    TEST_ASSERT_EQUAL_HEX32(0x03U, state.done_mask);

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_ORDER,
                            tcas_wdg_phase_done(&state, 4U));
    TEST_ASSERT_EQUAL_HEX32(0x03U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(2U, state.order_total);
}

void test_phase_skipped_then_corrected(void)
{
    /* a skipped phase cannot be hidden by marking a later one (rule R7) */
    begin();
    mark_first(3U);                                /* phases 0, 1, 2 */

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_ORDER,
                            tcas_wdg_phase_done(&state, 4U));
    TEST_ASSERT_EQUAL_HEX32(0x07U, state.done_mask);

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_phase_done(&state, 3U));
    TEST_ASSERT_EQUAL_HEX32(0x0FU, state.done_mask);

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_phase_done(&state, 4U));
    TEST_ASSERT_EQUAL_HEX32(0x1FU, state.done_mask);

    check_counters(1U, 0U, 0U, 0U, 1U, 0U);
}

void test_phase_check_precedence(void)
{
    /* a NULL state beats everything */
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NULL, tcas_wdg_phase_done(NULL, 9U));

    /* uninitialized beats a bad phase and a missing cycle */
    (void)memset(&state, 0, sizeof(state));
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_CONFIG,
                            tcas_wdg_phase_done(&state, 9U));

    /* initialized but no cycle begun: NO_CYCLE beats a bad phase */
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_init(&state));
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NO_CYCLE,
                            tcas_wdg_phase_done(&state, 9U));

    /* a cycle has begun: now the bad phase is reported */
    begin();
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_PHASE,
                            tcas_wdg_phase_done(&state, 9U));
}

void test_phase_duplicate_is_checked_before_order(void)
{
    /* a FORCED state: phase 1 is marked but phase 0 is not. The mark of
     * phase 1 is both a duplicate and out of order; the duplicate is
     * checked first (rule T5) */
    begin();
    state.done_mask = 0x02U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_DUPLICATE,
                            tcas_wdg_phase_done(&state, 1U));
    TEST_ASSERT_EQUAL_HEX32(0x02U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(1U, state.order_total);
}

void test_phase_order_counter_saturates(void)
{
    begin();
    state.order_total = UINT32_MAX;

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_ORDER,
                            tcas_wdg_phase_done(&state, 1U));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.order_total);

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_phase_done(&state, 0U));
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_DUPLICATE,
                            tcas_wdg_phase_done(&state, 0U));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.order_total);
}

// group 4: kick

void test_kick_null_state(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NULL, tcas_wdg_kick(NULL, &hw));
    check_hw_untouched();
}

void test_kick_null_hw(void)
{
    begin();
    mark_first(5U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NULL, tcas_wdg_kick(&state, NULL));
    TEST_ASSERT_EQUAL_UINT32(0U, state.kicked);    /* nothing happened */
    check_counters(1U, 0U, 0U, 0U, 0U, 0U);
}

void test_kick_both_null(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NULL, tcas_wdg_kick(NULL, NULL));
}

void test_kick_uninitialized_is_a_config_error(void)
{
    (void)memset(&state, 0, sizeof(state));

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_CONFIG, tcas_wdg_kick(&state, &hw));
    check_hw_untouched();
}

void test_kick_initialized_flag_must_be_exactly_one(void)
{
    begin();
    mark_first(5U);
    state.initialized = 2U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_CONFIG, tcas_wdg_kick(&state, &hw));
    check_hw_untouched();
}

void test_kick_no_cycle_begun(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NO_CYCLE, tcas_wdg_kick(&state, &hw));
    check_hw_untouched();
}

void test_kick_in_cycle_flag_must_be_exactly_one(void)
{
    begin();
    mark_first(5U);
    state.in_cycle = 2U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_NO_CYCLE, tcas_wdg_kick(&state, &hw));
    check_hw_untouched();
}

void test_kick_incomplete_for_every_count_of_marked_phases(void)
{
    uint32_t n = 0U;

    for (n = 0U; n < 5U; n++)                      /* 0, 1, 2, 3, 4 marked */
    {
        TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_init(&state));
        reset_hw();
        begin();
        mark_first(n);

        TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_INCOMPLETE,
                                tcas_wdg_kick(&state, &hw));
        check_hw_untouched();
        TEST_ASSERT_EQUAL_UINT32(0U, state.kicked);
        check_counters(1U, 0U, 1U, 0U, 0U, 0U);
    }
}

void test_kick_good(void)
{
    begin();
    mark_first(5U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_kick(&state, &hw));
    TEST_ASSERT_EQUAL_HEX32(TCAS_WDG_KEY_2, hw.kick);   /* KEY_2 is written last */
    TEST_ASSERT_EQUAL_UINT32(1U, state.kicked);
    check_counters(1U, 1U, 0U, 0U, 0U, 0U);
}

void test_kick_second_in_the_same_cycle_is_refused(void)
{
    begin();
    mark_first(5U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_kick(&state, &hw));

    reset_hw();                                    /* forget the first kick */

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_REPEAT, tcas_wdg_kick(&state, &hw));
    check_hw_untouched();
    TEST_ASSERT_EQUAL_UINT32(1U, state.kicked);
    check_counters(1U, 1U, 0U, 1U, 0U, 0U);
}

void test_kick_repeat_is_checked_before_incomplete(void)
{
    /* a FORCED state: already kicked, and no phase marked. The repeat is
     * checked first (rule R8) */
    begin();
    state.kicked    = 1U;
    state.done_mask = 0U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_REPEAT, tcas_wdg_kick(&state, &hw));
    check_hw_untouched();
    check_counters(1U, 0U, 0U, 1U, 0U, 0U);
}

void test_kick_extra_mask_bits_are_incomplete_and_do_not_latch(void)
{
    /* only EXACTLY the five phase bits may kick (fail closed) */
    begin();
    mark_first(5U);

    state.done_mask = 0x3FU;                       /* a sixth bit */
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_INCOMPLETE,
                            tcas_wdg_kick(&state, &hw));
    check_hw_untouched();

    state.done_mask = 0x8000001FU;                 /* a bit far above */
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_INCOMPLETE,
                            tcas_wdg_kick(&state, &hw));
    check_hw_untouched();

    state.done_mask = 0xFFFFFFFFU;                 /* everything set */
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_INCOMPLETE,
                            tcas_wdg_kick(&state, &hw));
    check_hw_untouched();
    TEST_ASSERT_EQUAL_UINT32(3U, state.incomplete_total);

    /* the refusals did not latch: the exact mask kicks */
    state.done_mask = 0x1FU;
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_kick(&state, &hw));
    TEST_ASSERT_EQUAL_HEX32(TCAS_WDG_KEY_2, hw.kick);
    check_counters(1U, 1U, 3U, 0U, 0U, 0U);
}

void test_kick_next_cycle_works_again(void)
{
    begin();
    mark_first(5U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_kick(&state, &hw));
    reset_hw();

    begin();                                       /* a new cycle */
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_INCOMPLETE,
                            tcas_wdg_kick(&state, &hw));   /* no marks yet */
    check_hw_untouched();

    mark_first(5U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_kick(&state, &hw));
    TEST_ASSERT_EQUAL_HEX32(TCAS_WDG_KEY_2, hw.kick);
    check_counters(2U, 2U, 1U, 0U, 0U, 0U);
}

void test_kick_keeps_the_marks(void)
{
    begin();
    mark_first(5U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_kick(&state, &hw));

    TEST_ASSERT_EQUAL_HEX32(0x1FU, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(1U, state.in_cycle);
}

void test_kick_counters_saturate(void)
{
    /* kicks_total */
    state.kicks_total = UINT32_MAX;
    begin();
    mark_first(5U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_kick(&state, &hw));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.kicks_total);

    /* repeat_total */
    state.repeat_total = UINT32_MAX;
    reset_hw();
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_REPEAT, tcas_wdg_kick(&state, &hw));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.repeat_total);

    /* incomplete_total */
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_init(&state));
    state.incomplete_total = UINT32_MAX;
    reset_hw();
    begin();
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_INCOMPLETE,
                            tcas_wdg_kick(&state, &hw));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.incomplete_total);
    check_hw_untouched();
}

// group 5: missing

void test_missing_null(void)
{
    TEST_ASSERT_EQUAL_HEX32(0x1FU, tcas_wdg_missing(NULL));
}

void test_missing_uninitialized(void)
{
    (void)memset(&state, 0, sizeof(state));

    TEST_ASSERT_EQUAL_HEX32(0x1FU, tcas_wdg_missing(&state));
}

void test_missing_no_cycle_begun(void)
{
    TEST_ASSERT_EQUAL_HEX32(0x1FU, tcas_wdg_missing(&state));
}

void test_missing_right_after_begin(void)
{
    begin();

    TEST_ASSERT_EQUAL_HEX32(0x1FU, tcas_wdg_missing(&state));
}

void test_missing_for_every_count_of_marked_phases(void)
{
    static const uint32_t expected[6] =
        { 0x1FU, 0x1EU, 0x1CU, 0x18U, 0x10U, 0x00U };
    uint32_t n = 0U;

    for (n = 0U; n < 6U; n++)
    {
        TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_init(&state));
        begin();
        mark_first(n);

        TEST_ASSERT_EQUAL_HEX32(expected[n], tcas_wdg_missing(&state));
    }
}

void test_missing_ignores_bits_above_the_five_phases(void)
{
    begin();
    state.done_mask = 0xFFFFFFE3U;                 /* low five bits: 00011 */

    TEST_ASSERT_EQUAL_HEX32(0x1CU, tcas_wdg_missing(&state));
}

void test_missing_has_no_side_effects(void)
{
    tcas_wdg_state_t before;

    begin();
    mark_first(3U);
    (void)memcpy(&before, &state, sizeof(before));

    (void)tcas_wdg_missing(&state);

    check_unchanged(&before);
}

// group 6: a stuck cycle between two good ones

void test_scenario_stuck_cycle(void)
{
    /* cycle 1: everything runs, the kick goes through */
    begin();
    mark_first(5U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_kick(&state, &hw));
    TEST_ASSERT_EQUAL_HEX32(TCAS_WDG_KEY_2, hw.kick);
    reset_hw();

    /* cycle 2: the advisory phase never completes (a stuck phase) */
    begin();
    mark_first(3U);                                /* phases 0, 1, 2 only */
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_ERR_INCOMPLETE,
                            tcas_wdg_kick(&state, &hw));
    check_hw_untouched();                          /* the hardware will expire */

    /* cycle 3: the unit recovers; the missed cycle is counted */
    begin();
    TEST_ASSERT_EQUAL_UINT32(1U, state.missed_total);
    mark_first(5U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_WDG_OK, tcas_wdg_kick(&state, &hw));
    TEST_ASSERT_EQUAL_HEX32(TCAS_WDG_KEY_2, hw.kick);

    /* cycle 4: cycle 3 was kicked, so nothing more is missed */
    begin();
    check_counters(4U, 2U, 1U, 0U, 0U, 1U);
    TEST_ASSERT_EQUAL_UINT32(1U, state.in_cycle);
    TEST_ASSERT_EQUAL_UINT32(0U, state.done_mask);
    TEST_ASSERT_EQUAL_UINT32(0U, state.kicked);
}

// group 7: sweep of 7 776 mark sequences against the oracle (rule T2)

/* Every sequence of 5 marks over six symbols: the phases 0, 1, 2, 3, 4 and
 * one invalid phase (9). 6 ** 5 = 7 776 sequences. The oracle keeps a
 * COUNTER k of "phases marked so far" (the marks are always 0 .. k-1):
 *   phase invalid : ERR_PHASE, nothing changes
 *   phase <  k    : ERR_DUPLICATE
 *   phase >  k    : ERR_ORDER
 *   phase == k    : OK, k grows
 * After the sequence the kick is OK exactly when k is 5. */
void test_sweep_all_mark_sequences(void)
{
    uint32_t combos = 0U;
    uint32_t bad    = 0U;
    uint32_t seq    = 0U;

    for (seq = 0U; seq < 7776U; seq++)
    {
        tcas_wdg_state_t s;
        tcas_wdg_hw_t    h;
        uint32_t code    = seq;
        uint32_t k       = 0U;     /* oracle: phases marked so far          */
        uint32_t refused = 0U;     /* oracle: duplicates plus order faults  */
        uint32_t ok      = 1U;
        uint32_t step    = 0U;
        uint32_t expected_mask = 0U;

        h.kick = HW_SENTINEL;

        if (tcas_wdg_init(&s) != TCAS_WDG_OK)
        {
            ok = 0U;
        }
        if (tcas_wdg_begin_cycle(&s) != TCAS_WDG_OK)
        {
            ok = 0U;
        }

        for (step = 0U; step < 5U; step++)
        {
            const uint32_t digit = code % 6U;
            const tcas_wdg_phase_t phase =
                (digit == 5U) ? (tcas_wdg_phase_t)9U
                              : (tcas_wdg_phase_t)digit;
            tcas_wdg_status_t expected = TCAS_WDG_OK;

            code = code / 6U;

            if (phase >= TCAS_WDG_PHASE_COUNT)
            {
                expected = TCAS_WDG_ERR_PHASE;
            }
            else if (phase < k)
            {
                expected = TCAS_WDG_ERR_DUPLICATE;
                refused++;
            }
            else if (phase > k)
            {
                expected = TCAS_WDG_ERR_ORDER;
                refused++;
            }
            else
            {
                k++;
            }

            if (tcas_wdg_phase_done(&s, phase) != expected)
            {
                ok = 0U;
            }
        }

        expected_mask = (UINT32_C(1) << k) - UINT32_C(1);

        if (s.done_mask != expected_mask)
        {
            ok = 0U;
        }
        if (s.order_total != refused)
        {
            ok = 0U;
        }
        if (tcas_wdg_missing(&s) != (TCAS_WDG_ALL_DONE & ~expected_mask))
        {
            ok = 0U;
        }

        if (k == 5U)
        {
            /* all five phases were marked: the kick goes through */
            if ((tcas_wdg_kick(&s, &h) != TCAS_WDG_OK) ||
                (h.kick != TCAS_WDG_KEY_2) ||
                (s.kicks_total != 1U) || (s.incomplete_total != 0U))
            {
                ok = 0U;
            }

            /* a second kick in the same cycle is refused */
            h.kick = HW_SENTINEL;
            if ((tcas_wdg_kick(&s, &h) != TCAS_WDG_ERR_REPEAT) ||
                (h.kick != HW_SENTINEL) || (s.repeat_total != 1U))
            {
                ok = 0U;
            }
        }
        else
        {
            /* a phase is missing: the kick is refused, the register is not
             * written */
            if ((tcas_wdg_kick(&s, &h) != TCAS_WDG_ERR_INCOMPLETE) ||
                (h.kick != HW_SENTINEL) ||
                (s.kicks_total != 0U) || (s.incomplete_total != 1U) ||
                (s.kicked != 0U))
            {
                ok = 0U;
            }
        }

        combos++;

        if (ok != 1U)
        {
            if (bad == 0U)
            {
                (void)printf("sweep first mismatch: sequence %u\n",
                             (unsigned int)seq);
            }
            bad++;
        }
    }

    TEST_ASSERT_EQUAL_UINT32(7776U, combos);
    TEST_ASSERT_EQUAL_UINT32(0U, bad);
}

// test runner

int main(void)
{
    UNITY_BEGIN();

    /* init */
    RUN_TEST(test_init_null);
    RUN_TEST(test_init_valid);
    RUN_TEST(test_init_wipes_a_dirty_state);

    /* begin_cycle */
    RUN_TEST(test_begin_null);
    RUN_TEST(test_begin_uninitialized_is_a_config_error);
    RUN_TEST(test_begin_initialized_flag_must_be_exactly_one);
    RUN_TEST(test_begin_first_cycle);
    RUN_TEST(test_begin_clears_marks_and_kicked);
    RUN_TEST(test_begin_after_a_kicked_cycle_is_not_missed);
    RUN_TEST(test_begin_after_an_unkicked_cycle_counts_a_missed_cycle);
    RUN_TEST(test_begin_missed_cycles_accumulate);
    RUN_TEST(test_begin_counters_saturate);

    /* phase_done */
    RUN_TEST(test_phase_null);
    RUN_TEST(test_phase_uninitialized_is_a_config_error);
    RUN_TEST(test_phase_no_cycle_begun);
    RUN_TEST(test_phase_in_cycle_flag_must_be_exactly_one);
    RUN_TEST(test_phase_all_five_in_order);
    RUN_TEST(test_phase_out_of_range_and_shift_traps);
    RUN_TEST(test_phase_duplicate);
    RUN_TEST(test_phase_order_first_phase_skipped);
    RUN_TEST(test_phase_order_gap);
    RUN_TEST(test_phase_skipped_then_corrected);
    RUN_TEST(test_phase_check_precedence);
    RUN_TEST(test_phase_duplicate_is_checked_before_order);
    RUN_TEST(test_phase_order_counter_saturates);

    /* kick */
    RUN_TEST(test_kick_null_state);
    RUN_TEST(test_kick_null_hw);
    RUN_TEST(test_kick_both_null);
    RUN_TEST(test_kick_uninitialized_is_a_config_error);
    RUN_TEST(test_kick_initialized_flag_must_be_exactly_one);
    RUN_TEST(test_kick_no_cycle_begun);
    RUN_TEST(test_kick_in_cycle_flag_must_be_exactly_one);
    RUN_TEST(test_kick_incomplete_for_every_count_of_marked_phases);
    RUN_TEST(test_kick_good);
    RUN_TEST(test_kick_second_in_the_same_cycle_is_refused);
    RUN_TEST(test_kick_repeat_is_checked_before_incomplete);
    RUN_TEST(test_kick_extra_mask_bits_are_incomplete_and_do_not_latch);
    RUN_TEST(test_kick_next_cycle_works_again);
    RUN_TEST(test_kick_keeps_the_marks);
    RUN_TEST(test_kick_counters_saturate);

    /* missing */
    RUN_TEST(test_missing_null);
    RUN_TEST(test_missing_uninitialized);
    RUN_TEST(test_missing_no_cycle_begun);
    RUN_TEST(test_missing_right_after_begin);
    RUN_TEST(test_missing_for_every_count_of_marked_phases);
    RUN_TEST(test_missing_ignores_bits_above_the_five_phases);
    RUN_TEST(test_missing_has_no_side_effects);

    /* scenario and sweep */
    RUN_TEST(test_scenario_stuck_cycle);
    RUN_TEST(test_sweep_all_mark_sequences);

    return UNITY_END();
}