/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : tests/unit/c/test_timer.c
 * Module      : Unit tests of the cycle clock and deadlines (Unity)
 * Language    : C11, Unity test framework
 * Assurance   : DO-178C Software Level B (target), verification artifact
 * Tests       : src/c/hal/timer/tcas_timer.c and .h
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Checks every behavior promised in tcas_timer.h: the elapsed time given
 *   to the scheduler, the late-cycle test, the five phase deadlines, the
 *   earliest kick, the wrap of the 32-bit counter and the saturating
 *   counters. Each test checks one thing and has a name that says what it
 *   is.
 *
 * DESIGN RULES OF THE TESTS
 *   T1  THE TEST IS THE CLOCK. The hardware counter is plain memory (see
 *       LIMITS below), so each test writes the exact value it wants into
 *       it before each call. Every boundary is reached exactly, with no
 *       real time, no hardware and no operating system.
 *   T2  Expected values are written as LITERALS taken from the header
 *       (period, jitter, deadlines, earliest kick, saturation limit), not
 *       computed with the driver's code. The sweeps use hand-written
 *       tables of expected results, in a different form from the driver
 *       (a table instead of an if / else-if chain and a subtraction).
 *   T3  Every test builds its own state (setUp does it), so tests do not
 *       depend on each other or on their order.
 *   T4  Every error path starts its output at a sentinel pattern and checks
 *       that it came back as 0, and checks that the state was not changed
 *       (except for the counters that count the failure).
 *   T5  Wrap is tested with start values near the top of the 32-bit range
 *       (0xFFFFFF00, 0xFFFF0000, 0xFFFFF000, 0xFFFFFFFF), where "now" is
 *       numerically BELOW "start". Time is always (now - start) in
 *       unsigned arithmetic, so the answers must be the same as without a
 *       wrap.
 *   T6  The boundary of every comparison is tested on BOTH sides: one tick
 *       before, exactly at, and one tick after.
 *
 * KEY NUMBERS (microseconds, from the header)
 *   period 1 000 000    jitter 50 000    late above 1 050 000 (strict)
 *   deadlines 150 000, 450 000, 750 000, 900 000, 1 000 000 (strict)
 *   earliest kick 900 000 (a kick at exactly 900 000 is allowed)
 *   elapsed given to the scheduler: gap / 1 000, saturated at 3 600 000 ms
 *
 * TESTS BY GROUP
 *   init       : NULL, valid, dirty state
 *   deadlines  : the five values, the window lengths, invalid phases
 *   begin      : refusals, first cycle, truncation, late boundary, wrap,
 *                saturation, counters
 *   elapsed    : refusals, values, wrap, largest gap
 *   phase      : refusals, invalid phases, deadline edges, wrap, counters
 *   kick       : refusals, the 900 000 boundary, late kicks, wrap, counters
 *   scenario   : a normal cycle, a cycle with missed deadlines, a late start
 *   sweeps     : 68 begin combinations, 432 phase / kick / elapsed ones
 *
 * LIMITS (to be listed as an open item for v2)
 *   The fake counter is plain memory: it holds a value and does not count.
 *   The tests therefore cannot show that every public function reads the
 *   counter exactly ONCE (rule R5). A check of the read count needs a read
 *   hook in the driver or a link-time mock of the register access, the same
 *   item as in the other driver tests.
 *
 * USAGE (from the project root; UNITY_DIR is the folder that holds
 * unity.c and unity.h, for example the one used by the MIL1553 project)
 *   UNITY_DIR=../MIL1553/tests/unity   (adjust to your path)
 *   gcc -std=c11 -Wall -Wextra -Wpedantic \
 *       -I src/c/hal/timer -I $UNITY_DIR \
 *       tests/unit/c/test_timer.c \
 *       src/c/hal/timer/tcas_timer.c \
 *       $UNITY_DIR/unity.c -o build/bin/test_timer.exe
 *   build/bin/test_timer.exe
 *
 * MISRA NOTES
 *   * The rules of MISRA C apply to the driver, not to its tests. The
 *     tests use <string.h> and <stdio.h> and plain loops for clarity.
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "tcas_timer.h"

// test fixtures

#define SENT  UINT32_C(0xDEADBEEF)          /* "this was not written" */

static tcas_timer_state_t state;
static tcas_timer_hw_t    hw;

// a gap in microseconds and the milliseconds expected for it 
typedef struct
{
    uint32_t gap;
    uint32_t ms;
} gap_ms_t;

// the same, with the expected late flag, for the sweep 
typedef struct
{
    uint32_t gap;
    uint32_t ms;
    uint32_t late;
} gap_case_t;

// helpers

// sets the counter to t and begins a cycle 
static tcas_timer_status_t begin_at(uint32_t t, uint32_t * ms)
{
    hw.count = t;
    return tcas_timer_begin_cycle(&state, &hw, ms);
}

// the FIRST cycle after init: begins at t and asserts the result 
static void start_cycle_at(uint32_t t)
{
    uint32_t ms = SENT;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(t, &ms));
    TEST_ASSERT_EQUAL_UINT32(0U, ms);
}

// sets the counter to t and checks one phase against the expected status 
static void phase_at(uint32_t t, tcas_timer_phase_t phase,
                     tcas_timer_status_t expected)
{
    hw.count = t;
    TEST_ASSERT_EQUAL_UINT8(expected,
                            tcas_timer_check_phase(&state, &hw, phase));
}

// sets the counter to t and checks the kick request 
static void kick_at(uint32_t t, tcas_timer_status_t expected)
{
    hw.count = t;
    TEST_ASSERT_EQUAL_UINT8(expected, tcas_timer_kick_allowed(&state, &hw));
}

static void check_counters(uint32_t cycle, uint32_t late,
                           uint32_t overrun, uint32_t early)
{
    TEST_ASSERT_EQUAL_UINT32(cycle,   state.cycle);
    TEST_ASSERT_EQUAL_UINT32(late,    state.late_cycle_total);
    TEST_ASSERT_EQUAL_UINT32(overrun, state.phase_overrun_total);
    TEST_ASSERT_EQUAL_UINT32(early,   state.early_kick_total);
}

// the state is exactly what it was (a refusal changes nothing) 
static void check_unchanged(const tcas_timer_state_t * before)
{
    TEST_ASSERT_EQUAL_INT(0, memcmp(before, &state, sizeof(state)));
}

void setUp(void)
{
    (void)memset(&state, 0, sizeof(state));
    hw.count = 0U;
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, tcas_timer_init(&state));
}

void tearDown(void)
{
}

// group 1: init

void test_init_null(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL, tcas_timer_init(NULL));
}

void test_init_valid(void)
{
    (void)memset(&state, 0xFF, sizeof(state));

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, tcas_timer_init(&state));
    TEST_ASSERT_EQUAL_UINT32(1U, state.initialized);
    TEST_ASSERT_EQUAL_UINT32(0U, state.in_cycle);
    TEST_ASSERT_EQUAL_UINT32(0U, state.start_ticks);
    check_counters(0U, 0U, 0U, 0U);
}

void test_init_wipes_a_dirty_state(void)
{
    state.start_ticks         = 99U;
    state.in_cycle            = 98U;
    state.cycle               = 97U;
    state.late_cycle_total    = 96U;
    state.phase_overrun_total = 95U;
    state.early_kick_total    = 94U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, tcas_timer_init(&state));
    TEST_ASSERT_EQUAL_UINT32(1U, state.initialized);
    TEST_ASSERT_EQUAL_UINT32(0U, state.in_cycle);
    TEST_ASSERT_EQUAL_UINT32(0U, state.start_ticks);
    check_counters(0U, 0U, 0U, 0U);
}

// group 2: deadlines

void test_deadline_values(void)
{
    TEST_ASSERT_EQUAL_UINT32(150000U,  tcas_timer_deadline_us(0U));
    TEST_ASSERT_EQUAL_UINT32(450000U,  tcas_timer_deadline_us(1U));
    TEST_ASSERT_EQUAL_UINT32(750000U,  tcas_timer_deadline_us(2U));
    TEST_ASSERT_EQUAL_UINT32(900000U,  tcas_timer_deadline_us(3U));
    TEST_ASSERT_EQUAL_UINT32(1000000U, tcas_timer_deadline_us(4U));
}

void test_deadline_windows_match_the_config(void)
{
    /* the windows of Tcas.Config: 150, 300, 300, 150 and 100 ms, which add
     * up to Cycle_Ms = 1 000 ms */
    TEST_ASSERT_EQUAL_UINT32(150000U,
        tcas_timer_deadline_us(0U));
    TEST_ASSERT_EQUAL_UINT32(300000U,
        tcas_timer_deadline_us(1U) - tcas_timer_deadline_us(0U));
    TEST_ASSERT_EQUAL_UINT32(300000U,
        tcas_timer_deadline_us(2U) - tcas_timer_deadline_us(1U));
    TEST_ASSERT_EQUAL_UINT32(150000U,
        tcas_timer_deadline_us(3U) - tcas_timer_deadline_us(2U));
    TEST_ASSERT_EQUAL_UINT32(100000U,
        tcas_timer_deadline_us(4U) - tcas_timer_deadline_us(3U));
    TEST_ASSERT_EQUAL_UINT32(1000000U, tcas_timer_deadline_us(4U));
}

void test_deadline_invalid_phase_is_zero(void)
{
    static const uint8_t bad[6] = { 5U, 6U, 31U, 32U, 33U, 255U };
    uint32_t i = 0U;

    for (i = 0U; i < 6U; i++)
    {
        TEST_ASSERT_EQUAL_UINT32(0U, tcas_timer_deadline_us(bad[i]));
    }
}

// group 3: begin_cycle

void test_begin_null_state(void)
{
    uint32_t ms = SENT;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_begin_cycle(NULL, &hw, &ms));
    TEST_ASSERT_EQUAL_UINT32(0U, ms);
}

void test_begin_null_hw(void)
{
    tcas_timer_state_t before;
    uint32_t ms = SENT;

    (void)memcpy(&before, &state, sizeof(before));

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_begin_cycle(&state, NULL, &ms));
    TEST_ASSERT_EQUAL_UINT32(0U, ms);
    check_unchanged(&before);
}

void test_begin_null_elapsed(void)
{
    tcas_timer_state_t before;

    (void)memcpy(&before, &state, sizeof(before));

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_begin_cycle(&state, &hw, NULL));
    check_unchanged(&before);
}

void test_begin_all_null(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_begin_cycle(NULL, NULL, NULL));
}

void test_begin_uninitialized_is_a_config_error(void)
{
    tcas_timer_state_t before;
    uint32_t ms = SENT;

    (void)memset(&state, 0, sizeof(state));         /* zero-filled: NOT valid */
    (void)memcpy(&before, &state, sizeof(before));
    hw.count = 777U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_CONFIG,
                            tcas_timer_begin_cycle(&state, &hw, &ms));
    TEST_ASSERT_EQUAL_UINT32(0U, ms);
    check_unchanged(&before);
}

void test_begin_initialized_flag_must_be_exactly_one(void)
{
    tcas_timer_state_t before;
    uint32_t ms = SENT;

    state.initialized = 2U;
    (void)memcpy(&before, &state, sizeof(before));

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_CONFIG,
                            tcas_timer_begin_cycle(&state, &hw, &ms));
    TEST_ASSERT_EQUAL_UINT32(0U, ms);
    check_unchanged(&before);
    TEST_ASSERT_EQUAL_UINT32(0U, state.in_cycle);
}

void test_begin_first_cycle(void)
{
    uint32_t ms = SENT;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(123456U, &ms));
    TEST_ASSERT_EQUAL_UINT32(0U, ms);
    TEST_ASSERT_EQUAL_UINT32(123456U, state.start_ticks);
    TEST_ASSERT_EQUAL_UINT32(1U, state.in_cycle);
    check_counters(1U, 0U, 0U, 0U);
}

void test_begin_first_cycle_is_never_late(void)
{
    uint32_t ms = SENT;

    /* whatever the counter holds, nothing came before the first cycle */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(UINT32_MAX, &ms));
    TEST_ASSERT_EQUAL_UINT32(0U, ms);
    check_counters(1U, 0U, 0U, 0U);
}

void test_begin_regular_cycle(void)
{
    uint32_t ms = SENT;

    start_cycle_at(1000U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(1001000U, &ms));
    TEST_ASSERT_EQUAL_UINT32(1000U, ms);
    TEST_ASSERT_EQUAL_UINT32(1001000U, state.start_ticks);
    check_counters(2U, 0U, 0U, 0U);
}

void test_begin_elapsed_truncates_to_ms(void)
{
    static const gap_ms_t cases[11] =
    {
        { 0U, 0U },          { 1U, 0U },          { 999U, 0U },
        { 1000U, 1U },       { 1001U, 1U },       { 1999U, 1U },
        { 2000U, 2U },       { 999999U, 999U },   { 1000000U, 1000U },
        { 1049999U, 1049U }, { 1050000U, 1050U }
    };
    uint32_t i = 0U;

    for (i = 0U; i < 11U; i++)
    {
        uint32_t ms = SENT;

        TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, tcas_timer_init(&state));
        start_cycle_at(5000U);

        TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK,
                                begin_at(5000U + cases[i].gap, &ms));
        TEST_ASSERT_EQUAL_UINT32(cases[i].ms, ms);
    }
}

void test_begin_late_boundary(void)
{
    uint32_t ms = SENT;

    /* exactly period + jitter (1 050 000 us) is on time (strict compare) */
    start_cycle_at(10000U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(10000U + 1050000U, &ms));
    TEST_ASSERT_EQUAL_UINT32(1050U, ms);
    check_counters(2U, 0U, 0U, 0U);

    /* one tick more is late */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, tcas_timer_init(&state));
    start_cycle_at(10000U);
    ms = SENT;
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_OVERRUN,
                            begin_at(10000U + 1050001U, &ms));
    TEST_ASSERT_EQUAL_UINT32(1050U, ms);
    check_counters(2U, 1U, 0U, 0U);
}

void test_begin_late_cycle_still_begins(void)
{
    uint32_t ms = SENT;

    start_cycle_at(10000U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_OVERRUN,
                            begin_at(10000U + 1050001U, &ms));
    TEST_ASSERT_EQUAL_UINT32(1050U, ms);                /* still delivered */
    TEST_ASSERT_EQUAL_UINT32(1060001U, state.start_ticks);  /* new start   */
    TEST_ASSERT_EQUAL_UINT32(1U, state.in_cycle);
    check_counters(2U, 1U, 0U, 0U);

    /* the next cycle is measured from the late start and is on time */
    ms = SENT;
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(1060001U + 1000000U, &ms));
    TEST_ASSERT_EQUAL_UINT32(1000U, ms);
    check_counters(3U, 1U, 0U, 0U);
}

void test_begin_wrap_short_gap(void)
{
    uint32_t ms = SENT;

    /* the counter goes from 0xFFFFFF00 to 0x00000100: 512 ticks */
    start_cycle_at(UINT32_C(0xFFFFFF00));

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK,
                            begin_at(UINT32_C(0x00000100), &ms));
    TEST_ASSERT_EQUAL_UINT32(0U, ms);                   /* 512 us is 0 ms */
    TEST_ASSERT_EQUAL_UINT32(UINT32_C(0x00000100), state.start_ticks);
    check_counters(2U, 0U, 0U, 0U);
}

void test_begin_wrap_full_period(void)
{
    const uint32_t start = UINT32_C(0xFFFF0000);
    const uint32_t now   = start + UINT32_C(1000000);   /* wraps around */
    uint32_t ms = SENT;

    TEST_ASSERT_TRUE(now < start);                      /* it really wrapped */

    start_cycle_at(start);
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(now, &ms));
    TEST_ASSERT_EQUAL_UINT32(1000U, ms);
    check_counters(2U, 0U, 0U, 0U);
}

void test_begin_wrap_late(void)
{
    const uint32_t start = UINT32_C(0xFFFFF000);
    const uint32_t now   = start + UINT32_C(1050001);   /* wraps around */
    uint32_t ms = SENT;

    TEST_ASSERT_TRUE(now < start);

    start_cycle_at(start);
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_OVERRUN, begin_at(now, &ms));
    TEST_ASSERT_EQUAL_UINT32(1050U, ms);
    check_counters(2U, 1U, 0U, 0U);
}

void test_begin_elapsed_saturates(void)
{
    /* every gap here is also a late cycle; the elapsed time is still
     * delivered, saturated at 3 600 000 ms */
    static const gap_ms_t cases[6] =
    {
        { UINT32_C(3599999999), 3599999U },   /* below the limit          */
        { UINT32_C(3600000000), 3600000U },   /* exactly the limit        */
        { UINT32_C(3600000999), 3600000U },   /* truncated to the limit   */
        { UINT32_C(3600001000), 3600000U },   /* would be 3 600 001       */
        { UINT32_C(3600999999), 3600000U },   /* would be 3 600 999       */
        { UINT32_MAX,           3600000U }    /* would be 4 294 967       */
    };
    uint32_t i = 0U;

    for (i = 0U; i < 6U; i++)
    {
        uint32_t ms = SENT;

        TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, tcas_timer_init(&state));
        start_cycle_at(0U);

        TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_OVERRUN,
                                begin_at(cases[i].gap, &ms));
        TEST_ASSERT_EQUAL_UINT32(cases[i].ms, ms);
        check_counters(2U, 1U, 0U, 0U);
    }
}

void test_begin_elapsed_saturates_across_a_wrap(void)
{
    uint32_t ms = SENT;

    /* start 1, now 0: the difference is 0xFFFFFFFF ticks */
    start_cycle_at(1U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_OVERRUN, begin_at(0U, &ms));
    TEST_ASSERT_EQUAL_UINT32(3600000U, ms);
    check_counters(2U, 1U, 0U, 0U);
}

void test_begin_counters_over_a_sequence(void)
{
    uint32_t ms = SENT;

    start_cycle_at(0U);                                           /* cycle 1 */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(1000000U, &ms));  /* 2   */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_OVERRUN,
                            begin_at(2200000U, &ms));      /* 3: gap 1 200 000 */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(3200000U, &ms));  /* 4   */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_OVERRUN,
                            begin_at(5200000U, &ms));      /* 5: gap 2 000 000 */

    check_counters(5U, 2U, 0U, 0U);
}

void test_begin_counters_saturate(void)
{
    uint32_t ms = SENT;

    state.in_cycle         = 1U;
    state.start_ticks      = 0U;
    state.cycle            = UINT32_MAX;
    state.late_cycle_total = UINT32_MAX;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_OVERRUN, begin_at(2000000U, &ms));
    check_counters(UINT32_MAX, UINT32_MAX, 0U, 0U);
}

// group 4: elapsed_us

void test_elapsed_null_args(void)
{
    uint32_t us = SENT;

    start_cycle_at(0U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_elapsed_us(NULL, &hw, &us));
    TEST_ASSERT_EQUAL_UINT32(0U, us);

    us = SENT;
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_elapsed_us(&state, NULL, &us));
    TEST_ASSERT_EQUAL_UINT32(0U, us);

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_elapsed_us(&state, &hw, NULL));
}

void test_elapsed_uninitialized(void)
{
    uint32_t us = SENT;

    (void)memset(&state, 0, sizeof(state));

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_CONFIG,
                            tcas_timer_elapsed_us(&state, &hw, &us));
    TEST_ASSERT_EQUAL_UINT32(0U, us);
}

void test_elapsed_no_cycle(void)
{
    uint32_t us = SENT;

    /* setUp initialized the state but no cycle has begun */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NO_CYCLE,
                            tcas_timer_elapsed_us(&state, &hw, &us));
    TEST_ASSERT_EQUAL_UINT32(0U, us);
}

void test_elapsed_in_cycle_flag_must_be_exactly_one(void)
{
    uint32_t us = SENT;

    start_cycle_at(0U);
    state.in_cycle = 2U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NO_CYCLE,
                            tcas_timer_elapsed_us(&state, &hw, &us));
    TEST_ASSERT_EQUAL_UINT32(0U, us);
}

void test_elapsed_values_and_no_side_effects(void)
{
    static const uint32_t offsets[5] =
        { 0U, 1U, 250000U, 1000000U, 5000000U };
    tcas_timer_state_t before;
    uint32_t i = 0U;

    start_cycle_at(1000U);
    (void)memcpy(&before, &state, sizeof(before));

    for (i = 0U; i < 5U; i++)
    {
        uint32_t us = SENT;

        hw.count = 1000U + offsets[i];
        TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK,
                                tcas_timer_elapsed_us(&state, &hw, &us));
        TEST_ASSERT_EQUAL_UINT32(offsets[i], us);
        check_unchanged(&before);
    }
}

void test_elapsed_wrap(void)
{
    uint32_t us = SENT;

    start_cycle_at(UINT32_C(0xFFFFFF00));

    hw.count = UINT32_C(0x00000100);                    /* wrapped */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK,
                            tcas_timer_elapsed_us(&state, &hw, &us));
    TEST_ASSERT_EQUAL_UINT32(512U, us);

    hw.count = UINT32_C(0xFFFFFF00);                    /* the same instant */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK,
                            tcas_timer_elapsed_us(&state, &hw, &us));
    TEST_ASSERT_EQUAL_UINT32(0U, us);

    hw.count = UINT32_C(0xFFFFFFFF);                    /* just before it */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK,
                            tcas_timer_elapsed_us(&state, &hw, &us));
    TEST_ASSERT_EQUAL_UINT32(255U, us);
}

void test_elapsed_largest(void)
{
    uint32_t us = SENT;

    start_cycle_at(0U);
    hw.count = UINT32_MAX;
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK,
                            tcas_timer_elapsed_us(&state, &hw, &us));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, us);

    /* start 1, now 0: the difference is also 0xFFFFFFFF */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, tcas_timer_init(&state));
    start_cycle_at(1U);
    hw.count = 0U;
    us = SENT;
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK,
                            tcas_timer_elapsed_us(&state, &hw, &us));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, us);
}

// group 5: check_phase

void test_phase_null_args(void)
{
    start_cycle_at(0U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_check_phase(NULL, &hw, 0U));
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_check_phase(&state, NULL, 0U));
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_check_phase(NULL, NULL, 0U));
    check_counters(1U, 0U, 0U, 0U);
}

void test_phase_uninitialized(void)
{
    tcas_timer_state_t before;

    (void)memset(&state, 0, sizeof(state));
    (void)memcpy(&before, &state, sizeof(before));
    hw.count = 5000000U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_CONFIG,
                            tcas_timer_check_phase(&state, &hw, 0U));
    check_unchanged(&before);
}

void test_phase_no_cycle(void)
{
    hw.count = 5000000U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NO_CYCLE,
                            tcas_timer_check_phase(&state, &hw, 0U));
    check_counters(0U, 0U, 0U, 0U);
}

void test_phase_in_cycle_flag_must_be_exactly_one(void)
{
    start_cycle_at(0U);
    state.in_cycle = 2U;
    hw.count = 5000000U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NO_CYCLE,
                            tcas_timer_check_phase(&state, &hw, 0U));
    check_counters(1U, 0U, 0U, 0U);
}

void test_phase_invalid_numbers(void)
{
    static const uint8_t bad[6] = { 5U, 6U, 31U, 32U, 33U, 255U };
    uint32_t i = 0U;

    start_cycle_at(0U);
    hw.count = 5000000U;           /* past EVERY deadline: only the phase
                                    * check can explain ERR_PHASE */

    for (i = 0U; i < 6U; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_PHASE,
                                tcas_timer_check_phase(&state, &hw, bad[i]));
    }

    check_counters(1U, 0U, 0U, 0U);       /* none counted as overruns */
}

void test_phase_check_precedence(void)
{
    /* a NULL state beats everything */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_check_phase(NULL, &hw, 9U));

    /* uninitialized beats a bad phase */
    (void)memset(&state, 0, sizeof(state));
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_CONFIG,
                            tcas_timer_check_phase(&state, &hw, 9U));

    /* initialized but no cycle begun: NO_CYCLE beats a bad phase */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, tcas_timer_init(&state));
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NO_CYCLE,
                            tcas_timer_check_phase(&state, &hw, 9U));

    /* a cycle has begun: now the bad phase is reported */
    start_cycle_at(0U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_PHASE,
                            tcas_timer_check_phase(&state, &hw, 9U));
}

void test_phase_deadline_edges(void)
{
    static const uint32_t d[5] =
        { 150000U, 450000U, 750000U, 900000U, 1000000U };
    uint32_t p = 0U;

    for (p = 0U; p < 5U; p++)
    {
        TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, tcas_timer_init(&state));
        start_cycle_at(100U);

        phase_at(100U + d[p] - 1U, (tcas_timer_phase_t)p, TCAS_TIMER_OK);
        phase_at(100U + d[p],      (tcas_timer_phase_t)p, TCAS_TIMER_OK);
        check_counters(1U, 0U, 0U, 0U);

        phase_at(100U + d[p] + 1U, (tcas_timer_phase_t)p,
                 TCAS_TIMER_ERR_OVERRUN);
        check_counters(1U, 0U, 1U, 0U);
    }
}

void test_phase_earlier_deadlines_do_not_apply_to_later_phases(void)
{
    start_cycle_at(0U);

    /* at 200 000: only phase 0 (150 000) has passed its deadline */
    phase_at(200000U, 0U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(200000U, 1U, TCAS_TIMER_OK);
    phase_at(200000U, 2U, TCAS_TIMER_OK);
    phase_at(200000U, 3U, TCAS_TIMER_OK);
    phase_at(200000U, 4U, TCAS_TIMER_OK);
    check_counters(1U, 0U, 1U, 0U);

    /* at 500 000: phases 0 and 1 (450 000) */
    phase_at(500000U, 0U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(500000U, 1U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(500000U, 2U, TCAS_TIMER_OK);
    phase_at(500000U, 3U, TCAS_TIMER_OK);
    phase_at(500000U, 4U, TCAS_TIMER_OK);
    check_counters(1U, 0U, 3U, 0U);

    /* at 800 000: phases 0, 1 and 2 (750 000) */
    phase_at(800000U, 0U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(800000U, 1U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(800000U, 2U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(800000U, 3U, TCAS_TIMER_OK);
    phase_at(800000U, 4U, TCAS_TIMER_OK);
    check_counters(1U, 0U, 6U, 0U);

    /* at 950 000: phases 0 .. 3 (900 000) */
    phase_at(950000U, 0U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(950000U, 1U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(950000U, 2U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(950000U, 3U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(950000U, 4U, TCAS_TIMER_OK);
    check_counters(1U, 0U, 10U, 0U);

    /* at 1 000 001: every phase */
    phase_at(1000001U, 0U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(1000001U, 1U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(1000001U, 2U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(1000001U, 3U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(1000001U, 4U, TCAS_TIMER_ERR_OVERRUN);
    check_counters(1U, 0U, 15U, 0U);
}

void test_phase_at_time_zero_all_ok(void)
{
    uint32_t p = 0U;

    start_cycle_at(7U);

    for (p = 0U; p < 5U; p++)
    {
        phase_at(7U, (tcas_timer_phase_t)p, TCAS_TIMER_OK);
    }

    check_counters(1U, 0U, 0U, 0U);
}

void test_phase_wrap(void)
{
    const uint32_t start = UINT32_C(0xFFFFFF00);

    start_cycle_at(start);

    /* phase 0 deadline is 150 000: exactly at it is on time */
    phase_at(start + 150000U, 0U, TCAS_TIMER_OK);
    TEST_ASSERT_TRUE((start + 150000U) < start);        /* it wrapped */
    phase_at(start + 150001U, 0U, TCAS_TIMER_ERR_OVERRUN);
    check_counters(1U, 0U, 1U, 0U);
}

void test_phase_does_not_change_the_cycle(void)
{
    start_cycle_at(1000U);

    phase_at(1000U + 200000U, 0U, TCAS_TIMER_ERR_OVERRUN);
    phase_at(1000U + 100U,    4U, TCAS_TIMER_OK);

    TEST_ASSERT_EQUAL_UINT32(1000U, state.start_ticks);
    TEST_ASSERT_EQUAL_UINT32(1U, state.in_cycle);
    TEST_ASSERT_EQUAL_UINT32(1U, state.cycle);
    TEST_ASSERT_EQUAL_UINT32(0U, state.late_cycle_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.early_kick_total);
}

void test_phase_overrun_counter_saturates(void)
{
    state.phase_overrun_total = UINT32_MAX;
    start_cycle_at(0U);

    phase_at(5000000U, 0U, TCAS_TIMER_ERR_OVERRUN);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.phase_overrun_total);
}

// group 6: kick_allowed

void test_kick_null_args(void)
{
    start_cycle_at(0U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_kick_allowed(NULL, &hw));
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_kick_allowed(&state, NULL));
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NULL,
                            tcas_timer_kick_allowed(NULL, NULL));
    check_counters(1U, 0U, 0U, 0U);
}

void test_kick_uninitialized(void)
{
    tcas_timer_state_t before;

    (void)memset(&state, 0, sizeof(state));
    (void)memcpy(&before, &state, sizeof(before));
    hw.count = 5000000U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_CONFIG,
                            tcas_timer_kick_allowed(&state, &hw));
    check_unchanged(&before);
}

void test_kick_no_cycle(void)
{
    hw.count = 5000000U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NO_CYCLE,
                            tcas_timer_kick_allowed(&state, &hw));
    check_counters(0U, 0U, 0U, 0U);
}

void test_kick_in_cycle_flag_must_be_exactly_one(void)
{
    start_cycle_at(0U);
    state.in_cycle = 2U;
    hw.count = 5000000U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_NO_CYCLE,
                            tcas_timer_kick_allowed(&state, &hw));
    check_counters(1U, 0U, 0U, 0U);
}

void test_kick_boundaries(void)
{
    start_cycle_at(500U);

    kick_at(500U,             TCAS_TIMER_ERR_EARLY);    /* at the very start */
    kick_at(500U + 899999U,   TCAS_TIMER_ERR_EARLY);    /* one tick early    */
    check_counters(1U, 0U, 0U, 2U);

    kick_at(500U + 900000U,   TCAS_TIMER_OK);           /* exactly 900 000   */
    kick_at(500U + 900001U,   TCAS_TIMER_OK);
    kick_at(500U + 1000000U,  TCAS_TIMER_OK);
    kick_at(500U + 5000000U,  TCAS_TIMER_OK);           /* LATE is not refused */
    check_counters(1U, 0U, 0U, 2U);                     /* only early counted */
}

void test_kick_wrap(void)
{
    const uint32_t start = UINT32_C(0xFFFFFF00);

    start_cycle_at(start);

    kick_at(start + 899999U, TCAS_TIMER_ERR_EARLY);
    kick_at(start + 900000U, TCAS_TIMER_OK);
    TEST_ASSERT_TRUE((start + 900000U) < start);        /* it wrapped */
    check_counters(1U, 0U, 0U, 1U);
}

void test_kick_changes_only_its_counter(void)
{
    start_cycle_at(1000U);

    kick_at(1000U + 100U, TCAS_TIMER_ERR_EARLY);
    kick_at(1000U + 950000U, TCAS_TIMER_OK);

    TEST_ASSERT_EQUAL_UINT32(1000U, state.start_ticks);
    TEST_ASSERT_EQUAL_UINT32(1U, state.in_cycle);
    TEST_ASSERT_EQUAL_UINT32(1U, state.cycle);
    TEST_ASSERT_EQUAL_UINT32(0U, state.late_cycle_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.phase_overrun_total);
    TEST_ASSERT_EQUAL_UINT32(1U, state.early_kick_total);
}

void test_kick_early_counter_saturates(void)
{
    state.early_kick_total = UINT32_MAX;
    start_cycle_at(0U);

    kick_at(100U, TCAS_TIMER_ERR_EARLY);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.early_kick_total);
}

// group 7: a normal cycle, a cycle with missed deadlines, a late start

void test_scenario_normal_stalled_and_late_cycles(void)
{
    uint32_t ms = SENT;

    /* cycle 1 begins at 0 and runs inside every deadline */
    start_cycle_at(0U);
    phase_at(100000U, 0U, TCAS_TIMER_OK);
    phase_at(400000U, 1U, TCAS_TIMER_OK);
    phase_at(700000U, 2U, TCAS_TIMER_OK);
    phase_at(850000U, 3U, TCAS_TIMER_OK);
    phase_at(960000U, 4U, TCAS_TIMER_OK);
    kick_at(960000U, TCAS_TIMER_OK);

    /* cycle 2 begins on time; phases 3 and 4 miss their deadlines (920 000
     * and 1 020 000 us into the cycle) but the kick at 930 000 is allowed:
     * a late kick is not refused here */
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(1000000U, &ms));
    TEST_ASSERT_EQUAL_UINT32(1000U, ms);
    phase_at(1100000U, 0U, TCAS_TIMER_OK);
    phase_at(1400000U, 1U, TCAS_TIMER_OK);
    phase_at(1700000U, 2U, TCAS_TIMER_OK);
    phase_at(1920000U, 3U, TCAS_TIMER_ERR_OVERRUN);
    kick_at(1930000U, TCAS_TIMER_OK);
    phase_at(2020000U, 4U, TCAS_TIMER_ERR_OVERRUN);

    /* cycle 3 begins 1 100 000 us after cycle 2: late, but it begins and
     * the elapsed time (1 100 ms) is delivered */
    ms = SENT;
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_ERR_OVERRUN, begin_at(2100000U, &ms));
    TEST_ASSERT_EQUAL_UINT32(1100U, ms);

    /* cycle 4 is back to normal */
    ms = SENT;
    TEST_ASSERT_EQUAL_UINT8(TCAS_TIMER_OK, begin_at(3100000U, &ms));
    TEST_ASSERT_EQUAL_UINT32(1000U, ms);

    check_counters(4U, 1U, 2U, 0U);
}

// group 8: sweeps against hand-written tables (rule T2)

/* 4 starting counter values x 17 gaps = 68 combinations. The expected
 * milliseconds and late flag of each gap are written by hand (see the
 * header: truncation to ms, saturation at 3 600 000, late above 1 050 000
 * us). Starting values near the top of the range make most gaps wrap. 
*/
void test_sweep_begin_cycle(void)
{
    static const gap_case_t gaps[17] =
    {
        { UINT32_C(0),          0U,       0U },
        { UINT32_C(1),          0U,       0U },
        { UINT32_C(999),        0U,       0U },
        { UINT32_C(1000),       1U,       0U },
        { UINT32_C(1001),       1U,       0U },
        { UINT32_C(999999),     999U,     0U },
        { UINT32_C(1000000),    1000U,    0U },
        { UINT32_C(1049999),    1049U,    0U },
        { UINT32_C(1050000),    1050U,    0U },   /* on time, at the edge  */
        { UINT32_C(1050001),    1050U,    1U },   /* late, one tick over   */
        { UINT32_C(1999999),    1999U,    1U },
        { UINT32_C(2000000),    2000U,    1U },
        { UINT32_C(3599999999), 3599999U, 1U },
        { UINT32_C(3600000000), 3600000U, 1U },
        { UINT32_C(3600000999), 3600000U, 1U },
        { UINT32_C(3600001000), 3600000U, 1U },   /* saturated             */
        { UINT32_MAX,           3600000U, 1U }    /* saturated             */
    };
    static const uint32_t starts[4] =
        { UINT32_C(0), UINT32_C(12345), UINT32_C(0x80000000),
          UINT32_C(0xFFFFFFFF) };

    uint32_t combos = 0U;
    uint32_t bad    = 0U;
    uint32_t si     = 0U;
    uint32_t gi     = 0U;

    for (si = 0U; si < 4U; si++)
    {
        for (gi = 0U; gi < 17U; gi++)
        {
            uint32_t ok  = 1U;
            uint32_t ms  = SENT;
            const uint32_t now = starts[si] + gaps[gi].gap;   /* may wrap */
            const tcas_timer_status_t expected =
                (gaps[gi].late == 1U) ? TCAS_TIMER_ERR_OVERRUN
                                      : TCAS_TIMER_OK;
            tcas_timer_status_t st = TCAS_TIMER_OK;

            if (tcas_timer_init(&state) != TCAS_TIMER_OK)
            {
                ok = 0U;
            }

            /* the first cycle: 0 ms, on time */
            hw.count = starts[si];
            if ((tcas_timer_begin_cycle(&state, &hw, &ms) != TCAS_TIMER_OK) ||
                (ms != 0U))
            {
                ok = 0U;
            }

            /* the second cycle, one gap later */
            ms = SENT;
            hw.count = now;
            st = tcas_timer_begin_cycle(&state, &hw, &ms);

            if ((st != expected) || (ms != gaps[gi].ms) ||
                (state.start_ticks != now) || (state.in_cycle != 1U) ||
                (state.cycle != 2U) ||
                (state.late_cycle_total != gaps[gi].late))
            {
                ok = 0U;
            }

            combos++;

            if (ok != 1U)
            {
                if (bad == 0U)
                {
                    (void)printf("sweep first mismatch: start %u gap %u\n",
                                 (unsigned int)starts[si],
                                 (unsigned int)gaps[gi].gap);
                }
                bad++;
            }
        }
    }

    TEST_ASSERT_EQUAL_UINT32(68U, combos);
    TEST_ASSERT_EQUAL_UINT32(0U, bad);
}

/* 4 starting values x 18 elapsed times x 6 phases (0 .. 4 and the invalid
 * 9) = 432 combinations. For each, the phase check, the kick request and
 * the elapsed time are compared with a hand-written table of deadlines:
 *   phase invalid : ERR_PHASE, no overrun counted
 *   elapsed > deadline of the phase : ERR_OVERRUN, one overrun counted
 *   elapsed < 900 000 : the kick is ERR_EARLY, one early counted
 *   elapsed_us equals the elapsed time 
*/
void test_sweep_phase_kick_and_elapsed(void)
{
    static const uint32_t starts[4] =
        { UINT32_C(0), UINT32_C(0xFFFFFF00), UINT32_C(0x80000000),
          UINT32_C(0xFFFFFFFF) };
    static const uint32_t times[18] =
        { 0U, 1U, 149999U, 150000U, 150001U, 449999U, 450000U, 450001U,
          749999U, 750000U, 750001U, 899999U, 900000U, 900001U, 999999U,
          1000000U, 1000001U, 2000000U };
    static const uint8_t phases[6] = { 0U, 1U, 2U, 3U, 4U, 9U };
    static const uint32_t deadline[5] =
        { 150000U, 450000U, 750000U, 900000U, 1000000U };

    uint32_t combos = 0U;
    uint32_t bad    = 0U;
    uint32_t si     = 0U;
    uint32_t ti     = 0U;
    uint32_t pi     = 0U;

    for (si = 0U; si < 4U; si++)
    {
        for (ti = 0U; ti < 18U; ti++)
        {
            for (pi = 0U; pi < 6U; pi++)
            {
                uint32_t ok = 1U;
                uint32_t ms = SENT;
                uint32_t us = SENT;
                const uint32_t now = starts[si] + times[ti];   /* may wrap */
                const uint8_t phase = phases[pi];

                const tcas_timer_status_t expected_phase =
                    (phase > 4U) ? TCAS_TIMER_ERR_PHASE
                    : ((times[ti] > deadline[phase]) ? TCAS_TIMER_ERR_OVERRUN
                                                     : TCAS_TIMER_OK);
                const uint32_t expected_overrun =
                    (expected_phase == TCAS_TIMER_ERR_OVERRUN) ? 1U : 0U;
                const tcas_timer_status_t expected_kick =
                    (times[ti] < 900000U) ? TCAS_TIMER_ERR_EARLY
                                          : TCAS_TIMER_OK;
                const uint32_t expected_early =
                    (expected_kick == TCAS_TIMER_ERR_EARLY) ? 1U : 0U;

                if (tcas_timer_init(&state) != TCAS_TIMER_OK)
                {
                    ok = 0U;
                }

                hw.count = starts[si];
                if ((tcas_timer_begin_cycle(&state, &hw, &ms) !=
                     TCAS_TIMER_OK) || (ms != 0U))
                {
                    ok = 0U;
                }

                hw.count = now;

                if (tcas_timer_check_phase(&state, &hw, phase) !=
                    expected_phase)
                {
                    ok = 0U;
                }
                if (state.phase_overrun_total != expected_overrun)
                {
                    ok = 0U;
                }

                if (tcas_timer_kick_allowed(&state, &hw) != expected_kick)
                {
                    ok = 0U;
                }
                if (state.early_kick_total != expected_early)
                {
                    ok = 0U;
                }

                if ((tcas_timer_elapsed_us(&state, &hw, &us) !=
                     TCAS_TIMER_OK) || (us != times[ti]))
                {
                    ok = 0U;
                }

                combos++;

                if (ok != 1U)
                {
                    if (bad == 0U)
                    {
                        (void)printf("sweep first mismatch: start %u "
                                     "elapsed %u phase %u\n",
                                     (unsigned int)starts[si],
                                     (unsigned int)times[ti],
                                     (unsigned int)phase);
                    }
                    bad++;
                }
            }
        }
    }

    TEST_ASSERT_EQUAL_UINT32(432U, combos);
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

    /* deadlines */
    RUN_TEST(test_deadline_values);
    RUN_TEST(test_deadline_windows_match_the_config);
    RUN_TEST(test_deadline_invalid_phase_is_zero);

    /* begin_cycle */
    RUN_TEST(test_begin_null_state);
    RUN_TEST(test_begin_null_hw);
    RUN_TEST(test_begin_null_elapsed);
    RUN_TEST(test_begin_all_null);
    RUN_TEST(test_begin_uninitialized_is_a_config_error);
    RUN_TEST(test_begin_initialized_flag_must_be_exactly_one);
    RUN_TEST(test_begin_first_cycle);
    RUN_TEST(test_begin_first_cycle_is_never_late);
    RUN_TEST(test_begin_regular_cycle);
    RUN_TEST(test_begin_elapsed_truncates_to_ms);
    RUN_TEST(test_begin_late_boundary);
    RUN_TEST(test_begin_late_cycle_still_begins);
    RUN_TEST(test_begin_wrap_short_gap);
    RUN_TEST(test_begin_wrap_full_period);
    RUN_TEST(test_begin_wrap_late);
    RUN_TEST(test_begin_elapsed_saturates);
    RUN_TEST(test_begin_elapsed_saturates_across_a_wrap);
    RUN_TEST(test_begin_counters_over_a_sequence);
    RUN_TEST(test_begin_counters_saturate);

    /* elapsed_us */
    RUN_TEST(test_elapsed_null_args);
    RUN_TEST(test_elapsed_uninitialized);
    RUN_TEST(test_elapsed_no_cycle);
    RUN_TEST(test_elapsed_in_cycle_flag_must_be_exactly_one);
    RUN_TEST(test_elapsed_values_and_no_side_effects);
    RUN_TEST(test_elapsed_wrap);
    RUN_TEST(test_elapsed_largest);

    /* check_phase */
    RUN_TEST(test_phase_null_args);
    RUN_TEST(test_phase_uninitialized);
    RUN_TEST(test_phase_no_cycle);
    RUN_TEST(test_phase_in_cycle_flag_must_be_exactly_one);
    RUN_TEST(test_phase_invalid_numbers);
    RUN_TEST(test_phase_check_precedence);
    RUN_TEST(test_phase_deadline_edges);
    RUN_TEST(test_phase_earlier_deadlines_do_not_apply_to_later_phases);
    RUN_TEST(test_phase_at_time_zero_all_ok);
    RUN_TEST(test_phase_wrap);
    RUN_TEST(test_phase_does_not_change_the_cycle);
    RUN_TEST(test_phase_overrun_counter_saturates);

    /* kick_allowed */
    RUN_TEST(test_kick_null_args);
    RUN_TEST(test_kick_uninitialized);
    RUN_TEST(test_kick_no_cycle);
    RUN_TEST(test_kick_in_cycle_flag_must_be_exactly_one);
    RUN_TEST(test_kick_boundaries);
    RUN_TEST(test_kick_wrap);
    RUN_TEST(test_kick_changes_only_its_counter);
    RUN_TEST(test_kick_early_counter_saturates);

    /* scenario and sweeps */
    RUN_TEST(test_scenario_normal_stalled_and_late_cycles);
    RUN_TEST(test_sweep_begin_cycle);
    RUN_TEST(test_sweep_phase_kick_and_elapsed);

    return UNITY_END();
}