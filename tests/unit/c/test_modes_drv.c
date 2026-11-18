/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : tests/unit/c/test_modes_drv.c
 * Module      : Unit tests of the Mode S receive driver (Unity)
 * Language    : C11, Unity test framework
 * Assurance   : DO-178C Software Level B (target), verification artifact
 * Tests       : src/c/drivers/modes_s/tcas_modes_drv.c and .h
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Checks every behavior promised in tcas_modes_drv.h: the argument and
 *   state checks, the empty mailbox, a valid reply (short and long), each
 *   error code in its order, the fail-closed output (rule R6), the
 *   acknowledge of every reply taken (rule R7) and the saturating
 *   counters. Each test checks one thing and has a name that says what it
 *   is.
 *
 * DESIGN RULES OF THE TESTS
 *   T1  The mailbox is plain memory (see LIMITS below), so the tests need
 *       no hardware, no operating system and no timing.
 *   T2  Expected values are written as literals taken from the header
 *       (status codes, limits), not computed with the driver's code. The
 *       sweep has its own small oracle (expected_status), written from
 *       the ORDER OF CHECKS of the header.
 *   T3  Every test builds its own state and mailbox (setUp does it), so
 *       tests do not depend on each other or on their order.
 *   T4  Every error path checks the WHOLE output frame: valid = 0, every
 *       other field 0, and the status. The output starts filled with a
 *       sentinel pattern, so a field the driver forgets to write shows up.
 *   T5  The mailbox "control" register starts at an idle pattern, so the
 *       tests can tell "acknowledged" (it holds 1) from "not touched" (it
 *       still holds the pattern).
 *   T6  The mailbox data bytes hold a visible pattern (0xA0 + index) in ALL
 *       14 positions, also after a short length, so a driver that copies
 *       too much shows up as non-zero bytes after the length.
 *   T7  Some values are chosen as TRUNCATION TRAPS: a bearing of 65 536 and
 *       lengths of 263 and 270 become 0, 7 and 14 if the driver casts to a
 *       small type BEFORE checking. They must be rejected.
 *
 * TESTS BY GROUP
 *   init        : NULL, valid, dirty state, second init
 *   arguments   : each NULL pointer, all three, uninitialized state
 *   no reply    : empty mailbox, other bits set without "ready"
 *   valid       : short, long, extremes, unknown status bits
 *   overrun     : code, acknowledge, counters
 *   measurement : missing flag, bearing limits, truncation traps
 *   length      : bad lengths, truncation traps
 *   order       : overrun before measurement before length
 *   mailbox     : only "control" is written
 *   counters    : mixed sequence, saturation of each counter
 *   sweep       : 320 combinations against the oracle
 *
 * LIMITS (to be listed as an open item for v2)
 *   The fake mailbox does not change by itself: writing "control" does not
 *   clear "reply ready". The tests therefore cannot show that every
 *   register is read exactly ONCE (rule R5), or that a second receive finds
 *   the mailbox empty after an acknowledge. Each test makes one receive
 *   call (the counter test and the sweep re-prepare the mailbox between
 *   calls). A check of the read count needs a read hook in the driver or a
 *   link-time mock of the register access, the same item as in the ARINC
 *   429 test.
 *
 * USAGE (from the project root; UNITY_DIR is the folder that holds
 * unity.c and unity.h, for example the one used by the MIL1553 project)
 *   UNITY_DIR=../MIL1553/tests/unity   (adjust to your path)
 *   gcc -std=c11 -Wall -Wextra -Wpedantic \
 *       -I src/c/drivers/modes_s -I $UNITY_DIR \
 *       tests/unit/c/test_modes_drv.c \
 *       src/c/drivers/modes_s/tcas_modes_drv.c \
 *       $UNITY_DIR/unity.c -o build/bin/test_modes_drv.exe
 *   build/bin/test_modes_drv.exe
 *
 * MISRA NOTES
 *   * The rules of MISRA C apply to the driver, not to its tests. The
 *     tests use <string.h> and plain loops for clarity.
*/

#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "tcas_modes_drv.h"

// test fixtures

#define SENTINEL_BYTE  (0xA5U)               /* "this was not written"        */
#define CTRL_IDLE      UINT32_C(0x55555555)  /* control before any acknowledge */
#define DATA_PATTERN   (0xA0U)               /* data byte i holds 0xA0 + i    */

// a reply that is ready and has a valid measurement, no overrun 
#define HW_GOOD  (TCAS_MODES_HW_READY | TCAS_MODES_HW_MEAS_OK)

#define GOOD_RTT      UINT32_C(148000)
#define GOOD_BEARING  UINT32_C(9000)

static tcas_modes_state_t       state;
static tcas_modes_hw_mailbox_t  hw;
static tcas_modes_frame_t       out;

// helpers

// fills the mailbox with one reply; the data bytes get the pattern in ALL 14 positions (rule T6) and "control" gets the idle pattern 
// (rule T5) 
static void set_reply(uint32_t status, uint32_t length, uint32_t rtt,
                      uint32_t bearing)
{
    uint32_t i = 0U;

    hw.status        = status;
    hw.length        = length;
    hw.round_trip_ns = rtt;
    hw.bearing_cdeg  = bearing;
    hw.control       = CTRL_IDLE;
    hw.reserved      = 0U;

    for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
    {
        hw.data[i] = (uint8_t)(DATA_PATTERN + i);
    }
}

// fills the output frame with the "not written" pattern 
static void sentinel_out(void)
{
    (void)memset(&out, (int)SENTINEL_BYTE, sizeof(out));
}

// 1U when the frame carries nothing but this status (rule R6) 
static uint32_t frame_is_empty(const tcas_modes_frame_t * f, uint8_t status)
{
    uint32_t ok = 1U;
    uint32_t i  = 0U;

    for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
    {
        if (f->data[i] != 0U)
        {
            ok = 0U;
        }
    }

    if ((f->length != 0U) || (f->status != status) ||
        (f->round_trip_ns != 0U) || (f->bearing != 0U) ||
        (f->valid != 0U) || (f->reserved != 0U))
    {
        ok = 0U;
    }

    return ok;
}

// 1U when the frame is the valid copy of the reply set by set_reply: the pattern up to the length, zero after it 
static uint32_t frame_is_valid_copy(const tcas_modes_frame_t * f,
                                    uint32_t length, uint32_t rtt,
                                    uint32_t bearing)
{
    uint32_t ok = 1U;
    uint32_t i  = 0U;

    for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
    {
        const uint8_t expected =
            (i < length) ? (uint8_t)(DATA_PATTERN + i) : 0U;

        if (f->data[i] != expected)
        {
            ok = 0U;
        }
    }

    if ((f->length != length) || (f->status != TCAS_MODES_OK) ||
        (f->round_trip_ns != rtt) || (f->bearing != bearing) ||
        (f->valid != 1U) || (f->reserved != 0U))
    {
        ok = 0U;
    }

    return ok;
}

// checks the whole output frame of a rejected reply (rule T4) 
static void check_empty(uint8_t status)
{
    uint32_t i = 0U;

    TEST_ASSERT_EQUAL_UINT8(status, out.status);
    TEST_ASSERT_EQUAL_UINT8(0U, out.valid);
    TEST_ASSERT_EQUAL_UINT8(0U, out.length);
    TEST_ASSERT_EQUAL_UINT32(0U, out.round_trip_ns);
    TEST_ASSERT_EQUAL_UINT16(0U, out.bearing);
    TEST_ASSERT_EQUAL_UINT8(0U, out.reserved);

    for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(0U, out.data[i]);
    }
}

static void check_acked(void)
{
    TEST_ASSERT_EQUAL_HEX32(TCAS_MODES_HW_ACK, hw.control);
}

static void check_not_acked(void)
{
    TEST_ASSERT_EQUAL_HEX32(CTRL_IDLE, hw.control);
}

static void check_counters(uint32_t replies, uint32_t valid,
                           uint32_t overrun, uint32_t measure,
                           uint32_t length)
{
    TEST_ASSERT_EQUAL_UINT32(replies, state.replies_total);
    TEST_ASSERT_EQUAL_UINT32(valid,   state.valid_total);
    TEST_ASSERT_EQUAL_UINT32(overrun, state.overrun_total);
    TEST_ASSERT_EQUAL_UINT32(measure, state.measure_fail_total);
    TEST_ASSERT_EQUAL_UINT32(length,  state.length_fail_total);
}

void setUp(void)
{
    (void)memset(&state, 0, sizeof(state));
    (void)memset(&hw, 0, sizeof(hw));
    set_reply(0U, 0U, 0U, 0U);
    sentinel_out();
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK, tcas_modes_init(&state));
}

void tearDown(void)
{
}

// group 1: init

void test_init_null_state(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_NULL, tcas_modes_init(NULL));
}

void test_init_valid(void)
{
    (void)memset(&state, 0xFF, sizeof(state));

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK, tcas_modes_init(&state));
    TEST_ASSERT_EQUAL_UINT32(1U, state.initialized);
    check_counters(0U, 0U, 0U, 0U, 0U);
}

void test_init_wipes_a_dirty_state(void)
{
    state.replies_total      = 99U;
    state.valid_total        = 98U;
    state.overrun_total      = 97U;
    state.measure_fail_total = 96U;
    state.length_fail_total  = 95U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK, tcas_modes_init(&state));
    check_counters(0U, 0U, 0U, 0U, 0U);
    TEST_ASSERT_EQUAL_UINT32(1U, state.initialized);
}

// group 2: arguments and state

void test_receive_null_state(void)
{
    set_reply(HW_GOOD, 7U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_NULL,
                            tcas_modes_receive(NULL, &hw, &out));
    check_empty(TCAS_MODES_ERR_NULL);
    check_not_acked();
}

void test_receive_null_hw(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_NULL,
                            tcas_modes_receive(&state, NULL, &out));
    check_empty(TCAS_MODES_ERR_NULL);
    check_counters(0U, 0U, 0U, 0U, 0U);
}

void test_receive_null_out(void)
{
    set_reply(HW_GOOD, 7U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_NULL,
                            tcas_modes_receive(&state, &hw, NULL));
    check_not_acked();                  /* nothing was taken                  */
    check_counters(0U, 0U, 0U, 0U, 0U);
}

void test_receive_all_null(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_NULL,
                            tcas_modes_receive(NULL, NULL, NULL));
}

void test_receive_uninitialized_state_is_a_config_error(void)
{
    set_reply(HW_GOOD, 7U, GOOD_RTT, GOOD_BEARING);
    (void)memset(&state, 0, sizeof(state));          /* zero-filled: NOT valid */

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_CONFIG,
                            tcas_modes_receive(&state, &hw, &out));
    check_empty(TCAS_MODES_ERR_CONFIG);
    check_not_acked();                  /* the mailbox was not even looked at */
    check_counters(0U, 0U, 0U, 0U, 0U);
}

// group 3: no reply

void test_no_reply_empty_mailbox(void)
{
    set_reply(0U, 0U, 0U, 0U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_NO_REPLY,
                            tcas_modes_receive(&state, &hw, &out));
    check_empty(TCAS_MODES_ERR_NO_REPLY);
    check_not_acked();
    check_counters(0U, 0U, 0U, 0U, 0U);
}

void test_no_reply_other_bits_without_ready(void)
{
    /* measurement ok and overrun set, but no "ready": nothing to take */
    set_reply(TCAS_MODES_HW_MEAS_OK | TCAS_MODES_HW_OVERRUN, 7U, GOOD_RTT,
              GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_NO_REPLY,
                            tcas_modes_receive(&state, &hw, &out));
    check_empty(TCAS_MODES_ERR_NO_REPLY);
    check_not_acked();
    check_counters(0U, 0U, 0U, 0U, 0U);
}

// group 4: valid replies

void test_valid_short_reply(void)
{
    set_reply(HW_GOOD, 7U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                            tcas_modes_receive(&state, &hw, &out));
    TEST_ASSERT_EQUAL_UINT32(1U, frame_is_valid_copy(&out, 7U, GOOD_RTT,
                                                     GOOD_BEARING));
    TEST_ASSERT_EQUAL_UINT8(7U, out.length);
    TEST_ASSERT_EQUAL_HEX8(0xA0U, out.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA6U, out.data[6]);
    TEST_ASSERT_EQUAL_HEX8(0x00U, out.data[7]);    /* the mailbox holds 0xA7 */
    TEST_ASSERT_EQUAL_HEX8(0x00U, out.data[13]);   /* the mailbox holds 0xAD */
    check_acked();
    check_counters(1U, 1U, 0U, 0U, 0U);
}

void test_valid_long_reply(void)
{
    set_reply(HW_GOOD, 14U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                            tcas_modes_receive(&state, &hw, &out));
    TEST_ASSERT_EQUAL_UINT32(1U, frame_is_valid_copy(&out, 14U, GOOD_RTT,
                                                     GOOD_BEARING));
    TEST_ASSERT_EQUAL_HEX8(0xA7U, out.data[7]);
    TEST_ASSERT_EQUAL_HEX8(0xADU, out.data[13]);
    check_acked();
    check_counters(1U, 1U, 0U, 0U, 0U);
}

void test_valid_all_ones_and_all_zeros_data(void)
{
    uint32_t i = 0U;

    set_reply(HW_GOOD, 14U, GOOD_RTT, GOOD_BEARING);
    for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
    {
        hw.data[i] = 0xFFU;
    }

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                            tcas_modes_receive(&state, &hw, &out));
    for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0xFFU, out.data[i]);
    }

    set_reply(HW_GOOD, 14U, GOOD_RTT, GOOD_BEARING);
    for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
    {
        hw.data[i] = 0x00U;
    }
    sentinel_out();

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                            tcas_modes_receive(&state, &hw, &out));
    for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0x00U, out.data[i]);   /* overwrote sentinel */
    }
}

void test_valid_extremes_of_bearing_and_round_trip(void)
{
    set_reply(HW_GOOD, 7U, 0U, 0U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                            tcas_modes_receive(&state, &hw, &out));
    TEST_ASSERT_EQUAL_UINT32(1U, frame_is_valid_copy(&out, 7U, 0U, 0U));

    set_reply(HW_GOOD, 14U, UINT32_MAX, 35999U);
    sentinel_out();
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                            tcas_modes_receive(&state, &hw, &out));
    TEST_ASSERT_EQUAL_UINT32(1U, frame_is_valid_copy(&out, 14U, UINT32_MAX,
                                                     35999U));
}

void test_valid_unknown_status_bits_are_ignored(void)
{
    /* bits 3 .. 31 are not defined: they must not change the outcome */
    set_reply(HW_GOOD | UINT32_C(0xFFFFFFF8), 7U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                            tcas_modes_receive(&state, &hw, &out));
    TEST_ASSERT_EQUAL_UINT32(1U, frame_is_valid_copy(&out, 7U, GOOD_RTT,
                                                     GOOD_BEARING));
}

// group 5: overrun

void test_overrun_discards_the_reply_but_acknowledges(void)
{
    set_reply(HW_GOOD | TCAS_MODES_HW_OVERRUN, 7U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_OVERRUN,
                            tcas_modes_receive(&state, &hw, &out));
    check_empty(TCAS_MODES_ERR_OVERRUN);
    check_acked();                                  /* rule R7 */
    check_counters(1U, 0U, 1U, 0U, 0U);
}

// group 6: measurement

void test_measurement_flag_missing(void)
{
    set_reply(TCAS_MODES_HW_READY, 7U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_TIMESTAMP,
                            tcas_modes_receive(&state, &hw, &out));
    check_empty(TCAS_MODES_ERR_TIMESTAMP);
    check_acked();
    check_counters(1U, 0U, 0U, 1U, 0U);
}

void test_bearing_limits(void)
{
    set_reply(HW_GOOD, 7U, GOOD_RTT, 35999U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                            tcas_modes_receive(&state, &hw, &out));

    set_reply(HW_GOOD, 7U, GOOD_RTT, 36000U);
    sentinel_out();
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_TIMESTAMP,
                            tcas_modes_receive(&state, &hw, &out));
    check_empty(TCAS_MODES_ERR_TIMESTAMP);
    check_acked();
}

void test_bearing_truncation_traps(void)
{
    /* 65 536 is 0 in 16 bits, 65 636 is 100: both must be rejected */
    set_reply(HW_GOOD, 7U, GOOD_RTT, 65536U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_TIMESTAMP,
                            tcas_modes_receive(&state, &hw, &out));
    check_empty(TCAS_MODES_ERR_TIMESTAMP);

    set_reply(HW_GOOD, 7U, GOOD_RTT, 65636U);
    sentinel_out();
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_TIMESTAMP,
                            tcas_modes_receive(&state, &hw, &out));
    check_empty(TCAS_MODES_ERR_TIMESTAMP);

    set_reply(HW_GOOD, 7U, GOOD_RTT, UINT32_MAX);
    sentinel_out();
    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_TIMESTAMP,
                            tcas_modes_receive(&state, &hw, &out));
    check_empty(TCAS_MODES_ERR_TIMESTAMP);
    check_counters(3U, 0U, 0U, 3U, 0U);
}

// group 7: length

void test_bad_lengths(void)
{
    /* 263 is 7 in 8 bits, 270 is 14 in 8 bits: the truncation traps */
    static const uint32_t lengths[10] =
        { 0U, 1U, 6U, 8U, 13U, 15U, 255U, 263U, 270U, UINT32_MAX };
    uint32_t i = 0U;

    for (i = 0U; i < 10U; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK, tcas_modes_init(&state));
        set_reply(HW_GOOD, lengths[i], GOOD_RTT, GOOD_BEARING);
        sentinel_out();

        TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_LENGTH,
                                tcas_modes_receive(&state, &hw, &out));
        check_empty(TCAS_MODES_ERR_LENGTH);
        check_acked();
        check_counters(1U, 0U, 0U, 0U, 1U);
    }
}

// group 8: order of checks (the first failure wins)

void test_order_overrun_before_measurement(void)
{
    /* overrun AND no measurement */
    set_reply(TCAS_MODES_HW_READY | TCAS_MODES_HW_OVERRUN, 7U, GOOD_RTT,
              GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_OVERRUN,
                            tcas_modes_receive(&state, &hw, &out));
    check_counters(1U, 0U, 1U, 0U, 0U);
}

void test_order_overrun_before_length(void)
{
    set_reply(HW_GOOD | TCAS_MODES_HW_OVERRUN, 8U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_OVERRUN,
                            tcas_modes_receive(&state, &hw, &out));
    check_counters(1U, 0U, 1U, 0U, 0U);
}

void test_order_measurement_before_length(void)
{
    /* bad bearing AND bad length */
    set_reply(HW_GOOD, 8U, GOOD_RTT, 36000U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_TIMESTAMP,
                            tcas_modes_receive(&state, &hw, &out));
    check_counters(1U, 0U, 0U, 1U, 0U);
}

void test_order_everything_wrong(void)
{
    /* overrun, no measurement flag, bad bearing and bad length at once */
    set_reply(TCAS_MODES_HW_READY | TCAS_MODES_HW_OVERRUN, 8U, GOOD_RTT,
              36000U);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_OVERRUN,
                            tcas_modes_receive(&state, &hw, &out));
    check_empty(TCAS_MODES_ERR_OVERRUN);
    check_counters(1U, 0U, 1U, 0U, 0U);
}

// group 9: the mailbox is only acknowledged, never changed

void test_only_control_is_written(void)
{
    tcas_modes_hw_mailbox_t before;
    uint32_t i = 0U;

    set_reply(HW_GOOD, 14U, GOOD_RTT, GOOD_BEARING);
    (void)memcpy(&before, &hw, sizeof(before));

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                            tcas_modes_receive(&state, &hw, &out));

    TEST_ASSERT_EQUAL_HEX32(before.status, hw.status);
    TEST_ASSERT_EQUAL_UINT32(before.length, hw.length);
    TEST_ASSERT_EQUAL_UINT32(before.round_trip_ns, hw.round_trip_ns);
    TEST_ASSERT_EQUAL_UINT32(before.bearing_cdeg, hw.bearing_cdeg);
    TEST_ASSERT_EQUAL_UINT16(before.reserved, hw.reserved);

    for (i = 0U; i < TCAS_MODES_LONG_LENGTH; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(before.data[i], hw.data[i]);
    }

    check_acked();                       /* the one thing that changed */
}

// group 10: counters

void test_counters_accumulate_over_a_mixed_sequence(void)
{
    /* three valid, one overrun, one bad measurement, one bad length, then
     * one empty mailbox (which is not counted) */
    set_reply(HW_GOOD, 7U, GOOD_RTT, GOOD_BEARING);
    (void)tcas_modes_receive(&state, &hw, &out);
    set_reply(HW_GOOD, 14U, GOOD_RTT, GOOD_BEARING);
    (void)tcas_modes_receive(&state, &hw, &out);
    set_reply(HW_GOOD, 7U, GOOD_RTT, 0U);
    (void)tcas_modes_receive(&state, &hw, &out);

    set_reply(HW_GOOD | TCAS_MODES_HW_OVERRUN, 7U, GOOD_RTT, GOOD_BEARING);
    (void)tcas_modes_receive(&state, &hw, &out);
    set_reply(TCAS_MODES_HW_READY, 7U, GOOD_RTT, GOOD_BEARING);
    (void)tcas_modes_receive(&state, &hw, &out);
    set_reply(HW_GOOD, 8U, GOOD_RTT, GOOD_BEARING);
    (void)tcas_modes_receive(&state, &hw, &out);

    set_reply(0U, 0U, 0U, 0U);
    (void)tcas_modes_receive(&state, &hw, &out);

    check_counters(6U, 3U, 1U, 1U, 1U);
}

void test_counter_saturation_replies_and_valid(void)
{
    state.replies_total = UINT32_MAX;
    state.valid_total   = UINT32_MAX;
    set_reply(HW_GOOD, 7U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                            tcas_modes_receive(&state, &hw, &out));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.replies_total);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.valid_total);
}

void test_counter_saturation_overrun(void)
{
    state.replies_total = UINT32_MAX;
    state.overrun_total = UINT32_MAX;
    set_reply(HW_GOOD | TCAS_MODES_HW_OVERRUN, 7U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_OVERRUN,
                            tcas_modes_receive(&state, &hw, &out));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.replies_total);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.overrun_total);
}

void test_counter_saturation_measurement(void)
{
    state.measure_fail_total = UINT32_MAX;
    set_reply(TCAS_MODES_HW_READY, 7U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_TIMESTAMP,
                            tcas_modes_receive(&state, &hw, &out));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.measure_fail_total);
}

void test_counter_saturation_length(void)
{
    state.length_fail_total = UINT32_MAX;
    set_reply(HW_GOOD, 8U, GOOD_RTT, GOOD_BEARING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_ERR_LENGTH,
                            tcas_modes_receive(&state, &hw, &out));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.length_fail_total);
}

// group 11: sweep of 320 combinations against the oracle (rule T2)

// the expected status, written from the ORDER OF CHECKS of the header (it does not call the driver) 
static uint8_t expected_status(uint32_t ready, uint32_t overrun,
                               uint32_t meas_ok, uint32_t length,
                               uint32_t bearing)
{
    uint8_t status = TCAS_MODES_OK;

    if (ready == 0U)
    {
        status = TCAS_MODES_ERR_NO_REPLY;
    }
    else if (overrun == 1U)
    {
        status = TCAS_MODES_ERR_OVERRUN;
    }
    else if ((meas_ok == 0U) || (bearing > 35999U))
    {
        status = TCAS_MODES_ERR_TIMESTAMP;
    }
    else if ((length != 7U) && (length != 14U))
    {
        status = TCAS_MODES_ERR_LENGTH;
    }
    else
    {
        status = TCAS_MODES_OK;
    }

    return status;
}

void test_sweep_all_combinations(void)
{
    static const uint32_t lengths[10] =
        { 0U, 1U, 6U, 7U, 8U, 13U, 14U, 15U, 255U, 263U };
    static const uint32_t bearings[4] = { 0U, 35999U, 36000U, 65536U };

    uint32_t combos = 0U;
    uint32_t bad    = 0U;
    uint32_t ready  = 0U;
    uint32_t over   = 0U;
    uint32_t meas   = 0U;
    uint32_t li     = 0U;
    uint32_t bi     = 0U;

    for (ready = 0U; ready < 2U; ready++)
    {
        for (over = 0U; over < 2U; over++)
        {
            for (meas = 0U; meas < 2U; meas++)
            {
                for (li = 0U; li < 10U; li++)
                {
                    for (bi = 0U; bi < 4U; bi++)
                    {
                        uint32_t flags = 0U;
                        uint8_t  expected = TCAS_MODES_OK;
                        uint8_t  result   = TCAS_MODES_OK;
                        uint32_t ok       = 1U;

                        flags |= (ready == 1U) ? TCAS_MODES_HW_READY : 0U;
                        flags |= (over == 1U)  ? TCAS_MODES_HW_OVERRUN : 0U;
                        flags |= (meas == 1U)  ? TCAS_MODES_HW_MEAS_OK : 0U;

                        TEST_ASSERT_EQUAL_UINT8(TCAS_MODES_OK,
                                                tcas_modes_init(&state));
                        set_reply(flags, lengths[li], GOOD_RTT,
                                  bearings[bi]);
                        sentinel_out();

                        expected = expected_status(ready, over, meas,
                                                   lengths[li], bearings[bi]);
                        result   = tcas_modes_receive(&state, &hw, &out);

                        if ((result != expected) || (out.status != expected))
                        {
                            ok = 0U;
                        }

                        if (expected == TCAS_MODES_OK)
                        {
                            if (frame_is_valid_copy(&out, lengths[li],
                                                    GOOD_RTT,
                                                    bearings[bi]) != 1U)
                            {
                                ok = 0U;
                            }
                        }
                        else if (frame_is_empty(&out, expected) != 1U)
                        {
                            ok = 0U;
                        }
                        else
                        {
                            /* rejected frame is empty: fine */
                        }

                        /* acknowledged exactly when a reply was ready */
                        if ((ready == 1U) && (hw.control != TCAS_MODES_HW_ACK))
                        {
                            ok = 0U;
                        }
                        if ((ready == 0U) && (hw.control != CTRL_IDLE))
                        {
                            ok = 0U;
                        }

                        /* a reply was counted exactly when it was ready */
                        if (state.replies_total != ready)
                        {
                            ok = 0U;
                        }

                        combos++;
                        if (ok != 1U)
                        {
                            bad++;
                        }
                    }
                }
            }
        }
    }

    TEST_ASSERT_EQUAL_UINT32(320U, combos);
    TEST_ASSERT_EQUAL_UINT32(0U, bad);
}

// test runner

int main(void)
{
    UNITY_BEGIN();

    // init 
    RUN_TEST(test_init_null_state);
    RUN_TEST(test_init_valid);
    RUN_TEST(test_init_wipes_a_dirty_state);

    // arguments and state 
    RUN_TEST(test_receive_null_state);
    RUN_TEST(test_receive_null_hw);
    RUN_TEST(test_receive_null_out);
    RUN_TEST(test_receive_all_null);
    RUN_TEST(test_receive_uninitialized_state_is_a_config_error);

    // no reply 
    RUN_TEST(test_no_reply_empty_mailbox);
    RUN_TEST(test_no_reply_other_bits_without_ready);

    // valid replies
    RUN_TEST(test_valid_short_reply);
    RUN_TEST(test_valid_long_reply);
    RUN_TEST(test_valid_all_ones_and_all_zeros_data);
    RUN_TEST(test_valid_extremes_of_bearing_and_round_trip);
    RUN_TEST(test_valid_unknown_status_bits_are_ignored);

    // overrun 
    RUN_TEST(test_overrun_discards_the_reply_but_acknowledges);

    // measurement 
    RUN_TEST(test_measurement_flag_missing);
    RUN_TEST(test_bearing_limits);
    RUN_TEST(test_bearing_truncation_traps);

    // length 
    RUN_TEST(test_bad_lengths);

    // order of checks 
    RUN_TEST(test_order_overrun_before_measurement);
    RUN_TEST(test_order_overrun_before_length);
    RUN_TEST(test_order_measurement_before_length);
    RUN_TEST(test_order_everything_wrong);

    // mailbox 
    RUN_TEST(test_only_control_is_written);

    // counters 
    RUN_TEST(test_counters_accumulate_over_a_mixed_sequence);
    RUN_TEST(test_counter_saturation_replies_and_valid);
    RUN_TEST(test_counter_saturation_overrun);
    RUN_TEST(test_counter_saturation_measurement);
    RUN_TEST(test_counter_saturation_length);

    // sweep 
    RUN_TEST(test_sweep_all_combinations);

    return UNITY_END();
}