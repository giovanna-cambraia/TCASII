/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : tests/unit/c/test_display_drv.c
 * Module      : Unit tests of the display and audio output driver (Unity)
 * Language    : C11, Unity test framework
 * Assurance   : DO-178C Software Level B (target), verification artifact
 * Tests       : src/c/drivers/display/tcas_display_drv.c and .h
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Checks every behavior promised in tcas_display_drv.h: the independent
 *   frame check (the same rules as Display_Port.Is_Well_Formed), the exact
 *   register words written for a frame, the handling of signed fields, the
 *   refusal of a bad frame or a busy unit without touching a register, the
 *   FAIL annunciation and the saturating counters. Each test checks one
 *   thing and has a name that says what it is.
 *
 * DESIGN RULES OF THE TESTS
 *   T1  The hardware is plain memory (see LIMITS below), so the tests need
 *       no hardware, no operating system and no timing.
 *   T2  Expected register words are worked out by hand and written as
 *       literals (see HAND-WORKED WORDS), not computed with the driver's
 *       code. The sweep has its own oracle (expected_ok), written from the
 *       RULES of the header as a list of violations, in a different form
 *       from the driver's chain of helpers.
 *   T3  Every test builds its own state, register block and frame (setUp
 *       does it), so tests do not depend on each other or on their order.
 *   T4  The seven writable registers start at a sentinel pattern. A refusal
 *       must leave ALL of them at the sentinel; a good show or fail must
 *       change exactly the registers that the header lists.
 *   T5  The status register starts at READY. A test that needs a busy unit
 *       clears it.
 *   T6  Signed values are tested at both ends and around zero: -1 must not
 *       set the upper half of the rate register, and a negative relative
 *       altitude must be the 32-bit pattern of the value (rule R7).
 *
 * HAND-WORKED WORDS
 *   flags = mode | advisory << 8 | class << 16 | valid << 24
 *     traffic advisory (1, 0, 2, 1) = 0x01020001
 *     climb            (2, 1, 3, 1) = 0x01030102
 *     descend          (2, 2, 3, 1) = 0x01030202
 *   rate register = the low 16 bits of the signed rate, upper half 0
 *     +1 500  = 0x000005DC            -1     = 0x0000FFFF
 *     -1 500  = 0x10000 - 0x5DC  = 0x0000FA24
 *     -10 000 = 0x10000 - 0x2710 = 0x0000D8F0
 *   relative altitude register = the 32-bit two's complement pattern
 *     +127 900 = 0x0001F39C
 *     -127 900 = 0x100000000 - 0x1F39C = 0xFFFE0C64
 *     -1       = 0xFFFFFFFF
 *   control: COMMIT = 1, COMMIT | FAIL = 3
 *
 * TESTS BY GROUP
 *   init      : NULL, valid, dirty state
 *   frame_ok  : every rule, every boundary, the audio matrix
 *   show      : exact words, signed values, limits, flags, refusals
 *   fail      : words written, refusals, precedence
 *   counters  : saturation of each counter
 *   sweep     : 1 512 frames against the oracle
 *
 * LIMITS (to be listed as an open item for v2)
 *   The fake registers are plain memory: they hold the last value written
 *   and do not record the order. The tests therefore cannot show that the
 *   commit is written LAST, or that the status register is read exactly
 *   once (rule R6). They show the final register values and that a
 *   refusal writes nothing. An ordered-write check needs a recorder like
 *   the one in test_mpu.c, or a hook in the driver.
 *
 * USAGE (from the project root; UNITY_DIR is the folder that holds
 * unity.c and unity.h, for example the one used by the MIL1553 project)
 *   UNITY_DIR=../MIL1553/tests/unity   (adjust to your path)
 *   gcc -std=c11 -Wall -Wextra -Wpedantic \
 *       -I src/c/drivers/display -I $UNITY_DIR \
 *       tests/unit/c/test_display_drv.c \
 *       src/c/drivers/display/tcas_display_drv.c \
 *       $UNITY_DIR/unity.c -o build/bin/test_display_drv.exe
 *   build/bin/test_display_drv.exe
 *
 * MISRA NOTES
 *   * The rules of MISRA C apply to the driver, not to its tests. The tests
 *     use <string.h> and <stdio.h> and plain loops for clarity.
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "tcas_display_drv.h"

// test fixtures

#define SENT   UINT32_C(0xDEADBEEF)          /* "this register was not written" */
#define READY  TCAS_DISPLAY_HW_READY

static tcas_display_state_t state;
static tcas_display_hw_t    hw;
static tcas_display_frame_t f;

// helpers

// the seven writable registers at the sentinel, the unit ready (T4, T5) 
static void reset_hw(void)
{
    hw.status  = READY;
    hw.slant   = SENT;
    hw.rel_alt = SENT;
    hw.bearing = SENT;
    hw.rate    = SENT;
    hw.flags   = SENT;
    hw.audio   = SENT;
    hw.control = SENT;
}

// a good Climb with an intruder at the same altitude 
static void make_climb(tcas_display_frame_t * p)
{
    (void)memset(p, 0, sizeof(*p));
    p->slant_ft        = 3000U;
    p->rel_alt_ft      = 0;
    p->bearing_cdeg    = 9000U;
    p->target_rate_fpm = 1500;
    p->mode            = TCAS_DISPLAY_MODE_TA_RA;
    p->advisory        = TCAS_DISPLAY_ADV_CLIMB;
    p->audio           = TCAS_DISPLAY_AUDIO_CLIMB;
    p->intruder_class  = TCAS_DISPLAY_CLASS_RESOLUTION;
    p->intruder_valid  = 1U;
}

// a good Descend 
static void make_descend(tcas_display_frame_t * p)
{
    make_climb(p);
    p->target_rate_fpm = -1500;
    p->advisory        = TCAS_DISPLAY_ADV_DESCEND;
    p->audio           = TCAS_DISPLAY_AUDIO_DESCEND;
}

// a good frame with nothing to show: TA only mode, no advisory, no intruder, silence 
static void make_none(tcas_display_frame_t * p)
{
    (void)memset(p, 0, sizeof(*p));
    p->mode = TCAS_DISPLAY_MODE_TA_ONLY;
}

// a good traffic advisory: intruder 750 ft above, the phrase "Traffic" 
static void make_ta(tcas_display_frame_t * p)
{
    (void)memset(p, 0, sizeof(*p));
    p->slant_ft       = 3000U;
    p->rel_alt_ft     = 750;
    p->bearing_cdeg   = 9000U;
    p->mode           = TCAS_DISPLAY_MODE_TA_ONLY;
    p->audio          = TCAS_DISPLAY_AUDIO_TRAFFIC;
    p->intruder_class = TCAS_DISPLAY_CLASS_TRAFFIC;
    p->intruder_valid = 1U;
}

// a frame for the audio matrix: TA and RA mode, an intruder of class "resolution advisory", the given advisory (with a matching rate) and the given audio code 
static void make_audio_case(tcas_display_frame_t * p, uint8_t adv,
                            uint8_t audio)
{
    make_climb(p);
    p->mode     = TCAS_DISPLAY_MODE_TA_RA;
    p->advisory = adv;
    p->audio    = audio;

    if (adv == TCAS_DISPLAY_ADV_CLIMB)
    {
        p->target_rate_fpm = 1500;
    }
    else if (adv == TCAS_DISPLAY_ADV_DESCEND)
    {
        p->target_rate_fpm = -1500;
    }
    else
    {
        p->target_rate_fpm = 0;
    }
}

static void expect_ok(const char * name, uint8_t expected)
{
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected, tcas_display_frame_ok(&f), name);
}

// all seven writable registers are still at the sentinel (T4) 
static void check_hw_untouched(void)
{
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.slant);
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.rel_alt);
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.bearing);
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.rate);
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.flags);
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.audio);
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.control);
}

// the seven registers after a good show 
static void check_regs(uint32_t slant, uint32_t rel_alt, uint32_t bearing,
                       uint32_t rate, uint32_t flags, uint32_t audio,
                       uint32_t control)
{
    TEST_ASSERT_EQUAL_UINT32(slant,   hw.slant);
    TEST_ASSERT_EQUAL_HEX32(rel_alt,  hw.rel_alt);
    TEST_ASSERT_EQUAL_UINT32(bearing, hw.bearing);
    TEST_ASSERT_EQUAL_HEX32(rate,     hw.rate);
    TEST_ASSERT_EQUAL_HEX32(flags,    hw.flags);
    TEST_ASSERT_EQUAL_UINT32(audio,   hw.audio);
    TEST_ASSERT_EQUAL_HEX32(control,  hw.control);
}

static void check_counters(uint32_t shown, uint32_t rejected, uint32_t busy,
                           uint32_t fail)
{
    TEST_ASSERT_EQUAL_UINT32(shown,    state.shown_total);
    TEST_ASSERT_EQUAL_UINT32(rejected, state.rejected_total);
    TEST_ASSERT_EQUAL_UINT32(busy,     state.busy_total);
    TEST_ASSERT_EQUAL_UINT32(fail,     state.fail_total);
}

void setUp(void)
{
    (void)memset(&state, 0, sizeof(state));
    reset_hw();
    make_climb(&f);
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_init(&state));
}

void tearDown(void)
{
}

// group 1: init

void test_init_null(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_NULL, tcas_display_init(NULL));
}

void test_init_valid(void)
{
    (void)memset(&state, 0xFF, sizeof(state));

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_init(&state));
    TEST_ASSERT_EQUAL_UINT32(1U, state.initialized);
    check_counters(0U, 0U, 0U, 0U);
}

void test_init_wipes_a_dirty_state(void)
{
    state.shown_total    = 99U;
    state.rejected_total = 98U;
    state.busy_total     = 97U;
    state.fail_total     = 96U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_init(&state));
    TEST_ASSERT_EQUAL_UINT32(1U, state.initialized);
    check_counters(0U, 0U, 0U, 0U);
}

// group 2: frame_ok

void test_frame_ok_null(void)
{
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_display_frame_ok(NULL));
}

void test_frame_ok_good_frames(void)
{
    make_climb(&f);      expect_ok("climb", 1U);
    make_descend(&f);    expect_ok("descend", 1U);
    make_none(&f);       expect_ok("none", 1U);
    make_ta(&f);         expect_ok("traffic advisory", 1U);

    /* standby: nothing shown and nothing said */
    make_none(&f);
    f.mode = TCAS_DISPLAY_MODE_STANDBY;
    expect_ok("standby", 1U);
}

void test_frame_ok_reserved_fields(void)
{
    make_climb(&f);  f.reserved8 = 1U;       expect_ok("reserved8 = 1", 0U);
    make_climb(&f);  f.reserved8 = 255U;     expect_ok("reserved8 = 255", 0U);
    make_climb(&f);  f.reserved16 = 1U;      expect_ok("reserved16 = 1", 0U);
    make_climb(&f);  f.reserved16 = 0x8000U; expect_ok("reserved16 = 0x8000", 0U);
}

void test_frame_ok_codes_out_of_range(void)
{
    make_climb(&f);  f.mode = 3U;            expect_ok("mode 3", 0U);
    make_climb(&f);  f.mode = 255U;          expect_ok("mode 255", 0U);
    make_climb(&f);  f.advisory = 3U;        expect_ok("advisory 3", 0U);
    make_climb(&f);  f.advisory = 255U;      expect_ok("advisory 255", 0U);
    make_climb(&f);  f.audio = 7U;           expect_ok("audio 7", 0U);
    make_climb(&f);  f.audio = 255U;         expect_ok("audio 255", 0U);
    make_climb(&f);  f.intruder_class = 4U;  expect_ok("class 4", 0U);
    make_climb(&f);  f.intruder_class = 255U; expect_ok("class 255", 0U);
    make_climb(&f);  f.intruder_valid = 2U;  expect_ok("intruder_valid 2", 0U);
    make_climb(&f);  f.intruder_valid = 255U; expect_ok("intruder_valid 255", 0U);
}

void test_frame_ok_slant_limit(void)
{
    make_climb(&f);  f.slant_ft = 0U;           expect_ok("slant 0", 1U);
    make_climb(&f);  f.slant_ft = 150000U;      expect_ok("slant 150000", 1U);
    make_climb(&f);  f.slant_ft = 150001U;      expect_ok("slant 150001", 0U);
    make_climb(&f);  f.slant_ft = UINT32_MAX;   expect_ok("slant UINT32_MAX", 0U);
}

void test_frame_ok_bearing_limit(void)
{
    make_climb(&f);  f.bearing_cdeg = 0U;       expect_ok("bearing 0", 1U);
    make_climb(&f);  f.bearing_cdeg = 35999U;   expect_ok("bearing 35999", 1U);
    make_climb(&f);  f.bearing_cdeg = 36000U;   expect_ok("bearing 36000", 0U);
    make_climb(&f);  f.bearing_cdeg = 65535U;   expect_ok("bearing 65535", 0U);
}

void test_frame_ok_rel_alt_limit(void)
{
    make_climb(&f);  f.rel_alt_ft = 127900;     expect_ok("rel alt +127900", 1U);
    make_climb(&f);  f.rel_alt_ft = -127900;    expect_ok("rel alt -127900", 1U);
    make_climb(&f);  f.rel_alt_ft = 127901;     expect_ok("rel alt +127901", 0U);
    make_climb(&f);  f.rel_alt_ft = -127901;    expect_ok("rel alt -127901", 0U);
    make_climb(&f);  f.rel_alt_ft = INT32_MAX;  expect_ok("rel alt INT32_MAX", 0U);
    make_climb(&f);  f.rel_alt_ft = INT32_MIN;  expect_ok("rel alt INT32_MIN", 0U);
}

void test_frame_ok_no_intruder_means_zeros(void)
{
    make_none(&f);                              expect_ok("none is ok", 1U);
    make_none(&f);  f.slant_ft = 1U;            expect_ok("none + slant", 0U);
    make_none(&f);  f.rel_alt_ft = 1;           expect_ok("none + rel alt +1", 0U);
    make_none(&f);  f.rel_alt_ft = -1;          expect_ok("none + rel alt -1", 0U);
    make_none(&f);  f.bearing_cdeg = 1U;        expect_ok("none + bearing", 0U);
    make_none(&f);  f.intruder_class = 1U;      expect_ok("none + class 1", 0U);
    make_none(&f);  f.intruder_class = 3U;      expect_ok("none + class 3", 0U);

    /* the same fields are fine once an intruder is selected */
    make_none(&f);
    f.slant_ft = 1U;
    f.intruder_valid = 1U;
    expect_ok("slant + intruder", 1U);
}

void test_frame_ok_rate_limits(void)
{
    make_climb(&f);    f.target_rate_fpm = 10000;      expect_ok("climb +10000", 1U);
    make_climb(&f);    f.target_rate_fpm = 10001;      expect_ok("climb +10001", 0U);
    make_climb(&f);    f.target_rate_fpm = INT16_MAX;  expect_ok("climb INT16_MAX", 0U);
    make_descend(&f);  f.target_rate_fpm = -10000;     expect_ok("descend -10000", 1U);
    make_descend(&f);  f.target_rate_fpm = -10001;     expect_ok("descend -10001", 0U);
    make_descend(&f);  f.target_rate_fpm = INT16_MIN;  expect_ok("descend INT16_MIN", 0U);
}

void test_frame_ok_rate_agrees_with_advisory(void)
{
    /* no advisory: no rate */
    make_none(&f);                              expect_ok("none, rate 0", 1U);
    make_none(&f);  f.target_rate_fpm = 1;      expect_ok("none, rate +1", 0U);
    make_none(&f);  f.target_rate_fpm = -1;     expect_ok("none, rate -1", 0U);

    /* climb: positive */
    make_climb(&f);  f.target_rate_fpm = 1;     expect_ok("climb, rate +1", 1U);
    make_climb(&f);  f.target_rate_fpm = 0;     expect_ok("climb, rate 0", 0U);
    make_climb(&f);  f.target_rate_fpm = -1500; expect_ok("climb, rate -1500", 0U);

    /* descend: negative */
    make_descend(&f);  f.target_rate_fpm = -1;  expect_ok("descend, rate -1", 1U);
    make_descend(&f);  f.target_rate_fpm = 0;   expect_ok("descend, rate 0", 0U);
    make_descend(&f);  f.target_rate_fpm = 1500; expect_ok("descend, rate +1500", 0U);
}

void test_frame_ok_advisory_needs_ta_ra_mode(void)
{
    make_climb(&f);    f.mode = TCAS_DISPLAY_MODE_STANDBY;  expect_ok("climb, standby", 0U);
    make_climb(&f);    f.mode = TCAS_DISPLAY_MODE_TA_ONLY;  expect_ok("climb, TA only", 0U);
    make_climb(&f);    f.mode = TCAS_DISPLAY_MODE_TA_RA;    expect_ok("climb, TA RA", 1U);
    make_descend(&f);  f.mode = TCAS_DISPLAY_MODE_STANDBY;  expect_ok("descend, standby", 0U);
    make_descend(&f);  f.mode = TCAS_DISPLAY_MODE_TA_ONLY;  expect_ok("descend, TA only", 0U);
    make_descend(&f);  f.mode = TCAS_DISPLAY_MODE_TA_RA;    expect_ok("descend, TA RA", 1U);

    /* no advisory: any mode is fine */
    make_none(&f);  f.mode = TCAS_DISPLAY_MODE_STANDBY;     expect_ok("none, standby", 1U);
    make_none(&f);  f.mode = TCAS_DISPLAY_MODE_TA_ONLY;     expect_ok("none, TA only", 1U);
    make_none(&f);  f.mode = TCAS_DISPLAY_MODE_TA_RA;       expect_ok("none, TA RA", 1U);
}

void test_frame_ok_audio_matrix(void)
{
    /* rows: audio 0 .. 6. columns: advisory none, climb, descend. The
     * intruder is a resolution advisory, so "Traffic" is refused in every
     * column. Worked out by hand from the rules of the header */
    static const uint8_t expected[7][3] =
    {
        { 1U, 1U, 1U },      /* 0 none              */
        { 0U, 0U, 0U },      /* 1 "Traffic"         */
        { 0U, 1U, 0U },      /* 2 "Climb"           */
        { 0U, 0U, 1U },      /* 3 "Descend"         */
        { 0U, 1U, 0U },      /* 4 "Climb now"       */
        { 0U, 0U, 1U },      /* 5 "Descend now"     */
        { 1U, 0U, 0U }       /* 6 "Clear of conflict" */
    };
    char     msg[48];
    uint32_t audio = 0U;
    uint32_t adv   = 0U;

    for (audio = 0U; audio < 7U; audio++)
    {
        for (adv = 0U; adv < 3U; adv++)
        {
            make_audio_case(&f, (uint8_t)adv, (uint8_t)audio);
            (void)snprintf(msg, sizeof(msg), "audio %u, advisory %u",
                           (unsigned int)audio, (unsigned int)adv);
            expect_ok(msg, expected[audio][adv]);
        }
    }
}

void test_frame_ok_traffic_audio(void)
{
    /* "Traffic, traffic" needs a selected intruder of class "traffic" */
    make_ta(&f);                                expect_ok("traffic, class traffic", 1U);

    make_ta(&f);  f.intruder_class = TCAS_DISPLAY_CLASS_PROXIMATE;
    expect_ok("traffic, class proximate", 0U);

    make_ta(&f);  f.intruder_class = TCAS_DISPLAY_CLASS_OTHER;
    expect_ok("traffic, class other", 0U);

    make_ta(&f);  f.intruder_class = TCAS_DISPLAY_CLASS_RESOLUTION;
    expect_ok("traffic, class resolution", 0U);

    make_ta(&f);  f.intruder_valid = 0U;
    expect_ok("traffic, no intruder", 0U);

    /* a traffic phrase while an advisory is held is allowed: the intruder
     * is a traffic advisory and the advisory fields are consistent */
    make_climb(&f);
    f.audio          = TCAS_DISPLAY_AUDIO_TRAFFIC;
    f.intruder_class = TCAS_DISPLAY_CLASS_TRAFFIC;
    expect_ok("traffic phrase while climbing", 1U);
}

void test_frame_ok_has_no_side_effects(void)
{
    tcas_display_frame_t before;

    make_climb(&f);
    (void)memcpy(&before, &f, sizeof(before));
    (void)tcas_display_frame_ok(&f);
    TEST_ASSERT_EQUAL_INT(0, memcmp(&before, &f, sizeof(f)));

    f.mode = 3U;                                  /* a bad frame too */
    (void)memcpy(&before, &f, sizeof(before));
    (void)tcas_display_frame_ok(&f);
    TEST_ASSERT_EQUAL_INT(0, memcmp(&before, &f, sizeof(f)));
}

// group 3: show

void test_show_null_arguments(void)
{
    make_ta(&f);

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_NULL,
                            tcas_display_show(NULL, &hw, &f));
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_NULL,
                            tcas_display_show(&state, NULL, &f));
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_NULL,
                            tcas_display_show(&state, &hw, NULL));
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_NULL,
                            tcas_display_show(NULL, NULL, NULL));

    check_hw_untouched();
    check_counters(0U, 0U, 0U, 0U);
}

void test_show_uninitialized_is_a_config_error(void)
{
    make_ta(&f);
    (void)memset(&state, 0, sizeof(state));          /* zero-filled: NOT valid */

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_CONFIG,
                            tcas_display_show(&state, &hw, &f));
    check_hw_untouched();
    check_counters(0U, 0U, 0U, 0U);
}

void test_show_initialized_flag_must_be_exactly_one(void)
{
    make_ta(&f);
    state.initialized = 2U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_CONFIG,
                            tcas_display_show(&state, &hw, &f));
    check_hw_untouched();
}

void test_show_traffic_advisory_registers(void)
{
    make_ta(&f);

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK,
                            tcas_display_show(&state, &hw, &f));
    check_regs(3000U, 750U, 9000U, 0x00000000U, 0x01020001U, 1U, 1U);
    check_counters(1U, 0U, 0U, 0U);
}

void test_show_climb_registers(void)
{
    make_climb(&f);

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK,
                            tcas_display_show(&state, &hw, &f));
    check_regs(3000U, 0x00000000U, 9000U, 0x000005DCU, 0x01030102U, 2U, 1U);
}

void test_show_descend_registers(void)
{
    make_descend(&f);

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK,
                            tcas_display_show(&state, &hw, &f));
    check_regs(3000U, 0x00000000U, 9000U, 0x0000FA24U, 0x01030202U, 3U, 1U);
}

void test_show_signed_values_have_no_sign_extension(void)
{
    /* rate -1: only the low 16 bits are set, the upper half stays 0 (T6) */
    make_descend(&f);
    f.target_rate_fpm = -1;
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    TEST_ASSERT_EQUAL_HEX32(0x0000FFFFU, hw.rate);

    /* rate -10000 */
    reset_hw();
    f.target_rate_fpm = -10000;
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    TEST_ASSERT_EQUAL_HEX32(0x0000D8F0U, hw.rate);

    /* relative altitude -1: the 32-bit pattern */
    reset_hw();
    make_climb(&f);
    f.rel_alt_ft = -1;
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, hw.rel_alt);

    /* relative altitude +1 */
    reset_hw();
    f.rel_alt_ft = 1;
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    TEST_ASSERT_EQUAL_HEX32(0x00000001U, hw.rel_alt);
}

void test_show_limits(void)
{
    make_climb(&f);
    f.slant_ft        = 150000U;
    f.bearing_cdeg    = 35999U;
    f.rel_alt_ft      = 127900;
    f.target_rate_fpm = 10000;

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    check_regs(150000U, 0x0001F39CU, 35999U, 0x00002710U, 0x01030102U, 2U, 1U);

    /* the lowest relative altitude */
    reset_hw();
    f.rel_alt_ft = -127900;
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    TEST_ASSERT_EQUAL_HEX32(0xFFFE0C64U, hw.rel_alt);

    /* the smallest values */
    reset_hw();
    make_none(&f);
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    check_regs(0U, 0U, 0U, 0U, 0x00000001U, 0U, 1U);
}

void test_show_flags_are_packed_byte_by_byte(void)
{
    /* climb: mode 2, advisory 1, class 3, intruder valid 1 */
    make_climb(&f);

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    TEST_ASSERT_EQUAL_UINT32(2U, hw.flags & 0xFFU);
    TEST_ASSERT_EQUAL_UINT32(1U, (hw.flags >> 8U) & 0xFFU);
    TEST_ASSERT_EQUAL_UINT32(3U, (hw.flags >> 16U) & 0xFFU);
    TEST_ASSERT_EQUAL_UINT32(1U, (hw.flags >> 24U) & 0xFFU);
}

void test_show_bad_frame_touches_no_register(void)
{
    make_climb(&f);
    f.mode = 3U;                                      /* a code out of range */

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_FRAME,
                            tcas_display_show(&state, &hw, &f));
    check_hw_untouched();
    check_counters(0U, 1U, 0U, 0U);

    /* a frame that contradicts itself: "Climb" audio with a Descend */
    make_descend(&f);
    f.audio = TCAS_DISPLAY_AUDIO_CLIMB;
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_FRAME,
                            tcas_display_show(&state, &hw, &f));
    check_hw_untouched();
    check_counters(0U, 2U, 0U, 0U);
}

void test_show_bad_frame_is_checked_before_busy(void)
{
    make_climb(&f);
    f.mode   = 3U;
    hw.status = 0U;                                   /* the unit is busy too */

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_FRAME,
                            tcas_display_show(&state, &hw, &f));
    check_hw_untouched();
    check_counters(0U, 1U, 0U, 0U);                   /* not counted as busy */
}

void test_show_busy_touches_no_register(void)
{
    make_ta(&f);
    hw.status = 0U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_BUSY,
                            tcas_display_show(&state, &hw, &f));
    check_hw_untouched();
    check_counters(0U, 0U, 1U, 0U);
}

void test_show_ready_bit_only(void)
{
    make_ta(&f);

    /* every bit except READY set: still busy */
    hw.status = UINT32_C(0xFFFFFFFE);
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_BUSY,
                            tcas_display_show(&state, &hw, &f));
    check_hw_untouched();

    /* READY and every other bit: ready */
    reset_hw();
    hw.status = UINT32_C(0xFFFFFFFF);
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK,
                            tcas_display_show(&state, &hw, &f));
    TEST_ASSERT_EQUAL_UINT32(1U, hw.control);
}

void test_show_counts_and_repeats(void)
{
    tcas_display_frame_t before;

    make_ta(&f);
    (void)memcpy(&before, &f, sizeof(before));

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    reset_hw();
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    reset_hw();
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));

    check_counters(3U, 0U, 0U, 0U);
    TEST_ASSERT_EQUAL_INT(0, memcmp(&before, &f, sizeof(f)));   /* frame untouched */
}

// group 4: fail

void test_fail_null_arguments(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_NULL,
                            tcas_display_fail(NULL, &hw));
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_NULL,
                            tcas_display_fail(&state, NULL));
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_NULL,
                            tcas_display_fail(NULL, NULL));

    check_hw_untouched();
    check_counters(0U, 0U, 0U, 0U);
}

void test_fail_uninitialized(void)
{
    (void)memset(&state, 0, sizeof(state));

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_CONFIG,
                            tcas_display_fail(&state, &hw));
    check_hw_untouched();

    state.initialized = 2U;                           /* must be exactly 1 */
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_CONFIG,
                            tcas_display_fail(&state, &hw));
    check_hw_untouched();
}

void test_fail_busy(void)
{
    hw.status = 0U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_BUSY,
                            tcas_display_fail(&state, &hw));
    check_hw_untouched();
    check_counters(0U, 0U, 1U, 0U);
}

void test_fail_good(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_fail(&state, &hw));

    /* silence and the annunciation... */
    TEST_ASSERT_EQUAL_UINT32(0U, hw.audio);
    TEST_ASSERT_EQUAL_HEX32(0x00000003U, hw.control);

    /* ...and the data registers are not touched */
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.slant);
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.rel_alt);
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.bearing);
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.rate);
    TEST_ASSERT_EQUAL_HEX32(SENT, hw.flags);

    check_counters(0U, 0U, 0U, 1U);
}

void test_fail_check_precedence(void)
{
    /* NULL beats everything */
    hw.status = 0U;
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_NULL, tcas_display_fail(NULL, &hw));

    /* uninitialized beats busy */
    (void)memset(&state, 0, sizeof(state));
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_CONFIG,
                            tcas_display_fail(&state, &hw));

    /* initialized: now busy is reported */
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_init(&state));
    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_BUSY,
                            tcas_display_fail(&state, &hw));
    check_hw_untouched();
}

// group 5: counters

void test_counter_saturation_shown(void)
{
    make_ta(&f);
    state.shown_total = UINT32_MAX;

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_show(&state, &hw, &f));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.shown_total);
}

void test_counter_saturation_rejected(void)
{
    make_climb(&f);
    f.mode = 3U;
    state.rejected_total = UINT32_MAX;

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_FRAME,
                            tcas_display_show(&state, &hw, &f));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.rejected_total);
}

void test_counter_saturation_busy(void)
{
    make_ta(&f);
    hw.status = 0U;
    state.busy_total = UINT32_MAX;

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_BUSY,
                            tcas_display_show(&state, &hw, &f));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.busy_total);

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_ERR_BUSY,
                            tcas_display_fail(&state, &hw));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.busy_total);
}

void test_counter_saturation_fail(void)
{
    state.fail_total = UINT32_MAX;

    TEST_ASSERT_EQUAL_UINT8(TCAS_DISPLAY_OK, tcas_display_fail(&state, &hw));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.fail_total);
}

// group 6: sweep of 1 512 frames against the oracle (rule T2)

/* the expected result, written as a LIST OF VIOLATIONS of the rules of the
 * header, one line per rule (it does not call the driver):
 *   m mode, a advisory, x audio, c class, v intruder valid, r rate option
 *   (0 means 0, 1 means +1 500, 2 means -1 500) 
*/
static uint8_t expected_ok(uint32_t m, uint32_t a, uint32_t x, uint32_t c,
                           uint32_t v, uint32_t r)
{
    uint32_t violations = 0U;

    if ((v == 0U) && (c != 0U))                       /* no intruder: class 0   */
    {
        violations++;
    }
    if ((a == 0U) && (r != 0U))                       /* no advisory: no rate   */
    {
        violations++;
    }
    if ((a == 1U) && (r != 1U))                       /* climb: positive rate   */
    {
        violations++;
    }
    if ((a == 2U) && (r != 2U))                       /* descend: negative rate */
    {
        violations++;
    }
    if ((a != 0U) && (m != 2U))                       /* advisory: TA and RA    */
    {
        violations++;
    }
    if ((x == 1U) && !((v == 1U) && (c == 2U)))       /* "Traffic": TA shown    */
    {
        violations++;
    }
    if (((x == 2U) || (x == 4U)) && (a != 1U))        /* "Climb": a climb       */
    {
        violations++;
    }
    if (((x == 3U) || (x == 5U)) && (a != 2U))        /* "Descend": a descend   */
    {
        violations++;
    }
    if ((x == 6U) && (a != 0U))                       /* "Clear": no advisory   */
    {
        violations++;
    }

    return (violations == 0U) ? 1U : 0U;
}

void test_sweep_all_frames(void)
{
    static const int16_t rates[3] = { 0, 1500, -1500 };

    uint32_t combos = 0U;
    uint32_t nbad   = 0U;
    uint32_t m = 0U;
    uint32_t a = 0U;
    uint32_t x = 0U;
    uint32_t c = 0U;
    uint32_t v = 0U;
    uint32_t r = 0U;

    for (m = 0U; m < 3U; m++)
    {
        for (a = 0U; a < 3U; a++)
        {
            for (x = 0U; x < 7U; x++)
            {
                for (c = 0U; c < 4U; c++)
                {
                    for (v = 0U; v < 2U; v++)
                    {
                        for (r = 0U; r < 3U; r++)
                        {
                            tcas_display_frame_t g;
                            const uint8_t  exp_ok = expected_ok(m, a, x, c, v, r);
                            const uint32_t exp_slant   = (v == 1U) ? 3000U : 0U;
                            const uint32_t exp_rel     = (v == 1U) ? 750U : 0U;
                            const uint32_t exp_bearing = (v == 1U) ? 9000U : 0U;
                            const uint32_t exp_rate    =
                                (uint32_t)(uint16_t)rates[r];
                            const uint32_t exp_flags   =
                                m | (a << 8U) | (c << 16U) | (v << 24U);
                            uint32_t ok = 1U;
                            tcas_display_status_t st = TCAS_DISPLAY_OK;

                            (void)memset(&g, 0, sizeof(g));
                            g.slant_ft        = exp_slant;
                            g.rel_alt_ft      = (int32_t)exp_rel;
                            g.bearing_cdeg    = (uint16_t)exp_bearing;
                            g.target_rate_fpm = rates[r];
                            g.mode            = (uint8_t)m;
                            g.advisory        = (uint8_t)a;
                            g.audio           = (uint8_t)x;
                            g.intruder_class  = (uint8_t)c;
                            g.intruder_valid  = (uint8_t)v;

                            (void)tcas_display_init(&state);
                            reset_hw();

                            if (tcas_display_frame_ok(&g) != exp_ok)
                            {
                                ok = 0U;
                            }

                            st = tcas_display_show(&state, &hw, &g);

                            if (exp_ok == 1U)
                            {
                                if ((st != TCAS_DISPLAY_OK) ||
                                    (hw.slant != exp_slant) ||
                                    (hw.rel_alt != exp_rel) ||
                                    (hw.bearing != exp_bearing) ||
                                    (hw.rate != exp_rate) ||
                                    (hw.flags != exp_flags) ||
                                    (hw.audio != x) ||
                                    (hw.control != TCAS_DISPLAY_CTRL_COMMIT) ||
                                    (state.shown_total != 1U))
                                {
                                    ok = 0U;
                                }
                            }
                            else if ((st != TCAS_DISPLAY_ERR_FRAME) ||
                                     (hw.slant != SENT) ||
                                     (hw.rel_alt != SENT) ||
                                     (hw.bearing != SENT) ||
                                     (hw.rate != SENT) ||
                                     (hw.flags != SENT) ||
                                     (hw.audio != SENT) ||
                                     (hw.control != SENT) ||
                                     (state.rejected_total != 1U))
                            {
                                ok = 0U;
                            }
                            else
                            {
                                /* refused and nothing written: fine */
                            }

                            combos++;

                            if (ok != 1U)
                            {
                                if (nbad == 0U)
                                {
                                    (void)printf("sweep first mismatch: mode %u "
                                                 "advisory %u audio %u class %u "
                                                 "valid %u rate option %u\n",
                                                 (unsigned int)m,
                                                 (unsigned int)a,
                                                 (unsigned int)x,
                                                 (unsigned int)c,
                                                 (unsigned int)v,
                                                 (unsigned int)r);
                                }
                                nbad++;
                            }
                        }
                    }
                }
            }
        }
    }

    TEST_ASSERT_EQUAL_UINT32(1512U, combos);
    TEST_ASSERT_EQUAL_UINT32(0U, nbad);
}

// test runner

int main(void)
{
    UNITY_BEGIN();

    /* init */
    RUN_TEST(test_init_null);
    RUN_TEST(test_init_valid);
    RUN_TEST(test_init_wipes_a_dirty_state);

    /* frame_ok */
    RUN_TEST(test_frame_ok_null);
    RUN_TEST(test_frame_ok_good_frames);
    RUN_TEST(test_frame_ok_reserved_fields);
    RUN_TEST(test_frame_ok_codes_out_of_range);
    RUN_TEST(test_frame_ok_slant_limit);
    RUN_TEST(test_frame_ok_bearing_limit);
    RUN_TEST(test_frame_ok_rel_alt_limit);
    RUN_TEST(test_frame_ok_no_intruder_means_zeros);
    RUN_TEST(test_frame_ok_rate_limits);
    RUN_TEST(test_frame_ok_rate_agrees_with_advisory);
    RUN_TEST(test_frame_ok_advisory_needs_ta_ra_mode);
    RUN_TEST(test_frame_ok_audio_matrix);
    RUN_TEST(test_frame_ok_traffic_audio);
    RUN_TEST(test_frame_ok_has_no_side_effects);

    /* show */
    RUN_TEST(test_show_null_arguments);
    RUN_TEST(test_show_uninitialized_is_a_config_error);
    RUN_TEST(test_show_initialized_flag_must_be_exactly_one);
    RUN_TEST(test_show_traffic_advisory_registers);
    RUN_TEST(test_show_climb_registers);
    RUN_TEST(test_show_descend_registers);
    RUN_TEST(test_show_signed_values_have_no_sign_extension);
    RUN_TEST(test_show_limits);
    RUN_TEST(test_show_flags_are_packed_byte_by_byte);
    RUN_TEST(test_show_bad_frame_touches_no_register);
    RUN_TEST(test_show_bad_frame_is_checked_before_busy);
    RUN_TEST(test_show_busy_touches_no_register);
    RUN_TEST(test_show_ready_bit_only);
    RUN_TEST(test_show_counts_and_repeats);

    /* fail */
    RUN_TEST(test_fail_null_arguments);
    RUN_TEST(test_fail_uninitialized);
    RUN_TEST(test_fail_busy);
    RUN_TEST(test_fail_good);
    RUN_TEST(test_fail_check_precedence);

    /* counters */
    RUN_TEST(test_counter_saturation_shown);
    RUN_TEST(test_counter_saturation_rejected);
    RUN_TEST(test_counter_saturation_busy);
    RUN_TEST(test_counter_saturation_fail);

    /* sweep */
    RUN_TEST(test_sweep_all_frames);

    return UNITY_END();
}