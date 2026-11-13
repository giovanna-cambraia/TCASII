/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : tests/unit/c/test_a429_drv.c
 * Module      : Unit tests of the ARINC 429 receive driver (Unity)
 * Language    : C11, Unity test framework
 * Assurance   : DO-178C Software Level B (target), verification artifact
 * Tests       : src/c/drivers/arinc429/tcas_a429_drv.c and .h
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Checks every behavior promised in tcas_a429_drv.h: the configuration
 *   checks, the parity and label helpers, the order of the word checks,
 *   the fail-closed output (rule R7) and the rate-limited drain (rule R8).
 *   Each test checks one thing and has a name that says what it is.
 *
 * DESIGN RULES OF THE TESTS
 *   T1  The test words are built by make_word, which has its OWN code for
 *       label reversal and odd parity. It never calls the driver. If the
 *       driver and the test agree, it is because both are right.
 *   T2  Expected values are written as literals taken from the header, not
 *       computed with the driver's functions.
 *   T3  Every test builds its own state and configuration (setUp does it),
 *       so tests do not depend on each other or on their order.
 *   T4  Every error path checks the whole output, not only the status:
 *       valid = 0, data = 0, and the label, sdi and ssm rules of R7.
 *   T5  The fake FIFO is plain memory (see LIMITS below), so the tests
 *       need no hardware, no operating system and no timing.
 *
 * TESTS BY GROUP
 *   init      : 10   configuration errors and the safe empty state
 *   helpers   :  5   parity and label reversal
 *   decode    : 14   valid path, each error code, order of checks,
 *                    counter saturation
 *   drain     : 10   arguments, capacity, empty, backlog, overrun
 *
 * LIMITS (to be listed as an open item for v2)
 *   The fake FIFO does not pop: reading its data register leaves the status
 *   register unchanged. The tests therefore cannot show that a drain stops
 *   early when the FIFO empties in the middle of the loop. They show that
 *   the drain takes exactly max_words words when the FIFO stays non-empty,
 *   that it reports an empty FIFO, an overrun and a backlog, and that it
 *   never writes past max_words entries. An early stop needs a read hook
 *   in the driver or a link-time mock of the register access.
 *
 * USAGE (from the project root; UNITY_DIR is the folder that holds
 * unity.c and unity.h, for example the one used by the MIL1553 project)
 *   UNITY_DIR=../MIL1553/tests/unity   (adjust to your path)
 *   gcc -std=c11 -Wall -Wextra -Wpedantic \
 *       -I src/c/drivers/arinc429 -I $UNITY_DIR \
 *       tests/unit/c/test_a429_drv.c \
 *       src/c/drivers/arinc429/tcas_a429_drv.c \
 *       $UNITY_DIR/unity.c -o build/bin/test_a429_drv.exe
 *   build/bin/test_a429_drv.exe
 *
 * MISRA NOTES
 *   * The rules of MISRA C apply to the driver, not to its tests. The
 *     tests use <string.h> and plain loops for clarity.
 * ===========================================================================
 */

#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "tcas_a429_drv.h"



// test fixtures
#define TEST_MAX_WORDS   (8U)           // max_words used by most tests      
#define TEST_FRAME       UINT32_C(77)   // frame number written to outputs   
#define SENTINEL_BYTE    (0xA5U)        // pattern: "this was not written"   

static tcas_a429_state_t state;
static tcas_a429_cfg_t   cfg;
static tcas_a429_word_t  out[TCAS_A429_MAX_WORDS_PER_CYCLE];
static tcas_a429_hw_fifo_t fifo;



// helpers (independent of the driver, rule T1)

// reverses the order of the 8 bits of v 
static uint8_t rev8(uint8_t v)
{
    uint8_t  r = 0U;
    uint32_t i = 0U;

    for (i = 0U; i < 8U; i++)
    {
        r = (uint8_t)((uint32_t)(r << 1U) | (uint32_t)((v >> i) & 1U));
    }

    return r;
}

// number of bits set in v 
static uint32_t popcount32(uint32_t v)
{
    uint32_t n = 0U;
    uint32_t i = 0U;

    for (i = 0U; i < 32U; i++)
    {
        n += (v >> i) & 1U;
    }

    return n;
}

// builds a raw ARINC 429 word with odd parity; label is the label value (the label is stored bit-reversed in the word) 
static uint32_t make_word(uint8_t label, uint8_t sdi, uint32_t data, uint8_t ssm)
{
    uint32_t w = (uint32_t)rev8(label)
               | ((uint32_t)sdi << 8U)
               | ((data & UINT32_C(0x7FFFF)) << 10U)
               | ((uint32_t)ssm << 29U);

    // bit 31 is still 0: set it only when the word has an even bit count 
    if ((popcount32(w) % 2U) == 0U)
    {
        w |= UINT32_C(0x80000000);
    }

    return w;
}

// fills a configuration with the three project labels 
static void make_cfg(tcas_a429_cfg_t * c, uint16_t max_words)
{
    (void)memset(c, 0, sizeof(*c));
    c->labels[0]  = TCAS_A429_LABEL_PRESSURE_ALT;
    c->labels[1]  = TCAS_A429_LABEL_RADIO_ALT;
    c->labels[2]  = TCAS_A429_LABEL_ALT_RATE;
    c->num_labels = 3U;
    c->max_words  = max_words;
}

// fills the output array with the "not written" pattern 
static void sentinel_out(void)
{
    (void)memset(out, (int)SENTINEL_BYTE, sizeof(out));
}

// checks the fail-closed output of a rejected word (rule R7) 
static void check_rejected(const tcas_a429_word_t * w, tcas_a429_status_t status)
{
    TEST_ASSERT_EQUAL_UINT8(status, w->status);
    TEST_ASSERT_EQUAL_UINT8(0U, w->valid);
    TEST_ASSERT_EQUAL_UINT32(0U, w->data);
    TEST_ASSERT_EQUAL_UINT32(TEST_FRAME, w->frame);
    TEST_ASSERT_EQUAL_UINT8(0U, w->reserved8);
    TEST_ASSERT_EQUAL_UINT16(0U, w->reserved16);
}

void setUp(void)
{
    make_cfg(&cfg, (uint16_t)TEST_MAX_WORDS);
    (void)memset(&state, 0, sizeof(state));
    (void)memset(&fifo, 0, sizeof(fifo));
    sentinel_out();
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK, tcas_a429_init(&state, &cfg));
}

void tearDown(void)
{
}



// group 1: init
void test_init_null_state(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_NULL, tcas_a429_init(NULL, &cfg));
}

void test_init_null_cfg(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_NULL, tcas_a429_init(&state, NULL));
    TEST_ASSERT_EQUAL_UINT32(0U, state.initialized);
}

void test_init_valid_config(void)
{
    (void)memset(&state, 0xFF, sizeof(state));

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK, tcas_a429_init(&state, &cfg));
    TEST_ASSERT_EQUAL_UINT32(1U, state.initialized);
    TEST_ASSERT_EQUAL_UINT32(0U, state.words_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.parity_fail_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.label_reject_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.ssm_reject_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.overrun_total);
}

void test_init_zero_labels(void)
{
    cfg.num_labels = 0U;
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_CONFIG, tcas_a429_init(&state, &cfg));
}

void test_init_too_many_labels(void)
{
    cfg.num_labels = (uint8_t)(TCAS_A429_MAX_LABELS + 1U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_CONFIG, tcas_a429_init(&state, &cfg));
}

void test_init_full_label_table_is_accepted(void)
{
    uint32_t i = 0U;

    for (i = 0U; i < TCAS_A429_MAX_LABELS; i++)
    {
        cfg.labels[i] = (uint8_t)(0x10U + i);   // 16 different labels 
    }
    cfg.num_labels = (uint8_t)TCAS_A429_MAX_LABELS;

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK, tcas_a429_init(&state, &cfg));
}

void test_init_duplicate_label(void)
{
    cfg.labels[2] = cfg.labels[0];
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_CONFIG, tcas_a429_init(&state, &cfg));
}

void test_init_max_words_zero(void)
{
    cfg.max_words = 0U;
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_CONFIG, tcas_a429_init(&state, &cfg));
}

void test_init_max_words_too_big(void)
{
    cfg.max_words = (uint16_t)(TCAS_A429_MAX_WORDS_PER_CYCLE + 1U);
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_CONFIG, tcas_a429_init(&state, &cfg));
}

void test_init_failure_leaves_safe_empty_state(void)
{
    // a state that looks initialized and has counts, then a bad config 
    state.initialized = 1U;
    state.words_total = 99U;
    cfg.num_labels    = 0U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_CONFIG, tcas_a429_init(&state, &cfg));
    TEST_ASSERT_EQUAL_UINT32(0U, state.initialized);
    TEST_ASSERT_EQUAL_UINT32(0U, state.words_total);
}



// group 2: helpers (parity and label)
void test_parity_known_vectors(void)
{
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_a429_parity_ok(UINT32_C(0x00000000))); //  0 ones: even 
    TEST_ASSERT_EQUAL_UINT8(1U, tcas_a429_parity_ok(UINT32_C(0x00000001))); //  1 one : odd  
    TEST_ASSERT_EQUAL_UINT8(1U, tcas_a429_parity_ok(UINT32_C(0x80000000))); //  1 one : odd  
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_a429_parity_ok(UINT32_C(0x00000003))); //  2 ones: even 
    TEST_ASSERT_EQUAL_UINT8(1U, tcas_a429_parity_ok(UINT32_C(0x00000007))); //  3 ones: odd  
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_a429_parity_ok(UINT32_C(0xFFFFFFFF))); // 32 ones: even 
    TEST_ASSERT_EQUAL_UINT8(1U, tcas_a429_parity_ok(UINT32_C(0x7FFFFFFF))); // 31 ones: odd  
}

void test_parity_every_single_bit_flip_toggles_the_result(void)
{
    const uint32_t good = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 1U,
                                    UINT32_C(0x2AAAA), TCAS_A429_SSM_NORMAL);
    uint32_t bit = 0U;

    TEST_ASSERT_EQUAL_UINT8(1U, tcas_a429_parity_ok(good));

    for (bit = 0U; bit < 32U; bit++)
    {
        TEST_ASSERT_EQUAL_UINT8(0U, tcas_a429_parity_ok(good ^ (UINT32_C(1) << bit)));
    }
}

void test_label_known_vectors(void)
{
    // the label arrives bit-reversed: 0xC1 is the reverse of 0x83 
    TEST_ASSERT_EQUAL_HEX8(0x83U, tcas_a429_label_value(UINT32_C(0x000000C1)));
    // 0x2E is the reverse of 0x74 
    TEST_ASSERT_EQUAL_HEX8(0x74U, tcas_a429_label_value(UINT32_C(0x0000002E)));
    TEST_ASSERT_EQUAL_HEX8(0x00U, tcas_a429_label_value(UINT32_C(0x00000000)));
    TEST_ASSERT_EQUAL_HEX8(0xFFU, tcas_a429_label_value(UINT32_C(0x000000FF)));
    TEST_ASSERT_EQUAL_HEX8(0x80U, tcas_a429_label_value(UINT32_C(0x00000001)));
    TEST_ASSERT_EQUAL_HEX8(0x01U, tcas_a429_label_value(UINT32_C(0x00000080)));
}

void test_label_reversal_twice_is_identity_for_all_256_values(void)
{
    uint32_t v = 0U;

    for (v = 0U; v < 256U; v++)
    {
        const uint8_t once = tcas_a429_label_value(v);
        TEST_ASSERT_EQUAL_UINT8((uint8_t)v, tcas_a429_label_value((uint32_t)once));
    }
}

void test_label_ignores_bits_above_the_label_field(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x00U, tcas_a429_label_value(UINT32_C(0xFFFFFF00)));
    TEST_ASSERT_EQUAL_HEX8(0x83U, tcas_a429_label_value(UINT32_C(0xFFFFFFC1)));
}



// group 3: decode_word
void test_decode_valid_word(void)
{
    const uint32_t w = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 2U,
                                 UINT32_C(0x12345), TCAS_A429_SSM_NORMAL);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK,
        tcas_a429_decode_word(&state, &cfg, w, TEST_FRAME, &out[0]));

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK, out[0].status);
    TEST_ASSERT_EQUAL_UINT8(1U, out[0].valid);
    TEST_ASSERT_EQUAL_HEX32(0x12345U, out[0].data);
    TEST_ASSERT_EQUAL_HEX8(0x83U, out[0].label);
    TEST_ASSERT_EQUAL_UINT8(2U, out[0].sdi);
    TEST_ASSERT_EQUAL_UINT8(3U, out[0].ssm);
    TEST_ASSERT_EQUAL_UINT32(TEST_FRAME, out[0].frame);
    TEST_ASSERT_EQUAL_UINT8(0U, out[0].reserved8);
    TEST_ASSERT_EQUAL_UINT16(0U, out[0].reserved16);
    TEST_ASSERT_EQUAL_UINT32(1U, state.words_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.parity_fail_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.label_reject_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.ssm_reject_total);
}

void test_decode_data_extremes(void)
{
    // smallest and largest 19-bit values, right-justified 
    const uint32_t lo = make_word(TCAS_A429_LABEL_RADIO_ALT, 0U,
                                  UINT32_C(0x00000), TCAS_A429_SSM_NORMAL);
    const uint32_t hi = make_word(TCAS_A429_LABEL_RADIO_ALT, 3U,
                                  UINT32_C(0x7FFFF), TCAS_A429_SSM_NORMAL);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK,
        tcas_a429_decode_word(&state, &cfg, lo, TEST_FRAME, &out[0]));
    TEST_ASSERT_EQUAL_UINT8(1U, out[0].valid);
    TEST_ASSERT_EQUAL_HEX32(0x00000U, out[0].data);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK,
        tcas_a429_decode_word(&state, &cfg, hi, TEST_FRAME, &out[1]));
    TEST_ASSERT_EQUAL_UINT8(1U, out[1].valid);
    TEST_ASSERT_EQUAL_HEX32(0x7FFFFU, out[1].data);
    TEST_ASSERT_EQUAL_UINT8(3U, out[1].sdi);
}

void test_decode_each_accepted_label(void)
{
    const uint8_t labels[3] = { TCAS_A429_LABEL_PRESSURE_ALT,
                                TCAS_A429_LABEL_RADIO_ALT,
                                TCAS_A429_LABEL_ALT_RATE };
    uint32_t i = 0U;

    for (i = 0U; i < 3U; i++)
    {
        const uint32_t w = make_word(labels[i], 0U, UINT32_C(0x100) + i,
                                     TCAS_A429_SSM_NORMAL);

        TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK,
            tcas_a429_decode_word(&state, &cfg, w, TEST_FRAME, &out[i]));
        TEST_ASSERT_EQUAL_UINT8(labels[i], out[i].label);
        TEST_ASSERT_EQUAL_UINT8(1U, out[i].valid);
    }
}

void test_decode_null_out(void)
{
    const uint32_t w = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U, 1U,
                                 TCAS_A429_SSM_NORMAL);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_NULL,
        tcas_a429_decode_word(&state, &cfg, w, TEST_FRAME, NULL));
    TEST_ASSERT_EQUAL_UINT32(0U, state.words_total);   // nothing was counted 
}

void test_decode_null_state_still_writes_a_rejected_output(void)
{
    const uint32_t w = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U, 5U,
                                 TCAS_A429_SSM_NORMAL);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_NULL,
        tcas_a429_decode_word(NULL, &cfg, w, TEST_FRAME, &out[0]));
    check_rejected(&out[0], TCAS_A429_ERR_NULL);
}

void test_decode_null_cfg(void)
{
    const uint32_t w = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U, 5U,
                                 TCAS_A429_SSM_NORMAL);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_NULL,
        tcas_a429_decode_word(&state, NULL, w, TEST_FRAME, &out[0]));
    check_rejected(&out[0], TCAS_A429_ERR_NULL);
}

void test_decode_uninitialized_state_is_a_config_error(void)
{
    const uint32_t w = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U, 5U,
                                 TCAS_A429_SSM_NORMAL);

    (void)memset(&state, 0, sizeof(state));     // zero-filled: NOT valid 

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_CONFIG,
        tcas_a429_decode_word(&state, &cfg, w, TEST_FRAME, &out[0]));
    check_rejected(&out[0], TCAS_A429_ERR_CONFIG);
    TEST_ASSERT_EQUAL_UINT32(0U, state.words_total);
}

void test_decode_parity_failure_trusts_nothing(void)
{
    const uint32_t good = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 3U,
                                    UINT32_C(0x7FFFF), TCAS_A429_SSM_NORMAL);
    const uint32_t bad  = good ^ UINT32_C(0x00000400);   // flip one data bit 

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_PARITY,
        tcas_a429_decode_word(&state, &cfg, bad, TEST_FRAME, &out[0]));
    check_rejected(&out[0], TCAS_A429_ERR_PARITY);
    TEST_ASSERT_EQUAL_UINT8(0U, out[0].label);   // rule R7: nothing trusted 
    TEST_ASSERT_EQUAL_UINT8(0U, out[0].sdi);
    TEST_ASSERT_EQUAL_UINT8(0U, out[0].ssm);
    TEST_ASSERT_EQUAL_UINT32(1U, state.words_total);
    TEST_ASSERT_EQUAL_UINT32(1U, state.parity_fail_total);
}

void test_decode_label_not_accepted(void)
{
    const uint32_t w = make_word(0x01U, 1U, UINT32_C(0x3FFFF),
                                 TCAS_A429_SSM_NORMAL);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_LABEL,
        tcas_a429_decode_word(&state, &cfg, w, TEST_FRAME, &out[0]));
    check_rejected(&out[0], TCAS_A429_ERR_LABEL);
    TEST_ASSERT_EQUAL_HEX8(0x01U, out[0].label);   // delivered for diagnostics 
    TEST_ASSERT_EQUAL_UINT8(1U, out[0].sdi);
    TEST_ASSERT_EQUAL_UINT8(3U, out[0].ssm);
    TEST_ASSERT_EQUAL_UINT32(1U, state.label_reject_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.ssm_reject_total);
}

void test_decode_ssm_failure_warning(void)
{
    const uint32_t w = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U,
                                 UINT32_C(0x7FFFF), TCAS_A429_SSM_FAILURE_WARNING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_SSM_FW,
        tcas_a429_decode_word(&state, &cfg, w, TEST_FRAME, &out[0]));
    check_rejected(&out[0], TCAS_A429_ERR_SSM_FW);
    TEST_ASSERT_EQUAL_UINT8(0U, out[0].ssm);
    TEST_ASSERT_EQUAL_UINT32(1U, state.ssm_reject_total);
}

void test_decode_ssm_no_computed_data(void)
{
    const uint32_t w = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U,
                                 UINT32_C(0x7FFFF), TCAS_A429_SSM_NO_COMPUTED_DATA);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_SSM_NCD,
        tcas_a429_decode_word(&state, &cfg, w, TEST_FRAME, &out[0]));
    check_rejected(&out[0], TCAS_A429_ERR_SSM_NCD);
    TEST_ASSERT_EQUAL_UINT8(1U, out[0].ssm);
}

void test_decode_ssm_functional_test(void)
{
    const uint32_t w = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U,
                                 UINT32_C(0x7FFFF), TCAS_A429_SSM_FUNCTIONAL_TEST);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_SSM_FT,
        tcas_a429_decode_word(&state, &cfg, w, TEST_FRAME, &out[0]));
    check_rejected(&out[0], TCAS_A429_ERR_SSM_FT);
    TEST_ASSERT_EQUAL_UINT8(2U, out[0].ssm);
}

void test_order_parity_is_checked_before_label(void)
{
    // both faults at once: bad label AND bad parity -> parity wins 
    const uint32_t good = make_word(0x01U, 0U, 1U, TCAS_A429_SSM_NORMAL);
    const uint32_t bad  = good ^ UINT32_C(0x80000000);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_PARITY,
        tcas_a429_decode_word(&state, &cfg, bad, TEST_FRAME, &out[0]));
    TEST_ASSERT_EQUAL_UINT32(1U, state.parity_fail_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.label_reject_total);
}

void test_order_label_is_checked_before_ssm(void)
{
    // both faults at once: bad label AND failure-warning SSM -> label wins 
    const uint32_t w = make_word(0x01U, 0U, 1U, TCAS_A429_SSM_FAILURE_WARNING);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_LABEL,
        tcas_a429_decode_word(&state, &cfg, w, TEST_FRAME, &out[0]));
    TEST_ASSERT_EQUAL_UINT32(1U, state.label_reject_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.ssm_reject_total);
}

void test_decode_counters_saturate_and_never_wrap(void)
{
    const uint32_t good = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U, 1U,
                                    TCAS_A429_SSM_NORMAL);
    const uint32_t bad  = good ^ UINT32_C(0x00000400);

    state.words_total       = UINT32_MAX;
    state.parity_fail_total = UINT32_MAX;

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_PARITY,
        tcas_a429_decode_word(&state, &cfg, bad, TEST_FRAME, &out[0]));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.words_total);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, state.parity_fail_total);
}



// group 4: drain (the fake FIFO does not pop, see LIMITS in the header)
void test_drain_null_arguments(void)
{
    uint32_t count = 99U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_NULL,
        tcas_a429_drain(NULL, &cfg, &fifo, TEST_FRAME, out, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(0U, count);

    count = 99U;
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_NULL,
        tcas_a429_drain(&state, NULL, &fifo, TEST_FRAME, out, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(0U, count);

    count = 99U;
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_NULL,
        tcas_a429_drain(&state, &cfg, NULL, TEST_FRAME, out, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(0U, count);

    count = 99U;
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_NULL,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, NULL, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(0U, count);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_NULL,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, out, TEST_MAX_WORDS, NULL));
}

void test_drain_capacity_smaller_than_max_words(void)
{
    uint32_t count = 99U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_CONFIG,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, out,
                        TEST_MAX_WORDS - 1U, &count));
    TEST_ASSERT_EQUAL_UINT32(0U, count);
}

void test_drain_uninitialized_state(void)
{
    uint32_t count = 99U;

    (void)memset(&state, 0, sizeof(state));

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_CONFIG,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, out, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(0U, count);
}

void test_drain_empty_fifo(void)
{
    uint32_t count = 99U;

    fifo.status = 0U;     // nothing waiting, no overrun 

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, out, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(0U, count);
    TEST_ASSERT_EQUAL_UINT32(0U, state.words_total);
}

void test_drain_takes_exactly_max_words_and_reports_backlog(void)
{
    uint32_t count = 0U;

    fifo.status = TCAS_A429_HW_NOT_EMPTY;     // the fake never empties 
    fifo.data   = make_word(TCAS_A429_LABEL_RADIO_ALT, 1U, UINT32_C(0x0ABCD),
                            TCAS_A429_SSM_NORMAL);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_BACKLOG,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, out, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(TEST_MAX_WORDS, count);
    TEST_ASSERT_EQUAL_UINT32(TEST_MAX_WORDS, state.words_total);
    TEST_ASSERT_EQUAL_UINT32(0U, state.overrun_total);

    {
        uint32_t i = 0U;

        for (i = 0U; i < TEST_MAX_WORDS; i++)
        {
            TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK, out[i].status);
            TEST_ASSERT_EQUAL_UINT8(1U, out[i].valid);
            TEST_ASSERT_EQUAL_HEX32(0x0ABCDU, out[i].data);
            TEST_ASSERT_EQUAL_HEX8(0x74U, out[i].label);
            TEST_ASSERT_EQUAL_UINT32(TEST_FRAME, out[i].frame);
        }
    }

    // the entry after the last one must be untouched (rule R8) 
    TEST_ASSERT_EQUAL_HEX32(0xA5A5A5A5U, out[TEST_MAX_WORDS].data);
    TEST_ASSERT_EQUAL_HEX32(0xA5A5A5A5U, out[TEST_MAX_WORDS].frame);
}

void test_drain_overrun_is_reported_and_counted(void)
{
    uint32_t count = 0U;

    fifo.status = TCAS_A429_HW_NOT_EMPTY | TCAS_A429_HW_OVERRUN;
    fifo.data   = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U, 1U,
                            TCAS_A429_SSM_NORMAL);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_OVERRUN,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, out, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(TEST_MAX_WORDS, count);   // words still delivered 
    TEST_ASSERT_EQUAL_UINT32(1U, state.overrun_total);
}

void test_drain_overrun_wins_over_backlog(void)
{
    uint32_t count = 0U;

    // the FIFO is still not empty at the limit (backlog) AND it overran 
    fifo.status = TCAS_A429_HW_NOT_EMPTY | TCAS_A429_HW_OVERRUN;
    fifo.data   = make_word(TCAS_A429_LABEL_ALT_RATE, 0U, 2U,
                            TCAS_A429_SSM_NORMAL);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_OVERRUN,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, out, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(1U, state.overrun_total);
}

void test_drain_overrun_with_empty_fifo(void)
{
    uint32_t count = 99U;

    fifo.status = TCAS_A429_HW_OVERRUN;       // lost words, nothing waiting 

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_OVERRUN,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, out, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(0U, count);
    TEST_ASSERT_EQUAL_UINT32(1U, state.overrun_total);
}

void test_drain_rejected_words_are_still_delivered_with_their_status(void)
{
    uint32_t count = 0U;
    uint32_t i     = 0U;

    fifo.status = TCAS_A429_HW_NOT_EMPTY;
    // wrong parity on every word 
    fifo.data   = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U, 1U,
                            TCAS_A429_SSM_NORMAL) ^ UINT32_C(0x80000000);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_BACKLOG,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, out, TEST_MAX_WORDS, &count));
    TEST_ASSERT_EQUAL_UINT32(TEST_MAX_WORDS, count);
    TEST_ASSERT_EQUAL_UINT32(TEST_MAX_WORDS, state.parity_fail_total);

    for (i = 0U; i < TEST_MAX_WORDS; i++)
    {
        check_rejected(&out[i], TCAS_A429_ERR_PARITY);
    }
}

void test_drain_respects_the_hardware_limit_of_128_words(void)
{
    uint32_t count = 0U;

    make_cfg(&cfg, (uint16_t)TCAS_A429_MAX_WORDS_PER_CYCLE);
    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_OK, tcas_a429_init(&state, &cfg));

    fifo.status = TCAS_A429_HW_NOT_EMPTY;
    fifo.data   = make_word(TCAS_A429_LABEL_PRESSURE_ALT, 0U, 7U,
                            TCAS_A429_SSM_NORMAL);

    TEST_ASSERT_EQUAL_UINT8(TCAS_A429_ERR_BACKLOG,
        tcas_a429_drain(&state, &cfg, &fifo, TEST_FRAME, out,
                        TCAS_A429_MAX_WORDS_PER_CYCLE, &count));
    TEST_ASSERT_EQUAL_UINT32(TCAS_A429_MAX_WORDS_PER_CYCLE, count);
}



// test runner
int main(void)
{
    UNITY_BEGIN();

    // init 
    RUN_TEST(test_init_null_state);
    RUN_TEST(test_init_null_cfg);
    RUN_TEST(test_init_valid_config);
    RUN_TEST(test_init_zero_labels);
    RUN_TEST(test_init_too_many_labels);
    RUN_TEST(test_init_full_label_table_is_accepted);
    RUN_TEST(test_init_duplicate_label);
    RUN_TEST(test_init_max_words_zero);
    RUN_TEST(test_init_max_words_too_big);
    RUN_TEST(test_init_failure_leaves_safe_empty_state);

    // helpers 
    RUN_TEST(test_parity_known_vectors);
    RUN_TEST(test_parity_every_single_bit_flip_toggles_the_result);
    RUN_TEST(test_label_known_vectors);
    RUN_TEST(test_label_reversal_twice_is_identity_for_all_256_values);
    RUN_TEST(test_label_ignores_bits_above_the_label_field);

    // decode_word 
    RUN_TEST(test_decode_valid_word);
    RUN_TEST(test_decode_data_extremes);
    RUN_TEST(test_decode_each_accepted_label);
    RUN_TEST(test_decode_null_out);
    RUN_TEST(test_decode_null_state_still_writes_a_rejected_output);
    RUN_TEST(test_decode_null_cfg);
    RUN_TEST(test_decode_uninitialized_state_is_a_config_error);
    RUN_TEST(test_decode_parity_failure_trusts_nothing);
    RUN_TEST(test_decode_label_not_accepted);
    RUN_TEST(test_decode_ssm_failure_warning);
    RUN_TEST(test_decode_ssm_no_computed_data);
    RUN_TEST(test_decode_ssm_functional_test);
    RUN_TEST(test_order_parity_is_checked_before_label);
    RUN_TEST(test_order_label_is_checked_before_ssm);
    RUN_TEST(test_decode_counters_saturate_and_never_wrap);

    // drain 
    RUN_TEST(test_drain_null_arguments);
    RUN_TEST(test_drain_capacity_smaller_than_max_words);
    RUN_TEST(test_drain_uninitialized_state);
    RUN_TEST(test_drain_empty_fifo);
    RUN_TEST(test_drain_takes_exactly_max_words_and_reports_backlog);
    RUN_TEST(test_drain_overrun_is_reported_and_counted);
    RUN_TEST(test_drain_overrun_wins_over_backlog);
    RUN_TEST(test_drain_overrun_with_empty_fifo);
    RUN_TEST(test_drain_rejected_words_are_still_delivered_with_their_status);
    RUN_TEST(test_drain_respects_the_hardware_limit_of_128_words);

    return UNITY_END();
}