/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : tests/unit/c/test_mpu.c
 * Module      : Unit tests of the memory protection table (Unity)
 * Language    : C11, Unity test framework
 * Assurance   : DO-178C Software Level B (target), verification artifact
 * Tests       : src/c/hal/mpu/tcas_mpu.c and .h
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Checks every behavior promised in tcas_mpu.h: the validation of a
 *   region table (attributes, alignment, range, W xor X, overlap), the two
 *   register words of a region, the pure "would this access be allowed?"
 *   function that models the partition isolation (REQ-001), and the exact
 *   order in which a valid table is programmed. Each test checks one thing
 *   and has a name that says what it is.
 *
 * DESIGN RULES OF THE TESTS
 *   T1  THE TEST IS THE HARDWARE. The three port functions that touch the
 *       real MPU (tcas_mpu_port_disable, _write_region and _enable) are
 *       defined HERE, as a recorder: each call is stored with its kind,
 *       index and words. The test is linked with tcas_mpu.c and no BSP, and
 *       it can check the ORDER of the writes, which the plain-memory fakes
 *       of the other driver tests could not.
 *   T2  Expected register words and results are worked out by hand and
 *       written as literals (see the model table below). The sweeps use
 *       hand-written tables (an overlap matrix, a permission matrix, a
 *       field rule) in a different form from the driver's code.
 *   T3  Every test builds its own table (setUp clears the recorder), so
 *       tests do not depend on each other or on their order.
 *   T4  Entries of a table past "count" hold GARBAGE (every byte 0xFF).
 *       Any use of an unused entry shows up: as a validation error, or as
 *       a non-zero word written to an unused slot.
 *   T5  Every refusal also checks that NO port function was called
 *       (rule R4 of the header).
 *   T6  The boundary of every limit is tested on BOTH sides, and the two
 *       ends of the 32-bit address space are tested on purpose: a region
 *       that ends at 0xFFFFFFFF would wrap if the code ever computed
 *       "limit + 1" (rule R8 of the header).
 *
 * LIMITS (to be listed as an open item for v2)
 *   The recorder shows which words are written and in what order. It
 *   cannot show what the hardware does with them: the fault of a denied
 *   access, the meaning of the access-permission and attribute fields, and
 *   the barrier sequence can only be shown on the target, or in an
 *   emulator that models the Cortex-R52 MPU. Every one of those details is
 *   [VERIFY] in tcas_mpu.h.
 *
 * THE MODEL MEMORY MAP (the test's own partition layout, table index i)
 *   i  region           range                      access   xn
 *   0  core code        0x00000000 .. 0x0003FFFF   RO_ANY   0
 *   1  driver code      0x00040000 .. 0x0004FFFF   RO_ANY   0
 *   2  core constants   0x00050000 .. 0x0005FFFF   RO_PRIV  1
 *   3  core data        0x20000000 .. 0x2000FFFF   RW_PRIV  1
 *   4  driver data      0x20010000 .. 0x2001FFFF   RW_ANY   1
 *   5  device registers 0x40000000 .. 0x4000FFFF   RW_ANY   1   (device)
 *   Regions 0, 1 and 2 are adjacent, and so are 3 and 4: an access that
 *   crosses from one into the next is denied even when both would allow
 *   it. The I/O drivers run unprivileged, the core runs privileged.
 *
 *   HAND-WORKED REGISTER WORDS (PRBAR = base | AP << 2 | XN << 1,
 *   PRLAR = (limit with low 6 bits cleared) | ATTR << 1 | 1)
 *   i  PRBAR        PRLAR
 *   0  0x0000000C   0x0003FFC1     AP 3: 0xC
 *   1  0x0004000C   0x0004FFC1     AP 3: 0xC
 *   2  0x0005000A   0x0005FFC1     AP 2 (8) + XN (2) = 0xA
 *   3  0x20000002   0x2000FFC1     AP 0 + XN (2)
 *   4  0x20010006   0x2001FFC1     AP 1 (4) + XN (2) = 6
 *   5  0x40000006   0x4000FFC3     device: ATTR 1 << 1 = 2, plus EN
 *
 * TESTS BY GROUP
 *   validate  : arguments, count, per-region checks, precedence
 *   overlap   : identical, nested, adjacent, edges of memory, index
 *   encode    : the two register words of a region
 *   allows    : model table, edges of memory, invalid tables, isolation
 *   program   : the exact write sequence, refusals, repeated programming
 *   sweeps    : 144 region field combinations, 252 interval combinations,
 *               108 access combinations
 *
 * USAGE (from the project root; UNITY_DIR is the folder that holds
 * unity.c and unity.h, for example the one used by the MIL1553 project)
 *   UNITY_DIR=../MIL1553/tests/unity   (adjust to your path)
 *   gcc -std=c11 -Wall -Wextra -Wpedantic \
 *       -I src/c/hal/mpu -I $UNITY_DIR \
 *       tests/unit/c/test_mpu.c \
 *       src/c/hal/mpu/tcas_mpu.c \
 *       $UNITY_DIR/unity.c -o build/bin/test_mpu.exe
 *   build/bin/test_mpu.exe
 *
 * MISRA NOTES
 *   * The rules of MISRA C apply to the HAL, not to its tests. The tests
 *     use <string.h> and <stdio.h> and plain loops for clarity.
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "tcas_mpu.h"

// names used by the tests

#define NO_IDX   TCAS_MPU_NO_INDEX
#define SENT8    ((uint8_t)0x55U)          /* "this was not written" */

#define P_PRIV   TCAS_MPU_PRIVILEGED
#define P_USER   TCAS_MPU_UNPRIVILEGED
#define OP_RD    TCAS_MPU_OP_READ
#define OP_WR    TCAS_MPU_OP_WRITE
#define OP_EX    TCAS_MPU_OP_EXEC

// the recorder (rule T1): the three port functions, defined by the test

#define KIND_DISABLE  (1U)
#define KIND_WRITE    (2U)
#define KIND_ENABLE   (3U)
#define MAX_CALLS     (64U)

typedef struct
{
    uint32_t kind;
    uint32_t index;
    uint32_t prbar;
    uint32_t prlar;
} port_call_t;

static port_call_t calls[MAX_CALLS];
static uint32_t    ncalls;                 /* counts EVERY call, even past 64 */

static void record(uint32_t kind, uint32_t index, uint32_t prbar,
                   uint32_t prlar)
{
    if (ncalls < MAX_CALLS)
    {
        calls[ncalls].kind  = kind;
        calls[ncalls].index = index;
        calls[ncalls].prbar = prbar;
        calls[ncalls].prlar = prlar;
    }

    ncalls++;
}

void tcas_mpu_port_disable(void)
{
    record(KIND_DISABLE, 0U, 0U, 0U);
}

void tcas_mpu_port_write_region(uint32_t index, uint32_t prbar, uint32_t prlar)
{
    record(KIND_WRITE, index, prbar, prlar);
}

void tcas_mpu_port_enable(void)
{
    record(KIND_ENABLE, 0U, 0U, 0U);
}

// the model memory map (see the header of this file)

static const uint32_t m_base[6] =
{
    UINT32_C(0x00000000), UINT32_C(0x00040000), UINT32_C(0x00050000),
    UINT32_C(0x20000000), UINT32_C(0x20010000), UINT32_C(0x40000000)
};
static const uint32_t m_limit[6] =
{
    UINT32_C(0x0003FFFF), UINT32_C(0x0004FFFF), UINT32_C(0x0005FFFF),
    UINT32_C(0x2000FFFF), UINT32_C(0x2001FFFF), UINT32_C(0x4000FFFF)
};
static const uint8_t m_access[6] = { 3U, 3U, 2U, 0U, 1U, 1U };
static const uint8_t m_attr[6]   = { 0U, 0U, 0U, 0U, 0U, 1U };
static const uint8_t m_xn[6]     = { 0U, 0U, 1U, 1U, 1U, 1U };

// hand-worked register words of the model table 
static const uint32_t hand_prbar[6] =
{
    UINT32_C(0x0000000C), UINT32_C(0x0004000C), UINT32_C(0x0005000A),
    UINT32_C(0x20000002), UINT32_C(0x20010006), UINT32_C(0x40000006)
};
static const uint32_t hand_prlar[6] =
{
    UINT32_C(0x0003FFC1), UINT32_C(0x0004FFC1), UINT32_C(0x0005FFC1),
    UINT32_C(0x2000FFC1), UINT32_C(0x2001FFC1), UINT32_C(0x4000FFC3)
};

// fixtures and helpers

static tcas_mpu_table_t tbl;
static uint8_t          bad;

static void set_region(tcas_mpu_table_t * t, uint32_t i, uint32_t base,
                       uint32_t limit, uint8_t access, uint8_t attr,
                       uint8_t xn)
{
    t->region[i].base     = base;
    t->region[i].limit    = limit;
    t->region[i].access   = access;
    t->region[i].attr     = attr;
    t->region[i].xn       = xn;
    t->region[i].reserved = 0U;
}

// one region on its own (for the encoders) 
static void reg(tcas_mpu_region_t * r, uint32_t base, uint32_t limit,
                uint8_t access, uint8_t attr, uint8_t xn)
{
    r->base     = base;
    r->limit    = limit;
    r->access   = access;
    r->attr     = attr;
    r->xn       = xn;
    r->reserved = 0U;
}

// the six model regions; every other byte of the table is garbage (T4) 
static void make_model(tcas_mpu_table_t * t)
{
    uint32_t i = 0U;

    (void)memset(t, 0xFF, sizeof(*t));

    for (i = 0U; i < 6U; i++)
    {
        set_region(t, i, m_base[i], m_limit[i], m_access[i], m_attr[i],
                   m_xn[i]);
    }

    t->count = 6U;
}

// n regions from two arrays; read-only, executable, normal memory 
static void make_from(tcas_mpu_table_t * t, uint32_t n,
                      const uint32_t * bases, const uint32_t * limits)
{
    uint32_t i = 0U;

    (void)memset(t, 0xFF, sizeof(*t));

    for (i = 0U; i < n; i++)
    {
        set_region(t, i, bases[i], limits[i], TCAS_MPU_ACCESS_RO_ANY,
                   TCAS_MPU_ATTR_NORMAL, 0U);
    }

    t->count = n;
}

static void make_two(tcas_mpu_table_t * t, uint32_t b0, uint32_t l0,
                     uint32_t b1, uint32_t l1)
{
    const uint32_t b[2] = { b0, b1 };
    const uint32_t l[2] = { l0, l1 };

    make_from(t, 2U, b, l);
}

// n adjacent regions of 0x1000 bytes, from address 0 
static void make_flat(tcas_mpu_table_t * t, uint32_t n)
{
    uint32_t i = 0U;

    (void)memset(t, 0xFF, sizeof(*t));

    for (i = 0U; i < n; i++)
    {
        set_region(t, i, i * UINT32_C(0x1000), (i * UINT32_C(0x1000)) + 0xFFFU,
                   TCAS_MPU_ACCESS_RO_ANY, TCAS_MPU_ATTR_NORMAL, 0U);
    }

    t->count = n;
}

// validates and checks the status and the reported index 
static void expect_validate(const tcas_mpu_table_t * t,
                            tcas_mpu_status_t status, uint8_t index)
{
    bad = SENT8;
    TEST_ASSERT_EQUAL_UINT8(status, tcas_mpu_validate(t, &bad));
    TEST_ASSERT_EQUAL_UINT8(index, bad);
}

/* 1U when the recorder holds the exact programming sequence of a table
 * with "used" regions: disable, slots 0 .. 15 in order (the used ones from
 * the table, the rest zero), enable 
*/
static uint32_t sequence_ok(const tcas_mpu_table_t * t, uint32_t used)
{
    uint32_t ok = 1U;
    uint32_t k  = 0U;

    if (ncalls != 18U)
    {
        ok = 0U;
    }
    else
    {
        if ((calls[0].kind != KIND_DISABLE) || (calls[17].kind != KIND_ENABLE))
        {
            ok = 0U;
        }

        for (k = 0U; k < 16U; k++)
        {
            const uint32_t exp_bar =
                (k < used) ? tcas_mpu_prbar(&t->region[k]) : 0U;
            const uint32_t exp_lar =
                (k < used) ? tcas_mpu_prlar(&t->region[k]) : 0U;

            if ((calls[1U + k].kind != KIND_WRITE) ||
                (calls[1U + k].index != k) ||
                (calls[1U + k].prbar != exp_bar) ||
                (calls[1U + k].prlar != exp_lar))
            {
                ok = 0U;
            }
        }
    }

    return ok;
}

// program refused: the status, the index, and NO port call (rule T5) 
static void expect_refused(const tcas_mpu_table_t * t,
                           tcas_mpu_status_t status, uint8_t index)
{
    ncalls = 0U;
    bad    = SENT8;

    TEST_ASSERT_EQUAL_UINT8(status, tcas_mpu_program(t, &bad));
    TEST_ASSERT_EQUAL_UINT8(index, bad);
    TEST_ASSERT_EQUAL_UINT32(0U, ncalls);
}

// a table of accesses worked out by hand 
typedef struct
{
    uint32_t addr;
    uint32_t size;
    uint8_t  op;
    uint32_t priv;
    uint8_t  exp;
} access_row_t;

static void run_rows(const tcas_mpu_table_t * t, const access_row_t * rows,
                     uint32_t n)
{
    char     msg[48];
    uint32_t i = 0U;

    for (i = 0U; i < n; i++)
    {
        (void)snprintf(msg, sizeof(msg), "access row %u", (unsigned int)i);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(
            rows[i].exp,
            tcas_mpu_allows(t, rows[i].addr, rows[i].size, rows[i].op,
                            rows[i].priv),
            msg);
    }
}

void setUp(void)
{
    (void)memset(&tbl, 0, sizeof(tbl));
    (void)memset(calls, 0, sizeof(calls));
    ncalls = 0U;
    bad    = SENT8;
}

void tearDown(void)
{
}

// group 1: validate, arguments and count

void test_validate_null_table(void)
{
    bad = SENT8;

    TEST_ASSERT_EQUAL_UINT8(TCAS_MPU_ERR_NULL, tcas_mpu_validate(NULL, &bad));
    TEST_ASSERT_EQUAL_UINT8(NO_IDX, bad);          /* written: its pointer is usable */
}

void test_validate_null_bad_index(void)
{
    make_model(&tbl);

    TEST_ASSERT_EQUAL_UINT8(TCAS_MPU_ERR_NULL, tcas_mpu_validate(&tbl, NULL));
}

void test_validate_both_null(void)
{
    TEST_ASSERT_EQUAL_UINT8(TCAS_MPU_ERR_NULL, tcas_mpu_validate(NULL, NULL));
}

void test_validate_count_out_of_range(void)
{
    static const uint32_t counts[4] =
        { 0U, 17U, 100U, UINT32_C(0xFFFFFFFF) };
    uint32_t i = 0U;

    for (i = 0U; i < 4U; i++)
    {
        make_model(&tbl);
        tbl.count = counts[i];
        expect_validate(&tbl, TCAS_MPU_ERR_COUNT, NO_IDX);
    }
}

void test_validate_count_one_and_sixteen(void)
{
    make_model(&tbl);
    tbl.count = 1U;
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);

    make_flat(&tbl, 16U);
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);
}

void test_validate_sixteen_with_overlap(void)
{
    make_flat(&tbl, 16U);
    tbl.region[15].base  = 0U;                      /* the same as region 0 */
    tbl.region[15].limit = 0xFFFU;

    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 15U);
}

void test_validate_ignores_unused_entries(void)
{
    make_model(&tbl);

    TEST_ASSERT_EQUAL_UINT8(0xFFU, tbl.region[6].access);   /* garbage (T4) */
    TEST_ASSERT_EQUAL_UINT8(0xFFU, tbl.region[15].xn);
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);
}

void test_validate_does_not_modify_the_table(void)
{
    tcas_mpu_table_t before;

    make_model(&tbl);
    (void)memcpy(&before, &tbl, sizeof(before));

    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);
    TEST_ASSERT_EQUAL_INT(0, memcmp(&before, &tbl, sizeof(tbl)));

    tbl.region[3].access = 9U;                      /* now it fails: still unchanged */
    (void)memcpy(&before, &tbl, sizeof(before));
    expect_validate(&tbl, TCAS_MPU_ERR_ATTR, 3U);
    TEST_ASSERT_EQUAL_INT(0, memcmp(&before, &tbl, sizeof(tbl)));
}

// group 2: validate, one region at a time

void test_validate_attr_fields(void)
{
    /* per field: two bad values, the first just past the range */
    static const uint8_t bad_values[4][2] =
        { { 4U, 255U }, { 2U, 255U }, { 2U, 255U }, { 1U, 255U } };
    uint32_t field = 0U;
    uint32_t v     = 0U;

    for (field = 0U; field < 4U; field++)
    {
        for (v = 0U; v < 2U; v++)
        {
            make_model(&tbl);

            if (field == 0U)
            {
                tbl.region[2].access = bad_values[field][v];
            }
            else if (field == 1U)
            {
                tbl.region[2].attr = bad_values[field][v];
            }
            else if (field == 2U)
            {
                tbl.region[2].xn = bad_values[field][v];
            }
            else
            {
                tbl.region[2].reserved = bad_values[field][v];
            }

            expect_validate(&tbl, TCAS_MPU_ERR_ATTR, 2U);
        }
    }
}

void test_validate_align(void)
{
    static const uint32_t bad_bases[3] =
        { UINT32_C(0x20000001), UINT32_C(0x2000003F), UINT32_C(0x20000020) };
    static const uint32_t bad_limits[5] =
        { UINT32_C(0x2000FFFE), UINT32_C(0x2000FFC0), UINT32_C(0x2000FFFD),
          UINT32_C(0x2000FF00), UINT32_C(0x00000000) };
    uint32_t i = 0U;

    for (i = 0U; i < 3U; i++)
    {
        make_model(&tbl);
        tbl.region[3].base = bad_bases[i];
        expect_validate(&tbl, TCAS_MPU_ERR_ALIGN, 3U);
    }

    for (i = 0U; i < 5U; i++)
    {
        make_model(&tbl);
        tbl.region[3].limit = bad_limits[i];
        expect_validate(&tbl, TCAS_MPU_ERR_ALIGN, 3U);
    }

    /* one granule in from either end is still aligned */
    make_model(&tbl);
    tbl.region[3].base = UINT32_C(0x20000040);
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);

    make_model(&tbl);
    tbl.region[3].limit = UINT32_C(0x2000FFBF);
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);
}

void test_validate_range(void)
{
    static const uint32_t b1[1] = { UINT32_C(0x1000) };
    static const uint32_t l1[1] = { UINT32_C(0x103F) };       /* one granule */
    static const uint32_t b2[1] = { UINT32_C(0x1040) };
    static const uint32_t l2[1] = { UINT32_C(0x103F) };       /* one below   */
    static const uint32_t b3[1] = { UINT32_C(0xFFFFFFC0) };
    static const uint32_t l3[1] = { UINT32_C(0x0000003F) };   /* far below   */

    /* the limit is the last byte before the base, both aligned */
    make_model(&tbl);
    tbl.region[4].base  = UINT32_C(0x20010000);
    tbl.region[4].limit = UINT32_C(0x2000FFFF);
    expect_validate(&tbl, TCAS_MPU_ERR_RANGE, 4U);

    make_from(&tbl, 1U, b2, l2);
    expect_validate(&tbl, TCAS_MPU_ERR_RANGE, 0U);

    make_from(&tbl, 1U, b3, l3);
    expect_validate(&tbl, TCAS_MPU_ERR_RANGE, 0U);

    /* the smallest region, one granule, is valid */
    make_from(&tbl, 1U, b1, l1);
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);
}

void test_validate_wx(void)
{
    /* access 0, 1 are writable; 2, 3 read only. xn 0 may execute. Only a
     * writable region that may execute is refused */
    static const tcas_mpu_status_t expected[4][2] =
    {
        { TCAS_MPU_ERR_WX, TCAS_MPU_OK },
        { TCAS_MPU_ERR_WX, TCAS_MPU_OK },
        { TCAS_MPU_OK,     TCAS_MPU_OK },
        { TCAS_MPU_OK,     TCAS_MPU_OK }
    };
    uint32_t a = 0U;
    uint32_t x = 0U;

    for (a = 0U; a < 4U; a++)
    {
        for (x = 0U; x < 2U; x++)
        {
            make_model(&tbl);
            tbl.region[3].access = (uint8_t)a;
            tbl.region[3].xn     = (uint8_t)x;

            expect_validate(&tbl, expected[a][x],
                            (expected[a][x] == TCAS_MPU_OK) ? NO_IDX : 3U);
        }
    }
}

void test_validate_precedence_within_a_region(void)
{
    /* attributes before alignment */
    make_model(&tbl);
    tbl.region[1].access = 9U;
    tbl.region[1].base   = UINT32_C(0x00040001);
    expect_validate(&tbl, TCAS_MPU_ERR_ATTR, 1U);

    /* alignment before range: the base is misaligned and the limit is
     * below it */
    make_model(&tbl);
    tbl.region[4].base  = UINT32_C(0x20010001);
    tbl.region[4].limit = UINT32_C(0x2000FFFF);
    expect_validate(&tbl, TCAS_MPU_ERR_ALIGN, 4U);

    /* range before W xor X */
    make_model(&tbl);
    tbl.region[4].base   = UINT32_C(0x20010000);
    tbl.region[4].limit  = UINT32_C(0x2000FFFF);
    tbl.region[4].access = 0U;
    tbl.region[4].xn     = 0U;
    expect_validate(&tbl, TCAS_MPU_ERR_RANGE, 4U);

    /* alignment before W xor X */
    make_model(&tbl);
    tbl.region[4].base   = UINT32_C(0x20010001);
    tbl.region[4].access = 0U;
    tbl.region[4].xn     = 0U;
    expect_validate(&tbl, TCAS_MPU_ERR_ALIGN, 4U);

    /* attributes before W xor X */
    make_model(&tbl);
    tbl.region[4].attr   = 9U;
    tbl.region[4].access = 0U;
    tbl.region[4].xn     = 0U;
    expect_validate(&tbl, TCAS_MPU_ERR_ATTR, 4U);
}

void test_validate_first_failing_region_wins(void)
{
    /* region 1 fails W xor X, region 3 has a bad attribute: the first
     * region in index order is reported */
    make_model(&tbl);
    tbl.region[1].access = 0U;
    tbl.region[1].xn     = 0U;
    tbl.region[3].access = 9U;
    expect_validate(&tbl, TCAS_MPU_ERR_WX, 1U);

    /* region 2 is misaligned, region 4 has a bad attribute */
    make_model(&tbl);
    tbl.region[2].base   = UINT32_C(0x00050001);
    tbl.region[4].access = 9U;
    expect_validate(&tbl, TCAS_MPU_ERR_ALIGN, 2U);
}

void test_validate_single_checks_before_overlap(void)
{
    /* regions 0 and 1 overlap, and region 3 has a bad attribute: the
     * single-region failure is reported, not the overlap */
    make_model(&tbl);
    tbl.region[1].base  = UINT32_C(0x00020000);
    tbl.region[1].limit = UINT32_C(0x0002FFFF);
    tbl.region[3].access = 9U;
    expect_validate(&tbl, TCAS_MPU_ERR_ATTR, 3U);

    /* the same overlap with every region valid: now it is reported */
    make_model(&tbl);
    tbl.region[1].base  = UINT32_C(0x00020000);
    tbl.region[1].limit = UINT32_C(0x0002FFFF);
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);
}

// group 3: overlap

void test_overlap_identical_and_nested(void)
{
    make_two(&tbl, 0x10000U, 0x1FFFFU, 0x10000U, 0x1FFFFU);
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);

    make_two(&tbl, 0x10000U, 0x1FFFFU, 0x14000U, 0x14FFFU);   /* inside */
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);

    make_two(&tbl, 0x14000U, 0x14FFFU, 0x10000U, 0x1FFFFU);   /* around */
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);
}

void test_overlap_one_granule_and_adjacent(void)
{
    /* they share exactly one 64-byte granule: 0x10040 .. 0x1007F */
    make_two(&tbl, 0x10000U, 0x1007FU, 0x10040U, 0x1009FU);
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);

    make_two(&tbl, 0x10040U, 0x1009FU, 0x10000U, 0x1007FU);
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);

    /* adjacent regions share nothing, in either order */
    make_two(&tbl, 0x10000U, 0x1003FU, 0x10040U, 0x1007FU);
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);

    make_two(&tbl, 0x10040U, 0x1007FU, 0x10000U, 0x1003FU);
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);
}

void test_overlap_top_and_bottom_of_memory(void)
{
    /* the top: a region that ends at 0xFFFFFFFF would wrap if the code
     * computed "limit + 1" (rule T6) */
    make_two(&tbl, UINT32_C(0xFFFFFFC0), UINT32_C(0xFFFFFFFF),
                   UINT32_C(0xFFFFFF80), UINT32_C(0xFFFFFFBF));
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);                /* adjacent */

    make_two(&tbl, UINT32_C(0xFFFFFF80), UINT32_C(0xFFFFFFBF),
                   UINT32_C(0xFFFFFFC0), UINT32_C(0xFFFFFFFF));
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);

    make_two(&tbl, UINT32_C(0xFFFFFFC0), UINT32_C(0xFFFFFFFF),
                   UINT32_C(0xFFFFFF80), UINT32_C(0xFFFFFFFF));
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);

    /* the bottom */
    make_two(&tbl, 0x00U, 0x3FU, 0x40U, 0x7FU);
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);

    make_two(&tbl, 0x00U, 0x3FU, 0x00U, 0x3FU);
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);
}

void test_overlap_whole_memory(void)
{
    /* one region that covers every address is valid on its own */
    make_two(&tbl, 0U, UINT32_C(0xFFFFFFFF), 0x1000U, 0x1FFFU);
    tbl.count = 1U;
    expect_validate(&tbl, TCAS_MPU_OK, NO_IDX);

    /* and overlaps anything else, whichever comes first */
    tbl.count = 2U;
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);

    make_two(&tbl, 0x1000U, 0x1FFFU, 0U, UINT32_C(0xFFFFFFFF));
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);
}

void test_overlap_reports_the_later_region(void)
{
    /* regions 2 and 3 both overlap an earlier one: the smallest later
     * index is reported (pair order: j ascending, then i ascending) */
    static const uint32_t b1[4] = { 0x10000U, 0x20000U, 0x14000U, 0x24000U };
    static const uint32_t l1[4] = { 0x1FFFFU, 0x2FFFFU, 0x14FFFU, 0x24FFFU };

    /* region 3 overlaps region 1 only */
    static const uint32_t b2[4] = { 0x10000U, 0x20000U, 0x30000U, 0x24000U };
    static const uint32_t l2[4] = { 0x1FFFFU, 0x2FFFFU, 0x30FFFU, 0x24FFFU };

    /* regions 2 and 3 overlap each other and nothing else */
    static const uint32_t b3[4] = { 0x10000U, 0x20000U, 0x30000U, 0x30000U };
    static const uint32_t l3[4] = { 0x1FFFFU, 0x2FFFFU, 0x30FFFU, 0x30FFFU };

    /* four identical regions */
    static const uint32_t b4[4] = { 0x10000U, 0x10000U, 0x10000U, 0x10000U };
    static const uint32_t l4[4] = { 0x1FFFFU, 0x1FFFFU, 0x1FFFFU, 0x1FFFFU };

    make_from(&tbl, 4U, b1, l1);
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 2U);

    make_from(&tbl, 4U, b2, l2);
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 3U);

    make_from(&tbl, 4U, b3, l3);
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 3U);

    make_from(&tbl, 4U, b4, l4);
    expect_validate(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);
}

// group 4: register words

void test_encode_model_regions(void)
{
    uint32_t i = 0U;

    make_model(&tbl);

    for (i = 0U; i < 6U; i++)
    {
        TEST_ASSERT_EQUAL_HEX32(hand_prbar[i], tcas_mpu_prbar(&tbl.region[i]));
        TEST_ASSERT_EQUAL_HEX32(hand_prlar[i], tcas_mpu_prlar(&tbl.region[i]));
    }
}

void test_encode_access_and_xn_fields(void)
{
    /* base 0x1000: PRBAR = 0x1000 | AP << 2 | XN << 1 */
    static const uint32_t expected[4][2] =
    {
        { 0x1000U, 0x1002U },      /* AP 0 */
        { 0x1004U, 0x1006U },      /* AP 1 */
        { 0x1008U, 0x100AU },      /* AP 2 */
        { 0x100CU, 0x100EU }       /* AP 3 */
    };
    tcas_mpu_region_t r;
    uint32_t a = 0U;
    uint32_t x = 0U;

    for (a = 0U; a < 4U; a++)
    {
        for (x = 0U; x < 2U; x++)
        {
            reg(&r, 0x1000U, 0x1FFFU, (uint8_t)a, 0U, (uint8_t)x);
            TEST_ASSERT_EQUAL_HEX32(expected[a][x], tcas_mpu_prbar(&r));
        }
    }
}

void test_encode_attr_and_limit(void)
{
    tcas_mpu_region_t r;

    /* limit 0x1FFF: the low 6 bits are cleared, 0x1FC0; plus ATTR << 1
     * and the enable bit */
    reg(&r, 0x1000U, 0x1FFFU, 0U, 0U, 1U);
    TEST_ASSERT_EQUAL_HEX32(0x1FC1U, tcas_mpu_prlar(&r));
    reg(&r, 0x1000U, 0x1FFFU, 0U, 1U, 1U);
    TEST_ASSERT_EQUAL_HEX32(0x1FC3U, tcas_mpu_prlar(&r));

    /* the attribute index is masked to three bits: 7 stays 7, 8 becomes
     * 0, 255 becomes 7 */
    reg(&r, 0x1000U, 0x1FFFU, 0U, 7U, 1U);
    TEST_ASSERT_EQUAL_HEX32(0x1FCFU, tcas_mpu_prlar(&r));
    reg(&r, 0x1000U, 0x1FFFU, 0U, 8U, 1U);
    TEST_ASSERT_EQUAL_HEX32(0x1FC1U, tcas_mpu_prlar(&r));
    reg(&r, 0x1000U, 0x1FFFU, 0U, 255U, 1U);
    TEST_ASSERT_EQUAL_HEX32(0x1FCFU, tcas_mpu_prlar(&r));

    /* other limits */
    reg(&r, 0x1000U, 0x103FU, 0U, 0U, 1U);          /* one granule */
    TEST_ASSERT_EQUAL_HEX32(0x1001U, tcas_mpu_prlar(&r));
    reg(&r, 0x1000U, 0x107FU, 0U, 0U, 1U);          /* two granules */
    TEST_ASSERT_EQUAL_HEX32(0x1041U, tcas_mpu_prlar(&r));
    reg(&r, 0x1000U, 0x1040U, 0U, 0U, 1U);          /* an unaligned limit */
    TEST_ASSERT_EQUAL_HEX32(0x1041U, tcas_mpu_prlar(&r));
}

void test_encode_masking(void)
{
    tcas_mpu_region_t r;

    /* every field out of range: each is masked to its own width, so none
     * leaks into a neighbor. access 0xFF -> 3 (0xC), xn 0xFF -> 1 (2) */
    reg(&r, 0U, 0x3FU, 0xFFU, 0xFFU, 0xFFU);
    TEST_ASSERT_EQUAL_HEX32(0x0000000EU, tcas_mpu_prbar(&r));
    TEST_ASSERT_EQUAL_HEX32(0x0000000FU, tcas_mpu_prlar(&r));

    /* unaligned base and limit: the low 6 bits are cleared */
    reg(&r, UINT32_C(0x12345678), UINT32_C(0x12345678), 0U, 0U, 0U);
    TEST_ASSERT_EQUAL_HEX32(0x12345640U, tcas_mpu_prbar(&r));
    TEST_ASSERT_EQUAL_HEX32(0x12345641U, tcas_mpu_prlar(&r));

    /* a base below one granule becomes 0 */
    reg(&r, 0x3FU, 0x3FU, 0U, 0U, 0U);
    TEST_ASSERT_EQUAL_HEX32(0x00000000U, tcas_mpu_prbar(&r));
}

void test_encode_extremes(void)
{
    tcas_mpu_region_t r;

    /* the top of memory */
    reg(&r, UINT32_C(0xFFFFFFC0), UINT32_C(0xFFFFFFFF), 0U, 0U, 1U);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFC2U, tcas_mpu_prbar(&r));
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFC1U, tcas_mpu_prlar(&r));

    reg(&r, UINT32_C(0xFFFFFFC0), UINT32_C(0xFFFFFFFF), 3U, 1U, 0U);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFCCU, tcas_mpu_prbar(&r));
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFC3U, tcas_mpu_prlar(&r));

    /* the bottom of memory */
    reg(&r, 0U, 0x3FU, 0U, 0U, 1U);
    TEST_ASSERT_EQUAL_HEX32(0x00000002U, tcas_mpu_prbar(&r));
    TEST_ASSERT_EQUAL_HEX32(0x00000001U, tcas_mpu_prlar(&r));
}

void test_encode_null_and_reserved(void)
{
    tcas_mpu_region_t a;
    tcas_mpu_region_t b;

    TEST_ASSERT_EQUAL_HEX32(0U, tcas_mpu_prbar(NULL));
    TEST_ASSERT_EQUAL_HEX32(0U, tcas_mpu_prlar(NULL));

    /* the reserved byte never changes a word */
    reg(&a, 0x1000U, 0x1FFFU, 1U, 1U, 1U);
    reg(&b, 0x1000U, 0x1FFFU, 1U, 1U, 1U);
    b.reserved = 0xFFU;
    TEST_ASSERT_EQUAL_HEX32(tcas_mpu_prbar(&a), tcas_mpu_prbar(&b));
    TEST_ASSERT_EQUAL_HEX32(tcas_mpu_prlar(&a), tcas_mpu_prlar(&b));

    /* the same call twice gives the same words (a pure function) */
    TEST_ASSERT_EQUAL_HEX32(tcas_mpu_prbar(&a), tcas_mpu_prbar(&a));
    TEST_ASSERT_EQUAL_HEX32(tcas_mpu_prlar(&a), tcas_mpu_prlar(&a));
}

// group 5: allows

void test_allows_model_table(void)
{
    static const access_row_t rows[] =
    {
        /* core code: read and execute for everyone, write for no one */
        { 0x00000000U, 4U, OP_RD, P_PRIV, 1U },
        { 0x00000000U, 4U, OP_RD, P_USER, 1U },
        { 0x00000000U, 4U, OP_WR, P_PRIV, 0U },
        { 0x00000000U, 4U, OP_WR, P_USER, 0U },
        { 0x00000000U, 4U, OP_EX, P_PRIV, 1U },
        { 0x00000000U, 4U, OP_EX, P_USER, 1U },
        { 0x0003FFFFU, 1U, OP_RD, P_USER, 1U },     /* its last byte     */
        { 0x0003FFFCU, 4U, OP_RD, P_USER, 1U },     /* ends on it        */
        { 0x0003FFFCU, 8U, OP_RD, P_USER, 0U },     /* crosses into 1    */
        { 0x0003FFFCU, 8U, OP_RD, P_PRIV, 0U },

        /* driver code */
        { 0x00040000U, 2U, OP_EX, P_USER, 1U },
        { 0x00040000U, 2U, OP_WR, P_USER, 0U },
        { 0x0004FFFFU, 1U, OP_EX, P_PRIV, 1U },

        /* core constants: read for privileged code only, never written,
         * never executed */
        { 0x00050000U, 4U, OP_RD, P_PRIV, 1U },
        { 0x00050000U, 4U, OP_RD, P_USER, 0U },
        { 0x00050000U, 4U, OP_WR, P_PRIV, 0U },
        { 0x00050000U, 4U, OP_EX, P_PRIV, 0U },
        { 0x0005FFFFU, 1U, OP_RD, P_PRIV, 1U },
        { 0x0005FFFFU, 2U, OP_RD, P_PRIV, 0U },     /* runs past its end */

        /* core data: privileged read and write, never executed, closed to
         * unprivileged code */
        { 0x20000000U, 4U, OP_RD, P_PRIV, 1U },
        { 0x20000000U, 4U, OP_WR, P_PRIV, 1U },
        { 0x20000000U, 4U, OP_EX, P_PRIV, 0U },
        { 0x20000000U, 4U, OP_RD, P_USER, 0U },
        { 0x20000000U, 4U, OP_WR, P_USER, 0U },
        { 0x20000000U, 4U, OP_EX, P_USER, 0U },
        { 0x2000FFFCU, 4U, OP_WR, P_PRIV, 1U },     /* ends on it        */
        { 0x2000FFFCU, 8U, OP_WR, P_PRIV, 0U },     /* crosses into 4    */
        { 0x2000FFFFU, 2U, OP_RD, P_PRIV, 0U },

        /* driver data: read and write for everyone, never executed */
        { 0x20010000U, 4U, OP_RD, P_USER, 1U },
        { 0x20010000U, 4U, OP_WR, P_USER, 1U },
        { 0x20010000U, 4U, OP_EX, P_USER, 0U },
        { 0x20010000U, 4U, OP_EX, P_PRIV, 0U },
        { 0x20010000U, 4U, OP_WR, P_PRIV, 1U },
        { 0x20010000U, 0x10000U, OP_WR, P_USER, 1U },   /* the whole region */
        { 0x20010000U, 0x10001U, OP_WR, P_USER, 0U },   /* one byte more    */
        { 0x2001FFFFU, 1U, OP_WR, P_USER, 1U },
        { 0x2001FFFFU, 2U, OP_WR, P_USER, 0U },

        /* device registers */
        { 0x40000000U, 4U, OP_WR, P_USER, 1U },
        { 0x40000000U, 4U, OP_EX, P_USER, 0U },
        { 0x4000FFFFU, 1U, OP_RD, P_USER, 1U },

        /* addresses that no region covers */
        { 0x00060000U, 4U, OP_RD, P_PRIV, 0U },
        { 0x10000000U, 4U, OP_RD, P_PRIV, 0U },
        { 0x20020000U, 4U, OP_RD, P_PRIV, 0U },
        { 0x3FFFFFFFU, 1U, OP_RD, P_PRIV, 0U },
        { 0x40010000U, 1U, OP_RD, P_PRIV, 0U },
        { UINT32_C(0xFFFFFFFF), 1U, OP_RD, P_PRIV, 0U },

        /* size 0, wraparound, unknown operations */
        { 0x20010000U, 0U, OP_RD, P_PRIV, 0U },
        { UINT32_C(0xFFFFFFFF), 2U, OP_RD, P_PRIV, 0U },
        { 0x20010000U, UINT32_C(0xFFFFFFFF), OP_RD, P_PRIV, 0U },
        { 0x20010000U, 4U, (uint8_t)3U, P_PRIV, 0U },
        { 0x20010000U, 4U, (uint8_t)255U, P_PRIV, 0U },

        /* only exactly TCAS_MPU_PRIVILEGED counts as privileged */
        { 0x20000000U, 4U, OP_RD, 2U, 0U },
        { 0x20000000U, 4U, OP_RD, UINT32_C(0xFFFFFFFF), 0U },
        { 0x20010000U, 4U, OP_RD, 2U, 1U },
        { 0x20010000U, 4U, OP_WR, UINT32_C(0xFFFFFFFF), 1U }
    };

    make_model(&tbl);
    run_rows(&tbl, rows, (uint32_t)(sizeof(rows) / sizeof(rows[0])));
}

void test_allows_top_of_memory(void)
{
    static const uint32_t b[1] = { UINT32_C(0xFFFFFFC0) };
    static const uint32_t l[1] = { UINT32_C(0xFFFFFFFF) };
    static const access_row_t rows[] =
    {
        { UINT32_C(0xFFFFFFC0), 0x40U, OP_WR, P_USER, 1U },   /* exactly to the top */
        { UINT32_C(0xFFFFFFC0), 0x41U, OP_WR, P_USER, 0U },   /* wraps              */
        { UINT32_C(0xFFFFFFFF), 1U,    OP_RD, P_USER, 1U },
        { UINT32_C(0xFFFFFFFF), 2U,    OP_RD, P_USER, 0U },
        { UINT32_C(0xFFFFFFBF), 1U,    OP_RD, P_USER, 0U },   /* just below it      */
        { UINT32_C(0xFFFFFFBF), 2U,    OP_RD, P_USER, 0U },   /* starts below it    */
        { UINT32_C(0xFFFFFFC0), 1U,    OP_EX, P_USER, 0U }    /* xn is 1            */
    };

    make_from(&tbl, 1U, b, l);
    tbl.region[0].access = TCAS_MPU_ACCESS_RW_ANY;
    tbl.region[0].xn     = 1U;

    run_rows(&tbl, rows, (uint32_t)(sizeof(rows) / sizeof(rows[0])));
}

void test_allows_whole_memory(void)
{
    static const uint32_t b[1] = { 0U };
    static const uint32_t l[1] = { UINT32_C(0xFFFFFFFF) };
    static const access_row_t rows[] =
    {
        { 0U, 1U, OP_RD, P_USER, 1U },
        { 0U, 1U, OP_EX, P_USER, 1U },
        { 0U, 1U, OP_WR, P_PRIV, 0U },
        { UINT32_C(0xFFFFFFFF), 1U, OP_RD, P_USER, 1U },
        { 0U, UINT32_C(0xFFFFFFFF), OP_RD, P_USER, 1U },   /* every byte but one  */
        { 1U, UINT32_C(0xFFFFFFFF), OP_RD, P_USER, 1U },   /* ends on the top     */
        { 2U, UINT32_C(0xFFFFFFFF), OP_RD, P_USER, 0U }    /* would pass the top  */
    };

    make_from(&tbl, 1U, b, l);

    run_rows(&tbl, rows, (uint32_t)(sizeof(rows) / sizeof(rows[0])));
}

void test_allows_invalid_tables(void)
{
    static const uint32_t b[1] = { 0x1000U };
    static const uint32_t l[1] = { 0x1FFFU };
    static const access_row_t inside_rd[] = { { 0x1000U, 4U, OP_RD, P_PRIV, 0U } };
    static const access_row_t valid_rows[] =
    {
        { 0x1000U, 4U, OP_WR, P_PRIV, 1U },
        { 0x1000U, 4U, OP_EX, P_PRIV, 0U }
    };
    static const access_row_t model_rd[] =
        { { 0x20000000U, 4U, OP_RD, P_PRIV, 0U } };

    /* no table */
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(NULL, 0x1000U, 4U, OP_RD, P_PRIV));

    /* a writable and executable region: the table is not valid, so even an
     * access that the region would grant is denied */
    make_from(&tbl, 1U, b, l);
    tbl.region[0].access = 0U;
    tbl.region[0].xn     = 0U;
    run_rows(&tbl, inside_rd, 1U);

    /* the same region made execute-never is valid, and the access is
     * granted: the failure above was the table, not the address */
    tbl.region[0].xn = 1U;
    run_rows(&tbl, valid_rows, 2U);

    /* an overlapping table */
    make_two(&tbl, 0x1000U, 0x1FFFU, 0x1000U, 0x1FFFU);
    run_rows(&tbl, inside_rd, 1U);

    /* a misaligned region */
    make_from(&tbl, 1U, b, l);
    tbl.region[0].base = 0x1001U;
    run_rows(&tbl, inside_rd, 1U);

    /* no regions at all */
    make_model(&tbl);
    tbl.count = 0U;
    run_rows(&tbl, model_rd, 1U);

    /* too many regions */
    make_model(&tbl);
    tbl.count = 17U;
    run_rows(&tbl, model_rd, 1U);
}

void test_allows_partition_isolation(void)
{
    /* REQ-001: the I/O drivers (unprivileged) cannot corrupt the core */
    make_model(&tbl);

    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x20000000U, 4U, OP_WR, P_USER));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x20000000U, 4U, OP_RD, P_USER));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x2000FFFCU, 4U, OP_WR, P_USER));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x00050000U, 4U, OP_RD, P_USER));

    /* nobody can write any code or constant, not even the core */
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x00000000U, 4U, OP_WR, P_PRIV));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x00040000U, 4U, OP_WR, P_PRIV));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x00050000U, 4U, OP_WR, P_PRIV));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x00040000U, 4U, OP_WR, P_USER));

    /* data can never run as code (W xor X) */
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x20010000U, 4U, OP_EX, P_USER));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x20010000U, 4U, OP_EX, P_PRIV));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x20000000U, 4U, OP_EX, P_PRIV));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x40000000U, 4U, OP_EX, P_USER));

    /* the drivers can do their own work */
    TEST_ASSERT_EQUAL_UINT8(1U, tcas_mpu_allows(&tbl, 0x20010000U, 4U, OP_WR, P_USER));
    TEST_ASSERT_EQUAL_UINT8(1U, tcas_mpu_allows(&tbl, 0x20010000U, 4U, OP_RD, P_USER));
    TEST_ASSERT_EQUAL_UINT8(1U, tcas_mpu_allows(&tbl, 0x40000000U, 4U, OP_WR, P_USER));
    TEST_ASSERT_EQUAL_UINT8(1U, tcas_mpu_allows(&tbl, 0x00040000U, 4U, OP_EX, P_USER));

    /* and the core can read what the drivers produced, but not run it */
    TEST_ASSERT_EQUAL_UINT8(1U, tcas_mpu_allows(&tbl, 0x20010000U, 4U, OP_RD, P_PRIV));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x20010000U, 4U, OP_EX, P_PRIV));

    /* one access cannot reach from a driver buffer into the core data
     * next to it */
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x2000FFFCU, 8U, OP_WR, P_PRIV));
    TEST_ASSERT_EQUAL_UINT8(0U, tcas_mpu_allows(&tbl, 0x2000FFFCU, 8U, OP_RD, P_USER));
}

void test_allows_no_side_effects(void)
{
    tcas_mpu_table_t before;

    make_model(&tbl);
    (void)memcpy(&before, &tbl, sizeof(before));
    ncalls = 0U;

    (void)tcas_mpu_allows(&tbl, 0x20010000U, 4U, OP_WR, P_USER);
    (void)tcas_mpu_allows(&tbl, 0x20000000U, 4U, OP_WR, P_USER);
    (void)tcas_mpu_allows(&tbl, UINT32_C(0xFFFFFFFF), 2U, OP_RD, P_PRIV);

    TEST_ASSERT_EQUAL_INT(0, memcmp(&before, &tbl, sizeof(tbl)));
    TEST_ASSERT_EQUAL_UINT32(0U, ncalls);          /* never touches the port */
}

// group 6: program

void test_program_model_table(void)
{
    tcas_mpu_table_t before;
    uint32_t k = 0U;

    make_model(&tbl);
    (void)memcpy(&before, &tbl, sizeof(before));
    ncalls = 0U;
    bad    = SENT8;

    TEST_ASSERT_EQUAL_UINT8(TCAS_MPU_OK, tcas_mpu_program(&tbl, &bad));
    TEST_ASSERT_EQUAL_UINT8(NO_IDX, bad);
    TEST_ASSERT_EQUAL_UINT32(18U, ncalls);

    /* first the MPU is turned off */
    TEST_ASSERT_EQUAL_UINT32(KIND_DISABLE, calls[0].kind);

    /* then the six used slots, with the hand-worked words */
    for (k = 0U; k < 6U; k++)
    {
        TEST_ASSERT_EQUAL_UINT32(KIND_WRITE, calls[1U + k].kind);
        TEST_ASSERT_EQUAL_UINT32(k, calls[1U + k].index);
        TEST_ASSERT_EQUAL_HEX32(hand_prbar[k], calls[1U + k].prbar);
        TEST_ASSERT_EQUAL_HEX32(hand_prlar[k], calls[1U + k].prlar);
    }

    /* then the ten unused slots, both words 0: the 0xFF garbage in the
     * table never reaches the hardware (rule T4) */
    for (k = 6U; k < 16U; k++)
    {
        TEST_ASSERT_EQUAL_UINT32(KIND_WRITE, calls[1U + k].kind);
        TEST_ASSERT_EQUAL_UINT32(k, calls[1U + k].index);
        TEST_ASSERT_EQUAL_HEX32(0U, calls[1U + k].prbar);
        TEST_ASSERT_EQUAL_HEX32(0U, calls[1U + k].prlar);
    }

    /* and last the MPU is turned on */
    TEST_ASSERT_EQUAL_UINT32(KIND_ENABLE, calls[17].kind);

    TEST_ASSERT_EQUAL_INT(0, memcmp(&before, &tbl, sizeof(tbl)));
}

void test_program_refuses_invalid_tables(void)
{
    /* no table */
    expect_refused(NULL, TCAS_MPU_ERR_NULL, NO_IDX);

    /* a valid table, but nowhere to report the index: nothing is written */
    make_model(&tbl);
    ncalls = 0U;
    TEST_ASSERT_EQUAL_UINT8(TCAS_MPU_ERR_NULL, tcas_mpu_program(&tbl, NULL));
    TEST_ASSERT_EQUAL_UINT32(0U, ncalls);

    /* the count */
    make_model(&tbl);
    tbl.count = 0U;
    expect_refused(&tbl, TCAS_MPU_ERR_COUNT, NO_IDX);

    make_model(&tbl);
    tbl.count = 17U;
    expect_refused(&tbl, TCAS_MPU_ERR_COUNT, NO_IDX);

    /* each kind of region fault */
    make_model(&tbl);
    tbl.region[2].access = 4U;
    expect_refused(&tbl, TCAS_MPU_ERR_ATTR, 2U);

    make_model(&tbl);
    tbl.region[3].base = UINT32_C(0x20000001);
    expect_refused(&tbl, TCAS_MPU_ERR_ALIGN, 3U);

    make_model(&tbl);
    tbl.region[4].base  = UINT32_C(0x20010000);
    tbl.region[4].limit = UINT32_C(0x2000FFFF);
    expect_refused(&tbl, TCAS_MPU_ERR_RANGE, 4U);

    make_model(&tbl);
    tbl.region[3].xn = 0U;
    expect_refused(&tbl, TCAS_MPU_ERR_WX, 3U);

    /* an overlap */
    make_two(&tbl, 0x1000U, 0x1FFFU, 0x1000U, 0x1FFFU);
    expect_refused(&tbl, TCAS_MPU_ERR_OVERLAP, 1U);
}

void test_program_single_region(void)
{
    static const uint32_t b[1] = { 0x1000U };
    static const uint32_t l[1] = { 0x1FFFU };

    make_from(&tbl, 1U, b, l);
    ncalls = 0U;
    bad    = SENT8;

    TEST_ASSERT_EQUAL_UINT8(TCAS_MPU_OK, tcas_mpu_program(&tbl, &bad));
    TEST_ASSERT_EQUAL_UINT32(1U, sequence_ok(&tbl, 1U));

    /* access 3 (0xC), xn 0: PRBAR 0x100C; limit 0x1FFF -> 0x1FC0, plus
     * the enable bit: PRLAR 0x1FC1. Every other slot is empty */
    TEST_ASSERT_EQUAL_HEX32(0x100CU, calls[1].prbar);
    TEST_ASSERT_EQUAL_HEX32(0x1FC1U, calls[1].prlar);
    TEST_ASSERT_EQUAL_HEX32(0U, calls[2].prbar);
    TEST_ASSERT_EQUAL_HEX32(0U, calls[16].prlar);
}

void test_program_sixteen_regions(void)
{
    uint32_t k = 0U;

    make_flat(&tbl, 16U);
    ncalls = 0U;
    bad    = SENT8;

    TEST_ASSERT_EQUAL_UINT8(TCAS_MPU_OK, tcas_mpu_program(&tbl, &bad));
    TEST_ASSERT_EQUAL_UINT32(18U, ncalls);
    TEST_ASSERT_EQUAL_UINT32(1U, sequence_ok(&tbl, 16U));

    /* region k: base k * 0x1000, access 3 (0xC), limit base + 0xFFF,
     * whose low 6 bits cleared give base + 0xFC0, plus the enable bit */
    for (k = 0U; k < 16U; k++)
    {
        TEST_ASSERT_EQUAL_HEX32((k * 0x1000U) | 0xCU, calls[1U + k].prbar);
        TEST_ASSERT_EQUAL_HEX32((k * 0x1000U) + 0xFC1U, calls[1U + k].prlar);
    }
}

void test_program_twice(void)
{
    uint32_t k = 0U;

    make_model(&tbl);
    ncalls = 0U;

    TEST_ASSERT_EQUAL_UINT8(TCAS_MPU_OK, tcas_mpu_program(&tbl, &bad));
    TEST_ASSERT_EQUAL_UINT8(TCAS_MPU_OK, tcas_mpu_program(&tbl, &bad));
    TEST_ASSERT_EQUAL_UINT32(36U, ncalls);

    /* the second sequence is the same as the first */
    for (k = 0U; k < 18U; k++)
    {
        TEST_ASSERT_EQUAL_UINT32(calls[k].kind,  calls[18U + k].kind);
        TEST_ASSERT_EQUAL_UINT32(calls[k].index, calls[18U + k].index);
        TEST_ASSERT_EQUAL_HEX32(calls[k].prbar,  calls[18U + k].prbar);
        TEST_ASSERT_EQUAL_HEX32(calls[k].prlar,  calls[18U + k].prlar);
    }
}

// group 7: sweeps against hand-written tables (rule T2)

/* 6 x 4 x 3 x 2 = 144 single regions: the access code, the attribute, the
 * xn flag and the reserved byte each take some good and some bad values.
 * The expected result follows a hand-written rule:
 *   any field out of range, or reserved not 0     ERR_ATTR
 *   else writable (access 0 or 1) and xn 0        ERR_WX
 *   else                                          OK
 * The table is also programmed: 18 port calls when valid, none otherwise. 
*/
void test_sweep_region_fields(void)
{
    static const uint32_t b[1] = { 0x1000U };
    static const uint32_t l[1] = { 0x1FFFU };
    static const uint8_t accs[6]  = { 0U, 1U, 2U, 3U, 4U, 255U };
    static const uint8_t attrs[4] = { 0U, 1U, 2U, 255U };
    static const uint8_t xns[3]   = { 0U, 1U, 2U };

    uint32_t combos = 0U;
    uint32_t nbad   = 0U;
    uint32_t ai = 0U;
    uint32_t ti = 0U;
    uint32_t xi = 0U;
    uint32_t ri = 0U;

    for (ai = 0U; ai < 6U; ai++)
    {
        for (ti = 0U; ti < 4U; ti++)
        {
            for (xi = 0U; xi < 3U; xi++)
            {
                for (ri = 0U; ri < 2U; ri++)
                {
                    const uint8_t field_bad =
                        ((accs[ai] > 3U) || (attrs[ti] > 1U) ||
                         (xns[xi] > 1U) || (ri != 0U)) ? 1U : 0U;
                    const uint8_t wx =
                        ((accs[ai] < 2U) && (xns[xi] == 0U)) ? 1U : 0U;
                    const tcas_mpu_status_t exp_status =
                        (field_bad == 1U) ? TCAS_MPU_ERR_ATTR
                        : ((wx == 1U) ? TCAS_MPU_ERR_WX : TCAS_MPU_OK);
                    const uint8_t exp_index =
                        (exp_status == TCAS_MPU_OK) ? NO_IDX : 0U;

                    uint8_t idx = SENT8;
                    uint32_t ok = 1U;

                    make_from(&tbl, 1U, b, l);
                    tbl.region[0].access   = accs[ai];
                    tbl.region[0].attr     = attrs[ti];
                    tbl.region[0].xn       = xns[xi];
                    tbl.region[0].reserved = (uint8_t)ri;

                    if ((tcas_mpu_validate(&tbl, &idx) != exp_status) ||
                        (idx != exp_index))
                    {
                        ok = 0U;
                    }

                    ncalls = 0U;
                    idx    = SENT8;
                    if ((tcas_mpu_program(&tbl, &idx) != exp_status) ||
                        (idx != exp_index))
                    {
                        ok = 0U;
                    }

                    if (exp_status == TCAS_MPU_OK)
                    {
                        if (sequence_ok(&tbl, 1U) != 1U)
                        {
                            ok = 0U;
                        }
                    }
                    else if (ncalls != 0U)
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
                            (void)printf("sweep first mismatch: access %u "
                                         "attr %u xn %u reserved %u\n",
                                         (unsigned int)accs[ai],
                                         (unsigned int)attrs[ti],
                                         (unsigned int)xns[xi],
                                         (unsigned int)ri);
                        }
                        nbad++;
                    }
                }
            }
        }
    }

    TEST_ASSERT_EQUAL_UINT32(144U, combos);
    TEST_ASSERT_EQUAL_UINT32(0U, nbad);
}

/* six aligned intervals, worked out by hand, and which of them overlap:
 *   I0 0x1000..0x1FFF   I1 0x1000..0x103F (inside I0)   I2 0x2000..0x2FFF
 *   I3 0x1FC0..0x203F (reaches into I0 and I2)          I4 0x3000..0x3FFF
 *   I5 0xFFFFF000..0xFFFFFFFF (the top of memory)
 * I0 and I2 are adjacent, and so are I2 and I4. 
*/
static const uint32_t i_base[6] =
{
    UINT32_C(0x00001000), UINT32_C(0x00001000), UINT32_C(0x00002000),
    UINT32_C(0x00001FC0), UINT32_C(0x00003000), UINT32_C(0xFFFFF000)
};
static const uint32_t i_limit[6] =
{
    UINT32_C(0x00001FFF), UINT32_C(0x0000103F), UINT32_C(0x00002FFF),
    UINT32_C(0x0000203F), UINT32_C(0x00003FFF), UINT32_C(0xFFFFFFFF)
};
static const uint8_t overlap_matrix[6][6] =
{
    /*        I0  I1  I2  I3  I4  I5 */
    /* I0 */ { 1U, 1U, 0U, 1U, 0U, 0U },
    /* I1 */ { 1U, 1U, 0U, 0U, 0U, 0U },
    /* I2 */ { 0U, 0U, 1U, 1U, 0U, 0U },
    /* I3 */ { 1U, 0U, 1U, 1U, 0U, 0U },
    /* I4 */ { 0U, 0U, 0U, 0U, 1U, 0U },
    /* I5 */ { 0U, 0U, 0U, 0U, 0U, 1U }
};

/* builds a table from n intervals (n is 2 or 3), validates and programs it,
 * and compares with the matrix. Returns 1U when everything agrees. The
 * expected answer: the smallest later region that overlaps an earlier one
 * is reported; no overlap means OK 
*/
static uint32_t interval_case(const uint32_t * ids, uint32_t n)
{
    uint32_t b[3];
    uint32_t l[3];
    uint32_t k = 0U;
    uint32_t ok = 1U;
    uint8_t  idx = SENT8;
    tcas_mpu_status_t exp_status = TCAS_MPU_OK;
    uint8_t  exp_index = NO_IDX;

    for (k = 0U; k < n; k++)
    {
        b[k] = i_base[ids[k]];
        l[k] = i_limit[ids[k]];
    }

    make_from(&tbl, n, b, l);

    if (overlap_matrix[ids[0]][ids[1]] == 1U)
    {
        exp_status = TCAS_MPU_ERR_OVERLAP;
        exp_index  = 1U;
    }
    else if ((n == 3U) && ((overlap_matrix[ids[0]][ids[2]] == 1U) ||
                           (overlap_matrix[ids[1]][ids[2]] == 1U)))
    {
        exp_status = TCAS_MPU_ERR_OVERLAP;
        exp_index  = 2U;
    }
    else
    {
        /* no overlap: valid */
    }

    if ((tcas_mpu_validate(&tbl, &idx) != exp_status) || (idx != exp_index))
    {
        ok = 0U;
    }

    ncalls = 0U;
    idx    = SENT8;
    if ((tcas_mpu_program(&tbl, &idx) != exp_status) || (idx != exp_index))
    {
        ok = 0U;
    }

    if (exp_status == TCAS_MPU_OK)
    {
        if (sequence_ok(&tbl, n) != 1U)
        {
            ok = 0U;
        }
    }
    else if (ncalls != 0U)
    {
        ok = 0U;
    }
    else
    {
        /* refused and nothing written: fine */
    }

    return ok;
}

// 36 ordered pairs and 216 ordered triples of the six intervals above, 252 tables in all, against the hand-written matrix 
void test_sweep_overlap_intervals(void)
{
    uint32_t combos = 0U;
    uint32_t nbad   = 0U;
    uint32_t a = 0U;
    uint32_t b = 0U;
    uint32_t c = 0U;

    for (a = 0U; a < 6U; a++)
    {
        for (b = 0U; b < 6U; b++)
        {
            const uint32_t pair[2] = { a, b };

            combos++;
            if (interval_case(pair, 2U) != 1U)
            {
                if (nbad == 0U)
                {
                    (void)printf("sweep first mismatch: pair I%u I%u\n",
                                 (unsigned int)a, (unsigned int)b);
                }
                nbad++;
            }

            for (c = 0U; c < 6U; c++)
            {
                const uint32_t triple[3] = { a, b, c };

                combos++;
                if (interval_case(triple, 3U) != 1U)
                {
                    if (nbad == 0U)
                    {
                        (void)printf("sweep first mismatch: triple "
                                     "I%u I%u I%u\n",
                                     (unsigned int)a, (unsigned int)b,
                                     (unsigned int)c);
                    }
                    nbad++;
                }
            }
        }
    }

    TEST_ASSERT_EQUAL_UINT32(252U, combos);
    TEST_ASSERT_EQUAL_UINT32(0U, nbad);
}

/* 6 regions x 3 addresses (first byte, 0x1000 in, last byte) x 3
 * operations x 2 privileges = 108 accesses of one byte. The permission
 * matrix is written by hand from the model table:
 *   expected[region][operation][privilege], operation 0 read, 1 write,
 *   2 execute; privilege 0 privileged, 1 unprivileged 
*/
void test_sweep_allows_matrix(void)
{
    static const uint8_t expected[6][3][2] =
    {
        { { 1U, 1U }, { 0U, 0U }, { 1U, 1U } },   /* 0 core code     RO_ANY  xn0 */
        { { 1U, 1U }, { 0U, 0U }, { 1U, 1U } },   /* 1 driver code   RO_ANY  xn0 */
        { { 1U, 0U }, { 0U, 0U }, { 0U, 0U } },   /* 2 core consts   RO_PRIV xn1 */
        { { 1U, 0U }, { 1U, 0U }, { 0U, 0U } },   /* 3 core data     RW_PRIV xn1 */
        { { 1U, 1U }, { 1U, 1U }, { 0U, 0U } },   /* 4 driver data   RW_ANY  xn1 */
        { { 1U, 1U }, { 1U, 1U }, { 0U, 0U } }    /* 5 device        RW_ANY  xn1 */
    };

    uint32_t combos = 0U;
    uint32_t nbad   = 0U;
    uint32_t r  = 0U;
    uint32_t ai = 0U;
    uint32_t op = 0U;
    uint32_t pv = 0U;

    make_model(&tbl);

    for (r = 0U; r < 6U; r++)
    {
        for (ai = 0U; ai < 3U; ai++)
        {
            const uint32_t addr = (ai == 0U) ? m_base[r]
                                : ((ai == 1U) ? (m_base[r] + 0x1000U)
                                              : m_limit[r]);

            for (op = 0U; op < 3U; op++)
            {
                for (pv = 0U; pv < 2U; pv++)
                {
                    const uint32_t priv = (pv == 0U) ? P_PRIV : P_USER;
                    const uint8_t  got  =
                        tcas_mpu_allows(&tbl, addr, 1U, (tcas_mpu_op_t)op, priv);

                    combos++;

                    if (got != expected[r][op][pv])
                    {
                        if (nbad == 0U)
                        {
                            (void)printf("sweep first mismatch: region %u "
                                         "address 0x%08X op %u privilege %u\n",
                                         (unsigned int)r, (unsigned int)addr,
                                         (unsigned int)op, (unsigned int)pv);
                        }
                        nbad++;
                    }
                }
            }
        }
    }

    TEST_ASSERT_EQUAL_UINT32(108U, combos);
    TEST_ASSERT_EQUAL_UINT32(0U, nbad);
}

// test runner

int main(void)
{
    UNITY_BEGIN();

    /* validate: arguments and count */
    RUN_TEST(test_validate_null_table);
    RUN_TEST(test_validate_null_bad_index);
    RUN_TEST(test_validate_both_null);
    RUN_TEST(test_validate_count_out_of_range);
    RUN_TEST(test_validate_count_one_and_sixteen);
    RUN_TEST(test_validate_sixteen_with_overlap);
    RUN_TEST(test_validate_ignores_unused_entries);
    RUN_TEST(test_validate_does_not_modify_the_table);

    /* validate: one region at a time */
    RUN_TEST(test_validate_attr_fields);
    RUN_TEST(test_validate_align);
    RUN_TEST(test_validate_range);
    RUN_TEST(test_validate_wx);
    RUN_TEST(test_validate_precedence_within_a_region);
    RUN_TEST(test_validate_first_failing_region_wins);
    RUN_TEST(test_validate_single_checks_before_overlap);

    /* overlap */
    RUN_TEST(test_overlap_identical_and_nested);
    RUN_TEST(test_overlap_one_granule_and_adjacent);
    RUN_TEST(test_overlap_top_and_bottom_of_memory);
    RUN_TEST(test_overlap_whole_memory);
    RUN_TEST(test_overlap_reports_the_later_region);

    /* register words */
    RUN_TEST(test_encode_model_regions);
    RUN_TEST(test_encode_access_and_xn_fields);
    RUN_TEST(test_encode_attr_and_limit);
    RUN_TEST(test_encode_masking);
    RUN_TEST(test_encode_extremes);
    RUN_TEST(test_encode_null_and_reserved);

    /* allows */
    RUN_TEST(test_allows_model_table);
    RUN_TEST(test_allows_top_of_memory);
    RUN_TEST(test_allows_whole_memory);
    RUN_TEST(test_allows_invalid_tables);
    RUN_TEST(test_allows_partition_isolation);
    RUN_TEST(test_allows_no_side_effects);

    /* program */
    RUN_TEST(test_program_model_table);
    RUN_TEST(test_program_refuses_invalid_tables);
    RUN_TEST(test_program_single_region);
    RUN_TEST(test_program_sixteen_regions);
    RUN_TEST(test_program_twice);

    /* sweeps */
    RUN_TEST(test_sweep_region_fields);
    RUN_TEST(test_sweep_overlap_intervals);
    RUN_TEST(test_sweep_allows_matrix);

    return UNITY_END();
}