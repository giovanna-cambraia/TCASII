/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : tests/unit/c/check_modes_layout.c
 * Module      : Layout and constant cross-check, C side versus Ada side
 * Language    : C11
 * Assurance   : DO-178C Software Level B (target), verification artifact
 * Checks      : src/c/drivers/modes_s/tcas_modes_drv.h
 *               against src/bindings/tcas-modes_port.ads
 *               and the limits in src/ada/common/config/tcas-config.ads
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   The C driver leaves a tcas_modes_frame_t in memory and the SPARK parser
 *   reads it as Tcas.Modes_Port.Frame, in place, without a copy. If the two
 *   descriptions differ by one byte, the parser reads garbage and nothing
 *   in either language notices. This file is the check that they agree:
 *   size, field offsets, field widths, and every constant that has a twin
 *   on the other side.
 *
 *   The hardware mailbox (tcas_modes_hw_mailbox_t) never crosses into Ada,
 *   so it is not checked here. Its layout is a placeholder until the
 *   transceiver is chosen.
 *
 * DESIGN RULES OF THE CHECK
 *   C1  The Ada side is written here as LITERALS, copied from the
 *       representation clause and the constants of tcas-modes_port.ads and
 *       tcas-config.ads. The check never reads the Ada files, so the two
 *       descriptions stay independent: they agree only if both are right.
 *   C2  Layout is checked at COMPILE time (_Static_assert). A mismatch
 *       stops the build, so it cannot be skipped or ignored.
 *   C3  Constants are checked at RUN time, one line per constant, so a
 *       failure names the constant that differs.
 *   C4  The layout table is printed as well, so that a person can read it
 *       and compare it with the Ada file by eye (the evidence for review).
 *   C5  Exit status is 1 if anything differs, else 0, so a script can use
 *       it.
 *
 * WHEN TO RUN IT
 *   After any change to tcas_modes_drv.h, tcas-modes_port.ads or the
 *   Mode S limits in tcas-config.ads, and always before a release. When it
 *   fails, fix the side that is wrong, then change the literals here only
 *   if the Ada side was changed on purpose.
 *
 * USAGE (from the project root)
 *   gcc -std=c11 -Wall -Wextra -Wpedantic \
 *       -I src/c/drivers/modes_s \
 *       tests/unit/c/check_modes_layout.c -o build/bin/check_modes_layout.exe
 *   build/bin/check_modes_layout.exe
 *
 * NOTES
 *   * Offsets are in bytes from the start of the structure. The Ada
 *     representation clause uses the same unit ("Length at 14 range 0 ..
 *     7"). The data field is a 14-byte array in both languages
 *     (Frame_Index is 0 .. 13 in Ada).
 *   * The rules of MISRA C apply to the driver, not to this check, which
 *     uses <stdio.h> for the report.
*/

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "tcas_modes_drv.h"


/* ---------------------------------------------------------------------------
 * layout, checked at compile time (rule C2)
 * each number is copied from the representation clause of
 * Tcas.Modes_Port.Frame
 * ------------------------------------------------------------------------ */

TCAS_STATIC_ASSERT(sizeof(tcas_modes_frame_t) == 24U,
                   "size: Ada says Frame'Size = 192 bits");

TCAS_STATIC_ASSERT(offsetof(tcas_modes_frame_t, data) == 0U,
                   "Data at 0");
TCAS_STATIC_ASSERT(offsetof(tcas_modes_frame_t, length) == 14U,
                   "Length at 14");
TCAS_STATIC_ASSERT(offsetof(tcas_modes_frame_t, status) == 15U,
                   "Status at 15");
TCAS_STATIC_ASSERT(offsetof(tcas_modes_frame_t, round_trip_ns) == 16U,
                   "Round_Trip_Ns at 16");
TCAS_STATIC_ASSERT(offsetof(tcas_modes_frame_t, bearing) == 20U,
                   "Bearing at 20");
TCAS_STATIC_ASSERT(offsetof(tcas_modes_frame_t, valid) == 22U,
                   "Valid at 22");
TCAS_STATIC_ASSERT(offsetof(tcas_modes_frame_t, reserved) == 23U,
                   "Reserved at 23");

// field widths: Ada Data = 14 bytes (range 0 .. 111 bits), U8 = 1 byte, U16 = 2 bytes, U32 = 4 bytes 
TCAS_STATIC_ASSERT(sizeof(((tcas_modes_frame_t *)0)->data) == 14U,
                   "Data is 14 bytes");
TCAS_STATIC_ASSERT(sizeof(((tcas_modes_frame_t *)0)->length) == 1U,
                   "Length is 8 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_modes_frame_t *)0)->status) == 1U,
                   "Status is 8 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_modes_frame_t *)0)->round_trip_ns) == 4U,
                   "Round_Trip_Ns is 32 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_modes_frame_t *)0)->bearing) == 2U,
                   "Bearing is 16 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_modes_frame_t *)0)->valid) == 1U,
                   "Valid is 8 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_modes_frame_t *)0)->reserved) == 1U,
                   "Reserved is 8 bits");

// the data array has one element per byte: Frame_Index is 0 .. 13 
TCAS_STATIC_ASSERT(sizeof(((tcas_modes_frame_t *)0)->data[0]) == 1U,
                   "Data elements are bytes");

// constants, checked at run time (rule C3)

static unsigned int failed = 0U;
static unsigned int total  = 0U;

// compares one C constant with its Ada twin and reports a difference 
static void check_const(const char * name,
                        unsigned int c_value,
                        unsigned int ada_value)
{
    total++;

    if (c_value != ada_value)
    {
        failed++;
        printf("FAIL: %s  C = %u, Ada = %u\n", name, c_value, ada_value);
    }
}

#define CHECK(c_macro, ada_literal) \
    check_const(#c_macro, (unsigned int)(c_macro), (unsigned int)(ada_literal))

int main(void)
{
    /* status codes: Status_* in tcas-modes_port.ads */
    CHECK(TCAS_MODES_OK,            0U);   /* Status_OK            */
    CHECK(TCAS_MODES_ERR_NULL,      1U);   /* Status_Err_Null      */
    CHECK(TCAS_MODES_ERR_CONFIG,    2U);   /* Status_Err_Config    */
    CHECK(TCAS_MODES_ERR_LENGTH,    3U);   /* Status_Err_Length    */
    CHECK(TCAS_MODES_ERR_NO_REPLY,  4U);   /* Status_Err_No_Reply  */
    CHECK(TCAS_MODES_ERR_TIMESTAMP, 5U);   /* Status_Err_Timestamp */
    CHECK(TCAS_MODES_ERR_OVERRUN,   6U);   /* Status_Err_Overrun   */

    /* frame lengths: Short_Length and Long_Length in tcas-modes_port.ads */
    CHECK(TCAS_MODES_SHORT_LENGTH, 7U);    /* Short_Length         */
    CHECK(TCAS_MODES_LONG_LENGTH,  14U);   /* Long_Length          */

    /* largest bearing: Bearing_Max in tcas-modes_port.ads */
    CHECK(TCAS_MODES_MAX_BEARING, 35999U); /* Bearing_Max          */

    /* replies per cycle: Max_Replies_Per_Cycle in tcas-config.ads */
    CHECK(TCAS_MODES_MAX_REPLIES_PER_CYCLE, 256U);

    /* layout table, for review by eye against the Ada representation
     * clause (rule C4) */
    printf("tcas_modes_frame_t  size %u bytes\n",
           (unsigned int)sizeof(tcas_modes_frame_t));
    printf("  data           offset %2u  size %2u\n",
           (unsigned int)offsetof(tcas_modes_frame_t, data),
           (unsigned int)sizeof(((tcas_modes_frame_t *)0)->data));
    printf("  length         offset %2u  size %2u\n",
           (unsigned int)offsetof(tcas_modes_frame_t, length),
           (unsigned int)sizeof(((tcas_modes_frame_t *)0)->length));
    printf("  status         offset %2u  size %2u\n",
           (unsigned int)offsetof(tcas_modes_frame_t, status),
           (unsigned int)sizeof(((tcas_modes_frame_t *)0)->status));
    printf("  round_trip_ns  offset %2u  size %2u\n",
           (unsigned int)offsetof(tcas_modes_frame_t, round_trip_ns),
           (unsigned int)sizeof(((tcas_modes_frame_t *)0)->round_trip_ns));
    printf("  bearing        offset %2u  size %2u\n",
           (unsigned int)offsetof(tcas_modes_frame_t, bearing),
           (unsigned int)sizeof(((tcas_modes_frame_t *)0)->bearing));
    printf("  valid          offset %2u  size %2u\n",
           (unsigned int)offsetof(tcas_modes_frame_t, valid),
           (unsigned int)sizeof(((tcas_modes_frame_t *)0)->valid));
    printf("  reserved       offset %2u  size %2u\n",
           (unsigned int)offsetof(tcas_modes_frame_t, reserved),
           (unsigned int)sizeof(((tcas_modes_frame_t *)0)->reserved));

    printf("check_modes_layout: %u constants checked, %u failed\n",
           total, failed);

    if (failed != 0U)
    {
        printf("FAIL\n");
        return 1;
    }

    printf("OK\n");
    return 0;
}