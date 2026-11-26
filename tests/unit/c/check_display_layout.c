/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : tests/unit/c/check_display_layout.c
 * Module      : Layout and constant cross-check, C side versus Ada side
 * Language    : C11
 * Assurance   : DO-178C Software Level B (target), verification artifact
 * Checks      : src/c/drivers/display/tcas_display_drv.h
 *               against src/bindings/tcas-display_port.ads
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   The Ada formatter fills a Tcas.Display_Port.Display_Frame and the C
 *   driver reads it in place as a tcas_display_frame_t, without a copy. If
 *   the two descriptions differ by one byte, or if one signed field becomes
 *   unsigned on one side, the cockpit gets wrong numbers and nothing in
 *   either language notices. This file is the check that they agree: size,
 *   field offsets, field widths, the SIGN of every field, and every
 *   constant that has a twin on the other side.
 *
 *   The hardware display unit (tcas_display_hw_t) never crosses into Ada,
 *   so it is not checked here. Its layout is a placeholder until the
 *   display and audio interface control document exists.
 *
 * DESIGN RULES OF THE CHECK
 *   C1  The Ada side is written here as LITERALS, copied from the
 *       representation clause and the constants of tcas-display_port.ads.
 *       The check never reads the Ada file, so the two descriptions stay
 *       independent: they agree only if both are right.
 *   C2  Layout and types are checked at COMPILE time (_Static_assert). A
 *       mismatch stops the build, so it cannot be skipped or ignored.
 *   C3  Constants are checked at RUN time, one line per constant, so a
 *       failure names the constant that differs.
 *   C4  The layout table is printed as well, so that a person can read it
 *       and compare it with the Ada file by eye (the evidence for review).
 *   C5  Exit status is 1 if anything differs, else 0, so a script can use
 *       it.
 *   C6  The sign of a field is part of the contract. The Ada record uses
 *       Integer_32 (I32) and Integer_16 (I16) for the two signed fields
 *       and Unsigned_32 / Unsigned_16 / Unsigned_8 for the others; the C
 *       field types must be the same, and _Generic checks them.
 *
 * ADA SIDE USED (tcas-display_port.ads)
 *   Display_Frame, Convention C, 160 bits = 20 bytes:
 *     Slant_Ft        U32 at  0     Intruder_Class  U8  at 15
 *     Rel_Alt_Ft      I32 at  4     Intruder_Valid  U8  at 16
 *     Bearing_Cdeg    U16 at  8     Reserved8       U8  at 17
 *     Target_Rate_Fpm I16 at 10     Reserved16      U16 at 18
 *     Mode            U8  at 12     Advisory        U8  at 13
 *     Audio           U8  at 14
 *   Mode: 0 standby, 1 TA only, 2 TA and RA. Advisory: 0 none, 1 climb,
 *   2 descend. Audio: 0 none, 1 Traffic, 2 Climb, 3 Descend, 4 Climb now,
 *   5 Descend now, 6 Clear of conflict. Class: 0 other, 1 proximate,
 *   2 traffic, 3 resolution. Limits: Slant_Max 150 000, Bearing_Max
 *   35 999, Alt_Diff_Max 127 900, Rate_Max 10 000.
 *
 * WHEN TO RUN IT
 *   After any change to tcas_display_drv.h or tcas-display_port.ads, and
 *   always before a release. When it fails, fix the side that is wrong,
 *   then change the literals here only if the Ada side was changed on
 *   purpose.
 *
 * USAGE (from the project root)
 *   gcc -std=c11 -Wall -Wextra -Wpedantic \
 *       -I src/c/drivers/display \
 *       tests/unit/c/check_display_layout.c \
 *       -o build/bin/check_display_layout.exe
 *   build/bin/check_display_layout.exe
 *
 * NOTES
 *   * Offsets are in bytes from the start of the structure. The Ada
 *     representation clause uses the same unit ("Rel_Alt_Ft at 4 range
 *     0 .. 31").
 *   * The rules of MISRA C apply to the driver, not to this check, which
 *     uses <stdio.h> for the report.
*/

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "tcas_display_drv.h"


// the type of a member, as a number, for _Generic (rule C6) 
#define MEMBER(name) (((tcas_display_frame_t *)0)->name)

#define IS_U32(x) _Generic((x), uint32_t: 1, default: 0)
#define IS_I32(x) _Generic((x), int32_t:  1, default: 0)
#define IS_U16(x) _Generic((x), uint16_t: 1, default: 0)
#define IS_I16(x) _Generic((x), int16_t:  1, default: 0)
#define IS_U8(x)  _Generic((x), uint8_t:  1, default: 0)


/*
 * layout, checked at compile time (rule C2)
 * each number is copied from the representation clause of
 * Tcas.Display_Port.Display_Frame
*/

TCAS_STATIC_ASSERT(sizeof(tcas_display_frame_t) == 20U,
                   "size: Ada says Display_Frame'Size = 160 bits");

TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, slant_ft) == 0U,
                   "Slant_Ft at 0");
TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, rel_alt_ft) == 4U,
                   "Rel_Alt_Ft at 4");
TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, bearing_cdeg) == 8U,
                   "Bearing_Cdeg at 8");
TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, target_rate_fpm) == 10U,
                   "Target_Rate_Fpm at 10");
TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, mode) == 12U,
                   "Mode at 12");
TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, advisory) == 13U,
                   "Advisory at 13");
TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, audio) == 14U,
                   "Audio at 14");
TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, intruder_class) == 15U,
                   "Intruder_Class at 15");
TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, intruder_valid) == 16U,
                   "Intruder_Valid at 16");
TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, reserved8) == 17U,
                   "Reserved8 at 17");
TCAS_STATIC_ASSERT(offsetof(tcas_display_frame_t, reserved16) == 18U,
                   "Reserved16 at 18");

// field widths: Ada U32 / I32 = 4 bytes, U16 / I16 = 2 bytes, U8 = 1 byte 
TCAS_STATIC_ASSERT(sizeof(MEMBER(slant_ft)) == 4U,        "Slant_Ft is 32 bits");
TCAS_STATIC_ASSERT(sizeof(MEMBER(rel_alt_ft)) == 4U,      "Rel_Alt_Ft is 32 bits");
TCAS_STATIC_ASSERT(sizeof(MEMBER(bearing_cdeg)) == 2U,    "Bearing_Cdeg is 16 bits");
TCAS_STATIC_ASSERT(sizeof(MEMBER(target_rate_fpm)) == 2U, "Target_Rate_Fpm is 16 bits");
TCAS_STATIC_ASSERT(sizeof(MEMBER(mode)) == 1U,            "Mode is 8 bits");
TCAS_STATIC_ASSERT(sizeof(MEMBER(advisory)) == 1U,        "Advisory is 8 bits");
TCAS_STATIC_ASSERT(sizeof(MEMBER(audio)) == 1U,           "Audio is 8 bits");
TCAS_STATIC_ASSERT(sizeof(MEMBER(intruder_class)) == 1U,  "Intruder_Class is 8 bits");
TCAS_STATIC_ASSERT(sizeof(MEMBER(intruder_valid)) == 1U,  "Intruder_Valid is 8 bits");
TCAS_STATIC_ASSERT(sizeof(MEMBER(reserved8)) == 1U,       "Reserved8 is 8 bits");
TCAS_STATIC_ASSERT(sizeof(MEMBER(reserved16)) == 2U,      "Reserved16 is 16 bits");

// signedness (rule C6): the two signed fields are I32 and I16 in Ada, the rest are unsigned. A field that changed sign would keep its size and its
// offset, so only the type check can catch it 
TCAS_STATIC_ASSERT(IS_U32(MEMBER(slant_ft)) == 1,
                   "Slant_Ft is unsigned 32 bits (U32)");
TCAS_STATIC_ASSERT(IS_I32(MEMBER(rel_alt_ft)) == 1,
                   "Rel_Alt_Ft is SIGNED 32 bits (I32)");
TCAS_STATIC_ASSERT(IS_U16(MEMBER(bearing_cdeg)) == 1,
                   "Bearing_Cdeg is unsigned 16 bits (U16)");
TCAS_STATIC_ASSERT(IS_I16(MEMBER(target_rate_fpm)) == 1,
                   "Target_Rate_Fpm is SIGNED 16 bits (I16)");
TCAS_STATIC_ASSERT(IS_U8(MEMBER(mode)) == 1,           "Mode is U8");
TCAS_STATIC_ASSERT(IS_U8(MEMBER(advisory)) == 1,       "Advisory is U8");
TCAS_STATIC_ASSERT(IS_U8(MEMBER(audio)) == 1,          "Audio is U8");
TCAS_STATIC_ASSERT(IS_U8(MEMBER(intruder_class)) == 1, "Intruder_Class is U8");
TCAS_STATIC_ASSERT(IS_U8(MEMBER(intruder_valid)) == 1, "Intruder_Valid is U8");
TCAS_STATIC_ASSERT(IS_U8(MEMBER(reserved8)) == 1,      "Reserved8 is U8");
TCAS_STATIC_ASSERT(IS_U16(MEMBER(reserved16)) == 1,    "Reserved16 is U16");

// constants, checked at run time (rule C3)

static unsigned int failed = 0U;
static unsigned int total  = 0U;

// compares one C constant with its Ada twin and reports a difference; the values are passed as long so that the signed limits print correctly 
static void check_const(const char * name, long c_value, long ada_value)
{
    total++;

    if (c_value != ada_value)
    {
        failed++;
        printf("FAIL: %s  C = %ld, Ada = %ld\n", name, c_value, ada_value);
    }
}

#define CHECK(c_macro, ada_literal) \
    check_const(#c_macro, (long)(c_macro), (long)(ada_literal))

int main(void)
{
    /* mode: Mode_* in tcas-display_port.ads */
    CHECK(TCAS_DISPLAY_MODE_STANDBY, 0);    /* Mode_Standby */
    CHECK(TCAS_DISPLAY_MODE_TA_ONLY, 1);    /* Mode_TA_Only */
    CHECK(TCAS_DISPLAY_MODE_TA_RA,   2);    /* Mode_TA_RA   */

    /* advisory: Advisory_* */
    CHECK(TCAS_DISPLAY_ADV_NONE,    0);     /* Advisory_None    */
    CHECK(TCAS_DISPLAY_ADV_CLIMB,   1);     /* Advisory_Climb   */
    CHECK(TCAS_DISPLAY_ADV_DESCEND, 2);     /* Advisory_Descend */

    /* audio: Audio_* */
    CHECK(TCAS_DISPLAY_AUDIO_NONE,        0);   /* Audio_None              */
    CHECK(TCAS_DISPLAY_AUDIO_TRAFFIC,     1);   /* Audio_Traffic           */
    CHECK(TCAS_DISPLAY_AUDIO_CLIMB,       2);   /* Audio_Climb             */
    CHECK(TCAS_DISPLAY_AUDIO_DESCEND,     3);   /* Audio_Descend           */
    CHECK(TCAS_DISPLAY_AUDIO_CLIMB_NOW,   4);   /* Audio_Climb_Now         */
    CHECK(TCAS_DISPLAY_AUDIO_DESCEND_NOW, 5);   /* Audio_Descend_Now       */
    CHECK(TCAS_DISPLAY_AUDIO_CLEAR,       6);   /* Audio_Clear_Of_Conflict */

    /* intruder class: Class_* (the values of Tcas.Types.Threat_Class) */
    CHECK(TCAS_DISPLAY_CLASS_OTHER,      0);    /* Class_Other      */
    CHECK(TCAS_DISPLAY_CLASS_PROXIMATE,  1);    /* Class_Proximate  */
    CHECK(TCAS_DISPLAY_CLASS_TRAFFIC,    2);    /* Class_Traffic    */
    CHECK(TCAS_DISPLAY_CLASS_RESOLUTION, 3);    /* Class_Resolution */

    /* limits: Slant_Max, Bearing_Max, Alt_Diff_Max, Rate_Max */
    CHECK(TCAS_DISPLAY_SLANT_MAX,    150000);   /* Slant_Max    */
    CHECK(TCAS_DISPLAY_BEARING_MAX,  35999);    /* Bearing_Max  */
    CHECK(TCAS_DISPLAY_ALT_DIFF_MAX, 127900);   /* Alt_Diff_Max */
    CHECK(TCAS_DISPLAY_RATE_MAX,     10000);    /* Rate_Max     */

    /* layout table, for review by eye against the Ada representation
     * clause (rule C4) */
    printf("tcas_display_frame_t  size %u bytes\n",
           (unsigned int)sizeof(tcas_display_frame_t));
    printf("  slant_ft         offset %2u  size %u  unsigned\n",
           (unsigned int)offsetof(tcas_display_frame_t, slant_ft),
           (unsigned int)sizeof(MEMBER(slant_ft)));
    printf("  rel_alt_ft       offset %2u  size %u  SIGNED\n",
           (unsigned int)offsetof(tcas_display_frame_t, rel_alt_ft),
           (unsigned int)sizeof(MEMBER(rel_alt_ft)));
    printf("  bearing_cdeg     offset %2u  size %u  unsigned\n",
           (unsigned int)offsetof(tcas_display_frame_t, bearing_cdeg),
           (unsigned int)sizeof(MEMBER(bearing_cdeg)));
    printf("  target_rate_fpm  offset %2u  size %u  SIGNED\n",
           (unsigned int)offsetof(tcas_display_frame_t, target_rate_fpm),
           (unsigned int)sizeof(MEMBER(target_rate_fpm)));
    printf("  mode             offset %2u  size %u  unsigned\n",
           (unsigned int)offsetof(tcas_display_frame_t, mode),
           (unsigned int)sizeof(MEMBER(mode)));
    printf("  advisory         offset %2u  size %u  unsigned\n",
           (unsigned int)offsetof(tcas_display_frame_t, advisory),
           (unsigned int)sizeof(MEMBER(advisory)));
    printf("  audio            offset %2u  size %u  unsigned\n",
           (unsigned int)offsetof(tcas_display_frame_t, audio),
           (unsigned int)sizeof(MEMBER(audio)));
    printf("  intruder_class   offset %2u  size %u  unsigned\n",
           (unsigned int)offsetof(tcas_display_frame_t, intruder_class),
           (unsigned int)sizeof(MEMBER(intruder_class)));
    printf("  intruder_valid   offset %2u  size %u  unsigned\n",
           (unsigned int)offsetof(tcas_display_frame_t, intruder_valid),
           (unsigned int)sizeof(MEMBER(intruder_valid)));
    printf("  reserved8        offset %2u  size %u  unsigned\n",
           (unsigned int)offsetof(tcas_display_frame_t, reserved8),
           (unsigned int)sizeof(MEMBER(reserved8)));
    printf("  reserved16       offset %2u  size %u  unsigned\n",
           (unsigned int)offsetof(tcas_display_frame_t, reserved16),
           (unsigned int)sizeof(MEMBER(reserved16)));

    printf("check_display_layout: %u constants checked, %u failed\n",
           total, failed);

    if (failed != 0U)
    {
        printf("FAIL\n");
        return 1;
    }

    printf("OK\n");
    return 0;
}