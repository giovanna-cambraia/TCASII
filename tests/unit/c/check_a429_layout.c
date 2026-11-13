/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : tests/unit/c/check_a429_layout.c
 * Module      : Layout and constant cross-check, C side versus Ada side
 * Language    : C11
 * Assurance   : DO-178C Software Level B (target), verification artifact
 * Checks      : src/c/drivers/arinc429/tcas_a429_drv.h
 *               against src/bindings/tcas-a429_port.ads
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   The C driver leaves a tcas_a429_word_t in memory and the SPARK decoder
 *   reads it as Tcas.A429_Port.Word, in place, without a copy. If the two
 *   descriptions differ by one byte, the decoder reads garbage and
 *   nothing in either language notices. This file is the check that they
 *   agree: sizes, field offsets, field widths, and every constant that has
 *   a twin on the other side.
 *
 * DESIGN RULES OF THE CHECK
 *   C1  The Ada side is written here as LITERALS, copied from the
 *       representation clause and the constants of tcas-a429_port.ads.
 *       The check never reads the Ada file, so the two descriptions stay
 *       independent: they agree only if both are right.
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
 *   After any change to tcas_a429_drv.h or tcas-a429_port.ads, and always
 *   before a release. When it fails, fix the side that is wrong, then
 *   change the literals here only if the Ada side was changed on purpose.
 *
 * USAGE (from the project root)
 *   gcc -std=c11 -Wall -Wextra -Wpedantic \
 *       -I src/c/drivers/arinc429 \
 *       tests/unit/c/check_a429_layout.c -o build/bin/check_a429_layout.exe
 *   build/bin/check_a429_layout.exe
 *
 * NOTES
 *   * Offsets are in bytes from the start of the structure. The Ada
 *     representation clause uses the same unit ("at 4 range 0 .. 31").
 *   * The rules of MISRA C apply to the driver, not to this check, which
 *     uses <stdio.h> for the report.
 * ===========================================================================
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "tcas_a429_drv.h"

// layout, checked at compile time (rule C2)
// each number is copied from the representation clause of Tcas.A429_Port.Word

TCAS_STATIC_ASSERT(sizeof(tcas_a429_word_t) == 16U,
                   "size: Ada says Word'Size = 128 bits");

TCAS_STATIC_ASSERT(offsetof(tcas_a429_word_t, data) == 0U,
                   "Data at 0");
TCAS_STATIC_ASSERT(offsetof(tcas_a429_word_t, frame) == 4U,
                   "Frame at 4");
TCAS_STATIC_ASSERT(offsetof(tcas_a429_word_t, label) == 8U,
                   "Label at 8");
TCAS_STATIC_ASSERT(offsetof(tcas_a429_word_t, sdi) == 9U,
                   "Sdi at 9");
TCAS_STATIC_ASSERT(offsetof(tcas_a429_word_t, ssm) == 10U,
                   "Ssm at 10");
TCAS_STATIC_ASSERT(offsetof(tcas_a429_word_t, status) == 11U,
                   "Status at 11");
TCAS_STATIC_ASSERT(offsetof(tcas_a429_word_t, valid) == 12U,
                   "Valid at 12");
TCAS_STATIC_ASSERT(offsetof(tcas_a429_word_t, reserved8) == 13U,
                   "Reserved8 at 13");
TCAS_STATIC_ASSERT(offsetof(tcas_a429_word_t, reserved16) == 14U,
                   "Reserved16 at 14");

// field widths: Ada U32 = 4 bytes, U8 = 1 byte, U16 = 2 bytes 
TCAS_STATIC_ASSERT(sizeof(((tcas_a429_word_t *)0)->data) == 4U,
                   "Data is 32 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_a429_word_t *)0)->frame) == 4U,
                   "Frame is 32 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_a429_word_t *)0)->label) == 1U,
                   "Label is 8 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_a429_word_t *)0)->sdi) == 1U,
                   "Sdi is 8 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_a429_word_t *)0)->ssm) == 1U,
                   "Ssm is 8 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_a429_word_t *)0)->status) == 1U,
                   "Status is 8 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_a429_word_t *)0)->valid) == 1U,
                   "Valid is 8 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_a429_word_t *)0)->reserved8) == 1U,
                   "Reserved8 is 8 bits");
TCAS_STATIC_ASSERT(sizeof(((tcas_a429_word_t *)0)->reserved16) == 2U,
                   "Reserved16 is 16 bits");

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
    // status codes: Status_/ in tcas-a429_port.ads 
    CHECK(TCAS_A429_OK,          0U);   // Status_OK          
    CHECK(TCAS_A429_ERR_NULL,    1U);   // Status_Err_Null    
    CHECK(TCAS_A429_ERR_CONFIG,  2U);   // Status_Err_Config  
    CHECK(TCAS_A429_ERR_PARITY,  3U);   // Status_Err_Parity  
    CHECK(TCAS_A429_ERR_LABEL,   4U);   // Status_Err_Label   
    CHECK(TCAS_A429_ERR_SSM_FW,  5U);   // Status_Err_Ssm_Fw  
    CHECK(TCAS_A429_ERR_SSM_NCD, 6U);   // Status_Err_Ssm_Ncd 
    CHECK(TCAS_A429_ERR_SSM_FT,  7U);   // Status_Err_Ssm_Ft  
    CHECK(TCAS_A429_ERR_OVERRUN, 8U);   // Status_Err_Overrun 
    CHECK(TCAS_A429_ERR_BACKLOG, 9U);   // Status_Err_Backlog 

    // sign/status matrix: Ssm_* in tcas-a429_port.ads 
    CHECK(TCAS_A429_SSM_FAILURE_WARNING,  0U);   // Ssm_Failure_Warning  
    CHECK(TCAS_A429_SSM_NO_COMPUTED_DATA, 1U);   // Ssm_No_Computed_Data 
    CHECK(TCAS_A429_SSM_FUNCTIONAL_TEST,  2U);   // Ssm_Functional_Test  
    CHECK(TCAS_A429_SSM_NORMAL,           3U);   // Ssm_Normal           

    // labels: Label_* in tcas-a429_port.ads (16#83#, 16#74#, 16#8A#) 
    CHECK(TCAS_A429_LABEL_PRESSURE_ALT, 0x83U);  // Label_Pressure_Alt 
    CHECK(TCAS_A429_LABEL_RADIO_ALT,    0x74U);  // Label_Radio_Alt    
    CHECK(TCAS_A429_LABEL_ALT_RATE,     0x8AU);  // Label_Alt_Rate     

    // the 19-bit data field: Data_Max = 16#7_FFFF# in tcas-a429_port.ads; the C side has it as the data mask shifted down 
    CHECK(TCAS_A429_DATA_MASK >> TCAS_A429_DATA_SHIFT, 0x7FFFFU);

    // layout table, for review by eye against the Ada representation clause (rule C4) 
    printf("tcas_a429_word_t  size %u bytes\n",
           (unsigned int)sizeof(tcas_a429_word_t));
    printf("  data       offset %2u  size %u\n",
           (unsigned int)offsetof(tcas_a429_word_t, data),
           (unsigned int)sizeof(((tcas_a429_word_t *)0)->data));
    printf("  frame      offset %2u  size %u\n",
           (unsigned int)offsetof(tcas_a429_word_t, frame),
           (unsigned int)sizeof(((tcas_a429_word_t *)0)->frame));
    printf("  label      offset %2u  size %u\n",
           (unsigned int)offsetof(tcas_a429_word_t, label),
           (unsigned int)sizeof(((tcas_a429_word_t *)0)->label));
    printf("  sdi        offset %2u  size %u\n",
           (unsigned int)offsetof(tcas_a429_word_t, sdi),
           (unsigned int)sizeof(((tcas_a429_word_t *)0)->sdi));
    printf("  ssm        offset %2u  size %u\n",
           (unsigned int)offsetof(tcas_a429_word_t, ssm),
           (unsigned int)sizeof(((tcas_a429_word_t *)0)->ssm));
    printf("  status     offset %2u  size %u\n",
           (unsigned int)offsetof(tcas_a429_word_t, status),
           (unsigned int)sizeof(((tcas_a429_word_t *)0)->status));
    printf("  valid      offset %2u  size %u\n",
           (unsigned int)offsetof(tcas_a429_word_t, valid),
           (unsigned int)sizeof(((tcas_a429_word_t *)0)->valid));
    printf("  reserved8  offset %2u  size %u\n",
           (unsigned int)offsetof(tcas_a429_word_t, reserved8),
           (unsigned int)sizeof(((tcas_a429_word_t *)0)->reserved8));
    printf("  reserved16 offset %2u  size %u\n",
           (unsigned int)offsetof(tcas_a429_word_t, reserved16),
           (unsigned int)sizeof(((tcas_a429_word_t *)0)->reserved16));

    printf("check_a429_layout: %u constants checked, %u failed\n",
           total, failed);

    if (failed != 0U)
    {
        printf("FAIL\n");
        return 1;
    }

    printf("OK\n");
    return 0;
}