/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : tests/unit/c/check_timer_consts.c
 * Module      : Constant cross-check, C clock and watchdog versus Ada config
 * Language    : C11
 * Assurance   : DO-178C Software Level B (target), verification artifact
 * Checks      : src/c/hal/timer/tcas_timer.h
 *               src/c/hal/watchdog/tcas_wdg.h
 *               against src/ada/common/config/tcas-config.ads
 *               and src/ada/common/types/tcas-types.ads
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   The schedule exists in two languages. The Ada side
 *   (Tcas.Config.Window_*, Tcas.Types.Cycle_Ms, Tcas.Types.Time_Ms) says
 *   how the 1 000 ms cycle is divided into five windows. The C side (the
 *   deadlines in tcas_timer.h, the phase numbers in tcas_timer.h and
 *   tcas_wdg.h) says when each phase must be over and which phase is which.
 *   If the two differ, the cyclic executive supervises a schedule that the
 *   core does not follow, and nothing in either language notices. This file
 *   is the check that they agree.
 *
 *   It also checks the two HAL modules against each other: the clock and
 *   the watchdog must number the phases the same way, because the cyclic
 *   executive passes the same phase to both.
 *
 * DESIGN RULES OF THE CHECK
 *   C1  The Ada side is written here as LITERALS, copied from
 *       tcas-config.ads and tcas-types.ads. The check never reads the Ada
 *       files, so the two descriptions stay independent: they agree only
 *       if both are right.
 *   C2  Relations are checked at COMPILE time (_Static_assert), so a
 *       mismatch stops the build and cannot be skipped or ignored. The
 *       same values are checked again at RUN time, one line per constant,
 *       so a failure names the constant that differs.
 *   C3  The window lengths are checked as DIFFERENCES of consecutive
 *       deadlines. Together with the deadlines themselves, they pin down
 *       every window, not only the sum.
 *   C4  The schedule is printed as a table, so that a person can read it
 *       and compare it with the Ada file by eye (the evidence for review).
 *   C5  Exit status is 1 if anything differs, else 0, so a script can use
 *       it.
 *
 * ADA LITERALS USED (all in milliseconds unless stated)
 *   Tcas.Config.Window_Ownship_Ms      = 150    ( 0 ..  150)
 *   Tcas.Config.Window_Interrogate_Ms  = 300    (150 ..  450)
 *   Tcas.Config.Window_Threat_Ms       = 300    (450 ..  750)
 *   Tcas.Config.Window_Advisory_Ms     = 150    (750 ..  900)
 *   Tcas.Config.Window_Output_Ms       = 100    (900 .. 1000)
 *   Tcas.Types.Cycle_Ms                = 1 000
 *   Tcas.Types.Time_Ms'Last            = 3 600 000
 *   The windows add up to Cycle_Ms (Tcas.Config checks it with a compile
 *   time error). One tick of the clock is one microsecond, so the
 *   deadlines in microseconds are the cumulative windows times 1 000.
 *
 * WHEN TO RUN IT
 *   After any change to tcas_timer.h, tcas_wdg.h, or the schedule and time
 *   range in tcas-config.ads and tcas-types.ads, and always before a
 *   release. When it fails, fix the side that is wrong, then change the
 *   literals here only if the Ada side was changed on purpose.
 *
 * USAGE (from the project root)
 *   gcc -std=c11 -Wall -Wextra -Wpedantic \
 *       -I src/c/hal/timer -I src/c/hal/watchdog \
 *       tests/unit/c/check_timer_consts.c -o build/bin/check_timer_consts.exe
 *   build/bin/check_timer_consts.exe
 *
 * NOTES
 *   * The jitter tolerance (TCAS_TIMER_JITTER_US) has no Ada twin: only the
 *     C side uses it. It is printed, not compared.
 *   * This is the first file that includes two HAL headers. Both define
 *     TCAS_STATIC_ASSERT with identical text, which C allows. If a compiler
 *     or checker objects, move the macro into a shared tcas_common.h.
 *   * The rules of MISRA C apply to the HAL, not to this check, which uses
 *     <stdio.h> for the report.
*/

#include <stdint.h>
#include <stdio.h>

#include "tcas_timer.h"
#include "tcas_wdg.h"


/* 
 * the schedule, checked at compile time (rule C2)
 * each number is copied from tcas-config.ads and tcas-types.ads
*/

// the five deadlines are the cumulative windows, in microseconds 
TCAS_STATIC_ASSERT(TCAS_TIMER_DEADLINE_0_US == UINT32_C(150000),
                   "deadline 0: Window_Ownship_Ms = 150");
TCAS_STATIC_ASSERT(TCAS_TIMER_DEADLINE_1_US == UINT32_C(450000),
                   "deadline 1: 150 + Window_Interrogate_Ms 300");
TCAS_STATIC_ASSERT(TCAS_TIMER_DEADLINE_2_US == UINT32_C(750000),
                   "deadline 2: 450 + Window_Threat_Ms 300");
TCAS_STATIC_ASSERT(TCAS_TIMER_DEADLINE_3_US == UINT32_C(900000),
                   "deadline 3: 750 + Window_Advisory_Ms 150");
TCAS_STATIC_ASSERT(TCAS_TIMER_DEADLINE_4_US == UINT32_C(1000000),
                   "deadline 4: 900 + Window_Output_Ms 100");

// the period is Cycle_Ms, in microseconds 
TCAS_STATIC_ASSERT(TCAS_TIMER_PERIOD_US == UINT32_C(1000) * UINT32_C(1000),
                   "period: Cycle_Ms = 1000");

// the earliest kick is the start of the last window: Cycle_Ms minus Window_Output_Ms = 1000 - 100 = 900 ms
TCAS_STATIC_ASSERT(TCAS_TIMER_KICK_EARLIEST_US ==
                       (UINT32_C(1000) - UINT32_C(100)) * UINT32_C(1000),
                   "earliest kick: Cycle_Ms - Window_Output_Ms = 900");

// the elapsed time given to the scheduler fits Time_Ms: 0 .. 3 600 000 
TCAS_STATIC_ASSERT(TCAS_TIMER_MAX_ELAPSED_MS == UINT32_C(3600000),
                   "Time_Ms'Last = 3600000");

// one tick is one microsecond, so 1 000 ticks make one millisecond 
TCAS_STATIC_ASSERT(TCAS_TIMER_TICKS_PER_MS == UINT32_C(1000),
                   "1 tick = 1 us, 1000 ticks per ms");

// five windows, five phases, in both modules 
TCAS_STATIC_ASSERT(TCAS_TIMER_PHASE_COUNT == 5U,
                   "five windows in Tcas.Config, five phases in the timer");
TCAS_STATIC_ASSERT(TCAS_WDG_PHASE_COUNT == TCAS_TIMER_PHASE_COUNT,
                   "the watchdog and the timer count the same phases");

// the phase numbers are the same in the two HAL modules 
TCAS_STATIC_ASSERT(TCAS_WDG_PHASE_OWNSHIP == TCAS_TIMER_PHASE_OWNSHIP,
                   "phase 0: ownship");
TCAS_STATIC_ASSERT(TCAS_WDG_PHASE_INTERROGATE == TCAS_TIMER_PHASE_INTERROGATE,
                   "phase 1: interrogate");
TCAS_STATIC_ASSERT(TCAS_WDG_PHASE_THREAT == TCAS_TIMER_PHASE_THREAT,
                   "phase 2: threat");
TCAS_STATIC_ASSERT(TCAS_WDG_PHASE_ADVISORY == TCAS_TIMER_PHASE_ADVISORY,
                   "phase 3: advisory");
TCAS_STATIC_ASSERT(TCAS_WDG_PHASE_OUTPUT == TCAS_TIMER_PHASE_OUTPUT,
                   "phase 4: output");

// the same values, checked at run time (rule C2), one line per constant

static unsigned int failed = 0U;
static unsigned int total  = 0U;

// compares one C value with its expected value and reports a difference 
static void check_const(const char * name,
                        unsigned int c_value,
                        unsigned int expected)
{
    total++;

    if (c_value != expected)
    {
        failed++;
        printf("FAIL: %s  C = %u, expected = %u\n", name, c_value, expected);
    }
}

// a C macro against a literal copied from the Ada side 
#define CHECK(c_macro, ada_literal) \
    check_const(#c_macro, (unsigned int)(c_macro), (unsigned int)(ada_literal))

// two C macros against each other (timer against watchdog) 
#define PAIR(first, second) \
    check_const(#first " vs " #second, (unsigned int)(first), \
                (unsigned int)(second))

int main(void)
{
    /* phase numbers: 0 .. 4, in the order of the windows */
    CHECK(TCAS_TIMER_PHASE_OWNSHIP,     0U);
    CHECK(TCAS_TIMER_PHASE_INTERROGATE, 1U);
    CHECK(TCAS_TIMER_PHASE_THREAT,      2U);
    CHECK(TCAS_TIMER_PHASE_ADVISORY,    3U);
    CHECK(TCAS_TIMER_PHASE_OUTPUT,      4U);

    /* the watchdog numbers the phases the same way */
    PAIR(TCAS_WDG_PHASE_OWNSHIP,     TCAS_TIMER_PHASE_OWNSHIP);
    PAIR(TCAS_WDG_PHASE_INTERROGATE, TCAS_TIMER_PHASE_INTERROGATE);
    PAIR(TCAS_WDG_PHASE_THREAT,      TCAS_TIMER_PHASE_THREAT);
    PAIR(TCAS_WDG_PHASE_ADVISORY,    TCAS_TIMER_PHASE_ADVISORY);
    PAIR(TCAS_WDG_PHASE_OUTPUT,      TCAS_TIMER_PHASE_OUTPUT);

    /* the number of phases: five windows in Tcas.Config */
    CHECK(TCAS_TIMER_PHASE_COUNT, 5U);
    PAIR(TCAS_WDG_PHASE_COUNT, TCAS_TIMER_PHASE_COUNT);

    /* the watchdog mask has exactly one bit per phase: 2 ** 5 - 1 */
    CHECK(TCAS_WDG_ALL_DONE, 0x1FU);

    /* deadlines: cumulative windows, in microseconds */
    CHECK(TCAS_TIMER_DEADLINE_0_US,  150000U);
    CHECK(TCAS_TIMER_DEADLINE_1_US,  450000U);
    CHECK(TCAS_TIMER_DEADLINE_2_US,  750000U);
    CHECK(TCAS_TIMER_DEADLINE_3_US,  900000U);
    CHECK(TCAS_TIMER_DEADLINE_4_US, 1000000U);

    /* window lengths: Window_*_Ms in tcas-config.ads, in microseconds
     * (rule C3) */
    check_const("window 0 (Window_Ownship_Ms = 150)",
                (unsigned int)TCAS_TIMER_DEADLINE_0_US, 150000U);
    check_const("window 1 (Window_Interrogate_Ms = 300)",
                (unsigned int)(TCAS_TIMER_DEADLINE_1_US
                               - TCAS_TIMER_DEADLINE_0_US), 300000U);
    check_const("window 2 (Window_Threat_Ms = 300)",
                (unsigned int)(TCAS_TIMER_DEADLINE_2_US
                               - TCAS_TIMER_DEADLINE_1_US), 300000U);
    check_const("window 3 (Window_Advisory_Ms = 150)",
                (unsigned int)(TCAS_TIMER_DEADLINE_3_US
                               - TCAS_TIMER_DEADLINE_2_US), 150000U);
    check_const("window 4 (Window_Output_Ms = 100)",
                (unsigned int)(TCAS_TIMER_DEADLINE_4_US
                               - TCAS_TIMER_DEADLINE_3_US), 100000U);

    /* period, earliest kick, elapsed limit, tick rate */
    CHECK(TCAS_TIMER_PERIOD_US,         1000000U);
    CHECK(TCAS_TIMER_KICK_EARLIEST_US,   900000U);
    CHECK(TCAS_TIMER_MAX_ELAPSED_MS,    3600000U);
    CHECK(TCAS_TIMER_TICKS_PER_MS,         1000U);

    /* the schedule as a table, for review by eye against tcas-config.ads
     * (rule C4) */
    printf("cycle schedule, microseconds after the cycle began\n");
    printf("  phase 0 ownship      ends %7u  window %7u\n",
           (unsigned int)TCAS_TIMER_DEADLINE_0_US,
           (unsigned int)TCAS_TIMER_DEADLINE_0_US);
    printf("  phase 1 interrogate  ends %7u  window %7u\n",
           (unsigned int)TCAS_TIMER_DEADLINE_1_US,
           (unsigned int)(TCAS_TIMER_DEADLINE_1_US - TCAS_TIMER_DEADLINE_0_US));
    printf("  phase 2 threat       ends %7u  window %7u\n",
           (unsigned int)TCAS_TIMER_DEADLINE_2_US,
           (unsigned int)(TCAS_TIMER_DEADLINE_2_US - TCAS_TIMER_DEADLINE_1_US));
    printf("  phase 3 advisory     ends %7u  window %7u\n",
           (unsigned int)TCAS_TIMER_DEADLINE_3_US,
           (unsigned int)(TCAS_TIMER_DEADLINE_3_US - TCAS_TIMER_DEADLINE_2_US));
    printf("  phase 4 output       ends %7u  window %7u\n",
           (unsigned int)TCAS_TIMER_DEADLINE_4_US,
           (unsigned int)(TCAS_TIMER_DEADLINE_4_US - TCAS_TIMER_DEADLINE_3_US));
    printf("  period %u, earliest kick %u, jitter tolerance %u (C only)\n",
           (unsigned int)TCAS_TIMER_PERIOD_US,
           (unsigned int)TCAS_TIMER_KICK_EARLIEST_US,
           (unsigned int)TCAS_TIMER_JITTER_US);
    printf("  elapsed limit %u ms (Time_Ms'Last)\n",
           (unsigned int)TCAS_TIMER_MAX_ELAPSED_MS);

    printf("check_timer_consts: %u constants checked, %u failed\n",
           total, failed);

    if (failed != 0U)
    {
        printf("FAIL\n");
        return 1;
    }

    printf("OK\n");
    return 0;
}