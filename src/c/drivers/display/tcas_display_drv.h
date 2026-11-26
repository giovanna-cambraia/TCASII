/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/drivers/display/tcas_display_drv.h
 * Module      : Display and audio output driver, public interface
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   This is the border where the result of the SPARK core leaves for the
 *   cockpit. The Ada formatter (Tcas.Display, partition 4) fills one
 *   Tcas.Display_Port.Display_Frame per cycle. This driver reads it in
 *   place (tcas_display_frame_t has the same layout), checks it AGAIN,
 *   and writes it to the display and audio hardware.
 *
 *   The driver does not trust the formatter. tcas_display_frame_ok repeats
 *   every rule of Tcas.Display_Port.Is_Well_Formed in C, so an inconsistent
 *   advisory cannot reach the cockpit even if the formatter proof had a
 *   gap: a second, independent check at the last step before the pilot.
 *
 *   When there is no frame that can be shown, the caller asks for the FAIL
 *   annunciation (tcas_display_fail): the display shows that TCAS is not
 *   working and the audio is silent. "No valid frame" always ends in an
 *   annunciated failure, never in a stale display.
 *
 * FRAME LAYOUT (must equal Tcas.Display_Port.Display_Frame, 20 bytes)
 *   offset  size  field
 *      0      4   slant_ft         selected intruder, feet, 0 when none
 *      4      4   rel_alt_ft       intruder minus ownship, feet (signed)
 *      8      2   bearing_cdeg     hundredths of a degree, 0 .. 35 999
 *     10      2   target_rate_fpm  RA target rate (signed), 0 when none
 *     12      1   mode             0 standby, 1 TA only, 2 TA and RA
 *     13      1   advisory         0 none, 1 climb, 2 descend
 *     14      1   audio            0 .. 6, see AUDIO CODES
 *     15      1   intruder_class   0 other .. 3 resolution advisory
 *     16      1   intruder_valid   1 an intruder is selected, 0 none
 *     17      1   reserved8        always 0
 *     18      2   reserved16       always 0
 *   [VERIFY] The layout is a PLACEHOLDER until the display and audio
 *   interface control document exists.
 *
 * AUDIO CODES
 *   0 none                    4 "Climb, climb now"      (reversal)
 *   1 "Traffic, traffic"      5 "Descend, descend now"  (reversal)
 *   2 "Climb, climb"          6 "Clear of conflict"
 *   3 "Descend, descend"
 *
 * DESIGN RULES (each one maps to a MISRA or DO-178C argument)
 *   R1  No dynamic memory. All state lives in a structure that the CALLER
 *       owns (MISRA Rule 21.3).
 *   R2  No hidden state. The driver has no static variables; everything it
 *       remembers is in tcas_display_state_t.
 *   R3  Bounded execution. There are no loops at all: the frame check is
 *       one chain of comparisons and the transfer is a fixed list of
 *       register writes. No recursion (MISRA Rule 17.2).
 *   R4  Every pointer argument is checked for NULL and the failure is
 *       reported, never ignored.
 *   R5  FAIL CLOSED. A frame that is not well-formed touches NO hardware.
 *       The same goes for an uninitialized state and for a display that is
 *       not ready. A refusal never leaves a half-written frame behind.
 *   R6  The status register is read exactly ONCE per call, into a local,
 *       and only after the frame has been checked. The data registers are
 *       only written. The commit register is written LAST and exactly
 *       once: the display latches the frame on commit, so it never shows a
 *       frame that was only partly written.
 *   R7  Signed fields are written as plain two's complement with no sign
 *       extension into a wider field: rel_alt_ft as the 32-bit pattern,
 *       target_rate_fpm as the low 16 bits of its register (the upper 16
 *       bits are 0).
 *   R8  The frame check is the same rule set as Display_Port.Is_Well_Formed,
 *       in the same order of ideas: reserved fields, code ranges,
 *       intruder fields, rate against advisory, mode against advisory, and
 *       audio against advisory and intruder.
 *   R9  Counters saturate at UINT32_MAX and never wrap, so a long-running
 *       unit cannot turn "many faults" into "zero faults".
 *
 * ORDER OF CHECKS
 *   tcas_display_show
 *     1 arguments non-NULL                       ERR_NULL
 *     2 state initialized                        ERR_CONFIG
 *     3 the frame is well-formed                 ERR_FRAME
 *     4 the display is ready for a new frame     ERR_BUSY
 *   tcas_display_fail
 *     1 arguments non-NULL                       ERR_NULL
 *     2 state initialized                        ERR_CONFIG
 *     3 the display is ready for a new frame     ERR_BUSY
 *   The first failure wins, so the reported cause is stable. The status
 *   register is read only after check 3 (rule R6).
 *
 * TRANSFER ORDER (the only place that writes the hardware)
 *   show : slant, relative altitude, bearing, rate, flags, audio, then
 *          control = COMMIT
 *   fail : audio = 0 (silence), then control = COMMIT | FAIL; the data
 *          registers are NOT touched, and the display ignores them while
 *          FAIL is set
 *
 * TIMING BUDGET
 *   One show: one register read and seven writes, about 2 microseconds.
 *   It belongs to the last phase of the cycle (900 to 1 000 ms).
 *
 * USAGE (the caller, the cyclic executive, once per cycle)
 *   tcas_display_init(&disp);                                  once
 *   status = tcas_display_show(&disp, disp_hw_ptr, &frame);
 *   if (status != TCAS_DISPLAY_OK) {
 *       status = tcas_display_fail(&disp, disp_hw_ptr);
 *   }
 *   The caller must use every return value (MISRA Rule 17.7).
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * Status codes are uint8_t constants, not enum values, so the layout
 *     shared with Ada is fixed and no signed/unsigned mixing occurs
 *     (Rule 10.4).
 *   * _Static_assert needs C11. Allowed under MISRA C:2012 Amendment 3.
 *   * No deviations are requested by this header.
 * ===========================================================================
*/

#ifndef TCAS_DISPLAY_DRV_H
#define TCAS_DISPLAY_DRV_H

#include <stdint.h>


// compile-time assertion helper 
#define TCAS_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)

// codes (must equal the constants of tcas-display_port.ads)

// mode 
#define TCAS_DISPLAY_MODE_STANDBY   ((uint8_t)0U)
#define TCAS_DISPLAY_MODE_TA_ONLY   ((uint8_t)1U)
#define TCAS_DISPLAY_MODE_TA_RA     ((uint8_t)2U)

// advisory 
#define TCAS_DISPLAY_ADV_NONE       ((uint8_t)0U)
#define TCAS_DISPLAY_ADV_CLIMB      ((uint8_t)1U)
#define TCAS_DISPLAY_ADV_DESCEND    ((uint8_t)2U)

// audio 
#define TCAS_DISPLAY_AUDIO_NONE            ((uint8_t)0U)
#define TCAS_DISPLAY_AUDIO_TRAFFIC         ((uint8_t)1U)
#define TCAS_DISPLAY_AUDIO_CLIMB           ((uint8_t)2U)
#define TCAS_DISPLAY_AUDIO_DESCEND         ((uint8_t)3U)
#define TCAS_DISPLAY_AUDIO_CLIMB_NOW       ((uint8_t)4U)
#define TCAS_DISPLAY_AUDIO_DESCEND_NOW     ((uint8_t)5U)
#define TCAS_DISPLAY_AUDIO_CLEAR           ((uint8_t)6U)

// intruder class (the values of Tcas.Types.Threat_Class) 
#define TCAS_DISPLAY_CLASS_OTHER       ((uint8_t)0U)
#define TCAS_DISPLAY_CLASS_PROXIMATE   ((uint8_t)1U)
#define TCAS_DISPLAY_CLASS_TRAFFIC     ((uint8_t)2U)
#define TCAS_DISPLAY_CLASS_RESOLUTION  ((uint8_t)3U)

// limits (must equal the constants of tcas-display_port.ads)

#define TCAS_DISPLAY_SLANT_MAX     UINT32_C(150000)   /* Range_Ft'Last         */
#define TCAS_DISPLAY_BEARING_MAX   UINT16_C(35999)    /* 359.99 degrees        */
#define TCAS_DISPLAY_ALT_DIFF_MAX  INT32_C(127900)    /* Altitude_Diff_Ft'Last */
#define TCAS_DISPLAY_RATE_MAX      INT16_C(10000)     /* Vertical_Rate_Fpm'Last*/

// result codes

typedef uint8_t tcas_display_status_t;

#define TCAS_DISPLAY_OK           ((tcas_display_status_t)0U)
#define TCAS_DISPLAY_ERR_NULL     ((tcas_display_status_t)1U)  /* NULL argument   */
#define TCAS_DISPLAY_ERR_CONFIG   ((tcas_display_status_t)2U)  /* state not init  */
#define TCAS_DISPLAY_ERR_FRAME    ((tcas_display_status_t)3U)  /* frame not ok    */
#define TCAS_DISPLAY_ERR_BUSY     ((tcas_display_status_t)4U)  /* display not ready*/


/* ---------------------------------------------------------------------------
 * the frame (what the Ada formatter fills; layout is mirrored in
 * Tcas.Display_Port.Display_Frame with Convention C, 20 bytes)
 * ------------------------------------------------------------------------ */

typedef struct
{
    uint32_t slant_ft;          /* selected intruder, feet, 0 when none        */
    int32_t  rel_alt_ft;        /* intruder minus ownship, feet, signed        */
    uint16_t bearing_cdeg;      /* hundredths of a degree, 0 .. 35 999         */
    int16_t  target_rate_fpm;   /* RA target rate, signed, 0 when none         */
    uint8_t  mode;              /* TCAS_DISPLAY_MODE_*                         */
    uint8_t  advisory;          /* TCAS_DISPLAY_ADV_*                          */
    uint8_t  audio;             /* TCAS_DISPLAY_AUDIO_*                        */
    uint8_t  intruder_class;    /* TCAS_DISPLAY_CLASS_*                        */
    uint8_t  intruder_valid;    /* 1U an intruder is selected, 0U none         */
    uint8_t  reserved8;         /* always 0                                    */
    uint16_t reserved16;        /* always 0                                    */
} tcas_display_frame_t;

TCAS_STATIC_ASSERT(sizeof(tcas_display_frame_t) == 20U, "frame must be 20 bytes");


/* ---------------------------------------------------------------------------
 * hardware display and audio unit (volatile registers, 32 bytes)
 * placeholder layout until the display and audio ICD exists
 * [VERIFY] register offsets and bit positions against the ICD
 *   status  : bit 0 READY, the unit has consumed the previous frame
 *   slant   : slant range, feet
 *   rel_alt : relative altitude, 32-bit two's complement
 *   bearing : low 16 bits hold the bearing, hundredths of a degree
 *   rate    : low 16 bits hold the target rate, two's complement; the
 *             upper 16 bits are written as 0
 *   flags   : mode in bits 0 .. 7, advisory in 8 .. 15, intruder class in
 *             16 .. 23, intruder valid in 24 .. 31
 *   audio   : the audio code, 0 .. 6
 *   control : bit 0 COMMIT latches the frame; bit 1 FAIL shows the TCAS
 *             FAIL annunciation and ignores the data registers
 * ------------------------------------------------------------------------ */

#define TCAS_DISPLAY_HW_READY      UINT32_C(0x00000001)

#define TCAS_DISPLAY_CTRL_COMMIT   UINT32_C(0x00000001)
#define TCAS_DISPLAY_CTRL_FAIL     UINT32_C(0x00000002)

#define TCAS_DISPLAY_FLAGS_ADV_SHIFT    (8U)
#define TCAS_DISPLAY_FLAGS_CLASS_SHIFT  (16U)
#define TCAS_DISPLAY_FLAGS_VALID_SHIFT  (24U)

typedef struct
{
    volatile uint32_t status;     /* offset  0, read only      */
    volatile uint32_t slant;      /* offset  4                 */
    volatile uint32_t rel_alt;    /* offset  8                 */
    volatile uint32_t bearing;    /* offset 12                 */
    volatile uint32_t rate;       /* offset 16                 */
    volatile uint32_t flags;      /* offset 20                 */
    volatile uint32_t audio;      /* offset 24                 */
    volatile uint32_t control;    /* offset 28, written LAST   */
} tcas_display_hw_t;

TCAS_STATIC_ASSERT(sizeof(tcas_display_hw_t) == 32U, "display unit must be 32 bytes");


/* ---------------------------------------------------------------------------
 * driver state (owned by the caller, one per display)
 * initialize with tcas_display_init before the first frame; a zero-filled
 * structure is NOT a valid state: use the init function.
 * all counters saturate at UINT32_MAX and never wrap
 * ------------------------------------------------------------------------ */

typedef struct
{
    uint32_t shown_total;      /* frames written and committed                 */
    uint32_t rejected_total;   /* frames refused because they were not ok      */
    uint32_t busy_total;       /* requests refused because the unit was busy   */
    uint32_t fail_total;       /* FAIL annunciations committed                 */
    uint32_t initialized;      /* 1U once tcas_display_init succeeded          */
} tcas_display_state_t;

// interface

/*
 * tcas_display_init
 *   prepares one driver state: every counter zero. Returns
 *   TCAS_DISPLAY_OK or TCAS_DISPLAY_ERR_NULL; on NULL nothing is written
 */
tcas_display_status_t tcas_display_init(tcas_display_state_t * state);

/*
 * tcas_display_frame_ok
 *   returns 1U when the frame obeys every rule of
 *   Tcas.Display_Port.Is_Well_Formed (rule R8), else 0U; a NULL frame
 *   returns 0U. Pure function, no side effects; exposed so the tests can
 *   check it on its own. The rules, in order:
 *     reserved fields are 0; mode, advisory, audio and class are in range;
 *     intruder_valid is 0 or 1; with an intruder, slant, bearing and
 *     relative altitude are inside their limits; without one, slant,
 *     relative altitude, bearing and class are all 0; the rate is inside
 *     its limit and agrees with the advisory (0 for none, positive for a
 *     climb, negative for a descend); an advisory needs the TA and RA
 *     mode; the audio agrees with the advisory and the intruder
 */
uint8_t tcas_display_frame_ok(const tcas_display_frame_t * frame);

/*
 * tcas_display_show
 *   checks the frame, then writes it to the display and audio unit in the
 *   TRANSFER ORDER, with the commit last. Returns the first failing check
 *   of ORDER OF CHECKS or TCAS_DISPLAY_OK. On any error the hardware is
 *   NOT written (rule R5)
 */
tcas_display_status_t tcas_display_show(tcas_display_state_t * state,
                                        tcas_display_hw_t * hw,
                                        const tcas_display_frame_t * frame);

/*
 * tcas_display_fail
 *   asks the unit to show the TCAS FAIL annunciation and to stay silent
 *   (see TRANSFER ORDER). Returns the first failing check of ORDER OF
 *   CHECKS or TCAS_DISPLAY_OK. On any error the hardware is NOT written
 */
tcas_display_status_t tcas_display_fail(tcas_display_state_t * state,
                                        tcas_display_hw_t * hw);

#endif