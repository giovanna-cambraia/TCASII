/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/drivers/modes_s/tcas_modes_drv.h
 * Module      : Mode S reply receive driver, public interface
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   This is the border where raw Mode S replies from the transceiver first
 *   meet the typed world of the SPARK core. The transceiver leaves one
 *   reply at a time in a hardware mailbox: the reply bytes, the round-trip
 *   time and the bearing that it measured. This driver takes the reply,
 *   checks it, copies it into a tcas_modes_frame_t and acknowledges the
 *   mailbox. The SPARK parser (Tcas.Modes_Parser, partition 2) then reads
 *   the frame in place, through Tcas.Modes_Port.Frame, which has the same
 *   layout. Nothing reaches the parser without passing every check below.
 *
 *   The driver does NOT compute the CRC and does NOT decode anything. The
 *   address check (CRC XOR the AP field), the altitude code and the range
 *   conversion are done by the SPARK parser, where they are proved free of
 *   runtime errors. The driver only guarantees that the frame is
 *   well-formed (rule R6) and that the measurements are present.
 *
 * FRAME LAYOUT (must equal Tcas.Modes_Port.Frame, 24 bytes)
 *   offset  size  field
 *      0     14   data           reply bytes, first byte received first
 *     14      1   length         7 or 14 when valid, else 0
 *     15      1   status         OK or an ERR code, 0 .. 6
 *     16      4   round_trip_ns  nanoseconds, 0 when invalid
 *     20      2   bearing        hundredths of a degree, 0 .. 35 999
 *     22      1   valid          1 valid, 0 invalid
 *     23      1   reserved       always 0
 *   Mode S bit 1 is the most significant bit of data[0], so data[0] holds
 *   bits 1 .. 8, data[1] holds bits 9 .. 16, and so on.
 *
 * DESIGN RULES (each one maps to a MISRA or DO-178C argument)
 *   R1  No dynamic memory. All state lives in structures that the CALLER
 *       owns (MISRA Rule 21.3).
 *   R2  No hidden state. The driver has no static variables; everything it
 *       remembers is in tcas_modes_state_t.
 *   R3  Bounded execution. The only loops copy 14 bytes or clear 14 bytes;
 *       no recursion (MISRA Rule 17.2). The cost of one receive is
 *       constant. The caller loops over replies and bounds that loop by
 *       TCAS_MODES_MAX_REPLIES_PER_CYCLE.
 *   R4  Every pointer argument is checked for NULL and the failure is
 *       reported, never ignored.
 *   R5  The mailbox is volatile. Every register and every data byte is
 *       read exactly ONCE, into a local snapshot, and the checks run on
 *       the snapshot, never on the live registers. The mailbox is
 *       acknowledged once for every reply taken, after the copy, and
 *       never when there was no reply.
 *   R6  FAIL CLOSED. A frame that is not valid carries NOTHING: length 0,
 *       round_trip_ns 0, bearing 0 and every data byte 0, with the error
 *       in status. A valid frame has 0 in every byte after its length, so
 *       the parser never sees stale data. This is exactly what
 *       Tcas.Modes_Port.Is_Well_Formed checks on the Ada side.
 *   R7  A reply that cannot be used is still taken and acknowledged, so
 *       the transceiver is never blocked by one bad reply. The parser or
 *       the next interrogation deals with the missing data.
 *
 * ORDER OF CHECKS IN tcas_modes_receive (the first failure wins, so the
 * reported cause is stable)
 *   1 arguments non-NULL                       ERR_NULL
 *   2 state initialized                        ERR_CONFIG
 *   3 reply ready in the mailbox               ERR_NO_REPLY (nothing taken)
 *   4 no overrun since the last reply          ERR_OVERRUN
 *   5 measurement valid and bearing <= 35 999  ERR_TIMESTAMP
 *   6 length is 7 or 14 bytes                  ERR_LENGTH
 *   From check 4 on, the reply is taken and acknowledged.
 *
 * TIMING BUDGET
 *   One receive: target 3 microseconds (about 20 register reads). A cycle
 *   takes at most TCAS_MODES_MAX_REPLIES_PER_CYCLE (256) replies, about
 *   770 microseconds, inside the interrogation window of the schedule
 *   (Tcas.Config.Window_Interrogate_Ms). [VERIFY] with the transceiver ICD
 *   (RISK-001 of the architecture document).
 *
 * USAGE (the caller, in partition P1, once per cycle)
 *   for (n = 0; n < TCAS_MODES_MAX_REPLIES_PER_CYCLE; n++) {
 *       status = tcas_modes_receive(&state, mailbox_ptr, &frame[n]);
 *       if (status == TCAS_MODES_ERR_NO_REPLY) { break; }
 *       ...
 *   }
 *   The caller must use the return value (MISRA Rule 17.7).
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * Status codes are uint8_t constants, not enum values, so the layout
 *     shared with Ada is fixed and no signed/unsigned mixing occurs
 *     (Rule 10.4).
 *   * _Static_assert needs C11. Allowed under MISRA C:2012 Amendment 3.
 *   * No deviations are requested by this header.
*/

#ifndef TCAS_MODES_DRV_H
#define TCAS_MODES_DRV_H

#include <stdint.h>


// compile-time assertion helper 
#define TCAS_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)

// sizes and limits

#define TCAS_MODES_SHORT_LENGTH  (7U)     /* 56 bits, Tcas.Modes_Port         */
#define TCAS_MODES_LONG_LENGTH   (14U)    /* 112 bits                         */
#define TCAS_MODES_MAX_BEARING   (35999U) /* 359.99 degrees                   */

// replies taken per cycle; must equal Tcas.Config.Max_Replies_Per_Cycle 
#define TCAS_MODES_MAX_REPLIES_PER_CYCLE (256U)

// result codes (must equal Status_* in tcas-modes_port.ads)

typedef uint8_t tcas_modes_status_t;

#define TCAS_MODES_OK             ((tcas_modes_status_t)0U)
#define TCAS_MODES_ERR_NULL       ((tcas_modes_status_t)1U)  /* NULL argument     */
#define TCAS_MODES_ERR_CONFIG     ((tcas_modes_status_t)2U)  /* state not init    */
#define TCAS_MODES_ERR_LENGTH     ((tcas_modes_status_t)3U)  /* not 7 or 14 bytes */
#define TCAS_MODES_ERR_NO_REPLY   ((tcas_modes_status_t)4U)  /* nothing received  */
#define TCAS_MODES_ERR_TIMESTAMP  ((tcas_modes_status_t)5U)  /* no valid hardware measurement: the timestamp or the bearing */
#define TCAS_MODES_ERR_OVERRUN    ((tcas_modes_status_t)6U)  /* replies were lost */


/*
 * the output frame (what the SPARK parser reads; layout is mirrored in
 * Tcas.Modes_Port.Frame with Convention C, 24 bytes)
 *   valid = 1U : the frame is checked, see rule R6
 *   valid = 0U : the frame carries nothing, status says why
*/

typedef struct
{
    uint8_t  data[TCAS_MODES_LONG_LENGTH];  /* reply bytes, 0 after the length */
    uint8_t  length;                        /* 7 or 14 when valid, else 0      */
    uint8_t  status;                        /* TCAS_MODES_OK or an ERR code    */
    uint32_t round_trip_ns;                 /* nanoseconds, 0 when invalid     */
    uint16_t bearing;                       /* hundredths of a degree          */
    uint8_t  valid;                         /* 1U valid, 0U invalid            */
    uint8_t  reserved;                      /* always 0                        */
} tcas_modes_frame_t;

TCAS_STATIC_ASSERT(sizeof(tcas_modes_frame_t) == 24U, "frame must be 24 bytes");


/*
 * hardware mailbox (volatile registers, 36 bytes, no padding)
 * placeholder layout until the transceiver is chosen
 * [VERIFY] register offsets and bit positions against the transceiver ICD
 *   status : bit 0 reply ready, bit 1 overrun, bit 2 measurement valid
 *   length : reply length in bytes, as the transceiver counted it
 *   control: writing 1 acknowledges the reply and frees the mailbox
 *   data   : the reply bytes, first byte received first
 * The transceiver does not change the mailbox while "reply ready" is set;
 * a reply that arrives in the meantime is lost and sets "overrun".
*/

#define TCAS_MODES_HW_READY      UINT32_C(0x00000001)
#define TCAS_MODES_HW_OVERRUN    UINT32_C(0x00000002)
#define TCAS_MODES_HW_MEAS_OK    UINT32_C(0x00000004)
#define TCAS_MODES_HW_ACK        UINT32_C(0x00000001)

typedef struct
{
    volatile uint32_t status;                          /* offset  0 */
    volatile uint32_t length;                          /* offset  4 */
    volatile uint32_t round_trip_ns;                   /* offset  8 */
    volatile uint32_t bearing_cdeg;                    /* offset 12 */
    volatile uint32_t control;                         /* offset 16 */
    volatile uint8_t  data[TCAS_MODES_LONG_LENGTH];    /* offset 20 */
    volatile uint16_t reserved;                        /* offset 34 */
} tcas_modes_hw_mailbox_t;

TCAS_STATIC_ASSERT(sizeof(tcas_modes_hw_mailbox_t) == 36U,
                   "mailbox must be 36 bytes");


/*
 * driver state (owned by the caller, one per receive channel)
 * initialize with tcas_modes_init before the first receive; a zero-filled
 * structure is NOT a valid state: use the init function.
 * all counters saturate at UINT32_MAX and never wrap
*/

typedef struct
{
    uint32_t replies_total;     /* replies taken since init                   */
    uint32_t valid_total;       /* replies delivered as valid                 */
    uint32_t overrun_total;     /* replies discarded because of an overrun    */
    uint32_t measure_fail_total;/* replies discarded for a bad measurement    */
    uint32_t length_fail_total; /* replies discarded for a bad length         */
    uint32_t initialized;       /* 1U once tcas_modes_init succeeded          */
} tcas_modes_state_t;

// interface

/*
 * tcas_modes_init
 *   prepares one driver state. Returns TCAS_MODES_OK or
 *   TCAS_MODES_ERR_NULL; on NULL nothing is written
*/
tcas_modes_status_t tcas_modes_init(tcas_modes_state_t * state);

/*
 * tcas_modes_receive
 *   takes one reply from the mailbox, if there is one, and produces one
 *   frame
 *     state : in/out, counters
 *     hw    : the live (volatile) mailbox; read once, acknowledged once
 *     out   : receives the frame, written on EVERY call (except when out
 *             is NULL), including errors
 *   returns the same code that is stored in out->status; when it returns
 *   anything but TCAS_MODES_OK, out->valid is 0U and the frame carries
 *   nothing (rule R6). TCAS_MODES_ERR_NO_REPLY means the mailbox was empty:
 *   nothing was taken and nothing was acknowledged. Every other error from
 *   check 4 on means a reply WAS taken and acknowledged (rule R7).
*/
tcas_modes_status_t tcas_modes_receive(tcas_modes_state_t * state,
                                       tcas_modes_hw_mailbox_t * hw,
                                       tcas_modes_frame_t * out);

#endif