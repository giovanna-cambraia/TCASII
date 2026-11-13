/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/drivers/arinc429/tcas_a429_drv.h
 * Module      : ARINC 429 receive driver, public interface
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   This is the border where raw ARINC 429 words from the ownship sensors
 *   (air data, radio altimeter) first meet the typed world of the SPARK
 *   core. The hardware leaves each received word in a FIFO. This driver
 *   pops the words, checks them, splits them into fields and hands the
 *   decoder (partition 2) a word that is either VALID (safe to decode) or
 *   INVALID (to be ignored). Nothing reaches the decoder without passing
 *   every check below.
 *
 *   The driver does NOT scale data to engineering units. A BNR data field
 *   is delivered as a raw 19-bit value; the conversion to feet or ft/min is
 *   done by the SPARK decoder, where it is proved free of overflow.
 *
 * ARINC 429 WORD LAYOUT (bit 0 = least significant bit of the uint32_t,
 * bit 0 is "bit 1" of the standard)
 *   bits  0 ..  7   label (arrives bit-reversed, see tcas_a429_label_value)
 *   bits  8 ..  9   SDI  (source/destination identifier)
 *   bits 10 .. 28   data (19 bits, BNR)
 *   bits 29 .. 30   SSM  (sign/status matrix)
 *   bit  31         parity (the whole word must have ODD parity)
 *   [VERIFY] bit order delivered by the target ARINC 429 controller.
 *
 * DESIGN RULES (each one maps to a MISRA or DO-178C argument)
 *   R1  No dynamic memory. All state lives in structures that the CALLER
 *       owns (MISRA Rule 21.3).
 *   R2  No hidden state. The driver has no static variables; everything it
 *       remembers is in tcas_a429_state_t.
 *   R3  Bounded execution. Every loop is bounded by a constant
 *       (TCAS_A429_MAX_WORDS_PER_CYCLE or TCAS_A429_MAX_LABELS); no
 *       recursion (MISRA Rule 17.2). The cost of one drain has a known
 *       upper limit.
 *   R4  Every pointer argument is checked for NULL and the failure is
 *       reported, never ignored.
 *   R5  The FIFO registers are volatile. Each register is read once per
 *       word, and the value read is the value checked.
 *   R6  FAIL CLOSED. Only a BNR word with SSM = Normal Operation is
 *       accepted. Failure Warning, No Computed Data and Functional Test
 *       are rejected, each with its own code so the decoder can fall back
 *       safely (sensitivity level 2 when the altitude source is lost).
 *   R7  On any failure the data field delivered is 0 and valid is 0. A
 *       garbage value is never published, even by accident. After a
 *       parity failure nothing in the word is trusted, so label, SDI and
 *       SSM are 0 as well.
 *   R8  A full FIFO is handled by rate limiting: at most
 *       TCAS_A429_MAX_WORDS_PER_CYCLE words are taken per call; the rest
 *       wait in the hardware FIFO (denial-of-service limit of the security
 *       analysis, never a watchdog reset).
 *
 * ORDER OF CHECKS (the first failure wins, so the reported cause is stable)
 *   word level (tcas_a429_decode_word)
 *     1 arguments non-NULL                     ERR_NULL
 *     2 configuration consistent               ERR_CONFIG
 *     3 odd parity of the 32-bit word          ERR_PARITY
 *     4 label is in the accepted list          ERR_LABEL
 *     5 SSM is Normal Operation                ERR_SSM_FW / ERR_SSM_NCD /
 *                                              ERR_SSM_FT
 *   drain level (tcas_a429_drain)
 *     1 arguments non-NULL                     ERR_NULL
 *     2 configuration and capacity consistent  ERR_CONFIG
 *     3 hardware reports overrun               ERR_OVERRUN (words are
 *                                              still delivered)
 *     4 FIFO still not empty at the limit      ERR_BACKLOG
 *
 * TIMING BUDGET
 *   One word: target 2 microseconds. A full drain of
 *   TCAS_A429_MAX_WORDS_PER_CYCLE (128) words is about 256 microseconds,
 *   inside the ownship ingestion window of the schedule
 *   (Tcas.Config.Window_Ownship_Ms).
 *
 * USAGE (the caller, in partition P1, once per cycle)
 *   status = tcas_a429_drain(&state, &cfg, fifo_ptr, frame,
 *                            words, TCAS_A429_MAX_WORDS_PER_CYCLE, &count);
 *   for each words[i], i < count: read words[i].valid and words[i].status.
 *   The caller must use the return value (MISRA Rule 17.7).
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * Status codes are uint8_t constants, not enum values, so the layout
 *     shared with Ada is fixed and no signed/unsigned mixing occurs
 *     (Rule 10.4).
 *   * Label constants are written in hexadecimal, not octal (Rule 7.1);
 *     the octal value stands in the comment.
 *   * _Static_assert needs C11. Allowed under MISRA C:2012 Amendment 3.
 *   * No deviations are requested by this header.
 * ===========================================================================
 */

#ifndef TCAS_A429_DRV_H
#define TCAS_A429_DRV_H

#include <stdint.h>


// compile-time assertion helper 
#define TCAS_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)


/* 
 * word fields (bit 0 = least significant bit)
*/

#define TCAS_A429_LABEL_MASK     UINT32_C(0x000000FF)
#define TCAS_A429_SDI_MASK       UINT32_C(0x00000300)
#define TCAS_A429_SDI_SHIFT      (8U)
#define TCAS_A429_DATA_MASK      UINT32_C(0x1FFFFC00)
#define TCAS_A429_DATA_SHIFT     (10U)
#define TCAS_A429_SSM_MASK       UINT32_C(0x60000000)
#define TCAS_A429_SSM_SHIFT      (29U)
#define TCAS_A429_PARITY_MASK    UINT32_C(0x80000000)

// the five fields together must cover all 32 bits, with no overlap 
TCAS_STATIC_ASSERT(
    ((TCAS_A429_LABEL_MASK | TCAS_A429_SDI_MASK | TCAS_A429_DATA_MASK |
      TCAS_A429_SSM_MASK | TCAS_A429_PARITY_MASK) == UINT32_C(0xFFFFFFFF)),
    "word fields must cover all 32 bits");

// sign/status matrix, BNR coding (this driver accepts BNR words only) 
#define TCAS_A429_SSM_FAILURE_WARNING  ((uint8_t)0U)   /* 00 */
#define TCAS_A429_SSM_NO_COMPUTED_DATA ((uint8_t)1U)   /* 01 */
#define TCAS_A429_SSM_FUNCTIONAL_TEST  ((uint8_t)2U)   /* 10 */
#define TCAS_A429_SSM_NORMAL           ((uint8_t)3U)   /* 11 */


/*
 * labels accepted by the receive filter
 * placeholders until the Interface Control Document is written
 * [VERIFY] each value against the ARINC 429 ICD of the installation
 * the value is the label's numeric value (octal notation in the comment)
*/

#define TCAS_A429_LABEL_PRESSURE_ALT  ((uint8_t)0x83U)  // octal 203, altitude   
#define TCAS_A429_LABEL_RADIO_ALT     ((uint8_t)0x74U)  // octal 164, radio alt  
#define TCAS_A429_LABEL_ALT_RATE      ((uint8_t)0x8AU)  // octal 212, alt rate   



// limits
#define TCAS_A429_MAX_LABELS           (16U)    // size of the label filter   
#define TCAS_A429_MAX_WORDS_PER_CYCLE  (128U)   // rate limit (rule R8)       



// result codes
typedef uint8_t tcas_a429_status_t;

#define TCAS_A429_OK            ((tcas_a429_status_t)0U)
#define TCAS_A429_ERR_NULL      ((tcas_a429_status_t)1U)  // NULL argument      
#define TCAS_A429_ERR_CONFIG    ((tcas_a429_status_t)2U)  // bad configuration  
#define TCAS_A429_ERR_PARITY    ((tcas_a429_status_t)3U)  // parity not odd     
#define TCAS_A429_ERR_LABEL     ((tcas_a429_status_t)4U)  // label not accepted
#define TCAS_A429_ERR_SSM_FW    ((tcas_a429_status_t)5U)  // failure warning    
#define TCAS_A429_ERR_SSM_NCD   ((tcas_a429_status_t)6U)  // no computed data   
#define TCAS_A429_ERR_SSM_FT    ((tcas_a429_status_t)7U)  // functional test    
#define TCAS_A429_ERR_OVERRUN   ((tcas_a429_status_t)8U)  // FIFO lost words    
#define TCAS_A429_ERR_BACKLOG   ((tcas_a429_status_t)9U)  // FIFO not drained   


/*
 * hardware FIFO (volatile registers, 8 bytes, no padding)
 * placeholder layout until the target SoC is chosen
 * [VERIFY] register offsets and bit positions against the controller manual
 *   status bit 0 : FIFO not empty
 *   status bit 1 : overrun, words were lost since the last status read
 *   data         : reading pops one word from the FIFO
*/

#define TCAS_A429_HW_NOT_EMPTY  UINT32_C(0x00000001)
#define TCAS_A429_HW_OVERRUN    UINT32_C(0x00000002)

typedef struct
{
    volatile uint32_t status;   // FIFO status                                
    volatile uint32_t data;     // next received word, read pops it           
} tcas_a429_hw_fifo_t;

TCAS_STATIC_ASSERT(sizeof(tcas_a429_hw_fifo_t) == 8U, "FIFO must be 8 bytes");


/*
 * configuration (read-only at run time)
 * the filter lists the labels that may pass; every other label is rejected
 *
 * consistency required (checked by tcas_a429_init, ERR_CONFIG otherwise):
 *   1 <= num_labels <= TCAS_A429_MAX_LABELS, no label listed twice,
 *   1 <= max_words <= TCAS_A429_MAX_WORDS_PER_CYCLE.
*/

typedef struct
{
    uint8_t  labels[TCAS_A429_MAX_LABELS];  // accepted label values          
    uint8_t  num_labels;                    // how many entries are in use    
    uint8_t  reserved8;                     // keeps the layout fixed, 0      
    uint16_t max_words;                     // words taken per drain call     
} tcas_a429_cfg_t;


/*
 * driver state (owned by the caller, one per receive channel)
 * initialize with tcas_a429_init before the first drain; a zero-filled
 * structure is NOT a valid state: use the init function.
 * all counters saturate at UINT32_MAX and never wrap
*/

typedef struct
{
    uint32_t words_total;          // words popped since init                 
    uint32_t parity_fail_total;    // words rejected for parity               
    uint32_t label_reject_total;   // words rejected by the label filter      
    uint32_t ssm_reject_total;     // words rejected for a non-normal SSM     
    uint32_t overrun_total;        // drain calls that saw a hardware overrun 
    uint32_t initialized;          // 1U once tcas_a429_init succeeded        
} tcas_a429_state_t;


/*
 * binding with Convention C, 16 bytes)
 *   valid = 1U : data is a checked 19-bit BNR field, right-justified
 *   valid = 0U : ignore data (it is set to 0), status says why
*/

typedef struct
{
    uint32_t           data;       // 19-bit BNR data field, or 0 when invalid 
    uint32_t           frame;      // cycle number of this read                
    uint8_t            label;      // label value after bit reversal           
    uint8_t            sdi;        // source/destination identifier, 0 .. 3    
    uint8_t            ssm;        // sign/status matrix, 0 .. 3               
    tcas_a429_status_t status;     // TCAS_A429_OK or an ERR code              
    uint8_t            valid;      // 1U valid, 0U invalid                     
    uint8_t            reserved8;  // always 0                                 
    uint16_t           reserved16; // always 0                                 
} tcas_a429_word_t;

TCAS_STATIC_ASSERT(sizeof(tcas_a429_word_t) == 16U, "word must be 16 bytes");



// interface

/*
 * tcas_a429_init
 *   prepares one driver state and validates the configuration. Returns
 *   TCAS_A429_OK, TCAS_A429_ERR_NULL or TCAS_A429_ERR_CONFIG; on any error
 *   the state is left filled with a safe empty value (initialized = 0U)
*/
tcas_a429_status_t tcas_a429_init(tcas_a429_state_t * state,
                                  const tcas_a429_cfg_t * cfg);

/*
 * tcas_a429_parity_ok
 *   returns 1U when the 32-bit word has odd parity (the ARINC 429 rule),
 *   else 0U; pure function, no side effects; exposed so the tests can
 *   check it on its own
*/
uint8_t tcas_a429_parity_ok(uint32_t word);

/*
 * tcas_a429_label_value
 *   returns the numeric value of the label of a word: the low 8 bits with
 *   their order reversed (the label is transmitted most significant bit
 *   first); pure function, no side effects
*/
uint8_t tcas_a429_label_value(uint32_t word);

/*
 * tcas_a429_decode_word
 *   checks one raw word and produces one output word
 *     state : in/out, counters
 *     cfg   : label filter
 *     word  : the raw 32-bit word, already copied from the FIFO
 *     frame : current cycle number, recorded in the output
 *     out   : receives the result, written on EVERY call, including errors
 *   returns the same code that is stored in out->status; when it returns
 *   anything but TCAS_A429_OK, out->valid is 0U and out->data is 0 (rule R7)
*/
tcas_a429_status_t tcas_a429_decode_word(tcas_a429_state_t * state,
                                         const tcas_a429_cfg_t * cfg,
                                         uint32_t word,
                                         uint32_t frame,
                                         tcas_a429_word_t * out);

/*
 * tcas_a429_drain
 *   pops words from the hardware FIFO until it is empty or cfg->max_words
 *   words have been taken (rule R8), decoding each one
 *     fifo     : the live (volatile) FIFO registers
 *     out      : array that receives one tcas_a429_word_t per word taken
 *     capacity : length of out; must be at least cfg->max_words
 *     count    : receives the number of entries written to out
 *   returns TCAS_A429_OK, TCAS_A429_ERR_NULL, TCAS_A429_ERR_CONFIG,
 *   TCAS_A429_ERR_OVERRUN or TCAS_A429_ERR_BACKLOG (overrun wins when both
 *   apply). Words are delivered in every case except NULL and CONFIG;
 *   *count is 0U in those two. The status of each word is in out[i].status
*/
tcas_a429_status_t tcas_a429_drain(tcas_a429_state_t * state,
                                   const tcas_a429_cfg_t * cfg,
                                   const tcas_a429_hw_fifo_t * fifo,
                                   uint32_t frame,
                                   tcas_a429_word_t * out,
                                   uint32_t capacity,
                                   uint32_t * count);

#endif