/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/hal/mpu/tcas_mpu.h
 * Module      : Memory protection table, validation and programming
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS (HAL), used before any partition starts
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   REQ-001 says the I/O drivers (C) must be isolated from the collision
 *   avoidance logic (SPARK), so that a fault in a driver cannot corrupt an
 *   advisory. The isolation is made by the MPU of the processor: a static
 *   table of memory regions, each with its own access rights, and a fault
 *   for every access that no region allows. This module is that table.
 *
 *   It does four things:
 *     1 VALIDATE a table before any hardware is touched: attributes in
 *       range, alignment, no overlap, no region that is both writable and
 *       executable (W xor X).
 *     2 PROGRAM the Cortex-R52 EL1 MPU from a valid table, in a fixed
 *       order: MPU off, every slot written, MPU on.
 *     3 ENCODE one region into its two register words (pure function), so
 *       the bits can be checked by hand.
 *     4 ANSWER "would this access be allowed?" (pure function), so the
 *       partition isolation can be tested on the host, where the real
 *       fault cannot happen.
 *
 *   WHAT THIS MODULE DOES NOT DO
 *   It does not choose the addresses: the memory map of the board is a
 *   separate file (to be written when the SoC is chosen). It does not
 *   program the memory attribute registers (MAIR): the BSP does, and the
 *   two attribute indices below must match it. It does not use the EL2 MPU
 *   or a second stage of translation. The effect of a programmed table (the
 *   fault itself) can only be shown on the target or in an emulator that
 *   models the R52 MPU.
 *
 * REGION MODEL (ARMv8-R PMSAv8, EL1 MPU)
 *   A region covers base .. limit INCLUSIVE, both 64-byte granules:
 *     base  has its low 6 bits 0           (64-byte aligned)
 *     limit has its low 6 bits 1 (0x3F)    (the last byte of a granule)
 *   An INCLUSIVE limit is used so that no function ever computes
 *   "end + 1", which would overflow for a region ending at 0xFFFFFFFF.
 *   Access rights are one of four codes, equal to the AP field:
 *     0  read/write, privileged only        2  read only, privileged only
 *     1  read/write, any privilege          3  read only, any privilege
 *   There is no "no access" code: to deny an address, no region covers it.
 *   An address covered by no region faults (the background region is off).
 *   [VERIFY] every item of this model against the Cortex-R52 TRM: the AP
 *   encodings, the granule, the number of regions, the attribute indices,
 *   the handling of overlapping regions (treated as an error here).
 *
 * REGISTER WORDS (see tcas_mpu_prbar and tcas_mpu_prlar)
 *   PRBAR = base | (SH << 4) | (AP << 2) | (XN << 1)       SH is always 0
 *   PRLAR = (limit with low 6 bits cleared) | (ATTR << 1) | EN
 *
 * DESIGN RULES (each one maps to a MISRA or DO-178C argument)
 *   R1  No dynamic memory and NO STATE AT ALL. The module has no state
 *       structure: it works on a table that the CALLER owns, and every
 *       function except tcas_mpu_program is pure (MISRA Rule 21.3).
 *   R2  Bounded execution. The only loops run over at most
 *       TCAS_MPU_MAX_REGIONS (16) entries; validation compares at most 120
 *       pairs. No recursion (MISRA Rule 17.2). It runs once, at start-up,
 *       never inside the 1 Hz cycle.
 *   R3  Every pointer argument is checked for NULL and the failure is
 *       reported, never ignored.
 *   R4  FAIL CLOSED. An invalid table touches NO hardware: tcas_mpu_program
 *       validates first and calls no port function on an error. The caller
 *       must not start the application partitions after an error.
 *   R5  The hardware is reached only through THREE port functions
 *       (tcas_mpu_port_*), declared at the end of this header. The target
 *       BSP implements them with the real instructions; the unit tests
 *       implement them as a recorder, so the ORDER of the writes can be
 *       checked on the host.
 *   R6  Fixed programming order: the MPU is disabled, ALL 16 slots are
 *       written in index order (the used ones from the table, the unused
 *       ones disabled), then the MPU is enabled. A stale region from a
 *       previous configuration can never survive a warm start.
 *   R7  W xor X. A region that is writable must be execute-never. There is
 *       no way to make a table with a writable and executable region pass
 *       validation, so injected data can never run as code.
 *   R8  No overlap. Two regions never share an address (the hardware would
 *       fault or be unpredictable). Inclusive limits make the test exact.
 *
 * ORDER OF CHECKS IN tcas_mpu_validate (the first failure wins, so the
 * reported cause and index are stable)
 *   1 arguments non-NULL                            ERR_NULL
 *   2 the count is 1 .. TCAS_MPU_MAX_REGIONS        ERR_COUNT
 *   3 for each used region, in index order:
 *       a access, attr and xn in range, reserved 0  ERR_ATTR
 *       b base 64-byte aligned, limit low bits 0x3F ERR_ALIGN
 *       c limit not below base                      ERR_RANGE
 *       d writable but not execute-never            ERR_WX
 *   4 for each pair i < j of used regions           ERR_OVERLAP
 *   Check 3 runs on ALL regions before check 4 starts. *bad_index is the
 *   index of the region that failed (for an overlap, the LATER one of the
 *   pair), or TCAS_MPU_NO_INDEX when the table is valid or the failure is
 *   not about one region.
 *
 * USAGE (the BSP, once, before any partition starts)
 *   status = tcas_mpu_program(&table, &bad);
 *   if (status != TCAS_MPU_OK) { ... stay in the safe state ... }
 *   The caller must use every return value (MISRA Rule 17.7).
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * Status, access and attribute codes are uint8_t constants, not enum
 *     values, so the layout is fixed and no signed/unsigned mixing occurs
 *     (Rule 10.4).
 *   * _Static_assert needs C11. Allowed under MISRA C:2012 Amendment 3.
 *   * No deviations are requested by this header.
*/

#ifndef TCAS_MPU_H
#define TCAS_MPU_H

#include <stdint.h>


// compile-time assertion helper 
#define TCAS_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)

// limits

// slots of the EL1 MPU that this module programs [VERIFY] the Cortex-R52 can have more; 16 is the conservative minimum 
#define TCAS_MPU_MAX_REGIONS  (16U)

// a region starts and ends on a 64-byte granule 
#define TCAS_MPU_GRANULE_MASK UINT32_C(0x0000003F)

// value of *bad_index when no single region is to blame 
#define TCAS_MPU_NO_INDEX     ((uint8_t)0xFFU)

// access rights (equal to the AP field of PRBAR)

typedef uint8_t tcas_mpu_access_t;

#define TCAS_MPU_ACCESS_RW_PRIV  ((tcas_mpu_access_t)0U)  /* read/write, EL1  */
#define TCAS_MPU_ACCESS_RW_ANY   ((tcas_mpu_access_t)1U)  /* read/write, any  */
#define TCAS_MPU_ACCESS_RO_PRIV  ((tcas_mpu_access_t)2U)  /* read only, EL1   */
#define TCAS_MPU_ACCESS_RO_ANY   ((tcas_mpu_access_t)3U)  /* read only, any   */

// bit 1 of the AP field is set for the read-only codes, bit 0 for the codes that an unprivileged partition may use 
TCAS_STATIC_ASSERT(((TCAS_MPU_ACCESS_RW_PRIV & 2U) == 0U) &&
                   ((TCAS_MPU_ACCESS_RW_ANY  & 2U) == 0U) &&
                   ((TCAS_MPU_ACCESS_RO_PRIV & 2U) == 2U) &&
                   ((TCAS_MPU_ACCESS_RO_ANY  & 2U) == 2U),
                   "bit 1 of the access code means read only");
TCAS_STATIC_ASSERT(((TCAS_MPU_ACCESS_RW_PRIV & 1U) == 0U) &&
                   ((TCAS_MPU_ACCESS_RW_ANY  & 1U) == 1U) &&
                   ((TCAS_MPU_ACCESS_RO_PRIV & 1U) == 0U) &&
                   ((TCAS_MPU_ACCESS_RO_ANY  & 1U) == 1U),
                   "bit 0 of the access code means any privilege");


/*
 * memory attribute indices (into MAIR, programmed by the BSP)
 * [VERIFY] the BSP must program MAIR with the same meaning
*/

typedef uint8_t tcas_mpu_attr_t;

#define TCAS_MPU_ATTR_NORMAL  ((tcas_mpu_attr_t)0U)   /* cached memory        */
#define TCAS_MPU_ATTR_DEVICE  ((tcas_mpu_attr_t)1U)   /* device registers     */

#define TCAS_MPU_ATTR_COUNT   (2U)

// result codes

typedef uint8_t tcas_mpu_status_t;

#define TCAS_MPU_OK            ((tcas_mpu_status_t)0U)
#define TCAS_MPU_ERR_NULL      ((tcas_mpu_status_t)1U)  /* NULL argument       */
#define TCAS_MPU_ERR_COUNT     ((tcas_mpu_status_t)2U)  /* not 1 .. 16 regions */
#define TCAS_MPU_ERR_ATTR      ((tcas_mpu_status_t)3U)  /* a field out of range*/
#define TCAS_MPU_ERR_ALIGN     ((tcas_mpu_status_t)4U)  /* base or limit bits  */
#define TCAS_MPU_ERR_RANGE     ((tcas_mpu_status_t)5U)  /* limit below base    */
#define TCAS_MPU_ERR_WX        ((tcas_mpu_status_t)6U)  /* writable and exec   */
#define TCAS_MPU_ERR_OVERLAP   ((tcas_mpu_status_t)7U)  /* two regions overlap */

// operations asked of tcas_mpu_allows

typedef uint8_t tcas_mpu_op_t;

#define TCAS_MPU_OP_READ   ((tcas_mpu_op_t)0U)
#define TCAS_MPU_OP_WRITE  ((tcas_mpu_op_t)1U)
#define TCAS_MPU_OP_EXEC   ((tcas_mpu_op_t)2U)

/* privilege of the code that makes the access */
#define TCAS_MPU_PRIVILEGED    (1U)
#define TCAS_MPU_UNPRIVILEGED  (0U)


/*
 * one region and the table (owned by the caller)
 *   base, limit : first and last byte, see REGION MODEL
 *   access      : one of TCAS_MPU_ACCESS_*
 *   attr        : one of TCAS_MPU_ATTR_*
 *   xn          : 1U = execute never, 0U = may execute
 *   reserved    : always 0
 * only the first "count" entries of the table are used
*/

typedef struct
{
    uint32_t base;
    uint32_t limit;
    uint8_t  access;
    uint8_t  attr;
    uint8_t  xn;
    uint8_t  reserved;
} tcas_mpu_region_t;

TCAS_STATIC_ASSERT(sizeof(tcas_mpu_region_t) == 12U, "region must be 12 bytes");

typedef struct
{
    tcas_mpu_region_t region[TCAS_MPU_MAX_REGIONS];
    uint32_t          count;
} tcas_mpu_table_t;

TCAS_STATIC_ASSERT(sizeof(tcas_mpu_table_t) == 196U, "table must be 196 bytes");

// register words

#define TCAS_MPU_PRLAR_EN  UINT32_C(0x00000001)   /* region enabled  */

// interface

/*
 * tcas_mpu_validate
 *   checks a table (see ORDER OF CHECKS). Does not touch the hardware and
 *   does not change the table. *bad_index receives the index of the failing
 *   region, or TCAS_MPU_NO_INDEX. On TCAS_MPU_ERR_NULL, *bad_index is
 *   written only when its own pointer is usable
 */
tcas_mpu_status_t tcas_mpu_validate(const tcas_mpu_table_t * table,
                                    uint8_t * bad_index);

/*
 * tcas_mpu_prbar, tcas_mpu_prlar
 *   the two register words of one region, see REGISTER WORDS. Pure
 *   functions; a NULL region returns 0 (a disabled, empty region). The
 *   region is NOT validated here: the caller validates the table
 */
uint32_t tcas_mpu_prbar(const tcas_mpu_region_t * region);
uint32_t tcas_mpu_prlar(const tcas_mpu_region_t * region);

/*
 * tcas_mpu_allows
 *   returns 1U when an access of "size" bytes at "address", by code with
 *   the given privilege (TCAS_MPU_PRIVILEGED or TCAS_MPU_UNPRIVILEGED), of
 *   the given kind (TCAS_MPU_OP_*), would be allowed by the table, else 0U.
 *   The WHOLE access must lie inside ONE region: an access that is not
 *   covered, or that straddles two regions, is denied. A size of 0, an
 *   access that wraps past 0xFFFFFFFF, an unknown operation, a NULL table
 *   or a table that does not pass tcas_mpu_validate all give 0U. Rules for
 *   a covering region:
 *     read   : allowed to privileged code; to unprivileged code only when
 *              the region's access code is "any privilege"
 *     write  : as read, and only for a read/write access code
 *     exec   : as read, and only when xn is 0U
 *   Pure function, no side effects
 */
uint8_t tcas_mpu_allows(const tcas_mpu_table_t * table,
                        uint32_t address,
                        uint32_t size,
                        tcas_mpu_op_t op,
                        uint32_t privileged);

/*
 * tcas_mpu_program
 *   validates the table and, only if it is valid, programs the MPU in the
 *   order of rule R6: tcas_mpu_port_disable, then tcas_mpu_port_write_region
 *   for index 0 .. 15 (the used ones from the table, the unused ones with
 *   both words 0), then tcas_mpu_port_enable. On any error from
 *   tcas_mpu_validate it returns that error and calls NO port function (rule
 *   R4). *bad_index is set as in tcas_mpu_validate
 */
tcas_mpu_status_t tcas_mpu_program(const tcas_mpu_table_t * table,
                                   uint8_t * bad_index);


/*
 * port functions (rule R5)
 * implemented by the target BSP with the real instructions, and by the unit
 * tests as a recorder
 *   tcas_mpu_port_disable      : turns the MPU off, then a data and an
 *                                instruction barrier
 *   tcas_mpu_port_write_region : selects slot "index" and writes its two
 *                                words (select, PRBAR, PRLAR)
 *   tcas_mpu_port_enable       : turns the MPU on with the background
 *                                region OFF, then a data and an
 *                                instruction barrier
 * [VERIFY] the register names and the barrier sequence against the
 * Cortex-R52 TRM
*/

void tcas_mpu_port_disable(void);
void tcas_mpu_port_write_region(uint32_t index, uint32_t prbar, uint32_t prlar);
void tcas_mpu_port_enable(void);

#endif