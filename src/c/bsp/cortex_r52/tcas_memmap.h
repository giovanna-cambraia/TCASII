/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/bsp/cortex_r52/tcas_memmap.h
 * Module      : Board memory map and MPU region table, public interface
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : BSP, used once at start-up before any partition runs
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   REQ-001 asks for spatial partitioning: the I/O drivers (C) must not be
 *   able to corrupt the collision avoidance core (SPARK). tcas_mpu.h
 *   provides the MECHANISM (a validated region table and its programming).
 *   This header provides the POLICY: which address range belongs to which
 *   partition, and with what rights. It holds the address constants of the
 *   board, and one function that builds the region table from them.
 *
 *   The constants are the single source for three things: the table built
 *   here, the linker script (link.ld, which must place each section inside
 *   its region), and the host test that proves the isolation of this very
 *   layout with tcas_mpu_allows.
 *
 * THE MAP (placeholder addresses for a Cortex-R52 part)
 *   i  name        range                       access   xn  attr    holds
 *   0  CODE_CORE   0x00000000 .. 0x0007FFFF    RO_PRIV  0   normal  SPARK core code
 *   1  CODE_DRV    0x00080000 .. 0x0008FFFF    RO_ANY   0   normal  C driver code and
 *                                                                    its constants
 *   2  RODATA      0x00090000 .. 0x0009FFFF    RO_PRIV  1   normal  core constants
 *   3  DATA_CORE   0x20000000 .. 0x2000FFFF    RW_PRIV  1   normal  track table, RA
 *                                                                    state, core stack
 *   4  DATA_DRV    0x20010000 .. 0x2001FFFF    RW_ANY   1   normal  driver state and
 *                                                                    stack, the frames
 *                                                                    that cross the
 *                                                                    Ada / C boundary
 *   5  DEV_IO      0x40000000 .. 0x4000FFFF    RW_ANY   1   device  ARINC 429 FIFO,
 *                                                                    Mode S mailbox,
 *                                                                    display unit
 *   6  DEV_SUP     0x40010000 .. 0x4001FFFF    RW_PRIV  1   device  watchdog and cycle
 *                                                                    clock
 *   Limits are INCLUSIVE (the address of the last byte), as in tcas_mpu.h.
 *   [VERIFY] every address, size and attribute against the SoC memory map,
 *   and the section placement against the linker script.
 *
 * WHAT THE MAP GUARANTEES (each is proved by the host test, not assumed)
 *   * Unprivileged code (the I/O drivers) cannot read or write the core
 *     data, cannot read the core constants or the core code, and cannot
 *     reach the watchdog or the clock registers.
 *   * Nobody can write any code or any constant, not even the core.
 *   * No data region and no device region can execute (W xor X).
 *   * An access that starts in one region and ends in the next is denied.
 *   * Every address outside the seven regions is denied (the background
 *     region is off).
 *
 * OPEN ITEM FOR THE v1 DOCUMENTATION
 *   The table says what unprivileged code MAY do. It does not by itself
 *   make the drivers run unprivileged. With one static table and one cyclic
 *   executive, that needs a privilege switch on every driver call (an SVC
 *   or an EL0 trampoline), or one table per partition. That belongs to the
 *   BSP start-up code and is not part of this module.
 *
 * DESIGN RULES (each one maps to a MISRA or DO-178C argument)
 *   R1  No dynamic memory and no state. The function works on a table that
 *       the CALLER owns (MISRA Rule 21.3).
 *   R2  Bounded execution. One fixed list of region assignments and one
 *       call to tcas_mpu_validate (which itself is bounded). No loops of
 *       its own. No recursion (MISRA Rule 17.2). It runs once, at start-up.
 *   R3  Every pointer argument is checked for NULL and the failure is
 *       reported, never ignored.
 *   R4  FAIL CLOSED. The function validates its OWN result with
 *       tcas_mpu_validate. If the table it built is not valid, it returns
 *       an error and leaves the table with count 0, which tcas_mpu_validate
 *       and tcas_mpu_program both refuse. A table that fails validation is
 *       never handed back as if it were good.
 *   R5  The map is checked at COMPILE time (see the static assertions at
 *       the end of this header): alignment, limit not below base, strictly
 *       ascending and non-overlapping regions, W xor X, and a count that
 *       fits the 16 MPU slots. A bad edit of an address stops the build.
 *   R6  The region indices are named constants, so the linker script, the
 *       tests and the executive never use a bare number for a region.
 *
 * ORDER OF CHECKS IN tcas_memmap_build
 *   1 argument non-NULL                              ERR_NULL
 *   2 the table built passes tcas_mpu_validate       ERR_INVALID
 *   (check 2 cannot fail while the static assertions below hold; it is the
 *   run-time second line of defense, and its refusal is tested)
 *
 * USAGE (the BSP, once, before any partition starts)
 *   status = tcas_memmap_build(&table);
 *   if (status == TCAS_MEMMAP_OK) { status = tcas_mpu_program(&table, &bad); }
 *   The caller must use every return value (MISRA Rule 17.7).
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * Status codes are uint8_t constants, not enum values (Rule 10.4).
 *   * _Static_assert needs C11. Allowed under MISRA C:2012 Amendment 3.
 *   * No deviations are requested by this header.
 *
 * INCLUDE PATH
 *   -I src/c/common -I src/c/hal/mpu -I src/c/bsp/cortex_r52
*/

#ifndef TCAS_MEMMAP_H
#define TCAS_MEMMAP_H

#include <stdint.h>

#include "tcas_common.h"
#include "tcas_mpu.h"


// region indices (rule R6): the order is the order of ascending addresses
#define TCAS_MEMMAP_CODE_CORE   (0U)
#define TCAS_MEMMAP_CODE_DRV    (1U)
#define TCAS_MEMMAP_RODATA      (2U)
#define TCAS_MEMMAP_DATA_CORE   (3U)
#define TCAS_MEMMAP_DATA_DRV    (4U)
#define TCAS_MEMMAP_DEV_IO      (5U)
#define TCAS_MEMMAP_DEV_SUP     (6U)

#define TCAS_MEMMAP_REGION_COUNT (7U)

// the map: base and limit (inclusive), access, attribute and execute-never of every region. [VERIFY] every value against the SoC memory map

// 0 SPARK core code: readable by privileged code only, may execute */
#define TCAS_MEMMAP_CODE_CORE_BASE    UINT32_C(0x00000000)
#define TCAS_MEMMAP_CODE_CORE_LIMIT   UINT32_C(0x0007FFFF)
#define TCAS_MEMMAP_CODE_CORE_ACCESS  TCAS_MPU_ACCESS_RO_PRIV
#define TCAS_MEMMAP_CODE_CORE_ATTR    TCAS_MPU_ATTR_NORMAL
#define TCAS_MEMMAP_CODE_CORE_XN      (0U)

/* 1 C driver code and its constants: readable and executable by any code */
#define TCAS_MEMMAP_CODE_DRV_BASE     UINT32_C(0x00080000)
#define TCAS_MEMMAP_CODE_DRV_LIMIT    UINT32_C(0x0008FFFF)
#define TCAS_MEMMAP_CODE_DRV_ACCESS   TCAS_MPU_ACCESS_RO_ANY
#define TCAS_MEMMAP_CODE_DRV_ATTR     TCAS_MPU_ATTR_NORMAL
#define TCAS_MEMMAP_CODE_DRV_XN       (0U)

/* 2 core constants (threshold tables): privileged read only, never executed */
#define TCAS_MEMMAP_RODATA_BASE       UINT32_C(0x00090000)
#define TCAS_MEMMAP_RODATA_LIMIT      UINT32_C(0x0009FFFF)
#define TCAS_MEMMAP_RODATA_ACCESS     TCAS_MPU_ACCESS_RO_PRIV
#define TCAS_MEMMAP_RODATA_ATTR       TCAS_MPU_ATTR_NORMAL
#define TCAS_MEMMAP_RODATA_XN         (1U)

/* 3 core data: privileged read and write, closed to unprivileged code */
#define TCAS_MEMMAP_DATA_CORE_BASE    UINT32_C(0x20000000)
#define TCAS_MEMMAP_DATA_CORE_LIMIT   UINT32_C(0x2000FFFF)
#define TCAS_MEMMAP_DATA_CORE_ACCESS  TCAS_MPU_ACCESS_RW_PRIV
#define TCAS_MEMMAP_DATA_CORE_ATTR    TCAS_MPU_ATTR_NORMAL
#define TCAS_MEMMAP_DATA_CORE_XN      (1U)

/* 4 driver data and the frames that cross the Ada / C boundary */
#define TCAS_MEMMAP_DATA_DRV_BASE     UINT32_C(0x20010000)
#define TCAS_MEMMAP_DATA_DRV_LIMIT    UINT32_C(0x2001FFFF)
#define TCAS_MEMMAP_DATA_DRV_ACCESS   TCAS_MPU_ACCESS_RW_ANY
#define TCAS_MEMMAP_DATA_DRV_ATTR     TCAS_MPU_ATTR_NORMAL
#define TCAS_MEMMAP_DATA_DRV_XN       (1U)

/* 5 I/O device registers used by the drivers (device memory) */
#define TCAS_MEMMAP_DEV_IO_BASE       UINT32_C(0x40000000)
#define TCAS_MEMMAP_DEV_IO_LIMIT      UINT32_C(0x4000FFFF)
#define TCAS_MEMMAP_DEV_IO_ACCESS     TCAS_MPU_ACCESS_RW_ANY
#define TCAS_MEMMAP_DEV_IO_ATTR       TCAS_MPU_ATTR_DEVICE
#define TCAS_MEMMAP_DEV_IO_XN         (1U)

/* 6 supervision registers, watchdog and clock: privileged only (device) */
#define TCAS_MEMMAP_DEV_SUP_BASE      UINT32_C(0x40010000)
#define TCAS_MEMMAP_DEV_SUP_LIMIT     UINT32_C(0x4001FFFF)
#define TCAS_MEMMAP_DEV_SUP_ACCESS    TCAS_MPU_ACCESS_RW_PRIV
#define TCAS_MEMMAP_DEV_SUP_ATTR      TCAS_MPU_ATTR_DEVICE
#define TCAS_MEMMAP_DEV_SUP_XN        (1U)

// result codes

typedef uint8_t tcas_memmap_status_t;

#define TCAS_MEMMAP_OK           ((tcas_memmap_status_t)0U)
#define TCAS_MEMMAP_ERR_NULL     ((tcas_memmap_status_t)1U)  /* NULL argument     */
#define TCAS_MEMMAP_ERR_INVALID  ((tcas_memmap_status_t)2U)  /* table not valid   */

// interface

/*
 * tcas_memmap_build
 *   fills a table with the seven regions of THE MAP, in index order, count
 *   TCAS_MEMMAP_REGION_COUNT, and validates its own result with
 *   tcas_mpu_validate (rule R4). Returns TCAS_MEMMAP_OK, or
 *   TCAS_MEMMAP_ERR_NULL (nothing written), or TCAS_MEMMAP_ERR_INVALID. On
 *   ERR_INVALID the table is left with count 0, which tcas_mpu_validate and
 *   tcas_mpu_program both refuse. Entries of the table past the seventh are
 *   not changed
 */
tcas_memmap_status_t tcas_memmap_build(tcas_mpu_table_t * table);


/*
 * the map, checked at COMPILE time (rule R5)
 * a bad edit of an address stops the build with a message that names the
 * region
*/

// shape of one region: base starts a granule, limit ends one, limit is not below base 
#define TCAS_MEMMAP_SHAPE_OK(base, limit) \
    ((((base) & TCAS_MPU_GRANULE_MASK) == UINT32_C(0)) && \
     (((limit) & TCAS_MPU_GRANULE_MASK) == TCAS_MPU_GRANULE_MASK) && \
     ((limit) >= (base)))

// W xor X: an access code below 2 is writable, and then xn must be 1 
#define TCAS_MEMMAP_WX_OK(access, xn) \
    (((access) >= 2U) || ((xn) == 1U))

TCAS_STATIC_ASSERT(TCAS_MEMMAP_SHAPE_OK(TCAS_MEMMAP_CODE_CORE_BASE,
                                        TCAS_MEMMAP_CODE_CORE_LIMIT),
                   "CODE_CORE: base and limit must be 64-byte granules");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_SHAPE_OK(TCAS_MEMMAP_CODE_DRV_BASE,
                                        TCAS_MEMMAP_CODE_DRV_LIMIT),
                   "CODE_DRV: base and limit must be 64-byte granules");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_SHAPE_OK(TCAS_MEMMAP_RODATA_BASE,
                                        TCAS_MEMMAP_RODATA_LIMIT),
                   "RODATA: base and limit must be 64-byte granules");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_SHAPE_OK(TCAS_MEMMAP_DATA_CORE_BASE,
                                        TCAS_MEMMAP_DATA_CORE_LIMIT),
                   "DATA_CORE: base and limit must be 64-byte granules");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_SHAPE_OK(TCAS_MEMMAP_DATA_DRV_BASE,
                                        TCAS_MEMMAP_DATA_DRV_LIMIT),
                   "DATA_DRV: base and limit must be 64-byte granules");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_SHAPE_OK(TCAS_MEMMAP_DEV_IO_BASE,
                                        TCAS_MEMMAP_DEV_IO_LIMIT),
                   "DEV_IO: base and limit must be 64-byte granules");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_SHAPE_OK(TCAS_MEMMAP_DEV_SUP_BASE,
                                        TCAS_MEMMAP_DEV_SUP_LIMIT),
                   "DEV_SUP: base and limit must be 64-byte granules");

// the regions are strictly ascending, so none of them can overlap another: each limit is below the next base 
TCAS_STATIC_ASSERT(TCAS_MEMMAP_CODE_CORE_LIMIT < TCAS_MEMMAP_CODE_DRV_BASE,
                   "CODE_CORE must end below CODE_DRV");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_CODE_DRV_LIMIT < TCAS_MEMMAP_RODATA_BASE,
                   "CODE_DRV must end below RODATA");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_RODATA_LIMIT < TCAS_MEMMAP_DATA_CORE_BASE,
                   "RODATA must end below DATA_CORE");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_DATA_CORE_LIMIT < TCAS_MEMMAP_DATA_DRV_BASE,
                   "DATA_CORE must end below DATA_DRV");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_DATA_DRV_LIMIT < TCAS_MEMMAP_DEV_IO_BASE,
                   "DATA_DRV must end below DEV_IO");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_DEV_IO_LIMIT < TCAS_MEMMAP_DEV_SUP_BASE,
                   "DEV_IO must end below DEV_SUP");

/* W xor X in every region (rule R7 of tcas_mpu.h) */
TCAS_STATIC_ASSERT(TCAS_MEMMAP_WX_OK(TCAS_MEMMAP_CODE_CORE_ACCESS,
                                     TCAS_MEMMAP_CODE_CORE_XN),
                   "CODE_CORE: a writable region must be execute-never");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_WX_OK(TCAS_MEMMAP_CODE_DRV_ACCESS,
                                     TCAS_MEMMAP_CODE_DRV_XN),
                   "CODE_DRV: a writable region must be execute-never");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_WX_OK(TCAS_MEMMAP_RODATA_ACCESS,
                                     TCAS_MEMMAP_RODATA_XN),
                   "RODATA: a writable region must be execute-never");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_WX_OK(TCAS_MEMMAP_DATA_CORE_ACCESS,
                                     TCAS_MEMMAP_DATA_CORE_XN),
                   "DATA_CORE: a writable region must be execute-never");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_WX_OK(TCAS_MEMMAP_DATA_DRV_ACCESS,
                                     TCAS_MEMMAP_DATA_DRV_XN),
                   "DATA_DRV: a writable region must be execute-never");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_WX_OK(TCAS_MEMMAP_DEV_IO_ACCESS,
                                     TCAS_MEMMAP_DEV_IO_XN),
                   "DEV_IO: a writable region must be execute-never");
TCAS_STATIC_ASSERT(TCAS_MEMMAP_WX_OK(TCAS_MEMMAP_DEV_SUP_ACCESS,
                                     TCAS_MEMMAP_DEV_SUP_XN),
                   "DEV_SUP: a writable region must be execute-never");

/* the map fits the MPU slots, and the indices are the ascending order */
TCAS_STATIC_ASSERT(TCAS_MEMMAP_REGION_COUNT <= TCAS_MPU_MAX_REGIONS,
                   "the map must fit the MPU slots");
TCAS_STATIC_ASSERT((TCAS_MEMMAP_CODE_CORE == 0U) && (TCAS_MEMMAP_CODE_DRV == 1U) &&
                   (TCAS_MEMMAP_RODATA == 2U) && (TCAS_MEMMAP_DATA_CORE == 3U) &&
                   (TCAS_MEMMAP_DATA_DRV == 4U) && (TCAS_MEMMAP_DEV_IO == 5U) &&
                   (TCAS_MEMMAP_DEV_SUP == 6U) &&
                   (TCAS_MEMMAP_REGION_COUNT == 7U),
                   "region indices must be 0 .. 6 in ascending address order");

#endif