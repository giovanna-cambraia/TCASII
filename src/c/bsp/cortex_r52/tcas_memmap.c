/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/bsp/cortex_r52/tcas_memmap.c
 * Module      : Board memory map and MPU region table, implementation
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : BSP, used once at start-up before any partition runs
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Implements tcas_memmap_build, declared in tcas_memmap.h. The header
 *   holds the contract (the map, its guarantees, the compile-time checks,
 *   the order of checks); this file holds the code that makes it true. Read
 *   the header first.
 *
 * DESIGN RULES (as in the header, with the way each is met here)
 *   R1  No dynamic memory and no state: there are no variables with static
 *       storage and no tables. This file includes only <stddef.h> (NULL)
 *       and the module header.
 *   R2  Bounded execution: no loops. Seven calls to one helper, then one
 *       call to tcas_mpu_validate (which is itself bounded).
 *   R3  The table pointer is checked for NULL before anything is written.
 *   R4  Fail closed: the table is validated with tcas_mpu_validate AFTER it
 *       is filled. If that fails, the count is set back to 0 before the
 *       function returns, so the caller cannot program or use the table.
 *       The count is set to TCAS_MEMMAP_REGION_COUNT only so that the
 *       validation can look at the regions, and it is undone on a failure.
 *   R5  The map is not repeated here. Every address and attribute comes
 *       from the constants of the header, which the compile-time
 *       assertions of the header check. This file adds no number of its
 *       own except the region indices, which are the named constants of
 *       the header (rule R6 of the header).
 *   R6  Entries of the table past the last region are never touched, so a
 *       caller that keeps other data there (or garbage, as the host test
 *       does on purpose) sees it unchanged.
 *
 * STRUCTURE OF THE CODE
 *   One helper (set_region), then the public function. The public function
 *   has ONE exit (a single return at the end) and builds its result in a
 *   local status variable, with a flat if / else-if chain in the order of
 *   the header, so each condition is a separate, testable branch for MC/DC
 *   coverage (DO-178C Table A-7).
 *
 * THE run-time SECOND LINE OF DEFENSE
 *   While the static assertions of the header hold, the table built here is
 *   always valid, and the ERR_INVALID branch cannot be reached. It is kept
 *   on purpose: a static assertion can be removed by a bad edit, and this
 *   branch is what then stops the unit from programming a bad table. The
 *   host test reaches it by wrapping tcas_mpu_validate at link time.
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * No pointer arithmetic: regions are reached by indexing (Rule 18.4).
 *   * The region index is a named constant, below 16 (checked at compile
 *     time in the header), so no index can leave the table (Rule 18.1).
 *   * Every if / else-if chain ends in an else (Rule 15.7).
 *   * The casts to uint8_t are applied only to the access, attribute and
 *     execute-never constants of the header, which are 0 .. 3, 0 .. 1 and
 *     0 .. 1 (Rule 10.8).
 *   * No deviations are requested by this file.
*/

#include <stddef.h>

#include "tcas_memmap.h"

// helper (static, no state)

/* writes all six fields of one region of the table; the reserved byte is
 * always 0. The index is one of the TCAS_MEMMAP_* constants, so it is
 * below TCAS_MEMMAP_REGION_COUNT, which the header proves fits the table */
static void set_region(tcas_mpu_table_t * table,
                       uint32_t index,
                       uint32_t base,
                       uint32_t limit,
                       uint8_t access,
                       uint8_t attr,
                       uint8_t xn)
{
    table->region[index].base     = base;
    table->region[index].limit    = limit;
    table->region[index].access   = access;
    table->region[index].attr     = attr;
    table->region[index].xn       = xn;
    table->region[index].reserved = 0U;
}

// public interface

tcas_memmap_status_t tcas_memmap_build(tcas_mpu_table_t * table)
{
    tcas_memmap_status_t status = TCAS_MEMMAP_OK;
    uint8_t              bad    = TCAS_MPU_NO_INDEX;

    /* check 1: argument */
    if (table == NULL)
    {
        status = TCAS_MEMMAP_ERR_NULL;
    }
    else
    {
        /* the seven regions, in index order (the order of THE MAP) */
        set_region(table, TCAS_MEMMAP_CODE_CORE,
                   TCAS_MEMMAP_CODE_CORE_BASE, TCAS_MEMMAP_CODE_CORE_LIMIT,
                   (uint8_t)TCAS_MEMMAP_CODE_CORE_ACCESS,
                   (uint8_t)TCAS_MEMMAP_CODE_CORE_ATTR,
                   (uint8_t)TCAS_MEMMAP_CODE_CORE_XN);

        set_region(table, TCAS_MEMMAP_CODE_DRV,
                   TCAS_MEMMAP_CODE_DRV_BASE, TCAS_MEMMAP_CODE_DRV_LIMIT,
                   (uint8_t)TCAS_MEMMAP_CODE_DRV_ACCESS,
                   (uint8_t)TCAS_MEMMAP_CODE_DRV_ATTR,
                   (uint8_t)TCAS_MEMMAP_CODE_DRV_XN);

        set_region(table, TCAS_MEMMAP_RODATA,
                   TCAS_MEMMAP_RODATA_BASE, TCAS_MEMMAP_RODATA_LIMIT,
                   (uint8_t)TCAS_MEMMAP_RODATA_ACCESS,
                   (uint8_t)TCAS_MEMMAP_RODATA_ATTR,
                   (uint8_t)TCAS_MEMMAP_RODATA_XN);

        set_region(table, TCAS_MEMMAP_DATA_CORE,
                   TCAS_MEMMAP_DATA_CORE_BASE, TCAS_MEMMAP_DATA_CORE_LIMIT,
                   (uint8_t)TCAS_MEMMAP_DATA_CORE_ACCESS,
                   (uint8_t)TCAS_MEMMAP_DATA_CORE_ATTR,
                   (uint8_t)TCAS_MEMMAP_DATA_CORE_XN);

        set_region(table, TCAS_MEMMAP_DATA_DRV,
                   TCAS_MEMMAP_DATA_DRV_BASE, TCAS_MEMMAP_DATA_DRV_LIMIT,
                   (uint8_t)TCAS_MEMMAP_DATA_DRV_ACCESS,
                   (uint8_t)TCAS_MEMMAP_DATA_DRV_ATTR,
                   (uint8_t)TCAS_MEMMAP_DATA_DRV_XN);

        set_region(table, TCAS_MEMMAP_DEV_IO,
                   TCAS_MEMMAP_DEV_IO_BASE, TCAS_MEMMAP_DEV_IO_LIMIT,
                   (uint8_t)TCAS_MEMMAP_DEV_IO_ACCESS,
                   (uint8_t)TCAS_MEMMAP_DEV_IO_ATTR,
                   (uint8_t)TCAS_MEMMAP_DEV_IO_XN);

        set_region(table, TCAS_MEMMAP_DEV_SUP,
                   TCAS_MEMMAP_DEV_SUP_BASE, TCAS_MEMMAP_DEV_SUP_LIMIT,
                   (uint8_t)TCAS_MEMMAP_DEV_SUP_ACCESS,
                   (uint8_t)TCAS_MEMMAP_DEV_SUP_ATTR,
                   (uint8_t)TCAS_MEMMAP_DEV_SUP_XN);

        /* the count is set so that the validation can look at the regions,
         * and it is undone below if the validation fails (rule R4) */
        table->count = TCAS_MEMMAP_REGION_COUNT;

        /* check 2: the table that was just built is valid */
        if (tcas_mpu_validate(table, &bad) != TCAS_MPU_OK)
        {
            table->count = 0U;                  /* no one can use this table */
            status = TCAS_MEMMAP_ERR_INVALID;
        }
        else
        {
            status = TCAS_MEMMAP_OK;
        }
    }

    return status;
}