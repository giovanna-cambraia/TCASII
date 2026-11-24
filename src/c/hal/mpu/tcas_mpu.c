/* ===========================================================================
 * TCAS II - Traffic Alert and Collision Avoidance System
 * ===========================================================================
 * File        : src/c/hal/mpu/tcas_mpu.c
 * Module      : Memory protection table, validation and programming
 * Language    : C11, written to MISRA C:2012 (Amendment 2/3)
 * Assurance   : DO-178C Software Level B (target)
 * Partition   : P1 IO_DRIVERS (HAL), used before any partition starts
 * Requirements: REQ-001, REQ-004 (placeholder IDs)
 *
 * PURPOSE
 *   Implements the interface declared in tcas_mpu.h. The header holds the
 *   contract (the region model, the register words, design rules, order of
 *   checks); this file holds the code that makes it true. Read the header
 *   first.
 *
 *   The three port functions (tcas_mpu_port_disable, _write_region and
 *   _enable) are NOT defined here. They are the only code that touches the
 *   hardware: the target BSP defines them with the real instructions, and
 *   the unit tests define them as a recorder (rule R5).
 *
 * DESIGN RULES (as in the header, with the way each is met here)
 *   R1  No dynamic memory and no state: there are no variables with static
 *       storage and no tables. Every function except tcas_mpu_program is
 *       pure. This file includes only <stddef.h> (NULL) and the header.
 *   R2  Bounded execution: every loop runs over at most 16 entries (the
 *       overlap check has at most 120 pairs). No recursion. The count is
 *       checked against TCAS_MPU_MAX_REGIONS BEFORE any region is indexed,
 *       so no loop can read past the table.
 *   R3  Every pointer argument is checked for NULL before anything is read
 *       or written.
 *   R4  Fail closed: tcas_mpu_program validates first and calls no port
 *       function unless the table is valid.
 *   R5  The hardware is reached only through the three port functions, in
 *       tcas_mpu_program, and nowhere else.
 *   R6  Fixed programming order: disable, slots 0 .. 15 in index order (the
 *       used ones from the table, the unused ones with both words 0),
 *       enable. The unused slots are written on purpose, so a region from
 *       a previous configuration cannot survive a warm start.
 *   R7  W xor X is part of the per-region check (a writable region must be
 *       execute-never), so no valid table can contain a region that is
 *       both.
 *   R8  The overlap test is inclusive on both ends and never computes
 *       "end + 1" (which would overflow for a region that ends at
 *       0xFFFFFFFF).
 *
 * STRUCTURE OF THE CODE
 *   Every public function has ONE exit (a single return at the end) and
 *   builds its result in local variables. The decision chains are flat
 *   if / else-if sequences, one branch per check, in the order of the
 *   header, so each condition is a separate, testable branch for MC/DC
 *   coverage (DO-178C Table A-7). The loops have no early exit: a status
 *   guard keeps the FIRST failure, so the reported cause and index are
 *   stable and the cost is constant.
 *
 * OVERLAP ORDER
 *   Pairs are visited with the later index j ascending (1 .. count-1) and,
 *   for each j, the earlier index i ascending (0 .. j-1). The first
 *   overlap found wins, so *bad_index is the smallest j that overlaps any
 *   earlier region.
 *
 * PRIVILEGE (tcas_mpu_allows)
 *   Only the value TCAS_MPU_PRIVILEGED counts as privileged. Every other
 *   value is treated as unprivileged (fail closed).
 *
 * COST OF tcas_mpu_allows
 *   It validates the table on EVERY call (up to 16 region checks and 120
 *   pair comparisons). It is meant for the start-up self-check and for the
 *   unit tests, not for the 1 Hz cycle.
 *
 * MISRA NOTES
 *   * Fixed-width types only (Directive 4.6).
 *   * No pointer arithmetic: regions are reached by indexing (Rule 18.4).
 *   * Every if / else-if chain ends in an else (Rule 15.7).
 *   * Loop counters are not modified in the loop body (Rule 14.2).
 *   * Casts to uint8_t are applied only to loop indices below 16, and to
 *     values already checked (Rule 10.8).
 *   * No deviations are requested by this file.
*/

#include <stddef.h>

#include "tcas_mpu.h"


// helpers (static, no state)

// the checks of ONE region, in the order of the header (check 3 a .. d); the first failure wins 
static tcas_mpu_status_t check_region(const tcas_mpu_region_t * r)
{
    tcas_mpu_status_t status = TCAS_MPU_OK;

    /* 3 a: every field in range, reserved is 0 */
    if ((r->access > TCAS_MPU_ACCESS_RO_ANY) ||
        (r->attr >= TCAS_MPU_ATTR_COUNT) ||
        (r->xn > 1U) ||
        (r->reserved != 0U))
    {
        status = TCAS_MPU_ERR_ATTR;
    }
    /* 3 b: base starts a granule, limit ends one */
    else if (((r->base & TCAS_MPU_GRANULE_MASK) != 0U) ||
             ((r->limit & TCAS_MPU_GRANULE_MASK) != TCAS_MPU_GRANULE_MASK))
    {
        status = TCAS_MPU_ERR_ALIGN;
    }
    /* 3 c: the limit is not below the base */
    else if (r->limit < r->base)
    {
        status = TCAS_MPU_ERR_RANGE;
    }
    /* 3 d: W xor X: a writable region must be execute-never (rule R7);
     * bit 1 of the access code means read only */
    else if (((r->access & 2U) == 0U) && (r->xn == 0U))
    {
        status = TCAS_MPU_ERR_WX;
    }
    else
    {
        status = TCAS_MPU_OK;
    }

    return status;
}

// 1U when the two regions share at least one address (rule R8) 
static uint8_t overlaps(const tcas_mpu_region_t * a,
                        const tcas_mpu_region_t * b)
{
    uint8_t shared = 0U;

    if ((a->base <= b->limit) && (b->base <= a->limit))
    {
        shared = 1U;
    }
    else
    {
        shared = 0U;
    }

    return shared;
}

// 1U when the code with this privilege may use the region at all: only exactly TCAS_MPU_PRIVILEGED counts as privileged, and unprivileged code needs a region whose access 
// code has bit 0 set ("any privilege") 
static uint8_t privilege_ok(const tcas_mpu_region_t * r, uint32_t privileged)
{
    uint8_t ok = 0U;

    if (privileged == TCAS_MPU_PRIVILEGED)
    {
        ok = 1U;
    }
    else if ((r->access & 1U) == 1U)
    {
        ok = 1U;
    }
    else
    {
        ok = 0U;
    }

    return ok;
}

// 1U when the region allows this kind of access for this privilege 
static uint8_t region_allows(const tcas_mpu_region_t * r,
                             tcas_mpu_op_t op,
                             uint32_t privileged)
{
    uint8_t allowed = 0U;

    if (privilege_ok(r, privileged) == 0U)
    {
        allowed = 0U;
    }
    else if (op == TCAS_MPU_OP_READ)
    {
        allowed = 1U;
    }
    else if (op == TCAS_MPU_OP_WRITE)
    {
        /* bit 1 of the access code means read only */
        if ((r->access & 2U) == 0U)
        {
            allowed = 1U;
        }
        else
        {
            allowed = 0U;
        }
    }
    else if (op == TCAS_MPU_OP_EXEC)
    {
        if (r->xn == 0U)
        {
            allowed = 1U;
        }
        else
        {
            allowed = 0U;
        }
    }
    else
    {
        allowed = 0U;                  /* unknown operation */
    }

    return allowed;
}

// public interface

tcas_mpu_status_t tcas_mpu_validate(const tcas_mpu_table_t * table,
                                    uint8_t * bad_index)
{
    tcas_mpu_status_t status = TCAS_MPU_OK;
    uint8_t           bad    = TCAS_MPU_NO_INDEX;
    uint32_t          i      = 0U;
    uint32_t          j      = 0U;

    /* check 1: arguments; check 2: the count */
    if ((table == NULL) || (bad_index == NULL))
    {
        status = TCAS_MPU_ERR_NULL;
    }
    else if ((table->count < 1U) || (table->count > TCAS_MPU_MAX_REGIONS))
    {
        status = TCAS_MPU_ERR_COUNT;
    }
    else
    {
        /* check 3: every used region on its own, in index order; the
         * first failure is kept (no early exit) */
        for (i = 0U; i < table->count; i++)
        {
            if (status == TCAS_MPU_OK)
            {
                status = check_region(&table->region[i]);

                if (status != TCAS_MPU_OK)
                {
                    bad = (uint8_t)i;
                }
                else
                {
                    /* this region is fine */
                }
            }
            else
            {
                /* an earlier region already failed */
            }
        }

        /* check 4: every pair, only when every region passed check 3 */
        if (status == TCAS_MPU_OK)
        {
            for (j = 1U; j < table->count; j++)
            {
                for (i = 0U; i < j; i++)
                {
                    if ((status == TCAS_MPU_OK) &&
                        (overlaps(&table->region[i], &table->region[j]) == 1U))
                    {
                        status = TCAS_MPU_ERR_OVERLAP;
                        bad    = (uint8_t)j;     /* the LATER region */
                    }
                    else
                    {
                        /* no overlap, or one was already found */
                    }
                }
            }
        }
        else
        {
            /* a region failed check 3: pairs are not examined */
        }
    }

    /* written once, and only when its own pointer is usable */
    if (bad_index != NULL)
    {
        *bad_index = bad;
    }
    else
    {
        /* nowhere to report the index */
    }

    return status;
}

uint32_t tcas_mpu_prbar(const tcas_mpu_region_t * region)
{
    uint32_t word = 0U;

    if (region == NULL)
    {
        word = 0U;                     /* a disabled, empty region */
    }
    else
    {
        /* base (upper 26 bits) | AP << 2 | XN << 1; SH is always 0. Each
         * field is masked to its width, so a bad field of an unvalidated
         * region cannot leak into its neighbor */
        word = (region->base & ~TCAS_MPU_GRANULE_MASK)
             | (((uint32_t)region->access & UINT32_C(3)) << 2U)
             | (((uint32_t)region->xn & UINT32_C(1)) << 1U);
    }

    return word;
}

uint32_t tcas_mpu_prlar(const tcas_mpu_region_t * region)
{
    uint32_t word = 0U;

    if (region == NULL)
    {
        word = 0U;                     /* a disabled, empty region */
    }
    else
    {
        /* limit (upper 26 bits) | ATTR << 1 | EN. A used slot is always
         * enabled. The attribute index is masked to its three bits */
        word = (region->limit & ~TCAS_MPU_GRANULE_MASK)
             | (((uint32_t)region->attr & UINT32_C(7)) << 1U)
             | TCAS_MPU_PRLAR_EN;
    }

    return word;
}

uint8_t tcas_mpu_allows(const tcas_mpu_table_t * table,
                        uint32_t address,
                        uint32_t size,
                        tcas_mpu_op_t op,
                        uint32_t privileged)
{
    uint8_t  allowed = 0U;
    uint8_t  bad     = TCAS_MPU_NO_INDEX;
    uint32_t i       = 0U;

    if (table == NULL)
    {
        allowed = 0U;
    }
    else if (size == 0U)
    {
        allowed = 0U;
    }
    else if ((size - 1U) > (UINT32_MAX - address))
    {
        allowed = 0U;                  /* the access wraps past the top */
    }
    else if (op > TCAS_MPU_OP_EXEC)
    {
        allowed = 0U;                  /* unknown operation */
    }
    else if (tcas_mpu_validate(table, &bad) != TCAS_MPU_OK)
    {
        allowed = 0U;                  /* a table that is not valid allows
                                        * nothing */
    }
    else
    {
        /* address of the LAST byte: the whole access must be in ONE
         * region. The table is valid, so its count is 1 .. 16 and no two
         * regions overlap: at most one region covers the first byte */
        const uint32_t last = address + (size - 1U);

        for (i = 0U; i < table->count; i++)
        {
            if ((address >= table->region[i].base) &&
                (address <= table->region[i].limit) &&
                (last <= table->region[i].limit))
            {
                allowed = region_allows(&table->region[i], op, privileged);
            }
            else
            {
                /* not this region, or the access runs past its limit */
            }
        }
    }

    return allowed;
}

tcas_mpu_status_t tcas_mpu_program(const tcas_mpu_table_t * table,
                                   uint8_t * bad_index)
{
    /* validate first: an invalid table touches no hardware (rule R4) */
    tcas_mpu_status_t status = tcas_mpu_validate(table, bad_index);
    uint32_t          i      = 0U;

    if (status == TCAS_MPU_OK)
    {
        /* the table is not NULL and its count is 1 .. 16 (validated) */
        tcas_mpu_port_disable();

        /* all 16 slots, in index order (rule R6) */
        for (i = 0U; i < TCAS_MPU_MAX_REGIONS; i++)
        {
            if (i < table->count)
            {
                tcas_mpu_port_write_region(i,
                                           tcas_mpu_prbar(&table->region[i]),
                                           tcas_mpu_prlar(&table->region[i]));
            }
            else
            {
                /* unused slot: both words 0, so it is disabled */
                tcas_mpu_port_write_region(i, 0U, 0U);
            }
        }

        tcas_mpu_port_enable();
    }
    else
    {
        /* invalid table: no port function was called */
    }

    return status;
}