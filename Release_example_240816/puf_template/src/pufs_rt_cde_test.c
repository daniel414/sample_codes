/**
 * @file pufs_rt_cde_test.c
 * @brief CDE API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_rt_cde_test.h"
#include "pufs_log.h"
#include "pufs_rt.h"
#include "pufs_rt_cde.h"
#include "pufs_rt_cde_reg.h"
#include "pufs_rt_internal.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_rt_cde_read_write_test(void)
{
    uint32_t rand, expected;
    uint32_t old_val[32];
    pufs_rand((uint8_t *)&rand, 1);

    for (uint32_t index = 0; index < 32; index++)
    {
        old_val[index] = rt_cde_regs->otp[index];
    }

    for (uint32_t index = 0; index < 32; index++)
    {
        rt_cde_regs->otp[index] = rand;
        expected                = old_val[index] | rand;
        if (rt_cde_regs->otp[index] != expected)
        {
            LOG_ERROR("expected value 0x%08" PRIx32 " but got 0x%08" PRIx32 "",
                      expected,
                      rt_cde_regs->otp[index]);
            return E_VERFAIL;
        }
    }

    return SUCCESS;
}

pufs_status_t
pufs_rt_cde_rolck_test(void)
{
    pufs_status_t   check;
    pufs_otp_lock_t lock;
    uint32_t        value, cde_index = 0, start_offset = 256, length = 256; // (2 segment)

    if ((check = rt_cde_write_lock(start_offset, 256, RO)) != SUCCESS)
    {
        return check;
    }

    for (uint32_t offset = start_offset; offset < (start_offset + length); offset += 4)
    {
        lock = rt_cde_read_lock(offset);
        if (lock != RO)
        {
            LOG_ERROR("expected value %d but got %d", RO, lock);
            return E_VERFAIL;
        }
        cde_index = offset / WORD_SIZE;
        value     = rt_cde_regs->otp[cde_index];
        rt_cde_regs->otp[cde_index] |= 0X0C0C0C0C;
        if (rt_cde_regs->otp[cde_index] != value)
        {
            LOG_ERROR("expected value 0x%08" PRIx32 " but got 0x%08" PRIx32 "",
                      value,
                      rt_cde_regs->otp[cde_index]);
            return E_VERFAIL;
        }
    }
    return SUCCESS;
}

pufs_status_t
pufs_rt_cde_psmsk_test(void)
{
    uint32_t start_index = 128;
    rt_cde_write_mask(128 * WORD_SIZE);

    for (uint32_t index = start_index; index < start_index + 32; index++)
    {
        if (rt_cde_regs->otp[index] != 0xDEADDEAD)
        {
            LOG_ERROR("%s", "expected the OTP value cannot be read");
            return E_VERFAIL;
        }
    }
    return SUCCESS;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
