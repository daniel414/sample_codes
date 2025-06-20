/**
 * @file pufs_rt_user_test.c
 * @brief RT API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_rt_user_test.h"
#include "pufs_log.h"
#include "pufs_rt.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/
static pufs_status_t pufs_set_otp_rwlck(pufs_otp_lock_t lock);
static pufs_status_t rt_otp_read_write_test(void);

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_rt_otp_rwlck_test(void)
{
    uint8_t       otp1[OTP_LEN], otp2[OTP_LEN];
    pufs_status_t check;

    // Check RW access
    if ((check = pufs_set_otp_rwlck(RW)) != SUCCESS)
    {
        printf("failed to set OTP RW\n");
        return check;
    }

    if ((check = rt_otp_read_write_test()) != SUCCESS)
    {
        printf("failed to pass OTP read write test\n");
        return check;
    }

    // Read OTP values
    if ((check = pufs_read_otp(otp1, OTP_LEN, 0)) != SUCCESS)
    {
        printf("failed to read OTP value\n");
        return check;
    }

    // Set RO access
    if ((check = pufs_set_otp_rwlck(RO)) != SUCCESS)
    {
        printf("failed to set OTP RO\n");
        return check;
    }

    // Write OTP to zeroes (success is not expected due to RO access)
    memset(otp2, PUFRT_VALUE8(0), OTP_LEN);
    pufs_program_otp(otp2, OTP_LEN, 0);

    // Read OTP values
    if ((check = pufs_read_otp(otp2, OTP_LEN, 0)) != SUCCESS)
    {
        printf("failed to read OTP value\n");
        return check;
    }

    if (memcmp(otp1, otp2, OTP_LEN) != 0)
    {
        printf("RO OTP test failed\n");
        return E_VERFAIL;
    }

    // Set NA access
    if ((check = pufs_set_otp_rwlck(NA)) != SUCCESS)
    {
        printf("failed to set OTP NA\n");
        return check;
    }

    // Read OTP values
    if ((check = pufs_read_otp(otp1, OTP_LEN, 0)) != SUCCESS)
    {
        printf("failed to read OTP value\n");
        return check;
    }

    // Check NA access functionality
    memset(otp2, PUFRT_VALUE8(0), OTP_LEN);

    if (memcmp(otp1, otp2, OTP_LEN) != 0)
    {
        printf("NA OTP test failed\n");
        return E_VERFAIL;
    }

    return SUCCESS;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/
static pufs_status_t
pufs_set_otp_rwlck(pufs_otp_lock_t lock)
{
    pufs_status_t check;

    // Set OTP rwlck test on OTP words
    for (pufs_otp_addr_t i = 0; i < OTP_LEN; i += 4)
    {
        // Set OTP rwlck
        if ((check = pufs_lock_otp(i, 4, lock)) != SUCCESS)
        {
            return check;
        }

        // Read OTP rwlck and check
        if (pufs_get_otp_rwlck(i) != lock)
        {
            return E_DENY;
        }
    }

    return SUCCESS;
}

static pufs_status_t
rt_otp_read_write_test(void)
{
    pufs_status_t check;
    uint32_t      r[OTP_LEN / 4], w[OTP_LEN / 4], n[OTP_LEN / 4];
    memset(w, PUFRT_VALUE8(0xff), OTP_LEN);

    // Read old OTP value
    if ((check = pufs_read_otp((uint8_t *)r, OTP_LEN, 0)) != SUCCESS)
    {
        return check;
    }

    // Randomly choose a test value
    for (int i = OTP_LEN / 4 - 1; i >= 0; i--)
    {
        // Find the one which is not all zeros for OTP read/write test
        if (r[i] == PUFRT_VALUE32(0))
        {
            continue;
        }

        // Generate test pattern
        if ((check = pufs_rand((uint8_t *)&(w[i]), 1)) != SUCCESS)
        {
            return check;
        }
    }

    // Write and read test
    if ((check = pufs_program_otp((const uint8_t *)w, OTP_LEN, 0)) != SUCCESS)
    {
        return check;
    }
    if ((check = pufs_read_otp((uint8_t *)n, OTP_LEN, 0)) != SUCCESS)
    {
        return check;
    }
    for (int i = OTP_LEN / 4 - 1; i >= 0; i--)
    {
        if (PUFRT_VALUE32(n[i]) != (PUFRT_VALUE32(r[i]) & PUFRT_VALUE32(w[i])))
        {
            return E_VERFAIL;
        }
    }

    return SUCCESS;
}

/*** end of file ***/
