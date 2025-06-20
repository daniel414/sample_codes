/**
 * @file pufs_rt_factory_test.c
 * @brief RT API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_rt_factory_test.h"
#include "pufs_log.h"
#include "pufs_rt.h"
#include "pufs_rt_internal.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_rt_uids_read_test(void)
{
    pufs_status_t check;
    pufs_uid_st   uid;

    rt_write_enroll();

    for (pufs_rt_slot_t slot = PUFSLOT_0; slot < PUFSLOT_3; slot++)
    {
        if ((check = pufs_get_uid(&uid, slot)) != SUCCESS)
        {
            return check;
        }

        uint32_t *uid_word = (uint32_t *)uid.uid;
        for (uint32_t j = 0; j < UIDLEN / WORD_SIZE; j++)
        {
            if (*uid_word == PUFRT_VALUE32(0x0))
            {
                return E_VERFAIL;
            }
        }
    }

    if ((check = rt_write_set_flag(TMLCK_FLAG, 0x0)) != SUCCESS)
    {
        return check;
    }

    for (pufs_rt_slot_t slot = PUFSLOT_0; slot < PUFSLOT_3; slot++)
    {
        if ((check = pufs_get_uid(&uid, slot)) != SUCCESS)
        {
            return check;
        }

        uint32_t *uid_word = (uint32_t *)uid.uid;
        for (uint32_t j = 0; j < UIDLEN / WORD_SIZE; j++)
        {
            if ((slot == PUFSLOT_0 && *uid_word == PUFRT_VALUE32(0x0)) ||
                (slot != PUFSLOT_0 && *uid_word != PUFRT_VALUE32(0x0)))
            {
                return E_VERFAIL;
            }
        }
    }

    return SUCCESS;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
