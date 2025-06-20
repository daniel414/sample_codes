/**
 * @file pufs_dma_test.c
 * @brief DMA API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>

#include "pufs_dma_test.h"
#include "pufs_log.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/
static pufs_status_t pufs_dma_entropy_read(bool entropy);

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_dma_entropy_read_test(void)
{
    return pufs_dma_entropy_read(true);
}

pufs_status_t
pufs_dma_random_read_test(void)
{
    return pufs_dma_entropy_read(false);
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/
pufs_status_t pufs_dump_rand_dma(uint8_t *rand, uint32_t len, bool entropy);

static pufs_status_t
pufs_dma_entropy_read(bool entropy)
{
    pufs_status_t check = SUCCESS;
    uint8_t      *buff  = malloc(16);
    if ((check = pufs_dump_rand_dma(buff, 16, entropy)) != SUCCESS)
    {
        LOG_ERROR("Failed to read dma %s", entropy ? "entropy" : "random");
        goto cleanup;
    }

    check = E_VERFAIL;
    for (uint32_t i = 0; i < 16; i++)
    {
        if ((buff[i] != 0x0) && (buff[i] != 0xFF))
        {
            check = SUCCESS;
        }
    }
cleanup:
    free(buff);
    return check;
}

/*** end of file ***/
