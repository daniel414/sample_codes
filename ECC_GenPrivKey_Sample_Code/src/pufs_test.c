/**
 * @file pufs_test.c
 * @brief pufs test flow
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_hmac_test.h"
#include "pufs_test.h"

/*******************************************************************************
 * Global functions
 ******************************************************************************/
void
pufs_test(void)
{
    pufs_status_t check;

    check = pufs_initialize();
    if (SUCCESS == check)
    {
        check = pufs_sha2_iuf_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_sha2_iuf_test ok \n");
        }
        check = pufs_sha2_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_sha2_test ok \n");
        }
        check = pufs_hmac_sha2_iuf_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_hmac_sha2_iuf_test ok \n");
        }
        check = pufs_hmac_sha2_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_hmac_sha2_test ok \n");
        }
        check = pufs_hmac_sha2_swkey_iuf_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_hmac_sha2_swkey_iuf_test ok \n");
        }
        check = pufs_hmac_sha2_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_hmac_sha2_swkey_test ok \n");
        }
    }

    return;
}

/*** end of file ***/
