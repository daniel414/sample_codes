/**
 * @file pufs_cmac_test.h
 * @brief CMAC API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_CMAC_TEST_H
#define PUFS_CMAC_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_test_common.h"

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
pufs_status_t pufs_cmac_test(void);
pufs_status_t pufs_cmac_swkey_test(void);
pufs_status_t pufs_cmac_multi_ctx_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_CMAC_TEST_H */

/*** end of file ***/
