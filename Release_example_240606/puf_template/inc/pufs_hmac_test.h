/**
 * @file pufs_hmac_test.h
 * @brief HMAC API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_HMAC_TEST_H
#define PUFS_HMAC_TEST_H

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
pufs_status_t pufs_sha2_iuf_test(void);
pufs_status_t pufs_sha2_test(void);
pufs_status_t pufs_hmac_sha2_iuf_test(void);
pufs_status_t pufs_hmac_sha2_test(void);
pufs_status_t pufs_hmac_sha2_swkey_iuf_test(void);
pufs_status_t pufs_hmac_sha2_swkey_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_HMAC_TEST_H */

/*** end of file ***/
