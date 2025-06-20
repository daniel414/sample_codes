/**
 * @file pufs_kdf_test.h
 * @brief KDF API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_KDF_TEST_H
#define PUFS_KDF_TEST_H

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
pufs_status_t pufs_hkdf2_test(void);
pufs_status_t pufs_hkdf2_swkey_test(void);
pufs_status_t pufs_pbkdf_test(void);
pufs_status_t pufs_pbkdf_swkey_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_KDF_TEST_H */

/*** end of file ***/
