/**
 * @file pufs_ka_test.h
 * @brief KA API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_KA_TEST_H
#define PUFS_KA_TEST_H

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
pufs_status_t pufs_aes_kw_test(void);
pufs_status_t pufs_aes_kw_inv_test(void);
pufs_status_t pufs_aes_kwp_test(void);
pufs_status_t pufs_aes_kwp_inv_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_KA_TEST_H */

/*** end of file ***/
