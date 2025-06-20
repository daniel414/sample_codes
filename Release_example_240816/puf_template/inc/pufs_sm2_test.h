/**
 * @file pufs_sm2_test.h
 * @brief SM2 API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_SM2_TEST_H
#define PUFS_SM2_TEST_H

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
pufs_status_t pufs_sm2_enc_test(void);
pufs_status_t pufs_sm2_dec_test(void);
pufs_status_t pufs_sm2_kex_test(void);
pufs_status_t pufs_sm2_sign_test(void);
pufs_status_t pufs_sm2_verify_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_SM2_TEST_H */

/*** end of file ***/
