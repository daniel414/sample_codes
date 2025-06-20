/**
 * @file pufs_sp38d_test.h
 * @brief SP38D API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_SP38D_TEST_H
#define PUFS_SP38D_TEST_H

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
pufs_status_t pufs_aes_gcm_enc_test(void);
pufs_status_t pufs_aes_gcm_enc_swkey_test(void);
pufs_status_t pufs_aes_gcm_enc_multi_ctx_test(void);
pufs_status_t pufs_aes_gcm_dec_test(void);
pufs_status_t pufs_aes_gcm_dec_swkey_test(void);
pufs_status_t pufs_aes_gcm_dec_multi_ctx_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_SP38D_TEST_H */

/*** end of file ***/
