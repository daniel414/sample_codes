/**
 * @file pufs_sp38a_test.h
 * @brief SP38A API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_SP38A_TEST_H
#define PUFS_SP38A_TEST_H

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
pufs_status_t pufs_aes_cbc_enc_test(void);
pufs_status_t pufs_aes_cbc_enc_swkey_test(void);
pufs_status_t pufs_aes_cbc_enc_multi_ctx_test(void);
pufs_status_t pufs_aes_cbc_dec_test(void);
pufs_status_t pufs_aes_cbc_dec_swkey_test(void);
pufs_status_t pufs_aes_cbc_dec_multi_ctx_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_SP38A_TEST_H */

/*** end of file ***/
