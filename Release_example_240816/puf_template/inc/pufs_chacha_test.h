/**
 * @file pufs_chacha_test.h
 * @brief ChaCha API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_CHACHA_TEST_H
#define PUFS_CHACHA_TEST_H

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
pufs_status_t pufs_chacha_enc_test(void);
pufs_status_t pufs_chacha_enc_swkey_test(void);
pufs_status_t pufs_chacha20_poly1305_enc_test(void);
pufs_status_t pufs_chacha20_poly1305_enc_swkey_test(void);
pufs_status_t pufs_chacha_dec_test(void);
pufs_status_t pufs_chacha_dec_swkey_test(void);
pufs_status_t pufs_chacha20_poly1305_dec_test(void);
pufs_status_t pufs_chacha20_poly1305_dec_swkey_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_CHACHA_TEST_H */

/*** end of file ***/
