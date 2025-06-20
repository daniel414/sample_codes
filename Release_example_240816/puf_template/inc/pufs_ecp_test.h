/**
 * @file pufs_ecp_test.h
 * @brief ECP API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_ECP_TEST_H
#define PUFS_ECP_TEST_H

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
pufs_status_t pufs_ecp_ecpuk_test(void);
pufs_status_t pufs_ecp_ecpkv_test(void);
pufs_status_t pufs_ecp_eprk_gen_test(void);
pufs_status_t pufs_ecp_ecdsa_verify_test(void);
pufs_status_t pufs_ecp_ecdsa_sign_verify_test(void);
pufs_status_t pufs_ecp_ecccdh_2e_test(void);
pufs_status_t pufs_ecp_ecccdh_2e2s_test(void);
pufs_status_t pufs_ecp_ecccdh_oss_test(void);
pufs_status_t pufs_rsa_sign_test(void);
pufs_status_t pufs_rsa_verify_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_ECP_TEST_H */

/*** end of file ***/
