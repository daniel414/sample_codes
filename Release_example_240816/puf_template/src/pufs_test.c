/**
 * @file pufs_test.c
 * @brief pufs test flow
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_test.h"
#include "pufs_hmac_test.h"
#include "pufs_sp38a_test.h"
#include "pufs_ecp_test.h"
#include "pufs_rt_cde_test.h"
#include "pufs_rt_factory_test.h"
#include "pufs_rt_user_test.h"
#include "pufs_ka_test.h"
#include "pufs_dma_test.h"
#include "pufs_cmac_test.h"
#include "pufs_sp90a_test.h"
#include "pufs_kdf_test.h"
#include "pufs_sp38e_test.h"
#include "pufs_sp38c_test.h"
#include "pufs_sp38d_test.h"
#include "pufs_sm2_test.h"
#include "pufs_chacha_test.h"

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
        check = pufs_aes_cbc_enc_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_cbc_enc_test ok \n");
        }
        check = pufs_aes_cbc_enc_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_cbc_enc_swkey_test ok \n");
        }
        check = pufs_aes_cbc_enc_multi_ctx_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_cbc_enc_multi_ctx_test ok \n");
        }
        check = pufs_aes_cbc_dec_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_cbc_dec_test ok \n");
        }
        check = pufs_aes_cbc_dec_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_cbc_dec_swkey_test ok \n");
        }
        check = pufs_aes_cbc_dec_multi_ctx_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_cbc_dec_multi_ctx_test ok \n");
        }
        check = pufs_ecp_ecpuk_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_ecp_ecpuk_test ok \n");
        }
        check = pufs_ecp_ecpkv_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_ecp_ecpkv_test ok \n");
        }
        check = pufs_ecp_eprk_gen_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_ecp_eprk_gen_test ok \n");
        }
        ////// rt test is irreversible.
        ////// rt test is irreversible.
        ////// rt test is irreversible.
        // check = pufs_rt_cde_read_write_test();
        // if (SUCCESS == check)
        // {
        // 	print((const char *)"	pufs_rt_cde_read_write_test ok \n");
        // }
        // check = pufs_rt_cde_rolck_test();
        // if (SUCCESS == check)
        // {
        // 	print((const char *)"	pufs_rt_cde_rolck_test ok \n");
        // }
        // check = pufs_rt_cde_psmsk_test();
        // if (SUCCESS == check)
        // {
        // 	print((const char *)"	pufs_rt_cde_psmsk_test ok \n");
        // }
        // check = pufs_rt_uids_read_test();
        // if (SUCCESS == check)
        // {
        // 	print((const char *)"	pufs_rt_uids_read_test ok \n");
        // }
        // check = pufs_rt_otp_rwlck_test();
        // if (SUCCESS == check)
        // {
        // 	print((const char *)"	pufs_rt_otp_rwlck_test ok \n");
        // }
        check = pufs_aes_kw_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_kw_test ok \n");
        }
        check = pufs_aes_kw_inv_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_kw_inv_test ok \n");
        }
        check = pufs_aes_kwp_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_kwp_test ok \n");
        }
        check = pufs_aes_kwp_inv_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_kwp_inv_test ok \n");
        }
        check = pufs_dma_entropy_read_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_dma_entropy_read_test ok \n");
        }
        check = pufs_dma_random_read_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_dma_random_read_test ok \n");
        }
        check = pufs_ecp_ecdsa_verify_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_ecp_ecdsa_verify_test ok \n");
        }
        check = pufs_ecp_ecdsa_sign_verify_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_ecp_ecdsa_sign_verify_test ok \n");
        }
        check = pufs_ecp_ecccdh_2e_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_ecp_ecccdh_2e_test ok \n");
        }
        check = pufs_ecp_ecccdh_2e2s_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_ecp_ecccdh_2e2s_test ok \n");
        }
        check = pufs_ecp_ecccdh_oss_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_ecp_ecccdh_oss_test ok \n");
        }
        check = pufs_cmac_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_cmac_test ok \n");
        }
        check = pufs_cmac_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_cmac_swkey_test ok \n");
        }
        check = pufs_cmac_multi_ctx_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_cmac_multi_ctx_test ok \n");
        }
        check = pufs_drbg_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_drbg_test ok \n");
        }
        check = pufs_hkdf2_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_hkdf2_test ok \n");
        }
        check = pufs_hkdf2_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_hkdf2_swkey_test ok \n");
        }
        check = pufs_pbkdf_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_pbkdf_test ok \n");
        }
        check = pufs_pbkdf_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_pbkdf_swkey_test ok \n");
        }
        check = pufs_aes_xts_enc_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_xts_enc_test ok \n");
        }
        check = pufs_aes_xts_enc_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_xts_enc_swkey_test ok \n");
        }
        check = pufs_aes_xts_enc_multi_ctx_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_xts_enc_multi_ctx_test ok \n");
        }
        check = pufs_aes_xts_dec_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_xts_dec_test ok \n");
        }
        check = pufs_aes_xts_dec_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_xts_dec_swkey_test ok \n");
        }
        check = pufs_aes_xts_dec_multi_ctx_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_xts_dec_multi_ctx_test ok \n");
        }
        check = pufs_aes_ccm_enc_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_ccm_enc_test ok \n");
        }
        check = pufs_aes_ccm_enc_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_ccm_enc_swkey_test ok \n");
        }
        check = pufs_aes_ccm_enc_multi_ctx_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_ccm_enc_multi_ctx_test ok \n");
        }
        check = pufs_aes_ccm_dec_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_ccm_dec_test ok \n");
        }
        check = pufs_aes_ccm_dec_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_ccm_dec_swkey_test ok \n");
        }
        check = pufs_aes_ccm_dec_multi_ctx_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_ccm_dec_multi_ctx_test ok \n");
        }
        check = pufs_aes_gcm_enc_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_gcm_enc_test ok \n");
        }
        check = pufs_aes_gcm_enc_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_gcm_enc_swkey_test ok \n");
        }
        check = pufs_aes_gcm_enc_multi_ctx_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_gcm_enc_multi_ctx_test ok \n");
        }
        check = pufs_aes_gcm_dec_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_gcm_dec_test ok \n");
        }
        check = pufs_aes_gcm_dec_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_gcm_dec_swkey_test ok \n");
        }
        check = pufs_aes_gcm_dec_multi_ctx_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_aes_gcm_dec_multi_ctx_test ok \n");
        }
        check = pufs_sm2_enc_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_sm2_enc_test ok \n");
        }
        check = pufs_sm2_dec_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_sm2_dec_test ok \n");
        }
        check = pufs_sm2_kex_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_sm2_kex_test ok \n");
        }
        check = pufs_sm2_sign_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_sm2_sign_test ok \n");
        }
        check = pufs_sm2_verify_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_sm2_verify_test ok \n");
        }
        check = pufs_rsa_sign_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_rsa_sign_test ok \n");
        }
        check = pufs_rsa_verify_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_rsa_verify_test ok \n");
        }
        check = pufs_chacha_enc_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_chacha_enc_test ok \n");
        }
        check = pufs_chacha_enc_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_chacha_enc_swkey_test ok \n");
        }
        check = pufs_chacha20_poly1305_enc_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_chacha20_poly1305_enc_test ok \n");
        }
        check = pufs_chacha20_poly1305_enc_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_chacha20_poly1305_enc_swkey_test ok \n");
        }
        check = pufs_chacha_dec_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_chacha_dec_test ok \n");
        }
        check = pufs_chacha_dec_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_chacha_dec_swkey_test ok \n");
        }
        check = pufs_chacha20_poly1305_dec_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_chacha20_poly1305_dec_test ok \n");
        }
        check = pufs_chacha20_poly1305_dec_swkey_test();
        if (SUCCESS == check)
        {
        	print((const char *)"	pufs_chacha20_poly1305_dec_swkey_test ok \n");
        }
    }

    return;
}

/*** end of file ***/
