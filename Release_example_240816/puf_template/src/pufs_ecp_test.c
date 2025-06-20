/**
 * @file pufs_ecp_test.c
 * @brief ECP API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_ecp_test.h"
#include "pufs_ecc.h"
#include "pufs_ecp.h"
#include "pufs_kdf.h"
#include "pufs_log.h"
#include "pufs_tv_ecccdh.h"
#include "pufs_tv_ecdsas.h"
#include "pufs_tv_ecdsav.h"
#include "pufs_tv_ecpkv.h"
#include "pufs_tv_ecpuk.h"
#include "pufs_tv_rsaexp.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_ecp_ecpuk_test(void)
{
    const uint32_t                   ntests = sizeof(ecpuk_tp) / sizeof(struct ecpuk_test_pattern);
    const struct ecpuk_test_pattern *tp     = ecpuk_tp;
    pufs_status_t                    check  = SUCCESS;

    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_ec_point_st puk;
        if (((check = pufs_import_plaintext_key(
                  PRKEY, PRK_0, tp[i].prk, g_ecc_param[tp[i].name].fbits)) != SUCCESS) ||
            ((check = pufs_ecp_set_curve_byname(tp[i].name)) != SUCCESS) ||
            ((check = pufs_ecp_gen_puk(&puk, PRKEY, PRK_0)) != SUCCESS) ||
            ((check = pufs_clear_key(PRKEY, PRK_0, g_ecc_param[tp[i].name].fbits)) != SUCCESS) ||
            ((check = pufs_test_memcmp(puk.x, tp[i].Qx, puk.qlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
    }
    return check;
}

pufs_status_t
pufs_ecp_ecpkv_test(void)
{
    const uint32_t                   ntests = sizeof(ecpkv_tp) / sizeof(struct ecpkv_test_pattern);
    const struct ecpkv_test_pattern *tp     = ecpkv_tp;
    pufs_status_t                    check  = SUCCESS;

    pufs_ec_point_st puk;
    for (uint32_t i = 0; i < ntests; i++)
    {
        puk.qlen = g_ecc_param[tp[i].name].len;
        memcpy(puk.x, tp[i].Qx, puk.qlen);
        memcpy(puk.y, tp[i].Qy, puk.qlen);
        if ((check = pufs_ecp_set_curve_byname(tp[i].name)) != SUCCESS)
        {
            goto cleanup;
        }

        check = pufs_ecp_validate_puk(puk, true);
        if (check != SUCCESS && check != E_VERFAIL)
        {
            goto cleanup;
        }

        if ((tp[i].valid == true && check == SUCCESS) ||
            (tp[i].valid == false && check == E_VERFAIL))
        {
            check = SUCCESS;
        }
        else
        {
            check = E_VERFAIL;
        }

    cleanup:
        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
    }
    return check;
}

pufs_status_t
pufs_ecp_eprk_gen_test(void)
{
    pufs_status_t    check = SUCCESS;
    pufs_ec_point_st puk;

    // Iterate through all possible curve and do eprk_gen
    for (pufs_ec_name_t curve = NISTB163; curve < N_ECNAME_T; curve++)
    {
        if (curve == SM2)
        {
            continue; // SM2 has no eprk_gen mprogm
        }

        if (((check = pufs_ecp_set_curve_byname(curve))) ||
            ((check = pufs_ecp_gen_eprk(PRK_0))) != SUCCESS ||
            ((check = pufs_ecp_gen_puk(&puk, PRKEY, PRK_0))) != SUCCESS ||
            ((check = pufs_clear_key(PRKEY, PRK_0, g_ecc_param[curve].fbits)) != SUCCESS))
        {
            LOG_ERROR("Case %" PRIu32 " Failed.", (uint32_t)curve);
            return check;
        }
    }
    return check;
}

pufs_status_t
pufs_ecp_ecdsa_verify_test(void)
{
    const uint32_t ntests                = sizeof(ecdsav_tp) / sizeof(struct ecdsav_test_pattern);
    const struct ecdsav_test_pattern *tp = ecdsav_tp;
    pufs_status_t                     check = SUCCESS;

    pufs_dgst_st      md;
    pufs_ec_point_st  puk;
    pufs_ecdsa_sig_st sig;
    for (uint32_t i = 0; i < ntests; i++)
    {
        md.dlen = g_ecc_param[tp[i].name].len;
        if (md.dlen > 64)
        {
            md.dlen = 64;
        }
        memcpy(md.dgst, tp[i].md, md.dlen);
        puk.qlen = g_ecc_param[tp[i].name].len;
        memcpy(puk.x, tp[i].Qx, puk.qlen);
        memcpy(puk.y, tp[i].Qy, puk.qlen);
        sig.qlen = g_ecc_param[tp[i].name].len;
        memcpy(sig.r, tp[i].r, sig.qlen);
        memcpy(sig.s, tp[i].s, sig.qlen);
        if ((check = pufs_ecp_set_curve_byname(tp[i].name)) != SUCCESS)
        {
            goto cleanup;
        }

        check = pufs_ecp_ecdsa_verify_dgst(sig, md, puk);
        if (check != SUCCESS && check != E_VERFAIL)
        {
            goto cleanup;
        }

        if ((tp[i].valid == true && check == SUCCESS) ||
            (tp[i].valid == false && check == E_VERFAIL))
        {
            check = SUCCESS;
        }
        else
        {
            check = E_VERFAIL;
        }

    cleanup:
        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
    }
    return check;
}

pufs_status_t
pufs_ecp_ecdsa_sign_verify_test(void)
{
    const uint32_t ntests                = sizeof(ecdsas_tp) / sizeof(struct ecdsas_test_pattern);
    const struct ecdsas_test_pattern *tp = ecdsas_tp;
    pufs_status_t                     check = SUCCESS;

    pufs_dgst_st      md;
    pufs_ec_point_st  puk;
    pufs_ecdsa_sig_st sig;
    for (uint32_t i = 0; i < ntests; i++)
    {
        md.dlen = g_ecc_param[tp[i].name].len;
        if (md.dlen > 64)
        {
            md.dlen = 64;
        }
        memcpy(md.dgst, tp[i].md, md.dlen);
        if (((check = pufs_ecp_set_curve_byname(tp[i].name)) != SUCCESS) ||
            ((check = pufs_import_plaintext_key(
                  PRKEY, PRK_0, tp[i].prk, g_ecc_param[tp[i].name].fbits)) != SUCCESS) ||
            ((check = pufs_ecp_gen_puk(&puk, PRKEY, PRK_0)) != SUCCESS) ||
            ((check = pufs_ecp_ecdsa_sign_dgst(&sig, md, PRKEY, PRK_0, NULL)) != SUCCESS) ||
            ((check = pufs_clear_key(PRKEY, PRK_0, g_ecc_param[tp[i].name].fbits)) != SUCCESS) ||
            ((check = pufs_ecp_ecdsa_verify_dgst(sig, md, puk)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
    }
    return check;
}

pufs_status_t
pufs_ecp_ecccdh_2e_test(void)
{
    const uint32_t ntests                = sizeof(ecccdh_tp) / sizeof(struct ecccdh_test_pattern);
    const struct ecccdh_test_pattern *tp = ecccdh_tp;
    pufs_status_t                     check = SUCCESS;

    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_ecp_set_curve_byname(tp[i].name)) != SUCCESS)
        {
            goto cleanup;
        }

        pufs_dgst_st     md1, md2;
        uint8_t          info[256], salt[64], kek[32], iv[16];
        pufs_ec_point_st puk0, puk1;
        pufs_rand(info, 64);
        pufs_rand(salt, 16);
        pufs_rand(kek, 8);
        pufs_rand(iv, 4);
        // ECC CDH -> KDF -> export
        if (((check = pufs_ecp_gen_eprk(PRK_0)) != SUCCESS) ||
            ((check = pufs_ecp_gen_eprk(PRK_1)) != SUCCESS) ||
            ((check = pufs_ecp_gen_puk(&puk0, PRKEY, PRK_0)) != SUCCESS) ||
            ((check = pufs_ecp_gen_puk(&puk1, PRKEY, PRK_1)) != SUCCESS) ||
            ((check = pufs_ecp_ecccdh_2e(puk1, PRK_0, NULL)) != SUCCESS) ||
            ((check = pufs_kdf(SSKEY,
                               SK512_0,
                               512,
                               PRF_HMAC,
                               SHA_256,
                               false,
                               NULL,
                               0,
                               4,
                               SHARESEC,
                               SHARESEC_0,
                               0,
                               salt,
                               64,
                               info,
                               256)) != SUCCESS) ||
            ((check = pufs_hmac(&md1, NULL, 0, SHA_256, SSKEY, SK512_0, 512)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, SK512_0, 512)) != SUCCESS) ||
            ((check = pufs_ecp_ecccdh_2e(puk0, PRK_1, NULL)) != SUCCESS) ||
            ((check = pufs_kdf(SSKEY,
                               SK512_1,
                               512,
                               PRF_HMAC,
                               SHA_256,
                               false,
                               NULL,
                               0,
                               4,
                               SHARESEC,
                               SHARESEC_0,
                               0,
                               salt,
                               64,
                               info,
                               256)) != SUCCESS) ||
            ((check = pufs_hmac(&md2, NULL, 0, SHA_256, SSKEY, SK512_1, 512)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, SK512_1, 512) != SUCCESS)) ||
            ((check = pufs_clear_key(PRKEY, PRK_0, g_ecc_param[tp[i].name].fbits)) != SUCCESS) ||
            ((check = pufs_clear_key(PRKEY, PRK_1, g_ecc_param[tp[i].name].fbits)) != SUCCESS) ||
            ((check = (md1.dlen == md2.dlen ? SUCCESS : E_VERFAIL)) != SUCCESS) ||
            ((check = pufs_test_memcmp(md1.dgst, md2.dgst, md1.dlen)) != SUCCESS))
        {
            goto cleanup;
        }
    cleanup:
        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
    }
    return check;
}

pufs_status_t
pufs_ecp_ecccdh_2e2s_test(void)
{
    const uint32_t ntests                = sizeof(ecccdh_tp) / sizeof(struct ecccdh_test_pattern);
    const struct ecccdh_test_pattern *tp = ecccdh_tp;
    pufs_status_t                     check = SUCCESS;

    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_ecp_set_curve_byname(tp[i].name)) != SUCCESS)
        {
            goto cleanup;
        }

        pufs_dgst_st     md1, md2;
        uint8_t          info[256], salt[64], kek[32], iv[16];
        pufs_ec_point_st puk0, puk1, puk2;
        pufs_rand(info, 64);
        pufs_rand(salt, 16);
        pufs_rand(kek, 8);
        pufs_rand(iv, 4);
        // ECC CDH -> KDF -> export
        if (((check = pufs_ecp_gen_eprk(PRK_0)) != SUCCESS) ||
            ((check = pufs_ecp_gen_eprk(PRK_1)) != SUCCESS) ||
            ((check = pufs_ecp_gen_eprk(PRK_2)) != SUCCESS) ||
            ((check = pufs_ecp_gen_puk(&puk0, PRKEY, PRK_0)) != SUCCESS) ||
            ((check = pufs_ecp_gen_puk(&puk1, PRKEY, PRK_1)) != SUCCESS) ||
            ((check = pufs_ecp_gen_puk(&puk2, PRKEY, PRK_2)) != SUCCESS) ||
            ((check = pufs_ecp_ecccdh_2e2s(puk1, puk2, PRK_0, PRKEY, PRK_1, NULL)) != SUCCESS) ||
            ((check = pufs_kdf(SSKEY,
                               SK512_0,
                               512,
                               PRF_HMAC,
                               SHA_256,
                               false,
                               NULL,
                               0,
                               4,
                               SHARESEC,
                               SHARESEC_0,
                               0,
                               salt,
                               64,
                               info,
                               256)) != SUCCESS) ||
            ((check = pufs_hmac(&md1, NULL, 0, SHA_256, SSKEY, SK512_0, 512)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, SK512_0, 512)) != SUCCESS) ||
            ((check = pufs_ecp_ecccdh_2e2s(puk0, puk1, PRK_1, PRKEY, PRK_2, NULL)) != SUCCESS) ||
            ((check = pufs_kdf(SSKEY,
                               SK512_1,
                               512,
                               PRF_HMAC,
                               SHA_256,
                               false,
                               NULL,
                               0,
                               4,
                               SHARESEC,
                               SHARESEC_0,
                               0,
                               salt,
                               64,
                               info,
                               256)) != SUCCESS) ||
            ((check = pufs_hmac(&md2, NULL, 0, SHA_256, SSKEY, SK512_1, 512)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, SK512_1, 512)) != SUCCESS) ||
            ((check = pufs_clear_key(PRKEY, PRK_0, g_ecc_param[tp[i].name].fbits)) != SUCCESS) ||
            ((check = pufs_clear_key(PRKEY, PRK_1, g_ecc_param[tp[i].name].fbits)) != SUCCESS) ||
            ((check = pufs_clear_key(PRKEY, PRK_2, g_ecc_param[tp[i].name].fbits)) != SUCCESS) ||
            ((check = (md1.dlen == md2.dlen ? SUCCESS : E_VERFAIL)) != SUCCESS) ||
            ((check = pufs_test_memcmp(md1.dgst, md2.dgst, md1.dlen)) != SUCCESS))
        {
            goto cleanup;
        }

    cleanup:
        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
    }
    return check;
}

pufs_status_t
pufs_ecp_ecccdh_oss_test(void)
{
    const uint32_t ntests                = sizeof(ecccdh_tp) / sizeof(struct ecccdh_test_pattern);
    const struct ecccdh_test_pattern *tp = ecccdh_tp;
    pufs_status_t                     check = SUCCESS;

    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_ecp_set_curve_byname(tp[i].name)) != SUCCESS)
        {
            goto cleanup;
        }

        uint8_t          out[72];
        pufs_ec_point_st puk;
        uint32_t         fbits = g_ecc_param[tp[i].name].fbits;
        puk.qlen               = g_ecc_param[tp[i].name].len;
        memcpy(puk.x, tp[i].Qx, puk.qlen);
        memcpy(puk.y, tp[i].Qy, puk.qlen);
        if (((check = pufs_import_plaintext_key(PRKEY, PRK_0, tp[i].prk, fbits)) != SUCCESS) ||
            ((check = pufs_ecp_ecccdh_2e(puk, PRK_0, out)) != SUCCESS) ||
            ((check = pufs_clear_key(PRKEY, PRK_0, fbits)) != SUCCESS) ||
            ((check = pufs_test_memcmp(out, tp[i].Z, puk.qlen)) != SUCCESS))
        {
            goto cleanup;
        }

    cleanup:
        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
    }
    return check;
}

pufs_status_t
pufs_rsa_sign_test(void)
{
    const uint32_t ntests = sizeof(rsa_exp_tp) / sizeof(struct rsa_exp_test_pattern);
    const struct rsa_exp_test_pattern *tp    = rsa_exp_tp;
    pufs_status_t                      check = SUCCESS;

    uint8_t out[512];
    for (uint32_t i = 0; i < ntests; i++)
    {
        uint32_t len = 0;
        switch (tp[i].rsatype)
        {
            case RSA1024:
                len = 128;
                break;
            case RSA2048:
                len = 256;
                break;
            case RSA3072:
                len = 384;
                break;
            case RSA4096:
                len = 512;
                break;
            default:
                return E_INVALID;
        }
        memset(out, 0x0, 512);
        if ((check = pufs_rsa_sign(out, tp[i].rsatype, tp[i].n, tp[i].e, tp[i].d, tp[i].m, NULL)) !=
                SUCCESS ||
            (check = pufs_test_memcmp(tp[i].s, out, len)) != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
    }
    return check;
}

pufs_status_t
pufs_rsa_verify_test(void)
{
    const uint32_t ntests = sizeof(rsa_exp_tp) / sizeof(struct rsa_exp_test_pattern);
    const struct rsa_exp_test_pattern *tp    = rsa_exp_tp;
    pufs_status_t                      check = SUCCESS;

    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_rsa_verify(tp[i].s, tp[i].rsatype, tp[i].n, tp[i].e, tp[i].m)) != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
    }
    return check;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
