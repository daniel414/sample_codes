/**
 * @file pufs_chacha_test.c
 * @brief ChaCha API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>

#include "pufs_chacha_test.h"
#include "pufs_chacha.h"
#include "pufs_log.h"
#include "pufs_tv_chacha20.h"
#include "pufs_tv_chacha20-poly1305.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_chacha_enc_test(void)
{
    pufs_status_t  check  = SUCCESS;
    uint8_t       *out    = NULL;
    const uint32_t ntests = sizeof(chacha20_tp) / sizeof(struct chacha20_test_pattern);
    const struct chacha20_test_pattern *tp = chacha20_tp;

    pufs_chacha_ctx *ctx = pufs_chacha_ctx_new();
    if (ctx == NULL)
    {
        return E_ERROR;
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        out                    = malloc(tp[i].ctlen);
        uint32_t       outlen  = 0;
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;

        pufs_chacha_params_st params = {
            .counter = tp[i].ibc,
            .keyaddr = keyslot,
            .iv      = tp[i].nonce,
            .ivlen   = tp[i].noncelen,
            .keybits = tp[i].keybits,
            .round   = 20,
            .keytype = SSKEY,
        };

        if ((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
            SUCCESS)
        {
            goto cleanup;
        }

        if ((check = pufs_chacha_enc(out, &outlen, &params, tp[i].pt, tp[i].ptlen)) != SUCCESS)
        {
            goto cleanup;
        }

        if (outlen != tp[i].ctlen || memcmp(out, tp[i].ct, tp[i].ctlen) != 0)
        {
            check = E_VERFAIL;
        }

    cleanup:
        pufs_clear_key(SSKEY, keyslot, tp[i].keybits);
        free(out);

        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }

    pufs_chacha_ctx_free(ctx);

    return check;
}

pufs_status_t
pufs_chacha_enc_swkey_test(void)
{
    pufs_status_t  check  = SUCCESS;
    uint8_t       *out    = NULL;
    const uint32_t ntests = sizeof(chacha20_tp) / sizeof(struct chacha20_test_pattern);
    const struct chacha20_test_pattern *tp = chacha20_tp;

    pufs_chacha_ctx *ctx = pufs_chacha_ctx_new();
    if (ctx == NULL)
    {
        return E_ERROR;
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        out             = malloc(tp[i].ctlen);
        uint32_t outlen = 0;

        pufs_chacha_params_st params = {
            .counter = tp[i].ibc,
            .keyaddr = (uintptr_t)tp[i].key,
            .iv      = tp[i].nonce,
            .ivlen   = tp[i].noncelen,
            .keybits = tp[i].keybits,
            .round   = 20,
            .keytype = SWKEY,
        };

        if ((check = pufs_chacha_enc(out, &outlen, &params, tp[i].pt, tp[i].ptlen)) != SUCCESS)
        {
            goto cleanup;
        }

        if (outlen != tp[i].ctlen || memcmp(out, tp[i].ct, tp[i].ctlen) != 0)
        {
            check = E_VERFAIL;
        }

    cleanup:
        free(out);

        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }

    pufs_chacha_ctx_free(ctx);

    return check;
}

pufs_status_t
pufs_chacha20_poly1305_enc_test(void)
{
    pufs_status_t  check = SUCCESS;
    uint8_t       *out   = NULL;
    uint8_t        tag[16];
    const uint32_t ntests =
        sizeof(chacha20_poly1305_tp) / sizeof(struct chacha20_poly1305_test_pattern);
    const struct chacha20_poly1305_test_pattern *tp = chacha20_poly1305_tp;

    pufs_chacha_ctx *ctx = pufs_chacha_ctx_new();
    if (ctx == NULL)
    {
        return E_ERROR;
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        out = malloc(tp[i].ctlen);
        memset(tag, 0x0, 16);
        uint32_t       outlen  = 0;
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;

        pufs_chacha20_poly1305_params_st params = {
            .keyaddr = keyslot,
            .iv      = tp[i].nonce,
            .ivlen   = tp[i].noncelen,
            .keybits = tp[i].keybits,
            .keytype = SSKEY,
        };

        if ((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
                SUCCESS ||
            (check = pufs_chacha20_poly1305_enc(
                 out, &outlen, tag, &params, tp[i].aad, tp[i].aadlen, tp[i].pt, tp[i].ptlen)) !=
                SUCCESS)
        {
            goto cleanup;
        }

        if (outlen != tp[i].ctlen || memcmp(out, tp[i].ct, tp[i].ctlen) != 0)
        {
            LOG_ERROR("test case %" PRIu32 ": encrypted data is not matched with expected result.",
                      i);
            check = E_VERFAIL;
        }

        if (memcmp(tag, tp[i].tag, tp[i].taglen) != 0)
        {
            LOG_ERROR("test case %" PRIu32 ": tag is not matched with expected tag.", i);
            check = E_VERFAIL;
        }
    cleanup:
        pufs_clear_key(SSKEY, keyslot, tp[i].keybits);
        free(out);

        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }

    pufs_chacha_ctx_free(ctx);

    return check;
}

pufs_status_t
pufs_chacha20_poly1305_enc_swkey_test(void)
{
    pufs_status_t  check = SUCCESS;
    uint8_t       *out   = NULL;
    uint8_t        tag[16];
    const uint32_t ntests =
        sizeof(chacha20_poly1305_tp) / sizeof(struct chacha20_poly1305_test_pattern);
    const struct chacha20_poly1305_test_pattern *tp = chacha20_poly1305_tp;

    pufs_chacha_ctx *ctx = pufs_chacha_ctx_new();
    if (ctx == NULL)
    {
        return E_ERROR;
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        out = malloc(tp[i].ctlen);
        memset(tag, 0x0, 16);
        uint32_t outlen = 0;

        pufs_chacha20_poly1305_params_st params = {
            .keyaddr = (uintptr_t)tp[i].key,
            .iv      = tp[i].nonce,
            .ivlen   = tp[i].noncelen,
            .keybits = tp[i].keybits,
            .keytype = SWKEY,
        };

        if ((check = pufs_chacha20_poly1305_enc(
                 out, &outlen, tag, &params, tp[i].aad, tp[i].aadlen, tp[i].pt, tp[i].ptlen)) !=
            SUCCESS)
        {
            goto cleanup;
        }

        if (outlen != tp[i].ctlen || memcmp(out, tp[i].ct, tp[i].ctlen) != 0)
        {
            LOG_ERROR("test case %" PRIu32 ": encrypted data is not matched with expected result.",
                      i);
            check = E_VERFAIL;
        }

        if (memcmp(tag, tp[i].tag, tp[i].taglen) != 0)
        {
            LOG_ERROR("test case %" PRIu32 ": tag is not matched with expected tag.", i);
            check = E_VERFAIL;
        }
    cleanup:
        free(out);

        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }

    pufs_chacha_ctx_free(ctx);

    return check;
}

pufs_status_t
pufs_chacha_dec_test(void)
{
    pufs_status_t  check  = SUCCESS;
    uint8_t       *out    = NULL;
    const uint32_t ntests = sizeof(chacha20_tp) / sizeof(struct chacha20_test_pattern);
    const struct chacha20_test_pattern *tp = chacha20_tp;

    pufs_chacha_ctx *ctx = pufs_chacha_ctx_new();
    if (ctx == NULL)
    {
        return E_ERROR;
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        out                    = malloc(tp[i].ptlen);
        uint32_t       outlen  = 0;
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;

        pufs_chacha_params_st params = {
            .counter = tp[i].ibc,
            .keyaddr = keyslot,
            .iv      = tp[i].nonce,
            .ivlen   = tp[i].noncelen,
            .keybits = tp[i].keybits,
            .round   = 20,
            .keytype = SSKEY,
        };

        if ((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
            SUCCESS)
        {
            goto cleanup;
        }

        if ((check = pufs_chacha_dec(out, &outlen, &params, tp[i].ct, tp[i].ctlen)) != SUCCESS)
        {
            goto cleanup;
        }

        if (outlen != tp[i].ptlen || memcmp(out, tp[i].pt, tp[i].ptlen) != 0)
        {
            LOG_ERROR("test case %" PRIu32 ": encrypted data is not matched with expected result.",
                      i + 1);
            check = E_VERFAIL;
        }
    cleanup:
        pufs_clear_key(SSKEY, keyslot, tp[i].keybits);
        free(out);
        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }

    pufs_chacha_ctx_free(ctx);

    return check;
}

pufs_status_t
pufs_chacha_dec_swkey_test(void)
{
    pufs_status_t  check  = SUCCESS;
    uint8_t       *out    = NULL;
    const uint32_t ntests = sizeof(chacha20_tp) / sizeof(struct chacha20_test_pattern);
    const struct chacha20_test_pattern *tp = chacha20_tp;

    pufs_chacha_ctx *ctx = pufs_chacha_ctx_new();
    if (ctx == NULL)
    {
        return E_ERROR;
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        out             = malloc(tp[i].ptlen);
        uint32_t outlen = 0;

        pufs_chacha_params_st params = {
            .counter = tp[i].ibc,
            .keyaddr = (uintptr_t)tp[i].key,
            .iv      = tp[i].nonce,
            .ivlen   = tp[i].noncelen,
            .keybits = tp[i].keybits,
            .round   = 20,
            .keytype = SWKEY,
        };

        if ((check = pufs_chacha_dec(out, &outlen, &params, tp[i].ct, tp[i].ctlen)) != SUCCESS)
        {
            goto cleanup;
        }

        if (outlen != tp[i].ptlen || memcmp(out, tp[i].pt, tp[i].ptlen) != 0)
        {
            check = E_VERFAIL;
        }

    cleanup:
        free(out);

        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }

    pufs_chacha_ctx_free(ctx);

    return check;
}

pufs_status_t
pufs_chacha20_poly1305_dec_test(void)
{
    pufs_status_t  check = SUCCESS;
    uint8_t       *out   = NULL;
    const uint32_t ntests =
        sizeof(chacha20_poly1305_tp) / sizeof(struct chacha20_poly1305_test_pattern);
    const struct chacha20_poly1305_test_pattern *tp = chacha20_poly1305_tp;

    pufs_chacha_ctx *ctx = pufs_chacha_ctx_new();
    if (ctx == NULL)
    {
        return E_ERROR;
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        out                   = malloc(tp[i].ptlen);
        uint32_t       outlen = 0, toutlen = 0;
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;

        pufs_chacha20_poly1305_params_st params = {
            .keyaddr = keyslot,
            .iv      = tp[i].nonce,
            .ivlen   = tp[i].noncelen,
            .keybits = tp[i].keybits,
            .keytype = SSKEY,
        };

        if ((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
                SUCCESS ||
            (check = pufs_chacha20_poly1305_dec(out,
                                                &outlen,
                                                &params,
                                                tp[i].aad,
                                                tp[i].aadlen,
                                                tp[i].ct,
                                                tp[i].ctlen,
                                                tp[i].tag)) != SUCCESS)
        {
            goto cleanup;
        }

        outlen += toutlen;

        if (outlen != tp[i].ptlen || memcmp(out, tp[i].pt, tp[i].ptlen) != 0)
        {
            check = E_VERFAIL;
        }

    cleanup:
        pufs_clear_key(SSKEY, keyslot, tp[i].keybits);
        free(out);

        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }

    pufs_chacha_ctx_free(ctx);

    return check;
}

pufs_status_t
pufs_chacha20_poly1305_dec_swkey_test(void)
{
    pufs_status_t  check = SUCCESS;
    uint8_t       *out   = NULL;
    const uint32_t ntests =
        sizeof(chacha20_poly1305_tp) / sizeof(struct chacha20_poly1305_test_pattern);
    const struct chacha20_poly1305_test_pattern *tp = chacha20_poly1305_tp;

    pufs_chacha_ctx *ctx = pufs_chacha_ctx_new();
    if (ctx == NULL)
    {
        return E_ERROR;
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        out             = malloc(tp[i].ptlen);
        uint32_t outlen = 0, toutlen = 0;

        pufs_chacha20_poly1305_params_st params = {
            .keyaddr = (uintptr_t)tp[i].key,
            .iv      = tp[i].nonce,
            .ivlen   = tp[i].noncelen,
            .keybits = tp[i].keybits,
            .keytype = SWKEY,
        };

        if ((check = pufs_chacha20_poly1305_dec(out,
                                                &outlen,
                                                &params,
                                                tp[i].aad,
                                                tp[i].aadlen,
                                                tp[i].ct,
                                                tp[i].ctlen,
                                                tp[i].tag)) != SUCCESS)
        {
            goto cleanup;
        }

        outlen += toutlen;

        if (outlen != tp[i].ptlen || memcmp(out, tp[i].pt, tp[i].ptlen) != 0)
        {
            check = E_VERFAIL;
        }

    cleanup:
        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }

    pufs_chacha_ctx_free(ctx);
    free(out);

    return check;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
