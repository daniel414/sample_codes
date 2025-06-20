/**
 * @file pufs_sp38d_test.c
 * @brief SP38D API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>

#include "pufs_sp38d_test.h"
#include "pufs_ka.h"
#include "pufs_sp38d.h"
#include "pufs_log.h"
#include "pufs_tv_aesgcm.h"

static uint8_t tag[BC_BLOCK_SIZE];
/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_aes_gcm_enc_test(void)
{
    const uint32_t                 ntests = sizeof(aes_gcm_tp) / sizeof(struct gcm_test_pattern);
    const struct gcm_test_pattern *tp __attribute__((unused)) = aes_gcm_tp;
    pufs_status_t                  check                      = SUCCESS;
    uint8_t                       *out                        = NULL;
    uint32_t                       outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;
        out                    = malloc(tp[i].ctlen == 0 ? 1 : tp[i].ctlen);
        if (((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_enc_gcm(out,
                                   &outlen,
                                   tp[i].pt,
                                   tp[i].ptlen,
                                   tp[i].cipher,
                                   SSKEY,
                                   keyslot,
                                   tp[i].keybits,
                                   tp[i].iv,
                                   tp[i].ivlen,
                                   tp[i].aad,
                                   tp[i].aadlen,
                                   tag,
                                   tp[i].taglen)) != SUCCESS) ||
            ((check = (outlen != tp[i].ctlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot, tp[i].keybits)) != SUCCESS) ||
            ((check = pufs_test_memcmp(out, tp[i].ct, tp[i].ctlen)) != SUCCESS) ||
            ((check = pufs_test_memcmp(tag, tp[i].tag, tp[i].taglen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
        free(out);
    }
    return check;
}

pufs_status_t
pufs_aes_gcm_enc_swkey_test(void)
{
    const uint32_t                 ntests = sizeof(aes_gcm_tp) / sizeof(struct gcm_test_pattern);
    const struct gcm_test_pattern *tp     = aes_gcm_tp;
    pufs_status_t                  check  = SUCCESS;
    uint8_t                       *out    = NULL;
    uint32_t                       outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out = malloc(tp[i].ctlen == 0 ? 1 : tp[i].ctlen);
        if (((check = pufs_enc_gcm(out,
                                   &outlen,
                                   tp[i].pt,
                                   tp[i].ptlen,
                                   tp[i].cipher,
                                   SWKEY,
                                   tp[i].key,
                                   tp[i].keybits,
                                   tp[i].iv,
                                   tp[i].ivlen,
                                   tp[i].aad,
                                   tp[i].aadlen,
                                   tag,
                                   tp[i].taglen)) != SUCCESS) ||
            ((check = (outlen != tp[i].ctlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_test_memcmp(out, tp[i].ct, tp[i].ctlen)) != SUCCESS) ||
            ((check = pufs_test_memcmp(tag, tp[i].tag, tp[i].taglen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
        free(out);
    }
    return check;
}

pufs_status_t
pufs_aes_gcm_enc_multi_ctx_test(void)
{
    const uint32_t                 ntests = sizeof(aes_gcm_tp) / sizeof(struct gcm_test_pattern);
    const struct gcm_test_pattern *tp __attribute__((unused)) = aes_gcm_tp;
    pufs_status_t                  check                      = SUCCESS;

    pufs_sp38d_ctx **sp38d_ctxs = (pufs_sp38d_ctx **)malloc(ntests * sizeof(pufs_sp38d_ctx *));
    uint32_t        *outlens    = (uint32_t *)malloc(ntests * sizeof(uint32_t));
    uint8_t        **outs       = (uint8_t **)malloc(ntests * sizeof(uint8_t *));
    uint8_t        **tags       = (uint8_t **)malloc(ntests * sizeof(uint8_t *));
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((sp38d_ctxs[i] = pufs_sp38d_ctx_new()) == NULL)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((outs[i] = malloc(51)) == NULL)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((tags[i] = malloc(16)) == NULL)
        {
            goto failed;
        }
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        outlens[i] = 0;
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_gcm_init(sp38d_ctxs[i],
                                       tp[i].cipher,
                                       SWKEY,
                                       tp[i].key,
                                       tp[i].keybits,
                                       tp[i].iv,
                                       tp[i].ivlen)) != SUCCESS)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_gcm_update(sp38d_ctxs[i], NULL, NULL, tp[i].aad, tp[i].aadlen)) !=
            SUCCESS)
        {
            goto failed;
        }
    }
    uint32_t toutlen = 0;
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_gcm_update(
                 sp38d_ctxs[i], outs[i] + outlens[i], &toutlen, tp[i].pt, tp[i].ptlen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_gcm_final(
                 sp38d_ctxs[i], outs[i] + outlens[i], &toutlen, tags[i], tp[i].taglen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
        if (((check = (outlens[i] == tp[i].ctlen ? SUCCESS : E_VERFAIL)) != SUCCESS) ||
            ((check = pufs_test_memcmp(outs[i], tp[i].ct, tp[i].ctlen)) != SUCCESS) ||
            ((check = pufs_test_memcmp(tags[i], tp[i].tag, tp[i].taglen)) != SUCCESS))
        {
            goto failed;
        }
    }

failed:
    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_sp38d_ctx_free(sp38d_ctxs[i]);
    }
    free(sp38d_ctxs);
    free(outlens);
    for (uint32_t i = 0; i < ntests; i++)
    {
        free(outs[i]);
    }
    free(outs);
    for (uint32_t i = 0; i < ntests; i++)
    {
        free(tags[i]);
    }
    free(tags);
    return check;
}

pufs_status_t
pufs_aes_gcm_dec_test(void)
{
    const uint32_t                 ntests = sizeof(aes_gcm_tp) / sizeof(struct gcm_test_pattern);
    const struct gcm_test_pattern *tp __attribute__((unused)) = aes_gcm_tp;
    pufs_status_t                  check                      = SUCCESS;
    uint8_t                       *out                        = NULL;
    uint32_t                       outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;
        out                    = malloc(tp[i].ptlen == 0 ? 1 : tp[i].ptlen);
        if (((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_dec_gcm(out,
                                   &outlen,
                                   tp[i].ct,
                                   tp[i].ctlen,
                                   tp[i].cipher,
                                   SSKEY,
                                   keyslot,
                                   tp[i].keybits,
                                   tp[i].iv,
                                   tp[i].ivlen,
                                   tp[i].aad,
                                   tp[i].aadlen,
                                   tp[i].tag,
                                   tp[i].taglen)) != SUCCESS) ||
            ((check = (outlen != tp[i].ptlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot, tp[i].keybits)) != SUCCESS) ||
            ((check = pufs_test_memcmp(out, tp[i].pt, tp[i].ptlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
        free(out);
    }
    return check;
}

pufs_status_t
pufs_aes_gcm_dec_swkey_test(void)
{
    const uint32_t                 ntests = sizeof(aes_gcm_tp) / sizeof(struct gcm_test_pattern);
    const struct gcm_test_pattern *tp __attribute__((unused)) = aes_gcm_tp;
    pufs_status_t                  check                      = SUCCESS;
    uint8_t                       *out                        = NULL;
    uint32_t                       outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out = malloc(tp[i].ptlen == 0 ? 1 : tp[i].ptlen);
        if (((check = pufs_dec_gcm(out,
                                   &outlen,
                                   tp[i].ct,
                                   tp[i].ctlen,
                                   tp[i].cipher,
                                   SWKEY,
                                   tp[i].key,
                                   tp[i].keybits,
                                   tp[i].iv,
                                   tp[i].ivlen,
                                   tp[i].aad,
                                   tp[i].aadlen,
                                   tp[i].tag,
                                   tp[i].taglen)) != SUCCESS) ||
            ((check = (outlen != tp[i].ptlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_test_memcmp(out, tp[i].pt, tp[i].ptlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
        free(out);
    }
    return check;
}

pufs_status_t
pufs_aes_gcm_dec_multi_ctx_test(void)
{
    const uint32_t                 ntests = sizeof(aes_gcm_tp) / sizeof(struct gcm_test_pattern);
    const struct gcm_test_pattern *tp __attribute__((unused)) = aes_gcm_tp;
    pufs_status_t                  check                      = SUCCESS;

    pufs_sp38d_ctx **sp38d_ctxs = (pufs_sp38d_ctx **)malloc(ntests * sizeof(pufs_sp38d_ctx *));
    uint32_t        *outlens    = (uint32_t *)malloc(ntests * sizeof(uint32_t));
    uint8_t        **outs       = (uint8_t **)malloc(ntests * sizeof(uint8_t *));
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((sp38d_ctxs[i] = pufs_sp38d_ctx_new()) == NULL)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((outs[i] = malloc(51)) == NULL)
        {
            goto failed;
        }
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        outlens[i] = 0;
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_dec_gcm_init(sp38d_ctxs[i],
                                       tp[i].cipher,
                                       SWKEY,
                                       tp[i].key,
                                       tp[i].keybits,
                                       tp[i].iv,
                                       tp[i].ivlen)) != SUCCESS)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_dec_gcm_update(sp38d_ctxs[i], NULL, NULL, tp[i].aad, tp[i].aadlen)) !=
            SUCCESS)
        {
            goto failed;
        }
    }
    uint32_t toutlen = 0;
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_dec_gcm_update(
                 sp38d_ctxs[i], outs[i] + outlens[i], &toutlen, tp[i].ct, tp[i].ctlen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_dec_gcm_final(
                 sp38d_ctxs[i], outs[i] + outlens[i], &toutlen, tp[i].tag, tp[i].taglen)) !=
            SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
        if (outlens[i] != tp[i].ptlen || memcmp(outs[i], tp[i].pt, tp[i].ptlen) != 0)
        {
            check = E_VERFAIL;
            goto failed;
        }
    }

failed:
    if (check != SUCCESS)
    {
        fprintf(stdout, "\t[FAILED] %s\n", "{ ERROR MESSAGE HERE! }");
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_sp38d_ctx_free(sp38d_ctxs[i]);
    }
    free(sp38d_ctxs);
    free(outlens);
    for (uint32_t i = 0; i < ntests; i++)
    {
        free(outs[i]);
    }
    free(outs);
    return check;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
