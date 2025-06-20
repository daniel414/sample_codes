/**
 * @file pufs_sp38e_test.c
 * @brief SP38E API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>

#include "pufs_sp38e_test.h"
#include "pufs_ka.h"
#include "pufs_sp38e.h"
#include "pufs_log.h"
#include "pufs_tv_aesxts.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_aes_xts_enc_test(void)
{
    const uint32_t                ntests = sizeof(aes_xts_tp) / sizeof(struct bc_test_pattern);
    const struct bc_test_pattern *tp     = aes_xts_tp;
    pufs_status_t                 check  = SUCCESS;
    uint8_t                      *out    = NULL;
    uint32_t                      outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out                     = malloc(tp[i].ctlen);
        pufs_ka_slot_t keyslot1 = (tp[i].keybits > 128) ? SK256_0 : SK128_0;
        pufs_ka_slot_t keyslot2 = (tp[i].keybits > 128) ? SK256_1 : SK128_1;
        if (((check = pufs_import_plaintext_key(SSKEY, keyslot1, tp[i].key, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_import_plaintext_key(
                  SSKEY, keyslot2, (uint8_t *)tp[i].key + b2B(tp[i].keybits), tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_enc_xts(out,
                                   &outlen,
                                   tp[i].pt,
                                   tp[i].ptlen,
                                   tp[i].cipher,
                                   SSKEY,
                                   keyslot1,
                                   tp[i].keybits,
                                   SSKEY,
                                   keyslot2,
                                   tp[i].iv,
                                   0)) != SUCCESS) ||
            ((check = (outlen != tp[i].ctlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot1, tp[i].keybits)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot2, tp[i].keybits)) != SUCCESS) ||
            ((check = pufs_test_memcmp(out, tp[i].ct, tp[i].ctlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
        free(out);
    }
    return check;
}

pufs_status_t
pufs_aes_xts_enc_swkey_test(void)
{
    const uint32_t                ntests = sizeof(aes_xts_tp) / sizeof(struct bc_test_pattern);
    const struct bc_test_pattern *tp     = aes_xts_tp;
    pufs_status_t                 check  = SUCCESS;
    uint8_t                      *out    = NULL;
    uint32_t                      outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out = malloc(tp[i].ctlen);
        if (((check = pufs_enc_xts(out,
                                   &outlen,
                                   tp[i].pt,
                                   tp[i].ptlen,
                                   tp[i].cipher,
                                   SWKEY,
                                   tp[i].key,
                                   tp[i].keybits,
                                   SWKEY,
                                   tp[i].key + b2B(tp[i].keybits),
                                   tp[i].iv,
                                   0)) != SUCCESS) ||
            ((check = (outlen != tp[i].ctlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_test_memcmp(out, tp[i].ct, tp[i].ctlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            return check;
        }
        free(out);
    }
    return check;
}

pufs_status_t
pufs_aes_xts_enc_multi_ctx_test(void)
{
    const uint32_t                ntests = sizeof(aes_xts_tp) / sizeof(struct bc_test_pattern);
    const struct bc_test_pattern *tp     = aes_xts_tp;
    pufs_status_t                 check  = SUCCESS;

    pufs_sp38e_ctx **sp38e_ctxs = (pufs_sp38e_ctx **)malloc(ntests * sizeof(pufs_sp38e_ctx *));
    uint32_t        *outlens    = (uint32_t *)malloc(ntests * sizeof(uint32_t));
    uint8_t        **outs       = (uint8_t **)malloc(ntests * sizeof(uint8_t *));
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((sp38e_ctxs[i] = pufs_sp38e_ctx_new()) == NULL)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((outs[i] = malloc(48)) == NULL)
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
        if ((check = pufs_enc_xts_init(sp38e_ctxs[i],
                                       tp[i].cipher,
                                       SWKEY,
                                       tp[i].key,
                                       tp[i].keybits,
                                       SWKEY,
                                       tp[i].key + b2B(tp[i].keybits),
                                       tp[i].iv,
                                       0)) != SUCCESS)
        {
            goto failed;
        }
    }
    uint32_t toutlen = 0;
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_xts_update(
                 sp38e_ctxs[i], outs[i] + outlens[i], &toutlen, tp[i].pt, tp[i].ptlen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_xts_final(sp38e_ctxs[i], outs[i] + outlens[i], &toutlen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
        if (outlens[i] != tp[i].ctlen || memcmp(outs[i], tp[i].ct, tp[i].ctlen) != 0)
        {
            check = E_VERFAIL;
            goto failed;
        }
    }

failed:
    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_sp38e_ctx_free(sp38e_ctxs[i]);
    }
    free(sp38e_ctxs);
    free(outlens);
    for (uint32_t i = 0; i < ntests; i++)
    {
        free(outs[i]);
    }
    free(outs);
    return check;
}

pufs_status_t
pufs_aes_xts_dec_test(void)
{
    const uint32_t                ntests = sizeof(aes_xts_tp) / sizeof(struct bc_test_pattern);
    const struct bc_test_pattern *tp     = aes_xts_tp;
    pufs_status_t                 check  = SUCCESS;
    uint8_t                      *out    = NULL;
    uint32_t                      outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out                     = malloc(tp[i].ptlen);
        pufs_ka_slot_t keyslot1 = (tp[i].keybits > 128) ? SK256_0 : SK128_0;
        pufs_ka_slot_t keyslot2 = (tp[i].keybits > 128) ? SK256_1 : SK128_1;
        if (((check = pufs_import_plaintext_key(SSKEY, keyslot1, tp[i].key, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_import_plaintext_key(
                  SSKEY, keyslot2, (uint8_t *)tp[i].key + b2B(tp[i].keybits), tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_dec_xts(out,
                                   &outlen,
                                   tp[i].ct,
                                   tp[i].ctlen,
                                   tp[i].cipher,
                                   SSKEY,
                                   keyslot1,
                                   tp[i].keybits,
                                   SSKEY,
                                   keyslot2,
                                   tp[i].iv,
                                   0)) != SUCCESS) ||
            ((check = (outlen != tp[i].ptlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot1, tp[i].keybits)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot2, tp[i].keybits)) != SUCCESS) ||
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
pufs_aes_xts_dec_swkey_test(void)
{
    const uint32_t                ntests = sizeof(aes_xts_tp) / sizeof(struct bc_test_pattern);
    const struct bc_test_pattern *tp     = aes_xts_tp;
    pufs_status_t                 check  = SUCCESS;
    uint8_t                      *out    = NULL;
    uint32_t                      outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out = malloc(tp[i].ptlen);
        if (((check = pufs_dec_xts(out,
                                   &outlen,
                                   tp[i].ct,
                                   tp[i].ctlen,
                                   tp[i].cipher,
                                   SWKEY,
                                   tp[i].key,
                                   tp[i].keybits,
                                   SWKEY,
                                   tp[i].key + b2B(tp[i].keybits),
                                   tp[i].iv,
                                   0)) != SUCCESS) ||
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
pufs_aes_xts_dec_multi_ctx_test(void)
{
    const uint32_t                ntests = sizeof(aes_xts_tp) / sizeof(struct bc_test_pattern);
    const struct bc_test_pattern *tp     = aes_xts_tp;
    pufs_status_t                 check  = SUCCESS;

    pufs_sp38e_ctx **sp38e_ctxs = (pufs_sp38e_ctx **)malloc(ntests * sizeof(pufs_sp38e_ctx *));
    uint32_t        *outlens    = (uint32_t *)malloc(ntests * sizeof(uint32_t));
    uint8_t        **outs       = (uint8_t **)malloc(ntests * sizeof(uint8_t *));
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((sp38e_ctxs[i] = pufs_sp38e_ctx_new()) == NULL)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((outs[i] = malloc(48)) == NULL)
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
        if ((check = pufs_dec_xts_init(sp38e_ctxs[i],
                                       tp[i].cipher,
                                       SWKEY,
                                       tp[i].key,
                                       tp[i].keybits,
                                       SWKEY,
                                       tp[i].key + b2B(tp[i].keybits),
                                       tp[i].iv,
                                       0)) != SUCCESS)
        {
            goto failed;
        }
    }
    uint32_t toutlen = 0;
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_dec_xts_update(
                 sp38e_ctxs[i], outs[i] + outlens[i], &toutlen, tp[i].ct, tp[i].ctlen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_dec_xts_final(sp38e_ctxs[i], outs[i] + outlens[i], &toutlen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
        if (((check = (outlens[i] == tp[i].ptlen ? SUCCESS : E_VERFAIL)) != SUCCESS) ||
            ((check = pufs_test_memcmp(outs[i], tp[i].pt, tp[i].ptlen)) != SUCCESS))
        {
            goto failed;
        }
    }

failed:
    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_sp38e_ctx_free(sp38e_ctxs[i]);
    }
    free(sp38e_ctxs);
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
