/**
 * @file pufs_sp38c_test.c
 * @brief SP38C API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>

#include "pufs_sp38c_test.h"
#include "pufs_ka.h"
#include "pufs_sp38c.h"
#include "pufs_log.h"
#include "pufs_tv_aesccm.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_aes_ccm_enc_test(void)
{
    const uint32_t                 ntests = sizeof(aes_ccm_tp) / sizeof(struct ccm_test_pattern);
    const struct ccm_test_pattern *tp     = aes_ccm_tp;
    pufs_status_t                  check  = SUCCESS;
    uint8_t                       *out = NULL, *tag = NULL;
    uint32_t                       outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out                    = malloc(tp[i].ctlen == 0 ? 1 : tp[i].ctlen);
        tag                    = malloc(BC_BLOCK_SIZE);
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;
        if (((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_enc_ccm(out,
                                   &outlen,
                                   tp[i].pt,
                                   tp[i].ptlen,
                                   tp[i].cipher,
                                   SSKEY,
                                   keyslot,
                                   tp[i].keybits,
                                   tp[i].nonce,
                                   tp[i].noncelen,
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
        free(tag);
    }
    return check;
}

pufs_status_t
pufs_aes_ccm_enc_swkey_test(void)
{
    const uint32_t                 ntests = sizeof(aes_ccm_tp) / sizeof(struct ccm_test_pattern);
    const struct ccm_test_pattern *tp     = aes_ccm_tp;
    pufs_status_t                  check  = SUCCESS;
    uint8_t                       *out = NULL, *tag = NULL;
    uint32_t                       outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out = malloc(tp[i].ctlen == 0 ? 1 : tp[i].ctlen);
        tag = malloc(BC_BLOCK_SIZE);
        if (((check = pufs_enc_ccm(out,
                                   &outlen,
                                   tp[i].pt,
                                   tp[i].ptlen,
                                   tp[i].cipher,
                                   SWKEY,
                                   tp[i].key,
                                   tp[i].keybits,
                                   tp[i].nonce,
                                   tp[i].noncelen,
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
        free(tag);
    }
    return check;
}

pufs_status_t
pufs_aes_ccm_enc_multi_ctx_test(void)
{
    const uint32_t                 ntests = sizeof(aes_ccm_tp) / sizeof(struct ccm_test_pattern);
    const struct ccm_test_pattern *tp     = aes_ccm_tp;
    pufs_status_t                  check  = SUCCESS;

    pufs_sp38c_ctx **sp38c_ctxs = (pufs_sp38c_ctx **)malloc(ntests * sizeof(pufs_sp38c_ctx *));
    uint32_t        *outlens    = (uint32_t *)malloc(ntests * sizeof(uint32_t));
    uint8_t        **outs       = (uint8_t **)malloc(ntests * sizeof(uint8_t *));
    uint8_t        **tags       = (uint8_t **)malloc(ntests * sizeof(uint8_t *));
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((sp38c_ctxs[i] = pufs_sp38c_ctx_new()) == NULL)
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
        if ((check = pufs_enc_ccm_init(sp38c_ctxs[i],
                                       tp[i].cipher,
                                       SWKEY,
                                       tp[i].key,
                                       tp[i].keybits,
                                       tp[i].nonce,
                                       tp[i].noncelen,
                                       tp[i].aadlen,
                                       tp[i].ptlen,
                                       tp[i].taglen)) != SUCCESS)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_ccm_update(sp38c_ctxs[i], NULL, NULL, tp[i].aad, tp[i].aadlen)) !=
            SUCCESS)
        {
            goto failed;
        }
    }
    uint32_t toutlen = 0;
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_ccm_update(
                 sp38c_ctxs[i], outs[i] + outlens[i], &toutlen, tp[i].pt, tp[i].ptlen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_ccm_final(sp38c_ctxs[i], outs[i] + outlens[i], &toutlen, tags[i])) !=
            SUCCESS)
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
        pufs_sp38c_ctx_free(sp38c_ctxs[i]);
    }
    free(sp38c_ctxs);
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
pufs_aes_ccm_dec_test(void)
{
    const uint32_t                 ntests = sizeof(aes_ccm_tp) / sizeof(struct ccm_test_pattern);
    const struct ccm_test_pattern *tp     = aes_ccm_tp;
    pufs_status_t                  check  = SUCCESS;
    uint8_t                       *out    = NULL;
    uint32_t                       outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out                    = malloc(tp[i].ptlen == 0 ? 1 : tp[i].ptlen);
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;
        if (((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_dec_ccm(out,
                                   &outlen,
                                   tp[i].ct,
                                   tp[i].ctlen,
                                   tp[i].cipher,
                                   SSKEY,
                                   keyslot,
                                   tp[i].keybits,
                                   tp[i].nonce,
                                   tp[i].noncelen,
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
pufs_aes_ccm_dec_swkey_test(void)
{
    const uint32_t                 ntests = sizeof(aes_ccm_tp) / sizeof(struct ccm_test_pattern);
    const struct ccm_test_pattern *tp     = aes_ccm_tp;
    pufs_status_t                  check  = SUCCESS;
    uint8_t                       *out    = NULL;
    uint32_t                       outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out = malloc(tp[i].ptlen == 0 ? 1 : tp[i].ptlen);
        if (((check = pufs_dec_ccm(out,
                                   &outlen,
                                   tp[i].ct,
                                   tp[i].ctlen,
                                   tp[i].cipher,
                                   SWKEY,
                                   tp[i].key,
                                   tp[i].keybits,
                                   tp[i].nonce,
                                   tp[i].noncelen,
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
pufs_aes_ccm_dec_multi_ctx_test(void)
{
    const uint32_t                 ntests = sizeof(aes_ccm_tp) / sizeof(struct ccm_test_pattern);
    const struct ccm_test_pattern *tp     = aes_ccm_tp;
    pufs_status_t                  check  = SUCCESS;

    pufs_sp38c_ctx **sp38c_ctxs = (pufs_sp38c_ctx **)malloc(ntests * sizeof(pufs_sp38c_ctx *));
    uint32_t        *outlens    = (uint32_t *)malloc(ntests * sizeof(uint32_t));
    uint8_t        **outs       = (uint8_t **)malloc(ntests * sizeof(uint8_t *));
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((sp38c_ctxs[i] = pufs_sp38c_ctx_new()) == NULL)
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
        if (pufs_dec_ccm_init(sp38c_ctxs[i],
                              tp[i].cipher,
                              SWKEY,
                              tp[i].key,
                              tp[i].keybits,
                              tp[i].nonce,
                              tp[i].noncelen,
                              tp[i].aadlen,
                              tp[i].ctlen,
                              tp[i].taglen) != SUCCESS)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if (pufs_dec_ccm_update(sp38c_ctxs[i], NULL, NULL, tp[i].aad, tp[i].aadlen) != SUCCESS)
        {
            goto failed;
        }
    }
    uint32_t toutlen = 0;
    for (uint32_t i = 0; i < ntests; i++)
    {
        if (pufs_dec_ccm_update(
                sp38c_ctxs[i], outs[i] + outlens[i], &toutlen, tp[i].ct, tp[i].ctlen) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if (pufs_dec_ccm_final(sp38c_ctxs[i], outs[i] + outlens[i], &toutlen, tp[i].tag) != SUCCESS)
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
        pufs_sp38c_ctx_free(sp38c_ctxs[i]);
    }
    free(sp38c_ctxs);
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
