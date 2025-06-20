/**
 * @file pufs_sp38a_test.c
 * @brief SP38A API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>

#include "pufs_sp38a_test.h"
#include "pufs_ka.h"
#include "pufs_sp38a.h"
#include "pufs_log.h"
#include "pufs_tv_aescbc.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/
static pufs_status_t pufs_cbc_enc_test(uint32_t ntests, const struct bc_test_pattern *tp, int cs);
static pufs_status_t pufs_cbc_enc_swkey_test(uint32_t                      ntests,
                                             const struct bc_test_pattern *tp,
                                             int                           cs);
static pufs_status_t pufs_cbc_enc_multi_ctx_test(uint32_t                      ntests,
                                                 const struct bc_test_pattern *tp,
                                                 int                           cs);
static pufs_status_t pufs_cbc_dec_test(uint32_t ntests, const struct bc_test_pattern *tp, int cs);
static pufs_status_t pufs_cbc_dec_swkey_test(uint32_t                      ntests,
                                             const struct bc_test_pattern *tp,
                                             int                           cs);
static pufs_status_t pufs_cbc_dec_multi_ctx_test(uint32_t                      ntests,
                                                 const struct bc_test_pattern *tp,
                                                 int                           cs);

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_aes_cbc_enc_test(void)
{
    return pufs_cbc_enc_test(TCPARAM(struct bc_test_pattern, aes_cbc_tp), 0);
}

pufs_status_t
pufs_aes_cbc_enc_swkey_test(void)
{
    return pufs_cbc_enc_swkey_test(TCPARAM(struct bc_test_pattern, aes_cbc_tp), 0);
}

pufs_status_t
pufs_aes_cbc_enc_multi_ctx_test(void)
{
    return pufs_cbc_enc_multi_ctx_test(TCPARAM(struct bc_test_pattern, aes_cbc_tp), 0);
}

pufs_status_t
pufs_aes_cbc_dec_test(void)
{
    return pufs_cbc_dec_test(TCPARAM(struct bc_test_pattern, aes_cbc_tp), 0);
}

pufs_status_t
pufs_aes_cbc_dec_swkey_test(void)
{
    return pufs_cbc_dec_swkey_test(TCPARAM(struct bc_test_pattern, aes_cbc_tp), 0);
}

pufs_status_t
pufs_aes_cbc_dec_multi_ctx_test(void)
{
    return pufs_cbc_dec_multi_ctx_test(TCPARAM(struct bc_test_pattern, aes_cbc_tp), 0);
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/
static pufs_status_t
pufs_cbc_enc_test(uint32_t ntests, const struct bc_test_pattern *tp, int cs)
{
    pufs_status_t check = SUCCESS;
    uint8_t      *out   = NULL;
    uint32_t      outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;
        out                    = malloc(tp[i].ctlen);
        if (((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_enc_cbc(out,
                                   &outlen,
                                   tp[i].pt,
                                   tp[i].ptlen,
                                   tp[i].cipher,
                                   SSKEY,
                                   keyslot,
                                   tp[i].keybits,
                                   tp[i].iv,
                                   cs)) != SUCCESS) ||
            ((check = (outlen != tp[i].ctlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot, tp[i].keybits)) != SUCCESS) ||
            ((check = pufs_test_memcmp(out, tp[i].ct, tp[i].ctlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
        free(out);
    }
    return check;
}

static pufs_status_t
pufs_cbc_enc_swkey_test(uint32_t ntests, const struct bc_test_pattern *tp, int cs)
{
    pufs_status_t check = SUCCESS;
    uint8_t      *out   = NULL;
    uint32_t      outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out = malloc(tp[i].ctlen);
        if (((check = pufs_enc_cbc(out,
                                   &outlen,
                                   tp[i].pt,
                                   tp[i].ptlen,
                                   tp[i].cipher,
                                   SWKEY,
                                   tp[i].key,
                                   tp[i].keybits,
                                   tp[i].iv,
                                   cs)) != SUCCESS) ||
            ((check = (outlen != tp[i].ctlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_test_memcmp(out, tp[i].ct, tp[i].ctlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
        free(out);
    }
    return check;
}

static pufs_status_t
pufs_cbc_enc_multi_ctx_test(uint32_t ntests, const struct bc_test_pattern *tp, int cs)
{
    pufs_status_t    check      = SUCCESS;
    pufs_sp38a_ctx **sp38a_ctxs = (pufs_sp38a_ctx **)malloc(ntests * sizeof(pufs_sp38a_ctx *));
    uint32_t        *outlens    = (uint32_t *)malloc(ntests * sizeof(uint32_t));
    uint8_t        **outs       = (uint8_t **)malloc(ntests * sizeof(uint8_t *));
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((sp38a_ctxs[i] = pufs_sp38a_ctx_new()) == NULL)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((outs[i] = malloc(80)) == NULL)
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
        if ((check = pufs_enc_cbc_init(
                 sp38a_ctxs[i], tp[i].cipher, SWKEY, tp[i].key, tp[i].keybits, tp[i].iv, cs)) !=
            SUCCESS)
        {
            goto failed;
        }
    }
    uint32_t toutlen = 0;
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_cbc_update(
                 sp38a_ctxs[i], outs[i] + outlens[i], &toutlen, tp[i].pt, tp[i].ptlen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_enc_cbc_final(sp38a_ctxs[i], outs[i] + outlens[i], &toutlen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
        if (((check = (outlens[i] == tp[i].ctlen ? SUCCESS : E_VERFAIL)) != SUCCESS) ||
            ((check = pufs_test_memcmp(outs[i], tp[i].ct, tp[i].ctlen)) != SUCCESS))
        {
            goto failed;
        }
    }

failed:
    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_sp38a_ctx_free(sp38a_ctxs[i]);
    }
    free(sp38a_ctxs);
    free(outlens);
    for (uint32_t i = 0; i < ntests; i++)
    {
        free(outs[i]);
    }
    free(outs);
    return check;
}

static pufs_status_t
pufs_cbc_dec_test(uint32_t ntests, const struct bc_test_pattern *tp, int cs)
{
    pufs_status_t check = SUCCESS;
    uint8_t      *out   = NULL;
    uint32_t      outlen;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out                    = malloc(tp[i].ptlen);
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;
        if (((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_dec_cbc(out,
                                   &outlen,
                                   tp[i].ct,
                                   tp[i].ctlen,
                                   tp[i].cipher,
                                   SSKEY,
                                   keyslot,
                                   tp[i].keybits,
                                   tp[i].iv,
                                   cs)) != SUCCESS) ||
            ((check = (outlen != tp[i].ptlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot, tp[i].keybits) != SUCCESS)) ||
            ((check = pufs_test_memcmp(out, tp[i].pt, tp[i].ptlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
        free(out);
    }
    return check;
}

static pufs_status_t
pufs_cbc_dec_swkey_test(uint32_t ntests, const struct bc_test_pattern *tp, int cs)
{
    pufs_status_t check = SUCCESS;
    uint32_t      outlen;
    uint8_t      *out = NULL;
    for (uint32_t i = 0; i < ntests; i++)
    {
        out = malloc(tp[i].ptlen);
        if (((check = pufs_dec_cbc(out,
                                   &outlen,
                                   tp[i].ct,
                                   tp[i].ctlen,
                                   tp[i].cipher,
                                   SWKEY,
                                   tp[i].key,
                                   tp[i].keybits,
                                   tp[i].iv,
                                   cs) != SUCCESS)) ||
            ((check = (outlen != tp[i].ptlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_test_memcmp(out, tp[i].pt, tp[i].ptlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
        free(out);
    }
    return check;
}

static pufs_status_t
pufs_cbc_dec_multi_ctx_test(uint32_t ntests, const struct bc_test_pattern *tp, int cs)
{
    pufs_status_t    check      = SUCCESS;
    pufs_sp38a_ctx **sp38a_ctxs = (pufs_sp38a_ctx **)malloc(ntests * sizeof(pufs_sp38a_ctx *));
    uint32_t        *outlens    = (uint32_t *)malloc(ntests * sizeof(uint32_t));
    uint8_t        **outs       = (uint8_t **)malloc(ntests * sizeof(uint8_t *));
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((sp38a_ctxs[i] = pufs_sp38a_ctx_new()) == NULL)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((outs[i] = malloc(80)) == NULL)
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
        if ((check = pufs_dec_cbc_init(
                 sp38a_ctxs[i], tp[i].cipher, SWKEY, tp[i].key, tp[i].keybits, tp[i].iv, cs)) !=
            SUCCESS)
        {
            goto failed;
        }
    }
    uint32_t toutlen = 0;
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_dec_cbc_update(
                 sp38a_ctxs[i], outs[i] + outlens[i], &toutlen, tp[i].ct, tp[i].ctlen)) != SUCCESS)
        {
            goto failed;
        }
        outlens[i] += toutlen;
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_dec_cbc_final(sp38a_ctxs[i], outs[i] + outlens[i], &toutlen)) != SUCCESS)
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
        pufs_sp38a_ctx_free(sp38a_ctxs[i]);
    }
    free(sp38a_ctxs);
    free(outlens);
    for (uint32_t i = 0; i < ntests; i++)
    {
        free(outs[i]);
    }
    free(outs);
    return check;
}

/*** end of file ***/
