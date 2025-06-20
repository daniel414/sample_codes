/**
 * @file pufs_hmac_test.c
 * @brief HMAC API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
// #include <stdint.h>
// #include <stdbool.h>
#include "pufs_tv_sha2.h"
#include "pufs_tv_hmac_sha2.h"
#include "pufs_hmac_test.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/
static pufs_status_t pufs_hash_iuf_test(uint32_t ntests, const struct hash_test_pattern *tp);
static pufs_status_t pufs_hash_test(uint32_t ntests, const struct hash_test_pattern *tp);

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_sha2_iuf_test(void)
{
    return pufs_hash_iuf_test(g_sha2_tp_count, g_sha2_tp);
}

pufs_status_t
pufs_sha2_test(void)
{
    return pufs_hash_test(g_sha2_tp_count, g_sha2_tp);
}

pufs_status_t
pufs_hmac_sha2_iuf_test(void)
{
    const struct hmac_test_pattern *tp     = g_hmac_sha2_tp;
    const uint32_t                  ntests = g_hmac_sha2_tp_count;

    uint32_t       kbits;
    pufs_status_t  check    = E_ERROR;
    pufs_hmac_ctx *hmac_ctx = pufs_hmac_ctx_new();
    if (NULL != hmac_ctx)
    {
        for (uint32_t i = 0; i < ntests; i++)
        {
            pufs_dgst_st md;

            kbits = tp[i].keybits;
            if (kbits > 512)
            {
                continue;
            }

            pufs_ka_slot_t keyslot;
            keyslot = (kbits > 256) ? SK512_0 : (kbits > 128) ? SK256_0 : SK128_0;
            check   = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, kbits);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_hmac_init(hmac_ctx, tp[i].hash, SSKEY, keyslot, kbits);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_hmac_update(hmac_ctx, tp[i].msg, tp[i].msglen);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_hmac_final(hmac_ctx, &md);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_clear_key(SSKEY, keyslot, kbits);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_test_memcmp(tp[i].md, md.dgst, md.dlen);
            if (SUCCESS != check)
            {
                break;
            }
        }
        pufs_hmac_ctx_free(hmac_ctx);
    }

    return check;
}

pufs_status_t
pufs_hmac_sha2_test(void)
{
    const struct hmac_test_pattern *tp     = g_hmac_sha2_tp;
    const uint32_t                  ntests = g_hmac_sha2_tp_count;

    uint32_t      kbits;
    pufs_status_t check = E_ERROR;
    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_dgst_st md;

        kbits = tp[i].keybits;
        if (kbits > 512)
        {
            continue;
        }

        pufs_ka_slot_t keyslot;
        keyslot = (kbits > 256) ? SK512_0 : (kbits > 128) ? SK256_0 : SK128_0;
        check   = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, kbits);
        if (SUCCESS != check)
        {
            break;
        }

        check = pufs_hmac(&md, tp[i].msg, tp[i].msglen, tp[i].hash, SSKEY, keyslot, kbits);
        if (SUCCESS != check)
        {
            break;
        }

        check = pufs_clear_key(SSKEY, keyslot, kbits);
        if (SUCCESS != check)
        {
            break;
        }

        check = pufs_test_memcmp(tp[i].md, md.dgst, md.dlen);
        if (SUCCESS != check)
        {
            break;
        }
    }

    return check;
}

pufs_status_t
pufs_hmac_sha2_swkey_iuf_test(void)
{
    const struct hmac_test_pattern *tp     = g_hmac_sha2_tp;
    const uint32_t                  ntests = g_hmac_sha2_tp_count;

    pufs_status_t  check    = E_ERROR;
    pufs_hmac_ctx *hmac_ctx = pufs_hmac_ctx_new();
    if (NULL != hmac_ctx)
    {
        for (uint32_t i = 0; i < ntests; i++)
        {
            pufs_dgst_st md;

            check = pufs_hmac_init(hmac_ctx, tp[i].hash, SWKEY, tp[i].key, tp[i].keybits);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_hmac_update(hmac_ctx, tp[i].msg, tp[i].msglen);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_hmac_final(hmac_ctx, &md);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_test_memcmp(tp[i].md, md.dgst, md.dlen);
            if (SUCCESS != check)
            {
                break;
            }
        }
        pufs_hmac_ctx_free(hmac_ctx);
    }

    return check;
}

pufs_status_t
pufs_hmac_sha2_swkey_test(void)
{
    const struct hmac_test_pattern *tp     = g_hmac_sha2_tp;
    const uint32_t                  ntests = g_hmac_sha2_tp_count;

    pufs_status_t check = E_ERROR;
    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_dgst_st md;

        check =
            pufs_hmac(&md, tp[i].msg, tp[i].msglen, tp[i].hash, SWKEY, tp[i].key, tp[i].keybits);
        if (SUCCESS != check)
        {
            break;
        }

        check = pufs_test_memcmp(tp[i].md, md.dgst, md.dlen);
        if (SUCCESS != check)
        {
            break;
        }
    }
    return check;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/
static pufs_status_t
pufs_hash_iuf_test(uint32_t ntests, const struct hash_test_pattern *tp)
{
    pufs_status_t  check    = E_ERROR;
    pufs_hash_ctx *hash_ctx = pufs_hash_ctx_new();
    if (NULL != hash_ctx)
    {
        for (uint32_t i = 0; i < ntests; i++)
        {
            pufs_dgst_st md;

            check = pufs_hash_init(hash_ctx, tp[i].hash);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_hash_update(hash_ctx, tp[i].msg, tp[i].msglen);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_hash_final(hash_ctx, &md);
            if (SUCCESS != check)
            {
                break;
            }

            check = pufs_test_memcmp(tp[i].md, md.dgst, md.dlen);
            if (SUCCESS != check)
            {
                break;
            }
        }
        pufs_hash_ctx_free(hash_ctx);
    }

    return check;
}

static pufs_status_t
pufs_hash_test(uint32_t ntests, const struct hash_test_pattern *tp)
{
    pufs_status_t check = E_ERROR;

    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_dgst_st md;

        check = pufs_hash(&md, tp[i].msg, tp[i].msglen, tp[i].hash);
        if (SUCCESS != check)
        {
            break;
        }

        check = pufs_test_memcmp(tp[i].md, md.dgst, md.dlen);
        if (SUCCESS != check)
        {
            break;
        }
    }

    return check;
}

/*** end of file ***/
