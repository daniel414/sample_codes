/**
 * @file pufs_cmac_test.c
 * @brief CMAC API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>

#include "pufs_cmac_test.h"
#include "pufs_cmac.h"
#include "pufs_ka.h"
#include "pufs_log.h"
#include "pufs_tv_cmac.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_cmac_test(void)
{
    pufs_status_t                   check  = SUCCESS;
    const uint32_t                  ntests = sizeof(cmac_tp) / sizeof(struct cmac_test_pattern);
    const struct cmac_test_pattern *tp     = cmac_tp;

    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_dgst_st   md;
        pufs_ka_slot_t keyslot = (tp[i].keybits > 128) ? SK256_0 : SK128_0;
        if (((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].key, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_cmac(
                  &md, tp[i].msg, tp[i].msglen, tp[i].cipher, SSKEY, keyslot, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot, tp[i].keybits)) != SUCCESS) ||
            ((check = pufs_test_memcmp(tp[i].md, md.dgst, md.dlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }
    return check;
}

pufs_status_t
pufs_cmac_swkey_test(void)
{
    pufs_status_t                   check  = SUCCESS;
    const uint32_t                  ntests = sizeof(cmac_tp) / sizeof(struct cmac_test_pattern);
    const struct cmac_test_pattern *tp     = cmac_tp;

    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_dgst_st md;
        if (((check = pufs_cmac(
                  &md, tp[i].msg, tp[i].msglen, tp[i].cipher, SWKEY, tp[i].key, tp[i].keybits)) !=
             SUCCESS) ||
            ((check = pufs_test_memcmp(tp[i].md, md.dgst, md.dlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }
    return check;
}

pufs_status_t
pufs_cmac_multi_ctx_test(void)
{
    const uint32_t                  ntests = sizeof(cmac_tp) / sizeof(struct cmac_test_pattern);
    const struct cmac_test_pattern *tp     = cmac_tp;
    pufs_status_t                   check  = SUCCESS;

    pufs_cmac_ctx **cmac_ctxs = (pufs_cmac_ctx **)malloc(ntests * sizeof(pufs_cmac_ctx *));
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((cmac_ctxs[i] = pufs_cmac_ctx_new()) == NULL)
        {
            goto failed;
        }
    }

    pufs_dgst_st md;
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_cmac_init(cmac_ctxs[i], tp[i].cipher, SWKEY, tp[i].key, tp[i].keybits)) !=
            SUCCESS)
        {
            goto failed;
        }
    }
    for (uint32_t i = 0; i < ntests; i++)
    {
        if ((check = pufs_cmac_update(cmac_ctxs[i], tp[i].msg, tp[i].msglen)) != SUCCESS)
        {
            goto failed;
        }
    }

    for (uint32_t i = 0; i < ntests; i++)
    {
        if (((check = pufs_cmac_final(cmac_ctxs[i], &md)) != SUCCESS) ||
            ((check = pufs_test_memcmp(tp[i].md, md.dgst, md.dlen)) != SUCCESS))
        {
            goto failed;
        }
    }

failed:
    for (uint32_t i = 0; i < ntests; i++)
    {
        pufs_cmac_ctx_free(cmac_ctxs[i]);
    }
    free(cmac_ctxs);
    return check;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
