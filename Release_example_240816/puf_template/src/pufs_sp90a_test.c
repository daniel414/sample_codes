/**
 * @file pufs_sp90a_test.c
 * @brief SP90A API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_sp90a_test.h"
#include "pufs_rt.h"
#include "pufs_sp90a.h"
#include "pufs_log.h"
#include "pufs_tv_drbgctr.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/
static pufs_status_t pufs_drbg_test_testmode(void);
static pufs_status_t pufs_drbg_test_normal(bool df);

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_drbg_test(void)
{
    // enable DRBG test mode
    pufs_drbg_enable_testmode();

    if (pufs_drbg_is_testmode())
    {
        return pufs_drbg_test_testmode();
    }
    else
    {
        pufs_status_t check = SUCCESS;
        if ((check = pufs_drbg_test_normal(false)) != SUCCESS)
        {
            return check;
        }
        return pufs_drbg_test_normal(true);
    }
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/
static pufs_status_t
pufs_drbg_test_testmode(void)
{
    const uint32_t                  ntests = sizeof(drbg_tp) / sizeof(struct drbg_test_pattern);
    const struct drbg_test_pattern *tp     = drbg_tp;
    pufs_status_t                   check  = SUCCESS;

    for (uint32_t i = 0; i < ntests; i++)
    {
        uint8_t out[64];
        bool    df = (tp[i].nonce != NULL);

        // instantiate
        pufs_drbg_testmode_entropy(tp[i].e_inst, tp[i].e_instlen);
        if ((check = pufs_drbg_instantiate(AES_CTR_DRBG,
                                           tp[i].security,
                                           df,
                                           tp[i].nonce,
                                           tp[i].noncelen,
                                           tp[i].pstr,
                                           tp[i].pstrlen)) != SUCCESS)
        {
            goto uninst;
        }

        // reseed
        if (tp[i].e_reseed != NULL)
        {
            pufs_drbg_testmode_entropy(tp[i].e_reseed, tp[i].e_reseedlen);
            if ((check = pufs_drbg_reseed(df, tp[i].adin_reseed, tp[i].adin_reseedlen)) != SUCCESS)
            {
                goto uninst;
            }
        }

        bool pr;
        // 1st generate
        pr = (tp[i].e_gen1 != NULL);
        if (pr)
        {
            pufs_drbg_testmode_entropy(tp[i].e_gen1, tp[i].e_gen1len);
        }
        if ((check = pufs_drbg_generate(
                 out, B2b(tp[i].outlen), pr, df, tp[i].adin_gen1, tp[i].adin_gen1len, true)) !=
            SUCCESS)
        {
            goto uninst;
        }

        // 2nd generate
        pr = (tp[i].e_gen2 != NULL);
        if (pr)
        {
            pufs_drbg_testmode_entropy(tp[i].e_gen2, tp[i].e_gen2len);
        }
        if ((check = pufs_drbg_generate(
                 out, B2b(tp[i].outlen), pr, df, tp[i].adin_gen2, tp[i].adin_gen2len, true)) !=
            SUCCESS)
        {
            goto uninst;
        }

        if (memcmp(out, tp[i].out, tp[i].outlen) != 0)
        {
            check = E_VERFAIL;
        }
    uninst:
        if (check != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
        }

        pufs_status_t uinst_check = SUCCESS;
        if ((uinst_check = pufs_drbg_uninstantiate()) != SUCCESS)
        {
            LOG_ERROR("Case%" PRIu32 ", %s", i, "Fail to uninstantiate drbg");
        }

        if (check != SUCCESS || uinst_check != SUCCESS)
        {
            return check != SUCCESS ? check : uinst_check;
        }
    }
    return check;
}

static pufs_status_t
pufs_drbg_test_normal(bool df)
{
    uint8_t       out[2][64];
    uint8_t       in[32];
    pufs_status_t check = SUCCESS;

    // Random input
    if ((check = pufs_rand(in, 8)) != SUCCESS)
    {
        return check;
    }

    // 2 trial
    for (uint32_t i = 0; i < 2; i++)
    {
        // instantiate + reseed + 1st generate + 2nd generate
        if (((check = pufs_drbg_instantiate(AES_CTR_DRBG, 256, df, in, 32, in, 32)) == SUCCESS) &&
            ((check = pufs_drbg_reseed(df, in, 32)) == SUCCESS) &&
            ((check = pufs_drbg_generate(out[i], 64, true, df, in, 32, false)) == SUCCESS))
        {
            check = pufs_drbg_generate(out[i], 64, true, df, in, 32, false);
        }

        pufs_status_t uinst_check = SUCCESS;
        if ((uinst_check = pufs_drbg_uninstantiate()) != SUCCESS)
        {
            LOG_ERROR("Trial%" PRIu32 ", %s", i, "Fail to uninstantiate drbg");
            check = check == SUCCESS ? uinst_check : check;
        }

        if (check != SUCCESS)
        {
            return check;
        }
    }
    // check output difference
    check = pufs_test_memcmp(out[0], out[1], 64);

    return check;
}

/*** end of file ***/
