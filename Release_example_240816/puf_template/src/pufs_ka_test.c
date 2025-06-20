/**
 * @file pufs_ka_test.c
 * @brief KA API test cases
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>

#include "pufs_ka_test.h"
#include "pufs_ka.h"
#include "pufs_kdf.h"
#include "pufs_rt.h"
#include "pufs_log.h"
#include "pufs_tv_aeskw.h"
#include "pufs_tv_aeskwp.h"

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/
static pufs_status_t pufs_aes_key_wrapping_test(const uint32_t                ntests,
                                                const struct kw_test_pattern *tp,
                                                pufs_key_wrap_t               kwptype);

/*******************************************************************************
 * Global functions
 ******************************************************************************/
pufs_status_t
pufs_aes_kw_test(void)
{
    return pufs_aes_key_wrapping_test(
        sizeof(aes_wrap_tp) / sizeof(struct kw_test_pattern), aes_wrap_tp, AES_KW);
}

pufs_status_t
pufs_aes_kw_inv_test(void)
{
    return pufs_aes_key_wrapping_test(
        sizeof(aes_wrap_tp) / sizeof(struct kw_test_pattern), aes_wrap_tp, AES_KW_INV);
}

pufs_status_t
pufs_aes_kwp_test(void)
{
    return pufs_aes_key_wrapping_test(
        sizeof(aes_wrap_pad_tp) / sizeof(struct kw_test_pattern), aes_wrap_pad_tp, AES_KWP);
}

pufs_status_t
pufs_aes_kwp_inv_test(void)
{
    return pufs_aes_key_wrapping_test(
        sizeof(aes_wrap_pad_tp) / sizeof(struct kw_test_pattern), aes_wrap_pad_tp, AES_KWP_INV);
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/
static pufs_status_t
pufs_aes_key_wrapping_test(const uint32_t                ntests,
                           const struct kw_test_pattern *tp,
                           pufs_key_wrap_t               kwptype)
{
    uint8_t      *out   = NULL;
    pufs_status_t check = SUCCESS;
    for (uint32_t i = 0; i < ntests; i++)
    {
        if (tp[i].ptlen == 0)
        {
            continue;
        }

        pufs_dgst_st   md1, md2;
        uint32_t       kekbits = tp[i].keybits;
        uint32_t       keybits = B2b(tp[i].ptlen);
        pufs_ka_slot_t kekslot = (kekbits > 128) ? SK256_2 : SK128_4;
        pufs_ka_slot_t keyslot = (keybits > 128) ? SK256_0 : SK128_0;
        out                    = malloc(tp[i].ctlen);

        if (((check = pufs_key_expansion(SSKEY,
                                         kekslot,
                                         kekbits,
                                         PRF_HMAC,
                                         SHA_256,
                                         false,
                                         NULL,
                                         0,
                                         4,
                                         PUFKEY,
                                         PUFSLOT_1,
                                         256,
                                         NULL,
                                         0)) != SUCCESS) ||
            ((check = pufs_import_plaintext_key(SSKEY, keyslot, tp[i].pt, keybits)) != SUCCESS) ||
            ((check = pufs_export_wrapped_key(
                  SSKEY, keyslot, out, keybits, kekslot, kekbits, kwptype, NULL)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot, keybits)) != SUCCESS) ||
            ((check = pufs_import_wrapped_key(
                  SSKEY, keyslot, out, keybits, kekslot, kekbits, kwptype, NULL)) != SUCCESS) ||
            ((check = pufs_hmac(&md1, NULL, 0, SHA_256, SSKEY, keyslot, keybits)) != SUCCESS) ||
            ((check = pufs_hmac(&md2, NULL, 0, SHA_256, SWKEY, tp[i].pt, keybits)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, keyslot, keybits)) != SUCCESS) ||
            ((check = pufs_clear_key(SSKEY, kekslot, kekbits)) != SUCCESS) ||
            ((check = (md1.dlen != md2.dlen ? E_VERFAIL : SUCCESS)) != SUCCESS) ||
            ((check = pufs_test_memcmp(md1.dgst, md2.dgst, md1.dlen)) != SUCCESS))
        {
            LOG_ERROR("Case%" PRIu32 " Failed.", i);
            break;
        }
    }

    free(out);
    return check;
}

/*** end of file ***/
