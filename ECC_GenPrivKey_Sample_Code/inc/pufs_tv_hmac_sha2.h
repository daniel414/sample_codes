/**
 * @file pufs_tv_hmac_sha2.h
 * @brief test vectors for HMAC-SHA2
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_TV_HMAC_SHA2_H
#define PUFS_TV_HMAC_SHA2_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_test_common.h"
#include "pufs_hmac.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
struct hmac_test_pattern
{
    pufs_hash_t hash;
    uint32_t    keybits;
    const void *key;
    uint32_t    msglen;
    const void *msg;
    const void *md;
};

/*******************************************************************************
 * Exported variables
 ******************************************************************************/
extern const struct hmac_test_pattern g_hmac_sha2_tp[];
extern uint32_t g_hmac_sha2_tp_count;

#ifdef __cplusplus
}
#endif

#endif /* PUFS_TV_HMAC_SHA2_H */

/*** end of file ***/
