/**
 * @file pufs_sp90a.h
 * @brief SP90A API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_SP90A_H
#define PUFS_SP90A_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_common.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
/**
 * @brief DRBG underlying cryptographic primitives
 */
typedef enum
{
    AES_CTR_DRBG, /**< DRBG mechanisms based on AES-CTR mode */
    HASH_DRBG,    /**< DRBG mechanisms based on Hash functions */
    HMAC_DRBG,    /**< DRBG mechanisms based on HMAC functions */
} pufs_drbg_t;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Instantiate DRBG
 *
 * @param[in] mode      DRBG underlying cryptographic primitive
 * @param[in] security  Required security strength in bits.
 * @param[in] df        Use derivation functi on or not.
 * @param[in] nonce     Nonce.
 * @param[in] noncelen  Length of nonce in bytes.
 * @param[in] pstr      Personalization string.
 * @param[in] pstrlen   Length of personalization string in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_drbg_instantiate(pufs_drbg_t    mode,
                                    uint32_t       security,
                                    bool           df,
                                    const uint8_t *nonce,
                                    uint32_t       noncelen,
                                    const uint8_t *pstr,
                                    uint32_t       pstrlen);

/**
 * @brief Reseed DRBG
 *
 * @param[in] df       Use derivation function or not.
 * @param[in] adin     Additional input.
 * @param[in] adinlen  Length of additional input in bytes.
 * @return             SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_drbg_reseed(bool df, const uint8_t *adin, uint32_t adinlen);

/**
 * @brief Generate random bits from DRBG
 *
 * @param[out] out       Random bits.
 * @param[in]  outbits   Length of output in bits.
 * @param[in]  pr        Prediction resistance request.
 * @param[in]  df        Use derivation function or not.
 * @param[in]  adin      Additional input.
 * @param[in]  adinlen   Length of additional input in bytes.
 * @param[in]  testmode  In test mode or not.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note error reported if \em testmode is false but DRBG test mode is enabled.
 */
pufs_status_t pufs_drbg_generate(uint8_t       *out,
                                 uint32_t       outbits,
                                 bool           pr,
                                 bool           df,
                                 const uint8_t *adin,
                                 uint32_t       adinlen,
                                 uint32_t       testmode);

/**
 * @brief Uninstantiate DRBG
 *
 * @return  SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_drbg_uninstantiate(void);

/**
 * @brief Check if SP90A hardware is in test mode.
 *
 * @return  True if SP90A is in test mode; false otherwise.
 */
bool pufs_drbg_is_testmode(void);

/**
 * @brief Enable DRBG test mode
 */
void pufs_drbg_enable_testmode(void);

/**
 * @brief Set entropy for test mode
 *
 * @param[in] entropy  Entropy.
 * @param[in] entlen   Entropy length in bytes.
 */
void pufs_drbg_testmode_entropy(const uint8_t *entropy, uint32_t entlen);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_SP90A_H */

/*** end of file ***/
