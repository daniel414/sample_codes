/**
 * @file pufs_hmac.h
 * @brief HMAC API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_HMAC_H
#define PUFS_HMAC_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include "pufs_common.h"
#include "pufs_ka.h"
#include "pufs_dma.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define pufs_hash_ctx_new()          pufs_hmac_ctx_new()
#define pufs_hash_ctx_free(hmac_ctx) pufs_hmac_ctx_free(hmac_ctx)

/**
 * @brief Initialize HMAC calculator
 *
 * @param[in] hmac_ctx  HMAC context.
 * @param[in] hash      Hash algorithm.
 * @param[in] keytype   Key type.
 * @param[in] keyaddr   Key address.
 * @param[in] keybits   Key length in bits.
 * @return              SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_hmac_init(hmac_ctx, hash, keytype, keyaddr, keybits) \
    _pufs_hmac_init(hmac_ctx, hash, keytype, (size_t)keyaddr, keybits)

/**
 * @brief Calculate HMAC hash value of a message with a key.
 *
 * @param[out] md       Message digest.
 * @param[in]  msg      Message.
 * @param[in]  msglen   Message length in bytes.
 * @param[in]  hash     Hash algorithm.
 * @param[in]  keytype  Key type.
 * @param[in]  keyaddr  Key address.
 * @param[in]  keybits  Key length in bits.
 * @return              SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_hmac(md, msg, msglen, hash, keytype, keyaddr, keybits) \
    _pufs_hmac(md, msg, msglen, hash, keytype, (size_t)keyaddr, keybits)

/**
 * @brief Cryptographic hash algorithms
 */
typedef enum
{
    MD5,                    /**< MD5 */
    SHA1,                   /**< SHA1 */
    SHA_224,                /**< SHA2-224 */
    SHA_256,                /**< SHA2-256 */
    HASH_DEFAULT = SHA_256, /**< Default to SHA2-256 */
    SHA_384,                /**< SHA2-384 */
    SHA_512,                /**< SHA2-512 */
    SHA_512_224,            /**< SHA2-512/224 */
    SHA_512_256,            /**< SHA2-512/256 */
    SM3,                    /**< SM3 */
    SHA3_224,               /**< SHA3-224 */
    SHA3_256,               /**< SHA3-256 */
    SHA3_384,               /**< SHA3-384 */
    SHA3_512,               /**< SHA3-512 */
    SHAKE_128,              /**< SHAKE-128 */
    SHAKE_256,              /**< SHAKE-256 */
    N_HASH_T,               // keep in the last one
} pufs_hash_t;

/**
 * @brief KMAC variants
 */
typedef enum
{
    KMAC_128, /**< KMAC-128 */
    KMAC_256, /**< KMAC-256 */
    N_KMAC_T, // keep in the last one
} pufs_kmac_t;

/*******************************************************************************
 * Typedefs
 ******************************************************************************/
typedef struct pufs_hmac_context pufs_hmac_ctx;
typedef pufs_hmac_ctx            pufs_hash_ctx;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Obtain a pointer to HMAC internal context
 *
 * @return A pointer to HMAC internal context, or NULL if error
 */
pufs_hmac_ctx *pufs_hmac_ctx_new(void);

/**
 * @brief Free a pointer to HMAC internal context
 *
 * @param[in] hmac_ctx  A pointer to HMAC context.
 */
void pufs_hmac_ctx_free(pufs_hmac_ctx *hmac_ctx);

/**
 * @brief Initialize hash calculator
 *
 * @param[in] hash_ctx  Hash context.
 * @param[in] hash      Hash algorithm.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_hash_init(pufs_hash_ctx *hash_ctx, pufs_hash_t hash);

/**
 * @brief Input data into hash calculator
 *
 * @param[in] hash_ctx  Hash context.
 * @param[in] msg       Message.
 * @param[in] msglen    Message length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_hash_update(pufs_hash_ctx *hash_ctx, const uint8_t *msg, uint32_t msglen);

/**
 * @brief Extract message digest from hash calculator
 *
 * @param[in]  hash_ctx  Hash context.
 * @param[out] md        Message digest.
 * @return               SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_hash_final(pufs_hash_ctx *hash_ctx, pufs_dgst_st *md);

/**
 * @brief Calculate hash value of a message.
 *
 * @param[out] md      Message digest.
 * @param[in]  msg     Message.
 * @param[in]  msglen  Message length in bytes.
 * @param[in]  hash    Hash algorithm.
 * @return             SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_hash(pufs_dgst_st *md, const uint8_t *msg, uint32_t msglen, pufs_hash_t hash);

/**
 * @brief HMAC calculator initializer with keyaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_hmac_init() instead.
 */
pufs_status_t _pufs_hmac_init(pufs_hmac_ctx  *hmac_ctx,
                              pufs_hash_t     hash,
                              pufs_key_type_t keytype,
                              size_t          keyaddr,
                              uint32_t        keybits);

/**
 * @brief Input data into HMAC calculator
 *
 * @param[in] hmac_ctx  HMAC context.
 * @param[in] msg     Message.
 * @param[in] msglen  Message length in bytes.
 * @return            SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_hmac_update(pufs_hmac_ctx *hmac_ctx, const uint8_t *msg, uint32_t msglen);

/**
 * @brief Extract message digest from HMAC calculator
 *
 * @param[in]  hmac_ctx  HMAC context.
 * @param[out] md        Message digest.
 * @return               SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_hmac_final(pufs_hmac_ctx *hmac_ctx, pufs_dgst_st *md);

/**
 * @brief HMAC calculator with keyaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_hmac() instead.
 */
pufs_status_t _pufs_hmac(pufs_dgst_st   *md,
                         const uint8_t  *msg,
                         uint32_t        msglen,
                         pufs_hash_t     hash,
                         pufs_key_type_t keytype,
                         size_t          keyaddr,
                         uint32_t        keybits);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_HMAC_H */

/*** end of file ***/
