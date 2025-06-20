/**
 * @file pufs_cmac.h
 * @brief CMAC API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_CMAC_H
#define PUFS_CMAC_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include "pufs_common.h"
#include "pufs_ka.h"
#include "pufs_dma.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
/**
 * @brief Initialize CMAC calculator
 *
 * @param[in] cmac_ctx  CMAC context to be initialized.
 * @param[in] cipher    Block cipher algorithm.
 * @param[in] keytype   Key type.
 * @param[in] keyaddr   Key address.
 * @param[in] keybits   Key length in bits.
 * @return              SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_cmac_init(cmac_ctx, cipher, keytype, keyaddr, keybits) \
    _pufs_cmac_init(cmac_ctx, cipher, keytype, (size_t)keyaddr, keybits)

/**
 * @brief Calculate CMAC hash value of a message with a key.
 *
 * @param[out] md       Message digest.
 * @param[in]  msg      Message.
 * @param[in]  msglen   Message length in bytes.
 * @param[in]  cipher   Block cipher algorithm.
 * @param[in]  keytype  Key type.
 * @param[in]  keyaddr  Key address.
 * @param[in]  keybits  Key length in bits.
 * @return              SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_cmac(md, msg, msglen, cipher, keytype, keyaddr, keybits) \
    _pufs_cmac(md, msg, msglen, cipher, keytype, (size_t)keyaddr, keybits)

/*******************************************************************************
 * Typedefs
 ******************************************************************************/
typedef struct pufs_cmac_context pufs_cmac_ctx;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Obtain a pointer to CMAC internal context
 *
 * @return A pointer to CMAC internal context, or NULL if error
 */
pufs_cmac_ctx *pufs_cmac_ctx_new(void);

/**
 * @brief Free a pointer to CMAC internal context
 *
 * @param[in] cmac_ctx  A pointer to CMAC context.
 */
void pufs_cmac_ctx_free(pufs_cmac_ctx *cmac_ctx);

/**
 * @brief CMAC calculator initializer with keyaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_cmac_init() instead.
 */
pufs_status_t _pufs_cmac_init(pufs_cmac_ctx  *cmac_ctx,
                              pufs_cipher_t   cipher,
                              pufs_key_type_t keytype,
                              size_t          keyaddr,
                              uint32_t        keybits);

/**
 * @brief Input data into CMAC calculator
 *
 * @param[in] cmac_ctx  CMAC context.
 * @param[in] msg       Message.
 * @param[in] msglen    Message length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_cmac_update(pufs_cmac_ctx *cmac_ctx, const uint8_t *msg, uint32_t msglen);

/**
 * @brief Extract message digest from CMAC calculator
 *
 * @param[in] cmac_ctx  CMAC context.
 * @param[out] md       Message digest.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_cmac_final(pufs_cmac_ctx *cmac_ctx, pufs_dgst_st *md);

/**
 * @brief CMAC calculator with keyaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_cmac() instead.
 */
pufs_status_t _pufs_cmac(pufs_dgst_st   *md,
                         const uint8_t  *msg,
                         uint32_t        msglen,
                         pufs_cipher_t   cipher,
                         pufs_key_type_t keytype,
                         size_t          keyaddr,
                         uint32_t        keybits);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_CMAC_H */

/*** end of file ***/
