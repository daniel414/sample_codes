/**
 * @file pufs_sp38c.h
 * @brief SP38C API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_SP38C_H
#define PUFS_SP38C_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_common.h"
#include "pufs_dma.h"
#include "pufs_ka.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
/**
 * @brief Initialize CCM encryptor
 *
 * @param[in] sp38c_ctx  SP38C context to be initialized.
 * @param[in] cipher     Block cipher algorithm.
 * @param[in] keytype    Key type.
 * @param[in] keyaddr    Key address.
 * @param[in] keybits    Key length in bits.
 * @param[in] nonce      Nonce.
 * @param[in] noncelen   Nonce length in bytes.
 * @param[in] aadlen     AAD length in bytes.
 * @param[in] inlen      Payload length in bytes.
 * @param[in] taglen     Tag length in bytes.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_enc_ccm_init(                                                                \
    sp38c_ctx, cipher, keytype, keyaddr, keybits, nonce, noncelen, aadlen, inlen, taglen) \
    _pufs_enc_ccm_init(sp38c_ctx,                                                         \
                       cipher,                                                            \
                       keytype,                                                           \
                       (size_t)keyaddr,                                                   \
                       keybits,                                                           \
                       nonce,                                                             \
                       noncelen,                                                          \
                       aadlen,                                                            \
                       inlen,                                                             \
                       taglen)

/**
 * @brief Encryption using CCM mode.
 *
 * @param[out] out       Output data.
 * @param[out] outlen    Output data length in bytes.
 * @param[in]  in        Input data.
 * @param[in]  inlen     Input data length in bytes.
 * @param[in]  cipher    Block cipher algorithm.
 * @param[in]  keytype   Key type.
 * @param[in]  keyaddr   Key address.
 * @param[in]  keybits   Key length in bits.
 * @param[in]  nonce     Nonce.
 * @param[in]  noncelen  Nonce length in bytes.
 * @param[in]  aad       Additional authentication data.
 * @param[in]  aadlen    Additional authentication data length in bytes.
 * @param[out] tag       Output tag.
 * @param[in]  taglen    Specified output tag length in bytes.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_enc_ccm(out,          \
                     outlen,       \
                     in,           \
                     inlen,        \
                     cipher,       \
                     keytype,      \
                     keyaddr,      \
                     keybits,      \
                     nonce,        \
                     noncelen,     \
                     aad,          \
                     aadlen,       \
                     tag,          \
                     taglen)       \
    _pufs_enc_ccm(out,             \
                  outlen,          \
                  in,              \
                  inlen,           \
                  cipher,          \
                  keytype,         \
                  (size_t)keyaddr, \
                  keybits,         \
                  nonce,           \
                  noncelen,        \
                  aad,             \
                  aadlen,          \
                  tag,             \
                  taglen)

/**
 * @brief Initialize CCM decryptor
 *
 * @param[in] sp38c_ctx  SP38C context to be initialized.
 * @param[in] cipher     Block cipher algorithm.
 * @param[in] keytype    Key type.
 * @param[in] keyaddr    Key address.
 * @param[in] keybits    Key length in bits.
 * @param[in] nonce      Nonce.
 * @param[in] noncelen   Nonce length in bytes.
 * @param[in] aadlen     AAD length in bytes.
 * @param[in] inlen      Payload length in bytes.
 * @param[in] taglen     Tag length in bytes.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_dec_ccm_init(                                                                \
    sp38c_ctx, cipher, keytype, keyaddr, keybits, nonce, noncelen, aadlen, inlen, taglen) \
    _pufs_dec_ccm_init(sp38c_ctx,                                                         \
                       cipher,                                                            \
                       keytype,                                                           \
                       (size_t)keyaddr,                                                   \
                       keybits,                                                           \
                       nonce,                                                             \
                       noncelen,                                                          \
                       aadlen,                                                            \
                       inlen,                                                             \
                       taglen)

/**
 * @brief Decryption using CCM mode.
 *
 * @param[out] out       Output data.
 * @param[out] outlen    Output data length in bytes.
 * @param[in]  in        Input data.
 * @param[in]  inlen     Input data length in bytes.
 * @param[in]  cipher    Block cipher algorithm.
 * @param[in]  keytype   Key type.
 * @param[in]  keyaddr   Key address.
 * @param[in]  keybits   Key length in bits.
 * @param[in]  nonce     Nonce.
 * @param[in]  noncelen  Nonce length in bytes.
 * @param[in]  aad       Additional authentication data.
 * @param[in]  aadlen    Additional authentication data length in bytes.
 * @param[in]  tag       Input tag.
 * @param[in]  taglen    Specified input tag length in bytes.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_dec_ccm(out,          \
                     outlen,       \
                     in,           \
                     inlen,        \
                     cipher,       \
                     keytype,      \
                     keyaddr,      \
                     keybits,      \
                     nonce,        \
                     noncelen,     \
                     aad,          \
                     aadlen,       \
                     tag,          \
                     taglen)       \
    _pufs_dec_ccm(out,             \
                  outlen,          \
                  in,              \
                  inlen,           \
                  cipher,          \
                  keytype,         \
                  (size_t)keyaddr, \
                  keybits,         \
                  nonce,           \
                  noncelen,        \
                  aad,             \
                  aadlen,          \
                  tag,             \
                  taglen)

typedef enum
{
    CCM_AAD,
    CCM_PLAINTEXT,
} pufs_ccm_data_t;

/*******************************************************************************
 * Typedefs
 ******************************************************************************/
typedef struct pufs_sp38c_context pufs_sp38c_ctx;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Obtain a pointer to SP38C internal context
 *
 * @return A pointer to SP38C internal context, or NULL if error
 */
pufs_sp38c_ctx *pufs_sp38c_ctx_new(void);

/**
 * @brief Free a pointer to SP38C internal context
 *
 * @param[in] sp38c_ctx  A pointer to SP38C context.
 */
void pufs_sp38c_ctx_free(pufs_sp38c_ctx *sp38c_ctx);

/**
 * @brief CCM encryptor initializer with keyaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_enc_ccm_init() instead.
 */
pufs_status_t _pufs_enc_ccm_init(pufs_sp38c_ctx *sp38c_ctx,
                                 pufs_cipher_t   cipher,
                                 pufs_key_type_t keytype,
                                 size_t          keyaddr,
                                 uint32_t        keybits,
                                 const uint8_t  *nonce,
                                 uint32_t        noncelen,
                                 uint64_t        aadlen,
                                 uint64_t        inlen,
                                 uint32_t        taglen);

/**
 * @brief Input data into CCM encryptor
 *
 * @param[in]  sp38c_ctx  SP38C context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[in]  in         Input data.
 * @param[in]  inlen      Input data length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 *
 * @note Input data may be either AAD or plaintext. Set \em out to NULL when
 *       input AAD. Complete AAD data must be passed first before the plaintext.
 */
pufs_status_t pufs_enc_ccm_update(
    pufs_sp38c_ctx *sp38c_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *in, uint32_t inlen);

/**
 * @brief Finalize CCM encryptor
 *
 * @param[in]  sp38c_ctx  SP38C context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[out] tag        Output tag.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_enc_ccm_final(pufs_sp38c_ctx *sp38c_ctx,
                                 uint8_t        *out,
                                 uint32_t       *outlen,
                                 uint8_t        *tag);

/**
 * @brief Encryption using CCM mode with keyaddr type casting.
 *
 * @warning DO NOT call this function directly. Use pufs_enc_ccm() instead.
 */
pufs_status_t _pufs_enc_ccm(uint8_t        *out,
                            uint32_t       *outlen,
                            const uint8_t  *in,
                            uint32_t        inlen,
                            pufs_cipher_t   cipher,
                            pufs_key_type_t keytype,
                            size_t          keyaddr,
                            uint32_t        keybits,
                            const uint8_t  *nonce,
                            uint32_t        noncelen,
                            const uint8_t  *aad,
                            uint32_t        aadlen,
                            uint8_t        *tag,
                            uint32_t        taglen);

/**
 * @brief CCM decryptor initializer with keyaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_dec_ccm_init() instead.
 */
pufs_status_t _pufs_dec_ccm_init(pufs_sp38c_ctx *sp38c_ctx,
                                 pufs_cipher_t   cipher,
                                 pufs_key_type_t keytype,
                                 size_t          keyaddr,
                                 uint32_t        keybits,
                                 const uint8_t  *nonce,
                                 uint32_t        noncelen,
                                 uint64_t        aadlen,
                                 uint64_t        inlen,
                                 uint32_t        taglen);

/**
 * @brief Input data into CCM decryptor
 *
 * @param[in]  sp38c_ctx  SP38C context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[in]  in         Input data.
 * @param[in]  inlen      Input data length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 *
 * @note Input data may be either AAD or plaintext. Set \em out to NULL when
 *       input AAD. Complete AAD data must be passed first before the plaintext.
 */
pufs_status_t pufs_dec_ccm_update(
    pufs_sp38c_ctx *sp38c_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *in, uint32_t inlen);

/**
 * @brief Finalize CCM decryptor with tag output
 *
 * @param[in]  sp38c_ctx  SP38C context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[out] tag        Output tag.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_dec_ccm_final_tag(pufs_sp38c_ctx *sp38c_ctx,
                                     uint8_t        *out,
                                     uint32_t       *outlen,
                                     uint8_t        *tag);

/**
 * @brief Finalize CCM decryptor with tag checking
 *
 * @param[in]  sp38c_ctx  SP38C context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[in]  tag        Input tag.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_dec_ccm_final(pufs_sp38c_ctx *sp38c_ctx,
                                 uint8_t        *out,
                                 uint32_t       *outlen,
                                 const uint8_t  *tag);

/**
 * @brief Decryption using CCM mode with keyaddr type casting.
 *
 * @warning DO NOT call this function directly. Use pufs_dec_ccm() instead.
 */
pufs_status_t _pufs_dec_ccm(uint8_t        *out,
                            uint32_t       *outlen,
                            const uint8_t  *in,
                            uint32_t        inlen,
                            pufs_cipher_t   cipher,
                            pufs_key_type_t keytype,
                            size_t          keyaddr,
                            uint32_t        keybits,
                            const uint8_t  *nonce,
                            int             noncelen,
                            const uint8_t  *aad,
                            int             aadlen,
                            const uint8_t  *tag,
                            int             taglen);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_SP38C_H */

/*** end of file ***/
