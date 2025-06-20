/**
 * @file pufs_sp38d.h
 * @brief SP38D API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_SP38D_H
#define PUFS_SP38D_H

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
 * @brief Initialize GCM encryptor
 *
 * @param[in] sp38d_ctx  SP38D context to be initialized.
 * @param[in] cipher     Block cipher algorithm.
 * @param[in] keytype    Key type.
 * @param[in] keyaddr    Key address.
 * @param[in] keybits    Key length in bits.
 * @param[in] iv         Initial vector.
 * @param[in] ivlen      Initial vector length in bytes.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_enc_gcm_init(sp38d_ctx, cipher, keytype, keyaddr, keybits, iv, ivlen) \
    _pufs_enc_gcm_init(sp38d_ctx, cipher, keytype, (size_t)keyaddr, keybits, iv, ivlen)

/**
 * @brief Encryption using GCM mode.
 *
 * @param[out] out      Output data.
 * @param[out] outlen   Output data length in bytes.
 * @param[in]  in       Input data.
 * @param[in]  inlen    Input data length in bytes.
 * @param[in]  cipher   Block cipher algorithm.
 * @param[in]  keytype  Key type.
 * @param[in]  keyaddr  Key address.
 * @param[in]  keybits  Key length in bits.
 * @param[in]  iv       Initial vector.
 * @param[in]  ivlen    Initial vector length in bytes.
 * @param[in]  aad      Additional authentication data.
 * @param[in]  aadlen   Additional authentication data length in bytes.
 * @param[out] tag      Output tag.
 * @param[in]  taglen   Specified output tag length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_enc_gcm(out,          \
                     outlen,       \
                     in,           \
                     inlen,        \
                     cipher,       \
                     keytype,      \
                     keyaddr,      \
                     keybits,      \
                     iv,           \
                     ivlen,        \
                     aad,          \
                     aadlen,       \
                     tag,          \
                     taglen)       \
    _pufs_enc_gcm(out,             \
                  outlen,          \
                  in,              \
                  inlen,           \
                  cipher,          \
                  keytype,         \
                  (size_t)keyaddr, \
                  keybits,         \
                  iv,              \
                  ivlen,           \
                  aad,             \
                  aadlen,          \
                  tag,             \
                  taglen)

/**
 * @brief Initialize GCM decryptor
 *
 * @param[in] sp38d_ctx  SP38D context to be initialized.
 * @param[in] cipher     Block cipher algorithm.
 * @param[in] keytype    Key type.
 * @param[in] keyaddr    Key address.
 * @param[in] keybits    Key length in bits.
 * @param[in] iv         Initial vector.
 * @param[in] ivlen      Initial vector length in bytes.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_dec_gcm_init(sp38d_ctx, cipher, keytype, keyaddr, keybits, iv, ivlen) \
    _pufs_dec_gcm_init(sp38d_ctx, cipher, keytype, (size_t)keyaddr, keybits, iv, ivlen)

/**
 * @brief Decryption using GCM mode.
 *
 * @param[out] out      Output data.
 * @param[out] outlen   Output data length in bytes.
 * @param[in]  in       Input data.
 * @param[in]  inlen    Input data length in bytes.
 * @param[in]  cipher   Block cipher algorithm.
 * @param[in]  keytype  Key type.
 * @param[in]  keyaddr  Key address.
 * @param[in]  keybits  Key length in bits.
 * @param[in]  iv       Initial vector.
 * @param[in]  ivlen    Initial vector length in bytes.
 * @param[in]  aad      Additional authentication data.
 * @param[in]  aadlen   Additional authentication data length in bytes.
 * @param[in]  tag      Input tag.
 * @param[in]  taglen   Specified input tag length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_dec_gcm(out,          \
                     outlen,       \
                     in,           \
                     inlen,        \
                     cipher,       \
                     keytype,      \
                     keyaddr,      \
                     keybits,      \
                     iv,           \
                     ivlen,        \
                     aad,          \
                     aadlen,       \
                     tag,          \
                     taglen)       \
    _pufs_dec_gcm(out,             \
                  outlen,          \
                  in,              \
                  inlen,           \
                  cipher,          \
                  keytype,         \
                  (size_t)keyaddr, \
                  keybits,         \
                  iv,              \
                  ivlen,           \
                  aad,             \
                  aadlen,          \
                  tag,             \
                  taglen)

typedef enum
{
    GCM_AAD,
    GCM_PLAINTEXT,
} pufs_gcm_input_data_t;

/*******************************************************************************
 * Typedefs
 ******************************************************************************/
typedef struct pufs_sp38d_context pufs_sp38d_ctx;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Obtain a pointer to SP38D internal context
 *
 * @return A pointer to SP38D internal context, or NULL if error
 */
pufs_sp38d_ctx *pufs_sp38d_ctx_new(void);

/**
 * @brief Free a pointer to SP38D internal context
 *
 * @param[in] sp38d_ctx  A pointer to SP38D context.
 */
void pufs_sp38d_ctx_free(pufs_sp38d_ctx *sp38d_ctx);

/**
 * @brief GCM encryptor initializer with keyaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_enc_gcm_init() instead.
 */
pufs_status_t _pufs_enc_gcm_init(pufs_sp38d_ctx *sp38d_ctx,
                                 pufs_cipher_t   cipher,
                                 pufs_key_type_t keytype,
                                 size_t          keyaddr,
                                 uint32_t        keybits,
                                 const uint8_t  *iv,
                                 uint32_t        ivlen);

/**
 * @brief Input data into GCM encryptor
 *
 * @param[in]  sp38d_ctx  SP38D context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[in]  in         Input data.
 * @param[in]  inlen      Input data length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 *
 * @note Input data may be either AAD or plaintext. Set \a out to NULL when
 *       input AAD. Complete AAD data must be passed first before the plaintext.
 */
pufs_status_t pufs_enc_gcm_update(
    pufs_sp38d_ctx *sp38d_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *in, uint32_t inlen);

/**
 * @brief Finalize GCM encryptor
 *
 * @param[in]  sp38d_ctx  SP38D context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[out] tag        Output tag.
 * @param[in]  taglen     Specified output tag length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_enc_gcm_final(
    pufs_sp38d_ctx *sp38d_ctx, uint8_t *out, uint32_t *outlen, uint8_t *tag, uint32_t taglen);

/**
 * @brief Encryption using GCM mode with keyaddr type casting.
 *
 * @warning DO NOT call this function directly. Use pufs_enc_gcm() instead.
 */
pufs_status_t _pufs_enc_gcm(uint8_t        *out,
                            uint32_t       *outlen,
                            const uint8_t  *in,
                            uint32_t        inlen,
                            pufs_cipher_t   cipher,
                            pufs_key_type_t keytype,
                            size_t          keyaddr,
                            uint32_t        keybits,
                            const uint8_t  *iv,
                            uint32_t        ivlen,
                            const uint8_t  *aad,
                            uint32_t        aadlen,
                            uint8_t        *tag,
                            uint32_t        taglen);

/**
 * @brief GCM decryptor initializer with keyaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_dec_gcm_init() instead.
 */
pufs_status_t _pufs_dec_gcm_init(pufs_sp38d_ctx *sp38d_ctx,
                                 pufs_cipher_t   cipher,
                                 pufs_key_type_t keytype,
                                 size_t          keyaddr,
                                 uint32_t        keybits,
                                 const uint8_t  *iv,
                                 uint32_t        ivlen);

/**
 * @brief Input data into GCM decryptor
 *
 * @param[in]  sp38d_ctx  SP38D context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[in]  in         Input data.
 * @param[in]  inlen      Input data length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 *
 * @note Input data may be either AAD or plaintext. Set \a out to NULL when
 *       input AAD. Complete AAD data must be passed first before the plaintext.
 */
pufs_status_t pufs_dec_gcm_update(
    pufs_sp38d_ctx *sp38d_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *in, uint32_t inlen);

/**
 * @brief Finalize GCM decryptor with tag output
 *
 * @param[in]  sp38d_ctx  SP38D context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[out] tag        Output tag.
 * @param[in]  taglen     Specified input tag length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_dec_gcm_final_tag(
    pufs_sp38d_ctx *sp38d_ctx, uint8_t *out, uint32_t *outlen, uint8_t *tag, uint32_t taglen);

/**
 * @brief Finalize GCM decryptor
 *
 * @param[in]  sp38d_ctx  SP38D context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[in]  tag        Input tag.
 * @param[in]  taglen     Specified input tag length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_dec_gcm_final(
    pufs_sp38d_ctx *sp38d_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *tag, uint32_t taglen);

/**
 * @brief Decryption using GCM mode with keyaddr type casting.
 *
 * @warning DO NOT call this function directly. Use pufs_dec_gcm() instead.
 */
pufs_status_t _pufs_dec_gcm(uint8_t        *out,
                            uint32_t       *outlen,
                            const uint8_t  *in,
                            uint32_t        inlen,
                            pufs_cipher_t   cipher,
                            pufs_key_type_t keytype,
                            size_t          keyaddr,
                            uint32_t        keybits,
                            const uint8_t  *iv,
                            int             ivlen,
                            const uint8_t  *aad,
                            int             aadlen,
                            const uint8_t  *tag,
                            int             taglen);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_SP38D_H */

/*** end of file ***/
