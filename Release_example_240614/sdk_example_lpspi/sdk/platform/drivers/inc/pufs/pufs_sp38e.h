/**
 * @file pufs_sp38e.h
 * @brief SP38E API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_SP38E_H
#define PUFS_SP38E_H

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
 * @brief Initialize XTS encryptor
 *
 * @param[in] sp38e_ctx  SP38E context to be initialized.
 * @param[in] cipher     Block cipher algorithm.
 * @param[in] keytype1   Key1 key type.
 * @param[in] keyaddr1   Key1 key address.
 * @param[in] keybits    Each key length in bits.
 * @param[in] keytype2   Key2 key type.
 * @param[in] keyaddr2   Key2 key address.
 * @param[in] i          Tweak value.
 * @param[in] j          Sequence number of the first input data.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_enc_xts_init(                                                    \
    sp38e_ctx, cipher, keytype1, keyaddr1, keybits, keytype2, keyaddr2, i, j) \
    _pufs_enc_xts_init(                                                       \
        sp38e_ctx, cipher, keytype1, (size_t)keyaddr1, keybits, keytype2, (size_t)keyaddr2, i, j)

/**
 * @brief Encryption using XTS mode.
 *
 * @param[out] out       Output data.
 * @param[out] outlen    Output data length in bytes.
 * @param[in]  in        Input data.
 * @param[in]  inlen     Input data length in bytes.
 * @param[in]  cipher    Block cipher algorithm.
 * @param[in]  keytype1  Key1 key type.
 * @param[in]  keyaddr1  Key1 key address.
 * @param[in]  keybits   Each key length in bits.
 * @param[in]  keytype2  Key2 key type.
 * @param[in]  keyaddr2  Key2 key address.
 * @param[in]  i         Tweak value.
 * @param[in]  j         Sequence number of the first input data.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_enc_xts(                                                                      \
    out, outlen, in, inlen, cipher, keytype1, keyaddr1, keybits, keytype2, keyaddr2, i, j) \
    _pufs_enc_xts(out,                                                                     \
                  outlen,                                                                  \
                  in,                                                                      \
                  inlen,                                                                   \
                  cipher,                                                                  \
                  keytype1,                                                                \
                  (size_t)keyaddr1,                                                        \
                  keybits,                                                                 \
                  keytype2,                                                                \
                  (size_t)keyaddr2,                                                        \
                  i,                                                                       \
                  j)

/**
 * @brief Initialize XTS decryptor
 *
 * @param[in] sp38e_ctx  SP38E context to be initialized.
 * @param[in] cipher     Block cipher algorithm.
 * @param[in] keytype1   Key1 key type.
 * @param[in] keyaddr1   Key1 key address.
 * @param[in] keybits    Each key length in bits.
 * @param[in] keytype2   Key2 key type.
 * @param[in] keyaddr2   Key2 key address.
 * @param[in] i          Tweak value.
 * @param[in] j          Sequence number of the first input data.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_dec_xts_init(                                                    \
    sp38e_ctx, cipher, keytype1, keyaddr1, keybits, keytype2, keyaddr2, i, j) \
    _pufs_dec_xts_init(                                                       \
        sp38e_ctx, cipher, keytype1, (size_t)keyaddr1, keybits, keytype2, (size_t)keyaddr2, i, j)

/**
 * @brief Decryption using XTS mode.
 *
 * @param[out] out       Output data.
 * @param[out] outlen    Output data length in bytes.
 * @param[in]  in        Input data.
 * @param[in]  inlen     Input data length in bytes.
 * @param[in]  cipher    Block cipher algorithm.
 * @param[in]  keytype1  Key1 key type.
 * @param[in]  keyaddr1  Key1 key address.
 * @param[in]  keybits   Each key length in bits.
 * @param[in]  keytype2  Key2 key type.
 * @param[in]  keyaddr2  Key2 key address.
 * @param[in]  i         Tweak value.
 * @param[in]  j         Sequence number of the first input data.
 * @return               SUCCESS on success, otherwise an error code.
 *
 * @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em keytype setting.
 * @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
 */
#define pufs_dec_xts(                                                                      \
    out, outlen, in, inlen, cipher, keytype1, keyaddr1, keybits, keytype2, keyaddr2, i, j) \
    _pufs_dec_xts(out,                                                                     \
                  outlen,                                                                  \
                  in,                                                                      \
                  inlen,                                                                   \
                  cipher,                                                                  \
                  keytype1,                                                                \
                  (size_t)keyaddr1,                                                        \
                  keybits,                                                                 \
                  keytype2,                                                                \
                  (size_t)keyaddr2,                                                        \
                  i,                                                                       \
                  j)

/*******************************************************************************
 * Typedefs
 ******************************************************************************/
typedef struct pufs_sp38e_context pufs_sp38e_ctx;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Obtain a pointer to SP38E internal context
 *
 * @return A pointer to SP38E internal context, or NULL if error
 */
pufs_sp38e_ctx *pufs_sp38e_ctx_new(void);

/**
 * @brief Free a pointer to SP38E internal context
 *
 * @param[in] sp38e_ctx  A pointer to SP38E context.
 */
void pufs_sp38e_ctx_free(pufs_sp38e_ctx *sp38e_ctx);

/**
 * @brief XTS encryptor initializer with keyaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_enc_xts_init() instead.
 */
pufs_status_t _pufs_enc_xts_init(pufs_sp38e_ctx *sp38e_ctx,
                                 pufs_cipher_t   cipher,
                                 pufs_key_type_t keytype1,
                                 size_t          keyaddr1,
                                 uint32_t        keybits,
                                 pufs_key_type_t keytype2,
                                 size_t          keyaddr2,
                                 const uint8_t  *i,
                                 uint32_t        j);

/**
 * @brief Input data into XTS encryptor
 *
 * @param[in]  sp38e_ctx  SP38E context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[in]  in         Input data.
 * @param[in]  inlen      Input data length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_enc_xts_update(
    pufs_sp38e_ctx *sp38e_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *in, uint32_t inlen);

/**
 * @brief Finalize XTS encryptor
 *
 * @param[in]  sp38e_ctx  SP38E context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_enc_xts_final(pufs_sp38e_ctx *sp38e_ctx, uint8_t *out, uint32_t *outlen);

/**
 * @brief Encryption using XTS mode with keyaddr type casting.
 *
 * @warning DO NOT call this function directly. Use pufs_enc_xts() instead.
 */
pufs_status_t _pufs_enc_xts(uint8_t        *out,
                            uint32_t       *outlen,
                            const uint8_t  *in,
                            uint32_t        inlen,
                            pufs_cipher_t   cipher,
                            pufs_key_type_t keytype1,
                            size_t          keyaddr1,
                            uint32_t        keybits,
                            pufs_key_type_t keytype2,
                            size_t          keyaddr2,
                            const uint8_t  *i,
                            uint32_t        j);

/**
 * @brief XTS decryptor initializer with keyaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_dec_xts_init() instead.
 */
pufs_status_t _pufs_dec_xts_init(pufs_sp38e_ctx *sp38e_ctx,
                                 pufs_cipher_t   cipher,
                                 pufs_key_type_t keytype1,
                                 size_t          keyaddr1,
                                 uint32_t        keybits,
                                 pufs_key_type_t keytype2,
                                 size_t          keyaddr2,
                                 const uint8_t  *i,
                                 uint32_t        j);

/**
 * @brief Input data into XTS decryptor
 *
 * @param[in]  sp38e_ctx  SP38E context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[in]  in         Input data.
 * @param[in]  inlen      Input data length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_dec_xts_update(
    pufs_sp38e_ctx *sp38e_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *in, uint32_t inlen);

/**
 * @brief Finalize XTS decryptor
 *
 * @param[in]  sp38e_ctx  SP38E context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_dec_xts_final(pufs_sp38e_ctx *sp38e_ctx, uint8_t *out, uint32_t *outlen);

/**
 * @brief Decryption using XTS mode with keyaddr type casting.
 *
 * @warning DO NOT call this function directly. Use pufs_dec_xts() instead.
 */
pufs_status_t _pufs_dec_xts(uint8_t        *out,
                            uint32_t       *outlen,
                            const uint8_t  *in,
                            uint32_t        inlen,
                            pufs_cipher_t   cipher,
                            pufs_key_type_t keytype1,
                            size_t          keyaddr1,
                            uint32_t        keybits,
                            pufs_key_type_t keytype2,
                            size_t          keyaddr2,
                            const uint8_t  *i,
                            uint32_t        j);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_SP38E_H */

/*** end of file ***/
