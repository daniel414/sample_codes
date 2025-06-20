/**
 * @file pufs_chacha.h
 * @brief ChaCha API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_CHACHA_H
#define PUFS_CHACHA_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_common.h"
#include "pufs_ka.h"
#include "pufs_dma.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
typedef enum
{
    CHACHA_AEAD_AAD,
    CHACHA_AEAD_PLAINTEXT,
} pufs_chacha_aead_data_t;

/**
 * @brief required parameters for chacha algorithm
 */
typedef struct chacha_params
{
    pufs_key_type_t keytype; /**< Key type */
    /** @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
     *            element, or a memory address according to the \em keytype setting.
     *  @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
     */
    size_t         keyaddr; /**< Key address */
    uint32_t       keybits; /**< Key length in bits */
    const uint8_t *iv;      /**< Initial vector */
    uint32_t       ivlen;   /**< Initial vector length in bytes */
    uint32_t       counter; /**< Initial block counter */
    // @note the default value of round is 20
    uint32_t round; /**< Initial quarter rounds */
} pufs_chacha_params_st;

/**
 * @brief required parameters for chacha20-poly1305 algorithm
 */
typedef struct chacha20_poly1305_params
{
    pufs_key_type_t keytype; /**< Key type */
    /** @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
     *            element, or a memory address according to the \em keytype setting.
     *  @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
     */
    size_t         keyaddr; /**< Key address */
    uint32_t       keybits; /**< Key length in bits */
    const uint8_t *iv;      /**< Initial vector */
    uint32_t       ivlen;   /**< Initial vector length in bytes */
} pufs_chacha20_poly1305_params_st;

/**
 * @brief required parameters for poly1305 algorithm
 */
typedef struct poly1305_params
{
    pufs_key_type_t keytype; /**< Key type */
    /** @note \em keyaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
     *            element, or a memory address according to the \em keytype setting.
     *  @warning Set \ref PUFKEY or \ref SHARESEC as \em keytype is not allowed.
     */
    size_t   keyaddr; /**< Key address */
    uint32_t keybits; /**< Key length in bits */
} pufs_poly1305_params_st;

/*******************************************************************************
 * Typedefs
 ******************************************************************************/
typedef struct pufs_chacha_context pufs_chacha_ctx;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Obtain a pointer to ChaCha internal context
 *
 * @return A pointer to ChaCha internal context, or NULL if error
 */
pufs_chacha_ctx *pufs_chacha_ctx_new(void);

/**
 * @brief Free a pointer to ChaCha internal context
 *
 * @param[in] chacha_ctx  A pointer to ChaCha context.
 */
void pufs_chacha_ctx_free(pufs_chacha_ctx *chacha_ctx);

/**
 * @brief Initialize ChaCha encryptor
 *
 * @param[in] chacha_ctx  ChaCha context to be initialized.
 * @param[in] params      Parameters for ChaCha algorithm.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha_enc_init(pufs_chacha_ctx *chacha_ctx, pufs_chacha_params_st *params);

/**
 * @brief Input data into ChaCha encryptor
 *
 * @param[in]  chacha_ctx  ChaCha context.
 * @param[out] out         Output data.
 * @param[out] outlen      Output data length in bytes.
 * @param[in]  in          Input data.
 * @param[in]  inlen       Input data length in bytes.
 * @return                 SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha_enc_update(
    pufs_chacha_ctx *chacha_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *in, uint32_t inlen);

/**
 * @brief Finalize ChaCha encryptor
 *
 * @param[in]  chacha_ctx ChaCha context.
 * @param[out] out        Output data. (set to NULL if using SGDMA descriptor)
 * @param[out] outlen     Output data length in bytes. (set to 0 if using SGDMA descriptor)
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha_enc_final(pufs_chacha_ctx *chacha_ctx, uint8_t *out, uint32_t *outlen);

/**
 * @brief Encryption using ChaCha algorithm.
 *
 * @param[out] out      Output data.
 * @param[out] outlen   Output data length in bytes.
 * @param[in]  params   Parameters for ChaCha algorithm.
 * @param[in]  in       Input data.
 * @param[in]  inlen    Input data length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha_enc(uint8_t               *out,
                              uint32_t              *outlen,
                              pufs_chacha_params_st *params,
                              const uint8_t         *in,
                              uint32_t               inlen);

/**
 * @brief Initialize ChaCha decryptor
 *
 * @param[in] chacha_ctx ChaCha context to be initialized.
 * @param[in] params     Parameters for ChaCha algorithm.
 * @return               SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha_dec_init(pufs_chacha_ctx *chacha_ctx, pufs_chacha_params_st *params);

/**
 * @brief Input data into ChaCha decryptor
 *
 * @param[in]  chacha_ctx ChaCha context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[in]  in         Input data.
 * @param[in]  inlen      Input data length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha_dec_update(
    pufs_chacha_ctx *chacha_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *in, uint32_t inlen);

/**
 * @brief Finalize ChaCha decryptor
 *
 * @param[in]  chacha_ctx ChaCha context.
 * @param[out] out        Output data. (set to NULL if using SGDMA descriptor)
 * @param[out] outlen     Output data length in bytes. (set to 0 if using SGDMA descriptor)
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha_dec_final(pufs_chacha_ctx *chacha_ctx, uint8_t *out, uint32_t *outlen);

/**
 * @brief Decryption using ChaCha algorithm.
 *
 * @param[out] out      Output data.
 * @param[out] outlen   Output data length in bytes.
 * @param[in]  params   Parameters for ChaCha algorithm.
 * @param[in]  in       Input data.
 * @param[in]  inlen    Input data length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha_dec(uint8_t               *out,
                              uint32_t              *outlen,
                              pufs_chacha_params_st *params,
                              const uint8_t         *in,
                              uint32_t               inlen);

/**
 * @brief Initialize ChaCha20-Poly1305 encryptor
 *
 * @param[in] chacha_ctx  ChaCha context to be initialized.
 * @param[in] params      Parameters for ChaCha20-Poly1305 algorithm.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha20_poly1305_enc_init(pufs_chacha_ctx                  *chacha_ctx,
                                              pufs_chacha20_poly1305_params_st *params);

/**
 * @brief Input data into ChaCha20-Poly1305 encryptor
 *
 * @param[in]  chacha_ctx  ChaCha context.
 * @param[out] out         Output data.
 * @param[out] outlen      Output data length in bytes.
 * @param[in]  in          Input data.
 * @param[in]  inlen       Input data length in bytes.
 * @return                 SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha20_poly1305_enc_update(
    pufs_chacha_ctx *chacha_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *in, uint32_t inlen);

/**
 * @brief Finalize ChaCha20-Poly1305 encryptor
 *
 * @param[in]  chacha_ctx ChaCha context.
 * @param[out] out        Output data. (set to NULL if using SGDMA descriptor)
 * @param[out] outlen     Output data length in bytes. (set to 0 if using SGDMA descriptor)
 * @param[out] tag        Output Tag.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha20_poly1305_enc_final(pufs_chacha_ctx *chacha_ctx,
                                               uint8_t         *out,
                                               uint32_t        *outlen,
                                               uint8_t         *tag);

/**
 * @brief Encryption using ChaCha20-Poly1305 algorithm.
 *
 * @param[out] out      Output data.
 * @param[out] outlen   Output data length in bytes.
 * @param[out] tag      Output Tag.
 * @param[in]  params   Parameters for ChaCha20-Poly1305 algorithm.
 * @param[in]  aad      Additional authenticated data.
 * @param[in]  aadlen   The length of additional authenticated data.
 * @param[in]  in       Input data.
 * @param[in]  inlen    Input data length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha20_poly1305_enc(uint8_t                          *out,
                                         uint32_t                         *outlen,
                                         uint8_t                          *tag,
                                         pufs_chacha20_poly1305_params_st *params,
                                         const uint8_t                    *aad,
                                         uint32_t                          aadlen,
                                         const uint8_t                    *in,
                                         uint32_t                          inlen);

/**
 * @brief Initialize ChaCha20-Poly1305 decryptor
 *
 * @param[in] chacha_ctx ChaCha context to be initialized.
 * @param[in] params     Parameters for ChaCha20-Poly1305 algorithm.
 * @return               SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha20_poly1305_dec_init(pufs_chacha_ctx                  *chacha_ctx,
                                              pufs_chacha20_poly1305_params_st *params);

/**
 * @brief Input data into ChaCha20-Poly1305 decryptor
 *
 * @param[in]  chacha_ctx ChaCha context.
 * @param[out] out        Output data.
 * @param[out] outlen     Output data length in bytes.
 * @param[in]  in         Input data.
 * @param[in]  inlen      Input data length in bytes.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha20_poly1305_dec_update(
    pufs_chacha_ctx *chacha_ctx, uint8_t *out, uint32_t *outlen, const uint8_t *in, uint32_t inlen);

/**
 * @brief Finalize ChaCha20-Poly1305 decryptor
 *
 * @param[in]  chacha_ctx ChaCha context.
 * @param[out] out        Output data. (set to NULL if using SGDMA descriptor)
 * @param[out] outlen     Output data length in bytes. (set to 0 if using SGDMA descriptor)
 * @param[in]  tag        Input tag.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha20_poly1305_dec_final(pufs_chacha_ctx *chacha_ctx,
                                               uint8_t         *out,
                                               uint32_t        *outlen,
                                               const uint8_t   *tag);

/**
 * @brief Decryption using ChaCha20-Poly1305 algorithm.
 *
 * @param[out] out      Output data.
 * @param[out] outlen   Output data length in bytes.
 * @param[in]  params   Parameters for ChaCha20-Poly1305 algorithm.
 * @param[in]  aad      Additional authenticated data.
 * @param[in]  aadlen   The length of additional authenticated data.
 * @param[in]  in       Input data.
 * @param[in]  inlen    Input data length in bytes.
 * @param[in]  tag      Input tag.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_chacha20_poly1305_dec(uint8_t                          *out,
                                         uint32_t                         *outlen,
                                         pufs_chacha20_poly1305_params_st *params,
                                         const uint8_t                    *aad,
                                         uint32_t                          aadlen,
                                         const uint8_t                    *in,
                                         uint32_t                          inlen,
                                         const uint8_t                    *tag);

/**
 * @brief Produce a tag for authentication using Poly1305 algorithm.
 *
 * @param[out] tag      Output tag.
 * @param[in]  params   Parameters for Poly1305 algorithm.
 * @param[in]  in       Input message.
 *  @param[in] inlen    Input message length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_poly1305(uint8_t                 *tag,
                            pufs_poly1305_params_st *params,
                            const uint8_t           *in,
                            const uint32_t           inlen);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_CHACHA_H */

/*** end of file ***/
