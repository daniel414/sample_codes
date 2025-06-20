/**
 * @file pufs_sm2.h
 * @brief SM2 API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_SM2_H
#define PUFS_SM2_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_common.h"
#include "pufs_ka.h"
#include "pufs_ecc.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
/**
 * @brief The format of SM2 encryption/decryption
 */
typedef enum
{
    N_SM2_T = -1,
    SM2_C1C2C3,
    SM2_C1C3C2
} pufs_sm2_format_t;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief SM2 encryption
 *
 * @param[out] out     SM2 ciphertext.
 * @param[out] outlen  SM2 ciphertext length in bytes.
 * @param[in]  in      Plaintext.
 * @param[in]  inlen   Plaintext length in bytes.
 * @param[in]  puk     SM2 public key.
 * @param[in]  format  Format for SM2 encryption.
 * @param[in]  k       SM2 encryption ephemeral key. k can be NULL.
 * @return             SUCCESS on success, otherwise an error code.
 *
 * @note Currently input \em k is not supported.
 */
pufs_status_t pufs_sm2_enc(uint8_t          *out,
                           uint32_t         *outlen,
                           const uint8_t    *in,
                           uint32_t          inlen,
                           pufs_ec_point_st  puk,
                           pufs_sm2_format_t format,
                           const uint8_t    *k);

/**
 * @brief SM2 decryption
 *
 * @param[out] out      Plaintext.
 * @param[out] outlen   Plaintext length in bytes.
 * @param[in]  in       SM2 ciphertext.
 * @param[in]  inlen    SM2 ciphertext length in bytes.
 * @param[in]  prk      Private key slot.
 * @param[in]  format   Format for SM2 encryption.
 * @return              SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_sm2_dec(uint8_t          *out,
                           uint32_t         *outlen,
                           const uint8_t    *in,
                           uint32_t          inlen,
                           pufs_ka_slot_t    prk,
                           pufs_sm2_format_t format);

/**
 * @brief SM2 key exchange protocol
 *
 * @param[out] s2         Shared secret hash starting with 0x02. (\f$S_B\f$)
 * @param[out] s3         Shared secret hash starting with 0x03. (\f$S_A\f$)
 * @param[out] key        The shared key.
 * @param[in]  keybits    The shared key length in bits.
 * @param[in]  idl        Local party identity.
 * @param[in]  idllen     Local party identity length in bytes.
 * @param[in]  idr        Remote party identity.
 * @param[in]  idrlen     Remote party identity length in bytes.
 * @param[in]  prkslotl   Local party private key slot.
 * @param[in]  tprkslotl  Local party ephemeral private key slot.
 * @param[in]  pukr       Remote party public key.
 * @param[in]  tpukr      Remote party ephemeral public key.
 * @param[in]  init       True if the key exchange protocol is initiated by local.
 * @return                SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_sm2_kex(pufs_dgst_st    *s2,
                           pufs_dgst_st    *s3,
                           uint8_t         *key,
                           uint32_t         keybits,
                           const uint8_t   *idl,
                           uint32_t         idllen,
                           const uint8_t   *idr,
                           uint32_t         idrlen,
                           pufs_ka_slot_t   prkslotl,
                           pufs_ka_slot_t   tprkslotl,
                           pufs_ec_point_st pukr,
                           pufs_ec_point_st tpukr,
                           bool             init);

/**
 * @brief SM2 signature verification
 *
 * @param[in]  sig     SM2 signature.
 * @param[in]  msg     Message.
 * @param[in]  msglen  Message length in bytes.
 * @param[in]  id      Identity.
 * @param[in]  idlen   Identity length in bytes.
 * @param[in]  puk     Public key.
 * @return             SUCCESS on success, otherwise an error code.
 */
pufs_status_t pufs_sm2_verify(pufs_ecdsa_sig_st sig,
                              const uint8_t    *msg,
                              uint32_t          msglen,
                              const uint8_t    *id,
                              uint32_t          idlen,
                              pufs_ec_point_st  puk);

/**
 * @brief SM2 signature signing
 *
 * @param[in]  sig      SM2 signature.
 * @param[in]  msg      Message.
 * @param[in]  msglen   Message length in bytes.
 * @param[in]  id       Identity.
 * @param[in]  idlen    Identity length in bytes.
 * @param[in]  prk      Private key slot.
 * @param[in]  k        Ephemeral private key. k can be NULL.
 * @return              SUCCESS on success, otherwise an error code.
 *
 * @note Currently input \em k is not supported.
 */
pufs_status_t pufs_sm2_sign(pufs_ecdsa_sig_st *sig,
                            const uint8_t     *msg,
                            uint32_t           msglen,
                            const uint8_t     *id,
                            uint32_t           idlen,
                            pufs_ka_slot_t     prk,
                            const uint8_t     *k);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_SM2_H */

/*** end of file ***/
