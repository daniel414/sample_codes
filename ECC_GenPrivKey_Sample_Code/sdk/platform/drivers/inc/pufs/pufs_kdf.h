/**
 * @file pufs_kdf.h
 * @brief KDF API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_KDF_H
#define PUFS_KDF_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include "pufs_common.h"
#include "pufs_hmac.h"
#include "pufs_ka.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
/**
 * @brief Generate a session key from a KDK derived from a shared secret.
 *
 * @param[in] keytype   Derived key type.
 * @param[in] keyslot   Slot in KA to store derived key.
 * @param[in] outbits   Derived key length in bits.
 * @param[in] prf       Pseudo-random function (PRF) family.
 * @param[in] hash      Hash algorithm for PRF used in HKDF.
 * @param[in] feedback  True/false to enable/disable feedback mode.
 * @param[in] iv        Initial vector.
 * @param[in] ctrpos    The position of the counter inserted in the fixedinfo.
 * @param[in] ctrlen    The length of the inserted counter in bytes.
 * @param[in] ztype     Key type of shared secret Z.
 * @param[in] zaddr     Key slot of shared secret Z.
 * @param[in] zbits     Length of shared secret Z in bits.
 * @param[in] salt      Salt used by KDF to derive KDK.
 * @param[in] saltlen   Salt length in bytes.
 * @param[in] info      Fixed info used by KDF key expansion.
 * @param[in] infolen   Fixed info length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 *
 * @note Only \ref SSKEY and \ref SHARESEC are allowed \em keytype.
 * @note \em iv is used in feedback mode. The length equals to the output of the
 *       underlying \em hash. It can be set to NULL if iv is not used in
 *       feedback mode.
 * @note Without feedback, \em ctrpos is in range [0, max(infolen,255)]. 0 or
 *       infolen means the counter is prepended or appended to the fixed info.
 *       In feedback mode, \em ctrpos is set to one of 0, 1, or 2 indicating the
 *       counter is before the feedback value, adter the feedback value, or
 *       after the fixed info.
 * @note \em ctrlen is in range [1, 4] in general. In feedback mode, \em ctrlen
 *       can be set to 0 indicating counter is not used.
 * @note \em zaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em ztype setting.
 */
#define pufs_kdf(keytype,    \
                 keyslot,    \
                 outbits,    \
                 prf,        \
                 hash,       \
                 feedback,   \
                 iv,         \
                 ctrpos,     \
                 ctrlen,     \
                 ztype,      \
                 zaddr,      \
                 zbits,      \
                 salt,       \
                 saltlen,    \
                 info,       \
                 infolen)    \
    _pufs_kdf(keytype,       \
              keyslot,       \
              outbits,       \
              prf,           \
              hash,          \
              feedback,      \
              iv,            \
              ctrpos,        \
              ctrlen,        \
              ztype,         \
              (size_t)zaddr, \
              zbits,         \
              salt,          \
              saltlen,       \
              info,          \
              infolen)

/**
 * @brief Derive a session key from a KDK.
 *
 * @param[in] keytype   Derived key type.
 * @param[in] keyslot   Slot in KA to store derived key.
 * @param[in] outbits   Derived key length in bits.
 * @param[in] prf       Pseudo-random function (PRF) family.
 * @param[in] hash      Hash algorithm for HMAC used in HKDF.
 * @param[in] feedback  True/false to enable/disable feedback mode.
 * @param[in] iv        Initial vector.
 * @param[in] ctrpos    The position of the counter inserted in the fixedinfo.
 * @param[in] ctrlen    The length of the inserted counter in bytes.
 * @param[in] kdktype   KDK key type.
 * @param[in] kdkaddr   KDK key address.
 * @param[in] kdkbits   Length of KDK key in bits.
 * @param[in] info      Fixed info used by KDF key expansion.
 * @param[in] infolen   Fixed info length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 *
 * @note Only \ref SSKEY and \ref SHARESEC are allowed \em keytype.
 * @note \em iv is used in feedback mode. The length equals to the output of the
 *       underlying \em hash. It can be set to NULL if iv is not used in
 *       feedback mode.
 * @note Without feedback, \em ctrpos is in range [0, max(infolen,255)]. 0 or
 *       infolen means the counter is prepended or appended to the fixed info.
 *       In feedback mode, \em ctrpos is set to one of 0, 1, or 2 indicating the
 *       counter is before the feedback value, adter the feedback value, or
 *       after the fixed info.
 * @note \em ctrlen is in range [1, 4] in general. In feedback mode, \em ctrlen
 *       can be set to 0 indicating counter is not used.
 * @note \em kdkaddr may be a \ref pufs_rt_slot_t element, a \ref pufs_ka_slot_t
 *       element, or a memory address according to the \em kdktype setting.
 */
#define pufs_key_expansion(keytype,      \
                           keyslot,      \
                           outbits,      \
                           prf,          \
                           hash,         \
                           feedback,     \
                           iv,           \
                           ctrpos,       \
                           ctrlen,       \
                           kdktype,      \
                           kdkaddr,      \
                           kdkbits,      \
                           info,         \
                           infolen)      \
    _pufs_key_expansion(keytype,         \
                        keyslot,         \
                        outbits,         \
                        prf,             \
                        hash,            \
                        feedback,        \
                        iv,              \
                        ctrpos,          \
                        ctrlen,          \
                        kdktype,         \
                        (size_t)kdkaddr, \
                        kdkbits,         \
                        info,            \
                        infolen)

/**
 * @brief Derive a session key by PBKDF.
 *
 * @param[in] keytype   Derived key type.
 * @param[in] keyslot   Slot in KA to store derived key.
 * @param[in] outbits   Derived key length in bits.
 * @param[in] prf       Pseudo-random function (PRF) family.
 * @param[in] hash      Hash algorithm for HMAC used in HKDF.
 * @param[in] iter      The number of iterations desired.
 * @param[in] salttype  PBKDF salt type.
 * @param[in] saltaddr  PBKDF salt address.
 * @param[in] saltbits  Length of PBKDF salt in bits.
 * @param[in] pass      PBKDF password.
 * @param[in] passlen   PBKDF password length in bytes.
 * @return              SUCCESS on success, otherwise an error code.
 *
 * @note Only \ref SSKEY and \ref SHARESEC are allowed \em keytype.
 * @note \em saltaddr may be a \ref pufs_rt_slot_t element, a \ref
 *       pufs_ka_slot_t element, or a memory address according to the
 *       \em salttype setting.
 */
#define pufs_pbkdf(                                                                          \
    keytype, keyslot, outbits, prf, hash, iter, salttype, saltaddr, saltbits, pass, passlen) \
    _pufs_pbkdf(keytype,                                                                     \
                keyslot,                                                                     \
                outbits,                                                                     \
                prf,                                                                         \
                hash,                                                                        \
                iter,                                                                        \
                salttype,                                                                    \
                (size_t)saltaddr,                                                            \
                saltbits,                                                                    \
                pass,                                                                        \
                passlen)

/**
 * @brief Pseudo-random function family used by KDF
 */
typedef enum
{
    PRF_HMAC,      /**< HMAC */
    PRF_HASH,      /**< HASH */
    PRF_CMAC,      /**< CMAC */
    N_PRFFAMILY_T, // keep in the last one
} pufs_prf_family_t;

/**
 * @brief KDF Method
 */
typedef enum
{
    METHOD_PBKDF,                /**< PBKDF */
    METHOD_KBKDF_EXPAND,         /**< one-step key derivation using expansion */
    METHOD_KBKDF_EXTRACT,        /**< one-step key derivation using extraction */
    METHOD_KBKDF_EXPAND_EXTRACT, /**< two-step key derivation */
    METHOD_SM2,                  /**< SM2 */
} pufs_kdf_md_t;

/**
 * @brief Position of the length of counter
 */
typedef enum
{
    COUNTER_POS_BEFORE,       /**< the counter coming before the interation variable */
    COUNTER_POS_BEFORE_FIXED, /**< the counter coming before the fixed input data (feedback mode) */
    COUNTER_POS_AFTER,        /**< the counter coming after the fixed input data */
    COUNTER_POS_MID, /**< the counter coming in the middle the fixed input data (counter mode) */
} pufs_kdf_cnt_pos_t;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief KDF function with zaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_kdf() instead.
 */
pufs_status_t _pufs_kdf(pufs_key_type_t   keytype,
                        pufs_ka_slot_t    keyslot,
                        uint32_t          outbits,
                        pufs_prf_family_t prf,
                        pufs_hash_t       hash,
                        bool              feedback,
                        const uint8_t    *iv,
                        uint32_t          ctrpos,
                        uint32_t          ctrlen,
                        pufs_key_type_t   ztype,
                        size_t            zaddr,
                        uint32_t          zbits,
                        const uint8_t    *salt,
                        uint32_t          saltlen,
                        const uint8_t    *info,
                        uint32_t          infolen);

/**
 * @brief Key expansion function with kdkaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_key_expansion()
 *          instead.
 */
pufs_status_t _pufs_key_expansion(pufs_key_type_t   keytype,
                                  pufs_ka_slot_t    keyslot,
                                  uint32_t          outbits,
                                  pufs_prf_family_t prf,
                                  pufs_hash_t       hash,
                                  bool              feedback,
                                  const uint8_t    *iv,
                                  uint32_t          ctrpos,
                                  uint32_t          ctrlen,
                                  pufs_key_type_t   kdktype,
                                  size_t            kdkaddr,
                                  uint32_t          kdkbits,
                                  const uint8_t    *info,
                                  uint32_t          infolen);

/**
 * @brief PBKDF function with saltaddr type casting
 *
 * @warning DO NOT call this function directly. Use pufs_pbkdf()
 *          instead.
 */
pufs_status_t _pufs_pbkdf(pufs_key_type_t   keytype,
                          pufs_ka_slot_t    keyslot,
                          uint32_t          outbits,
                          pufs_prf_family_t prf,
                          pufs_hash_t       hash,
                          uint32_t          iter,
                          pufs_key_type_t   salttype,
                          size_t            saltaddr,
                          uint32_t          saltbits,
                          const uint8_t    *pass,
                          uint32_t          passlen);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_KDF_H */

/*** end of file ***/
