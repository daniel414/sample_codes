/**
 * @file pufs_ecc.h
 * @brief ECC API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_ECC_H
#define PUFS_ECC_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
/**
 * @brief RSA variant.
 */
typedef enum
{
    RSA1024,      /**< RSA-1024 */
    RSA2048,      /**< RSA-2048 */
    RSA3072,      /**< RSA-3072 */
    RSA4096,      /**< RSA-4096 */
    N_RSA_TYPE_T, // keep in the last one
} pufs_rsa_type_t;

/**
 * @brief NIST standardized elliptic curves.
 */
typedef enum
{
    NISTB163,   /**< NIST B-163 */
    NISTB233,   /**< NIST B-233 */
    NISTB283,   /**< NIST B-283 */
    NISTB409,   /**< NIST B-409 */
    NISTB571,   /**< NIST B-571 */
    NISTK163,   /**< NIST K-163 */
    NISTK233,   /**< NIST K-233 */
    NISTK283,   /**< NIST K-283 */
    NISTK409,   /**< NIST K-409 */
    NISTK571,   /**< NIST K-571 */
    NISTP192,   /**< NIST P-192 */
    NISTP224,   /**< NIST P-224 */
    NISTP256,   /**< NIST P-256 */
    NISTP384,   /**< NIST P-384 */
    NISTP521,   /**< NIST P-521 */
    SM2,        /**< SM2 */
    N_ECNAME_T, // keep in the last one
} pufs_ec_name_t;

/**
 * @brief Elliptic curve (EC) domain parameters
 */
typedef struct
{
    const void *field; /**< Field modulus. */
    const void *a;     /**< EC parameter a. */
    const void *b;     /**< EC parameter b. */
    const void *px;    /**< x-coordinate of base point P. */
    const void *py;    /**< y-coordinate of base point P. */
    const void *order; /**< Subgroup order. */
    uint16_t    fbits; /**< Field element length in bits. */
    uint16_t    nbits; /**< Subgroup order length in bits. */
    uint8_t     ftype; /**< Field type in hardware. */
    uint8_t     h;     /**< Co-factor. */
    uint8_t     len;   /**< Field element length in bytes. */
    bool        pf;    /**< Prime field flag */
} pufs_ecc_param_st;

/**
 * @brief Maximum field element length in bytes.
 */
#ifndef QLEN_MAX
#define QLEN_MAX 72
#endif

/**
 * @brief Elliptic curve point (x,y).
 */
typedef struct
{
    uint32_t qlen;        /**< Field element length in bytes. */
    uint8_t  x[QLEN_MAX]; /**< x-coordinate */
    uint8_t  y[QLEN_MAX]; /**< y-coordinate */
} pufs_ec_point_st;

/**
 * @brief Maximum field element length in bytes.
 */
#ifndef NLEN_MAX
#define NLEN_MAX 72
#endif

/**
 * @brief ECDSA signature (r,s).
 */
typedef struct
{
    uint32_t qlen;        /**< Field element length in bytes. */
    uint8_t  r[NLEN_MAX]; /**< r */
    uint8_t  s[NLEN_MAX]; /**< s */
} pufs_ecdsa_sig_st;

/*******************************************************************************
 * Exported variables
 ******************************************************************************/
extern pufs_ecc_param_st g_ecc_param[];

#ifdef __cplusplus
}
#endif

#endif /* PUFS_ECC_H */

/*** end of file ***/
