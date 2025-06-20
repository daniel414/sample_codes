/**
 * @file pufs_common.h
 * @brief common API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_COMMON_H
#define PUFS_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <inttypes.h>

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
/**
 * @brief Convert number of bits to number of bytes
 *
 * @param[in] bits  Number of bits.
 * @return          Number of bytes.
 */
#define b2B(bits) (((bits) + 7) / 8)

/**
 * @brief Convert number of bytes to number of bits
 *
 * @param[in] len  Number of bytes.
 * @return         Number of bits.
 */
#define B2b(len) (8 * (len))

/**
 * @brief Block size in bytes of block cipher algorithms
 */
#define BC_BLOCK_SIZE 16

/**
 * @brief Status code
 */
typedef enum
{
    SUCCESS,     /**< Success. */
    E_ALIGN,     /**< Address alignment mismatch. */
    E_OVERFLOW,  /**< Space overflow. */
    E_UNDERFLOW, /**< Size too small. */
    E_INVALID,   /**< Invalid argument. */
    E_BUSY,      /**< Resource is occupied. */
    E_UNAVAIL,   /**< Resource is unavailable. */
    E_FIRMWARE,  /**< Firmware error. */
    E_VERFAIL,   /**< Invalid public key or digital signature. */
    E_ECMPROG,   /**< Invalid ECC microprogram. */
    E_DENY,      /**< Access denied. */
    E_UNSUPPORT, /**< Not support. */
    E_INFINITY,  /**< Point at infinity. */
    E_ERROR,     /**< Unspecific error. */
} pufs_status_t;

/**
 * @brief Block cipher algorithm.
 */
typedef enum
{
    AES,        /**< AES */
    SM4,        /**< SM4 */
    CHACHA,     /**< CHACHA */
    N_CIPHER_T, // keep in the last one
} pufs_cipher_t;

/**
 * @brief Maximum message digest length in bytes.
 */
#ifndef DLEN_MAX
#define DLEN_MAX 64
#endif

/**
 * @brief Message digest structure.
 */
typedef struct
{
    uint32_t dlen;           /**< Current message digest length in bytes. */
    uint8_t  dgst[DLEN_MAX]; /**< Message digest. */
} pufs_dgst_st;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Release PUFs module
 *
 */
void pufs_release(void);

/**
 * @brief Return status message
 *
 */
char *pufs_strstatus(pufs_status_t status);

/**
 * @brief Initialize PUFs module
 *
 */
pufs_status_t pufs_initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_COMMON_H */

/*** end of file ***/
