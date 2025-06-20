/**
 * @file pufs_dma.h
 * @brief DMA interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_DMA_H
#define PUFS_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include "pufs_common.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
typedef enum
{
    ALGO_TYPE_HKDF = 0,
    ALGO_TYPE_HMAC,
    ALGO_TYPE_CMAC,
    ALGO_TYPE_KLB    = 4,
    ALGO_TYPE_SM2ENC = 7,
    ALGO_TYPE_SP38A,
    ALGO_TYPE_GCM,
    ALGO_TYPE_XTS,
    ALGO_TYPE_CCM,
    ALGO_TYPE_CHACHA,
    ALGO_TYPE_CYPT_REG_IO,
    ALGO_TYPE_KEY_EXPORT,
    ALGO_TYPE_NONE,
} pufs_algo_type_t;

/**
 * @brief Memory attributes for DMA
 */
typedef struct dma_dsc_attrs
{
    bool    fix_read_addr;  /**< Enable dma fixed read address (read output data) */
    bool    fix_write_addr; /**< Enable dma fixed write address (write input data) */
    uint8_t read_protect;   /**< Memory protection of read address for DMA */
    uint8_t write_protect;  /**< Memory protection of write address for DMA */
} pufs_dma_dsc_attr_st;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Release DMA module
 */
void pufs_dma_module_release(void);

/**
 * @brief Setup DMA attributes for normal DMA mode.
 *
 * @param[in] attr DMA attribute
 */
void pufs_dma_set_dsc_attr(pufs_dma_dsc_attr_st *attr);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_DMA_H */

/*** end of file ***/
