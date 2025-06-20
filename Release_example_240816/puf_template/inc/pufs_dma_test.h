/**
 * @file pufs_dma_test.h
 * @brief DMA API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_DMA_TEST_H
#define PUFS_DMA_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_test_common.h"

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
pufs_status_t pufs_dma_entropy_read_test(void);
pufs_status_t pufs_dma_random_read_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_DMA_TEST_H */

/*** end of file ***/
