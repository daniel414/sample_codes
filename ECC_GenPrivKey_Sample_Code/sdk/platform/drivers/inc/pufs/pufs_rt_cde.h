/**
 * @file pufs_rt_cde.h
 * @brief PUFrt CDE API interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_RT_CDE_H
#define PUFS_RT_CDE_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include "pufs_common.h"
#include "pufs_rt.h"

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Set CDE lock state
 *
 * @param[in] offset  Starting CDE address lock state to be set.
 * @param[in] length  The length of CDE data in bytes.
 * @param[in] lock    The lock state.
 * @return            SUCCESS on success, otherwise an error code.
 *
 * @note \em addr must be aligned to 4 bytes boundary
 */
pufs_status_t rt_cde_write_lock(uint32_t offset, uint32_t length, pufs_otp_lock_t lock);

/**
 * @brief Get CDE rwlck value
 *
 * @param[in] offset  The address of the rwlck.
 * @return            The rwlck bits.
 */
pufs_otp_lock_t rt_cde_read_lock(uint32_t offset);

/**
 * @brief PUFrt CDE post masking, mask 1K bits(128 bytes) segment starting from offset input
 *
 * @param[in] offset  Starting address of the segment to be masked
 * @return            SUCCESS on success, otherwise an error code.
 */
pufs_status_t rt_cde_write_mask(uint32_t offset);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_RT_CDE_H */

/*** end of file ***/
