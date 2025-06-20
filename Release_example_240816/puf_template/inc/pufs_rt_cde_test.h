/**
 * @file pufs_rt_cde_test.h
 * @brief CDE API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_RT_CDE_TEST_H
#define PUFS_RT_CDE_TEST_H

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
pufs_status_t pufs_rt_cde_read_write_test(void);
pufs_status_t pufs_rt_cde_rolck_test(void);
pufs_status_t pufs_rt_cde_psmsk_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_RT_CDE_TEST_H */

/*** end of file ***/
