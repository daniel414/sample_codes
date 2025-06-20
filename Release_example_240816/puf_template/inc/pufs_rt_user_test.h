/**
 * @file pufs_rt_user_test.h
 * @brief RT API test cases interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_RT_USER_TEST_H
#define PUFS_RT_USER_TEST_H

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
pufs_status_t pufs_rt_otp_rwlck_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_RT_USER_TEST_H */

/*** end of file ***/
