/**
 * @file pufs_test.h
 * @brief pufs test flow interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_TEST_H
#define PUFS_TEST_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_test_common.h"

/*******************************************************************************
 * Exported variables
 ******************************************************************************/
 extern void print(const char *p_source_str);

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
void pufs_test(void);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_TEST_H */

/*** end of file ***/

