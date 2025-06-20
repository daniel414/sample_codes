/**
 * @file pufs_test_common.h
 * @brief test common utility interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_TEST_COMMON_H
#define PUFS_TEST_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string.h>
#include "pufs_common.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Exported variables
 ******************************************************************************/

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief A wrapper function of memcmp
 *
 * @param[in] __s1 buf1
 * @param[in] __s2 buf2
 * @param[in] __n  number of bytes to be compared in buf1 and buf2
 * @return SUCCESS if contents are identical, otherwise E_VERFAIL
 */
static inline pufs_status_t
pufs_test_memcmp(const void *__s1, const void *__s2, size_t __n)
{
    return memcmp(__s1, __s2, __n) == 0 ? SUCCESS : E_VERFAIL;
}

#ifdef __cplusplus
}
#endif

#endif /* PUFS_TEST_COMMON_H */

/*** end of file ***/
