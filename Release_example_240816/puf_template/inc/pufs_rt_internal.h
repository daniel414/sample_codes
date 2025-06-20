/**
 * @file pufs_rt_internal.h
 * @brief RT internal interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_RT_INTERNAL_TEST_H
#define PUFS_RT_INTERNAL_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define WORD_SIZE 4 // 4 bytes = 32 bits = an otp block

typedef enum
{
    PGMPRT_FLAG     = 0xb6,
    TMLCK_FLAG      = 0x71,
    OTPLCK_FLAG     = 0x65,
    PUFLCK_FLAG     = 0x2c,
    SHFREN_FLAG     = 0x99,
    SHFWEN_FLAG     = 0xc2,
    SET_SHFREN_FLAG = 0xe3,
    SET_SHFWEN_FLAG = 0x8e,
} pufs_ptm_flag_t;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
void rt_write_enroll(void);
pufs_status_t rt_write_set_flag(pufs_ptm_flag_t flag, uint32_t *status);

#ifdef __cplusplus
}
#endif

#endif /* PUFS_RT_INTERNAL_TEST_H */

/*** end of file ***/
