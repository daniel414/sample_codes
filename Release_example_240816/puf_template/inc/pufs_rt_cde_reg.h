/**
 * @file pufs_rt_cde_reg.h
 * @brief PUFrt CDE register definition
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_RT_CDE_REG_H
#define PUFS_RT_CDE_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
struct pufs_rt_cde_regs
{
    uint32_t otp[768]; // 24K-bit OTP
};

/*******************************************************************************
 * Exported variables
 ******************************************************************************/
extern volatile struct pufs_rt_cde_regs *rt_cde_regs;

#ifdef __cplusplus
}
#endif

#endif /* PUFS_RT_CDE_REG_H */

/*** end of file ***/
