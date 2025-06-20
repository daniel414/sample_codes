/**
 * @file device_registers.h
 * @brief Include the cpu specific register header files.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef DEVICE_REGISTERS_H
#define DEVICE_REGISTERS_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
/* Specific core definitions */
#include "core_cm0.h"
/* Register definitions */
#include "tw9001.h"
/* CPU specific feature definitions */
#include "tw9001_features.h"
/* Debug assertion */
#include "devassert.h"

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_REGISTERS_H */

/*** end of file ***/
