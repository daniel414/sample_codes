/**
 * @file pin_mux.h
 * @brief A header file is about pins configuration.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PIN_MUX_H
#define PIN_MUX_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "pins_driver.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define NUM_OF_CONFIGURED_PINS0 (4u)

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Exported variables
 ******************************************************************************/
extern const pin_settings_config_t g_pin_mux_init_cfg_arr[NUM_OF_CONFIGURED_PINS0];

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PIN_MUX_H */

/*** end of file ***/
