/**
 * @file clock_config.h
 * @brief Clock initialization configuration header file.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef CLOCK_CONFIG_H
#define CLOCK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "clock_tw9001.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
/** @brief Count of user configuration structures */
#define CLOCK_MANAGER_CONFIG_CNT 1U /**< Count of user configuration */

/** @brief Count of user Callbacks structures */
#define CLOCK_MANAGER_CALLBACK_CNT 0U /**< Count of user Callbacks */

/** @brief Count of peripheral clock user configuration 0*/
#define NUM_OF_PERIPHERAL_CLOCKS_0 13U /* Count of peripheral clock user configurations. */

/** @brief User configuration structure 0*/
extern clock_manager_user_config_t g_clock_man1_init_config0;

/** @brief User peripheral configuration structure 0*/
extern peripheral_clock_config_t g_peripheral_clock_config0[NUM_OF_PERIPHERAL_CLOCKS_0];

/** @brief Array of User callbacks */
extern clock_manager_callback_user_config_t *gp_clock_man_callback_arr[];

/** @brief Array of pointers to User configuration structures */
extern clock_manager_user_config_t const *gp_clock_man_configs_arr[CLOCK_MANAGER_CONFIG_CNT];

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_CONFIG_H */

/*** end of file ***/
