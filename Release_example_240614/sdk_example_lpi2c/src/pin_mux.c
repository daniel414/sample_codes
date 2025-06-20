/**
 * @file pin_mux.c
 * @brief A source file is about pins configuration.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pin_mux.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/

/*******************************************************************************
 * Private typedefs
 ******************************************************************************/

/*******************************************************************************
 * Global variables
 ******************************************************************************/
const pin_settings_config_t g_pin_mux_init_cfg_arr[NUM_OF_CONFIGURED_PINS0] = {
    {
        /** LPI2C1_SCL */
        .p_base           = PORTA,
        .pin_port_idx     = 0U,
        .pull_config      = PORT_INTERNAL_PULL_UP_ENABLED,
        .drive_select     = PORT_LOW_DRIVE_STRENGTH,
        .mux              = PORT_MUX_ALT3,
        .b_pin_lock       = false,
        .int_config       = PORT_DMA_INT_DISABLED,
        .b_clear_int_flag = false,
        .p_gpio_base      = NULL,
        .b_digital_filter = false,
    },
    {
        /** LPI2C1_SDA */
        .p_base           = PORTA,
        .pin_port_idx     = 1U,
        .pull_config      = PORT_INTERNAL_PULL_UP_ENABLED,
        .drive_select     = PORT_LOW_DRIVE_STRENGTH,
        .mux              = PORT_MUX_ALT3,
        .b_pin_lock       = false,
        .int_config       = PORT_DMA_INT_DISABLED,
        .b_clear_int_flag = false,
        .p_gpio_base      = NULL,
        .b_digital_filter = false,
    },
    {
        /** LPI2C0_SDA */
        .p_base           = PORTA,
        .pin_port_idx     = 2U,
        .pull_config      = PORT_INTERNAL_PULL_UP_ENABLED,
        .drive_select     = PORT_LOW_DRIVE_STRENGTH,
        .mux              = PORT_MUX_ALT3,
        .b_pin_lock       = false,
        .int_config       = PORT_DMA_INT_DISABLED,
        .b_clear_int_flag = false,
        .p_gpio_base      = NULL,
        .b_digital_filter = false,
    },
    {
        /** LPI2C0_SCL */
        .p_base           = PORTA,
        .pin_port_idx     = 3U,
        .pull_config      = PORT_INTERNAL_PULL_UP_ENABLED,
        .drive_select     = PORT_LOW_DRIVE_STRENGTH,
        .mux              = PORT_MUX_ALT3,
        .b_pin_lock       = false,
        .int_config       = PORT_DMA_INT_DISABLED,
        .b_clear_int_flag = false,
        .p_gpio_base      = NULL,
        .b_digital_filter = false,
    },
};

/*******************************************************************************
 * Static variables
 ******************************************************************************/

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
