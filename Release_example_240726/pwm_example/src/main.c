/**
 * @file main.c
 * @brief A description of the template's purpose.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "clock_config.h"
#include "pins_driver.h"
#include "lpuart_driver.h"
#include "ftm_driver.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define NUM_OF_CONFIGURED_PINS0 3
#define TIMEOUT                 500U
#define STARTUP_MSG             "PWM example\r\n"
#define INST_FTM0               0U

/*******************************************************************************
 * Private typedefs
 ******************************************************************************/

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int g_exit_code = 0; /**< exit code */
/*******************************************************************************
 * Static variables
 ******************************************************************************/

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
void
init_gpio(void)
{
    pin_settings_config_t g_pin_mux_InitConfigArr0[NUM_OF_CONFIGURED_PINS0] = {
        {
            .p_base           = PORTB,
            .pin_port_idx     = 0U,
            .pull_config      = PORT_INTERNAL_PULL_NOT_ENABLED,
            .drive_select     = PORT_LOW_DRIVE_STRENGTH,
            .mux              = PORT_MUX_ALT2,
            .b_pin_lock       = false,
            .int_config       = PORT_DMA_INT_DISABLED,
            .b_clear_int_flag = false,
            .p_gpio_base      = NULL,
            .b_digital_filter = false,
        },
        {
            .p_base           = PORTB,
            .pin_port_idx     = 1U,
            .pull_config      = PORT_INTERNAL_PULL_NOT_ENABLED,
            .drive_select     = PORT_LOW_DRIVE_STRENGTH,
            .mux              = PORT_MUX_ALT2,
            .b_pin_lock       = false,
            .int_config       = PORT_DMA_INT_DISABLED,
            .b_clear_int_flag = false,
            .p_gpio_base      = NULL,
            .b_digital_filter = false,
        },
        {
            .p_base           = PORTB,
            .pin_port_idx     = 4U,
            .pull_config      = PORT_INTERNAL_PULL_NOT_ENABLED,
            .drive_select     = PORT_LOW_DRIVE_STRENGTH,
            .mux              = PORT_MUX_ALT2,
            .b_pin_lock       = false,
            .int_config       = PORT_DMA_INT_DISABLED,
            .b_clear_int_flag = false,
            .p_gpio_base      = NULL,
            .b_digital_filter = false,
        },
    };
    pins_init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);
}

void
init_uart(void)
{
    static lpuart_state_t      s_lpuartState[1];
    const lpuart_user_config_t lpuart0_config = {
        .baud_rate          = 115200UL,
        .parity_mode        = LPUART_PARITY_DISABLED,
        .bit_count_per_char = LPUART_8_BITS_PER_CHAR,
        .stop_bit_count     = LPUART_ONE_STOP_BIT,
        .transfer_type      = LPUART_USING_INTERRUPTS,
        .rx_dma_channel     = 0UL,
        .tx_dma_channel     = 0UL,
    };

    lpuart_init(0, (lpuart_state_t *)(&s_lpuartState[0]), &lpuart0_config);
}
void
init_ftm(void)
{
    ftm_state_t       state;
    ftm_user_config_t flexTimer_pwm_1_InitConfig = {
        {
            false,
            false,
            false,
            false,
            false,
            false,
            FTM_PWM_SYNC,
            FTM_PWM_SYNC,
            FTM_PWM_SYNC,
            FTM_PWM_SYNC,
            true,
            FTM_UPDATE_NOW,
        },
        FTM_MODE_EDGE_ALIGNED_PWM,  /* Mode of operation for FTM */
        FTM_CLOCK_DIVID_BY_1,       /* FTM clock prescaler */
        FTM_CLOCK_SOURCE_SYSTEMCLK, /* FTM clock source */
        FTM_BDM_MODE_11,
        true,
        false
    };

    ftm_independent_ch_param_t flexTimer_pwm_1_IndependentChannelsConfig[1] = {
        {
            4,
            FTM_POLARITY_HIGH,
			19660U,             /* Duty cycle percent 0-0x8000 */
            false,
            FTM_LOW_STATE,
            false,
            FTM_MAIN_INVERTED,
            false,
        }};

    /* Fault configuration structure for flexTimer_pwm_1*/
    ftm_pwm_fault_param_t flexTimer_pwm_1_FaultConfig = {
        false,
        false,
        0U,
        FTM_FAULT_CONTROL_DISABLED,
        {
            {
                false,
                false,
                FTM_POLARITY_LOW,
            },
            {
                false,
                false,
                FTM_POLARITY_LOW,
            },
            {
                false,
                false,
                FTM_POLARITY_LOW,
            },
            {
                false,
                false,
                FTM_POLARITY_LOW,
            },
        }};

    ftm_pwm_param_t flexTimer_pwm_1_PwmConfig = {
        1U,                                        /* Number of independent PWM channels */
        0U,                                        /* Number of combined PWM channels */
        FTM_MODE_EDGE_ALIGNED_PWM,                 /* PWM mode */
        0U,
        FTM_DEADTIME_DIVID_BY_1,
        1000U,                                     /* PWM frequency */
        flexTimer_pwm_1_IndependentChannelsConfig,
        NULL,
        &flexTimer_pwm_1_FaultConfig
    };

    ftm_init(INST_FTM0, &flexTimer_pwm_1_InitConfig, &state);

    ftm_init_pwm(INST_FTM0, &flexTimer_pwm_1_PwmConfig);
}

void
board_init(void)
{
#if defined(CW_MCU_LQFP100_EVB)
    /* I2C */
    PORTA->PCR[2] =
        ((PORTA->PCR[2] & ~PORT_PCR_PE_MASK) & ~PORT_PCR_PS_MASK) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTA->PCR[3] =
        ((PORTA->PCR[3] & ~PORT_PCR_PE_MASK) & ~PORT_PCR_PS_MASK) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    /* TEST */
    PORTA->PCR[6] =
        ((PORTA->PCR[6] & ~PORT_PCR_PE_MASK) & ~PORT_PCR_PS_MASK) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    /* UART_RX */
    PORTB->PCR[0] =
        ((PORTB->PCR[0] & ~PORT_PCR_PE_MASK) & ~PORT_PCR_PS_MASK) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    /* Button */
    PORTD->PCR[5] =
        ((PORTD->PCR[5] & ~PORT_PCR_PE_MASK) & ~PORT_PCR_PS_MASK) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    /* SWD */
    PORTA->PCR[4] =
        ((PORTA->PCR[4] & ~PORT_PCR_PE_MASK) & ~PORT_PCR_PS_MASK) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTC->PCR[4] = ((PORTC->PCR[4] & ~PORT_PCR_PE_MASK) & ~PORT_PCR_PS_MASK) | PORT_PCR_PE(1);
#endif
}

/**
 * @brief The main function for the project.
 * @details The startup initialization sequence is the following:
 * - main()
 */
int
main(void)
{
    /* Initialize */
    pins_chip_init();
    board_init();

    /* Initialize and configure clocks */
    clock_init_config(gp_clock_man_configs_arr,
                      CLOCK_MANAGER_CONFIG_CNT,
                      gp_clock_man_callback_arr,
                      CLOCK_MANAGER_CALLBACK_CNT);
    clock_update_config(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    init_gpio();
    init_uart();
    init_ftm();
    lpuart_send_data_blocking(0, (uint8_t *)STARTUP_MSG, strlen(STARTUP_MSG), TIMEOUT);

    for (;;)
    {
    }

    return g_exit_code;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
