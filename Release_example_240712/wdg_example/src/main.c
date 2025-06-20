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
#include "wdg_driver.h"
#include "lpuart_driver.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define TIMEOUT       500UL  /* Timeout in ms for blocking operations */
#define WDG_FEED_TIME 1800UL /* Timeout in ms for blocking operations */
#define STARTUP_MSG   "SDK_WDG\r\n"
/*******************************************************************************
 * Private typedefs
 ******************************************************************************/

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int      g_exit_code = 0; /**< exit code */
volatile uint32_t g_led_count = 0; /**< led blink timer */
volatile uint32_t g_wdg_count = 0; /**< WDG counter */
char              g_message[50];

/*******************************************************************************
 * Static variables
 ******************************************************************************/

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
/** hardfault exception handler */
void
HardFault_Handler(void)
{
    for (;;)
    {
    }
}

void
init_pins(void)
{
    pin_settings_config_t g_pin_mux_init_config_arr0[2] = {
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
    };
    pins_init(2, g_pin_mux_init_config_arr0);
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
        .rx_dma_channel     = 1UL,
        .tx_dma_channel     = 0UL,
    };

    lpuart_init(0, (lpuart_state_t *)(&s_lpuartState[0]), &lpuart0_config);
}

void
init_wdg(void)
{
    WDOG_t *base = WDOG_BASE_PTRS;

    /* WDOG user configuration 0 */
    const wdog_user_config_t WDOG_Cfg0 = {
        .clkSource       = WDOG_LPO_CLOCK,
        .updateEnable    = true,
        .intEnable       = true,
        .winEnable       = false,
        .windowValue     = 0U,
        .timeoutValue    = 2000U,
        .prescalerEnable = true /* WDOG pre-scaler*/
    };

    /* Configure the WDOG module */
    wdog_config(base, &WDOG_Cfg0);
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

    uint32_t *SCG_OSCR_REG = (uint32_t *)0x40064008;
    SCG_OSCR_REG[0]        = SCG_OSCR_REG[0] | 0x01000000; // for PLL source: OSC

    /* Initialize and configure clocks */
    clock_init_config(gp_clock_man_configs_arr,
                      CLOCK_MANAGER_CONFIG_CNT,
                      gp_clock_man_callback_arr,
                      CLOCK_MANAGER_CALLBACK_CNT);
    clock_update_config(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    init_pins();
    init_uart();
    lpuart_send_data_blocking(0, (uint8_t *)STARTUP_MSG, strlen(STARTUP_MSG), TIMEOUT);
    init_wdg();

    for (;;)
    {
        g_wdg_count = wdog_get_counter(0);
        if (g_wdg_count >= WDG_FEED_TIME)
        {
            wdog_trigger(0); /*reset watch dog counter*/
            do
            {
                g_wdg_count = wdog_get_counter(0);
                if (g_wdg_count >= WDG_FEED_TIME)
                {
                    wdog_trigger(0);
                }
                else
                {
                    break;
                }
            } while (g_wdg_count >= WDG_FEED_TIME);
        }
    }

    return g_exit_code;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
