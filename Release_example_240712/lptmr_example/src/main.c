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
#include "lptmr_driver.h"
#include "pins_driver.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define LED_FLASH_TIME 500U /**< 500 x 1msec = 500msec */

/*******************************************************************************
 * Private typedefs
 ******************************************************************************/

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int      g_exit_code = 0; /**< exit code */
volatile uint32_t g_led_count = 0; /**< led blink timer */

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

/** lptmr exception handler */
void
LPTMR0_IRQHandler(void)
{
    LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;
    g_led_count++;
}

/** lptmr initialization */
void
init_lptmr(void)
{
    const lptmr_config_t lptmr_config0 = {.work_mode          = LPTMR_WORKMODE_TIMER,
                                          .b_dma_request      = false,
                                          .b_interrupt_enable = true,
                                          .b_free_run         = false,
                                          .compare_value      = 999UL,
                                          .clock_select       = LPTMR_CLOCKSOURCE_SIRCDIV2,
                                          .prescaler          = LPTMR_PRESCALE_8_GLITCHFILTER_4,
                                          .pin_select         = LPTMR_PINSELECT_TRGMUX,
                                          .pin_polarity       = LPTMR_PINPOLARITY_RISING};

    lptmr_init(0, &lptmr_config0, true);
    g_led_count = 0;
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

    /* Initialize for GPIO */
    pins_set_mux_mode_sel(PORTE, 15, PORT_MUX_AS_GPIO);
    pins_disable_digital_filter(PORTE, 15); /*disable filter*/
    pins_set_pin_direction(PTE, 15, 1);     /*Set pin out*/

    init_lptmr();

    for (;;)
    {
        if (LED_FLASH_TIME <= g_led_count)
        {
            g_led_count = 0;
            pins_toggle_pins(PTE, 0x8000); /*toggle pin*/
        }
    }

    return g_exit_code;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
