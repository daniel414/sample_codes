/**
 * @file main.c
 * @brief sample program for uart, gpio and power
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "osif_driver.h"
#include "clock_config.h"
#include "pins_driver.h"
#include "lptmr_driver.h"
#include "lpuart_driver.h"
#include "power_manager.h"
#include "clock_tw9001.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define TIMEOUT     200UL /* Timeout in ms for blocking operations */
#define BUFFER_SIZE 256UL
#define SMC_TIMEOUT 1000U
#define TEST_PW1

#define MODE_RUN   (0u) /* Run                 */
#define MODE_VLPR  (1u) /* Very low power run  */
#define MDOE_STOP1 (2u) /* Stop option 1       */
#define MODE_STOP2 (3u) /* Stop option 2       */
#define MODE_VLPS  (4u) /* Very low power stop */

#define MODE_RUN_VLPR (1u)
#define MODE_VLPR_RUN (2u)
#define MODE_RUN_VLPS (3u)
#define MODE_VLPS_RUN (4u)
#define MODE_End      (5u)

power_manager_user_config_t g_run_config = {
    .power_mode            = POWER_MANAGER_RUN,
    .b_sleep_on_exit_value = false,
};
power_manager_user_config_t g_vlpr_config = {
    .power_mode            = POWER_MANAGER_VLPR,
    .b_sleep_on_exit_value = false,
};
power_manager_user_config_t g_stop1_config = {
    .power_mode            = POWER_MANAGER_STOP1,
    .b_sleep_on_exit_value = false,
};
power_manager_user_config_t g_stop2_config = {
    .power_mode            = POWER_MANAGER_STOP2,
    .b_sleep_on_exit_value = false,
};
power_manager_user_config_t g_vlps_config = {
    .power_mode            = POWER_MANAGER_VLPS,
    .b_sleep_on_exit_value = false,
};
power_manager_user_config_t *gp_power_configs_arr[] = {
    &g_run_config,
    &g_vlpr_config,
    &g_stop1_config,
    &g_stop2_config,
    &g_vlps_config,
};

status_t pwr_callback0(power_manager_notify_struct_t *p_notify,
                       power_manager_callback_data_t *p_dataPtr);

// Callback configuration structure g_callback_cfg0
power_manager_callback_user_config_t g_callback_cfg0 = {
    .callback_function = &pwr_callback0,
    .callback_type     = POWER_MANAGER_CALLBACK_BEFORE_AFTER,
    .p_callback_data   = (void *)0,
};

// Callback configuration structures array
power_manager_callback_user_config_t *gp_callbacks_configs_arr[] = {&g_callback_cfg0};

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int g_exit_code = 0;
uint8_t      g_buffer0[BUFFER_SIZE];
uint8_t      g_buffer0_idx;
char         g_message[50];
volatile int g_transferCompleteIteration;
uint32_t    *SCG_OSCR_REG = (uint32_t *)0x40064008;
uint32_t     reg_val;

/*******************************************************************************
 * Global functions
 ******************************************************************************/
void
HardFault_Handler(void)
{
    for (;;)
    {
    }
}

void
print(const char *p_source_str)
{
    uint32_t bytes_remaining;
    /* Send data via LPUART */
    lpuart_send_data(0, (uint8_t *)p_source_str, strlen(p_source_str));
    /* Wait for transmission to be successful */
    while (lpuart_get_transmit_status(0, &bytes_remaining) != STATUS_SUCCESS)
    {
    };
}

void
PORT_IRQHandler(void)
{
    /* Clear button IRQ flag */
    if (PORTE->PCR[4] & PORT_PCR_ISF_MASK)
    {
        PORTE->PCR[4] |= PORT_PCR_ISF(1);
        g_transferCompleteIteration = 1;
    }
}

// Definition of power manager callback
status_t
pwr_callback0(power_manager_notify_struct_t *p_notify, power_manager_callback_data_t *p_dataPtr)
{
    status_t ret = STATUS_SUCCESS;
    sprintf(g_message, "pwr_callback0: p_notify = %d\n", p_notify->pwr_notify_type);
    print(g_message);
    return ret;
}

void
init_pins(void)
{
    pin_settings_config_t pin_mux_init_config_arr0[4] = {
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
            .p_base           = PORTE,
            .pin_port_idx     = 4U,
            .pull_config      = PORT_INTERNAL_PULL_NOT_ENABLED,
            .drive_select     = PORT_LOW_DRIVE_STRENGTH,
            .mux              = PORT_MUX_AS_GPIO,
            .b_pin_lock       = false,
            .int_config       = PORT_INT_EITHER_EDGE,
            .b_clear_int_flag = false,
            .p_gpio_base      = PTE,
            .direction        = GPIO_INPUT_DIRECTION,
            .b_digital_filter = false,
            .init_value       = 0U,
        },
        {
            .p_base           = PORTE,
            .pin_port_idx     = 15U,
            .pull_config      = PORT_INTERNAL_PULL_NOT_ENABLED,
            .drive_select     = PORT_LOW_DRIVE_STRENGTH,
            .mux              = PORT_MUX_AS_GPIO,
            .b_pin_lock       = false,
            .int_config       = PORT_DMA_INT_DISABLED,
            .b_clear_int_flag = false,
            .p_gpio_base      = PTE,
            .direction        = GPIO_OUTPUT_DIRECTION,
            .b_digital_filter = false,
            .init_value       = 0U,
        },
    };
    pins_init(4, pin_mux_init_config_arr0);
}

void
init_uart(void)
{
    static lpuart_state_t      s_lpuart_state[1];
    const lpuart_user_config_t lpuart0_config = {
        .baud_rate          = 115200UL,
        .parity_mode        = LPUART_PARITY_DISABLED,
        .bit_count_per_char = LPUART_8_BITS_PER_CHAR,
        .stop_bit_count     = LPUART_ONE_STOP_BIT,
        .transfer_type      = LPUART_USING_INTERRUPTS,
        .rx_dma_channel     = 1UL,
        .tx_dma_channel     = 0UL,
    };
    lpuart_init(0, (lpuart_state_t *)(&s_lpuart_state[0]), &lpuart0_config);
}

void
set_rsbosc_dis(void)
{
    reg_val = SCG_OSCR_REG[0];
    SCG_OSCR_REG[0] &= ~0x01000000; // for PLL source: OSC
}

void
set_rsbosc_en(void)
{
    osif_time_delay(20); // wait stable
    SCG_OSCR_REG[0] = reg_val;
}

void
init_power(void)
{
    // Calling of init method
    power_init(&gp_power_configs_arr, 5U, &gp_callbacks_configs_arr, 1U);
    print((const char *)"********** power_init.\n");
}

void
pw_test(void)
{
    uint8_t  powermode = MODE_RUN;
    uint32_t count     = 0;
    uint32_t value;

    print((const char *)"Power Mode Switch, please trigger pte4(touch) to wakeup !! \n");

    g_transferCompleteIteration = 0;

    while (MODE_End > powermode)
    {
        if (g_transferCompleteIteration)
        {
            powermode++;
            osif_time_delay(300);
            g_transferCompleteIteration = 0;
            switch (powermode)
            {
                case MODE_RUN_VLPR:
                    print((const char *)"RUN -> VLPR\n");
                    set_rsbosc_dis();
                    SMC->PMPROT = (SMC->PMPROT & ~(SMC_PMPROT_AVLP_MASK)) | SMC_PMPROT_AVLP(1U);
                    SMC->PMCTRL = (SMC->PMCTRL & ~(SMC_PMCTRL_RUNM_MASK)) | SMC_PMCTRL_RUNM(2);
                    for (count = 1U; count < SMC_TIMEOUT; count++)
                    {
                        value = SMC->PMSTAT & SMC_PMSTAT_PMSTAT_MASK;
                        if (value & 0x04) // Check VLPR mode
                        {
                            /* Power mode has been changed successfully */
                            break;
                        }
                    }
                    if (SMC_TIMEOUT > count)
                    {
                        print((const char *)"OK\n");
                    }
                    else
                    {
                        print((const char *)"FAIL\n");
                    }
                    break;
                case MODE_VLPR_RUN:
                    print((const char *)"VLPR -> RUN\n");
                    SMC->PMCTRL = (SMC->PMCTRL & ~(SMC_PMCTRL_RUNM_MASK)) | SMC_PMCTRL_RUNM(0);
                    for (count = 1U; count < SMC_TIMEOUT; count++)
                    {
                        value = SMC->PMSTAT & SMC_PMSTAT_PMSTAT_MASK;
                        if (value & 0x01) // Check RUN mode
                        {
                            /* Power mode has been changed successfully */
                            break;
                        }
                    }
                    set_rsbosc_en();
                    if (SMC_TIMEOUT > count)
                    {
                        print((const char *)"OK\n");
                    }
                    else
                    {
                        print((const char *)"FAIL\n");
                    }
                    break;
                case MODE_RUN_VLPS:
                    print((const char *)"RUN -> VLPS\n");
                    set_rsbosc_dis();
                    SysTick->CSR = 0;
                    SMC->PMPROT  = (SMC->PMPROT & ~(SMC_PMPROT_AVLP_MASK)) | SMC_PMPROT_AVLP(1U);
                    SMC->PMCTRL  = (SMC->PMCTRL & ~(SMC_PMCTRL_STOPM_MASK)) | SMC_PMCTRL_STOPM(2);
                    SCB->SCR |= SCB_SCR_SLEEPDEEP_MASK;
                    STANDBY();
                    set_rsbosc_en();
                    print((const char *)"WakeUp OK!\n");
                    break;
                case MODE_VLPS_RUN:
                    print((const char *)"VLPS -> RUN\n");
                    for (count = 1U; count < SMC_TIMEOUT; count++)
                    {
                        value = SMC->PMSTAT & SMC_PMSTAT_PMSTAT_MASK;
                        if (value & 0x01) // Check RUN mode
                        {
                            /* Power mode has been changed successfully */
                            break;
                        }
                    }
                    if (SMC_TIMEOUT > count)
                    {
                        print((const char *)"OK\n");
                    }
                    else
                    {
                        print((const char *)"FAIL\n");
                    }
                    break;
                default: // VLPR MODE
                    print((const char *)"...End\n");
                    break;
            }
        }
    }
    print((const char *)"End of test Power Mode Switch !! \n");
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
    PORTE->PCR[4] =
        ((PORTE->PCR[4] & ~PORT_PCR_PE_MASK) & ~PORT_PCR_PS_MASK) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
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

    SCG_OSCR_REG[0] = SCG_OSCR_REG[0] | 0x01000000; // for PLL source: OSC

    /* Initialize and configure clocks */
    clock_init_config(gp_clock_man_configs_arr,
                      CLOCK_MANAGER_CONFIG_CNT,
                      gp_clock_man_callback_arr,
                      CLOCK_MANAGER_CALLBACK_CNT);
    clock_update_config(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    init_pins();
    init_uart();
    init_power();

    /* Enable button port IRQ */
    int_enable_irq(PORT_IRQn);

    pw_test();

    return g_exit_code;
}

/*** end of file ***/
