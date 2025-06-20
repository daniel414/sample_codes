/**
 * @file main.c
 * @brief sample program for gpio
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "clock_config.h"
#include "pins_driver.h"
#include "osif_driver.h"
#include "lpuart_driver.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define LED_NUMBER  4U    // PTE12, PTE13, PTE14, PTE15
#define TIMEOUT     200UL /* Timeout in ms for blocking operations */
#define BUFFER_SIZE 256UL

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int g_exit_code = 0;
uint8_t      g_buffer0[BUFFER_SIZE];
uint8_t      g_buffer0_idx;
char         g_message[50];
bool         g_led_on;
uint8_t      g_pte4;

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
PORT_IRQHandler(void)
{
    /* Clear button IRQ flag */
    if (PORTE->PCR[4] & PORT_PCR_ISF_MASK)
    {
        PORTE->PCR[4] |= PORT_PCR_ISF(1);
        //	PORTE->ISFR = PORT_ISFR_ISF_MASK;
        g_pte4 = 1;
    }
}

void
print(const char *p_source_str)
{
    lpuart_send_data_blocking(0, (uint8_t *)p_source_str, strlen(p_source_str), TIMEOUT);
}

void
rx0_callback(void *p_driver_state, uart_event_t event, void *p_user_data)
{
    /* Unused parameters */
    (void)p_driver_state;
    (void)p_user_data;
    /* Check the event type */
    if (event == UART_EVENT_RX_FULL)
    {
        /* The reception stops when newline is received or the buffer is full */
        if ((g_buffer0[g_buffer0_idx] != '\n') && (g_buffer0_idx != (BUFFER_SIZE - 2U)))
        {
            /* Update the buffer index and the rx buffer */
            g_buffer0_idx++;
            lpuart_set_rx_buffer(0, &g_buffer0[g_buffer0_idx], 1U);
        }
    }
}

void
init_pins(void)
{
    pin_settings_config_t pin_mux_init_config_arr0[4] = {
        {
            .p_base           = PORTB,	// UART RX
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
            .p_base           = PORTB,	// UART TX
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
            .int_config       = PORT_INT_FALLING_EDGE,
            .b_clear_int_flag = true,
            .p_gpio_base      = PTE,
            .direction        = GPIO_INPUT_DIRECTION,
            .b_digital_filter = true,
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
    port_digital_filter_config_t DigiFiltConfig = {
        .clock = PORT_DIGITAL_FILTER_LPO_CLOCK,
        .width = 0x1F,
    };
    pins_init(4, pin_mux_init_config_arr0);
    pins_config_digital_filter(PORTE, &DigiFiltConfig);
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
    (void)lpuart_install_rx_callback(0, rx0_callback, NULL);
}

void
gpio_test(void)
{
    uint8_t  idx        = 0;
    g_pte4              = 0;
    g_led_on            = true;

    print((const char *)"gpio/port test code: switch function by pte4(touch)\n");
    /* Enable button port IRQ */
    int_enable_irq(PORT_IRQn);

    for (;;)
    {
        osif_time_delay(1000);
        if (g_pte4 == 1)
        {
            int_disable_irq(PORT_IRQn);
            osif_time_delay(300);
            int_enable_irq(PORT_IRQn);
            if (g_led_on == false)
            {
                print((const char *)"led blinky\n");
                g_led_on = true;
            }
            else
            {
                print((const char *)"stop led blinky\n");
                g_led_on = false;
            }
            g_pte4 = 0;
        }

        if (g_led_on == true)
        {
            idx++;
            if (idx % 2)
                pins_clear_pins(PTE, 0x8000);
            else
                pins_set_pins(PTE, 0x8000);
        }
    }
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
    gpio_test();

    return g_exit_code;
}

/*** end of file ***/
