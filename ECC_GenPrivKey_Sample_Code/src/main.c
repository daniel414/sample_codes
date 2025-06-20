/**
 * @file main.c
 * @brief sample program for uart, gpio and pufs
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
#include "lpuart_driver.h"
#include "pufs_test.h"
#include "pufs_ka.h"
#include "pufs_hmac.h"
#include "pufs_rt.h"
#include "pufs_ecp.h"
#include "pufs_ecc.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define LED_FLASH_TIME 20000U // 20000 x 10usec = 200000usec = 200msec
#define TIMEOUT        200UL  /* Timeout in ms for blocking operations */
#define BUFFER_SIZE    256UL

/*******************************************************************************
 * Private typedefs
 ******************************************************************************/

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int      g_exit_code = 0;
volatile uint32_t g_led_count = 0;
/* Buffer used to receive data from the console */
uint8_t g_buffer0[BUFFER_SIZE];
uint8_t g_buffer0_idx;
char    g_message[50];
bool    g_led_on;

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
HardFault_Handler(void)
{
    for (;;)
    {
    }
}

void
LPTMR0_IRQHandler(void)
{
    LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;

    g_led_count++;
    if ((LED_FLASH_TIME <= g_led_count) && (g_led_on))
    {
        g_led_count = 0;
        pins_toggle_pins(PTE, 0x8000); /*toggle pin*/
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
    pin_settings_config_t pin_mux_init_config_arr0[3] = {
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
            .pin_port_idx     = 15U,
            .pull_config      = PORT_INTERNAL_PULL_NOT_ENABLED,
            .drive_select     = PORT_LOW_DRIVE_STRENGTH,
            .mux              = PORT_MUX_AS_GPIO,
            .b_pin_lock       = false,
            .int_config       = PORT_INT_RISING_EDGE,
            .b_clear_int_flag = false,
            .p_gpio_base      = PTE,
            .direction        = GPIO_OUTPUT_DIRECTION,
            .b_digital_filter = false,
            .init_value       = 0U,
        },
    };
    pins_init(3, pin_mux_init_config_arr0);
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
init_lptmr(void)
{
    const lptmr_config_t lptmr_config0 = {.work_mode          = LPTMR_WORKMODE_TIMER,
                                          .b_dma_request      = false,
                                          .b_interrupt_enable = true,
                                          .b_free_run         = false,
                                          .compare_value      = 9UL,
                                          .clock_select       = LPTMR_CLOCKSOURCE_SIRCDIV2,
                                          .prescaler          = LPTMR_PRESCALE_8_GLITCHFILTER_4,
                                          .pin_select         = LPTMR_PINSELECT_TRGMUX,
                                          .pin_polarity       = LPTMR_PINPOLARITY_RISING};
    lptmr_init(0, &lptmr_config0, true);
    g_led_count = 0;
}

/**
 * @brief The main function for the project.
 * @details The startup initialization sequence is the following:
 * - main()
 */
int
main(void)
{
    /* Write your code here */
    uint32_t bytes_remaining = 0;
    status_t status;
    g_led_on = false;

    pins_chip_init();

    uint32_t *SCG_OSCR_REG = (uint32_t *)0x40064008;
    SCG_OSCR_REG[0]        = SCG_OSCR_REG[0] | 0x01000000; // for PLL source: OSC

    /* Initialize and configure clocks */
    clock_init_config(gp_clock_man_configs_arr,
                      CLOCK_MANAGER_CONFIG_CNT,
                      gp_clock_man_callback_arr,
                      CLOCK_MANAGER_CALLBACK_CNT);
    clock_update_config(0U, CLOCK_MANAGER_POLICY_AGREEMENT);
    pufs_status_t check = SUCCESS;
    pufs_ec_point_st puk;
    pufs_ka_slot_t prvslot = PRK_0;
    pufs_rt_slot_t pufslot = PUFSLOT_1;
    const char *salt = "pufsecurity salt";
    const char *info = "pufsecurity info";


    init_pins();
    init_lptmr();
    init_uart();
    print("ECC PrivKey Check\n");
    check = pufs_initialize();
    if(check == SUCCESS){
    	print("PUFcc Init Pass\n");
    	pufs_ecp_set_curve_byname(NISTB163);
		if ((check = pufs_ecp_gen_sprk(prvslot, pufslot, (uint8_t *)salt, 16, (uint8_t *)info, 16, HASH_DEFAULT)) == SUCCESS){
			print("Private Key Generation Success\n");
			check = pufs_ecp_gen_puk(&puk, PRKEY, PRK_0);
		}else{
			print("Private Key Generation Fail\n");

		}

		char puk_x_str[145], puk_y_str[145];
		for(int i=0;i<44;i++){
			sprintf(&puk_x_str[i*2], "%02x", (&puk)->x[i]);
			sprintf(&puk_y_str[i*2], "%02x", (&puk)->y[i]);
	   }
		puk_x_str[44]=puk_y_str[44]='\0';
		print("Pubkey x-coordinate: ");
		print(puk_x_str);
		print("\n");

		print("Pubkey y-coordinate: ");
		print(puk_y_str);
		print("\n");

    }else{
    	print("PUFcc Init Fail\n");

    }



    while(1);


}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
