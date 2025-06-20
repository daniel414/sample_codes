/**
 * @file main.c
 * @brief An example program is used to demonstrate a duplex transfer between master and slave.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pins_driver.h"
#include "clock_config.h"
#include "osif_driver.h"
#include "lpspi_interface.h"
#include "lpuart_driver.h"
#include "edma_driver.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define TIMEOUT                 500U
#define STARTUP_MSG             "SPI DMA example\r\n"

#define NUM_OF_CONFIGURED_PINS0 6
#define GENERIC_BUFFER_SIZE   512u  /**< The size of peripherals bus interfaces buffer. */
#define BLOCKING_XFER_TIMEOUT 1000u /**< 1000ms */

#define EDMA_CONFIGURED_CHANNELS_COUNT  2U
#define EDMA_CHN0_NUMBER   0U
#define EDMA_CHN1_NUMBER   1U
/*******************************************************************************
 * Private typedefs
 ******************************************************************************/

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int32_t g_exit_code = 0; /**< exit code */
edma_state_t dmaController_State;

edma_channel_config_t dmaControllerChn0_Config = {
    .channel_priority = EDMA_CHN_DEFAULT_PRIORITY,
    .virt_chn_config = EDMA_CHN0_NUMBER,
    .source = EDMA_REQ_LPSPI0_TX,
    .callback = NULL,
    .p_callback_param = NULL,
    .b_enable_trigger = false,
};

edma_channel_config_t dmaControllerChn1_Config = {
    .channel_priority = EDMA_CHN_DEFAULT_PRIORITY,
    .virt_chn_config = EDMA_CHN1_NUMBER,
    .source = EDMA_REQ_LPSPI0_RX,
    .callback = NULL,
    .p_callback_param = NULL,
    .b_enable_trigger = false,
};

edma_chn_state_t dmaControllerChn0_State;
edma_chn_state_t dmaControllerChn1_State;

edma_chn_state_t * const edmaChnStateArray[] = {
	    &dmaControllerChn0_State,
	    &dmaControllerChn1_State,
};

uint8_t g_master_tx_buf[GENERIC_BUFFER_SIZE] = {0u};
uint8_t g_master_rx_buf[GENERIC_BUFFER_SIZE] = {0u};

const uint8_t read_cmd[] = {0x3, 0, 0, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
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
    pin_settings_config_t g_pin_mux_InitConfigArr0[NUM_OF_CONFIGURED_PINS0] =
    {
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
    		        /** LPSPI0_SCK */
    		        .p_base           = PORTB,
    		        .pin_port_idx     = 2U,
    		        .pull_config      = PORT_INTERNAL_PULL_NOT_ENABLED,
    		        .drive_select     = PORT_LOW_DRIVE_STRENGTH,
    		        .mux              = PORT_MUX_ALT3,
    		        .b_pin_lock       = false,
    		        .int_config       = PORT_DMA_INT_DISABLED,
    		        .b_clear_int_flag = false,
    		        .p_gpio_base      = NULL,
    		        .b_digital_filter = false,
			},
			{
    		        /** LPSPI0_DI */
    		        .p_base           = PORTB,
    		        .pin_port_idx     = 3U,
    		        .pull_config      = PORT_INTERNAL_PULL_NOT_ENABLED,
    		        .drive_select     = PORT_LOW_DRIVE_STRENGTH,
    		        .mux              = PORT_MUX_ALT3,
    		        .b_pin_lock       = false,
    		        .int_config       = PORT_DMA_INT_DISABLED,
    		        .b_clear_int_flag = false,
    		        .p_gpio_base      = NULL,
    		        .b_digital_filter = false,
			},
			{
    		        /** LPSPI0_DO */
    		        .p_base           = PORTB,
    		        .pin_port_idx     = 4U,
    		        .pull_config      = PORT_INTERNAL_PULL_NOT_ENABLED,
    		        .drive_select     = PORT_LOW_DRIVE_STRENGTH,
    		        .mux              = PORT_MUX_ALT3,
    		        .b_pin_lock       = false,
    		        .int_config       = PORT_DMA_INT_DISABLED,
    		        .b_clear_int_flag = false,
    		        .p_gpio_base      = NULL,
    		        .b_digital_filter = false,
			},
			{
    		        /** LPSPI0_CS */
    		        .p_base           = PORTB,
    		        .pin_port_idx     = 5U,
    		        .pull_config      = PORT_INTERNAL_PULL_NOT_ENABLED,
    		        .drive_select     = PORT_LOW_DRIVE_STRENGTH,
    		        .mux              = PORT_MUX_ALT4,
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
        .rx_dma_channel     = 1UL,
        .tx_dma_channel     = 0UL,
    };

    lpuart_init(0, (lpuart_state_t *)(&s_lpuartState[0]), &lpuart0_config);
}

void
init_edma(void)
{
	const edma_user_config_t dmaController_InitConfig = {
	    .chn_arbitration = EDMA_ARBITRATION_FIXED_PRIORITY,
	    .b_halt_on_error = false
	};

	const edma_channel_config_t * const edmaChnConfigArray[] = {
	    &dmaControllerChn0_Config,
	    &dmaControllerChn1_Config,
	};

	/* Initialize eDMA module & channels */
	edma_init(&dmaController_State,
				&dmaController_InitConfig,
				edmaChnStateArray,
				edmaChnConfigArray,
				EDMA_CONFIGURED_CHANNELS_COUNT);
}

void
init_spi(void)
{
	lpspiif_master_config_t lpspiif_inst0_master_config = g_lpspiif_inst0_master_config;
	lpspiif_inst0_master_config.b_pcs_continuous = true;
	lpspiif_inst0_master_config.xfer_type = LPSPIIF_USING_DMA;
	lpspiif_inst0_master_config.rx_dma_channel = 1U;

	/* Initialize LPSPI0 as master. */
	lpspiif_master_init(INST_LPSPI0, &g_lpspiif_inst0_state, &lpspiif_inst0_master_config);
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
	char msg[200];
    status_t sts_for_xfer = STATUS_SUCCESS;

    /* Initialize */
    pins_chip_init();
    board_init();

    /* Initialize and configure clocks */
    clock_init_config(gp_clock_man_configs_arr,
                      CLOCK_MANAGER_CONFIG_CNT,
                      gp_clock_man_callback_arr,
                      CLOCK_MANAGER_CALLBACK_CNT);
    clock_update_config(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    /* Configure pins. */
    init_gpio();
    init_uart();
    init_edma();
    init_spi();

    lpuart_send_data_blocking(0, (uint8_t *)STARTUP_MSG, strlen(STARTUP_MSG), TIMEOUT);

	memcpy(g_master_tx_buf,	read_cmd, sizeof(read_cmd));

	sts_for_xfer = lpspiif_master_xfer_blocking(
	            INST_LPSPI0, g_master_tx_buf, g_master_rx_buf, sizeof(read_cmd), BLOCKING_XFER_TIMEOUT);

	if (STATUS_SUCCESS == sts_for_xfer)
	{
		//lpuart_send_data_blocking(0, (uint8_t *)STARTUP_MSG, strlen(STARTUP_MSG), TIMEOUT);
		sprintf(msg,"TX_Buf: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
				g_master_tx_buf[0], g_master_tx_buf[1], g_master_tx_buf[2], g_master_tx_buf[3],
				g_master_tx_buf[4], g_master_tx_buf[5], g_master_tx_buf[6], g_master_tx_buf[7],
				g_master_tx_buf[8], g_master_tx_buf[9], g_master_tx_buf[10], g_master_tx_buf[11],
				g_master_tx_buf[12], g_master_tx_buf[13], g_master_tx_buf[14], g_master_tx_buf[15]);
		lpuart_send_data_blocking(0, (uint8_t *)msg, strlen(msg), TIMEOUT);


		sprintf(msg,"RX_Buf: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
				g_master_rx_buf[0], g_master_rx_buf[1], g_master_rx_buf[2], g_master_rx_buf[3],
				g_master_rx_buf[4], g_master_rx_buf[5], g_master_rx_buf[6], g_master_rx_buf[7],
				g_master_rx_buf[8], g_master_rx_buf[9], g_master_rx_buf[10], g_master_rx_buf[11],
				g_master_rx_buf[12], g_master_rx_buf[13], g_master_rx_buf[14], g_master_rx_buf[15]);
		lpuart_send_data_blocking(0, (uint8_t *)msg, strlen(msg), TIMEOUT);
	}
	else
	{
		lpuart_send_data_blocking(0, (uint8_t *)"Transfer failed!\r\n", strlen("Transfer failed!\r\n"), TIMEOUT);
	}

    lpspiif_master_deinit(INST_LPSPI0);

    return g_exit_code;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
