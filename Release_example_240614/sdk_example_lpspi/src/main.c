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
#include "pin_mux.h"
#include "lpspi_interface.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define GENERIC_BUFFER_SIZE   512u  /**< The size of peripherals bus interfaces buffer. */
#define BLOCKING_XFER_TIMEOUT 1000u /**< 1000ms */

/*******************************************************************************
 * Private typedefs
 ******************************************************************************/

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int32_t g_exit_code = 0; /**< exit code */
/** Buffers are used for both of transmit and receive data with respect to distinct master or slave
 * site. */
uint8_t g_master_tx_buf[GENERIC_BUFFER_SIZE] = {0u};
uint8_t g_master_rx_buf[GENERIC_BUFFER_SIZE] = {0u};
uint8_t g_slave_tx_buf[GENERIC_BUFFER_SIZE]  = {0u};
uint8_t g_slave_rx_buf[GENERIC_BUFFER_SIZE]  = {0u};

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
    uint8_t     tmp_cntr;
    uint16_t    xfer_byte_cnt;
    uint32_t    remain_byte_cnt;
    status_t    sts_for_xfer       = STATUS_SUCCESS;
    semaphore_t sema_await_time_up = 255u;

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

    /* Configure pins. */
    pins_init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_init_cfg_arr);

    osif_sema_create(&sema_await_time_up, 0u);
    /* Initialize LPSPI1 as slave. */
    lpspiif_slave_init(INST_LPSPI1, &g_lpspiif_inst1_state, &g_lpspiif_inst1_slave_config);
    {
        lpspiif_master_config_t lpspiif_inst0_master_config = g_lpspiif_inst0_master_config;
        lpspiif_inst0_master_config.b_pcs_continuous        = true;
        /* Initialize LPSPI0 as master. */
        lpspiif_master_init(INST_LPSPI0, &g_lpspiif_inst0_state, &lpspiif_inst0_master_config);
    }
    for (; !g_exit_code;)
    {
        xfer_byte_cnt = sizeof("Here is a transmission from LPSPI0.");
        memcpy(g_master_tx_buf,
               "Here is a transmission from LPSPI0.",
               sizeof("Here is a transmission from LPSPI0."));
        memcpy(g_slave_tx_buf, "Ready to go on LPSPI1.", sizeof("Ready to go on LPSPI1."));
        /* Queue a job on slave site. */
        lpspiif_slave_xfer(INST_LPSPI1, g_slave_tx_buf, g_slave_rx_buf, xfer_byte_cnt);
        printf("Master: %s\n", g_master_tx_buf);
        printf("Slave: %s\n", g_slave_tx_buf);
        /* Send a transaction from master site. */
        sts_for_xfer = lpspiif_master_xfer_blocking(
            INST_LPSPI0, g_master_tx_buf, g_master_rx_buf, xfer_byte_cnt, BLOCKING_XFER_TIMEOUT);
        if (STATUS_SUCCESS == sts_for_xfer)
        {
            g_exit_code = 1;
        }
        else
        {
            g_exit_code = -1;
            lpspiif_slave_break_xfer_off(INST_LPSPI1);
            printf("Transmission failure!");
            break;
        }
        for (tmp_cntr = 0u; 3u > tmp_cntr; tmp_cntr++)
        {
            /* Check slave's status. */
            sts_for_xfer = lpspiif_slave_get_xfer_status(INST_LPSPI1, &remain_byte_cnt);
            if (STATUS_SUCCESS == sts_for_xfer)
            {
                g_exit_code = 2;
                break;
            }
            else if (STATUS_ERROR == sts_for_xfer)
            {
                g_exit_code = -2;
                lpspiif_slave_break_xfer_off(INST_LPSPI1);
                printf("Slave status error!");
                break;
            }
            osif_sema_wait(&sema_await_time_up, 1u);
        }
        if ((3u <= tmp_cntr) && (0 <= g_exit_code))
        {
            g_exit_code = -3;
            lpspiif_slave_break_xfer_off(INST_LPSPI1);
            printf("Slave stall failure!");
            break;
        }
        printf("\"%s\" - Master copy that.\n", g_master_rx_buf);
        printf("\"%s\" - Slave copy that.\n", g_slave_rx_buf);
        /* Compare RX data with respect to distinct master or slave. */
        if ((0 == strcmp((const char *)g_master_rx_buf, (const char *)g_slave_tx_buf)) &&
            (0 == strcmp((const char *)g_slave_rx_buf, (const char *)g_master_tx_buf)))
        {
            printf("Matched!\n");
        }
        else
        {
            g_exit_code = -4;
            printf("Mismatched!\n");
        }
    }
    /* Wait for a while for UART FIFO empty. */
    osif_sema_wait(&sema_await_time_up, 300u);
    lpspiif_master_deinit(INST_LPSPI0);
    lpspiif_slave_deinit(INST_LPSPI1);
    osif_sema_destroy(&sema_await_time_up);
    sema_await_time_up = 255u;

    return g_exit_code;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
