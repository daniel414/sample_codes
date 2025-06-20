/**
 * @file main.c
 * @brief An example program is used to demonstrate a transmission and receiver between master and
 * slave.
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
#include "lpi2c_interface.h"

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
/**
 * @brief A callback function for LPI2C rendering events, which occur during interrupt process.
 *
 * @param[in] slave_event Indicate which event is occurring
 * @param[in] p_param User defined object interface to be refer to
 */
void lpi2cif_inst1_slave_callback(i2c_slave_event_t slave_event, void *p_param);

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
    uint8_t     loop_cntr;
    uint16_t    xfer_byte_cnt;
    status_t    sts_for_xfer;
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

    memcpy(g_master_tx_buf, "How are you doing!", sizeof("How are you doing!"));
    memcpy(g_slave_tx_buf, "I am doing fine.^^", sizeof("I am doing fine.^^"));
    osif_sema_create(&sema_await_time_up, 0u);
    /* Initialize LPI2C1 (Slave). */
    lpi2cif_slave_init(INST_LPI2C1, &g_lpi2cif_inst1_state, &g_lpi2cif_inst1_slave_config);
    g_lpi2cif_inst1_state.slave.callback         = lpi2cif_inst1_slave_callback;
    g_lpi2cif_inst1_state.slave.p_callback_param = NULL;
    /* Initialize LPI2C0 (Master Send/Receive). */
    lpi2cif_master_init(INST_LPI2C0, &g_lpi2cif_inst0_state, &g_lpi2cif_inst0_master_config);
    for (loop_cntr = 1, xfer_byte_cnt = sizeof("How are you doing!");
         !g_exit_code && (0u < xfer_byte_cnt);
         loop_cntr++, xfer_byte_cnt--)
    {
        printf("Send length at loop %d: %d.\n", loop_cntr, xfer_byte_cnt);
        sts_for_xfer = STATUS_SUCCESS;
        sts_for_xfer = lpi2cif_master_send_data_blocking(
            INST_LPI2C0, g_master_tx_buf, xfer_byte_cnt, true, BLOCKING_XFER_TIMEOUT);
        if (STATUS_SUCCESS == sts_for_xfer)
        {
            printf("Status: Success!\n");
        }
        else
        {
            g_exit_code = -1;
            printf("Status: Failure!\n");
            break;
        }
        printf("Receive length at loop %d: %d.\n", loop_cntr, xfer_byte_cnt);
        sts_for_xfer = STATUS_SUCCESS;
        sts_for_xfer = lpi2cif_master_rcv_data_blocking(
            INST_LPI2C0, g_master_rx_buf, xfer_byte_cnt, true, OSIF_WAIT_FOREVER);
        if (STATUS_SUCCESS == sts_for_xfer)
        {
            printf("Status: Success!\n");
        }
        else
        {
            g_exit_code = -2;
            printf("Status: Failure!\n");
        }
        osif_sema_wait(&sema_await_time_up, 1u);
    }
    /* Wait for a while for UART FIFO empty. */
    osif_sema_wait(&sema_await_time_up, 300u);
    lpi2cif_master_deinit(INST_LPI2C0);
    lpi2cif_slave_deinit(INST_LPI2C1);
    osif_sema_destroy(&sema_await_time_up);
    sema_await_time_up = 255u;

    return g_exit_code;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/
void
lpi2cif_inst1_slave_callback(i2c_slave_event_t slave_event, void *p_param)
{
    /* Get the instance number from p_param */
    uint32_t instance;
    // instance = (uint32_t)p_param;
    instance = INST_LPI2C1;
    /* Depending on the event received, set the buffers or abort the transfer */
    switch (slave_event)
    {
        case I2C_SLAVE_EVENT_RX_REQ:
        {
            /**
             * If the bus master requests data, then set the destination RX buffer
             * and accepted transfer size
             */
            lpi2cif_slave_set_rx_buffer(instance, g_slave_rx_buf, GENERIC_BUFFER_SIZE);
        }
        break;
        case I2C_SLAVE_EVENT_TX_REQ:
        {
            /**
             * If the bus master sends data, then set the source TX buffer
             * and accepted transfer size
             */
            lpi2cif_slave_set_tx_buffer(instance, g_slave_tx_buf, GENERIC_BUFFER_SIZE);
        }
        break;
        case I2C_SLAVE_EVENT_TX_EMPTY:
        {
            /**
             * If the TX buffer is empty. Because the example does not handle
             * this case there is no action taken.
             */
        }
        break;
        case I2C_SLAVE_EVENT_RX_FULL:
        {
            /**
             * If the RX buffer is full, check the slave receive buffer is correct
             */
        }
        break;
        case I2C_SLAVE_EVENT_STOP:
        {
            /**
             * This case is used when a stop condition is on the bus. Because
             * the example does not handle this case there is no action taken.
             */
#if defined(REFER_TO_REF_DESIGN)
#else  /* REFER_TO_REF_DESIGN */
            lpi2cif_slave_release_buffer(
                instance, g_slave_tx_buf, GENERIC_BUFFER_SIZE, g_slave_rx_buf, GENERIC_BUFFER_SIZE);
#endif /* REFER_TO_REF_DESIGN */
        }
        break;
        default:
        {
        }
        break;
    }
}

/*** end of file ***/
