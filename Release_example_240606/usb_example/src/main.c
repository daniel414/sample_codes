/**
 * @file main.c
 * @brief An example program is used to demonstrate a daemon, which such an USB HID compliant device
 * can be mounted on an USB host.
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
#include "usbd_hid_ex.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define BUFFER_SIZE (64u)

/*******************************************************************************
 * Private typedefs
 ******************************************************************************/

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int g_exit_code            = 0;
bool         gb_transaction_arrival = false;
uint8_t      g_resp_buf_len         = 0u;
/* Buffer used to receive data from host interrupt in pipe */
uint8_t g_transaction_buffer[BUFFER_SIZE] = {0u};
uint8_t g_response_buffer[BUFFER_SIZE]    = {0u};

/*******************************************************************************
 * Static variables
 ******************************************************************************/

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/
/**
 * @brief A thread for USB device, which is mounted on a host facility, is working by round-robin
 * scheduling in main loop.
 */
static void thread_usb_dev_mount(void);

/*******************************************************************************
 * Global functions
 ******************************************************************************/
void
usbd_pipe_sram3_acquire_job(uint8_t group_ep_num, uint8_t *p_income, uint32_t byte_cnt)
{
    printf("Income EP: %d.\n", group_ep_num);
    memcpy(g_response_buffer, p_income, byte_cnt);
    g_resp_buf_len         = (uint8_t)byte_cnt;
    gb_transaction_arrival = true;
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
    // status_t status;

    pins_chip_init();

    uint32_t *SCG_OSCR_REG = (uint32_t *)0x40064008;
    SCG_OSCR_REG[0]        = SCG_OSCR_REG[0] | 0x01000000; // for PLL source: OSC

    /* Initialize and configure clocks */
    clock_init_config(gp_clock_man_configs_arr,
                      CLOCK_MANAGER_CONFIG_CNT,
                      gp_clock_man_callback_arr,
                      CLOCK_MANAGER_CALLBACK_CNT);
    clock_update_config(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    usbd_init();
    usbd_install_ep4_rcv_buffer(g_transaction_buffer, sizeof(g_transaction_buffer));

    for (; !g_exit_code;)
    {
        thread_usb_dev_mount();
        if (gb_transaction_arrival)
        {
            uint8_t tmp_cntr;
            gb_transaction_arrival = false;
            for (tmp_cntr = 0u; tmp_cntr < g_resp_buf_len; tmp_cntr++)
            {
                /** Toggle each bit */
                g_response_buffer[tmp_cntr] ^= 0xFFu;
            }
            usbd_pipe_sram3_queue_job(USBD_EP_NB1, g_response_buffer, g_resp_buf_len);
        }
    }

    usbd_deinit();

    return g_exit_code;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/
static void
thread_usb_dev_mount(void)
{
    usbd_check_bus_rst_immediately();
    if (!usbd_check_bus_power_on())
    {
        return;
    }
    usbd_check_ctrl_setup_duplication();
    usbd_check_ctrl_setup_zero_length();
    usbd_ctrl_pipe_proc();
    usbd_intrpt_in_pipe_proc();
    usbd_intrpt_out_pipe_proc();
}

/*** end of file ***/
