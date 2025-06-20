/**
 * @file main.c
 * @brief sample program for uart and crc
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "clock_config.h"
#include "crc_driver.h"
#include "test_crc.h"
#include "uart_driver.h"
#include "pins_driver.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define LED_FLASH_TIME 20000U  /* 20000 x 10usec = 200000usec = 200msec */
#define LED_NUMBER     4U      /* GPIO = PTE12, PTE13, PTE14, PTE15 */
#define UART_PORT      LPUART0 /* Console port */
#define INPUT_DATA     8U      /* CRC input data width */
#define INST_CRC       0U
#define CRC_TEST_NUM   50U

/*******************************************************************************
 * Global variables
 ******************************************************************************/
volatile int g_exit_code = 0;

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
    uart_send_wait(LPUART0, (uint8_t *)p_source_str, strlen(p_source_str));
}

void
init_uart(void)
{
    pins_set_mux_mode_sel(PORTB, 0, PORT_MUX_ALT2); /* Port B0: MUX = ALT2,UART0 TX */
    pins_set_mux_mode_sel(PORTB, 1, PORT_MUX_ALT2); /* Port B1: MUX = ALT2,UART0 RX */

    /* Initialize UART communication */
    uart_init(UART_PORT, 115200);
}

void
crc_test(void)
{
    char               message[30];
    uint32_t           crc_value       = 0;
    crc_user_config_t *test_crc_config = malloc(sizeof(crc_user_config_t));

    print((const char *)"Start test crc checksum !! \n");
    sprintf(message, "Input Data Width = %d bits\n", INPUT_DATA);
    print(message);

    for (uint8_t j = 0U; j < CRC_TEST_NUM; j++)
    {
        test_crc_config->crc_width             = crc_table[j].crc_width;
        test_crc_config->read_transpose        = crc_table[j].read_transpose;
        test_crc_config->polynomial            = crc_table[j].polynomial;
        test_crc_config->write_transpose       = crc_table[j].write_transpose;
        test_crc_config->b_complement_checksum = crc_table[j].b_complement_checksum;
        test_crc_config->seed                  = crc_table[j].seed;
        crc_set_config(0, test_crc_config);

        /* 8-bit writes till end of data buffer */
        crc_write_data(0, crc_table[j].crc_data, crc_table[j].crc_data_size);

        /* Get CRC Result */
        if (crc_table[j].crc_width <= CRC_BITS_32)
        {
            crc_value = crc_get_result(INST_CRC);
        }
        else
        {
            if (crc_table[j].read_transpose >= CRC_TRANSPOSE_BITS_AND_BYTES)
            {
                crc_value = CRC->DATAu.DATA_8.HU;
            }
            else
                crc_value = CRC->DATAu.DATA_8.LL;
        }

        /* Check CRC Result */
        if (crc_table[j].crc_result == crc_value)
        {
            sprintf(message, ">>> Test %d OK", j);
        }
        else
        {
            sprintf(message, ">>> Test %d FAIL", j);
        }
        print(message);
        sprintf(message, "(%s): CRC Result = 0x%lx \n", crc_table[j].name, crc_value);
        print(message);
    }

    free(test_crc_config);
    print((const char *)"End of test crc checksum !! \r\n");
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

    init_uart();
    crc_test();

    for (;;)
    {
        if (g_exit_code != 0)
        {
            break;
        }
    }

    return g_exit_code;
}

/*** end of file ***/
