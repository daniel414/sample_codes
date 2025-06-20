/**
 * @file uart_driver.c (simplify)
 * @brief This file provides access to the Lpuart module.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "uart_driver.h"

/*******************************************************************************
 * Global functions
 ******************************************************************************/
/**
 * @brief read receive buffer
 * @param[in] p_uart       base of UART port
 * @return unsign char received char
 */
uint8_t
uart_read_data_reg(LPUART_t *p_uart)
{
    /* Return the 8-bit data from the receiver */
    return p_uart->DATA;
}

/**
 * @brief write transmit buffer
 * @param[in] p_uart       base of UART port
 * @param[in] u8_char      char to send
 * @ return none
 */
void
uart_write_data_reg(LPUART_t *p_uart, uint8_t u8_char)
{
    /* Send the character */
    p_uart->DATA = (uint8_t)u8_char;
}

/**
 * @brief check whether tx is complete,i.e. data has been sent out.
 * @param[in] p_uart  base of UART port
 * @return
 *               1, Tx complete flag is set
 *               0, Tx complete flag is clear
 * @ Pass/ Fail criteria: none
 */
uint8_t
uart_is_tx_complete(LPUART_t *p_uart)
{
    return ((p_uart->STAT & LPUART_STAT_TC_MASK) >> LPUART_STAT_TC_SHIFT);
}

/**
 * @brief check whether Tx buffer is empty
 * @param[in] p_uart  base of UART port
 * @return
 *               1, Tx buffer is empty
 *               0, Tx buffer is not empty
 * @ Pass/ Fail criteria: none
 */
uint8_t
uart_is_tx_buff_full(LPUART_t *p_uart)
{
    return ((p_uart->STAT & LPUART_STAT_TDRE_MASK) >> LPUART_STAT_TDRE_SHIFT);
}

/**
 * @brief check whether Rx buffer is full, i.e. receive a character
 * @param[in] p_uart  base of UART port
 * @return
 *               1, Rx buffer is full
 *               0, Rx buffer is not full
 * @ Pass/ Fail criteria: none
 */
uint8_t
uart_is_rx_buff_full(LPUART_t *p_uart)
{
    return ((p_uart->STAT & LPUART_STAT_RDRF_MASK) >> LPUART_STAT_RDRF_SHIFT);
}

/**
 * @brief initialize the UART, interrupts disabled, and no hardware
 * flow-control.
 * @param[in] p_uart       base of UART port
 * @param[in] u32_baud     pointer to UART configuration structure
 * @ return none
 * @ Pass/ Fail criteria: none
 */
void
uart_init(LPUART_t *p_uart, uint32_t u32_baud)
{
    uint16_t u16_sbr;
    uint16_t u16_osr = 22;
    uint32_t freq    = 8000000;
    uint32_t div     = 0U;

    /* Enable the clock to the selected UART */
    if (p_uart == LPUART0)
    {
        /* Clock Src= 2 (SIRCDIV2_CLK), Enable clock for LPUART0 regs */
        PCC->PCCn[PCC_LPUART0_INDEX] = (PCC->PCCn[PCC_LPUART0_INDEX] & ~PCC_PCCn_PCS_MASK) |
                                       PCC_PCCn_PCS(2) | PCC_PCCn_CGC_MASK;
    }
    else if (p_uart == LPUART1)
    {
        /* Clock Src= 2 (SIRCDIV2_CLK), Enable clock for LPUART1 regs */
        PCC->PCCn[PCC_LPUART1_INDEX] = (PCC->PCCn[PCC_LPUART1_INDEX] & ~PCC_PCCn_PCS_MASK) |
                                       PCC_PCCn_PCS(2) | PCC_PCCn_CGC_MASK;
    }
    div = (SCG->SIRCDIV & SCG_SIRCDIV_SIRCDIV2_MASK) >> SCG_SIRCDIV_SIRCDIV2_SHIFT;

    if (div != 0U)
    {
        freq = (freq >> (div - 1U));
    }
    else /* Output disabled. */
    {
        freq = 0U;
    }

    /* Make sure that the transmitter and receiver are disabled while we
     * change settings.
     */

    p_uart->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);

    /* Configure the UART for 8-bit mode, no parity */
    p_uart->CTRL &=
        ~(LPUART_CTRL_M7_MASK | LPUART_CTRL_M_MASK | LPUART_CTRL_R8T9_MASK | LPUART_CTRL_R9T8_MASK);

    /* Calculate baud settings */
    p_uart->BAUD = (p_uart->BAUD & ~(LPUART_BAUD_OSR_MASK)) | LPUART_BAUD_OSR(u16_osr);
    u16_sbr      = (freq / u32_baud) / (u16_osr + 1);
    p_uart->BAUD = (p_uart->BAUD & ~(LPUART_BAUD_SBR_MASK)) | LPUART_BAUD_SBR(u16_sbr);

    /* Enable receiver and transmitter */
    p_uart->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

/**
 * @brief reset the UART,
 * @param[in] p_uart       base of UART port
 * @ return none
 * @ Pass/ Fail criteria: none
 */
void
uart_reset(LPUART_t *p_uart)
{
    /* Set to after reset state an disable UART Tx/Rx */
    p_uart->CTRL = 0x00000000;
    p_uart->BAUD = 0x00000000;

    /* Disable clock to UART */
    PCC->PCCn[PCC_LPUART0_INDEX] &= ~PCC_PCCn_CGC_MASK; /* Enable clock for LPUART1 regs */
    PCC->PCCn[PCC_LPUART1_INDEX] &= ~PCC_PCCn_CGC_MASK; /* Enable clock for LPUART1 regs */
}

/**
 * @brief receive a character.
 * @param[in] p_uart       base of UART port
 * @ return unsigned char
 */
uint8_t
uart_get_char(LPUART_t *p_uart)
{

    /* Wait until character has been received */
    while ((p_uart->STAT & LPUART_STAT_RDRF_MASK) >> LPUART_STAT_RDRF_SHIFT == 0)
        ;
    /* Return the 8-bit data from the receiver */
    return p_uart->DATA;
}
/**
 * @brief send a character.
 * @param[in] p_uart       base of UART port
 * @param[in] u8_char      char to send
 * @ return none
 */
void
uart_put_char(LPUART_t *p_uart, uint8_t u8_char)
{
    /* Wait until space is available in the FIFO */
    while ((p_uart->STAT & LPUART_STAT_TDRE_MASK) >> LPUART_STAT_TDRE_SHIFT == 0)
        ;
    /* Send the character */
    p_uart->DATA = (uint8_t)u8_char;
}

/**
 * @brief send a series of characters using polling mode.
 * @param[in] p_uart      base of UART port
 * @param[in] p_send_buff  pointer of characters to send
 * @param[in] u32_length  number of characters
 * @ return       none
 * @ Pass/ Fail criteria:
 */
void
uart_send_wait(LPUART_t *p_uart, uint8_t *p_send_buff, uint32_t u32_length)
{
    uint8_t  u8_tx_char;
    uint32_t idx;

    for (idx = 0; idx < u32_length; idx++)
    {
        u8_tx_char = p_send_buff[idx];
        while (!uart_is_tx_buff_full(p_uart))
        {
        }
        uart_write_data_reg(p_uart, u8_tx_char);
    }
}

/**
 * @brief receive a series of charecters using polling mode.
 * @param[in] p_uart          base of UART port
 * @param[in] p_receive_buff   pointer of charecters to receive
 * @param[in] u32_length      number of charecters
 * @ return       none
 * @ Pass/ Fail criteria:
 */
void
uart_receive_wait(LPUART_t *p_uart, uint8_t *p_receive_buff, uint32_t u32_length)
{
    uint8_t  u8_rx_char;
    uint32_t idx;

    for (idx = 0; idx < u32_length; idx++)
    {
        while (!uart_is_rx_buff_full(p_uart))
        {
        }
        u8_rx_char          = uart_read_data_reg(p_uart);
        p_receive_buff[idx] = u8_rx_char;
    }
}

/**
 * @brief wait tx complete.
 * @param[in] p_uart      base of UART port
 * @ return       none
 * @ Pass/ Fail criteria: none
 */
void
uart_wait_tx_complete(LPUART_t *p_uart)
{
    while (!uart_is_tx_complete(p_uart))
        ;
}

/*** end of file ***/
