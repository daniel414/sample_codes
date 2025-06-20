/**
 * @file uart_driver.h (simplify)
 * @brief Header file for lpuart driver.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "tw9001.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define MAX_UART_NO 2

/**
 * @brief UART setting type.
 *
 */
typedef struct
{
    uint32_t b_enable : 1; /**< 1: enable, 0: disable */
    uint32_t resvd : 31;   /**< 1: reserved bit field */
} UART_SettingType;

/**
 * @brief UART baudrate type structure.
 *
 */
typedef struct
{
    uint32_t u32_sys_clkhz; /**< system clock */
    uint32_t u32_baudrate;  /**< UART baudrate */
} UART_ConfigBaudrateType;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
void    uart_init(LPUART_t *p_uart, uint32_t u32_baud);
void    uart_reset(LPUART_t *p_uart);
uint8_t uart_get_char(LPUART_t *p_uart);
void    uart_put_char(LPUART_t *p_uart, uint8_t u8_char);
void    uart_send_wait(LPUART_t *p_uart, uint8_t *p_send_buff, uint32_t u32_length);
void    uart_receive_wait(LPUART_t *p_uart, uint8_t *p_receive_buff, uint32_t u32_length);
void    uart_wait_tx_complete(LPUART_t *p_uart);
uint8_t uart_is_rx_buff_full(LPUART_t *p_uart);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UART_DRIVER_H */

/*** end of file ***/
