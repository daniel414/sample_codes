/**
 * @file lpuart_driver.h
 * @brief Header file for the lpuart driver.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef LPUART_DRIVER_H
#define LPUART_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stddef.h>
#include "device_registers.h"
#include "lpuart_access.h"
#include "interrupt_manager.h"
#include "status.h"
#include "osif_driver.h"
#include "edma_driver.h"
#include "callbacks.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
/** @brief Device instance number. */
#define INST_LPUART0 0u

/** @brief Device instance number. */
#define INST_LPUART1 1u

#define LPUART_SHIFT        (16U)
#define LPUART_BAUD_REG_ID  (1U)
#define LPUART_STAT_REG_ID  (2U)
#define LPUART_CTRL_REG_ID  (3U)
#define LPUART_DATA_REG_ID  (4U)
#define LPUART_MATCH_REG_ID (5U)
#define LPUART_MODIR_REG_ID (6U)
#define LPUART_FIFO_REG_ID  (7U)

/**
 * @brief LPUART status flags.
 *
 * This provides constants for the LPUART status flags for use in the UART functions.
 */
typedef enum
{
    LPUART_TX_COMPLETE =
        (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) | (uint32_t)LPUART_STAT_TC_SHIFT),
    LPUART_RX_OVERRUN =
        (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) | (uint32_t)LPUART_STAT_OR_SHIFT),
    LPUART_NOISE_DETECT =
        (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) | (uint32_t)LPUART_STAT_NF_SHIFT),
    LPUART_FRAME_ERR =
        (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) | (uint32_t)LPUART_STAT_FE_SHIFT),
    LPUART_PARITY_ERR =
        (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) | (uint32_t)LPUART_STAT_PF_SHIFT),
    LPUART_TX_DATA_REG_EMPTY = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) |
                                (uint32_t)LPUART_STAT_TDRE_SHIFT),
    LPUART_RX_DATA_REG_FULL  = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) |
                               (uint32_t)LPUART_STAT_RDRF_SHIFT),
    LPUART_IDLE_LINE_DETECT  = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) |
                               (uint32_t)LPUART_STAT_IDLE_SHIFT),
    LPUART_MATCH_ADDR_ONE    = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) |
                             (uint32_t)LPUART_STAT_MA1F_SHIFT),
    LPUART_MATCH_ADDR_TWO    = (((uint32_t)LPUART_STAT_REG_ID << (uint32_t)LPUART_SHIFT) |
                             (uint32_t)LPUART_STAT_MA2F_SHIFT),
    LPUART_FIFO_TX_OF        = (((uint32_t)LPUART_FIFO_REG_ID << (uint32_t)LPUART_SHIFT) |
                         (uint32_t)LPUART_FIFO_TXOF_SHIFT),
    LPUART_FIFO_RX_UF        = (((uint32_t)LPUART_FIFO_REG_ID << (uint32_t)LPUART_SHIFT) |
                         (uint32_t)LPUART_FIFO_RXUF_SHIFT),
    LPUART_ALL_STATUS        = 0xC01FC000u /**< Used for clearing all w1c status flags */
} lpuart_status_flag_t;

/** @brief LPUART interrupt configuration structure, default settings are 0(disabled) */
typedef enum
{
    LPUART_INT_TX_DATA_REG_EMPTY = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) |
                                    (uint32_t)LPUART_CTRL_TIE_SHIFT),
    LPUART_INT_TX_COMPLETE       = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) |
                              (uint32_t)LPUART_CTRL_TCIE_SHIFT),
    LPUART_INT_RX_DATA_REG_FULL  = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) |
                                   (uint32_t)LPUART_CTRL_RIE_SHIFT),
    LPUART_INT_IDLE_LINE         = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) |
                            (uint32_t)LPUART_CTRL_ILIE_SHIFT),
    LPUART_INT_RX_OVERRUN        = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) |
                             (uint32_t)LPUART_CTRL_ORIE_SHIFT),
    LPUART_INT_NOISE_ERR_FLAG    = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) |
                                 (uint32_t)LPUART_CTRL_NEIE_SHIFT),
    LPUART_INT_FRAME_ERR_FLAG    = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) |
                                 (uint32_t)LPUART_CTRL_FEIE_SHIFT),
    LPUART_INT_PARITY_ERR_FLAG   = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) |
                                  (uint32_t)LPUART_CTRL_PEIE_SHIFT),
    LPUART_INT_MATCH_ADDR_ONE    = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) |
                                 (uint32_t)LPUART_CTRL_MA1IE_SHIFT),
    LPUART_INT_MATCH_ADDR_TWO    = (((uint32_t)LPUART_CTRL_REG_ID << (uint32_t)LPUART_SHIFT) |
                                 (uint32_t)LPUART_CTRL_MA2IE_SHIFT),
    LPUART_INT_FIFO_TXOF         = (((uint32_t)LPUART_FIFO_REG_ID << (uint32_t)LPUART_SHIFT) |
                            (uint32_t)LPUART_FIFO_TXOFE_SHIFT),
    LPUART_INT_FIFO_RXUF         = (((uint32_t)LPUART_FIFO_REG_ID << (uint32_t)LPUART_SHIFT) |
                            (uint32_t)LPUART_FIFO_RXUFE_SHIFT)
} lpuart_interrupt_t;

typedef enum
{
    LPUART_USING_DMA = 0,   /**< use DMA to perform UART transfer */
    LPUART_USING_INTERRUPTS /**< use interrupts to perform UART transfer */
} lpuart_transfer_type_t;

/**
 * @brief LPUART number of bits in a character
 *
 * Implements : lpuart_bit_count_per_char_t_Class
 */
typedef enum
{
    LPUART_8_BITS_PER_CHAR  = 0x0U, /**< 8-bit data characters */
    LPUART_9_BITS_PER_CHAR  = 0x1U, /**< 9-bit data characters */
    LPUART_10_BITS_PER_CHAR = 0x2U  /**< 10-bit data characters */
} lpuart_bit_count_per_char_t;

/**
 * @brief LPUART parity mode
 *
 * Implements : lpuart_parity_mode_t_Class
 */
typedef enum
{
    LPUART_PARITY_DISABLED = 0x0U, /**< parity disabled */
    LPUART_PARITY_EVEN     = 0x2U, /**< type even, bit setting: PE|PT = 10 */
    LPUART_PARITY_ODD      = 0x3U  /**< type odd,  bit setting: PE|PT = 11 */
} lpuart_parity_mode_t;

/**
 * @brief Runtime state of the LPUART driver.
 *
 * Note that the caller provides memory for the driver state structures during
 * initialization because the driver does not statically allocate memory.
 *
 * Implements : lpuart_state_t_Class
 */
typedef struct
{
    LPUART_t                   *p_base;
    const IRQn_t               *p_irq_id;
    const clock_names_t        *p_clk_name;
    const uint8_t              *txBuff;           /**< The buffer of data being sent.*/
    uint8_t                    *rxBuff;           /**< The buffer of received data.*/
    volatile uint32_t           tx_size;          /**< The remaining number of bytes. */
    volatile uint32_t           rx_size;          /**< The remaining number of bytes. */
    volatile bool               b_is_tx_busy;     /**< True if there is an active transmit.*/
    volatile bool               b_is_rx_busy;     /**< True if there is an active receive.*/
    volatile bool               b_is_tx_blocking; /**< True if transmit is blocking transaction. */
    volatile bool               b_is_rx_blocking; /**< True if receive is blocking transaction. */
    lpuart_bit_count_per_char_t bit_count_per_char;  /**< number of bits in a char (8/9/10) */
    uart_callback_t             rx_callback;         /**< RX Callback. */
    void                       *p_rx_callback_param; /**< Receive callback parameter pointer.*/
    uart_callback_t             tx_callback;         /**< tx_callback. */
    void                       *p_tx_callback_param; /**< Transmit callback parameter pointer.*/
    lpuart_transfer_type_t      transfer_type; /**< Type of LPUART transfer (interrupt/dma based) */
    uint8_t                     rx_dma_channel;  /**< DMA channel number for DMA-based rx. */
    uint8_t                     tx_dma_channel;  /**< DMA channel number for DMA-based tx. */
    uint8_t                     rx_complete;     /**< Sync for blocking Rx timeout condition */
    uint8_t                     tx_complete;     /**< Sync for blocking Tx timeout condition */
    volatile status_t           transmit_status; /**< Status of transmit operation */
    volatile status_t           receive_status;  /**< Status of receive operation */
} lpuart_state_t;

/**
 * @brief LPUART configuration structure
 *
 * Implements : lpuart_user_config_t_Class
 */
typedef struct
{
    uint32_t                    baud_rate;          /**< LPUART baud rate */
    lpuart_parity_mode_t        parity_mode;        /**< parity: disabled , even, odd */
    lpuart_stop_bit_count_t     stop_bit_count;     /**< stop bits: 1, 2 */
    lpuart_bit_count_per_char_t bit_count_per_char; /**< number: 8, 9 or 10 */
    lpuart_transfer_type_t      transfer_type;      /**< transfer: interrupt/dma */
    uint8_t                     rx_dma_channel;     /**< Channel number for DMA rx channel. */
    uint8_t                     tx_dma_channel;     /**< Channel number for DMA tx channel. */
} lpuart_user_config_t;

/*******************************************************************************
 * Exported variables
 ******************************************************************************/
/** @brief Table of base pointers for LPUART instances. */
extern LPUART_t *const gp_lpuart_base[LPUART_INSTANCE_COUNT];

/** @brief Table to save LPUART IRQ enumeration numbers defined in the CMSIS
 * header file. */
extern const IRQn_t g_lpuart_rx_tx_irq_id[LPUART_INSTANCE_COUNT];

/** @brief Table to save LPUART clock names as defined in clock manager. */
extern const clock_names_t g_lpuart_clk_names[LPUART_INSTANCE_COUNT];

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Initializes the LPUART controller.
 *
 * This function Initializes the LPUART controller to known state.
 *
 * @param[in] p_base LPUART base pointer.
 */
void lpuart_init_set(LPUART_t *p_base);

/**
 * @brief Configures the number of bits per character in the LPUART controller.
 *
 * This function configures the number of bits per character in the LPUART
 * controller. In some LPUART instances, the user should disable the
 * transmitter/receiver before calling this function. Generally, this may be
 * applied to all LPUARTs to ensure safe operation.
 *
 * @param[in] p_base LPUART base pointer.
 * @param[in] bit_count_per_char  Number of bits per char (8, 9, or 10, depending on the LPUART
 * instance)
 * @param[in] b_parity  Specifies whether parity bit is enabled
 */
void lpuart_set_bit_count_per_char(LPUART_t                   *p_base,
                                   lpuart_bit_count_per_char_t bit_count_per_char,
                                   bool                        b_parity);

/**
 * @brief Configures parity mode in the LPUART controller.
 *
 * This function configures parity mode in the LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param[in] p_base LPUART base pointer.
 * @param[in] parity_mode_type  Parity mode (enabled, disable, odd, even - see parity_mode_t struct)
 */
void lpuart_set_parity_mode(LPUART_t *p_base, lpuart_parity_mode_t parity_mode_type);

/**
 * @brief Configures the LPUART module interrupts.
 *
 * This function configures the LPUART module interrupts to enable/disable
 * various interrupt sources.
 *
 * @param[in]   p_base LPUART module base pointer.
 * @param[in]   int_src LPUART interrupt configuration data.
 * @param[in]   b_enable   true: enable, false: disable.
 */
void lpuart_set_int_mode(LPUART_t *p_base, lpuart_interrupt_t int_src, bool b_enable);

/**
 * @brief Returns LPUART module interrupts state.
 *
 * This function returns whether a certain LPUART module interrupt is enabled or disabled.
 *
 * @param[in]   p_base LPUART module base pointer.
 * @param[in]   int_src LPUART interrupt configuration data.
 * @return  true: enable, false: disable.
 */
bool lpuart_get_int_mode(const LPUART_t *p_base, lpuart_interrupt_t int_src);

/**
 * @brief Sends the LPUART 9-bit character.
 *
 * This functions sends a 9-bit character.
 *
 * @param[in] p_base LPUART Instance
 * @param[in] data data to send (9-bit)
 */
void lpuart_put_char9(LPUART_t *p_base, uint16_t data);

/**
 * @brief Sends the LPUART 10-bit character (Note: Feature available on select LPUART instances).
 *
 * This functions sends a 10-bit character.
 *
 * @param[in] p_base LPUART Instance
 * @param[in] data data to send (10-bit)
 */
void lpuart_put_char10(LPUART_t *p_base, uint16_t data);

/**
 * @brief Gets the LPUART 9-bit character.
 *
 * This functions receives a 9-bit character.
 *
 * @param[in] p_base LPUART base pointer
 * @param[out] p_read_data Data read from receive (9-bit)
 */
void lpuart_get_char9(const LPUART_t *p_base, uint16_t *p_read_data);

/**
 * @brief Gets the LPUART 10-bit character.
 *
 * This functions receives a 10-bit character.
 *
 * @param[in] p_base LPUART base pointer
 * @param[out] p_read_data Data read from receive (10-bit)
 */
void lpuart_get_char10(const LPUART_t *p_base, uint16_t *p_read_data);

/**
 * @brief Gets the LUART status bitmap state.
 *
 * This function returns the state of bitmap of the LPUART status at the moment of intrrupt event.
 *
 * @param[in] p_base Module base pointer of type LPUART_t.
 * @return State of bitmap of the status.
 */
uint32_t lpuart_get_status_bm(LPUART_t *p_base);

/**
 * @brief Clears the LPUART status bitmap state.
 *
 * @param[in] p_base Module base pointer of type LPUART_t.
 * @param[in] status_bm bitmap for w1c interrupt event.
 */
void lpuart_clear_status_bm(LPUART_t *p_base, uint32_t status_bm);

/**
 * @brief  Gets the PUART status flag
 *
 * This function returns the state of a status flag.
 *
 * @param[in] p_base LPUART base pointer
 * @param[in] status_flag  The status flag to query
 * @return Whether the current status flag is set(true) or not(false).
 */
bool lpuart_get_status_flag(const LPUART_t *p_base, lpuart_status_flag_t status_flag);

/**
 * @brief LPUART clears an individual status flag.
 *
 * This function clears an individual status flag (see lpuart_status_flag_t for
 * list of status bits).
 *
 * @param[in] p_base LPUART base pointer
 * @param[in] status_flag  Desired LPUART status flag to clear
 * @return STATUS_SUCCESS if successful or STATUS_ERROR if an error occured
 */
status_t lpuart_clear_status_flag(LPUART_t *p_base, lpuart_status_flag_t status_flag);

/**
 * @brief Enable or disable the LPUART error interrupts.
 *
 * This function configures the error interrupts (parity, noise, overrun, framing).
 *
 * @param[in] p_base LPUART base pointer
 * @param[in] b_enable true - enable, false - disable error interrupts
 */
void lpuart_set_error_interrupts(LPUART_t *p_base, bool b_enable);

/**
 * @brief Initializes the LPUART configuration structure with default values.
 *
 * This function initializes a configuration structure received from the
 * application with default values.
 *
 * @param[out] p_lpuart_user_config user configuration structure of type
 */
void lpuart_get_default_config(lpuart_user_config_t *p_lpuart_user_config);

/**
 * @brief Initializes an LPUART operation instance.
 *
 * The caller provides memory for the driver state structures during initialization.
 * The user must select the LPUART clock source in the application to initialize the LPUART.
 *
 * @param[in] instance  LPUART instance number
 * @param[in] p_lpuart_user_config user configuration structure of type
 * @param[in] p_lpuart_state_ptr pointer to the LPUART driver state structure
 * @return STATUS_SUCCESS if successful;
 *         STATUS_ERROR if an error occurred
 */
status_t lpuart_init(uint32_t                    instance,
                     lpuart_state_t             *p_lpuart_state_ptr,
                     const lpuart_user_config_t *p_lpuart_user_config);

/**
 * @brief Shuts down the LPUART by disabling interrupts and transmitter/receiver.
 *
 * @param[in] instance  LPUART instance number
 * @return STATUS_SUCCESS if successful;
 *         STATUS_ERROR if an error occurred
 */
status_t lpuart_deinit(uint32_t instance);

/**
 * @brief Installs callback function for the LPUART receive.
 *
 * @note After a callback is installed, it bypasses part of the LPUART IRQHandler logic.
 * Therefore, the callback needs to handle the indexes of txBuff and tx_size.
 *
 * @param[in] instance The LPUART instance number.
 * @param[in] function The LPUART receive callback function.
 * @param[in] p_callback_param The LPUART receive callback parameter pointer.
 * @return Former LPUART receive callback function pointer.
 */
uart_callback_t lpuart_install_rx_callback(uint32_t        instance,
                                           uart_callback_t function,
                                           void           *p_callback_param);

/**
 * @brief Installs callback function for the LPUART transmit.
 *
 * @note After a callback is installed, it bypasses part of the LPUART IRQHandler logic.
 * Therefore, the callback needs to handle the indexes of rxBuff and rx_size.
 *
 * @param[in] instance The LPUART instance number.
 * @param[in] function The LPUART transmit callback function.
 * @param[in] p_callback_param The LPUART transmit callback parameter pointer.
 * @return Former LPUART transmit callback function pointer.
 */
uart_callback_t lpuart_install_tx_callback(uint32_t        instance,
                                           uart_callback_t function,
                                           void           *p_callback_param);

/**
 * @brief Sends data out through the LPUART module using a blocking method.
 *
 * Blocking means that the function does not return until the transmission is complete.
 *
 * @param[in] instance  LPUART instance number
 * @param[in] p_tx_buff  source buffer containing 8-bit data chars to send
 * @param[in] tx_size the number of bytes to send
 * @param[in] timeout timeout value in milliseconds
 * @return STATUS_SUCCESS if successful;
 *         STATUS_TIMEOUT if the timeout was reached;
 *         STATUS_BUSY if the resource is busy;
 *         STATUS_ERROR if an error occurred
 */
status_t lpuart_send_data_blocking(uint32_t       instance,
                                   const uint8_t *p_tx_buff,
                                   uint32_t       tx_size,
                                   uint32_t       timeout);

/**
 * @brief Send out multiple bytes of data using polling method.
 *
 * @param[in]   instance  LPUART instance number.
 * @param[in]   p_tx_buff The buffer pointer which saves the data to be sent.
 * @param[in]   tx_size Size of data to be sent in unit of byte.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_BUSY if the resource is busy;
 */
status_t lpuart_send_data_polling(uint32_t instance, const uint8_t *p_tx_buff, uint32_t tx_size);

/**
 * @brief Sends data out through the LPUART module using a non-blocking method.
 * This enables an a-sync method for transmitting data. When used with a non-blocking receive,
 * the LPUART can perform a full duplex operation. Non-blocking  means that
 * the function returns immediately. The application has to get the transmit
 * status to know when the transmit is complete.
 *
 * @param[in] instance  LPUART instance number
 * @param[in] p_tx_buff  source buffer containing 8-bit data chars to send
 * @param[in] tx_size  the number of bytes to send
 * @return STATUS_SUCCESS if successful;
 *         STATUS_BUSY if the resource is busy;
 */
status_t lpuart_send_data(uint32_t instance, const uint8_t *p_tx_buff, uint32_t tx_size);

/**
 * @brief Returns whether the previous transmit is complete.
 *
 * @param[in] instance  LPUART instance number
 * @param[in] p_bytes_remaining Pointer to value that is populated with the number
 *         of bytes that have been sent in the active transfer
 *         @note In DMA mode, this parameter may not be accurate, in case the
 * transfer completes right after calling this function; in this edge-case,
 * the parameter will reflect the initial transfer size, due to automatic
 * reloading of the major loop count in the DMA transfer descriptor.
 * @return The transmit status.
 * @retval STATUS_SUCCESS The transmit has completed successfully.
 * @retval STATUS_BUSY The transmit is still in progress. @a bytesRemaining
 *         will be filled with the number of bytes that are yet to be transmitted.
 * @retval STATUS_UART_ABORTED The transmit was aborted.
 * @retval STATUS_TIMEOUT A timeout was reached.
 * @retval STATUS_ERROR An error occurred.
 */
status_t lpuart_get_transmit_status(uint32_t instance, uint32_t *p_bytes_remaining);

/**
 * @brief Terminates a non-blocking transmission early.
 *
 * @param[in] instance  LPUART instance number
 * @return Whether the aborting is successful or not.
 */
status_t lpuart_abort_sending_data(uint32_t instance);

/**
 * @brief Gets data from the LPUART module by using a blocking method.
 *  Blocking means that the function does not return until the receive is complete.
 *
 * @param[in] instance  LPUART instance number
 * @param[out] p_rx_buff  buffer containing 8-bit read data chars received
 * @param[in] rx_size the number of bytes to receive
 * @param[in] timeout timeout value in milliseconds
 * @return STATUS_SUCCESS if successful;
 *         STATUS_TIMEOUT if the timeout was reached;
 *         STATUS_BUSY if the resource is busy;
 *         STATUS_UART_FRAMING_ERROR if a framing error occurred;
 *         STATUS_UART_NOISE_ERROR if a noise error occurred;
 *         STATUS_UART_PARITY_ERROR if a parity error occurred;
 *         STATUS_UART_RX_OVERRUN if an overrun error occurred;
 *         STATUS_ERROR if a DMA error occurred;
 */
status_t lpuart_receive_data_blocking(uint32_t instance,
                                      uint8_t *p_rx_buff,
                                      uint32_t rx_size,
                                      uint32_t timeout);

/**
 * @brief Receive multiple bytes of data using polling method.
 *
 * @param[in]   instance  LPUART instance number.
 * @param[out]   p_rx_buff The buffer pointer which saves the data to be received.
 * @param[in]   rx_size Size of data need to be received in unit of byte.
 * @return  STATUS_SUCCESS if the transaction is successful;
 *          STATUS_BUSY if the resource is busy;
 *          STATUS_UART_RX_OVERRUN if an overrun error occurred.
 */
status_t lpuart_receive_data_polling(uint32_t instance, uint8_t *p_rx_buff, uint32_t rx_size);

/**
 * @brief Gets data from the LPUART module by using a non-blocking method.
 *  This enables an a-sync method for receiving data. When used with
 *  a non-blocking transmission, the LPUART can perform a full duplex
 * operation. Non-blocking means that the function returns immediately. The
 * application has to get the receive status to know when the receive is
 * complete.
 *
 * @param[in] instance  LPUART instance number
 * @param[out] p_rx_buff  buffer containing 8-bit read data chars received
 * @param[in] rx_size  the number of bytes to receive
 * @return STATUS_SUCCESS if successful;
 *         STATUS_BUSY if the resource is busy
 */
status_t lpuart_receive_data(uint32_t instance, uint8_t *p_rx_buff, uint32_t rx_size);

/**
 * @brief Returns whether the previous receive is complete.
 *
 * @param[in] instance  LPUART instance number
 * @param[out] p_bytes_remaining pointer to value that is filled  with the number of
 *         bytes that still need to be received in the active transfer.
 *        @note In DMA mode, this parameter may not be accurate, in case the
 *               transfer completes right after calling this function; in
 *               this edge-case, the parameter will reflect the initial
 *               transfer size, due to automatic reloading of the major loop
 *               count in the DMA transfer descriptor.
 * @return The receive status.
 * @retval STATUS_SUCCESS the receive has completed successfully.
 * @retval STATUS_BUSY the receive is still in progress. @a bytesReceived
 *         will be filled with the number of bytes that have been received
 *         so far.
 * @retval STATUS_UART_ABORTED The receive was aborted.
 * @retval STATUS_TIMEOUT A timeout was reached.
 * @retval STATUS_UART_RX_OVERRUN, STATUS_UART_FRAMING_ERROR,
 *         STATUS_UART_PARITY_ERROR or STATUS_UART_NOISE_ERROR,
 *         STATUS_ERROR An error occurred during reception.
 */
status_t lpuart_get_receive_status(uint32_t instance, uint32_t *p_bytes_remaining);

/**
 * @brief Terminates a non-blocking receive early.
 *
 * @param[in] instance  LPUART instance number
 *
 * @return Whether the receiving was successful or not.
 */
status_t lpuart_abort_receiving_data(uint32_t instance);

/**
 * @brief Configures the LPUART baud rate.
 *
 * This function configures the LPUART baud rate.
 * In some LPUART instances the user must disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param[in] instance  LPUART instance number.
 * @param[in] desired_baud_rate LPUART desired baud rate.
 * @return STATUS_BUSY if called during an on-going transfer, STATUS_SUCCESS otherwise
 */
status_t lpuart_set_baud_rate(uint32_t instance, uint32_t desired_baud_rate);

/**
 * @brief Returns the LPUART baud rate.
 *
 * This function returns the LPUART configured baud rate.
 *
 * @param[in] instance  LPUART instance number.
 * @param[out] p_configured_baud_rate LPUART configured baud rate.
 */
void lpuart_get_baud_rate(uint32_t instance, uint32_t *p_configured_baud_rate);

/**
 * @brief Sets the internal driver reference to the tx buffer.
 *
 * This function can be called from the tx callback to provide the driver
 * with a new buffer, for continuous transmission.
 *
 * @param[in] instance  LPUART instance number
 * @param[in] p_tx_buff  source buffer containing 8-bit data chars to send
 * @param[in] tx_size  the number of bytes to send
 * @return STATUS_SUCCESS
 */
status_t lpaurt_set_tx_buffer(uint32_t instance, const uint8_t *p_tx_buff, uint32_t tx_size);

/**
 * @brief Sets the internal driver reference to the rx buffer.
 *
 * This function can be called from the rx callback to provide the driver
 * with a new buffer, for continuous reception.
 *
 * @param[in] instance  LPUART instance number
 * @param[in] p_rx_buff  destination buffer containing 8-bit data chars to receive
 * @param[in] rx_size  the number of bytes to receive
 * @return STATUS_SUCCESS
 */
status_t lpuart_set_rx_buffer(uint32_t instance, uint8_t *p_rx_buff, uint32_t rx_size);

#ifdef __cplusplus
}
#endif

#endif /* LPUART_DRIVER_H */

/*** end of file ***/
