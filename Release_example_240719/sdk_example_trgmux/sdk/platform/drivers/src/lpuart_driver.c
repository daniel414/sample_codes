/**
 * @file lpuart_driver.c
 * @brief This file provides access to the Lpuart module.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stddef.h>
#include <stdbool.h>
#include "lpuart_driver.h"
#include "clock_tw9001.h"

/*******************************************************************************
 * Global variables
 ******************************************************************************/
/** @brief Table of base pointers for LPUART instances. */
LPUART_t *const gp_lpuart_base[LPUART_INSTANCE_COUNT] = LPUART_BASE_PTRS;

/** @brief Table to save LPUART IRQ enumeration numbers defined in the CMSIS
 * header file. */
const IRQn_t g_lpuart_rx_tx_irq_id[LPUART_INSTANCE_COUNT] = LPUART_RX_TX_IRQS;

/** @brief Table to save LPUART clock names as defined in clock manager. */
const clock_names_t g_lpuart_clk_names[LPUART_INSTANCE_COUNT] = LPUART_CLOCK_NAMES;

/*******************************************************************************
 * Static variables
 ******************************************************************************/
/* Pointer to lpuart runtime state structure */
static lpuart_state_t *sp_lpuart_state_ptr[LPUART_INSTANCE_COUNT] = {NULL};

/* Table of base addresses for lpuart instances. */
static LPUART_t *const sp_lpuart_base[LPUART_INSTANCE_COUNT] = LPUART_BASE_PTRS;

/* Table to save LPUART enum numbers defined in CMSIS files. */
static const IRQn_t s_lpuart_irq_id[LPUART_INSTANCE_COUNT] = LPUART_RX_TX_IRQS;

/* Table to save LPUART clock names as defined in clock manager. */
static const clock_names_t s_lpuart_clk_names[LPUART_INSTANCE_COUNT] = LPUART_CLOCK_NAMES;

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/
static status_t lpuart_start_send_data_using_int(uint32_t       instance,
                                                 const uint8_t *p_tx_buff,
                                                 uint32_t       tx_size);
static status_t lpuart_start_receive_data_using_int(uint32_t instance,
                                                    uint8_t *p_rx_buff,
                                                    uint32_t rx_size);
static status_t lpaurt_start_send_data_using_dma(uint32_t       instance,
                                                 const uint8_t *p_tx_buff,
                                                 uint32_t       tx_size);
static status_t lpuart_start_receive_data_using_dma(uint32_t instance,
                                                    uint8_t *p_rx_buff,
                                                    uint32_t rx_size);
static void     lpuart_complete_send_data_using_int(uint32_t instance);
static void     lpuart_complete_receive_data_using_int(uint32_t instance);
static void     lpuart_stop_tx_dma(uint32_t instance);
static void     lpuart_stop_rx_dma(uint32_t instance);
static void     lpuart_tx_dma_callback(void *p_parameter, edma_chn_status_t status);
static void     lpuart_rx_dma_callback(void *p_parameter, edma_chn_status_t status);
static void     lpuart_put_data(uint32_t instance);
static void     lpuart_get_data(uint32_t instance);
static void     lpuart_rx_irq_handler(uint32_t instance);
static void     lpuart_tx_empty_irq_handler(uint32_t instance);
static void     lpaurt_tx_complete_irq_handler(uint32_t instance);
static void     lpuart_err_irq_handler(uint32_t instance);

/*******************************************************************************
 * Global functions
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_init_set
 * Description   : Initializes the LPUART controller to known state, using
 *                 register reset values defined in the reference manual.
 *END**************************************************************************/
void
lpuart_init_set(LPUART_t *p_base)
{
    /* Set the default oversampling ratio (16) and baud-rate divider (4) */
    p_base->BAUD = ((uint32_t)((FEATURE_LPUART_DEFAULT_OSR << LPUART_BAUD_OSR_SHIFT) |
                               (FEATURE_LPUART_DEFAULT_SBR << LPUART_BAUD_SBR_SHIFT)));
    /* Clear the error/interrupt flags */
    p_base->STAT = FEATURE_LPUART_STAT_REG_FLAGS_MASK;
    /* Reset all features/interrupts by default */
    p_base->CTRL = 0x00000000;
    /* Reset match addresses */
    p_base->MATCH = 0x00000000;
    /* Reset FIFO feature */
    p_base->FIFO = FEATURE_LPUART_FIFO_RESET_MASK;
    /* Reset FIFO Watermark values */
    p_base->WATER = 0x00000000;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_set_bit_count_per_char
 * Description   : Configures the number of bits per char in LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *END**************************************************************************/
void
lpuart_set_bit_count_per_char(LPUART_t                   *p_base,
                              lpuart_bit_count_per_char_t bit_count_per_char,
                              bool                        b_parity)
{
    uint32_t tmp_bit_count_per_char = (uint32_t)bit_count_per_char;
    if (b_parity)
    {
        tmp_bit_count_per_char += 1U;
    }

    if (tmp_bit_count_per_char == (uint32_t)LPUART_10_BITS_PER_CHAR)
    {
        p_base->BAUD =
            (p_base->BAUD & ~LPUART_BAUD_M10_MASK) | ((uint32_t)1U << LPUART_BAUD_M10_SHIFT);
    }
    else
    {
        /* config 8-bit (M=0) or 9-bits (M=1) */
        p_base->CTRL =
            (p_base->CTRL & ~LPUART_CTRL_M_MASK) | (tmp_bit_count_per_char << LPUART_CTRL_M_SHIFT);
        /* clear M10 to make sure not 10-bit mode */
        p_base->BAUD &= ~LPUART_BAUD_M10_MASK;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_set_parity_mode
 * Description   : Configures parity mode in the LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *END**************************************************************************/
void
lpuart_set_parity_mode(LPUART_t *p_base, lpuart_parity_mode_t parity_mode_type)
{
    p_base->CTRL = (p_base->CTRL & ~LPUART_CTRL_PE_MASK) |
                   (((uint32_t)parity_mode_type >> 1U) << LPUART_CTRL_PE_SHIFT);
    p_base->CTRL = (p_base->CTRL & ~LPUART_CTRL_PT_MASK) |
                   (((uint32_t)parity_mode_type & 1U) << LPUART_CTRL_PT_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_set_int_mode
 * Description   : Configures the LPUART module interrupts to enable/disable
 * various interrupt sources.
 *END**************************************************************************/
void
lpuart_set_int_mode(LPUART_t *p_base, lpuart_interrupt_t int_src, bool b_enable)
{
    uint32_t reg            = (uint32_t)(int_src) >> LPUART_SHIFT;
    uint32_t int_reg_offset = (uint16_t)(int_src);

    switch (reg)
    {
        case LPUART_BAUD_REG_ID:
        {
            p_base->BAUD = (p_base->BAUD & ~(1UL << int_reg_offset)) |
                           ((b_enable ? 1U : 0U) << int_reg_offset);
        }
        break;
        case LPUART_CTRL_REG_ID:
        {
            p_base->CTRL = (p_base->CTRL & ~(1UL << int_reg_offset)) |
                           ((b_enable ? 1U : 0U) << int_reg_offset);
        }
        break;
        case LPUART_FIFO_REG_ID:
        {
            p_base->FIFO =
                (p_base->FIFO & (~FEATURE_LPUART_FIFO_REG_FLAGS_MASK & ~(1UL << int_reg_offset))) |
                ((b_enable ? 1U : 0U) << int_reg_offset);
        }
        break;
        default:
        { /* Invalid parameter: return */
        }
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_get_int_mode
 * Description   : Returns whether LPUART module interrupt is enabled/disabled.
 *END**************************************************************************/
bool
lpuart_get_int_mode(const LPUART_t *p_base, lpuart_interrupt_t int_src)
{
    uint32_t reg       = (uint32_t)(int_src) >> LPUART_SHIFT;
    bool     b_ret_val = false;

    switch (reg)
    {
        case LPUART_BAUD_REG_ID:
        {
            b_ret_val = (((p_base->BAUD >> (uint16_t)(int_src)) & 1U) > 0U);
        }
        break;
        case LPUART_CTRL_REG_ID:
        {
            b_ret_val = (((p_base->CTRL >> (uint16_t)(int_src)) & 1U) > 0U);
        }
        break;
        case LPUART_FIFO_REG_ID:
        {
            b_ret_val = (((p_base->FIFO >> (uint16_t)(int_src)) & 1U) > 0U);
        }
        break;
        default:
        { /* Invalid parameter: return */
        }
        break;
    }
    return b_ret_val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_put_char9
 * Description   : Sends the LPUART 9-bit character.
 *END**************************************************************************/
void
lpuart_put_char9(LPUART_t *p_base, uint16_t data)
{
    uint8_t           ninth_data_bit;
    volatile uint8_t *p_data_reg_bytes = (volatile uint8_t *)(&(p_base->DATA));

    ninth_data_bit = (uint8_t)((data >> 8U) & 0x1U);

    /* write to ninth data bit T8(where T[0:7]=8-bits, T8=9th bit) */
    p_base->CTRL = (p_base->CTRL & ~LPUART_CTRL_R9T8_MASK) |
                   ((uint32_t)(ninth_data_bit) << LPUART_CTRL_R9T8_SHIFT);

    /* write 8-bits to the data register*/
    p_data_reg_bytes[0] = (uint8_t)data;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_put_char10
 * Description   : Sends the LPUART 10-bit character.
 *END**************************************************************************/
void
lpuart_put_char10(LPUART_t *p_base, uint16_t data)
{
    uint8_t           ninth_data_bit, tenth_data_bit;
    uint32_t          reg_val;
    volatile uint8_t *p_data_reg_bytes = (volatile uint8_t *)(&(p_base->DATA));

    ninth_data_bit = (uint8_t)((data >> 8U) & 0x1U);
    tenth_data_bit = (uint8_t)((data >> 9U) & 0x1U);

    /* write to ninth/tenth data bit (T[0:7]=8-bits, T8=9th bit, T9=10th bit) */
    reg_val = p_base->CTRL;
    reg_val =
        (reg_val & ~LPUART_CTRL_R9T8_MASK) | ((uint32_t)ninth_data_bit << LPUART_CTRL_R9T8_SHIFT);
    reg_val =
        (reg_val & ~LPUART_CTRL_R8T9_MASK) | ((uint32_t)tenth_data_bit << LPUART_CTRL_R8T9_SHIFT);
    p_base->CTRL = reg_val;

    /* write to 8-bits to the data register */
    p_data_reg_bytes[0] = (uint8_t)data;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_get_char9
 * Description   : Gets the LPUART 9-bit character.
 *END**************************************************************************/
void
lpuart_get_char9(const LPUART_t *p_base, uint16_t *p_read_data)
{
    DEV_ASSERT(p_read_data != NULL);

    /* get ninth bit from lpuart data register */
    *p_read_data = (uint16_t)(((p_base->CTRL >> LPUART_CTRL_R8T9_SHIFT) & 1U) << 8);

    /* get 8-bit data from the lpuart data register */
    *p_read_data |= (uint8_t)p_base->DATA;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_get_char10
 * Description   : Gets the LPUART 10-bit character
 *END**************************************************************************/
void
lpuart_get_char10(const LPUART_t *p_base, uint16_t *p_read_data)
{
    DEV_ASSERT(p_read_data != NULL);

    /* read tenth data bit */
    *p_read_data = (uint16_t)(((p_base->CTRL >> LPUART_CTRL_R9T8_SHIFT) & 1U) << 9);
    /* read ninth data bit */
    *p_read_data |= (uint16_t)(((p_base->CTRL >> LPUART_CTRL_R8T9_SHIFT) & 1U) << 8);

    /* get 8-bit data */
    *p_read_data |= (uint8_t)p_base->DATA;
}

/**
 * @brief Gets the LPUART status bitmap state.
 *
 * This function returns the state of bitmap of the LPUART status at the moment of intrrupt event.
 *
 * @param[in] p_base Module base pointer of type LPUART_t.
 * @return State of bitmap of the status.
 */
uint32_t
lpuart_get_status_bm(LPUART_t *p_base)
{
    return p_base->STAT;
}

/**
 * @brief Clears the LPUART status bitmap state.
 *
 * @param[in] p_base Module base pointer of type LPUART_t.
 * @param[in] status_bm bitmap for w1c interrupt event.
 */
void
lpuart_clear_status_bm(LPUART_t *p_base, uint32_t status_bm)
{
    p_base->STAT = status_bm;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_get_status_flag
 * Description   : LPUART get status flag by passing flag enum.
 *END**************************************************************************/
bool
lpuart_get_status_flag(const LPUART_t *p_base, lpuart_status_flag_t status_flag)
{
    uint32_t reg     = (uint32_t)(status_flag) >> LPUART_SHIFT;
    bool     ret_val = false;

    switch (reg)
    {
        case LPUART_STAT_REG_ID:
        {
            ret_val = (((p_base->STAT >> (uint16_t)(status_flag)) & 1U) > 0U);
        }
        break;
        case LPUART_DATA_REG_ID:
        {
            ret_val = (((p_base->DATA >> (uint16_t)(status_flag)) & 1U) > 0U);
        }
        break;
        case LPUART_FIFO_REG_ID:
        {
            ret_val = (((p_base->FIFO >> (uint16_t)(status_flag)) & 1U) > 0U);
        }
        break;
        default:
        { /* Invalid parameter: return */
        }
        break;
    }

    return ret_val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_clear_status_flag
 * Description   : LPUART clears an individual status flag
 * (see lpuart_status_flag_t for list of status bits).
 *END**************************************************************************/
status_t
lpuart_clear_status_flag(LPUART_t *p_base, lpuart_status_flag_t status_flag)
{
    status_t return_code = STATUS_SUCCESS;

    switch (status_flag)
    {
        /* These flags are cleared automatically by other lpuart operations
         * and cannot be manually cleared, return error code */
        case LPUART_TX_DATA_REG_EMPTY:
        case LPUART_TX_COMPLETE:
        case LPUART_RX_DATA_REG_FULL:
        {
            return_code = STATUS_ERROR;
        }
        break;
        case LPUART_IDLE_LINE_DETECT:
        {
            p_base->STAT =
                (p_base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_IDLE_MASK;
        }
        break;
        case LPUART_RX_OVERRUN:
        {
            p_base->STAT =
                (p_base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_OR_MASK;
        }
        break;
        case LPUART_NOISE_DETECT:
        {
            p_base->STAT =
                (p_base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_NF_MASK;
        }
        break;
        case LPUART_FRAME_ERR:
        {
            p_base->STAT =
                (p_base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_FE_MASK;
        }
        break;
        case LPUART_PARITY_ERR:
        {
            p_base->STAT =
                (p_base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_PF_MASK;
        }
        break;
        case LPUART_MATCH_ADDR_ONE:
        {
            p_base->STAT =
                (p_base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_MA1F_MASK;
        }
        break;
        case LPUART_MATCH_ADDR_TWO:
        {
            p_base->STAT =
                (p_base->STAT & (~FEATURE_LPUART_STAT_REG_FLAGS_MASK)) | LPUART_STAT_MA2F_MASK;
        }
        break;
        case LPUART_FIFO_TX_OF:
        {
            p_base->FIFO =
                (p_base->FIFO & (~FEATURE_LPUART_FIFO_REG_FLAGS_MASK)) | LPUART_FIFO_TXOF_MASK;
        }
        break;
        case LPUART_FIFO_RX_UF:
        {
            p_base->FIFO =
                (p_base->FIFO & (~FEATURE_LPUART_FIFO_REG_FLAGS_MASK)) | LPUART_FIFO_RXUF_MASK;
        }
        break;
        default:
        {
            return_code = STATUS_ERROR;
        }
        break;
    }

    return (return_code);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_set_error_interrupts
 * Description   : Enable or disable the LPUART error interrupts.
 *END**************************************************************************/
void
lpuart_set_error_interrupts(LPUART_t *p_base, bool b_enable)
{
    /* Configure the error interrupts */
    lpuart_set_int_mode(p_base, LPUART_INT_RX_OVERRUN, b_enable);
    lpuart_set_int_mode(p_base, LPUART_INT_PARITY_ERR_FLAG, b_enable);
    lpuart_set_int_mode(p_base, LPUART_INT_NOISE_ERR_FLAG, b_enable);
    lpuart_set_int_mode(p_base, LPUART_INT_FRAME_ERR_FLAG, b_enable);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_get_default_config
 * Description   : Initializes the LPUART configuration structure with
 *                 default values.
 *
 * Implements    : lpuart_get_default_config_activity
 *END**************************************************************************/
void
lpuart_get_default_config(lpuart_user_config_t *p_lpuart_user_config)
{
    DEV_ASSERT(p_lpuart_user_config != NULL);

    p_lpuart_user_config->transfer_type      = LPUART_USING_INTERRUPTS;
    p_lpuart_user_config->baud_rate          = 115200U;
    p_lpuart_user_config->parity_mode        = LPUART_PARITY_DISABLED;
    p_lpuart_user_config->stop_bit_count     = LPUART_ONE_STOP_BIT;
    p_lpuart_user_config->bit_count_per_char = LPUART_8_BITS_PER_CHAR;
    p_lpuart_user_config->rx_dma_channel     = 0U;
    p_lpuart_user_config->tx_dma_channel     = 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_init
 * Description   : This function initializes a LPUART instance for operation.
 * This function will initialize the run-time state structure to keep track of
 * the on-going transfers, ungate the clock to the LPUART module, initialize the
 * module to user defined settings and default settings, configure the IRQ state
 * structure and enable the module-level interrupt to the core, and enable the
 * LPUART module transmitter and receiver.
 * The following is an example of how to set up the lpuart_state_t and the
 * lpuart_user_config_t parameters and how to call the lpuart_init function
 * by passing in these parameters:
 *    lpuart_user_config_t lpuartConfig;
 *    lpuartConfig.baud_rate = 115200;
 *    lpuartConfig.bit_count_per_char = LPUART_8_BITS_PER_CHAR;
 *    lpuartConfig.parity_mode = LPUART_PARITY_DISABLED;
 *    lpuartConfig.stop_bit_count = LPUART_ONE_STOP_BIT;
 *    lpuartConfig.transfer_type = LPUART_USING_INTERRUPTS;
 *    lpuart_state_t lpuartState;
 *    lpuart_init(instance, &lpuartState, &lpuartConfig);
 *
 * Implements    : lpuart_init_activity
 *END**************************************************************************/
status_t
lpuart_init(uint32_t                    instance,
            lpuart_state_t             *p_lpuart_state_ptr,
            const lpuart_user_config_t *p_lpuart_user_config)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_lpuart_state_ptr != NULL);
    DEV_ASSERT(p_lpuart_user_config != NULL);

    status_t      os_status_rx_sem;
    status_t      os_status_tx_sem;
    LPUART_t     *p_base = sp_lpuart_base[instance];
    uint32_t      idx;
    uint32_t      lpuart_source_clock;
    clock_names_t instance_clk_name = s_lpuart_clk_names[instance];

    /* Get the LPUART clock as configured in the clock manager */
    (void)clock_get_freq(instance_clk_name, &lpuart_source_clock);

    /* Check if current instance is clock gated off. */
    DEV_ASSERT(lpuart_source_clock > 0U);

    /* Check if current instance is already initialized. */
    DEV_ASSERT(sp_lpuart_state_ptr[instance] == NULL);

    /* In DMA mode, only 8-bits chars are supported */
    DEV_ASSERT((p_lpuart_user_config->transfer_type != LPUART_USING_DMA) ||
               (p_lpuart_user_config->bit_count_per_char == LPUART_8_BITS_PER_CHAR));

    /* For 10 bits per char, parity bit cannot be enabled */
    DEV_ASSERT((p_lpuart_user_config->bit_count_per_char != LPUART_10_BITS_PER_CHAR) ||
               (p_lpuart_user_config->parity_mode == LPUART_PARITY_DISABLED));

    /* Clear the state struct for this instance. */
    uint8_t *p_clear_struct_ptr = (uint8_t *)p_lpuart_state_ptr;
    for (idx = 0; idx < sizeof(lpuart_state_t); idx++)
    {
        p_clear_struct_ptr[idx] = 0;
    }

    /* Save runtime structure pointer.*/
    sp_lpuart_state_ptr[instance] = p_lpuart_state_ptr;

    /* Save the transfer information for runtime retrieval */
    p_lpuart_state_ptr->transfer_type      = p_lpuart_user_config->transfer_type;
    p_lpuart_state_ptr->bit_count_per_char = p_lpuart_user_config->bit_count_per_char;
    p_lpuart_state_ptr->rx_dma_channel     = p_lpuart_user_config->rx_dma_channel;
    p_lpuart_state_ptr->tx_dma_channel     = p_lpuart_user_config->tx_dma_channel;

    /* initialize the LPUART instance */
    lpuart_init_set(p_base);

    /* initialize the parameters of the LPUART config structure with desired
     * data */
    (void)lpuart_set_baud_rate(instance, p_lpuart_user_config->baud_rate);

    if (p_lpuart_user_config->parity_mode != LPUART_PARITY_DISABLED)
    {
        lpuart_set_bit_count_per_char(p_base, p_lpuart_user_config->bit_count_per_char, true);
    }
    else
    {
        lpuart_set_bit_count_per_char(p_base, p_lpuart_user_config->bit_count_per_char, false);
    }
    lpuart_set_parity_mode(p_base, p_lpuart_user_config->parity_mode);
    lpuart_set_stop_bit_count(p_base, p_lpuart_user_config->stop_bit_count);

    /* initialize last driver operation status */
    p_lpuart_state_ptr->transmit_status = STATUS_SUCCESS;
    p_lpuart_state_ptr->receive_status  = STATUS_SUCCESS;

    /* Create the synchronization objects */
    os_status_rx_sem = osif_sema_create(&p_lpuart_state_ptr->rx_complete, 0);
    os_status_tx_sem = osif_sema_create(&p_lpuart_state_ptr->tx_complete, 0);
    if ((os_status_rx_sem == STATUS_ERROR) || (os_status_tx_sem == STATUS_ERROR))
    {
        return STATUS_ERROR;
    }

    /* Enable LPUART interrupt. */
    int_enable_irq(s_lpuart_irq_id[instance]);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_deinit
 * Description   : This function shuts down the UART by disabling interrupts and
 *                 transmitter/receiver.
 *
 * Implements    : lpuart_deinit_activity
 *END**************************************************************************/
status_t
lpuart_deinit(uint32_t instance)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    clock_names_t         instance_clk_name = s_lpuart_clk_names[instance];
    uint32_t              lpuart_source_clock;
    const LPUART_t       *p_base         = sp_lpuart_base[instance];
    const lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    (void)clock_get_freq(instance_clk_name, &lpuart_source_clock);

    /* Check if current instance is already de-initialized or is gated.*/
    DEV_ASSERT(sp_lpuart_state_ptr[instance] != NULL);
    DEV_ASSERT(lpuart_source_clock > 0U);

    /* Wait until the data is completely shifted out of shift register */
    while (!lpuart_get_status_flag(p_base, LPUART_TX_COMPLETE))
    {
    }

    /* Destroy the synchronization objects */
    (void)osif_sema_destroy(&p_lpuart_state->rx_complete);
    (void)osif_sema_destroy(&p_lpuart_state->tx_complete);

    /* Disable LPUART interrupt. */
    int_disable_irq(s_lpuart_irq_id[instance]);

    /* Clear our saved pointer to the state structure */
    sp_lpuart_state_ptr[instance] = NULL;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_install_rx_callback
 * Description   : Install receive data callback function.
 *
 * Implements    : lpuart_install_rx_callback_activity
 *END**************************************************************************/
uart_callback_t
lpuart_install_rx_callback(uint32_t instance, uart_callback_t function, void *p_callback_param)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    uart_callback_t current_callback    = p_lpuart_state->rx_callback;
    p_lpuart_state->rx_callback         = function;
    p_lpuart_state->p_rx_callback_param = p_callback_param;

    return current_callback;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_install_tx_callback
 * Description   : Install transmit data callback function, pass in NULL pointer
 * as callback will uninstall.
 *
 * Implements    : lpuart_install_tx_callback_activity
 *END**************************************************************************/
uart_callback_t
lpuart_install_tx_callback(uint32_t instance, uart_callback_t function, void *p_callback_param)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    uart_callback_t current_callback    = p_lpuart_state->tx_callback;
    p_lpuart_state->tx_callback         = function;
    p_lpuart_state->p_tx_callback_param = p_callback_param;

    return current_callback;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_send_data_blocking
 * Description   : This function sends data out through the LPUART module using
 * blocking method. The function does not return until the transmit is complete.
 *
 * Implements    : lpuart_send_data_blocking_activity
 *END**************************************************************************/
status_t
lpuart_send_data_blocking(uint32_t       instance,
                          const uint8_t *p_tx_buff,
                          uint32_t       tx_size,
                          uint32_t       timeout)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_tx_buff != NULL);

    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    status_t        ret_val        = STATUS_SUCCESS;
    status_t        sync_status;

    /* Indicates this is a blocking transaction. */
    p_lpuart_state->b_is_tx_blocking = true;

    DEV_ASSERT((p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS) ||
               (p_lpuart_state->transfer_type == LPUART_USING_DMA));

    if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
    {
        /* Start the transmission process using interrupts */
        ret_val = lpuart_start_send_data_using_int(instance, p_tx_buff, tx_size);
    }
    else
    {
        /* Start the transmission process using DMA */
        ret_val = lpaurt_start_send_data_using_dma(instance, p_tx_buff, tx_size);
    }

    if (ret_val == STATUS_SUCCESS)
    {
        /* Wait until the transmit is complete. */
        sync_status = osif_sema_wait(&p_lpuart_state->tx_complete, timeout);

        /* Finish the transmission if timeout expired */
        if (sync_status == STATUS_TIMEOUT)
        {
            p_lpuart_state->b_is_tx_blocking = false;
            p_lpuart_state->transmit_status  = STATUS_TIMEOUT;

            if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
            {
                lpuart_complete_send_data_using_int(instance);
            }
            else
            {
                lpuart_stop_tx_dma(instance);
            }
        }
    }

    return p_lpuart_state->transmit_status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_send_data_polling
 * Description   : Send out multiple bytes of data using polling method.
 *
 * Implements    : lpuart_send_data_polling_activity
 *END**************************************************************************/
status_t
lpuart_send_data_polling(uint32_t instance, const uint8_t *p_tx_buff, uint32_t tx_size)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_tx_buff != NULL);

    LPUART_t       *p_base         = sp_lpuart_base[instance];
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Check the validity of the parameters */
    DEV_ASSERT(tx_size > 0U);
    DEV_ASSERT((p_lpuart_state->bit_count_per_char == LPUART_8_BITS_PER_CHAR) ||
               ((tx_size & 1U) == 0U));

    /* Check driver is not busy transmitting data from a previous asynchronous
     * call */
    if (p_lpuart_state->b_is_tx_busy)
    {
        return STATUS_BUSY;
    }

    /* Enable the LPUART transmitter */
    lpuart_set_transmitter_cmd(p_base, true);

    while (tx_size > 0U)
    {
        while (!lpuart_get_status_flag(p_base, LPUART_TX_DATA_REG_EMPTY))
        {
        }

        p_lpuart_state->txBuff = p_tx_buff;
        lpuart_put_data(instance);

        if (p_lpuart_state->bit_count_per_char == LPUART_8_BITS_PER_CHAR)
        {
            ++p_tx_buff;
            --tx_size;
        }
        else
        {
            ++p_tx_buff;
            ++p_tx_buff;
            tx_size -= 2U;
        }
    }

    /* Disable the LPUART transmitter */
    lpuart_set_transmitter_cmd(p_base, false);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_send_data
 * Description   : This function sends data out through the LPUART module using
 * non-blocking method. The function will return immediately after calling this
 * function.
 *
 * Implements    : lpuart_send_data_activity
 *END**************************************************************************/
status_t
lpuart_send_data(uint32_t instance, const uint8_t *p_tx_buff, uint32_t tx_size)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_tx_buff != NULL);

    status_t        retVal         = STATUS_SUCCESS;
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Indicates this is a non-blocking transaction. */
    p_lpuart_state->b_is_tx_blocking = false;

    DEV_ASSERT((p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS) ||
               (p_lpuart_state->transfer_type == LPUART_USING_DMA));

    if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
    {
        /* Start the transmission process using interrupts */
        retVal = lpuart_start_send_data_using_int(instance, p_tx_buff, tx_size);
    }
    else
    {
        /* Start the transmission process using DMA */
        retVal = lpaurt_start_send_data_using_dma(instance, p_tx_buff, tx_size);
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_get_transmit_status
 * Description   : This function returns whether the previous LPUART transmit
 * has finished. When performing non-blocking transmit, the user can call this
 * function to ascertain the state of the current transmission:
 * in progress (or busy) or complete (success). In addition, if the transmission
 * is still in progress, the user can obtain the number of words that have been
 * currently transferred.
 *
 * Implements    : lpuart_get_transmit_status_activity
 *END**************************************************************************/
status_t
lpuart_get_transmit_status(uint32_t instance, uint32_t *p_bytes_remaining)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    const lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    if (p_bytes_remaining != NULL)
    {
        if (p_lpuart_state->b_is_tx_busy)
        {
            /* Fill in the bytes not transferred yet. */
            if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
            {
                /* In interrupt-based communication, the remaining bytes are
                 * retrieved from the state structure
                 */
                *p_bytes_remaining = p_lpuart_state->tx_size;
            }
            else
            {
                /* In DMA-based communication, the remaining bytes are retrieved
                 * from the current DMA major loop count
                 */
                *p_bytes_remaining =
                    edma_get_remaining_major_iterations_count(p_lpuart_state->tx_dma_channel);
            }
        }
        else
        {
            *p_bytes_remaining = 0;
        }
    }

    return p_lpuart_state->transmit_status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_abort_sending_data
 * Description   : This function terminates an non-blocking LPUART transmission
 * early. During a non-blocking LPUART transmission, the user has the option to
 * terminate the transmission early if the transmission is still in progress.
 *
 * Implements    : lpuart_abort_sending_data_activity
 *END**************************************************************************/
status_t
lpuart_abort_sending_data(uint32_t instance)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Check if a transfer is running. */
    if (!p_lpuart_state->b_is_tx_busy)
    {
        return STATUS_SUCCESS;
    }

    /* Update the tx status */
    p_lpuart_state->transmit_status = STATUS_UART_ABORTED;

    /* Stop the running transfer. */
    if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
    {
        lpuart_complete_send_data_using_int(instance);
    }
    else
    {
        lpuart_stop_tx_dma(instance);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_receive_data_blocking
 * Description   : This function receives data from LPUART module using blocking
 * method, the function does not return until the receive is complete.
 *
 * Implements    : lpuart_receive_data_blocking_activity
 *END**************************************************************************/
status_t
lpuart_receive_data_blocking(uint32_t instance,
                             uint8_t *p_rx_buff,
                             uint32_t rx_size,
                             uint32_t timeout)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_rx_buff != NULL);

    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    status_t        ret_val        = STATUS_SUCCESS;
    status_t        syncStatus;

    /* Indicates this is a blocking transaction. */
    p_lpuart_state->b_is_rx_blocking = true;

    DEV_ASSERT((p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS) ||
               (p_lpuart_state->transfer_type == LPUART_USING_DMA));

    if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
    {
        /* Start the reception process using interrupts */
        ret_val = lpuart_start_receive_data_using_int(instance, p_rx_buff, rx_size);
    }
    else
    {
        /* Start the reception process using DMA */
        ret_val = lpuart_start_receive_data_using_dma(instance, p_rx_buff, rx_size);
    }

    if (ret_val == STATUS_SUCCESS)
    {
        /* Wait until the receive is complete. */
        syncStatus = osif_sema_wait(&p_lpuart_state->rx_complete, timeout);

        /* Finish the reception if timeout expired */
        if (syncStatus == STATUS_TIMEOUT)
        {
            p_lpuart_state->b_is_rx_blocking = false;
            p_lpuart_state->receive_status   = STATUS_TIMEOUT;

            if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
            {
                lpuart_complete_receive_data_using_int(instance);
            }
            else
            {
                lpuart_stop_rx_dma(instance);
            }
        }
    }

    return p_lpuart_state->receive_status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_receive_data_polling
 * Description   : Receive multiple bytes of data using polling method.
 *
 * Implements    : lpuart_receive_data_polling_activity
 *END**************************************************************************/
status_t
lpuart_receive_data_polling(uint32_t instance, uint8_t *p_rx_buff, uint32_t rx_size)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_rx_buff != NULL);

    uint8_t         tmp_byte;
    status_t        ret_val        = STATUS_SUCCESS;
    status_t        tmp_state      = STATUS_SUCCESS;
    LPUART_t       *p_base         = sp_lpuart_base[instance];
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Check the validity of the parameters */
    DEV_ASSERT(rx_size > 0U);
    DEV_ASSERT((p_lpuart_state->bit_count_per_char == LPUART_8_BITS_PER_CHAR) ||
               ((rx_size & 1U) == 0U));

    /* Check driver is not busy receiving data from a previous asynchronous call
     */
    if (p_lpuart_state->b_is_rx_busy)
    {
        return STATUS_BUSY;
    }

    /* Enable the LPUART receiver */
    lpuart_set_receiver_cmd((LPUART_t *)p_base, true);

    while (rx_size > 0U)
    {
        while (!lpuart_get_status_flag(p_base, LPUART_RX_DATA_REG_FULL))
        {
        }

        p_lpuart_state->rxBuff = p_rx_buff;
        lpuart_get_data(instance);

        if (p_lpuart_state->bit_count_per_char == LPUART_8_BITS_PER_CHAR)
        {
            ++p_rx_buff;
            --rx_size;
        }
        else
        {
            ++p_rx_buff;
            ++p_rx_buff;
            rx_size -= 2U;
        }

        /* Check for errors on received data */
        if (lpuart_get_status_flag(p_base, LPUART_FRAME_ERR))
        {
            tmp_state = STATUS_UART_FRAMING_ERROR;
            /* Disable the LPUART receiver */
            lpuart_set_receiver_cmd((LPUART_t *)p_base, false);
            /* Clear the flag */
            (void)lpuart_clear_status_flag(p_base, LPUART_FRAME_ERR);
            break;
        }
        if (lpuart_get_status_flag(p_base, LPUART_NOISE_DETECT))
        {
            tmp_state = STATUS_UART_NOISE_ERROR;
            /* Disable the LPUART receiver */
            lpuart_set_receiver_cmd((LPUART_t *)p_base, false);
            /* Clear the flag */
            (void)lpuart_clear_status_flag(p_base, LPUART_NOISE_DETECT);
            break;
        }
        if (lpuart_get_status_flag(p_base, LPUART_PARITY_ERR))
        {
            tmp_state = STATUS_UART_PARITY_ERROR;
            /* Disable the LPUART receiver */
            lpuart_set_receiver_cmd((LPUART_t *)p_base, false);
            /* Clear the flag */
            (void)lpuart_clear_status_flag(p_base, LPUART_PARITY_ERR);
            break;
        }
        if (lpuart_get_status_flag(p_base, LPUART_RX_OVERRUN))
        {
            tmp_state = STATUS_UART_RX_OVERRUN;
            /* Disable the LPUART receiver */
            lpuart_set_receiver_cmd((LPUART_t *)p_base, false);
            /* Clear the flag */
            (void)lpuart_clear_status_flag(p_base, LPUART_RX_OVERRUN);
            break;
        }
    }

    /* Update received status */
    if ((rx_size == 0U) && (tmp_state == STATUS_UART_RX_OVERRUN))
    {
        ret_val = STATUS_SUCCESS;
    }
    else
    {
        ret_val = tmp_state;
    }

    if (ret_val == STATUS_SUCCESS)
    {
        /* Disable the LPUART receiver */
        lpuart_set_receiver_cmd((LPUART_t *)p_base, false);
    }

    /* Read dummy to clear RDRF flag */
    lpuart_get_char(p_base, &tmp_byte);

    return ret_val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_receive_data
 * Description   : This function receives data from LPUART module using
 * non-blocking method.  This function returns immediately after initiating the
 * receive function. The application has to get the receive status to see when
 * the receive is complete. In other words, after calling non-blocking get
 * function, the application must get the receive status to check if receive
 * is completed or not.
 *
 * Implements    : lpuart_receive_data_activity
 *END**************************************************************************/
status_t
lpuart_receive_data(uint32_t instance, uint8_t *p_rx_buff, uint32_t rx_size)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_rx_buff != NULL);

    status_t        ret_val        = STATUS_SUCCESS;
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Indicates this is a non-blocking transaction. */
    p_lpuart_state->b_is_rx_blocking = false;

    DEV_ASSERT((p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS) ||
               (p_lpuart_state->transfer_type == LPUART_USING_DMA));

    if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
    {
        /* Start the reception process using interrupts */
        ret_val = lpuart_start_receive_data_using_int(instance, p_rx_buff, rx_size);
    }
    else
    {
        /* Start the reception process using DMA */
        ret_val = lpuart_start_receive_data_using_dma(instance, p_rx_buff, rx_size);
    }

    return ret_val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_get_receive_status
 * Description   : This function returns whether the previous LPUART receive is
 * complete. When performing a non-blocking receive, the user can call this
 * function to ascertain the state of the current receive progress: in progress
 * or complete. In addition, if the receive is still in progress, the user can
 * obtain the number of words that have been currently received.
 *
 * Implements    : lpuart_get_receive_status_activity
 *END**************************************************************************/
status_t
lpuart_get_receive_status(uint32_t instance, uint32_t *p_bytes_remaining)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    const lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    if (p_bytes_remaining != NULL)
    {
        if (p_lpuart_state->b_is_rx_busy)
        {
            /* Fill in the bytes transferred. */
            if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
            {
                /* In interrupt-based communication, the remaining bytes are
                 * retrieved from the state structure
                 */
                *p_bytes_remaining = p_lpuart_state->rx_size;
            }
            else
            {
                /* In DMA-based communication, the remaining bytes are retrieved
                 * from the current DMA major loop count
                 */
                *p_bytes_remaining =
                    edma_get_remaining_major_iterations_count(p_lpuart_state->rx_dma_channel);
            }
        }
        else
        {
            *p_bytes_remaining = 0;
        }
    }

    return p_lpuart_state->receive_status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_abort_receiving_data
 * Description   : Terminates a non-blocking receive early.
 *
 * Implements    : lpuart_abort_receiving_data_activity
 *END**************************************************************************/
status_t
lpuart_abort_receiving_data(uint32_t instance)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Check if a transfer is running. */
    if (!p_lpuart_state->b_is_rx_busy)
    {
        return STATUS_SUCCESS;
    }

    /* Update the rx status */
    p_lpuart_state->receive_status = STATUS_UART_ABORTED;

    /* Stop the running transfer. */
    if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
    {
        lpuart_complete_receive_data_using_int(instance);
    }
    else
    {
        lpuart_stop_rx_dma(instance);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_set_baud_rate
 * Description   : Configures the LPUART baud rate.
 * In some LPUART instances the user must disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * Implements    : lpuart_set_baud_rate_activity
 *END**************************************************************************/
status_t
lpuart_set_baud_rate(uint32_t instance, uint32_t desired_baud_rate)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    uint16_t              sbr, sbr_temp, idx;
    uint32_t              osr, temp_diff, calculated_baud, baud_diff, max_osr;
    uint32_t              lpuart_source_clock;
    clock_names_t         instance_clk_name = s_lpuart_clk_names[instance];
    LPUART_t             *p_base            = sp_lpuart_base[instance];
    const lpuart_state_t *p_lpuart_state;
    p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    if (p_lpuart_state != NULL)
    {
        /* Check if there is an ongoing transfer */
        if (p_lpuart_state->b_is_tx_busy == true)
        {
            return STATUS_BUSY;
        }
        if (p_lpuart_state->b_is_rx_busy == true)
        {
            return STATUS_BUSY;
        }
    }

    /* Get the LPUART clock as configured in the clock manager */
    (void)clock_get_freq(instance_clk_name, &lpuart_source_clock);

    /* Check if current instance is clock gated off. */
    DEV_ASSERT(lpuart_source_clock > 0U);
    /* Check if the desired baud rate can be configured with the current
     * protocol clock. */
    DEV_ASSERT(lpuart_source_clock >= (desired_baud_rate * 4U));

    /* This lpuart instantiation uses a slightly different baud rate calculation
     * The idea is to use the best OSR (over-sampling rate) possible
     * Note, osr is typically hard-set to 16 in other lpuart instantiations
     * First calculate the baud rate using the minimum OSR possible (4) */
    osr             = 4;
    sbr             = (uint16_t)(lpuart_source_clock / (desired_baud_rate * osr));
    calculated_baud = (lpuart_source_clock / (osr * sbr));
    if (calculated_baud > desired_baud_rate)
    {
        baud_diff = calculated_baud - desired_baud_rate;
    }
    else
    {
        baud_diff = desired_baud_rate - calculated_baud;
    }
    /* find maximum osr */
    max_osr = lpuart_source_clock / desired_baud_rate;
    if (max_osr > 32U)
    {
        max_osr = 32U;
    }
    /* loop to find the best osr value possible, one that generates minimum
     * baud_diff iterate through the rest of the supported values of osr */
    if (max_osr >= 5U)
    {
        for (idx = 5U; idx <= max_osr; idx++)
        {
            /* calculate the temporary sbr value   */
            sbr_temp = (uint16_t)(lpuart_source_clock / (desired_baud_rate * idx));
            /* calculate the baud rate based on the temporary osr and sbr values
             */
            calculated_baud = (lpuart_source_clock / (idx * sbr_temp));

            if (calculated_baud > desired_baud_rate)
            {
                temp_diff = calculated_baud - desired_baud_rate;
            }
            else
            {
                temp_diff = desired_baud_rate - calculated_baud;
            }

            if (temp_diff <= baud_diff)
            {
                baud_diff = temp_diff;
                osr       = idx;      /* update and store the best osr value calculated */
                sbr       = sbr_temp; /* update store the best sbr value calculated */
            }
        }
    }
    /* Check if osr is between 4x and 7x oversampling.
     * If so, then "BOTHEDGE" sampling must be turned on */
    if (osr < 8U)
    {
        lpuart_enable_both_edge_sampling(p_base);
    }

    /* program the osr value (bit value is one less than actual value) */
    lpuart_set_oversampling_ratio(p_base, (osr - 1U));

    /* write the sbr value to the BAUD registers */
    lpuart_set_baud_rate_divisor(p_base, sbr);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_get_baud_rate
 * Description   : Returns the LPUART configured baud rate.
 *
 * Implements    : lpuart_get_baud_rate_activity
 *END**************************************************************************/
void
lpuart_get_baud_rate(uint32_t instance, uint32_t *p_configured_baud_rate)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_configured_baud_rate != NULL);

    uint8_t         osr;
    uint16_t        sbr;
    uint32_t        lpuart_source_clock;
    clock_names_t   instance_clk_name = s_lpuart_clk_names[instance];
    const LPUART_t *p_base            = sp_lpuart_base[instance];

    /* Get the LPUART clock as configured in the clock manager */
    (void)clock_get_freq(instance_clk_name, &lpuart_source_clock);

    osr = lpuart_get_oversampling_ratio(p_base);
    sbr = lpuart_get_baud_rate_divisor(p_base);

    *p_configured_baud_rate = (lpuart_source_clock / ((osr + 1UL) * sbr));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpaurt_set_tx_buffer
 * Description   : Sets the driver internal reference to the tx buffer.
 *                 Can be called from the tx callback to provide a different
 *                 buffer for continuous transmission.
 *
 * Implements    : lpaurt_set_tx_buffer_activity
 *END**************************************************************************/
status_t
lpaurt_set_tx_buffer(uint32_t instance, const uint8_t *p_tx_buff, uint32_t tx_size)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_tx_buff != NULL);
    DEV_ASSERT(tx_size > 0U);

    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    p_lpuart_state->txBuff         = p_tx_buff;
    p_lpuart_state->tx_size        = tx_size;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_set_rx_buffer
 * Description   : Sets the driver internal reference to the rx buffer.
 *                 Can be called from the rx callback to provide a different
 *                 buffer for continuous reception.
 *
 * Implements    : lpuart_set_rx_buffer_activity
 *END**************************************************************************/
status_t
lpuart_set_rx_buffer(uint32_t instance, uint8_t *p_rx_buff, uint32_t rx_size)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_rx_buff != NULL);
    DEV_ASSERT(rx_size > 0U);

    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    p_lpuart_state->rxBuff         = p_rx_buff;
    p_lpuart_state->rx_size        = rx_size;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_irq_handler
 * Description   : Interrupt handler for LPUART.
 * This handler uses the buffers stored in the lpuart_state_t structs to
 *transfer data. This is not a public API as it is called by IRQ whenever an
 *interrupt occurs.
 *
 *END**************************************************************************/
void
lpuart_irq_handler(uint32_t instance)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    const LPUART_t *p_base = sp_lpuart_base[instance];

    lpuart_err_irq_handler(instance);

    /* Handle receive data full interrupt */
    if (lpuart_get_int_mode(p_base, LPUART_INT_RX_DATA_REG_FULL))
    {
        if (lpuart_get_status_flag(p_base, LPUART_RX_DATA_REG_FULL))
        {
            lpuart_rx_irq_handler(instance);
        }
    }

    /* Handle transmitter data register empty interrupt */
    if (lpuart_get_int_mode(p_base, LPUART_INT_TX_DATA_REG_EMPTY))
    {
        if (lpuart_get_status_flag(p_base, LPUART_TX_DATA_REG_EMPTY))
        {
            lpuart_tx_empty_irq_handler(instance);
        }
    }

    /* Handle transmission complete interrupt */
    if (lpuart_get_int_mode(p_base, LPUART_INT_TX_COMPLETE))
    {
        if (lpuart_get_status_flag(p_base, LPUART_TX_COMPLETE))
        {
            lpaurt_tx_complete_irq_handler(instance);
        }
    }
}

#if defined(CUSTOM_LPUART_IRQ)
#else
/* Implementation of LPUART0 handler named in startup code. */
void
LPUART0_IRQHandler(void)
{
    lpuart_irq_handler(0);
}

/* Implementation of LPUART1 handler named in startup code. */
void
LPUART1_IRQHandler(void)
{
    lpuart_irq_handler(1);
}
#endif

/*******************************************************************************
 * Static functions
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_rx_irq_handler
 * Description   : Rx Interrupt handler for LPUART.
 * This function treats the rx full interrupt.
 *
 *END**************************************************************************/
static void
lpuart_rx_irq_handler(uint32_t instance)
{
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Get data and put in receive buffer  */
    lpuart_get_data(instance);

    /* Update the internal state */
    if (p_lpuart_state->bit_count_per_char == LPUART_8_BITS_PER_CHAR)
    {
        ++p_lpuart_state->rxBuff;
        --p_lpuart_state->rx_size;
    }
    else
    {
        p_lpuart_state->rxBuff = &p_lpuart_state->rxBuff[2];
        p_lpuart_state->rx_size -= 2U;
    }

    /* Check if this was the last byte in the current buffer */
    if (p_lpuart_state->rx_size == 0U)
    {
        /* Invoke callback if there is one (callback may reset the rx buffer for
         * continuous reception) */
        if (p_lpuart_state->rx_callback != NULL)
        {
            p_lpuart_state->rx_callback(
                p_lpuart_state, UART_EVENT_RX_FULL, p_lpuart_state->p_rx_callback_param);
        }
    }

    /* Finish reception if this was the last byte received */
    if (p_lpuart_state->rx_size == 0U)
    {
        /* Complete transfer (disable rx logic) */
        lpuart_complete_receive_data_using_int(instance);

        /* Invoke callback if there is one */
        if (p_lpuart_state->rx_callback != NULL)
        {
            p_lpuart_state->rx_callback(
                p_lpuart_state, UART_EVENT_END_TRANSFER, p_lpuart_state->p_rx_callback_param);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_tx_empty_irq_handler
 * Description   : Tx Empty Interrupt handler for LPUART.
 * This function treats the tx empty interrupt.
 *
 *END**************************************************************************/
static void
lpuart_tx_empty_irq_handler(uint32_t instance)
{
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    LPUART_t       *p_base         = sp_lpuart_base[instance];

    /* Check if there are any more bytes to send */
    if (p_lpuart_state->tx_size > 0U)
    {
        /* Transmit the data */
        lpuart_put_data(instance);

        /* Update the internal state */
        if (p_lpuart_state->bit_count_per_char == LPUART_8_BITS_PER_CHAR)
        {
            ++p_lpuart_state->txBuff;
            --p_lpuart_state->tx_size;
        }
        else
        {
            p_lpuart_state->txBuff = &p_lpuart_state->txBuff[2];
            p_lpuart_state->tx_size -= 2U;
        }

        /* Check if this was the last byte in the current buffer */
        if (p_lpuart_state->tx_size == 0U)
        {
            /* Invoke callback if there is one (callback may reset the tx buffer
             * for continuous transmission)*/
            if (p_lpuart_state->tx_callback != NULL)
            {
                p_lpuart_state->tx_callback(
                    p_lpuart_state, UART_EVENT_TX_EMPTY, p_lpuart_state->p_tx_callback_param);
            }

            /* If there's no new data, disable tx empty interrupt and enable
             * transmission complete interrupt */
            if (p_lpuart_state->tx_size == 0U)
            {
                lpuart_set_int_mode(p_base, LPUART_INT_TX_DATA_REG_EMPTY, false);
                lpuart_set_int_mode(p_base, LPUART_INT_TX_COMPLETE, true);
            }
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpaurt_tx_complete_irq_handler
 * Description   : Tx Complete Interrupt handler for LPUART.
 * This function treats the tx complete interrupt.
 *
 *END**************************************************************************/
static void
lpaurt_tx_complete_irq_handler(uint32_t instance)
{
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    if (p_lpuart_state->tx_size == 0U)
    {
        if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
        {
            /* Complete the interrupt based transfer */
            lpuart_complete_send_data_using_int(instance);
        }
        else
        {
            /* Complete the DMA based transfer */
            lpuart_stop_tx_dma(instance);
        }
        /* Invoke callback if there is one */
        if (p_lpuart_state->tx_callback != NULL)
        {
            p_lpuart_state->tx_callback(
                p_lpuart_state, UART_EVENT_END_TRANSFER, p_lpuart_state->p_tx_callback_param);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_err_irq_handler
 * Description   : Error Interrupt handler for LPUART.
 * This function treats the error interrupts.
 *
 *END**************************************************************************/
static void
lpuart_err_irq_handler(uint32_t instance)
{
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    LPUART_t       *p_base         = sp_lpuart_base[instance];

    /* Handle receive overrun interrupt */
    if (lpuart_get_status_flag(p_base, LPUART_RX_OVERRUN))
    {
        /* Update the internal status */
        p_lpuart_state->receive_status = STATUS_UART_RX_OVERRUN;
        if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
        {
            /* Complete the transfer (disable rx logic) */
            lpuart_complete_receive_data_using_int(instance);
        }
        else
        {
            /* Complete the transfer (stop DMA channel) */
            lpuart_stop_rx_dma(instance);
        }
        /* Invoke callback if there is one */
        if (p_lpuart_state->rx_callback != NULL)
        {
            p_lpuart_state->rx_callback(
                p_lpuart_state, UART_EVENT_ERROR, p_lpuart_state->p_rx_callback_param);
        }

        /* Clear the flags */
        lpuart_clear_error_flags(p_base);
    }
    /* Handle framing error interrupt */
    if (lpuart_get_status_flag(p_base, LPUART_FRAME_ERR))
    {
        /* Update the internal status */
        p_lpuart_state->receive_status = STATUS_UART_FRAMING_ERROR;
        if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
        {
            /* Complete the transfer (disable rx logic) */
            lpuart_complete_receive_data_using_int(instance);
        }
        else
        {
            /* Complete the transfer (stop DMA channel) */
            lpuart_stop_rx_dma(instance);
        }
        /* Invoke callback if there is one */
        if (p_lpuart_state->rx_callback != NULL)
        {
            p_lpuart_state->rx_callback(
                p_lpuart_state, UART_EVENT_ERROR, p_lpuart_state->p_rx_callback_param);
        }

        /* Clear the flags */
        lpuart_clear_error_flags(p_base);
    }
    /* Handle parity error interrupt */
    if (lpuart_get_status_flag(p_base, LPUART_PARITY_ERR))
    {
        /* Update the internal status */
        p_lpuart_state->receive_status = STATUS_UART_PARITY_ERROR;
        if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
        {
            /* Complete the transfer (disable rx logic) */
            lpuart_complete_receive_data_using_int(instance);
        }
        else
        {
            /* Complete the transfer (stop DMA channel) */
            lpuart_stop_rx_dma(instance);
        }
        /* Invoke callback if there is one */
        if (p_lpuart_state->rx_callback != NULL)
        {
            p_lpuart_state->rx_callback(
                p_lpuart_state, UART_EVENT_ERROR, p_lpuart_state->p_rx_callback_param);
        }

        /* Clear the flags */
        lpuart_clear_error_flags(p_base);
    }
    /* Handle noise error interrupt */
    if (lpuart_get_status_flag(p_base, LPUART_NOISE_DETECT))
    {
        /* Update the internal status */
        p_lpuart_state->receive_status = STATUS_UART_NOISE_ERROR;
        if (p_lpuart_state->transfer_type == LPUART_USING_INTERRUPTS)
        {
            /* Complete transfer (disable rx logic) */
            lpuart_complete_receive_data_using_int(instance);
        }
        else
        {
            /* Complete the transfer (stop DMA channel) */
            lpuart_stop_rx_dma(instance);
        }
        /* Invoke callback if there is one */
        if (p_lpuart_state->rx_callback != NULL)
        {
            p_lpuart_state->rx_callback(
                p_lpuart_state, UART_EVENT_ERROR, p_lpuart_state->p_rx_callback_param);
        }

        /* Clear the flags */
        lpuart_clear_error_flags(p_base);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_start_send_data_using_int
 * Description   : Initiate (start) a transmit by beginning the process of
 * sending data and enabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static status_t
lpuart_start_send_data_using_int(uint32_t instance, const uint8_t *p_tx_buff, uint32_t tx_size)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_tx_buff != NULL);

    LPUART_t       *p_base         = sp_lpuart_base[instance];
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Check it's not busy transmitting data from a previous function call */
    if (p_lpuart_state->b_is_tx_busy)
    {
        return STATUS_BUSY;
    }

    /* Check the validity of the parameters */
    DEV_ASSERT(tx_size > 0U);
    DEV_ASSERT((p_lpuart_state->bit_count_per_char == LPUART_8_BITS_PER_CHAR) ||
               ((tx_size & 1U) == 0U));

    /* initialize the module driver state structure */
    p_lpuart_state->txBuff          = p_tx_buff;
    p_lpuart_state->tx_size         = tx_size;
    p_lpuart_state->b_is_tx_busy    = true;
    p_lpuart_state->transmit_status = STATUS_BUSY;

    /* Enable the LPUART transmitter */
    lpuart_set_transmitter_cmd(p_base, true);

    /* Enable tx empty interrupt */
    lpuart_set_int_mode(p_base, LPUART_INT_TX_DATA_REG_EMPTY, true);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpaurt_start_send_data_using_dma
 * Description   : Initiate (start) a transmit by beginning the process of
 * sending data using DMA transfers.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static status_t
lpaurt_start_send_data_using_dma(uint32_t instance, const uint8_t *p_tx_buff, uint32_t tx_size)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_tx_buff != NULL);

    LPUART_t       *p_base         = sp_lpuart_base[instance];
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Check it's not busy transmitting data from a previous function call */
    if (p_lpuart_state->b_is_tx_busy)
    {
        return STATUS_BUSY;
    }

    DEV_ASSERT(tx_size > 0U);

    /* Update state structure */
    p_lpuart_state->txBuff          = p_tx_buff;
    p_lpuart_state->tx_size         = 0U;
    p_lpuart_state->b_is_tx_busy    = true;
    p_lpuart_state->transmit_status = STATUS_BUSY;

    /* Configure the transfer control descriptor for the previously allocated
     * channel */
    (void)edma_config_multi_block_transfer(p_lpuart_state->tx_dma_channel,
                                           EDMA_TRANSFER_MEM2PERIPH,
                                           (uint32_t)p_tx_buff,
                                           (uint32_t)(&(p_base->DATA)),
                                           EDMA_TRANSFER_SIZE_1B,
                                           1U,
                                           tx_size,
                                           true);

    /* Call driver function to end the transmission when the DMA transfer is
     * done */
    (void)edma_install_callback(p_lpuart_state->tx_dma_channel,
                                (edma_callback_t)(lpuart_tx_dma_callback),
                                (void *)(instance));

    /* Start the DMA channel */
    (void)edma_start_channel(p_lpuart_state->tx_dma_channel);

    /* Enable the LPUART transmitter */
    lpuart_set_transmitter_cmd(p_base, true);

    /* Enable tx DMA requests for the current instance */
    lpuart_set_tx_dma_cmd(p_base, true);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_complete_send_data_using_int
 * Description   : Finish up a transmit by completing the process of sending
 * data and disabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void
lpuart_complete_send_data_using_int(uint32_t instance)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    LPUART_t       *p_base         = sp_lpuart_base[instance];
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    if (p_lpuart_state->transmit_status == STATUS_BUSY)
    {
        /* If the transfer is completed, update the transmit status */
        p_lpuart_state->transmit_status = STATUS_SUCCESS;
    }
    else
    {
        /* If the transfer is aborted or timed out, disable tx empty interrupt
         */
        lpuart_set_int_mode(p_base, LPUART_INT_TX_DATA_REG_EMPTY, false);
    }

    /* Disable transmission complete interrupt */
    lpuart_set_int_mode(p_base, LPUART_INT_TX_COMPLETE, false);

    /* Disable transmitter */
    lpuart_set_transmitter_cmd(p_base, false);

    /* Update the internal busy flag */
    p_lpuart_state->b_is_tx_busy = false;

    /* Signal the synchronous completion object. */
    if (p_lpuart_state->b_is_tx_blocking)
    {
        (void)osif_sema_post(&p_lpuart_state->tx_complete);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_tx_dma_callback
 * Description   : Finish up a transmit by completing the process of sending
 * data and disabling the DMA requests. This is a callback for DMA major loop
 * completion, so it must match the DMA callback signature.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void
lpuart_tx_dma_callback(void *p_parameter, edma_chn_status_t status)
{
    uint32_t        instance       = ((uint32_t)p_parameter);
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    LPUART_t       *p_base         = sp_lpuart_base[instance];

    /* Check if the DMA transfer completed with errors */
    if (status == EDMA_CHN_ERROR)
    {
        /* Update the status */
        p_lpuart_state->transmit_status = STATUS_ERROR;
        /* Stop the transfer */
        lpuart_stop_tx_dma(instance);
        /* Notify the application that an error occurred */
        if (p_lpuart_state->tx_callback != NULL)
        {
            p_lpuart_state->tx_callback(
                p_lpuart_state, UART_EVENT_ERROR, p_lpuart_state->p_tx_callback_param);
        }
    }
    else
    {
        /* Invoke callback if there is one */
        if (p_lpuart_state->tx_callback != NULL)
        {
            /* Allow the user to provide a new buffer, for continuous
             * transmission */
            p_lpuart_state->tx_callback(
                p_lpuart_state, UART_EVENT_TX_EMPTY, p_lpuart_state->p_tx_callback_param);
        }

        /* If the callback has updated the tx buffer, update the DMA descriptor
         * to continue the transfer; otherwise, stop the current transfer.
         */
        if (p_lpuart_state->tx_size > 0U)
        {
            /* Set the source address and the number of minor loops (bytes to be
             * transfered) */
            edma_set_src_addr(p_lpuart_state->tx_dma_channel, (uint32_t)(p_lpuart_state->txBuff));
            edma_set_major_loop_iteration_count(p_lpuart_state->tx_dma_channel,
                                                p_lpuart_state->tx_size);

            /* Now that this tx is set up, clear remaining bytes count */
            p_lpuart_state->tx_size = 0U;

            /* Re-start the channel */
            (void)edma_start_channel(p_lpuart_state->tx_dma_channel);
        }
        else
        {
            /* Enable transmission complete interrupt */
            lpuart_set_int_mode(p_base, LPUART_INT_TX_COMPLETE, true);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_start_receive_data_using_int
 * Description   : Initiate (start) a receive by beginning the process of
 * receiving data and enabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static status_t
lpuart_start_receive_data_using_int(uint32_t instance, uint8_t *p_rx_buff, uint32_t rx_size)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_rx_buff != NULL);

    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    LPUART_t       *p_base         = sp_lpuart_base[instance];

    /* Check it's not busy receiving data from a previous function call */
    if (p_lpuart_state->b_is_rx_busy)
    {
        return STATUS_BUSY;
    }

    /* Check the validity of the parameters */
    DEV_ASSERT(rx_size > 0U);
    DEV_ASSERT((p_lpuart_state->bit_count_per_char == LPUART_8_BITS_PER_CHAR) ||
               ((rx_size & 1U) == 0U));

    /* Initialize the module driver state struct to indicate transfer in
     * progress and with the buffer and byte count data. */
    p_lpuart_state->b_is_rx_busy   = true;
    p_lpuart_state->rxBuff         = p_rx_buff;
    p_lpuart_state->rx_size        = rx_size;
    p_lpuart_state->receive_status = STATUS_BUSY;

    /* Enable the receiver */
    lpuart_set_receiver_cmd(p_base, true);

    /* Enable error interrupts */
    lpuart_set_error_interrupts(p_base, true);

    /* Enable receive data full interrupt */
    lpuart_set_int_mode(p_base, LPUART_INT_RX_DATA_REG_FULL, true);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_start_receive_data_using_dma
 * Description   : Initiate (start) a receive by beginning the process of
 * receiving data using DMA transfers.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static status_t
lpuart_start_receive_data_using_dma(uint32_t instance, uint8_t *p_rx_buff, uint32_t rx_size)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
    DEV_ASSERT(p_rx_buff != NULL);

    LPUART_t       *p_base         = sp_lpuart_base[instance];
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Check it's not busy transmitting data from a previous function call */
    if (p_lpuart_state->b_is_rx_busy)
    {
        return STATUS_BUSY;
    }

    DEV_ASSERT(rx_size > 0U);

    /* Configure the transfer control descriptor for the previously allocated
     * channel */
    (void)edma_config_multi_block_transfer(p_lpuart_state->rx_dma_channel,
                                           EDMA_TRANSFER_PERIPH2MEM,
                                           (uint32_t)(&(p_base->DATA)),
                                           (uint32_t)p_rx_buff,
                                           EDMA_TRANSFER_SIZE_1B,
                                           1U,
                                           rx_size,
                                           true);

    /* Call driver function to end the reception when the DMA transfer is done
     */
    (void)edma_install_callback(p_lpuart_state->rx_dma_channel,
                                (edma_callback_t)(lpuart_rx_dma_callback),
                                (void *)(instance));

    /* Start the DMA channel */
    (void)edma_start_channel(p_lpuart_state->rx_dma_channel);

    /* Update the state structure */
    p_lpuart_state->rxBuff         = p_rx_buff;
    p_lpuart_state->rx_size        = 0U;
    p_lpuart_state->b_is_rx_busy   = true;
    p_lpuart_state->receive_status = STATUS_BUSY;

    /* Enable the receiver */
    lpuart_set_receiver_cmd(p_base, true);

    /* Enable error interrupts */
    lpuart_set_error_interrupts(p_base, true);

    /* Enable rx DMA requests for the current instance */
    lpuart_set_rx_dma_cmd(p_base, true);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_complete_receive_data_using_int
 * Description   : Finish up a receive by completing the process of receiving
 *data and disabling the interrupt. This is not a public API as it is called
 *from other driver functions.
 *
 *END**************************************************************************/
static void
lpuart_complete_receive_data_using_int(uint32_t instance)
{
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    uint8_t         tmp_byte;
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    LPUART_t       *p_base         = sp_lpuart_base[instance];

    /* Disable receiver */
    lpuart_set_receiver_cmd(p_base, false);

    /* Disable error interrupts */
    lpuart_set_error_interrupts(p_base, false);

    /* Read dummy to clear RDRF flag */
    lpuart_get_char(p_base, &tmp_byte);

    /* Disable receive data full and rx overrun interrupt. */
    lpuart_set_int_mode(p_base, LPUART_INT_RX_DATA_REG_FULL, false);

    /* Signal the synchronous completion object. */
    if (p_lpuart_state->b_is_rx_blocking)
    {
        (void)osif_sema_post(&p_lpuart_state->rx_complete);
        p_lpuart_state->b_is_rx_blocking = false;
    }

    /* Update the information of the module driver state */
    p_lpuart_state->b_is_rx_busy = false;
    if (p_lpuart_state->receive_status == STATUS_BUSY)
    {
        p_lpuart_state->receive_status = STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_rx_dma_callback
 * Description   : Finish up a receive by completing the process of receiving
 *data and disabling the DMA requests. This is a callback for DMA major loop
 * completion, so it must match the DMA callback signature.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void
lpuart_rx_dma_callback(void *p_parameter, edma_chn_status_t status)
{
    uint32_t        instance       = ((uint32_t)p_parameter);
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    LPUART_t       *p_base         = sp_lpuart_base[instance];

    if (status == EDMA_CHN_ERROR)
    {
        /* Update the status */
        p_lpuart_state->receive_status = STATUS_ERROR;
        /* Stop the transfer */
        lpuart_stop_rx_dma(instance);
        /* Notify the application that an error occurred */
        if (p_lpuart_state->rx_callback != NULL)
        {
            p_lpuart_state->rx_callback(
                p_lpuart_state, UART_EVENT_ERROR, p_lpuart_state->p_rx_callback_param);
        }

        /* Clear the flags */
        lpuart_clear_error_flags(p_base);
    }

    /* Return if an error occurred; error cases are treated by the interrupt
     * handler */
    if (p_lpuart_state->receive_status != STATUS_BUSY)
    {
        return;
    }

    /* Invoke callback if there is one */
    if (p_lpuart_state->rx_callback != NULL)
    {
        /* Allow the user to provide a new buffer inside the callback, to
         * continue the reception */
        p_lpuart_state->rx_callback(
            p_lpuart_state, UART_EVENT_RX_FULL, p_lpuart_state->p_rx_callback_param);
    }

    /* If the callback has updated the rx buffer, update the DMA descriptor to
     * continue the transfer; otherwise, stop the current transfer.
     */
    if (p_lpuart_state->rx_size > 0U)
    {
        /* Set the source address and the number of minor loops (bytes to be
         * transfered) */
        edma_set_dest_addr(p_lpuart_state->rx_dma_channel, (uint32_t)(p_lpuart_state->rxBuff));
        edma_set_major_loop_iteration_count(p_lpuart_state->rx_dma_channel,
                                            p_lpuart_state->rx_size);

        /* Now that this rx is set up, clear remaining bytes count */
        p_lpuart_state->rx_size = 0U;

        /* Re-start the channel */
        (void)edma_start_channel(p_lpuart_state->rx_dma_channel);
    }
    else
    {
        /* Stop the reception */
        lpuart_stop_rx_dma(instance);

        /* Invoke the callback to notify the end of the transfer */
        if (p_lpuart_state->rx_callback != NULL)
        {
            p_lpuart_state->rx_callback(
                p_lpuart_state, UART_EVENT_END_TRANSFER, p_lpuart_state->p_rx_callback_param);
        }

        /* Clear the flags */
        lpuart_clear_error_flags(p_base);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_put_data
 * Description   : Write data to the buffer register, according to configured
 * word length.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void
lpuart_put_data(uint32_t instance)
{
    const lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    LPUART_t             *p_base         = sp_lpuart_base[instance];
    uint16_t              data;
    const uint8_t        *p_tx_buff = p_lpuart_state->txBuff;

    if (p_lpuart_state->bit_count_per_char == LPUART_8_BITS_PER_CHAR)
    {
        /* Transmit the data */
        lpuart_put_char(p_base, *p_tx_buff);
    }
    else if (p_lpuart_state->bit_count_per_char == LPUART_9_BITS_PER_CHAR)
    {
        /* Create a 16-bits integer from two bytes */
        data = (uint16_t)(*p_tx_buff);
        ++p_tx_buff;
        data |= (uint16_t)(((uint16_t)(*p_tx_buff)) << 8U);

        /* Transmit the data */
        lpuart_put_char9(p_base, data);
    }
    else
    {
        /* Create a 16-bits integer from two bytes */
        data = (uint16_t)(*p_tx_buff);
        ++p_tx_buff;
        data |= (uint16_t)(((uint16_t)(*p_tx_buff)) << 8U);

        /* Transmit the data */
        lpuart_put_char10(p_base, data);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_get_data
 * Description   : Read data from the buffer register, according to configured
 * word length.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void
lpuart_get_data(uint32_t instance)
{
    const lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    const LPUART_t       *p_base         = sp_lpuart_base[instance];
    uint16_t              data;
    uint8_t              *p_rx_buff = p_lpuart_state->rxBuff;

    if (p_lpuart_state->bit_count_per_char == LPUART_8_BITS_PER_CHAR)
    {
        /* Receive the data */
        lpuart_get_char(p_base, p_rx_buff);
    }
    else if (p_lpuart_state->bit_count_per_char == LPUART_9_BITS_PER_CHAR)
    {
        /* Receive the data */
        lpuart_get_char9(p_base, &data);

        /* Write the least significant bits to the receive buffer */
        *p_rx_buff = (uint8_t)(data & 0xFFU);
        ++p_rx_buff;
        /* Write the ninth bit to the subsequent byte in the rx buffer */
        *p_rx_buff = (uint8_t)(data >> 8U);
    }
    else
    {
        /* Receive the data */
        lpuart_get_char10(p_base, &data);

        /* Write the least significant bits to the receive buffer */
        *p_rx_buff = (uint8_t)(data & 0xFFU);
        ++p_rx_buff;
        /* Write the ninth and tenth bits to the subsequent byte in the rx
         * buffer */
        *p_rx_buff = (uint8_t)(data >> 8U);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_stop_tx_dma
 * Description   : Finish up a DMA transmission by disabling the DMA requests,
 * transmission complete interrupt and tx logic. This function also resets the
 * internal driver state (busy flag/tx semaphore).
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void
lpuart_stop_tx_dma(uint32_t instance)
{
    LPUART_t       *p_base         = sp_lpuart_base[instance];
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];

    /* Disable tx DMA requests for the current instance */
    lpuart_set_tx_dma_cmd(p_base, false);

    /* Stop the dma channel */
    (void)edma_stop_channel(p_lpuart_state->tx_dma_channel);

    /* Disable transmission complete interrupt */
    lpuart_set_int_mode(p_base, LPUART_INT_TX_COMPLETE, false);

    /* Disable transmitter */
    lpuart_set_transmitter_cmd(p_base, false);

    /* Signal the synchronous completion object. */
    if (p_lpuart_state->b_is_tx_blocking)
    {
        (void)osif_sema_post(&p_lpuart_state->tx_complete);
    }

    if (p_lpuart_state->transmit_status == STATUS_BUSY)
    {
        /* If the transfer is completed, update the transmit status */
        p_lpuart_state->transmit_status = STATUS_SUCCESS;
    }

    /* Update the internal busy flag */
    p_lpuart_state->b_is_tx_busy = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_stop_rx_dma
 * Description   : Finish up a DMA reception by disabling the DMA requests,
 * error interrupts and rx logic. This function also resets the internal driver
 * state (busy flag/rx semaphore).
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void
lpuart_stop_rx_dma(uint32_t instance)
{
    LPUART_t       *p_base         = sp_lpuart_base[instance];
    lpuart_state_t *p_lpuart_state = (lpuart_state_t *)sp_lpuart_state_ptr[instance];
    uint8_t         tmp_byte;

    /* Disable receiver */
    lpuart_set_receiver_cmd(p_base, false);

    /* Disable error interrupts */
    lpuart_set_error_interrupts(p_base, false);

    /* Disable rx DMA requests for the current instance */
    lpuart_set_rx_dma_cmd(p_base, false);

    /* Read dummy to clear RDRF flag */
    lpuart_get_char(p_base, &tmp_byte);

    /* Stop the DMA channel */
    (void)edma_stop_channel(p_lpuart_state->rx_dma_channel);

    /* Signal the synchronous completion object. */
    if (p_lpuart_state->b_is_rx_blocking)
    {
        (void)osif_sema_post(&p_lpuart_state->rx_complete);
        p_lpuart_state->b_is_rx_blocking = false;
    }

    /* Update the internal driver status */
    if (p_lpuart_state->receive_status == STATUS_BUSY)
    {
        p_lpuart_state->receive_status = STATUS_SUCCESS;
    }

    /* Update the information of the module driver state */
    p_lpuart_state->b_is_rx_busy = false;
}

/*** end of file ***/
