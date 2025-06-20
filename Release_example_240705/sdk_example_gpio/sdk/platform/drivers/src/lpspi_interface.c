/**
 * @file lpspi_interface.c
 * @brief An application interface for SPI bus.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string.h>
#include "lpspi_interface.h"
#include "interrupt_manager.h"
#include "edma_driver.h"
#include "clock_tw9001.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/

/*******************************************************************************
 * Private typedefs
 ******************************************************************************/

/*******************************************************************************
 * Global variables
 ******************************************************************************/
lpspiif_state_t g_lpspiif_inst0_state = {0u};

const lpspiif_master_config_t g_lpspiif_inst0_master_config = {
    .bits_per_sec     = 200000UL,
    .which_pcs        = LPSPI_PCS0,
    .pcs_polarity     = LPSPI_ACTIVE_LOW,
    .b_pcs_continuous = false,
    .bits_per_frame   = 8U,
    .clk_phase        = LPSPI_CLOCK_PHASE_1ST_EDGE,
    .clk_polarity     = LPSPI_SCK_ACTIVE_HIGH,
    .b_lsb_first      = false,
    .b_byte_swap      = false,
    .xfer_type        = LPSPIIF_USING_INTERRUPTS,
    .rx_dma_channel   = 0U,
    .tx_dma_channel   = 0U,
    .callback         = NULL,
    .p_callback_param = NULL};

const lpspiif_slave_config_t g_lpspiif_inst0_slave_config = {.which_pcs      = LPSPI_PCS0,
                                                             .pcs_polarity   = LPSPI_ACTIVE_LOW,
                                                             .bits_per_frame = 8U,
                                                             .clk_phase =
                                                                 LPSPI_CLOCK_PHASE_1ST_EDGE,
                                                             .clk_polarity = LPSPI_SCK_ACTIVE_HIGH,
                                                             .b_lsb_first  = false,
                                                             .b_byte_swap  = false,
                                                             .xfer_type = LPSPIIF_USING_INTERRUPTS,
                                                             .rx_dma_channel   = 0U,
                                                             .tx_dma_channel   = 0U,
                                                             .callback         = NULL,
                                                             .p_callback_param = NULL};

lpspiif_state_t g_lpspiif_inst1_state = {0u};

const lpspiif_master_config_t g_lpspiif_inst1_master_config = {
    .bits_per_sec     = 200000UL,
    .which_pcs        = LPSPI_PCS0,
    .pcs_polarity     = LPSPI_ACTIVE_LOW,
    .b_pcs_continuous = false,
    .bits_per_frame   = 8U,
    .clk_phase        = LPSPI_CLOCK_PHASE_1ST_EDGE,
    .clk_polarity     = LPSPI_SCK_ACTIVE_HIGH,
    .b_lsb_first      = false,
    .b_byte_swap      = false,
    .xfer_type        = LPSPIIF_USING_INTERRUPTS,
    .rx_dma_channel   = 0U,
    .tx_dma_channel   = 0U,
    .callback         = NULL,
    .p_callback_param = NULL};

const lpspiif_slave_config_t g_lpspiif_inst1_slave_config = {.which_pcs      = LPSPI_PCS0,
                                                             .pcs_polarity   = LPSPI_ACTIVE_LOW,
                                                             .bits_per_frame = 8U,
                                                             .clk_phase =
                                                                 LPSPI_CLOCK_PHASE_1ST_EDGE,
                                                             .clk_polarity = LPSPI_SCK_ACTIVE_HIGH,
                                                             .b_lsb_first  = false,
                                                             .b_byte_swap  = false,
                                                             .xfer_type = LPSPIIF_USING_INTERRUPTS,
                                                             .rx_dma_channel   = 0U,
                                                             .tx_dma_channel   = 0U,
                                                             .callback         = NULL,
                                                             .p_callback_param = NULL};

lpspiif_state_t g_lpspiif_inst2_state = {0u};

const lpspiif_master_config_t g_lpspiif_inst2_master_config = {
    .bits_per_sec     = 200000UL,
    .which_pcs        = LPSPI_PCS0,
    .pcs_polarity     = LPSPI_ACTIVE_LOW,
    .b_pcs_continuous = false,
    .bits_per_frame   = 8U,
    .clk_phase        = LPSPI_CLOCK_PHASE_1ST_EDGE,
    .clk_polarity     = LPSPI_SCK_ACTIVE_HIGH,
    .b_lsb_first      = false,
    .b_byte_swap      = false,
    .xfer_type        = LPSPIIF_USING_INTERRUPTS,
    .rx_dma_channel   = 0U,
    .tx_dma_channel   = 0U,
    .callback         = NULL,
    .p_callback_param = NULL};

const lpspiif_slave_config_t g_lpspiif_inst2_slave_config = {.which_pcs      = LPSPI_PCS0,
                                                             .pcs_polarity   = LPSPI_ACTIVE_LOW,
                                                             .bits_per_frame = 8U,
                                                             .clk_phase =
                                                                 LPSPI_CLOCK_PHASE_1ST_EDGE,
                                                             .clk_polarity = LPSPI_SCK_ACTIVE_HIGH,
                                                             .b_lsb_first  = false,
                                                             .b_byte_swap  = false,
                                                             .xfer_type = LPSPIIF_USING_INTERRUPTS,
                                                             .rx_dma_channel   = 0U,
                                                             .tx_dma_channel   = 0U,
                                                             .callback         = NULL,
                                                             .p_callback_param = NULL};

/*******************************************************************************
 * Static variables
 ******************************************************************************/
/* Pointer to runtime state structure.*/
static lpspiif_state_t *sp_lpspiif_state_ptr[LPSPI_INSTANCE_COUNT] =
    FEATURE_LPSPI_STATE_STRUCTURES_NULL;

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/
static void lpspiif_isr(uint32_t instance);

/*!
 * @brief Fill up the TX FIFO with data.
 * This function fills up the TX FIFO with data based on the bytes/frame.
 * This is not a public API as it is called from other driver functions.
 */
static void lpspiif_fillup_tx_buffer(uint32_t instance);

/*!
 * @brief Read all data from RX FIFO
 * This function will read all data from RX FIFO and will transfer this
 * infromation in RX software bufeer. This is not a public API as it is called
 * from other driver functions.
 */
static void lpspiif_read_rx_buffer(uint32_t instance);

/*!
 * @brief Disable the TEIE interrupts at the end of a transfer.
 * Disable the interrupts and clear the status for transmit/receive errors.
 */
static void lpspiif_disable_teie_intrpts(uint32_t instance);

/*FUNCTION**********************************************************************
 *
 * Function Name : lpspiif_set_pcs
 * Description   : Select the chip to communicate with.
 *
 *
 * The main purpose of this function is to set the PCS and the appropriate
 *polarity. Implements : LPSPI_DRV_SetPcs_Activity
 *END**************************************************************************/
static status_t lpspiif_set_pcs(uint32_t                instance,
                                lpspi_which_pcs_t       which_pcs,
                                lpspi_signal_polarity_t polarity);

/*!
 * @brief Interrupt handler for LPSPI master mode.
 * This handler uses the buffers stored in the lpspiif_state_t structs to transfer
 * data. This is not a public API as it is called whenever an interrupt occurs.
 */
static void lpspiif_master_isr(uint32_t instance);

/*FUNCTION**********************************************************************
 *
 * Function Name : lpspiif_master_set_delay
 * Description   : Configures the LPSPI master mode bus timing delay options.
 *
 * This function involves the LPSPI module's delay options to
 * "fine tune" some of the signal timings and match the timing needs of a slower
 *peripheral device. This is an optional function that can be called after the
 *LPSPI module has been initialized for master mode. The timings are adjusted in
 *terms of cycles of the baud rate clock. The bus timing delays that can be
 *adjusted are listed below:
 *
 * SCK to PCS Delay: Adjustable delay option between the last edge of SCK to the
 *de-assertion of the PCS signal.
 *
 * PCS to SCK Delay: Adjustable delay option between the assertion of the PCS
 *signal to the first SCK edge.
 *
 * Delay between Transfers: Adjustable delay option between the de-assertion of
 *the PCS signal for a frame to the assertion of the PCS signal for the next
 *frame. Implements : LPSPI_DRV_MasterSetDelay_Activity
 *
 *END**************************************************************************/
static status_t lpspiif_master_set_delay(uint32_t instance,
                                         uint32_t delay_betwen_xfer,
                                         uint32_t delay_sck_to_pcs,
                                         uint32_t delay_pcs_to_sck);

/*FUNCTION**********************************************************************
 *
 * Function Name : lpspiif_master_configure_bus
 * Description   : Configures the LPSPI port physical parameters to access a
 *device on the bus when the LSPI instance is configured for interrupt
 *operation.
 *
 * In this function, the term "p_init_config" is used to indicate the SPI device for
 *which the LPSPI master is communicating. This is an optional function as the
 *p_init_config parameters are normally configured in the initialization function or
 *the transfer functions, where these various functions would call the configure
 *bus function. The user can pass in a different p_init_config structure to the
 *transfer function which contains the parameters for the SPI bus to allow for
 *communication to a different SPI device (the transfer function then calls this
 *function). However, the user also has the option to call this function
 *directly especially to get the calculated baud rate, at which point they may
 *pass in NULL for the p_init_config structure in the transfer function (assuming
 *they have called this configure bus function first). Implements :
 *LPSPI_DRV_MasterConfigureBus_Activity
 *
 *END**************************************************************************/
static status_t lpspiif_master_configure_bus(uint32_t                       instance,
                                             const lpspiif_master_config_t *p_init_config,
                                             uint32_t                      *p_calc_baud_rate);

/*FUNCTION**********************************************************************
 *
 * Function Name : lpspiif_master_abort_xfer
 * Description   : Terminates an interrupt driven asynchronous transfer early.
 *
 * During an a-sync (non-blocking) transfer, the user has the option to
 *terminate the transfer early if the transfer is still in progress. Implements
 *: LPSPI_DRV_MasterAbortTransfer_Activity
 *END**************************************************************************/
static status_t lpspiif_master_abort_xfer(uint32_t instance);

/*FUNCTION**********************************************************************
 *
 * Function Name : lpspiif_master_start_xfer
 * Description   : Configure a non-blocking transfer.
 *
 * The number of xfer_byte_count must be divided by number of bytes/frame.
 * The p_send_buffer must be not NULL, but p_rcv_buffer can be NULL.
 *
 *END**************************************************************************/
static status_t lpspiif_master_start_xfer(uint32_t       instance,
                                          const uint8_t *p_send_buffer,
                                          uint8_t       *p_rcv_buffer,
                                          uint16_t       xfer_byte_count);

/*!
 * @brief Finish up a transfer.
 * Cleans up after a transfer is complete. Interrupts are disabled, and the
 * LPSPI module is disabled. This is not a public API as it is called from other
 * driver functions.
 */
static void lpspiif_master_complete_xfer(uint32_t instance);

/*!
 * @brief Finish up a transfer DMA.
 * The main purpose of this function is to create a function compatible with DMA
 * callback type
 */
static void lpspiif_master_complete_dma_xfer(void *parameter, edma_chn_status_t status);

/*!
 * @brief Check if errors are detected on RX channel
 * The main purpose of this function is to check DMA errors on rx channel
 */
static void lpspiif_master_complete_rx(void *parameter, edma_chn_status_t status);

static void lpspiif_slave_isr(uint32_t instance);

/*
 * Implements : LPSPI_DRV_SlaveAbortTransfer_Activity
 */
static status_t lpspiif_slave_abort_xfer(uint32_t instance);

/*!
 * @brief Finish up a transfer DMA.
 * The main purpose of this function is to create a function compatible with DMA
 * callback type
 */
static void lpspiif_slave_complete_dma_xfer(void *parameter, edma_chn_status_t status);

/*******************************************************************************
 * Global functions
 ******************************************************************************/
/*!
 * @brief This function is the implementation of LPSPI0 handler named in startup
 * code.
 *
 * It passes the instance to the shared LPSPI IRQ handler.
 */
void
LPSPI0_IRQHandler(void)
{
    lpspiif_isr(INST_LPSPI0);
}

/*!
 * @brief This function is the implementation of LPSPI1 handler named in startup
 * code.
 *
 * It passes the instance to the shared LPSPI IRQ handler.
 */
void
LPSPI1_IRQHandler(void)
{
    lpspiif_isr(INST_LPSPI1);
}

/*!
 * @brief This function is the implementation of LPSPI1 handler named in startup
 * code.
 *
 * It passes the instance to the shared LPSPI IRQ handler.
 */
void
LPSPI2_IRQHandler(void)
{
    lpspiif_isr(INST_LPSPI2);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpspiif_master_init
 * Description   : Initializes a LPSPI instance for interrupt driven master mode
 *operation.
 *
 * This function uses an interrupt-driven method for transferring data.
 * In this function, the term "p_init_config" is used to indicate the SPI device for
 *which the LPSPI master is communicating. This function initializes the
 *run-time state structure to track the ongoing transfers, un-gates the clock to
 *the LPSPI module, resets the LPSPI module, configures the IRQ state structure,
 *enables the module-level interrupt to the core, and enables the LPSPI module.
 * Implements : LPSPI_DRV_MasterInit_Activity
 *
 *END**************************************************************************/
status_t
lpspiif_master_init(uint32_t                       instance,
                    lpspiif_state_t               *p_state_ptr,
                    const lpspiif_master_config_t *p_init_config)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(p_state_ptr != NULL);
    DEV_ASSERT(p_init_config != NULL);
    LPSPI_t *p_base       = NULL;
    status_t sts_err_code = STATUS_SUCCESS;
    uint32_t source_clk_freq;

    /* Clear the state struct for this instance. */
    memset((void *)p_state_ptr, 0x00u, sizeof(lpspiif_state_t));
    /* Save runtime structure pointers so irq handler can point to the correct
     * state structure */
    sp_lpspiif_state_ptr[instance] = p_state_ptr;
    p_state_ptr->p_base = p_base = gp_lpspi_base[instance];
    p_state_ptr->p_irq_id        = &g_lpspi_irq_id[instance];
    p_state_ptr->p_clk_name      = &g_lpspi_clk_names[instance];

    /* Get the LPSPI clock as configured in the clock manager */
    (void)clock_get_freq(*(p_state_ptr->p_clk_name), &source_clk_freq);
    DEV_ASSERT(0u < source_clk_freq);

    /* Reset the LPSPI registers to their default state */
    lpspi_init(p_base);
    /* Set for master mode */
    (void)lpspi_set_master_slave_mode(p_base, LPSPI_MASTER);
    /* Set Pin configuration such that SDO=out and SDI=in */
    (void)lpspi_set_pin_config_mode(p_base, LPSPI_SDI_IN_SDO_OUT, LPSPI_DATA_OUT_RETAINED, true);
    /* Calculate the FIFO size for the LPSPI */
    lpspi_get_fifo_sizes(p_base, &(p_state_ptr->fifo_size));

    /* Configure bus for this device. If NULL is passed, we assume the caller
     * has preconfigured the bus and doesn't wish to re-configure it again for
     * this transfer. Do nothing for p_calc_baud_rate. If the user wants to
     * know the p_calc_baud_rate then they can call this function separately.
     */
    sts_err_code = lpspiif_master_configure_bus(instance, p_init_config, NULL);
    if (sts_err_code != STATUS_SUCCESS)
    {
        return sts_err_code;
    }
    /* When TX is null the value sent on the bus will be 0 */
    p_state_ptr->dummy = 0;
    /* Initialize the semaphore */
    sts_err_code = osif_sema_create(&(p_state_ptr->sema_blocking_xfer), 0);
    DEV_ASSERT(sts_err_code == STATUS_SUCCESS);
    /* Enable the interrupt */
    int_enable_irq(*(p_state_ptr->p_irq_id));
    /* Finally, enable LPSPI */
    lpspi_enable(p_base);
    return sts_err_code;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpspiif_master_deinit
 * Description   : Shuts down a LPSPI instance.
 *
 * This function resets the LPSPI peripheral, gates its clock, and disables the
 *interrupt to the core.  It first checks to see if a transfer is in progress
 *and if so returns an error status. Implements :
 *LPSPI_DRV_MasterDeinit_Activity
 *
 *END**************************************************************************/
status_t
lpspiif_master_deinit(uint32_t instance)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    /* Instantiate local variable of type lpspiif_state_t and point to global
     * state */
    const lpspiif_state_t *p_state_ptr  = sp_lpspiif_state_ptr[instance];
    LPSPI_t               *p_base       = gp_lpspi_base[instance];
    status_t               sts_err_code = STATUS_SUCCESS;

    /* Check if a transfer is still in progress */
    DEV_ASSERT(p_state_ptr->b_xfer_in_progress == false);

    /* Reset the LPSPI registers to their default state, inlcuding disabling the
     * LPSPI */
    lpspi_init(p_base);
    /* Disable the interrupt */
    int_disable_irq(*(p_state_ptr->p_irq_id));
    /* Clear the state pointer. */
    sp_lpspiif_state_ptr[instance] = NULL;

    /* Destroy the semaphore */
    sts_err_code = osif_sema_destroy(&(p_state_ptr->sema_blocking_xfer));
    DEV_ASSERT(sts_err_code == STATUS_SUCCESS);
    return sts_err_code;
}

status_t
lpspiif_master_xfer_blocking(uint32_t       instance,
                             const uint8_t *p_send_buffer,
                             uint8_t       *p_rcv_buffer,
                             uint16_t       xfer_byte_count,
                             uint32_t       timeout)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    /* Instantiate local variable of type lpspiif_state_t and point to global
     * state */
    lpspiif_state_t *p_state_ptr  = sp_lpspiif_state_ptr[instance];
    LPSPI_t         *p_base       = gp_lpspi_base[instance];
    status_t         sts_err_code = STATUS_SUCCESS;
    status_t         sts_for_sema;
    /* If the transfer count is zero, then return immediately.*/
    if (xfer_byte_count == (uint16_t)0)
    {
        return sts_err_code;
    }

    /* Check if another transfer is in progress */
    if (lpspi_get_status_flag(p_base, LPSPI_MODULE_BUSY))
    {
        return STATUS_BUSY;
    }

    /* Dummy wait to ensure the semaphore is 0, no need to check result */
    (void)osif_sema_wait(&(p_state_ptr->sema_blocking_xfer), 0);
    p_state_ptr->b_blocking = true;

    sts_err_code =
        lpspiif_master_start_xfer(instance, p_send_buffer, p_rcv_buffer, xfer_byte_count);
    /* Start the transfer process, if it returns an error code, return this back
     * to user */
    if (sts_err_code != STATUS_SUCCESS)
    {
        /* Disable interrupt requests*/
        lpspi_set_int_mode(p_base, LPSPI_TX_DATA_FLAG, false);
        lpspi_set_int_mode(p_base, LPSPI_RX_DATA_FLAG, false);

        lpspiif_disable_teie_intrpts(instance);
        lpspi_set_int_mode(p_base, LPSPI_TRANSFER_COMPLETE, false);
        (void)lpspi_clear_status_flag(p_base, LPSPI_TRANSFER_COMPLETE);

        p_state_ptr->b_blocking = false;
        return sts_err_code;
    }

    /* As this is a synchronous transfer, wait until the transfer is complete.*/
    sts_for_sema = osif_sema_wait(&(p_state_ptr->sema_blocking_xfer), timeout);

    /* If a timeout occurs, stop the transfer by setting the
     * b_xfer_in_progress to false and disabling interrupts, then return the
     * timeout error status.
     */
    if (sts_for_sema == STATUS_TIMEOUT)
    {
        /* Set b_blocking variable to false to avoid dummy semaphore post. */
        p_state_ptr->b_blocking = false;
        /* Complete transfer. */
        lpspiif_master_complete_xfer(instance);
        return (STATUS_TIMEOUT);
    }

    lpspiif_disable_teie_intrpts(instance);
    lpspi_set_int_mode(p_base, LPSPI_TRANSFER_COMPLETE, false);
    (void)lpspi_clear_status_flag(p_base, LPSPI_TRANSFER_COMPLETE);

    return sts_err_code;
}

status_t
lpspiif_master_xfer(uint32_t       instance,
                    const uint8_t *p_send_buffer,
                    uint8_t       *p_rcv_buffer,
                    uint16_t       xfer_byte_count)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    status_t sts_err_code = STATUS_SUCCESS;
    /* If the transfer count is zero, then return immediately.*/
    if (xfer_byte_count == (uint16_t)0)
    {
        return STATUS_SUCCESS;
    }

    /* Start the transfer process, if it returns an error code, return this back
     * to user */
    sts_err_code =
        lpspiif_master_start_xfer(instance, p_send_buffer, p_rcv_buffer, xfer_byte_count);
    if (sts_err_code != STATUS_SUCCESS)
    {
        return sts_err_code;
    }

    /* Else, return immediately as this is an async transfer */
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpspiif_master_get_xfer_status
 * Description   : Returns whether the previous interrupt driven transfer is
 *completed.
 *
 * When performing an a-sync (non-blocking) transfer, the user can call this
 *function to ascertain the state of the current transfer: in progress (or busy)
 *or complete (success). In addition, if the transfer is still in progress, the
 *user can get the number of words that should be receive. Implements :
 *LPSPI_DRV_MasterGetTransferStatus_Activity
 *
 *END**************************************************************************/
status_t
lpspiif_master_get_xfer_status(uint32_t instance, uint32_t *p_remain_byte_count)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    /* Instantiate local variable of type lpspiif_state_t and point to global
     * state */
    const lpspiif_state_t *p_state_ptr = sp_lpspiif_state_ptr[instance];
    /* Fill in the bytes transferred.*/
    if (p_remain_byte_count != NULL)
    {
        *p_remain_byte_count = p_state_ptr->rx_remain_cntr;
    }
    if (p_state_ptr->xfer_status == LPSPIIF_TRANSFER_OK)
    {
        return (status_t)(p_state_ptr->b_xfer_in_progress ? STATUS_BUSY : STATUS_SUCCESS);
    }
    else
    {
        return STATUS_ERROR;
    }
}

status_t
lpspiif_slave_init(uint32_t                      instance,
                   lpspiif_state_t              *p_state_ptr,
                   const lpspiif_slave_config_t *p_init_config)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(p_state_ptr != NULL);
    DEV_ASSERT(p_init_config != NULL);
    LPSPI_t *p_base       = NULL;
    status_t sts_err_code = STATUS_SUCCESS;

    /* Clear the state struct for this instance. */
    memset((void *)p_state_ptr, 0x00u, sizeof(lpspiif_state_t));
    p_state_ptr->p_base = p_base = gp_lpspi_base[instance];
    p_state_ptr->p_irq_id        = &g_lpspi_irq_id[instance];
    p_state_ptr->p_clk_name      = &g_lpspi_clk_names[instance];

    p_state_ptr->b_lsb          = p_init_config->b_lsb_first;
    p_state_ptr->bits_per_frame = p_init_config->bits_per_frame;
    p_state_ptr->xfer_type      = p_init_config->xfer_type;
    p_state_ptr->b_blocking     = false;
    /* Store DMA channels numbers used for DMA transfer */
    p_state_ptr->rx_dma_channel = p_init_config->rx_dma_channel;
    p_state_ptr->tx_dma_channel = p_init_config->tx_dma_channel;
    /* Store callback */
    p_state_ptr->callback         = p_init_config->callback;
    p_state_ptr->p_callback_param = p_init_config->p_callback_param;
    /* Calculate the bytes/frame for p_state_ptr->bytes_per_frame. */
    p_state_ptr->bytes_per_frame = (uint16_t)((p_state_ptr->bits_per_frame + 7U) / 8U);
    /* Due to DMA limitations frames of 3 bytes/frame will be internally handled
     * as 4 bytes/frame. */
    if (p_state_ptr->bytes_per_frame == 3U)
    {
        p_state_ptr->bytes_per_frame = 4U;
    }
    /* Due to some limitations all frames bigger than 4 bytes/frame must be
     * composed only from 4 bytes chunks. */
    if (p_state_ptr->bytes_per_frame > 4U)
    {
        p_state_ptr->bytes_per_frame = (((p_state_ptr->bytes_per_frame - 1U) / 4U) + 1U) * 4U;
    }
    p_state_ptr->b_xfer_in_progress = false;
    /* Initialize the semaphore */
    sts_err_code = osif_sema_create(&(p_state_ptr->sema_blocking_xfer), 0);
    DEV_ASSERT(sts_err_code == STATUS_SUCCESS);
    /* Save runtime structure pointers so irq handler can point to the correct
     * state structure */
    sp_lpspiif_state_ptr[instance] = p_state_ptr;

    /* Configure registers */
    lpspi_init(p_base);

    /* Configure lpspi to slave mode */
    (void)lpspi_set_master_slave_mode(p_base, LPSPI_SLAVE);
    /* Set Pin settings */
    (void)lpspi_set_pin_config_mode(p_base, LPSPI_SDI_IN_SDO_OUT, LPSPI_DATA_OUT_RETAINED, true);
    /* Calculate the FIFO size for the LPSPI */
    lpspi_get_fifo_sizes(p_base, &(p_state_ptr->fifo_size));

    /* Set polarity */
    (void)lpspi_set_pcs_polarity_mode(
        p_base, p_init_config->which_pcs, p_init_config->pcs_polarity);

    /* Write the TCR for this transfer */
    lpspi_tx_cmd_config_t tx_cmd_cfg = {.frame_size   = p_state_ptr->bits_per_frame,
                                        .width        = LPSPI_SINGLE_BIT_XFER,
                                        .b_tx_mask    = false,
                                        .b_rx_mask    = false,
                                        .b_byte_swap  = p_init_config->b_byte_swap,
                                        .b_lsb_first  = p_init_config->b_lsb_first,
                                        .clk_phase    = p_init_config->clk_phase,
                                        .clk_polarity = p_init_config->clk_polarity,
                                        .which_pcs    = p_init_config->which_pcs};

    /* Write to the TX CMD register */
    lpspi_set_tx_cmd_reg_tcr(p_base, &tx_cmd_cfg);
    lpspi_enable(p_base);
    /* Enable the interrupt source */
    int_enable_irq(*(p_state_ptr->p_irq_id));

    return sts_err_code;
}

/*
 * Implements : LPSPI_DRV_SlaveDeinit_Activity
 */
status_t
lpspiif_slave_deinit(uint32_t instance)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    /* Instantiate local variable of type lpspi_master_state_t and point to
     * global state */
    const lpspiif_state_t *p_state_ptr  = (lpspiif_state_t *)sp_lpspiif_state_ptr[instance];
    LPSPI_t               *p_base       = gp_lpspi_base[instance];
    status_t               sts_err_code = STATUS_SUCCESS;

    /* Check if a transfer is still in progress */
    DEV_ASSERT(p_state_ptr->b_xfer_in_progress == false);
    /* Destroy the semaphore */
    sts_err_code = osif_sema_destroy(&(p_state_ptr->sema_blocking_xfer));
    DEV_ASSERT(sts_err_code == STATUS_SUCCESS);
    /* Reset the LPSPI registers to their default state, including disabling the
     * LPSPI */
    lpspi_init(p_base);

    /* Disable the interrupt*/
    int_disable_irq(*(p_state_ptr->p_irq_id));

    /* Clear the state pointer. */
    sp_lpspiif_state_ptr[instance] = NULL;

    return sts_err_code;
}

/*
 * Implements : LPSPI_DRV_SlaveTransferBlocking_Activity
 */
status_t
lpspiif_slave_xfer_blocking(uint32_t       instance,
                            const uint8_t *p_send_buffer,
                            uint8_t       *p_rcv_buffer,
                            uint16_t       xfer_byte_count,
                            uint32_t       timeout)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    lpspiif_state_t *p_state_ptr = (lpspiif_state_t *)sp_lpspiif_state_ptr[instance];
    const LPSPI_t   *p_base      = gp_lpspi_base[instance];
    status_t         sts_err_code;
    status_t         sts_for_sema;

    /* Check if another transfer is in progress */
    if (lpspi_get_status_flag(p_base, LPSPI_MODULE_BUSY))
    {
        return STATUS_BUSY;
    }

    /* Dummy wait to ensure the semaphore is 0, no need to check result */
    (void)osif_sema_wait(&(p_state_ptr->sema_blocking_xfer), 0);
    p_state_ptr->b_blocking = true;

    sts_err_code = lpspiif_slave_xfer(instance, p_send_buffer, p_rcv_buffer, xfer_byte_count);
    if (sts_err_code != STATUS_SUCCESS)
    {
        p_state_ptr->b_blocking = false;
        lpspiif_disable_teie_intrpts(instance);
        return sts_err_code;
    }
    /* As this is a synchronous transfer, wait until the transfer is complete.*/
    sts_for_sema = osif_sema_wait(&(p_state_ptr->sema_blocking_xfer), timeout);

    if (sts_for_sema == STATUS_TIMEOUT)
    {
        /* Set b_blocking variable to false to avoid dummy semaphore post. */
        p_state_ptr->b_blocking = false;
        /* Complete transfer. */
        (void)lpspiif_slave_abort_xfer(instance);
        return (STATUS_TIMEOUT);
    }

    lpspiif_disable_teie_intrpts(instance);

    return STATUS_SUCCESS;
}

/*
 * Implements : LPSPI_DRV_SlaveTransfer_Activity
 */
status_t
lpspiif_slave_xfer(uint32_t       instance,
                   const uint8_t *p_send_buffer,
                   uint8_t       *p_rcv_buffer,
                   uint16_t       xfer_byte_count)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    DEV_ASSERT(!((p_send_buffer == NULL) && (p_rcv_buffer == NULL)));
    LPSPI_t         *p_base                 = gp_lpspi_base[instance];
    lpspiif_state_t *p_state_ptr            = (lpspiif_state_t *)sp_lpspiif_state_ptr[instance];
    p_state_ptr->dummy                      = 0xFFU;
    edma_transfer_size_t dma_xfer_unit_size = EDMA_TRANSFER_SIZE_1B;
    const uint8_t       *buffer;

    /* The number of transferred bytes should be divisible by frame size */
    if ((uint16_t)(xfer_byte_count % p_state_ptr->bytes_per_frame) != (uint16_t)0)
    {
        return STATUS_ERROR;
    }
    /* Check if LPSPI module isn't busy */
    if (p_state_ptr->b_xfer_in_progress == true)
    {
        return STATUS_BUSY;
    }
    /* Initialize the status of the current transfer */
    p_state_ptr->xfer_status = LPSPIIF_TRANSFER_OK;
    /* Clean RX and TX buffers */
    lpspi_set_flush_fifo_cmd(p_base, true, true);
    /* The second flush command is used to avoid the case when one word is still
     * in shifter. */
    lpspi_set_flush_fifo_cmd(p_base, true, true);
    /* Clear all interrupts sources */
    (void)lpspi_clear_status_flag(p_base, LPSPI_ALL_STATUS);
    /* Enable fault interrupts sources */
    lpspi_set_int_mode(p_base, LPSPI_TRANSMIT_ERROR, true);
    lpspi_set_int_mode(p_base, LPSPI_RECEIVE_ERROR, true);
    /* Fill out the other members of the run-time state structure. */
    p_state_ptr->p_tx_buffer = (const uint8_t *)p_send_buffer;
    p_state_ptr->p_rx_buffer = (uint8_t *)p_rcv_buffer;
    if (p_state_ptr->xfer_type == LPSPIIF_USING_INTERRUPTS)
    {

        if (p_state_ptr->p_tx_buffer != NULL)
        {
            p_state_ptr->tx_remain_cntr = xfer_byte_count;
            lpspi_clear_tx_mask_bit(p_base);
        }
        else
        {
            p_state_ptr->tx_remain_cntr = 0;
            lpspi_set_tx_mask_bit(p_base);
        }

        if (p_state_ptr->p_rx_buffer != NULL)
        {
            p_state_ptr->rx_remain_cntr = xfer_byte_count;
            lpspi_clear_rx_mask_bit(p_base);
        }
        else
        {
            p_state_ptr->rx_remain_cntr = 0;
            lpspi_set_rx_mask_bit(p_base);
        }

        p_state_ptr->tx_frame_cntr    = 0;
        p_state_ptr->rx_frame_cntr    = 0;
        p_state_ptr->b_pcs_continuous = false;
        /* Configure watermarks */
        lpspi_set_rx_watermarks(p_base, 0U);
        lpspi_set_tx_watermarks(p_base, 2U);

        p_state_ptr->b_xfer_in_progress = true;
        /* Enable interrupts for RX and TX only if it's necessary */
        if (p_state_ptr->p_tx_buffer != NULL)
        {
            lpspi_set_int_mode(p_base, LPSPI_TX_DATA_FLAG, true);
        }
        if (p_state_ptr->p_rx_buffer != NULL)
        {
            lpspi_set_int_mode(p_base, LPSPI_RX_DATA_FLAG, true);
        }
    }
    else
    {
        /* Configure watermarks */
        lpspi_set_rx_watermarks(p_base, 0U);
        lpspi_set_tx_watermarks(p_base, 3U);
        /* When LPSPI use DMA frames with 3 bytes size are not accepted. */
        switch (p_state_ptr->bytes_per_frame)
        {
            case 1:
                dma_xfer_unit_size = EDMA_TRANSFER_SIZE_1B;
                break;
            case 2:
                dma_xfer_unit_size = EDMA_TRANSFER_SIZE_2B;
                break;
            case 4:
                dma_xfer_unit_size = EDMA_TRANSFER_SIZE_4B;
                break;
            default:
                dma_xfer_unit_size = EDMA_TRANSFER_SIZE_4B;
                break;
        }

        if (p_rcv_buffer != NULL)
        {
            p_state_ptr->rx_remain_cntr = xfer_byte_count;
            buffer                      = p_rcv_buffer;
        }
        else
        {
            p_state_ptr->rx_remain_cntr = 0U;
            /* if there is no data to receive, use dummy data as destination for
             * DMA transfer */
            buffer = (uint8_t *)(&(p_state_ptr->dummy));
        }
        (void)edma_config_multi_block_transfer(
            p_state_ptr->rx_dma_channel,
            EDMA_TRANSFER_PERIPH2MEM,
            (uint32_t)(&(p_base->RDR)),
            (uint32_t)buffer,
            dma_xfer_unit_size,
            (uint32_t)1U << (uint8_t)(dma_xfer_unit_size),
            (uint32_t)xfer_byte_count / (uint32_t)((uint32_t)1U << (uint8_t)(dma_xfer_unit_size)),
            true);
        if (p_rcv_buffer == NULL)
        {
            /* if there is no data to receive, don't increment destination
             * offset */
            edma_set_dest_offset(p_state_ptr->rx_dma_channel, 0);
        }

        if (p_send_buffer != NULL)
        {
            p_state_ptr->tx_remain_cntr = xfer_byte_count;
            buffer                      = p_send_buffer;
        }
        else
        {
            p_state_ptr->tx_remain_cntr = 0U;
            /* if there is no data to send, use dummy data as source for DMA
             * transfer */
            buffer = (uint8_t *)(&(p_state_ptr->dummy));
        }
        (void)edma_config_multi_block_transfer(
            p_state_ptr->tx_dma_channel,
            EDMA_TRANSFER_MEM2PERIPH,
            (uint32_t)buffer,
            (uint32_t)(&(p_base->TDR)),
            dma_xfer_unit_size,
            (uint32_t)1U << (uint8_t)(dma_xfer_unit_size),
            (uint32_t)xfer_byte_count / (uint32_t)((uint32_t)1U << (uint8_t)(dma_xfer_unit_size)),
            true);
        if (p_send_buffer == NULL)
        {
            /* if there is no data to transmit, don't increment source offset */
            edma_set_src_offset(p_state_ptr->tx_dma_channel, 0);
        }

        (void)edma_install_callback(
            p_state_ptr->rx_dma_channel, (lpspiif_slave_complete_dma_xfer), (void *)(instance));

        p_state_ptr->b_xfer_in_progress = true;

        /* Start RX channel */
        (void)edma_start_channel(p_state_ptr->rx_dma_channel);
        /* Start TX channel */
        (void)edma_start_channel(p_state_ptr->tx_dma_channel);
        /* Enable LPSPI DMA requests */
        lpspi_set_rx_dma_cmd(p_base, true);
        lpspi_set_tx_dma_cmd(p_base, true);
    }
    return STATUS_SUCCESS;
}

/*
 * Implements : LPSPI_DRV_SlaveGetTransferStatus_Activity
 */
status_t
lpspiif_slave_get_xfer_status(uint32_t instance, uint32_t *p_remain_byte_count)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    const lpspiif_state_t *p_state_ptr = (lpspiif_state_t *)sp_lpspiif_state_ptr[instance];

    /* Fill in the bytes transferred.*/
    if (p_remain_byte_count != NULL)
    {
        *p_remain_byte_count = p_state_ptr->tx_remain_cntr;
    }
    if (p_state_ptr->xfer_status == LPSPIIF_TRANSFER_OK)
    {
        return (status_t)(p_state_ptr->b_xfer_in_progress ? STATUS_BUSY : STATUS_SUCCESS);
    }
    else
    {
        return STATUS_ERROR;
    }
}

/*
 * Implements : lpspiif_slave_break_xfer_off
 */
status_t
lpspiif_slave_break_xfer_off(uint32_t instance)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    const lpspiif_state_t *p_state_ptr = (lpspiif_state_t *)sp_lpspiif_state_ptr[instance];

    /* Check if a transfer is still in progress */
    if (p_state_ptr->b_xfer_in_progress == true)
    {
        lpspiif_slave_abort_xfer(instance);
    }
    return STATUS_SUCCESS;
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/
static void
lpspiif_isr(uint32_t instance)
{
    if (instance < LPSPI_INSTANCE_COUNT)
    {
        const LPSPI_t *p_base = gp_lpspi_base[instance];

        if (lpspi_is_master(p_base))
        {
            /* Master mode.*/
            lpspiif_master_isr(instance);
        }
        else
        {
            /* Slave mode.*/
            lpspiif_slave_isr(instance);
        }
    }
}

static void
lpspiif_fillup_tx_buffer(uint32_t instance)
{
    /* Instantiate local variable of type dspi_master_state_t and point to
     * global state. */
    lpspiif_state_t *p_state_ptr  = sp_lpspiif_state_ptr[instance];
    LPSPI_t         *p_base       = gp_lpspi_base[instance];
    uint32_t         word_to_send = 0;
    uint16_t         num_bytes;
    uint8_t          available_space =
        (uint8_t)(p_state_ptr->fifo_size - (uint8_t)lpspi_read_tx_count(p_base));

    /* Fill the TX buffer. */
    while (available_space != 0U)
    {
        if (p_state_ptr->b_pcs_continuous == true)
        {
            if (p_state_ptr->tx_remain_cntr == 1U)
            {
                /* Disable continuous PCS */
                lpspi_clear_cont_cmd_bit(p_base);
                p_state_ptr->tx_remain_cntr = 0U;
                break;
            }
        }
        /* Get the number of bytes which can be written in a single 32 bits
         * word. */
        if ((p_state_ptr->bytes_per_frame - p_state_ptr->tx_frame_cntr) <= (uint16_t)4)
        {
            num_bytes = (uint16_t)(p_state_ptr->bytes_per_frame - p_state_ptr->tx_frame_cntr);
        }
        else
        {
            num_bytes = 4U;
        }
        word_to_send = 0;

        if (p_state_ptr->p_tx_buffer != NULL)
        {
            switch (p_state_ptr->bytes_per_frame)
            {
                case 1:
                    word_to_send = *((const uint8_t *)(p_state_ptr->p_tx_buffer));
                    p_state_ptr->p_tx_buffer += sizeof(uint8_t);
                    break;
                case 2:
                    word_to_send = *((const uint16_t *)(p_state_ptr->p_tx_buffer));
                    p_state_ptr->p_tx_buffer += sizeof(uint16_t);
                    break;
                default:
                    word_to_send = *((const uint32_t *)(p_state_ptr->p_tx_buffer));
                    p_state_ptr->p_tx_buffer += sizeof(uint32_t);
                    break;
            }
            p_state_ptr->tx_frame_cntr =
                (uint16_t)((p_state_ptr->tx_frame_cntr + num_bytes) % p_state_ptr->bytes_per_frame);
        }
        lpspi_write_data(p_base, word_to_send);
        /* Update internal variable used in transmission. */
        p_state_ptr->tx_remain_cntr = (uint16_t)(p_state_ptr->tx_remain_cntr - num_bytes);
        /* Verify if all bytes were send. */
        if (p_state_ptr->tx_remain_cntr == 0U)
        {
            break;
        }
        available_space = (uint8_t)(available_space - 1U);
    }
}

static void
lpspiif_read_rx_buffer(uint32_t instance)
{
    lpspiif_state_t *p_state_ptr = sp_lpspiif_state_ptr[instance];
    const LPSPI_t   *p_base      = gp_lpspi_base[instance];
    uint32_t         received_word;
    uint16_t         num_bytes;
    uint16_t         loop;
    uint8_t          filled_space = (uint8_t)lpspi_read_rx_count(p_base);
    while (filled_space != 0U)
    {
        received_word = lpspi_read_data(p_base);
        /* Get the number of bytes which can be read from this 32 bites */
        if ((p_state_ptr->bytes_per_frame - p_state_ptr->rx_frame_cntr) <= (uint16_t)4)
        {
            num_bytes = (uint16_t)(p_state_ptr->bytes_per_frame - p_state_ptr->rx_frame_cntr);
        }
        else
        {
            num_bytes = 4U;
        }
        /* Generate the word which will be write in buffer. */
        for (loop = 0; loop < num_bytes; loop++)
        {
            *(p_state_ptr->p_rx_buffer) = (uint8_t)(received_word >> (loop * 8U));
            p_state_ptr->p_rx_buffer++;
        }
        p_state_ptr->rx_frame_cntr =
            (uint16_t)((p_state_ptr->rx_frame_cntr + num_bytes) % p_state_ptr->bytes_per_frame);

        /* Update internal variable used in transmission. */
        p_state_ptr->rx_remain_cntr = (uint16_t)(p_state_ptr->rx_remain_cntr - num_bytes);
        /* Verify if all bytes were sent. */
        if (p_state_ptr->rx_remain_cntr == 0U)
        {
            break;
        }
        filled_space = (uint8_t)(filled_space - 1U);
    }
}

static void
lpspiif_disable_teie_intrpts(uint32_t instance)
{
    LPSPI_t *p_base = gp_lpspi_base[instance];

    lpspi_set_int_mode(p_base, LPSPI_TRANSMIT_ERROR, false);
    lpspi_set_int_mode(p_base, LPSPI_RECEIVE_ERROR, false);
    (void)lpspi_clear_status_flag(p_base, LPSPI_TRANSMIT_ERROR);
    (void)lpspi_clear_status_flag(p_base, LPSPI_RECEIVE_ERROR);
}

static status_t
lpspiif_set_pcs(uint32_t instance, lpspi_which_pcs_t which_pcs, lpspi_signal_polarity_t polarity)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    DEV_ASSERT((uint32_t)which_pcs < 4U);

    LPSPI_t *p_base = gp_lpspi_base[instance];
    status_t sts_err_code;

    if (lpspi_disable(p_base) != STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }
    sts_err_code = lpspi_set_pcs_polarity_mode(p_base, which_pcs, polarity);
    if (sts_err_code != STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }
    lpspi_enable(p_base);
    lpspi_set_pcs(p_base, which_pcs);
    return STATUS_SUCCESS;
}

static void
lpspiif_master_isr(uint32_t instance)
{
    /* Instantiate local variable of type dspi_master_state_t and point to
     * global state */
    lpspiif_state_t *p_state_ptr = sp_lpspiif_state_ptr[instance];
    LPSPI_t         *p_base      = gp_lpspi_base[instance];
    uint32_t         status_bm;

    status_bm = lpspi_get_status_bm(p_base);
    /** Todo. */
    // lpspi_clear_status_bm(p_base, status_bm & (uint32_t)LPSPI_ALL_STATUS);
    /* If an error is detected the transfer will be aborted */
    if ((bool)lpspi_get_status_flag(p_base, LPSPI_TRANSMIT_ERROR) &&
        (p_state_ptr->p_tx_buffer != NULL))
    {
        (void)lpspiif_master_abort_xfer(instance);
        (void)lpspi_clear_status_flag(p_base, LPSPI_TRANSMIT_ERROR);
        p_state_ptr->xfer_status = LPSPIIF_TRANSMIT_FAIL;
        return;
    }
    if (lpspi_get_status_flag(p_base, LPSPI_RECEIVE_ERROR) && (p_state_ptr->p_rx_buffer != NULL))
    {
        (void)lpspiif_master_abort_xfer(instance);
        (void)lpspi_clear_status_flag(p_base, LPSPI_RECEIVE_ERROR);
        p_state_ptr->xfer_status = LPSPIIF_RECEIVE_FAIL;
        return;
    }

    /* RECEIVE IRQ handler: Check read buffer only if there are remaining bytes
     * to read. */
    if (lpspi_get_status_flag(p_base, LPSPI_RX_DATA_FLAG))
    {
        if (p_state_ptr->rx_remain_cntr != (uint16_t)0)
        {
            lpspiif_read_rx_buffer(instance);
        }
    }
    /* Transmit data */
    if (lpspi_get_status_flag(p_base, LPSPI_TX_DATA_FLAG))
    {
        if ((p_state_ptr->tx_remain_cntr != (uint16_t)0))
        {
            lpspiif_fillup_tx_buffer(instance);
        }
    }
    if (p_state_ptr->tx_remain_cntr == (uint16_t)0)
    {
        /* Disable TX flag. Software buffer is empty.*/
        lpspi_set_int_mode(p_base, LPSPI_TX_DATA_FLAG, false);
        lpspi_set_int_mode(p_base, LPSPI_TRANSFER_COMPLETE, true);
        /* Check if we're done with this transfer.*/
        if (p_state_ptr->rx_remain_cntr == (uint16_t)0)
        {
            if (lpspi_get_status_flag(p_base, LPSPI_TRANSFER_COMPLETE) == true)
            {
                lpspiif_master_complete_xfer(instance);
            }
        }
    }
}

static status_t
lpspiif_master_set_delay(uint32_t instance,
                         uint32_t delay_betwen_xfer,
                         uint32_t delay_sck_to_pcs,
                         uint32_t delay_pcs_to_sck)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);

    /* Instantiate local variable of type LPSPI_t and point to global state
     */
    LPSPI_t *p_base       = gp_lpspi_base[instance];
    status_t sts_err_code = STATUS_SUCCESS;

    /* Disable module */
    sts_err_code = lpspi_disable(p_base);
    if (sts_err_code != STATUS_SUCCESS)
    {
        return sts_err_code;
    }

    (void)lpspi_set_delay(p_base, LPSPI_SCK_TO_PCS, delay_sck_to_pcs);
    (void)lpspi_set_delay(p_base, LPSPI_PCS_TO_SCK, delay_pcs_to_sck);
    (void)lpspi_set_delay(p_base, LPSPI_BETWEEN_TRANSFER, delay_betwen_xfer);
    /* Enable module */
    lpspi_enable(p_base);
    return sts_err_code;
}

static status_t
lpspiif_master_configure_bus(uint32_t                       instance,
                             const lpspiif_master_config_t *p_init_config,
                             uint32_t                      *p_calc_baud_rate)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(p_init_config != NULL);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    /* Instantiate local variable of type lpspiif_state_t and point to global
     * state */
    lpspiif_state_t *p_state_ptr = sp_lpspiif_state_ptr[instance];
    LPSPI_t         *p_base      = gp_lpspi_base[instance];
    uint32_t         baud_rate;

    /* The Transmit Command Register (TCR) Prescale value is calculated as part
       of the baud rate calculation. The value is stored in the run-time state
       structure for later programming in the TCR. */
    uint32_t tcr_prescale_value;

    /* First, per the spec, we need to disable the LPSPI module before setting
     * the delay */

    if (lpspi_disable(p_base) != STATUS_SUCCESS)
    {
        /* If error is returned, the LPSPI is busy */
        return STATUS_ERROR;
    }

    /* Check the bit count to make sure it falls within the boundary conditions
     */
    if ((p_init_config->bits_per_frame < 8U) || (p_init_config->bits_per_frame > 4096U))
    {
        return STATUS_ERROR;
    }

    /* Configure internal state structure for LPSPI */
    p_state_ptr->bits_per_frame   = p_init_config->bits_per_frame;
    p_state_ptr->b_pcs_continuous = p_init_config->b_pcs_continuous;
    p_state_ptr->b_lsb            = p_init_config->b_lsb_first;
    /* Save transfer type DMA/Interrupt */
    p_state_ptr->xfer_type = p_init_config->xfer_type;
    /* Update transfer status */
    p_state_ptr->b_xfer_in_progress = false;
    p_state_ptr->b_blocking         = false;
    /* Calculate the bytes/frame for p_state_ptr->bytes_per_frame. */
    p_state_ptr->bytes_per_frame = (uint16_t)((p_state_ptr->bits_per_frame + 7U) / 8U);
    (void)clock_get_freq(*(p_state_ptr->p_clk_name), &(p_state_ptr->src_clk_freq));
    /* Due to DMA limitations frames of 3 bytes/frame will be internally handled
     * as 4 bytes/frame. */
    if (p_state_ptr->bytes_per_frame == 3U)
    {
        p_state_ptr->bytes_per_frame = 4U;
    }
    /* Due to some limitations all frames bigger than 4 bytes/frame must be
     * composed only from 4 bytes chunks. */
    if (p_state_ptr->bytes_per_frame > 4U)
    {
        p_state_ptr->bytes_per_frame = (((p_state_ptr->bytes_per_frame - 1U) / 4U) + 1U) * 4U;
    }
    /* Store DMA channel number used in transfer */
    p_state_ptr->rx_dma_channel = p_init_config->rx_dma_channel;
    p_state_ptr->tx_dma_channel = p_init_config->tx_dma_channel;
    /* Store callback */
    p_state_ptr->callback         = p_init_config->callback;
    p_state_ptr->p_callback_param = p_init_config->p_callback_param;
    /* Configure the desired PCS polarity */
    (void)lpspi_set_pcs_polarity_mode(
        p_base, p_init_config->which_pcs, p_init_config->pcs_polarity);
    /* Set up the baud rate */
    baud_rate = lpspi_set_baud_rate(
        p_base, p_init_config->bits_per_sec, p_state_ptr->src_clk_freq, &tcr_prescale_value);
#if 0
    /* Enable sampling point delay */
    lpspi_set_sampling_point(p_base, true);
#endif
    /* Now, re-enable the LPSPI module */
    lpspi_enable(p_base);
    /* If the baud rate return is "0", it means there was an error */
    if (baud_rate == (uint32_t)0)
    {
        return STATUS_ERROR;
    }
    /* If the user wishes to know the calculated baud rate, then pass it back */
    if (p_calc_baud_rate != NULL)
    {
        *p_calc_baud_rate = baud_rate;
    }
    /* Write the TCR for this transfer. */
    lpspi_tx_cmd_config_t tx_cmd_cfg = {.frame_size   = p_state_ptr->bits_per_frame,
                                        .width        = LPSPI_SINGLE_BIT_XFER,
                                        .b_tx_mask    = false,
                                        .b_rx_mask    = false,
                                        .b_cont_cmd   = false,
                                        .b_cont_xfer  = p_init_config->b_pcs_continuous,
                                        .b_byte_swap  = p_init_config->b_byte_swap,
                                        .b_lsb_first  = p_init_config->b_lsb_first,
                                        .which_pcs    = p_init_config->which_pcs,
                                        .pre_div      = tcr_prescale_value,
                                        .clk_phase    = p_init_config->clk_phase,
                                        .clk_polarity = p_init_config->clk_polarity};
    lpspi_set_tx_cmd_reg_tcr(p_base, &tx_cmd_cfg);
    return STATUS_SUCCESS;
}

static status_t
lpspiif_master_abort_xfer(uint32_t instance)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    LPSPI_t *p_base = gp_lpspi_base[instance];
    /* Stop the running transfer. */
    lpspiif_master_complete_xfer(instance);
    lpspi_set_flush_fifo_cmd(p_base, true, true);
    /* The second flush command is used to avoid the case when one word is still
     * in shifter. */
    lpspi_set_flush_fifo_cmd(p_base, true, true);
    return STATUS_SUCCESS;
}

static status_t
lpspiif_master_start_xfer(uint32_t       instance,
                          const uint8_t *p_send_buffer,
                          uint8_t       *p_rcv_buffer,
                          uint16_t       xfer_byte_count)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    /* Instantiate local variable of type dspi_master_state_t and point to
     * global state */
    lpspiif_state_t     *p_state_ptr        = sp_lpspiif_state_ptr[instance];
    LPSPI_t             *p_base             = gp_lpspi_base[instance];
    edma_transfer_size_t dma_xfer_unit_size = EDMA_TRANSFER_SIZE_1B;

    /* Check that we're not busy. */
    if (lpspi_get_status_flag(p_base, LPSPI_MODULE_BUSY))
    {
        return STATUS_BUSY;
    }
#ifdef ERRATA_E10655
    else
    {
        /* Double check to fix errata e10655. */
        if (lpspi_get_status_flag(p_base, LPSPI_MODULE_BUSY))
        {
            return STATUS_BUSY;
        }
    }
#endif

    /* Verify if the number of bytes is divided by number of bytes/frame. */
    if ((xfer_byte_count % p_state_ptr->bytes_per_frame) != (uint16_t)0)
    {
        return STATUS_ERROR;
    }

    /* Clean RX and TX buffers */
    lpspi_set_flush_fifo_cmd(p_base, true, true);
    /* The second flush command is used to avoid the case when one word is still
     * in shifter. */
    lpspi_set_flush_fifo_cmd(p_base, true, true);

    if (p_state_ptr->b_pcs_continuous == true)
    {
        lpspi_set_cont_cmd_bit(p_base);
    }
    p_state_ptr->xfer_status = LPSPIIF_TRANSFER_OK;
    /* Clear all interrupts sources */
    (void)lpspi_clear_status_flag(p_base, LPSPI_ALL_STATUS);
    /* Enable fault interrupts sources */
    lpspi_set_int_mode(p_base, LPSPI_TRANSMIT_ERROR, true);
    if (p_rcv_buffer != NULL)
    {
        lpspi_set_int_mode(p_base, LPSPI_RECEIVE_ERROR, true);
    }

    /* Configure rx_remain_cntr depending on transfer type.*/
    if (p_rcv_buffer == NULL)
    {
        p_state_ptr->rx_remain_cntr = 0;
        lpspi_set_rx_mask_bit(p_base);
    }
    else
    {
        p_state_ptr->rx_remain_cntr = xfer_byte_count;
        lpspi_clear_rx_mask_bit(p_base);
    }

    /* Configure watermarks */
    lpspi_set_rx_watermarks(p_base, 0U);
    lpspi_set_tx_watermarks(p_base, 2U);

    if (p_state_ptr->xfer_type == LPSPIIF_USING_INTERRUPTS)
    {

        /* Fill out the other members of the run-time state structure. */
        p_state_ptr->p_tx_buffer    = (const uint8_t *)p_send_buffer;
        p_state_ptr->p_rx_buffer    = (uint8_t *)p_rcv_buffer;
        p_state_ptr->tx_frame_cntr  = 0;
        p_state_ptr->rx_frame_cntr  = 0;
        p_state_ptr->tx_remain_cntr = xfer_byte_count;
        /*For continuous mode an extra word must be written to negate the PCS */
        if (p_state_ptr->b_pcs_continuous == true)
        {
            p_state_ptr->tx_remain_cntr++;
        }

        /* Update transfer status */
        p_state_ptr->b_xfer_in_progress = true;
        /* Enable RDF interrupt if RX buffer is not NULL. */
        if (p_state_ptr->p_rx_buffer != NULL)
        {
            lpspi_set_int_mode(p_base, LPSPI_RX_DATA_FLAG, true);
        }
        /* Enable the TDF and RDF interrupt. */
        lpspi_set_int_mode(p_base, LPSPI_TX_DATA_FLAG, true);
    }
    else
    {

        /* When LPSPI use DMA frames with 3 bytes size are not accepted. */
        switch (p_state_ptr->bytes_per_frame)
        {
            case 1:
                dma_xfer_unit_size = EDMA_TRANSFER_SIZE_1B;
                break;
            case 2:
                dma_xfer_unit_size = EDMA_TRANSFER_SIZE_2B;
                break;
            case 4:
                dma_xfer_unit_size = EDMA_TRANSFER_SIZE_4B;
                break;
            default:
                dma_xfer_unit_size = EDMA_TRANSFER_SIZE_4B;
                break;
        }
        /* Configure TX DMA channel */
        if (p_send_buffer != NULL)
        {
            (void)edma_config_multi_block_transfer(
                p_state_ptr->tx_dma_channel,
                EDMA_TRANSFER_MEM2PERIPH,
                (uint32_t)p_send_buffer,
                (uint32_t)(&(p_base->TDR)),
                dma_xfer_unit_size,
                (uint32_t)1U << (uint8_t)(dma_xfer_unit_size),
                (uint32_t)xfer_byte_count /
                    (uint32_t)((uint32_t)1U << (uint8_t)(dma_xfer_unit_size)),
                true);
        }
        else
        {
            (void)edma_config_multi_block_transfer(
                p_state_ptr->tx_dma_channel,
                EDMA_TRANSFER_PERIPH2PERIPH,
                (uint32_t)(&(p_state_ptr->dummy)),
                (uint32_t)(&(p_base->TDR)),
                dma_xfer_unit_size,
                (uint32_t)1U << (uint8_t)(dma_xfer_unit_size),
                (uint32_t)xfer_byte_count /
                    (uint32_t)((uint32_t)1U << (uint8_t)(dma_xfer_unit_size)),
                true);
        }
        /* Configure RX DMA channel if is used in current transfer. */
        if (p_rcv_buffer != NULL)
        {
            (void)edma_config_multi_block_transfer(
                p_state_ptr->rx_dma_channel,
                EDMA_TRANSFER_PERIPH2MEM,
                (uint32_t)(&(p_base->RDR)),
                (uint32_t)p_rcv_buffer,
                dma_xfer_unit_size,
                (uint32_t)1U << (uint8_t)(dma_xfer_unit_size),
                (uint32_t)xfer_byte_count /
                    (uint32_t)((uint32_t)1U << (uint8_t)(dma_xfer_unit_size)),
                true);
            (void)edma_install_callback(
                p_state_ptr->rx_dma_channel, (lpspiif_master_complete_rx), (void *)(instance));
            /* Start RX channel */
            (void)edma_start_channel(p_state_ptr->rx_dma_channel);
        }

        /* If RX buffer is null the transfer is done when all bytes were sent.
         */
        (void)edma_install_callback(
            p_state_ptr->tx_dma_channel, (lpspiif_master_complete_dma_xfer), (void *)(instance));

        /* Start TX channel */
        (void)edma_start_channel(p_state_ptr->tx_dma_channel);
        /* Update transfer status */
        p_state_ptr->b_xfer_in_progress = true;
        /* Enable LPSPI DMA request */
        if (p_rcv_buffer != NULL)
        {
            lpspi_set_rx_dma_cmd(p_base, true);
        }
        lpspi_set_tx_dma_cmd(p_base, true);
    }
    return STATUS_SUCCESS;
}

static void
lpspiif_master_complete_xfer(uint32_t instance)
{
    /* instantiate local variable of type dspi_master_state_t and point to
     * global state */
    lpspiif_state_t *p_state_ptr = sp_lpspiif_state_ptr[instance];
    LPSPI_t         *p_base      = gp_lpspi_base[instance];
    /* The transfer is complete.*/
    p_state_ptr->b_xfer_in_progress = false;
    if (p_state_ptr->xfer_type == LPSPIIF_USING_DMA)
    {
        /* Disable LPSPI DMA request */
        lpspi_set_rx_dma_cmd(p_base, false);
        lpspi_set_tx_dma_cmd(p_base, false);
    }
    else
    {
        /* Disable (clear) interrupt requests */
        lpspi_set_int_mode(p_base, LPSPI_RX_DATA_FLAG, false);
        lpspi_set_int_mode(p_base, LPSPI_TX_DATA_FLAG, false);
    }

    lpspiif_disable_teie_intrpts(instance);
    lpspi_set_int_mode(p_base, LPSPI_TRANSFER_COMPLETE, false);
    (void)lpspi_clear_status_flag(p_base, LPSPI_TRANSFER_COMPLETE);
    if (p_state_ptr->b_blocking == true)
    {
        (void)osif_sema_post(&(p_state_ptr->sema_blocking_xfer));
        p_state_ptr->b_blocking = false;
    }
    if (p_state_ptr->callback != NULL)
    {
        p_state_ptr->callback(p_state_ptr, SPI_EVENT_END_TRANSFER, p_state_ptr->p_callback_param);
    }
}

static void
lpspiif_master_complete_dma_xfer(void *parameter, edma_chn_status_t status)
{
    uint32_t         instance    = (uint32_t)parameter;
    lpspiif_state_t *p_state_ptr = sp_lpspiif_state_ptr[instance];
    LPSPI_t         *p_base      = gp_lpspi_base[instance];

    if (status == EDMA_CHN_ERROR)
    {
        (void)lpspiif_master_abort_xfer(instance);
        p_state_ptr->xfer_status = LPSPIIF_TRANSMIT_FAIL;
    }
    else
    {
        if (p_state_ptr->b_pcs_continuous == true)
        {
            lpspi_clear_cont_cmd_bit(p_base);
        }

        /* Enable transfer complete flag interrupt to catch the end of the
         * transfer. */
        p_state_ptr->tx_remain_cntr = 0;
        p_state_ptr->rx_remain_cntr = 0;
        lpspi_set_int_mode(p_base, LPSPI_TRANSFER_COMPLETE, true);
    }
}

static void
lpspiif_master_complete_rx(void *parameter, edma_chn_status_t status)
{
    uint32_t         instance    = (uint32_t)parameter;
    lpspiif_state_t *p_state_ptr = sp_lpspiif_state_ptr[instance];

    if (status == EDMA_CHN_ERROR)
    {
        (void)lpspiif_master_abort_xfer(instance);
        p_state_ptr->xfer_status = LPSPIIF_TRANSMIT_FAIL;
    }
}

static void
lpspiif_slave_isr(uint32_t instance)
{
    LPSPI_t         *p_base      = gp_lpspi_base[instance];
    lpspiif_state_t *p_state_ptr = (lpspiif_state_t *)sp_lpspiif_state_ptr[instance];

    /* If an error is detected the transfer will be aborted */
    if ((bool)lpspi_get_status_flag(p_base, LPSPI_TRANSMIT_ERROR) &&
        (p_state_ptr->p_tx_buffer != NULL))
    {
        (void)lpspiif_slave_abort_xfer(instance);
        p_state_ptr->xfer_status = LPSPIIF_TRANSMIT_FAIL;
        return;
    }
    if (lpspi_get_status_flag(p_base, LPSPI_RECEIVE_ERROR) && (p_state_ptr->p_rx_buffer != NULL))
    {
        (void)lpspiif_slave_abort_xfer(instance);
        p_state_ptr->xfer_status = LPSPIIF_RECEIVE_FAIL;
        return;
    }

    /* Receive data */
    if (lpspi_get_status_flag(p_base, LPSPI_RX_DATA_FLAG))
    {
        if ((p_state_ptr->rx_remain_cntr != (uint8_t)0))
        {
            lpspiif_read_rx_buffer(instance);
        }
    }
    /* Transmit data */
    if (lpspi_get_status_flag(p_base, LPSPI_TX_DATA_FLAG))
    {
        if ((p_state_ptr->tx_remain_cntr != (uint8_t)0))
        {
            lpspiif_fillup_tx_buffer(instance);
        }
    }
    /* If all bytes are sent disable interrupt TDF */
    if (p_state_ptr->tx_remain_cntr == (uint8_t)0)
    {
        lpspi_set_int_mode(p_base, LPSPI_TX_DATA_FLAG, false);
    }
    /* If all bytes are received disable interrupt RDF */
    if (p_state_ptr->rx_remain_cntr == (uint8_t)0)
    {
        lpspi_set_int_mode(p_base, LPSPI_RX_DATA_FLAG, false);
    }
    if (p_state_ptr->rx_remain_cntr == (uint8_t)0)
    {
        if (p_state_ptr->tx_remain_cntr == (uint8_t)0)
        {
            /* Disable fault interrupts sources */
            lpspi_set_int_mode(p_base, LPSPI_TRANSMIT_ERROR, false);
            lpspi_set_int_mode(p_base, LPSPI_RECEIVE_ERROR, false);

            /* Call the callback if it is defined */
            if (p_state_ptr->callback != NULL)
            {
                p_state_ptr->callback(
                    p_state_ptr, SPI_EVENT_END_TRANSFER, p_state_ptr->p_callback_param);
            }

            /* If the transfer is blocking post the semaphore */
            if (p_state_ptr->b_blocking == true)
            {
                (void)osif_sema_post(&(p_state_ptr->sema_blocking_xfer));
                p_state_ptr->b_blocking = false;
            }

            /* Update internal state of the transfer */
            p_state_ptr->b_xfer_in_progress = false;
        }
    }
}

/*
 * Implements : LPSPI_DRV_SlaveAbortTransfer_Activity
 */
static status_t
lpspiif_slave_abort_xfer(uint32_t instance)
{
    DEV_ASSERT(instance < LPSPI_INSTANCE_COUNT);
    DEV_ASSERT(sp_lpspiif_state_ptr[instance] != NULL);
    LPSPI_t         *p_base      = gp_lpspi_base[instance];
    lpspiif_state_t *p_state_ptr = (lpspiif_state_t *)sp_lpspiif_state_ptr[instance];

    if (p_state_ptr->xfer_type == LPSPIIF_USING_INTERRUPTS)
    {
        /* Disable interrupts */
        lpspi_set_int_mode(p_base, LPSPI_TX_DATA_FLAG, false);
        lpspi_set_int_mode(p_base, LPSPI_RX_DATA_FLAG, false);
    }
    else
    {
        /* Disable LPSPI DMA request */
        lpspi_set_rx_dma_cmd(p_base, false);
        lpspi_set_tx_dma_cmd(p_base, false);
    }

    lpspiif_disable_teie_intrpts(instance);

    p_state_ptr->b_xfer_in_progress = false;
    /* Clean RX and TX buffers */
    lpspi_set_flush_fifo_cmd(p_base, true, true);
    /* The second flush command is used to avoid the case when one word is still
     * in shifter. */
    lpspi_set_flush_fifo_cmd(p_base, true, true);
    if (p_state_ptr->b_blocking == true)
    {
        (void)osif_sema_post(&(p_state_ptr->sema_blocking_xfer));
        p_state_ptr->b_blocking = false;
    }
    return STATUS_SUCCESS;
}

/*!
 * @brief Finish up a transfer DMA.
 * The main purpose of this function is to create a function compatible with DMA
 * callback type
 */
static void
lpspiif_slave_complete_dma_xfer(void *parameter, edma_chn_status_t status)
{
    uint32_t         instance    = (uint32_t)parameter;
    lpspiif_state_t *p_state_ptr = (lpspiif_state_t *)sp_lpspiif_state_ptr[instance];

    (void)status;
    (void)lpspiif_slave_abort_xfer(instance);

    /* Check RX and TX DMA channels. */
    if (edma_get_channel_status(p_state_ptr->tx_dma_channel) == EDMA_CHN_ERROR)
    {
        p_state_ptr->xfer_status = LPSPIIF_TRANSMIT_FAIL;
    }
    if (edma_get_channel_status(p_state_ptr->rx_dma_channel) == EDMA_CHN_ERROR)
    {
        p_state_ptr->xfer_status = LPSPIIF_RECEIVE_FAIL;
    }

    if (p_state_ptr->callback != NULL)
    {
        p_state_ptr->callback(p_state_ptr, SPI_EVENT_END_TRANSFER, p_state_ptr->p_callback_param);
    }
}

/*** end of file ***/
