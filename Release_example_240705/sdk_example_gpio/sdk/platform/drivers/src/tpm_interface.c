/**
 * @file tpm_interface.c
 * @brief An implementation of runtime functions for TPM client service.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string.h>
#include "tpm_interface.h"
#include "interrupt_manager.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define LOCALITY_HEAD 0xD4
/*******************************************************************************
 * Private typedefs
 ******************************************************************************/

/*******************************************************************************
 * Global variables
 ******************************************************************************/
tpm_state_t g_tpm_state = {0u};

/*******************************************************************************
 * Static variables
 ******************************************************************************/
/** Pointer to uartif runtime state structure. */
static tpm_state_t *sp_tpm_state_ptr = NULL;

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global functions
 ******************************************************************************/
/**
 * @brief This function is the implementation of HSSPI handler named in startup
 * code.
 */
void
HSSPI_IRQHandler(void)
{
    DEV_ASSERT(NULL != sp_tpm_state_ptr);
    tpm_state_t *p_state_ptr = sp_tpm_state_ptr;

    if (get_tpm_rd_cmd_ok_flag() == true)
    {
        p_state_ptr->irq_status.rx_rd_ok = true;
        clear_tpm_rd_cmd_ok_flag();

        // Last CMD
        p_state_ptr->last_cmd.addr   = get_tpm_last_addr_rec();
        p_state_ptr->last_cmd.rw     = get_tpm_last_cmd_is_read();
        p_state_ptr->last_cmd.length = get_tpm_last_len_rec();

        if (get_tpm_wr_fifo_flag() == true)
        {
            p_state_ptr->irq_status.rx_wr_fifo = true;
            clear_tpm_wr_fifo_flag();
        }
    }

    if (get_tpm_rd_cmd_flag() == true)
    {
        p_state_ptr->irq_status.rx_rd_start = true;
        clear_tpm_rd_cmd_flag();
    }

    if (get_tpm_wr_cmd_flag() == true)
    {
        p_state_ptr->irq_status.rx_wr_ok = true;
        clear_tpm_wr_cmd_flag();

        // Last CMD
        p_state_ptr->last_cmd.addr   = get_tpm_last_addr_rec();
        p_state_ptr->last_cmd.rw     = get_tpm_last_cmd_is_read();
        p_state_ptr->last_cmd.length = get_tpm_last_len_rec();
    }

    if (get_tpm_busy_flag() == true)
    {
        p_state_ptr->irq_status.slave_busy = true;
    }
}

void
tpm_init(void)
{
    DEV_ASSERT(NULL == sp_tpm_state_ptr);
    tpm_state_t *p_state_ptr = &g_tpm_state;
    HSSPI_t     *p_base      = NULL;

    /** Clear the state struct for this instance. */
    memset((void *)p_state_ptr, 0x00u, sizeof(tpm_state_t));
    /** Save the runtime state structure pointer. */
    sp_tpm_state_ptr    = p_state_ptr;
    p_state_ptr->p_base = p_base = gp_hsspi_base[INST_HSSPI0];
    p_state_ptr->p_irq_id        = &g_hsspi_irq_id[INST_HSSPI0];
    p_state_ptr->p_clk_name      = &g_hsspi_clk_names[INST_HSSPI0];

    hsspi_set_function_enable_bit(p_base, HSSPI_FUN_TPMEN_BM | HSSPI_FUN_RPEN_BM);
    /** Enable the event alert gate of the HSSPI module. */
    hsspi_set_intrpt_enable_bit(p_base, HSSPI_INT_ALL_BM);
    /** Enable the vector interrupt. */
    int_enable_irq(*(p_state_ptr->p_irq_id));
}

void
tpm_deinit(void)
{
    DEV_ASSERT(NULL != sp_tpm_state_ptr);
    tpm_state_t *p_state_ptr = sp_tpm_state_ptr;
    HSSPI_t     *p_base      = p_state_ptr->p_base;

    /** Disable the event alert gate of the HSSPI module. */
    hsspi_clear_intrpt_enable_bit(p_base, HSSPI_INT_ALL_BM);
    /** Disable the vector interrupt. */
    int_disable_irq(*(p_state_ptr->p_irq_id));
    /** Clear the state pointer. */
    sp_tpm_state_ptr = NULL;
}

bool
get_tpm_wr_cmd_flag(void)
{
    HSSPI_t *p_base      = gp_hsspi_base[INST_HSSPI0];
    bool     WR_CMD_flag = hsspi_get_rx_wrcmd_flag(p_base);
    return WR_CMD_flag;
}

void
clear_tpm_wr_cmd_flag(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_clear_rx_wrcmd_flag(p_base);
}

bool
get_tpm_rd_cmd_flag(void)
{
    HSSPI_t *p_base      = gp_hsspi_base[INST_HSSPI0];
    bool     RD_CMD_flag = hsspi_get_rx_rdcmd_wait_flag(p_base);
    return RD_CMD_flag;
}

void
clear_tpm_rd_cmd_flag(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_clear_rx_rdcmd_wait_flag(p_base);
}

bool
get_tpm_rd_cmd_ok_flag(void)
{
    HSSPI_t *p_base         = gp_hsspi_base[INST_HSSPI0];
    bool     RD_CMD_OK_flag = hsspi_get_rx_rdcmd_ok_flag(p_base);
    return RD_CMD_OK_flag;
}

void
clear_tpm_rd_cmd_ok_flag(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_clear_rx_rdcmd_ok_flag(p_base);
}

bool
get_tpm_wr_fifo_flag(void)
{
    HSSPI_t *p_base       = gp_hsspi_base[INST_HSSPI0];
    bool     WR_FIFO_flag = hsspi_get_rx_fifo_wrcmd_flag(p_base);
    return WR_FIFO_flag;
}

void
clear_tpm_wr_fifo_flag(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_clear_rx_fifo_wrcmd_flag(p_base);
}

bool
get_tpm_busy_flag(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    bool     busy_flag = hsspi_busy_flag(p_base);
    return busy_flag;
}

uint32_t
get_tpm_last_addr_rec(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    uint32_t last_addr = hsspi_tpm_last_addr_rec(p_base);
    return last_addr;
}

bool
get_tpm_last_cmd_is_read(void)
{
    HSSPI_t *p_base  = gp_hsspi_base[INST_HSSPI0];
    bool     is_read = hsspi_tpm_last_cmd_is_read(p_base);
    return is_read;
}

uint8_t
get_tpm_last_len_rec(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    uint8_t  length = hsspi_tpm_last_len_rec(p_base);
    return length;
}

void
set_tpm_add_1_wait(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_clk_skew_cancellation_on_off(p_base, true);
}

void
set_tpm_locality(tpm_loc_num_sel_t loc_sel)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    // hsspi_tpm_loc_head(p_base, LOCALITY_HEAD);
    hsspi_tpm_loc_num_sel(p_base, loc_sel);
}

// TPM_DID_VID_x
uint32_t
get_tpm_did_vid_reg(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    uint32_t reg_value = hsspi_get_tpm_did_vid_reg(p_base);
    return reg_value;
}

void
set_tpm_did_vid_reg(uint32_t reg_value)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_did_vid_reg(p_base, reg_value);
}

void
set_tpm_vid(uint16_t vid)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_vid(p_base, vid);
}

void
set_tpm_did(uint16_t did)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_did(p_base, did);
}

uint16_t
get_tpm_vid(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    uint16_t vid    = hsspi_get_tpm_vid(p_base);
    return vid;
}

uint16_t
get_tpm_did(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    uint16_t did    = hsspi_get_tpm_did(p_base);
    return did;
}

// TPM_STS_x
uint32_t
get_tpm_sts_reg(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    uint32_t reg_value = hsspi_get_tpm_sts_reg(p_base);
    return reg_value;
}

void
set_tpm_sts_reg(uint32_t reg_value)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_reg(p_base, reg_value);
}

bool
get_tpm_sts_responseretry(void)
{
    HSSPI_t *p_base        = gp_hsspi_base[INST_HSSPI0];
    bool     responseretry = hsspi_get_tpm_sts_responseretry(p_base);
    return responseretry;
}

void
set_tpm_sts_responseretry(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_responseretry(p_base, b_enable);
}

bool
get_tpm_sts_selftestdone(void)
{
    HSSPI_t *p_base       = gp_hsspi_base[INST_HSSPI0];
    bool     selftestdone = hsspi_get_tpm_sts_selftestdone(p_base);
    return selftestdone;
}

void
set_tpm_sts_selftestdone(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_selftestdone(p_base, b_enable);
}

bool
get_tpm_sts_expect(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    bool     expect = hsspi_get_tpm_sts_expect(p_base);
    return expect;
}

void
set_tpm_sts_expect(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_expect(p_base, b_enable);
}

bool
get_tpm_sts_dataavail(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    bool     dataavail = hsspi_get_tpm_sts_dataavail(p_base);
    return dataavail;
}

void
set_tpm_sts_dataavail(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_dataavail(p_base, b_enable);
}

bool
get_tpm_sts_tpmgo(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    bool     tpmgo  = hsspi_get_tpm_sts_tpmgo(p_base);
    return tpmgo;
}

void
set_tpm_sts_tpmgo(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_tpmgo(p_base, b_enable);
}

bool
get_tpm_sts_commandready(void)
{
    HSSPI_t *p_base       = gp_hsspi_base[INST_HSSPI0];
    bool     commandready = hsspi_get_tpm_sts_commandready(p_base);
    return commandready;
}

void
set_tpm_sts_commandready(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_commandready(p_base, b_enable);
}

bool
get_tpm_sts_stsvalid(void)
{
    HSSPI_t *p_base   = gp_hsspi_base[INST_HSSPI0];
    bool     stsvalid = hsspi_get_tpm_sts_stsvalid(p_base);
    return stsvalid;
}

void
set_tpm_sts_stsvalid(bool b_valid)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_stsvalid(p_base, b_valid);
}

uint16_t
get_tpm_sts_burstcount(void)
{
    HSSPI_t *p_base     = gp_hsspi_base[INST_HSSPI0];
    uint16_t burstcount = hsspi_get_tpm_sts_burstcount(p_base);
    return burstcount;
}

void
set_tpm_sts_burstcount(uint16_t count)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_burstcount(p_base, count);
}

bool
get_tpm_sts_commandcancel(void)
{
    HSSPI_t *p_base        = gp_hsspi_base[INST_HSSPI0];
    bool     commandcancel = hsspi_get_tpm_sts_commandcancel(p_base);
    return commandcancel;
}

void
set_tpm_sts_commandcancel(bool b_cancel)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_commandcancel(p_base, b_cancel);
}

bool
get_tpm_sts_resetestablishment(void)
{
    HSSPI_t *p_base             = gp_hsspi_base[INST_HSSPI0];
    bool     resetestablishment = hsspi_get_tpm_sts_resetestablishment(p_base);
    return resetestablishment;
}

void
set_tpm_sts_resetestablishment(bool b_reset)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_resetestablishment(p_base, b_reset);
}

tpm_sts_tpmfamily_t
get_tpm_sts_tpmfamily(void)
{
    HSSPI_t            *p_base    = gp_hsspi_base[INST_HSSPI0];
    tpm_sts_tpmfamily_t tpmfamily = hsspi_get_tpm_sts_tpmfamily(p_base);
    return tpmfamily;
}

void
set_tpm_sts_tpmfamily(tpm_sts_tpmfamily_t select)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_sts_tpmfamily(p_base, select);
}

// TPM_ACCESS_x
uint32_t
get_tpm_access_reg(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    uint32_t reg_value = hsspi_get_tpm_access_reg(p_base);
    return reg_value;
}

void
set_tpm_access_reg(uint32_t reg_value)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_access_reg(p_base, reg_value);
}

bool
get_tpm_access_establishment(void)
{
    HSSPI_t *p_base        = gp_hsspi_base[INST_HSSPI0];
    bool     establishment = hsspi_get_tpm_access_establishment(p_base);
    return establishment;
}

void
set_tpm_access_establishment(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_access_establishment(p_base, b_enable);
}

bool
get_tpm_access_requestUse(void)
{
    HSSPI_t *p_base     = gp_hsspi_base[INST_HSSPI0];
    bool     requestUse = hsspi_get_tpm_access_requestUse(p_base);
    return requestUse;
}

void
set_tpm_access_requestUse(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_access_requestUse(p_base, b_enable);
}

bool
get_tpm_access_pendingRequest(void)
{
    HSSPI_t *p_base         = gp_hsspi_base[INST_HSSPI0];
    bool     pendingRequest = hsspi_get_tpm_access_pendingRequest(p_base);
    return pendingRequest;
}

void
set_tpm_access_pendingRequest(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_access_pendingRequest(p_base, b_enable);
}

bool
get_tpm_access_seize(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    bool     seize  = hsspi_get_tpm_access_seize(p_base);
    return seize;
}

void
set_tpm_access_seize(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_access_seize(p_base, b_enable);
}

bool
get_tpm_access_beenseize(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    bool     seize  = hsspi_get_tpm_access_beenseize(p_base);
    return seize;
}

void
set_tpm_access_beenseize(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_access_beenseize(p_base, b_enable);
}

bool
get_tpm_access_activeLocality(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    bool     seize  = hsspi_get_tpm_access_activeLocality(p_base);
    return seize;
}

void
set_tpm_access_activeLocality(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_access_activeLocality(p_base, b_enable);
}

bool
get_tpm_access_tpmRegValidSts(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    bool     seize  = hsspi_get_tpm_access_tpmRegValidSts(p_base);
    return seize;
}

void
set_tpm_access_tpmRegValidSts(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_access_tpmRegValidSts(p_base, b_enable);
}

// TPM_INT_VECTOR_x
uint8_t
get_tpm_intvector_sirqvec(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    uint8_t  vector = hsspi_get_tpm_intvector_sirqvec(p_base);
    return vector;
}

void
set_tpm_intvector_sirqvec(uint8_t vector)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intvector_sirqvec(p_base, vector);
}

// TPM_INT_ENABLE_x
uint32_t
get_tpm_inten_reg(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    uint32_t reg_value = hsspi_get_tpm_inten_reg(p_base);
    return reg_value;
}

void
set_tpm_inten_reg(uint32_t reg_value)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_inten_reg(p_base, reg_value);
}

bool
get_tpm_inten_dataavailint(void)
{
    HSSPI_t *p_base     = gp_hsspi_base[INST_HSSPI0];
    bool     data_avail = hsspi_get_tpm_inten_dataavailint(p_base);
    return data_avail;
}

void
set_tpm_inten_dataavailint(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_inten_dataavailint(p_base, b_enable);
}

bool
get_tpm_inten_stsvalidint(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    bool     sts_valid = hsspi_get_tpm_inten_stsvalidint(p_base);
    return sts_valid;
}

void
set_tpm_inten_stsvalidint(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_inten_stsvalidint(p_base, b_enable);
}

bool
get_tpm_inten_localitychangeint(void)
{
    HSSPI_t *p_base         = gp_hsspi_base[INST_HSSPI0];
    bool     localitychange = hsspi_get_tpm_inten_localitychangeint(p_base);
    return localitychange;
}

void
set_tpm_inten_localitychangeint(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_inten_localitychangeint(p_base, b_enable);
}

tpm_inten_typpol_t
get_tpm_inten_typepolarity(void)
{
    HSSPI_t           *p_base       = gp_hsspi_base[INST_HSSPI0];
    tpm_inten_typpol_t typepolarity = hsspi_get_tpm_inten_typepolarity(p_base);
    return typepolarity;
}

void
set_tpm_inten_typepolarity(tpm_inten_typpol_t polarity)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_inten_typepolarity(p_base, polarity);
}

bool
get_tpm_inten_commandready(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    bool     cmd_ready = hsspi_get_tpm_inten_commandready(p_base);
    return cmd_ready;
}

void
set_tpm_inten_commandready(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_inten_commandready(p_base, b_enable);
}

bool
get_tpm_inten_globalint(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    bool     global = hsspi_get_tpm_inten_globalint(p_base);
    return global;
}

void
set_tpm_inten_globalint(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_inten_globalint(p_base, b_enable);
}

// TPM_INT_STATUS_x
uint32_t
get_tpm_intstatus_reg(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    uint32_t reg_value = hsspi_get_tpm_intstatus_reg(p_base);
    return reg_value;
}

void
set_tpm_intstatus_reg(uint32_t reg_value)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intstatus_reg(p_base, reg_value);
}

bool
get_tpm_intstatus_dataAvailInt(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    bool     dataAvail = hsspi_get_tpm_intstatus_dataAvailInt(p_base);
    return dataAvail;
}

void
set_tpm_intstatus_dataAvailInt(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intstatus_dataAvailInt(p_base, b_enable);
}

bool
get_tpm_intstatus_stsValidInt(void)
{
    HSSPI_t *p_base   = gp_hsspi_base[INST_HSSPI0];
    bool     stsValid = hsspi_get_tpm_intstatus_stsValidInt(p_base);
    return stsValid;
}

void
set_tpm_intstatus_stsValidInt(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intstatus_stsValidInt(p_base, b_enable);
}

bool
get_tpm_intstatus_localityChangeInt(void)
{
    HSSPI_t *p_base         = gp_hsspi_base[INST_HSSPI0];
    bool     localityChange = hsspi_get_tpm_intstatus_localityChangeInt(p_base);
    return localityChange;
}

void
set_tpm_intstatus_localityChangeInt(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intstatus_localityChangeInt(p_base, b_enable);
}

bool
get_tpm_intstatus_commandReadyInt(void)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    bool     global = hsspi_get_tpm_intstatus_commandReadyInt(p_base);
    return global;
}

void
set_tpm_intstatus_commandReadyInt(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intstatus_commandReadyInt(p_base, b_enable);
}

// TPM_INTF_CAPABILITY_x
uint32_t
get_tpm_intfcap_reg(void)
{
    HSSPI_t *p_base    = gp_hsspi_base[INST_HSSPI0];
    uint32_t reg_value = hsspi_get_tpm_intfcap_reg(p_base);
    return reg_value;
}

void
set_tpm_intfcap_reg(uint32_t reg_value)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_reg(p_base, reg_value);
}

bool
get_tpm_intfcap_dataAvailInt(void)
{
    HSSPI_t *p_base       = gp_hsspi_base[INST_HSSPI0];
    bool     dataAvailInt = hsspi_get_tpm_intfcap_dataAvailInt(p_base);
    return dataAvailInt;
}

void
set_tpm_intfcap_dataAvailInt(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_dataAvailInt(p_base, b_enable);
}

bool
get_tpm_intfcap_stsValidInt(void)
{
    HSSPI_t *p_base      = gp_hsspi_base[INST_HSSPI0];
    bool     stsValidInt = hsspi_get_tpm_intfcap_stsValidInt(p_base);
    return stsValidInt;
}

void
set_tpm_intfcap_stsValidInt(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_stsValidInt(p_base, b_enable);
}

bool
get_tpm_intfcap_localityChangeInt(void)
{
    HSSPI_t *p_base            = gp_hsspi_base[INST_HSSPI0];
    bool     localityChangeInt = hsspi_get_tpm_intfcap_localityChangeInt(p_base);
    return localityChangeInt;
}

void
set_tpm_intfcap_localityChangeInt(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_localityChangeInt(p_base, b_enable);
}

bool
get_tpm_intfcap_InterruptLevelHigh(void)
{
    HSSPI_t *p_base             = gp_hsspi_base[INST_HSSPI0];
    bool     InterruptLevelHigh = hsspi_get_tpm_intfcap_InterruptLevelHigh(p_base);
    return InterruptLevelHigh;
}

void
set_tpm_intfcap_InterruptLevelHigh(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_InterruptLevelHigh(p_base, b_enable);
}

bool
get_tpm_intfcap_InterruptLevelLow(void)
{
    HSSPI_t *p_base            = gp_hsspi_base[INST_HSSPI0];
    bool     InterruptLevelLow = hsspi_get_tpm_intfcap_InterruptLevelLow(p_base);
    return InterruptLevelLow;
}

void
set_tpm_intfcap_InterruptLevelLow(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_InterruptLevelLow(p_base, b_enable);
}

bool
get_tpm_intfcap_InterruptEdgeRising(void)
{
    HSSPI_t *p_base              = gp_hsspi_base[INST_HSSPI0];
    bool     InterruptEdgeRising = hsspi_get_tpm_intfcap_InterruptEdgeRising(p_base);
    return InterruptEdgeRising;
}

void
set_tpm_intfcap_InterruptEdgeRising(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_InterruptEdgeRising(p_base, b_enable);
}

bool
get_tpm_intfcap_InterruptEdgeFalling(void)
{
    HSSPI_t *p_base               = gp_hsspi_base[INST_HSSPI0];
    bool     InterruptEdgeFalling = hsspi_get_tpm_intfcap_InterruptEdgeFalling(p_base);
    return InterruptEdgeFalling;
}

void
set_tpm_intfcap_InterruptEdgeFalling(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_InterruptEdgeFalling(p_base, b_enable);
}

bool
get_tpm_intfcap_CommandReadyInt(void)
{
    HSSPI_t *p_base          = gp_hsspi_base[INST_HSSPI0];
    bool     CommandReadyInt = hsspi_get_tpm_intfcap_CommandReadyInt(p_base);
    return CommandReadyInt;
}

void
set_tpm_intfcap_CommandReadyInt(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_CommandReadyInt(p_base, b_enable);
}

bool
get_tpm_intfcap_BurstCountStatic(void)
{
    HSSPI_t *p_base           = gp_hsspi_base[INST_HSSPI0];
    bool     BurstCountStatic = hsspi_get_tpm_intfcap_BurstCountStatic(p_base);
    return BurstCountStatic;
}

void
set_tpm_intfcap_BurstCountStatic(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_BurstCountStatic(p_base, b_enable);
}

tpm_intfcap_transfsize_t
get_tpm_intfcap_DataTransferSize(void)
{
    HSSPI_t                 *p_base = gp_hsspi_base[INST_HSSPI0];
    tpm_intfcap_transfsize_t Size   = hsspi_get_tpm_intfcap_DataTransferSize(p_base);
    return Size;
}

void
set_tpm_intfcap_DataTransferSize(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_DataTransferSize(p_base, b_enable);
}

tpm_intfcap_interface_t
get_tpm_intfcap_InterfaceVersion(void)
{
    HSSPI_t                *p_base  = gp_hsspi_base[INST_HSSPI0];
    tpm_intfcap_interface_t Version = hsspi_get_tpm_intfcap_InterfaceVersion(p_base);
    return Version;
}

void
set_tpm_intfcap_InterfaceVersion(bool b_enable)
{
    HSSPI_t *p_base = gp_hsspi_base[INST_HSSPI0];
    hsspi_set_tpm_intfcap_InterfaceVersion(p_base, b_enable);
}

/////////////////////////////

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*** end of file ***/
