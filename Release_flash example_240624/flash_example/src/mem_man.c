/**
 * @file mem_man.c
 * @brief flash test flow
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "mem_man.h"

/*******************************************************************************
 * Global variables
 ******************************************************************************/
uint8_t  g_is_mem_init           = 0; /* Variable used to flag is the memory has been initialized */
uint32_t g_flash_last_erased_sec = 0; /* Last erased flash sector count */

#define CMD_LENGTH 0x80u
uint32_t g_cmd_sequence_in_ram[CMD_LENGTH]; /* flash_cmd_sequence will be copied
                                             to this location */
#define CALLBACK_LENGTH 0x80u
uint32_t g_callback_in_ram[CALLBACK_LENGTH]; /* Callback function will be copied
                                              to this location */
uint8_t g_test_flag = 0;

/** @brief Configuration structure flashCfg_0 */
const flash_user_config_t g_flash_init_config0 = {
    /* for the TW9001 */
    .pflash_base = 0x00000000U,
    .pflash_size = 0x00040000U,
    .ee_ram_base = 0x14000000U,
    .callback    = NULL_CALLBACK,
};

/* Provide info about the flash blocks through an USER config structure */
flash_user_config_t g_flash_user_config;

/* Declare a FLASH config struct which initialized by flash_init, and will be
 * used by all flash operations */
flash_ssd_config_t g_flash_ssd_config;

void
accel_ram_write(const flash_ssd_config_t *p_ssd_config, uint16_t data_size, uint8_t *pDataArray)
{
    for (uint16_t idx = 0; idx < data_size / 4; idx++)
    {
        /*test write */
        *(volatile uint32_t *)(p_ssd_config->ee_ram_base + 4 * idx) =
            *((uint32_t *)&pDataArray[4 * idx]);
    }
}

flash_drv_status_t
mem_man_test1(uint32_t flash_prog_address, uint8_t data_size, uint8_t *pDataArray)
{
    flash_drv_status_t ret = FTFx_OK;
    char               message[50];
    if (g_is_mem_init == 0)
    { /* Initialize memory */
        flash_init(&g_flash_init_config0, &g_flash_ssd_config);
        g_is_mem_init = 1;
    }

    ret = flash_erase_sector(&g_flash_ssd_config,
                             flash_prog_address,
                             FEATURE_FLS_PF_BLOCK_SECTOR_SIZE,
                             flash_cmd_sequence); // 1 sector = 1024B
    sprintf(
        message, "\tEraseFlashSector 0x09: %s \n", (ret == FTFx_OK) ? "Erase Ok" : "Erase Error");
    print(message);
    if (ret != FTFx_OK)
        return ret;

    ret = flash_verify_section(&g_flash_ssd_config,
                               flash_prog_address,
                               data_size / FEATURE_FLS_PF_BLOCK_WRITE_UNIT_SIZE,
                               0x00,
                               flash_cmd_sequence); // 1 phrase = 8B
    sprintf(message, "\tRead1sSection 0x01: %s  \n", (ret == FTFx_OK) ? "Erased" : "Not Erased");
    print(message);
    if (ret != FTFx_OK)
        return ret;

    ret = flash_program(&g_flash_ssd_config,
                        flash_prog_address,
                        data_size,
                        pDataArray,
                        flash_cmd_sequence); // 1 phrase = 8B
    sprintf(
        message, "\tProgramPhrase 0x07: %s  \n", (ret == FTFx_OK) ? "Program Ok" : "Program Error");
    print(message);
    return ret;
}

flash_drv_status_t
mem_man_test2(uint32_t flash_prog_address, uint16_t data_size, uint8_t *pDataArray)
{
    flash_drv_status_t ret = FTFx_OK;
    char               message[50];
    uint16_t           data_count;
    uint16_t           record_size;
    uint16_t           offset = 0;

    if (g_is_mem_init == 0)
    { /* Initialize memory */
        flash_init(&g_flash_init_config0, &g_flash_ssd_config);
        g_is_mem_init = 1;
    }

    ret = flash_erase_sector(&g_flash_ssd_config,
                             flash_prog_address,
                             FEATURE_FLS_PF_BLOCK_SECTOR_SIZE,
                             flash_cmd_sequence); // 1 sector = 1024B
    sprintf(
        message, "\tEraseFlashSector 0x09: %s \n", (ret == FTFx_OK) ? "Erase Ok" : "Erase Error");
    print(message);
    if (ret != FTFx_OK)
        return ret;

    ret = flash_verify_section(&g_flash_ssd_config,
                               flash_prog_address,
                               data_size / FEATURE_FLS_PF_BLOCK_WRITE_UNIT_SIZE,
                               0x00,
                               flash_cmd_sequence); // 1 phrase = 8B
    sprintf(message, "\tRead1sSection 0x01: %s \n", (ret == FTFx_OK) ? "Erased" : "Not Erased");
    print(message);
    if (ret != FTFx_OK)
        return ret;

    data_count = data_size;
    while (data_count > 0)
    {
        if (data_count > FEATURE_FLS_FLEX_RAM_SIZE)
        {
            record_size = FEATURE_FLS_FLEX_RAM_SIZE;
            data_count -= FEATURE_FLS_FLEX_RAM_SIZE;
        }
        else
        {
            record_size = data_count;
            data_count  = 0;
        }
        accel_ram_write(
            &g_flash_ssd_config, record_size, pDataArray + offset); // accel_ram size = 512B
        ret = flash_program_section(&g_flash_ssd_config,
                                    flash_prog_address + offset,
                                    record_size / FEATURE_FLS_PF_BLOCK_WRITE_UNIT_SIZE,
                                    flash_cmd_sequence); // 1 section = 512B
        sprintf(message,
                "\tProgramSection 0x0B: %s \n",
                (ret == FTFx_OK) ? "Program Ok" : "Program Error");
        print(message);
        offset += record_size;
    }
    return ret;
}

/**
 * @brief Checks if the memory was enabled and disables it.
 * @param[in]   none
 * @return      none
 * @ Pass/ Fail criteria: none
 */
void
mem_man_disable(void)
{
    if (g_is_mem_init)
    {
        // FLASH_reset();
    }
}
