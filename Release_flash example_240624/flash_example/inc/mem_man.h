/**
 * @file mem_man.h
 * @brief flash test flow interface.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef MEM_MAN_H
#define MEM_MAN_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "flash_driver.h"

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/* Prototypes */
flash_drv_status_t mem_man_test1(uint32_t flash_prog_address,
                                 uint8_t  data_size,
                                 uint8_t *pDataArray);
flash_drv_status_t mem_man_test2(uint32_t flash_prog_address,
                                 uint16_t data_size,
                                 uint8_t *pDataArray);
void               mem_man_disable(void);

void print(const char *sourceStr);
#ifdef __cplusplus
}
#endif

#endif /* MEM_MAN_H */

/*** end of file ***/
