/**
 * @file system_tw9001.c
 * @brief CMSIS cortex-m0 device peripheral access layer system source file.
 *          This file contains the system clock configuration for Tw9001 devices.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdbool.h>
#include "device_registers.h"
#include "system_tw9001.h"

/*******************************************************************************
 * Global variables
 ******************************************************************************/
uint32_t g_system_core_clock = DEFAULT_SYSTEM_CLOCK;

/*******************************************************************************
 * Global functions
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : SystemInit
 * Description   : This function disables the watchdog, if the corresponding
 * feature macro is enabled. SystemInit is called from startup_device file.
 *
 * Implements    : SystemInit_Activity
 *END**************************************************************************/
void
SystemInit(void)
{
    /**************************************************************************/
    /* WDOG DISABLE*/
    /**************************************************************************/

#if (DISABLE_WDOG)
    /* Write of the WDOG unlock key to CNT register, must be done in order to
     * allow any modifications*/
    WDOG->CNT = (uint32_t)FEATURE_WDOG_UNLOCK_VALUE;
    /* The dummy read is used in order to make sure that the WDOG registers will
     * be configured only after the write of the unlock value was completed. */
    (void)WDOG->CNT;

    /* Initial write of WDOG configuration register:
     * enables support for 32-bit refresh/unlock command write words,
     * clock select from LPO, update enable, watchdog disabled */
    WDOG->CS = (uint32_t)((1UL << WDOG_CS_CMD32EN_SHIFT) |
                          (FEATURE_WDOG_CLK_FROM_LPO << WDOG_CS_CLK_SHIFT) |
                          (0U << WDOG_CS_EN_SHIFT) | (1U << WDOG_CS_UPDATE_SHIFT));

    /* Configure timeout */
    WDOG->TOVAL = (uint32_t)0xFFFF;
#endif /* (DISABLE_WDOG) */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SystemCoreClockUpdate
 * Description   : This function must be called whenever the core clock is
 * changed during program execution. It evaluates the clock register settings
 * and calculates the current core clock.
 *
 * Implements    : SystemCoreClockUpdate_Activity
 *END**************************************************************************/
void
SystemCoreClockUpdate(void)
{
    uint32_t scg_out_clock = 0U; /* Variable to store output SCG clock frequency */

    uint32_t divider;
    bool     b_valid_system_clock_source = true;

    divider = ((SCG->CSR & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT) + 1U;

    switch ((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT)
    {
        case 0x1:
        {
            /* System OSC */
            scg_out_clock = CPU_XTAL_CLK_HZ;
        }
        break;
        case 0x2:
        {
            /* Slow IRC */
            scg_out_clock = FEATURE_SCG_SIRC_HIGH_RANGE_FREQ;
        }
        break;
        case 0x3:
        {
            /* Fast IRC */
            //if (PCC->PCCn[PCC_FSUSB_INDEX] & PCC_PCCn_PCD_MASK)
#if defined(SPLL_96MHZ)
            {
                scg_out_clock = FEATURE_SCG_FIRC_FREQ0;
            }
#else       //SPLL 120MHZ
            {
                scg_out_clock = FEATURE_SCG_FIRC_FREQ1;
            }
#endif
        }
        break;
        default:
        {
            b_valid_system_clock_source = false;
        }
        break;
    }

    if (b_valid_system_clock_source == true)
    {
        g_system_core_clock = (scg_out_clock / divider);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SystemSoftwareReset
 * Description   : This function is used to initiate a system reset
 *
 * Implements    : SystemSoftwareReset_Activity
 *END**************************************************************************/
void
SystemSoftwareReset(void)
{
    uint32_t reg_val;

    /* Read Application Interrupt and Reset Control Register */
    reg_val = SCB->AIRCR;

    /* Clear register key */
    reg_val &= ~(SCB_AIRCR_VECTKEY_MASK);

    /* Configure System reset request bit and Register Key */
    reg_val |= SCB_AIRCR_VECTKEY(FEATURE_SCB_VECTKEY);
    reg_val |= SCB_AIRCR_SYSRESETREQ(0x1u);

    /* Write computed register value */
    SCB->AIRCR = reg_val;
}

/*** end of file ***/
