/**
 * @file system_TW9001.h
 * @brief CMSIS cortex-m0 device peripheral access layer system header file.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef SYSTEM_TW9001_H
#define SYSTEM_TW9001_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
/* Watchdog disable */
#ifndef DISABLE_WDOG
#define DISABLE_WDOG 1
#endif

/* Value of the external crystal or oscillator clock frequency in Hz */
#ifndef CPU_XTAL_CLK_HZ
#define CPU_XTAL_CLK_HZ 8000000u
#endif

/* Value of the fast internal oscillator clock frequency in Hz  */
#ifndef CPU_INT_FAST_CLK_HZ
#define CPU_INT_FAST_CLK_HZ 48000000u
#endif

/* Default System clock value */
#ifndef DEFAULT_SYSTEM_CLOCK
#define DEFAULT_SYSTEM_CLOCK 48000000u
#endif

/*******************************************************************************
 * Exported variables
 ******************************************************************************/
/**
 * @brief System clock frequency (core clock)
 *
 * The system clock frequency supplied to the SysTick timer and the
 * processor core clock. This variable can be used by the user application
 * to setup the SysTick timer or configure other parameters. It may also be
 * used by debugger to query the frequency of the debug timer or configure
 * the trace clock speed g_system_core_clock is initialized with a correct
 * predefined value.
 */
extern uint32_t g_system_core_clock;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
/**
 * @brief Setup the SoC.
 *
 * This function disables the watchdog.
 * if the corresponding feature macro is enabled.
 * SystemInit is called from startup_device file.
 */
void SystemInit(void);

/**
 * @brief Updates the g_system_core_clock variable.
 *
 * It must be called whenever the core clock is changed during program
 * execution. SystemCoreClockUpdate() evaluates the clock register settings
 * and calculates the current core clock. This function must be called when
 * user does not want to use clock manager component. If clock manager is
 * used, the clock_get_freq function must be used with CORE_CLOCK
 * parameter.
 *
 */
void SystemCoreClockUpdate(void);

/**
 * @brief Initiates a system reset.
 *
 * This function is used to initiate a system reset
 */
void SystemSoftwareReset(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_TW9001_H */

/*** end of file ***/
