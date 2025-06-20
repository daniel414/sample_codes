/**
 * @file clock_config.c
 * @brief Clock initialization configuration source file.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "clock_config.h"

/*******************************************************************************
 * Global variables
 ******************************************************************************/
/* *************************************************************************
 * Configuration structure for peripheral clock configuration 0
 * ************************************************************************* */
/** @brief peripheral clock configuration 0 */
peripheral_clock_config_t g_peripheral_clock_config0[NUM_OF_PERIPHERAL_CLOCKS_0] = {
    {
        .clk_name   = LPSPI0_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_FIRC_DIV2,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
    {
        .clk_name   = LPSPI1_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_FIRC_DIV2,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
    {
        .clk_name   = LPSPI2_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_FIRC_DIV2,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
    {
        .clk_name   = LPUART0_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_SIRC_DIV2,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
    {
        .clk_name   = LPUART1_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_SIRC_DIV2,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
    {
        .clk_name   = LPI2C0_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_FIRC_DIV2,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
    {
        .clk_name   = LPI2C1_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_FIRC_DIV2,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
    {
        .clk_name   = LPTMR0_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_SIRC_DIV2,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
    {
        .clk_name   = FTM0_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_SIRC_DIV1,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
    {
        .clk_name   = FTM1_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_SIRC_DIV1,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
#if defined(SPLL_96MHZ)
    {
        .clk_name   = FSUSB_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_OFF,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = SPLL_CLKOUT_96M,
    },
#else   //SPLL 120MHZ
    {
        .clk_name   = FSUSB_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_OFF,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = SPLL_CLKOUT_120M,
    },
#endif
    {
        .clk_name   = PUF_CLK,
        .b_clk_gate = true,
        .clk_src    = PUF_CLK_SPLLDIV2,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
    {
        .clk_name   = HSSPI_CLK,
        .b_clk_gate = true,
        .clk_src    = CLK_SRC_FIRC_DIV2,
        .frac       = MULTIPLY_BY_ONE,
        .divider    = DIVIDE_BY_ONE,
    },
};
/* *************************************************************************
 * Configuration structure for Clock Configuration 0
 * ************************************************************************* */
/** @brief User Configuration structure clock_managerCfg_0 */
clock_manager_user_config_t g_clock_man1_init_config0 = {
    .scg_config =
        {
            .sirc_config =
                {
                    .b_initialize = true,
                    .div1 = SCG_ASYNC_CLOCK_DIV_BY_1, /* Slow IRC Clock Divider 1: divided by 1 */
                    .div2 = SCG_ASYNC_CLOCK_DIV_BY_1, /* Slow IRC Clock Divider 1: divided by 1 */
                },
            .firc_config =
                {
                    .b_initialize = true,
                    .div1 = SCG_ASYNC_CLOCK_DIV_BY_1, /* Fast IRC Clock Divider 1: divided by 1 */
                    .div2 = SCG_ASYNC_CLOCK_DIV_BY_1, /* Fast IRC Clock Divider 1: divided by 1 */
                },
            .rtc_config =
                {
                    .b_initialize = false,
                },
            .sosc_config =
                {
                    .b_initialize = true,
                    .freq         = 8000000U,         /* System Oscillator frequency: 8000000Hz */
                    .div1 = SCG_ASYNC_CLOCK_DIV_BY_1, /* System OSC Clock Divider 1: divided by 1 */
                    .div2 = SCG_ASYNC_CLOCK_DIV_BY_1, /* System OSC Clock Divider 1: divided by 1 */
                },
            .spll_config =
                {
                    .b_initialize = false,
                    .div1 = SCG_ASYNC_CLOCK_DIV_BY_1, /* SPLL Clock Divider 1: divided by 1 */
                    .div2 = SCG_ASYNC_CLOCK_DIV_BY_4, /* SPLL Clock Divider 3: divided by 4 */
                },
            .clock_out_config =
                {
                    .b_initialize = true, .source = SCG_CLOCKOUT_SRC_FIRC, /* Fast IRC. */
                },
            .clock_mode_config =
                {
                    .b_initialize = true,
                    .rccr_config =
                        {
                            .src      = SCG_SYSTEM_CLOCK_SRC_FIRC, /* Fast FIRC */
                            .div_core = SCG_SYSTEM_CLOCK_DIV_BY_1, /* Core Clock : divided by 1 */
                            .div_slow = SCG_SYSTEM_CLOCK_DIV_BY_2, /* Slow Clock : divided by 2 */
                        },
                    .vccr_config =
                        {
                            .src      = SCG_SYSTEM_CLOCK_SRC_SIRC, /* Slow SIRC */
                            .div_core = SCG_SYSTEM_CLOCK_DIV_BY_2, /* Core Clock : divided by 2 */
                            .div_slow = SCG_SYSTEM_CLOCK_DIV_BY_4, /* Slow Clock : divided by 4 */
                        },
                },
        },
    .pcc_config =
        {
            .p_peripheral_clocks = g_peripheral_clock_config0, /**< PCC configurations  */
            .count = NUM_OF_PERIPHERAL_CLOCKS_0, /**< Number of the peripheral clock  */
        },
    .sim_config =
        {
            .clock_out_config =
                {
                    .b_initialize = true,                        /**< Initialize */
                    .b_enable     = true,                        /* enabled */
                    .source  = SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT, /* SCG CLKOUT: SCG slow clock */
                    .divider = SIM_CLKOUT_DIV_BY_1,              /* Divided by 1 */
                },
            .lpo_clock_config =
                {
                    .b_initialize    = true,                        /**< Initialize    */
                    .b_enable_lpo1k  = true,                        /**< LPO1KCLKEN    */
                    .b_enable_lpo10k = true,                        /**< LPO10KCLKEN   */
                    .source_lpo_clk  = SIM_LPO_CLK_SEL_LPO_125K,    /* 125 kHz LPO clock */
                    .source_rtc_clk  = SIM_RTCCLK_SEL_FIRCDIV1_CLK, /* FIRCDIV1 clock */
                },
            .tclk_config =
                {
                    .b_initialize = false, /**< Initialize    */
                },
        },
};

/** @brief Array of pointers to User configuration structures */
clock_manager_user_config_t const *gp_clock_man_configs_arr[] = {&g_clock_man1_init_config0};

/** @brief Array of pointers to User defined Callbacks configuration structures  */
clock_manager_callback_user_config_t *gp_clock_man_callback_arr[] = {(void *)0};

/*** end of file ***/
