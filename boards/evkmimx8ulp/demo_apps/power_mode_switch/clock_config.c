/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "clock_config.h"
#include "board.h"
#include "fsl_upower.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*
 * SYSOSC clock setting.
 * SYSOSC clock         : 24MHz
 */
const cgc_sosc_config_t g_cgcSysOscConfig = {.freq        = BOARD_XTAL0_CLK_HZ,
                                             .enableMode  = 0, /* Disabled in Deep Sleep/Power down */
                                             .monitorMode = kCGC_SysOscMonitorDisable,
                                             .workMode    = kCGC_SysOscModeOscLowPower};

/*
 * FRO clock setting.
 * FRO clock            : 192MHz
 */
const cgc_fro_config_t g_cgcFroConfig = {.enableMode = 0 /*0 - FRO is disabled in Sleep/Deep Sleep Modes*/};

/*
 * MAX FREQ:
 * PLL0 VCO: 480 MHz
 * PLL0 PFD0: 400 MHz
 * PLL0 PFD1: 480 MHz
 * PLL0 PFD2: 480 MHz
 * PLL0 PFD3: 400 MHz
 *
 * PLL0 clock setting.
 * PLL0 clock VCO       : 24 * 20 = 480MHz
 * PLL0 VCODIV output   : 0MHz
 * PLL0 PFD1DIV output  : 0MHz
 * PLL0 PFD2DIV output  : 0MHz
 */
const cgc_pll0_config_t g_cgcPll0Config = {.enableMode = kCGC_PllEnable,
                                           .div1 = 0U,
                                           .pfd1Div = 0U,
                                           .pfd2Div = 0U,
                                           .src = kCGC_PllSrcSysOsc,
                                           .mult = kCGC_Pll0Mult20
                                          }; /* x20 */

/*
 * MAX FREQ:
 * PLL1 VCO: 542 MHz
 * PLL1 PFD0: 432 MHz
 * PLL1 PFD1: 400 MHz
 * PLL1 PFD2: 542 MHz
 * PLL1 PFD3: 480 MHz
 *
 * PLL1 clock setting.
 * PLL1 clock VCO       : 24 * 22.528 = 540.672MHz
 * PLL1 VCODIV output   : 0MHz
 * PLL1 PFD1DIV output  : 0MHz
 * PLL1 PFD2DIV output  : 540.672 / 44 = 12.288MHz
 */
const cgc_pll1_config_t g_cgcPll1Config = {.enableMode = kCGC_PllEnable,
                                           .div1 = 0U,
                                           .pfd1Div = 0U,
                                           .pfd2Div = 44U,
                                           .src = kCGC_PllSrcSysOsc,
                                           .mult = kCGC_Pll1Mult22,
                                           .num = 528000U,
                                           .denom = 1000000U};

/*
 * Select FRO as system clock source, before configuring other clock sources.
 * Clock source   : FRO
 * Core clock     : 192 / (0 + 1) = 192MHz
 * Bus clock      : 192 / ((0 + 1) * (1 + 1)) = 96MHz
 * Slow clock     : 192 / ((0 + 1) * (1 + 3)) = 48MHz
 */
const cgc_rtd_sys_clk_config_t g_sysClkConfigFroSource = {
    .divCore = 0, /* Core clock divider. */
    .divBus  = 1, /* Bus clock divider. */
    .divSlow = 3, /* Slow clock divider. */
    .src     = kCGC_RtdSysClkSrcFro, /* System clock source. */
    .locked  = 0, /* Register not locked. */
};

// clang-format off
/*
 * System clock source and divider in Normal RUN mode.
 *
 * Clock source   : PLL0 main clock 480 MHz.
 * Core clock     : 480 / (2 + 1) = 160 MHz
 * Bus clock      : 480 / ((2 + 1) * (1 + 1)) = 80 MHz
 * Slow clock     : 480 / ((2 + 1) * (1 + 7)) = 20 MHz
 *
 * Clock source   : PLL0PFD0 main clock 392.7272 MHz.
 * Core clock     : 392.7272 / 2 = 196.3636 MHz
 * Bus clock      : 392.7272 / 4 = 98.1818 MHz
 * Slow clock     : 392.7272 / 16 = 24.5454 MHz
 *
 * Clock source   : PLL1PFD0 main clock 423.134608 MHz.
 * Core clock     : 423.134608 / 2 = 211.567304 MHz
 * Bus clock      : 423.134608 / 4 = 105.783652 MHz
 * Slow clock     : 423.134608 / 16 =  26.445913 MHz
 *
 * Clock source   : PLL1PFD0 main clock 405.504 MHz.
 * Core clock     : 405.504 / 2 = 202.752 MHz
 * Bus clock      : 405.504 / 4 = 101.376 MHz
 * Slow clock     : 405.504 / 16 = 25.344 MHz
 *
 * Clock source   : PLL1PFD0 main clock 389.28384 MHz.
 * Core clock     : 389.28384 / 2 =  194.64192 MHz
 * Bus clock      : 389.28384 / 4 = 97.32096 MHz
 * Slow clock     : 389.28384 / 16 = 24.33024 MHz
 * -----------------------------------------------------------
 * |    voltage            |maximum cm33 core clock frequency|
 * -----------------------------------------------------------
 * | OverDrive 1.1 V       |             216 MHz             |
 * -----------------------------------------------------------
 * | Norminal 1.0 V        |             172 MHz             |
 * -----------------------------------------------------------
 * | UnderDrive 0.9 V      |              80 MHz             |
 * -----------------------------------------------------------
 * | UnderDrive 0.9 V RBB  |              40 MHz             |
 * -----------------------------------------------------------
 */
// clang-format on
#define SELECT_PLL0PFD0_AS_SOURCE_CLK (0)
#define SELECT_PLL1PFD0_AS_SOURCE_CLK (0)
const cgc_rtd_sys_clk_config_t g_sysClkConfigRun = {
    .divBus  = 1,   /* Bus clock divider. */
    .divSlow = 7,   /* Slow clock divider. */
#if SELECT_PLL0PFD0_AS_SOURCE_CLK
    .src     = kCGC_RtdSysClkSrcPll0Pfd0, /* PLL0PFD0 main clock source. */
    .divCore = 1,   /* Core clock divider. */
#elif SELECT_PLL1PFD0_AS_SOURCE_CLK
    .src     = kCGC_RtdSysClkSrcPll1Pfd0, /* PLL1PFD0 main clock source. */
    .divCore = 1,   /* Core clock divider. */
#else
    .src     = kCGC_RtdSysClkSrcPll0, /* PLL0 main clock source. */
    .divCore = 2,   /* Core clock divider. */
#endif
    .locked  = 0, /* Register not locked. */
};

/*******************************************************************************
 * Code
 ******************************************************************************/
static void BOARD_InitClock(void)
{
    CLOCK_InitFro(&g_cgcFroConfig);

    if (!CLOCK_IsSysOscValid())
    {
        CLOCK_InitSysOsc(&g_cgcSysOscConfig);
    }

    CLOCK_SetXtal0Freq(BOARD_XTAL0_CLK_HZ);

    /* Then set SOSC, FRO DIV needed by application */
    CLOCK_SetRtdSysOscAsyncClkDiv(kCGC_AsyncDiv1Clk, 1);
    CLOCK_SetRtdSysOscAsyncClkDiv(kCGC_AsyncDiv2Clk, 1);
    CLOCK_SetRtdSysOscAsyncClkDiv(kCGC_AsyncDiv3Clk, 0);

    CLOCK_SetRtdFroAsyncClkDiv(kCGC_AsyncDiv1Clk, 1);
    CLOCK_SetRtdFroAsyncClkDiv(kCGC_AsyncDiv2Clk, 1);
    CLOCK_SetRtdFroAsyncClkDiv(kCGC_AsyncDiv3Clk, 0);

    /* RTC OSC clock is enabled by default, but spin till it is stable */
    while(!CLOCK_IsRtcOscValid())
    {
    }
    CLOCK_SetXtal32Freq(CLK_RTC_32K_CLK);
}

#if SELECT_PLL1PFD0_AS_SOURCE_CLK
#define PLL1PFD0_AS_423MHZ (0)
#define PLL1PFD0_AS_405MHZ (0)
#define PLL1PFD0_AS_389MHZ (1)
#endif
/*
 * Clock in RUN mode:
 * SYSOSC  : Enable
 * FRO    : Enable
 * FIRC    : Enable
 * PLL0  : Enable
 * AUXPLL  : Enable
 *
 * Note: 
 *     Pls decrease cortex-m33's clock frequency before decreasing voltage.
 *     Pls increase voltage before increasing cortex-m33's clock frequency.
 */
void BOARD_BootClockRUN(void)
{
    UPOWER_Init(NULL);
    BOARD_InitClock();

    /* default voltage is 1.0 V for Real Time Domain, default cortex-m33's clock frequency is 92 MHz(clock source is FRO192CLK, div is 1, so 192 MHz / (1 + 1) = 96 MHz) */

    /* Call function BOARD_FlexspiClockSafeConfig() to move FlexSPI clock to a stable clock source to avoid
       instruction/data fetch issue when updating PLL and Main clock if XIP(execute code on FLEXSPI memory). */
    BOARD_FlexspiClockSafeConfig();

    //CLOCK_SetCm33SysClkConfig(&g_sysClkConfigFroSource);

    /* Initialize PLL0 480MHz */
    CLOCK_InitPll0(&g_cgcPll0Config);

    /* Enable Pll0 Pfd0(MAX Freq is 400 MHz) 392.7272 MHz = (18 * 480) / 22 */
    CLOCK_EnablePll0PfdClkout(kCGC_PllPfd0Clk, 22U);
    /* Enable Pll0 Pfd1(MAX Freq is 480 MHz) 480 MHz = (18 * 480) / 18 */
    CLOCK_EnablePll0PfdClkout(kCGC_PllPfd1Clk, 18U);
    /* Enable Pll0 Pfd2(MAX Freq is 480 MHz) 480MHz */
    CLOCK_EnablePll0PfdClkout(kCGC_PllPfd2Clk, 18U);
    /* Enable Pll0 Pfd3(MAX Freq is 400 MHz) 392.7272 MHz = (18 * 480) / 22 */
    CLOCK_EnablePll0PfdClkout(kCGC_PllPfd3Clk, 22U);

    /* Initialize PLL1(MAX Freq is 542 MHz) 540.672 MHz */
    CLOCK_InitPll1(&g_cgcPll1Config);
#if PLL1PFD0_AS_423MHZ
    /* Enable Pll1 Pfd0(MAX Freq is 432 MHz) 423.134608 MHz = (18 * 540.672) / 23 */
    CLOCK_EnablePll1PfdClkout(kCGC_PllPfd0Clk, 23U);
#elif PLL1PFD0_AS_405MHZ
    /* Enable Pll1 Pfd0(MAX Freq is 432 MHz) 405.504 MHz = (18 * 540.672) / 24 */
    CLOCK_EnablePll1PfdClkout(kCGC_PllPfd0Clk, 24U);
#elif PLL1PFD0_AS_389MHZ
    /* Enable Pll1 Pfd0(MAX Freq is 432 MHz) 389.28384 MHz = (18 * 540.672) / 25 */
    CLOCK_EnablePll1PfdClkout(kCGC_PllPfd0Clk, 25U);
#endif
    /* Enable Pll1 Pfd2(MAX Freq is 542 MHz) 540.672MHz */
    CLOCK_EnablePll1PfdClkout(kCGC_PllPfd2Clk, 18U);

    BOARD_UpdateM33CoreFreq(g_sysClkConfigRun);

    /* Call function BOARD_SetFlexspiClock() to set user configured clock source/divider for FlexSPI. */
    BOARD_SetFlexspiClock(FLEXSPI0, 5U, 1U, 0U); /* CM33_PLATCLK / 2 */
}

/*
 * Init Clock after system exit from Power Down:
 * SYSOSC  : Enable
 * FRO    : Enable
 * FIRC    : Enable
 * PLL0  : Enable
 * AUXPLL  : Enable
 * Note: Currently the global interrupt is disabled, so don't use api that depend on interrupt, such as: UPOWER_Init(),BOARD_UpdateM33CoreFreq()...
 */
void BOARD_ResumeClockInit(void)
{
    BOARD_InitClock();

    /* Call function BOARD_FlexspiClockSafeConfig() to move FlexSPI clock to a stable clock source to avoid
       instruction/data fetch issue when updating PLL and Main clock if XIP(execute code on FLEXSPI memory). */
    BOARD_FlexspiClockSafeConfig();

    /* Initialize PLL0 480MHz */
    CLOCK_InitPll0(&g_cgcPll0Config);

    /* Enable Pll0 Pfd0(MAX Freq is 400 MHz) 392.7272 MHz = (18 * 480) / 22 */
    CLOCK_EnablePll0PfdClkout(kCGC_PllPfd0Clk, 22U);
    /* Enable Pll0 Pfd1(MAX Freq is 480 MHz) 480 MHz = (18 * 480) / 18 */
    CLOCK_EnablePll0PfdClkout(kCGC_PllPfd1Clk, 18U);
    /* Enable Pll0 Pfd2(MAX Freq is 480 MHz) 480MHz */
    CLOCK_EnablePll0PfdClkout(kCGC_PllPfd2Clk, 18U);
    /* Enable Pll0 Pfd3(MAX Freq is 400 MHz) 392.7272 MHz = (18 * 480) / 22 */
    CLOCK_EnablePll0PfdClkout(kCGC_PllPfd3Clk, 22U);

    /* Initialize PLL1(MAX Freq is 542 MHz) 540.672 MHz */
    CLOCK_InitPll1(&g_cgcPll1Config);
#if PLL1PFD0_AS_423MHZ
    /* Enable Pll1 Pfd0(MAX Freq is 432 MHz) 423.134608 MHz = (18 * 540.672) / 23 */
    CLOCK_EnablePll1PfdClkout(kCGC_PllPfd0Clk, 23U);
#elif PLL1PFD0_AS_405MHZ
    /* Enable Pll1 Pfd0(MAX Freq is 432 MHz) 405.504 MHz = (18 * 540.672) / 24 */
    CLOCK_EnablePll1PfdClkout(kCGC_PllPfd0Clk, 24U);
#elif PLL1PFD0_AS_389MHZ
    /* Enable Pll1 Pfd0(MAX Freq is 432 MHz) 389.28384 MHz = (18 * 540.672) / 25 */
    CLOCK_EnablePll1PfdClkout(kCGC_PllPfd0Clk, 25U);
#endif
    /* Enable Pll1 Pfd2(MAX Freq is 542 MHz) 540.672MHz */
    CLOCK_EnablePll1PfdClkout(kCGC_PllPfd2Clk, 18U);

    CLOCK_SetCm33SysClkConfig(&g_sysClkConfigRun); /* Pls enlarge the voltage of RTD(cortex-m33) before increasing core frequency */
    SystemCoreClockUpdate();

    /* Call function BOARD_SetFlexspiClock() to set user configured clock source/divider for FlexSPI. */
    BOARD_SetFlexspiClock(FLEXSPI0, 5U, 1U, 0U); /* CM33_PLATCLK / 2 */
}
