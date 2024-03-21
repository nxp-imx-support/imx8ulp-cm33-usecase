/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "fsl_rgpio.h"
#include "fsl_lptmr.h"
#include "fsl_upower.h"
#include "fsl_mu.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include "board.h"
#include "app_srtm.h"
#include "lpm.h"
#include "power_mode_switch.h"
#include "fsl_rtd_cmc.h"
#include "fsl_sentinel.h"
#include "fsl_rgpio.h"
#include "fsl_wuu.h"

#include "fsl_iomuxc.h"
#include "fsl_lpuart.h"
#include "fsl_reset.h"
/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
#define APP_DEBUG_UART_BAUDRATE       (115200U)             /* Debug console baud rate. */
#define APP_DEBUG_UART_DEFAULT_CLKSRC kCLOCK_IpSrcSircAsync /* SCG SIRC clock. */

typedef enum _app_wakeup_source
{
    kAPP_WakeupSourceLptmr, /*!< Wakeup by LPTMR.        */
    kAPP_WakeupSourcePin    /*!< Wakeup by external pin. */
} app_wakeup_source_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
extern void APP_PowerPreSwitchHook(lpm_rtd_power_mode_e targetMode);
extern void APP_PowerPostSwitchHook(lpm_rtd_power_mode_e targetMode, bool result);
extern void APP_CheckPedometerInterrupt(void);
extern void APP_SRTM_WakeupCA35(void);
extern void APP_RebootCA35(void);
extern void APP_ShutdownCA35(void);
extern void APP_BootCA35(void);
extern void UPOWER_InitBuck2Buck3Table(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static SemaphoreHandle_t s_wakeupSig;
static const char *s_modeNames[] = {"ACTIVE", "WAIT", "STOP", "Sleep", "Deep Sleep", "Power Down", "Deep Power Down"};
extern lpm_ad_power_mode_e AD_CurrentMode;
extern bool option_v_boot_flag;
extern lpm_rtd_power_mode_e s_curMode;
extern pca9460_buck3ctrl_t buck3_ctrl;
extern pca9460_ldo1_cfg_t ldo1_cfg;
// clang-format off
mode_combi_t mode_combi_array_for_single_boot[] = {
    {LPM_PowerModeActive, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_ACT, MODE_COMBI_NO},
    {LPM_PowerModePowerDown, AD_ACT, MODE_COMBI_NO},
    {LPM_PowerModeDeepPowerDown, AD_ACT, MODE_COMBI_NO},

    {LPM_PowerModeActive, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModePowerDown, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeDeepPowerDown, AD_PD, MODE_COMBI_NO},

    {LPM_PowerModeActive, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModePowerDown, AD_DPD, MODE_COMBI_YES},
    /* RTD not support Deep Power Down Mode when boot type is SINGLE BOOT TYPE */
    {LPM_PowerModeDeepPowerDown, AD_DPD, MODE_COMBI_NO},
};

mode_combi_t mode_combi_array_for_dual_or_lp_boot[] = {
    {LPM_PowerModeActive, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_ACT, MODE_COMBI_NO},
    {LPM_PowerModePowerDown, AD_ACT, MODE_COMBI_NO},
    {LPM_PowerModeDeepPowerDown, AD_ACT, MODE_COMBI_NO},

    {LPM_PowerModeActive, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModePowerDown, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeDeepPowerDown, AD_PD, MODE_COMBI_NO},

    {LPM_PowerModeActive, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModePowerDown, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeDeepPowerDown, AD_DPD, MODE_COMBI_YES},
};
// clang-format on

/*******************************************************************************
 * Function Code
 ******************************************************************************/
extern lpm_ad_power_mode_e AD_CurrentMode;
extern pca9460_buck3ctrl_t buck3_ctrl;
extern pca9460_ldo1_cfg_t ldo1_cfg;
static uint32_t iomuxBackup[3][25]; /* Backup 25 PTA, 16 PTB and 24 PTC IOMUX registers */
static uint32_t gpioICRBackup[3][25];

static uint32_t g_Wakeup_Pins[] = BOARD_WAKEUP_PINS_LIST;

void APP_SuspendTaskForWakeup(void)
{
    xSemaphoreTake(s_wakeupSig, portMAX_DELAY);
}

static bool APP_Is_WakeupPin(int32_t pin_grp, int32_t pin_idx)
{
    int32_t i = 0;

    for (i = 0; i < ARRAY_SIZE(g_Wakeup_Pins); ++i)
    {
        uint32_t gpio_pin_id = 0;
        int32_t gpio_pin_grp = 0, gpio_pin_index = 0;

        gpio_pin_id = g_Wakeup_Pins[i];
        gpio_pin_grp = (gpio_pin_id >> 8) & 0xFFU;
        gpio_pin_index = (gpio_pin_id & 0xFFU);

        if ((pin_grp == gpio_pin_grp) && (gpio_pin_index == pin_idx))
        {
            return true;
        }
    }

    return false;
}

/* Reconfigure GPIO wakup pin's pinmux as WUU_Px */
static void APP_ReconfigurePinForWakeup(lpm_rtd_power_mode_e target_mode)
{
    int32_t i = 0;
    volatile uint32_t *iomuxc_pcr0_iomuxcarry[3] = { NULL };
    uint32_t tmp_pe1 = 0, tmp_pe2 = 0;

    iomuxc_pcr0_iomuxcarry[0] = IOMUXC0->PCR0_IOMUXCARRAY0;
    iomuxc_pcr0_iomuxcarry[1] = IOMUXC0->PCR0_IOMUXCARRAY1;
    iomuxc_pcr0_iomuxcarry[2] = IOMUXC0->PCR0_IOMUXCARRAY2;

    /*
     * Disable interrupt temperarily to prevent glitch
     * interrupt during switching IOMUXC pin selection
     */
    tmp_pe1 = WUU0->PE1;
    tmp_pe2 = WUU0->PE2;
    WUU0->PE1 = 0;
    WUU0->PE2 = 0;

    /* Go through wakeup pins list and reconfigure pinmux */
    for (i = 0; i < ARRAY_SIZE(g_Wakeup_Pins); ++i)
    {
        uint32_t gpio_pin_id = 0;
        int32_t gpio_pin_grp = 0, gpio_pin_index = 0;
        RGPIO_Type *gpio_inst = NULL;

        gpio_pin_id = g_Wakeup_Pins[i];
        gpio_pin_grp = (gpio_pin_id >> 8) & 0xFFU;
        gpio_pin_index = (gpio_pin_id & 0xFFU);
        gpio_inst = RGPIO_GetBaseByInstance(gpio_pin_grp);

        /*
         * Deep Sleep wakeup via interrupt not WUU,
         * so do nothing in here for Deep Sleep Mode
         */
        /* Enable interrupts for wakeup pin */
        gpio_inst->ICR[gpio_pin_index] = gpioICRBackup[gpio_pin_grp][gpio_pin_index];

        if (target_mode == LPM_PowerModeDeepSleep)
        {
            iomuxc_pcr0_iomuxcarry[gpio_pin_grp][gpio_pin_index] = iomuxBackup[gpio_pin_grp][gpio_pin_index];
        }
        else /* Power Down and Deep Power Down */
        {
            /* Reconfigure IOMUX as WUU0_Px, the mux value is 13 */
            iomuxc_pcr0_iomuxcarry[gpio_pin_grp][gpio_pin_index] = IOMUXC0_PCR0_IOMUXCARRAY0_MUX(13);
        }
    }

    WUU0->PE1 = tmp_pe1;
    WUU0->PE2 = tmp_pe2;
}

static void APP_Suspend(void)
{
    uint32_t i;
    uint32_t setting;
    lpm_rtd_power_mode_e targetPowerMode = LPM_GetPowerMode();

    /* Backup PTA IOMUXC and GPIOA ICR registers then disable */
    for (i = 0; i <= 24; i++)
    {
        iomuxBackup[0][i] = IOMUXC0->PCR0_IOMUXCARRAY0[i];
        gpioICRBackup[0][i] = GPIOA->ICR[i];

        if (APP_Is_WakeupPin(0, i))
        {
            continue;
        }
        /* Skip PTA20 ~ 23(JTAG pins) if run on flash */
        if ((i != 19) && (i != 20) && (i != 21) && (i != 22) && (i != 23) || !BOARD_IS_XIP_FLEXSPI0())
        {
            GPIOA->ICR[i] = 0; /* Disable interrupts */
        }
        IOMUXC0->PCR0_IOMUXCARRAY0[i] = 0;
    }

    /* Backup PTB IOMUXC and GPIOB ICR registers then disable */
    for (i = 0; i <= 15; i++)
    {
        iomuxBackup[1][i] = IOMUXC0->PCR0_IOMUXCARRAY1[i];
        gpioICRBackup[1][i] = GPIOB->ICR[i];

        if (APP_Is_WakeupPin(1, i))
        {
            continue;
        }

        GPIOB->ICR[i] = 0; /* disable interrupts */
        if ((i == 13) && (WUU0->PE2 & WUU_PE2_WUPE25_MASK))
        {
            if (targetPowerMode == LPM_PowerModeDeepSleep)
            {
                /*
                 * Deep Sleep wakeup via interrupt not WUU,
                 * so do nothing in here for Deep Sleep Mode
                 */
                /* enable interrupts for PTB12 */
                GPIOB->ICR[i] = gpioICRBackup[1][i];
            }
            else
            {
                /*
                 * Disable interrupt temperarily to prevent glitch
                 * interrupt during switching IOMUXC pin selection
                 */
                setting = WUU0->PE2 & WUU_PE2_WUPE25_MASK;
                WUU0->PE2 &= !WUU_PE2_WUPE25_MASK;

                /* Change PTB12's function as WUU0_P24(IOMUXC_PTB12_WUU0_P24) */
                IOMUXC0->PCR0_IOMUXCARRAY1[i] = IOMUXC0_PCR0_IOMUXCARRAY1_MUX(13);

                WUU0->PE2 |= setting;

            }            
        }
        else if ((i != 10) && (i != 11)) /* PTB10 and PTB11 is used as i2c function by upower */
        {
            IOMUXC0->PCR0_IOMUXCARRAY1[i] = 0;
        }
    }

    /* Backup PTC IOMUXC and GPIOC ICR registers then disable */
    for (i = 0; i <= 23; i++)
    {
        iomuxBackup[2][i] = IOMUXC0->PCR0_IOMUXCARRAY2[i];
        gpioICRBackup[2][i] = GPIOC->ICR[i];

        if (APP_Is_WakeupPin(2, i))
        {
            continue;
        }

        GPIOC->ICR[i] = 0; /* disable interrupts */

        /* Skip PTC0 ~ 10(FlexSPI0 pins) if run on flash */
        if ((i > 10) || !BOARD_IS_XIP_FLEXSPI0())
        {
            IOMUXC0->PCR0_IOMUXCARRAY2[i] = 0;
        }
    }

    APP_ReconfigurePinForWakeup(targetPowerMode);

    /* Cleare any potential interrupts before enter Power Down */
    WUU0->PF = WUU0->PF;

    /* Save SRTM context */
    APP_SRTM_Suspend();
}

static void APP_Resume(bool resume)
{
    uint32_t i;

    /* Restore PTA IOMUXC and GPIOA ICR registers */
    for (i = 0; i <= 24; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY0[i] = iomuxBackup[0][i];
        GPIOA->ICR[i]                 = gpioICRBackup[0][i];
    }

    /* Restore PTB IOMUXC and GPIOB ICR registers */
    for (i = 0; i <= 15; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY1[i] = iomuxBackup[1][i];
        GPIOB->ICR[i]                 = gpioICRBackup[1][i];
    }

    /* Restore PTC IOMUXC and GPIOC ICR registers */
    for (i = 0; i <= 23; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY2[i] = iomuxBackup[2][i];
        GPIOC->ICR[i]                 = gpioICRBackup[2][i];
    }

    EnableIRQ(WUU0_IRQn);

    APP_SRTM_Resume(resume);
}

/* Disable gpio to save power */
void APP_DisableGPIO(void)
{
    int i = 0;

    /* Disable PTA and set PTA to Analog/HiZ state to save power */
    for (i = 0; i <= 24; i++)
    {
        if ((i != 4) && (i != 6) && (i != 7))
        {
            GPIOA->ICR[i] = 0; /* Disable interrupts */
        }
        
        /* Skip PTA20 ~ 23(JTAG pins) if run on flash */
        if ((i != 20) && (i != 21) && (i != 22) && (i != 23) || !BOARD_IS_XIP_FLEXSPI0())
        {
            IOMUXC0->PCR0_IOMUXCARRAY0[i] = 0; /* Set to Analog/HiZ state */
        }
    }

    /* Disable PTB and set PTB to Analog/HiZ state to save power */
    for (i = 0; i <= 15; i++)
    {
        if ((i != 10) && (i != 11)) /* PTB10 and PTB11 is used as i2c function by upower */
        {
            GPIOB->ICR[i]                 = 0; /* Disable interrupts */
            IOMUXC0->PCR0_IOMUXCARRAY1[i] = 0; /* Set to Analog/HiZ state */
        }
    }

    /* Disable PTC and set PTC to Analog/HiZ state to save power */
    for (i = 0; i <= 23; i++)
    {
        GPIOC->ICR[i] = 0; /* Disable interrupts */

        /* Skip PTC0 ~ 10(FlexSPI0 pins) if run on flash */
        if ((i > 10) || !BOARD_IS_XIP_FLEXSPI0())
        {
            IOMUXC0->PCR0_IOMUXCARRAY2[i] = 0; /* Set to Analog/HiZ state */
        }
    }
}

void APP_PowerPreSwitchHook(lpm_rtd_power_mode_e targetMode)
{
    uint32_t setting;

    if ((LPM_PowerModeActive != targetMode))
    {
        /* Wait for debug console output finished. */
        while (!(kLPUART_TransmissionCompleteFlag & LPUART_GetStatusFlags((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR)))
        {
        }
        DbgConsole_Deinit();
        /*
         * Set pin for current leakage.
         * Debug console RX pin: Set to pinmux to analog.
         * Debug console TX pin: Set to pinmux to analog.
         */
        IOMUXC_SetPinMux(IOMUXC_PTA10_LPUART1_TX, 0);
        IOMUXC_SetPinConfig(IOMUXC_PTA10_LPUART1_TX, 0);
        IOMUXC_SetPinMux(IOMUXC_PTA11_LPUART1_RX, 0);
        IOMUXC_SetPinConfig(IOMUXC_PTA11_LPUART1_RX, 0);

        if (LPM_PowerModePowerDown == targetMode || LPM_PowerModeDeepSleep == targetMode)
        {
            APP_Suspend();
        }
        else if (LPM_PowerModeDeepPowerDown == targetMode)
        {
            APP_DisableGPIO();
            /* If PTB12 is wakeup source, set to WUU0_P24 */
            if ((WUU0->PE1 & WUU_PE1_WUPE4_MASK) != 0)
            {
                /* Disable interrupt temperarily to prevent glitch
                 * interrupt during switching IOMUXC pin selection
                 */
                setting = WUU0->PE1 & WUU_PE1_WUPE4_MASK;
                WUU0->PE1 &= !WUU_PE1_WUPE4_MASK;

                IOMUXC0->PCR0_IOMUXCARRAY0[7] = IOMUXC0_PCR0_IOMUXCARRAY0_MUX(13);

                WUU0->PE1 |= setting;
            }

            if ((WUU0->PE2 & WUU_PE2_WUPE25_MASK) != 0)
            {
                /* Disable interrupt temperarily to prevent glitch
                 * interrupt during switching IOMUXC pin selection
                 */
                setting = WUU0->PE2 & WUU_PE2_WUPE25_MASK;
                WUU0->PE2 &= !WUU_PE2_WUPE25_MASK;

                IOMUXC0->PCR0_IOMUXCARRAY1[13] = IOMUXC0_PCR0_IOMUXCARRAY1_MUX(13);

                WUU0->PE2 |= setting;
            }
            /* Cleare any potential interrupts before enter Deep Power Down */
            WUU0->PF = WUU0->PF;
        }
    }
}

void APP_PowerPostSwitchHook(lpm_rtd_power_mode_e targetMode, bool result)
{
    if (LPM_PowerModeActive != targetMode)
    {
        if (LPM_PowerModePowerDown == targetMode || LPM_PowerModeDeepSleep == targetMode)
        {
            APP_Resume(result);
        }

        /*
         * Debug console RX pin was set to disable for current leakage, need to re-configure pinmux.
         * Debug console TX pin was set to disable for current leakage, need to re-configure pinmux.
         */
        IOMUXC_SetPinMux(IOMUXC_PTA10_LPUART1_TX, 0U);
        IOMUXC_SetPinConfig(IOMUXC_PTA10_LPUART1_TX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
        IOMUXC_SetPinMux(IOMUXC_PTA11_LPUART1_RX, 0U);
        IOMUXC_SetPinConfig(IOMUXC_PTA11_LPUART1_RX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);

        BOARD_InitDebugConsole();
    }
    PRINTF("== Power switch %s ==\r\n", result ? "OK" : "FAIL");
    /* Reinitialize TRDC */
    if (AD_CurrentMode == AD_PD)
    {
        BOARD_SetTrdcGlobalConfig();
    }
    else if (AD_CurrentMode == AD_DPD)
    {
        BOARD_SetTrdcAfterApdReset();
    }
}
static inline const char *APP_GetAllowCombiName(allow_combi_e allow)
{
    switch (allow)
    {
        GEN_CASE_ENUM_NAME(MODE_COMBI_NO);
        GEN_CASE_ENUM_NAME(MODE_COMBI_YES);
        default:
            return (char *)"WRONG_MODE_COMBI";
    }
}
static inline const char *APP_GetRtdPwrModeName(lpm_rtd_power_mode_e mode)
{
    switch (mode)
    {
        GEN_CASE_ENUM_NAME(LPM_PowerModeActive);
        GEN_CASE_ENUM_NAME(LPM_PowerModeWait);
        GEN_CASE_ENUM_NAME(LPM_PowerModeStop);
        GEN_CASE_ENUM_NAME(LPM_PowerModeSleep);
        GEN_CASE_ENUM_NAME(LPM_PowerModeDeepSleep);
        GEN_CASE_ENUM_NAME(LPM_PowerModePowerDown);
        GEN_CASE_ENUM_NAME(LPM_PowerModeDeepPowerDown);
        default:
            return (char *)"WRONG_LPM_RTD_PowerMode";
    }
}

static inline const char *APP_GetAdPwrModeName(lpm_ad_power_mode_e mode)
{
    switch (mode)
    {
        GEN_CASE_ENUM_NAME(AD_UNKOWN);
        GEN_CASE_ENUM_NAME(AD_ACT);
        GEN_CASE_ENUM_NAME(AD_PD);
        GEN_CASE_ENUM_NAME(AD_DPD);
        default:
            return (char *)"WRONG_LPM_AD_PowerMode";
    }
}

static allow_combi_e APP_GetModeAllowCombi(lpm_ad_power_mode_e ad_mode, lpm_rtd_power_mode_e rtd_mode)
{
    int i               = 0;
    allow_combi_e allow = MODE_COMBI_NO;
    ;

    if (BOARD_IsSingleBootType())
    {
        for (i = 0; i < ARRAY_SIZE(mode_combi_array_for_single_boot); i++)
        {
            if ((mode_combi_array_for_single_boot[i].rtd_mode == rtd_mode) &&
                (mode_combi_array_for_single_boot[i].ad_mode == ad_mode))
            {
                allow = mode_combi_array_for_single_boot[i].allow_combi;
                break;
            }
        }
    }
    else
    {
        for (i = 0; i < ARRAY_SIZE(mode_combi_array_for_dual_or_lp_boot); i++)
        {
            if ((mode_combi_array_for_dual_or_lp_boot[i].rtd_mode == rtd_mode) &&
                (mode_combi_array_for_dual_or_lp_boot[i].ad_mode == ad_mode))
            {
                allow = mode_combi_array_for_dual_or_lp_boot[i].allow_combi;
                break;
            }
        }
    }

    return allow;
}

static void APP_ShowModeCombi(void)
{
    int i = 0;

    PRINTF("###############################################\r\n");
    PRINTF("For Single Boot Type\r\n");
    for (i = 0; i < ARRAY_SIZE(mode_combi_array_for_single_boot); i++)
    {
        PRINTF("%s + %s: %s\r\n", APP_GetAdPwrModeName(mode_combi_array_for_single_boot[i].ad_mode),
               APP_GetRtdPwrModeName(mode_combi_array_for_single_boot[i].rtd_mode),
               APP_GetAllowCombiName(mode_combi_array_for_single_boot[i].allow_combi));
    }
    PRINTF("###############################################\r\n");
    PRINTF("\r\n");
    PRINTF("\r\n");

    PRINTF("###############################################\r\n");
    PRINTF("For Dual Boot Type/Low Power Boot Type\r\n");
    for (i = 0; i < ARRAY_SIZE(mode_combi_array_for_dual_or_lp_boot); i++)
    {
        PRINTF("%s + %s: %s\r\n", APP_GetAdPwrModeName(mode_combi_array_for_dual_or_lp_boot[i].ad_mode),
               APP_GetRtdPwrModeName(mode_combi_array_for_dual_or_lp_boot[i].rtd_mode),
               APP_GetAllowCombiName(mode_combi_array_for_dual_or_lp_boot[i].allow_combi));
    }
    PRINTF("###############################################\r\n");
}

/* WUU0 interrupt handler. */
void APP_WUU0_IRQHandler(void)
{
    bool wakeup = false;

    if (WUU_GetInternalWakeupModuleFlag(WUU0, WUU_MODULE_LPTMR1))
    {
        /* Woken up by LPTMR, then clear LPTMR flag. */
        LPTMR_ClearStatusFlags(LPTMR1, kLPTMR_TimerCompareFlag);
        LPTMR_DisableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
        LPTMR_StopTimer(LPTMR1);
        wakeup = true;
    }

#if 0
    if (WUU_GetExternalWakeupPinFlag(WUU0, WUU_WAKEUP_PIN_IDX))
    {
        /* Woken up by external pin. */
        WUU_ClearExternalWakeupPinFlag(WUU0, WUU_WAKEUP_PIN_IDX);
        wakeup = true;
    }
#else
    int32_t i = 0;

    /* Go through wakeup pins list and reconfigure pinmux */
    for (i = 0; i < ARRAY_SIZE(g_Wakeup_Pins); ++i)
    {
        uint32_t gpio_pin_id = 0;
        uint8_t wuu_index = 0;

        gpio_pin_id = g_Wakeup_Pins[i];

        wuu_index = APP_IO_GetWUUPinByIoId(gpio_pin_id);
        if (wuu_index == WUU_WAKEUP_LSMPIN_IDX)
        {
            if (WUU_GetExternalWakeupPinFlag(WUU0, wuu_index))
                {
                    /* Woken up by external pin. */
                    WUU_ClearExternalWakeupPinFlag(WUU0, wuu_index);
                    APP_SRTM_WakeupCA35();
                    wakeup = true;
                }
        }
        else if (WUU_GetExternalWakeupPinFlag(WUU0, wuu_index))
        {
            /* Woken up by external pin. */
            WUU_ClearExternalWakeupPinFlag(WUU0, wuu_index);
            wakeup = true;
        }
    }

#endif

    if (WUU_GetInternalWakeupModuleFlag(WUU0, WUU_MODULE_SYSTICK))
    {
        /* Woken up by Systick LPTMR, then clear LPTMR flag. */
        LPTMR_ClearStatusFlags(SYSTICK_BASE, kLPTMR_TimerCompareFlag);
    }

    if (wakeup)
    {
        xSemaphoreGiveFromISR(s_wakeupSig, NULL);
        portYIELD_FROM_ISR(pdTRUE);
    }
}

/* LPTMR1 interrupt handler. */
void LPTMR1_IRQHandler(void)
{
    bool wakeup = false;

    if (kLPTMR_TimerInterruptEnable & LPTMR_GetEnabledInterrupts(LPTMR1))
    {
        LPTMR_ClearStatusFlags(LPTMR1, kLPTMR_TimerCompareFlag);
        LPTMR_DisableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
        LPTMR_StopTimer(LPTMR1);
        wakeup = true;
    }

    if (wakeup)
    {
        xSemaphoreGiveFromISR(s_wakeupSig, NULL);
        portYIELD_FROM_ISR(pdTRUE);
    }
}

static void APP_IRQDispatcher(IRQn_Type irq, void *param)
{
    switch (irq)
    {
        case WUU0_IRQn:
            APP_WUU0_IRQHandler();
            break;
        case GPIOA_INT0_IRQn:
            if ((1U << GPIO_PIN_IDX(APP_WAKEUP_PIN_ID)) &
                RGPIO_GetPinsInterruptFlags(RGPIO_GetBaseByInstance(GPIO_PORT_IDX(APP_WAKEUP_PIN_ID)),
                                            kRGPIO_InterruptOutput2))
            {
                /* Flag will be cleared by app_srtm.c */
                xSemaphoreGiveFromISR(s_wakeupSig, NULL);
                portYIELD_FROM_ISR(pdTRUE);
            }
        case GPIOB_INT0_IRQn:
            if ((1U << GPIO_PIN_IDX(APP_PIN_LSM6DSO_INT1)) &
                RGPIO_GetPinsInterruptFlags(RGPIO_GetBaseByInstance(GPIO_PORT_IDX(APP_PIN_LSM6DSO_INT1)), kRGPIO_InterruptOutput2))
            {
                /* Flag will be cleared by app_srtm.c */
                xSemaphoreGiveFromISR(s_wakeupSig, NULL);
                portYIELD_FROM_ISR(pdTRUE);
            }
            break;
        default:
            break;
    }
}

/* Get input from user about wakeup timeout. */
static uint32_t APP_GetWakeupTimeout(void)
{
    uint32_t timeout = 0U;
    uint8_t c;

    while (1)
    {
        PRINTF("Select the wake up timeout in seconds.\r\n");
        PRINTF("The allowed range is 1s ~ 999s.\r\n");
        PRINTF("Eg. enter 5 to wake up in 5 seconds.\r\n");
        PRINTF("\r\nWaiting for input timeout value...\r\n\r\n");

        do
        {
            c = GETCHAR();
            if ((c >= '0') && (c <= '9'))
            {
                PRINTF("%c", c);
                timeout = timeout * 10U + c - '0';
            }
            else if ((c == '\r') || (c == '\n'))
            {
                break;
            }
            else
            {
                PRINTF("%c\r\nWrong value!\r\n", c);
                timeout = 0U;
            }
        } while (timeout != 0U && timeout < 100U);

        if (timeout > 0U)
        {
            PRINTF("\r\n");
            break;
        }
    }

    return timeout;
}

/* Get wakeup source by user input. */
static app_wakeup_source_t APP_GetWakeupSource(void)
{
    uint8_t ch;

    while (1)
    {
        PRINTF("Select the wake up source:\r\n");
        PRINTF("Press T for LPTMR - Low Power Timer\r\n");
        PRINTF("Press S for switch/button %s. \r\n", APP_WAKEUP_BUTTON_NAME);

        PRINTF("\r\nWaiting for key press..\r\n\r\n");

        ch = GETCHAR();

        if ((ch >= 'a') && (ch <= 'z'))
        {
            ch -= 'a' - 'A';
        }

        if (ch == 'T')
        {
            return kAPP_WakeupSourceLptmr;
        }
        else if (ch == 'S')
        {
            return kAPP_WakeupSourcePin;
        }
        else
        {
            PRINTF("Wrong value!\r\n");
        }
    }
}

/* Get wakeup timeout and wakeup source. */
static void APP_GetWakeupConfig(app_wakeup_source_t *wakeup_source, uint32_t *wakeup_timeout)
{
    /* Get wakeup source by user input. */
    *wakeup_source = APP_GetWakeupSource();

    if (kAPP_WakeupSourceLptmr == *wakeup_source)
    {
        /* Wakeup source is LPTMR, user should input wakeup timeout value. */
        *wakeup_timeout = APP_GetWakeupTimeout();
        PRINTF("Will wakeup in %d seconds.\r\n", *wakeup_timeout);
    }
    else
    {
        PRINTF("Press %s to wake up.\r\n", APP_WAKEUP_BUTTON_NAME);
    }
}

static void APP_SetWakeupConfig(lpm_rtd_power_mode_e targetMode, app_wakeup_source_t wakeup_source, uint32_t wakeup_timeout)
{
    if (kAPP_WakeupSourceLptmr == wakeup_source)
    {
        LPTMR_SetTimerPeriod(LPTMR1, (1000UL * wakeup_timeout / 16U));
        LPTMR_StartTimer(LPTMR1);
        LPTMR_EnableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
    }

    /* To avoid conflicting access of WUU with SRTM dispatcher, we put the WUU setting into SRTM dispatcher context.*/
    /* If targetMode is PD/DPD, setup WUU. */
    if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
    {
        if (kAPP_WakeupSourceLptmr == wakeup_source)
        {
            /* Set WUU LPTMR1 module wakeup source. */
            APP_SRTM_SetWakeupModule(WUU_MODULE_LPTMR1, kWUU_InternalModuleDMATrigger);
            PCC1->PCC_LPTMR1 &= ~PCC1_PCC_LPTMR1_SSADO_MASK;
            PCC1->PCC_LPTMR1 |= PCC1_PCC_LPTMR1_SSADO(1);
        }
        else
        {
            /* Set PORT and WUU wakeup pin. */
            APP_SRTM_SetWakeupPin(APP_PIN_LSM6DSO_INT1, (uint16_t)kWUU_ExternalPinRisingEdge | 0x100);
            APP_SRTM_SetWakeupPin(APP_WAKEUP_PIN_ID, (uint16_t)WUU_WAKEUP_PIN_TYPE | 0x100);
        }
    }
    else
    {
        /* Set PORT pin. */
        if (kAPP_WakeupSourcePin == wakeup_source)
        {
            uint16_t event = (uint16_t)WUU_WAKEUP_PIN_TYPE;
            /*
             * Need setup SSADO field when gate core, platform, bus clock(RTD clock mode), unless failed to wakeup
             * cortex-m33 by button. Currently will gate core, platform, bus clock when RTD enter Deep Sleep
             * Mode(LPM_SystemDeepSleep->RTDCMC_SetClockMode), so setup SSADO field here for Deep Sleep Mode.
             */
            if (LPM_PowerModeDeepSleep == targetMode)
            {
                PCC1->PCC_RGPIOA &= ~PCC1_PCC_RGPIOA_SSADO_MASK;
                PCC1->PCC_RGPIOA |= PCC1_PCC_RGPIOA_SSADO(1);

                PCC1->PCC_RGPIOB &= ~PCC1_PCC_RGPIOB_SSADO_MASK;
                PCC1->PCC_RGPIOB |= PCC1_PCC_RGPIOB_SSADO(1);
                event |= 0x100; /* enable wakeup flag */
            }
            APP_SRTM_SetWakeupPin(APP_PIN_LSM6DSO_INT1, (uint16_t)kWUU_ExternalPinRisingEdge | 0x100);
            APP_SRTM_SetWakeupPin(APP_WAKEUP_PIN_ID, event);
        }
    }
}

static void APP_ClearWakeupConfig(lpm_rtd_power_mode_e targetMode, app_wakeup_source_t wakeup_source)
{
    if (kAPP_WakeupSourcePin == wakeup_source)
    {
        APP_SRTM_SetWakeupPin(APP_PIN_LSM6DSO_INT1, (uint16_t)kWUU_ExternalPinDisable);
        APP_SRTM_SetWakeupPin(APP_WAKEUP_PIN_ID, (uint16_t)kWUU_ExternalPinDisable);
    }
    else if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
    {
        APP_SRTM_SetWakeupModule(WUU_MODULE_LPTMR1, false);
    }
}

/* Power Mode Switch task */
void PowerModeSwitchTask(void *pvParameters)
{
    lptmr_config_t lptmrConfig;
    lpm_rtd_power_mode_e targetPowerMode;
    uint32_t freq = 0U;
    uint8_t ch;

    /* As IRQ handler main entry locates in app_srtm.c to support services, here need an entry to handle application
     * IRQ events.
     */
    APP_SRTM_SetIRQHandler(APP_IRQDispatcher, NULL);
    /* Add Systick as Power Down wakeup source, depending on SYSTICK_WUU_WAKEUP value. */
    APP_SRTM_SetWakeupModule(WUU_MODULE_SYSTICK, SYSTICK_WUU_WAKEUP);

    /* Setup LPTMR. */
    LPTMR_GetDefaultConfig(&lptmrConfig);
    lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1; /* Use RTC 1KHz as clock source. */
    lptmrConfig.bypassPrescaler      = false;
    lptmrConfig.value                = kLPTMR_Prescale_Glitch_3; /* Divide clock source by 16. */
    LPTMR_Init(LPTMR1, &lptmrConfig);
    NVIC_SetPriority(LPTMR1_IRQn, APP_LPTMR1_IRQ_PRIO);

    EnableIRQ(LPTMR1_IRQn);

    SIM_SEC->DGO_GP10  = 2;
    SIM_SEC->DGO_CTRL1 = SIM_SEC_DGO_CTRL1_UPDATE_DGO_GP10_MASK;
    /* Wait DGO GP0 updated */
    while ((SIM_SEC->DGO_CTRL1 & SIM_SEC_DGO_CTRL1_WR_ACK_DGO_GP10_MASK) == 0)
    {
    }
    /* Clear DGO GP0 ACK and UPDATE bits */
    SIM_SEC->DGO_CTRL1 =
        (SIM_SEC->DGO_CTRL1 & ~(SIM_SEC_DGO_CTRL1_UPDATE_DGO_GP10_MASK)) | SIM_SEC_DGO_CTRL1_WR_ACK_DGO_GP10_MASK;

    SIM_SEC->DGO_GP11  = 1; // PTB range to 1.8V
    SIM_SEC->DGO_CTRL1 = SIM_SEC_DGO_CTRL1_UPDATE_DGO_GP11_MASK;
    /* Wait DGO GP0 updated */
    while ((SIM_SEC->DGO_CTRL1 & SIM_SEC_DGO_CTRL1_WR_ACK_DGO_GP11_MASK) == 0)
    {
    }
    /* Clear DGO GP0 ACK and UPDATE bits */
    SIM_SEC->DGO_CTRL1 =
        (SIM_SEC->DGO_CTRL1 & ~(SIM_SEC_DGO_CTRL1_UPDATE_DGO_GP11_MASK)) | SIM_SEC_DGO_CTRL1_WR_ACK_DGO_GP11_MASK;

    SIM_RTD->PTC_COMPCELL = 0x0; // PTC compensation off

    for (;;)
    {
        freq = CLOCK_GetFreq(kCLOCK_Cm33CorePlatClk);
        PRINTF("\r\n####################  Power Mode Switch Task ####################\n\r\n");
        PRINTF("    Build Time: %s--%s \r\n", __DATE__, __TIME__);
        PRINTF("    Core Clock: %dHz \r\n", freq);
        PRINTF("    Boot Type: %s \r\n", BOARD_GetBootTypeName());
        PRINTF("\r\nSelect the desired operation \n\r\n");
        PRINTF("Press  %c to enter: Active mode\r\n", kAPP_PowerModeActive);
        PRINTF("Press  %c to enter: Cortex M33 Wait mode\r\n", kAPP_PowerModeWait);
        PRINTF("Press  %c to enter: Cortex M33 STOP mode\r\n", kAPP_PowerModeStop);
        PRINTF("Press  %c to enter: Sleep mode\r\n", kAPP_PowerModeSleep);
        PRINTF("Press  %c to enter: Deep Sleep mode\r\n", kAPP_PowerModeDeepSleep);
        PRINTF("Press  %c to enter: Power Down(PD) mode\r\n", kAPP_PowerModePowerDown);
        PRINTF("Press  %c to enter: Deep Power Down(DPD) mode\r\n", kAPP_PowerModeDeepPowerDown);
        PRINTF("Press  W for wake up CA35 core from PD/DPD mode\r\n");
        PRINTF("Press  T for reboot CA35 core\r\n");
        PRINTF("Press  U for shutdown CA35 core.\r\n");
        PRINTF("Press  V for boot CA35 core.\r\n");
        PRINTF("Press  S for showing supported LPM Mode Combination.\r\n");
        PRINTF("Press  H for testing Pedometer.\r\n");
        PRINTF("Press  I for testing HeartRate.\r\n");
        PRINTF("Press  J for testing SpO2.\r\n");
        PRINTF("Press  K for testing Temperature.\r\n");
        PRINTF("Press  L for dumping MAX30101 registers\r\n");
        PRINTF("\r\nWaiting for power mode select..\r\n\r\n");

        /* Wait for user response */
        do
        {
            ch = GETCHAR();
        } while ((ch == '\r') || (ch == '\n'));

        if ((ch >= 'a') && (ch <= 'z'))
        {
            ch -= 'a' - 'A';
        }
        targetPowerMode = (lpm_rtd_power_mode_e)(ch - 'A');
        if (targetPowerMode <= LPM_PowerModeDeepPowerDown)
        {
            uint32_t wakeupTimeout = 0;           /* Wakeup timeout. (Unit: Second) */
            app_wakeup_source_t wakeupSource; /* Wakeup source.                 */

            if (targetPowerMode == s_curMode)
            {
                /* Same mode, skip it */
                continue;
            }
            if (APP_GetModeAllowCombi(AD_CurrentMode, targetPowerMode) == MODE_COMBI_NO)
            {
                PRINTF("Not support the mode combination: %s + %s\r\n", APP_GetAdPwrModeName(AD_CurrentMode),
                       APP_GetRtdPwrModeName(targetPowerMode));
                continue;
            }
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
            /* In Non-blocking mode, IRQ will prevent M33 from entering WFI.
             * So we will disable UART IRQ.
             * And when we set Power Mode, system will try to enter low power, as lpuart will pending for input.
             * Thus we should set Power Mode after getting wakeup configs.
             */
            if (!LPM_HandleTaskHooks(s_curMode, targetPowerMode))
            {
                LPM_HandleTaskHooks(targetPowerMode, s_curMode);
                PRINTF("Some task doesn't allow to enter mode %s\r\n", s_modeNames[targetPowerMode]);
            }
            else
            {
                APP_GetWakeupConfig(&wakeupSource, &wakeupTimeout);
                APP_SetWakeupConfig(targetPowerMode, wakeupSource, wakeupTimeout);
                LPM_SetPowerMode_Directly(targetPowerMode);
                xSemaphoreTake(s_wakeupSig, portMAX_DELAY);
                /* The call might be blocked by SRTM dispatcher task. Must be called after power mode reset. */
                APP_ClearWakeupConfig(targetPowerMode, wakeupSource);
            }
#else
            if (!LPM_SetPowerMode(targetPowerMode))
            {
                PRINTF("Some task doesn't allow to enter mode %s\r\n", s_modeNames[targetPowerMode]);
            }
            else /* Idle task will handle the low power state. */
            {
                APP_GetWakeupConfig(&wakeupSource, &wakeupTimeout);
                APP_SetWakeupConfig(targetPowerMode, wakeupSource, wakeupTimeout);
                APP_CheckPedometerInterrupt();
                APP_SuspendTaskForWakeup();
                /* The call might be blocked by SRTM dispatcher task. Must be called after power mode reset. */
                APP_ClearWakeupConfig(targetPowerMode, wakeupSource);
            }
#endif
        }
        else if ('W' == ch)
        {
            APP_SRTM_WakeupCA35();
        }
        else if ('T' == ch)
        {
            APP_RebootCA35();
        }
        else if ('U' == ch)
        {
            APP_ShutdownCA35();
        }
        else if ('V' == ch)
        {
            option_v_boot_flag = true;
            APP_BootCA35();
        }
        else if ('S' == ch)
        {
            APP_ShowModeCombi();
        }
        else if ('H' == ch)
        {
            APP_ShowPedometer();
        }
        else if ('I' == ch)
        {
            APP_ShowHeartRate();
        }
        else if ('J' == ch)
        {
            APP_ShowSpO2();
        }
        else if ('K' == ch)
        {
            APP_ShowTemperature();
        }
        else if ('L' == ch)
        {
            APP_DumpMAX30101Regs();
        }
        else
        {
            PRINTF("Invalid command %c[0x%x]\r\n", ch, ch);
        }
        /*update Mode state*/
        s_curMode = LPM_PowerModeActive;
        PRINTF("\r\nNext loop\r\n");
    }
}

void vApplicationMallocFailedHook(void)
{
    PRINTF("Malloc Failed!!!\r\n");
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint32_t irqMask;
    lpm_rtd_power_mode_e targetPowerMode;
    /* lpm_rtd_power_mode_e targetMode; */
    upwr_pwm_param_t param;
    bool result;

    /* targetMode = LPM_GetPowerMode(); */

    /* Workround for PD/DPD exit fail if sleep more than 1 second */
    /* if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode)) */
    {
        param.R              = 0;
        param.B.DPD_ALLOW    = 0;
        param.B.DSL_DIS      = 0;
        param.B.SLP_ALLOW    = 0;
        param.B.DSL_BGAP_OFF = 1;
        param.B.DPD_BGAP_ON  = 0;

        UPOWER_SetPwrMgmtParam(&param);
    }

    irqMask = DisableGlobalIRQ();

    /* Only when no context switch is pending and no task is waiting for the scheduler
     * to be unsuspended then enter low power entry.
     */
    if (eTaskConfirmSleepModeStatus() != eAbortSleep)
    {
        targetPowerMode = LPM_GetPowerMode();
        if (targetPowerMode != LPM_PowerModeActive)
        {
            /* Only wait when target power mode is not running */
            APP_PowerPreSwitchHook(targetPowerMode);
            result = LPM_WaitForInterrupt((uint64_t)1000 * xExpectedIdleTime / configTICK_RATE_HZ);
            APP_PowerPostSwitchHook(targetPowerMode, result);
        }
    }
    EnableGlobalIRQ(irqMask);
}

/* Called in PowerModeSwitchTask */
static bool APP_LpmListener(lpm_rtd_power_mode_e curMode, lpm_rtd_power_mode_e newMode, void *data)
{
    PRINTF("WorkingTask %d: Transfer from %s to %s\r\n", (uint32_t)data, s_modeNames[curMode], s_modeNames[newMode]);

    /* Do necessary preparation for this mode change */

    return true; /* allow this switch */
}

/*!
 * @brief simulating working task.
 */
static void WorkingTask(void *pvParameters)
{
    LPM_RegisterPowerListener(APP_LpmListener, pvParameters);

    for (;;)
    {
        /* Use App task logic to replace vTaskDelay */
        PRINTF("Task %d is working now\r\n", (uint32_t)pvParameters);
        vTaskDelay(portMAX_DELAY);
    }
}

/*! @brief Main function */
int main(void)
{
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_DMA0);
    UPOWER_ReduceBuck23VoltInSTBY();

    CLOCK_SetIpSrcDiv(kCLOCK_Tpm0, kCLOCK_Pcc1BusIpSrcCm33Bus, 1U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c1, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    /* Use Pll1Pfd2Div clock source 12.288MHz. */
    CLOCK_SetIpSrc(kCLOCK_Sai0, kCLOCK_Cm33SaiClkSrcPll1Pfd2Div);

    CLOCK_EnableClock(kCLOCK_Dma0Ch16);
    CLOCK_EnableClock(kCLOCK_Dma0Ch17);
    CLOCK_EnableClock(kCLOCK_RgpioA);
    CLOCK_EnableClock(kCLOCK_RgpioB);
    CLOCK_EnableClock(kCLOCK_RgpioC);
    CLOCK_EnableClock(kCLOCK_Wuu0);
    CLOCK_EnableClock(kCLOCK_Bbnsm);

    RESET_PeripheralReset(kRESET_Sai0);
    RESET_PeripheralReset(kRESET_Lpi2c0);
    RESET_PeripheralReset(kRESET_Lpi2c1);
    RESET_PeripheralReset(kRESET_Tpm0);

    APP_SRTM_Init();

    /* If RTD reset is due to DPD exit, should go different flow here */
    if (CMC_RTD->SSRS & CMC_SSRS_WAKEUP_MASK)
    {
        CMC_RTD->SSRS = CMC_SSRS_WAKEUP_MASK;
        /* Assume that Application Domain is entered Deep Power Down Mode */
        AD_CurrentMode = AD_DPD;
        /*
         * AD is also in Deep Power Down mode when RTD is in Deep Power Down Mode.
         * AD/RTD exiting from Deep Power Down Mode is same with cold boot flow.
         * So don't need setup TRDC when RTD exit from Deep Power Down mode.
         *
         */
        // BOARD_SetTrdcAfterApdReset();
        MU_Init(MU0_MUA);
        MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
    }
    else
    {
        APP_SRTM_StartCommunication();
    }

    LPM_Init();

    s_wakeupSig = xSemaphoreCreateBinary();

    xTaskCreate(PowerModeSwitchTask, "Main Task", 512U, NULL, tskIDLE_PRIORITY + 1U, NULL);
    xTaskCreate(WorkingTask, "Working Task", configMINIMAL_STACK_SIZE, (void *)1, tskIDLE_PRIORITY + 2U, NULL);

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Application should never reach this point. */
    for (;;)
    {
    }
}
