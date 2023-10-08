/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <math.h>
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
#include "lpm.h"
#include "app_srtm.h"
#include "sensor_hub_rtos.h"
#include "fsl_rtd_cmc.h"
#include "fsl_sentinel.h"
#include "fsl_rgpio.h"
#include "fsl_wuu.h"

#include "fsl_iomuxc.h"
#include "fsl_lpuart.h"
#include "fsl_reset.h"

#include "pwm_app.h"
#include "gpio_app.h"
#include "lpspi_app.h"
#include "sensor_app.h"

#include "gui_paint.h"
#include "fonts.h"
/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
#define APP_DEBUG_UART_BAUDRATE       (115200U)             /* Debug console baud rate. */
#define APP_DEBUG_UART_DEFAULT_CLKSRC kCLOCK_IpSrcSircAsync /* SCG SIRC clock. */

/* LPTMR0 is WUU internal module 0. */
#define WUU_MODULE_SYSTICK WUU_MODULE_LPTMR0
/* Allow systick to be a wakeup source in Power Down mode. */
#define SYSTICK_WUU_WAKEUP (false)

#define APP_LPTMR1_IRQ_PRIO (5U)
#define WUU_WAKEUP_PIN_IDX     (24U) /* WUU0_P24 used for RTD Button2 (SW8) */
#define WUU_WAKEUP_PIN_TYPE    kWUU_ExternalPinFallingEdge

#define MAX_D_SIZE              2U
#define MAX_XYZ_SIZE            9U
#define MAX_P_SIZE              15U
#define MAX_T_SIZE              11U

#define PI                      3.1415926

typedef enum _app_wakeup_source
{
    kAPP_WakeupSourceLptmr, /*!< Wakeup by LPTMR.        */
    kAPP_WakeupSourcePin    /*!< Wakeup by external pin. */
} app_wakeup_source_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
extern void APP_PowerPreSwitchHook(lpm_power_mode_t targetMode);
extern void APP_PowerPostSwitchHook(lpm_power_mode_t targetMode, bool result);
extern void APP_SRTM_WakeupCA35(void);
extern void APP_RebootCA35(void);
extern void APP_ShutdownCA35(void);
extern void APP_BootCA35(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint32_t s_wakeupTimeout;           /* Wakeup timeout. (Unit: Second) */
static app_wakeup_source_t s_wakeupSource; /* Wakeup source.                 */
static SemaphoreHandle_t s_wakeupSig;
static const char *s_modeNames[] = {"RUN", "WAIT", "STOP", "Sleep", "Deep Sleep", "Power Down", "Deep Power Down"};
extern enum AD_LPMode AD_CurrentMode;
extern bool option_v_boot_flag;
extern lpm_power_mode_t s_curMode;
static volatile bool sensorhub_flag = true;
static bool sleep_signal = 0; // 0: wakeup 1: sleep

/*******************************************************************************
 * Function Code
 ******************************************************************************/
static uint32_t iomuxBackup[25 + 16 + 24]; /* Backup 25 PTA, 16 PTB and 24 PTC IOMUX registers */
static uint32_t gpioICRBackup[25 + 16 + 24];


void APP_Suspend(void)
{
    uint32_t i;
    uint32_t backupIndex;

    backupIndex = 0;

    /* Backup PTA IOMUXC and GPIOA ICR registers then disable */
    for (i = 0; i <= 24; i++)
    {
        iomuxBackup[backupIndex] = IOMUXC0->PCR0_IOMUXCARRAY0[i];

        gpioICRBackup[backupIndex] = GPIOA->ICR[i];

        GPIOA->ICR[i] = 0; /* disable interrupts */

//        if (!sensorhub_flag || (6 != i && 15 != i && 18 != i && 21 != i && 22 != i && 24 != i))
        if (!sensorhub_flag || ( 18 != i )) //PWM PIN
        {
            IOMUXC0->PCR0_IOMUXCARRAY0[i] = 0;
        }
        backupIndex++;
    }

    /* Backup PTB IOMUXC and GPIOB ICR registers then disable */
    for (i = 0; i <= 15; i++)
    {
        iomuxBackup[backupIndex] = IOMUXC0->PCR0_IOMUXCARRAY1[i];

        gpioICRBackup[backupIndex] = GPIOB->ICR[i];
        if (i != 4) //SENSOR LSM6SDO PIN
            GPIOB->ICR[i] = 0; /* disable interrupts */

        if ((i != 10) && (i != 11) && (i != 4))

        {
            IOMUXC0->PCR0_IOMUXCARRAY1[i] = 0;
        }
        backupIndex++;
    }

    /* Backup PTC IOMUXC and GPIOC ICR registers then disable */
    for (i = 0; i <= 23; i++)
    {
        iomuxBackup[backupIndex] = IOMUXC0->PCR0_IOMUXCARRAY2[i];

        gpioICRBackup[backupIndex] = GPIOC->ICR[i];

        GPIOC->ICR[i] = 0; /* disable interrupts */

        IOMUXC0->PCR0_IOMUXCARRAY2[i] = 0;
        backupIndex++;
    }

    if (!sensorhub_flag)
    {
        /* Cleare any potential interrupts before enter Power Down */
        WUU0->PF = WUU0->PF;

        /* Save SRTM context */
        APP_SRTM_Suspend();
    }
}

static void APP_Resume(bool resume)
{
    uint32_t i;
    uint32_t backupIndex;

    backupIndex = 0;

    /* Restore PTA IOMUXC and GPIOA ICR registers */
    for (i = 0; i <= 24; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY0[i] = iomuxBackup[backupIndex];
        GPIOA->ICR[i]                 = gpioICRBackup[backupIndex];
        backupIndex++;
    }

    /* Restore PTB IOMUXC and GPIOB ICR registers */
    for (i = 0; i <= 15; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY1[i] = iomuxBackup[backupIndex];
        GPIOB->ICR[i]                 = gpioICRBackup[backupIndex];
        backupIndex++;
    }

    /* Restore PTC IOMUXC and GPIOC ICR registers */
    for (i = 0; i <= 23; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY2[i] = iomuxBackup[backupIndex];
        GPIOC->ICR[i]                 = gpioICRBackup[backupIndex];
        backupIndex++;
    }

    if (!sensorhub_flag)
    {
        EnableIRQ(WUU0_IRQn);

        APP_SRTM_Resume(resume);
    }
}

void APP_PowerPreSwitchHook(lpm_power_mode_t targetMode)
{
    uint32_t setting;

    if ((LPM_PowerModeRun != targetMode))
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
            /* If PTB12 is wakeup source, set to WUU0_P24 */
            if ((WUU0->PE2 & WUU_PE2_WUPE24_MASK) != 0)
            {
                /* Disable interrupt temperarily to prevent glitch
                 * interrupt during switching IOMUXC pin selection
                 */
                setting = WUU0->PE2 & WUU_PE2_WUPE24_MASK;
                WUU0->PE2 &= !WUU_PE2_WUPE24_MASK;

                IOMUXC0->PCR0_IOMUXCARRAY1[12] = IOMUXC0_PCR0_IOMUXCARRAY0_MUX(13);

                WUU0->PE2 |= setting;
            }

            /* Cleare any potential interrupts before enter Deep Power Down */
            WUU0->PF = WUU0->PF;
        }
    }
}

void APP_PowerPostSwitchHook(lpm_power_mode_t targetMode, bool result)
{
    if (LPM_PowerModeRun != targetMode)
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
    if (LPM_PowerModePowerDown == targetMode)
    {
        BOARD_SetTrdcGlobalConfig(); /* Reinitialize TRDC */
    }
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

    if (WUU_GetExternalWakeupPinFlag(WUU0, WUU_WAKEUP_PIN_IDX))
    {
        /* Woken up by external pin. */
        WUU_ClearExternalWakeupPinFlag(WUU0, WUU_WAKEUP_PIN_IDX);
        wakeup = true;
    }

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
    bool sleep = false;
    
    if (kLPTMR_TimerInterruptEnable & LPTMR_GetEnabledInterrupts(LPTMR1))
    {
        LPTMR_ClearStatusFlags(LPTMR1, kLPTMR_TimerCompareFlag);
        LPTMR_DisableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
        LPTMR_StopTimer(LPTMR1);
        sleep = true;
        PRINTF("kLPTMR_TimerInterruptEnable & LPTMR_GetEnabledInterrupts(LPTMR1)\r\n");
    }
    PRINTF("LPTMR HANDLER.\r\n");
    

    if (sleep)
    {
        sleep_signal = 1;
    }
}

static void APP_IRQDispatcher(IRQn_Type irq, void *param)
{
    switch (irq)
    {
        case WUU0_IRQn:
            APP_WUU0_IRQHandler();
            break;
        case GPIOB_INT0_IRQn:
            if ((1U << APP_PIN_IDX(APP_PIN_PTB4)) &
                RGPIO_GetPinsInterruptFlags(BOARD_SW8_GPIO, kRGPIO_InterruptOutput2))
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


static void APP_SetWakeupConfig(lpm_power_mode_t targetMode)
{
    if (kAPP_WakeupSourceLptmr == s_wakeupSource)
    {
        LPTMR_SetTimerPeriod(LPTMR1, (1000UL * s_wakeupTimeout / 16U));
        LPTMR_StartTimer(LPTMR1);
        LPTMR_EnableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
    }

    /* To avoid conflicting access of WUU with SRTM dispatcher, we put the WUU setting into SRTM dispatcher context.*/
    /* If targetMode is PD/DPD, setup WUU. */
    if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
    {
        if (kAPP_WakeupSourceLptmr == s_wakeupSource)
        {
            /* Set WUU LPTMR1 module wakeup source. */
            APP_SRTM_SetWakeupModule(WUU_MODULE_LPTMR1, kWUU_InternalModuleDMATrigger);
            PCC1->PCC_LPTMR1 &= ~PCC1_PCC_LPTMR1_SSADO_MASK;
            PCC1->PCC_LPTMR1 |= PCC1_PCC_LPTMR1_SSADO(1);
        }
        else
        {
            /* Set PORT and WUU wakeup pin. */
            APP_SRTM_SetWakeupPin(APP_PIN_PTB4, (uint16_t)WUU_WAKEUP_PIN_TYPE | 0x100);
        }
    }
    else
    {
        /* Set PORT pin. */
        if (kAPP_WakeupSourcePin == s_wakeupSource)
        {
            PRINTF(" *******************************************\r\n");
            PRINTF("Go to Deep Sleep mode.\r\n");
            PRINTF(" *******************************************\r\n");
            PCC1->PCC_RGPIOB &= ~PCC1_PCC_RGPIOB_SSADO_MASK;
            PCC1->PCC_RGPIOB |= PCC1_PCC_RGPIOB_SSADO(1); 
            APP_SRTM_SetWakeupPin(APP_PIN_PTB4, (uint16_t)WUU_WAKEUP_PIN_TYPE | 0x100);
        }
    }
}

static void APP_ClearWakeupConfig(lpm_power_mode_t targetMode)
{
    if (kAPP_WakeupSourcePin == s_wakeupSource)
    {
        APP_SRTM_SetWakeupPin(APP_PIN_PTB4, (uint16_t)kWUU_ExternalPinDisable);
    }
    else if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
    {
        APP_SRTM_SetWakeupModule(WUU_MODULE_LPTMR1, false);
    }
}

void vApplicationMallocFailedHook(void)
{
    PRINTF("Malloc Failed!!!\r\n");
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint32_t irqMask;
    lpm_power_mode_t targetPowerMode;
    //lpm_power_mode_t targetMode;
    upwr_pwm_param_t param;
    bool result;

    //targetMode = LPM_GetPowerMode();

    /* Workround for PD/DPD exit fail if sleep more than 1 second */
    //if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
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
        if (targetPowerMode != LPM_PowerModeRun)
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
static bool APP_LpmListener(lpm_power_mode_t curMode, lpm_power_mode_t newMode, void *data)
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

/* Init lptmr */
void InitLptmr()
{
    lptmr_config_t lptmrConfig;

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
    
              
}

/* Limit PTA/PTB voltages */
void LimitVoltage()
{
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

    CGC_RTD->LPOSCCSR &= ~(1<<23); // Unlock
    CGC_RTD->LPOSCCSR &= ~(0x3 << 1); // disabled in powerdown and deep sleep
    while (!(CGC_RTD->LPOSCCSR & (1<<24)));
    PRINTF("SOSC:0x%x\r\n", CGC_RTD->SOSCCSR);
    PRINTF("LPO:0x%x\r\n", CGC_RTD->LPOSCCSR);
}

void Set_String_XYZ(char c, int16_t val, char *str)
{
    if(0 < val)
        sprintf(str, "%c:%d", c, val);
    else
        sprintf(str, "%c:-%d", c, 0 - val);
}

void Set_String_T(int16_t val, char *str)
{
    if(0 < val)
        sprintf(str, "TEMP: %d C", val);
    else
        sprintf(str, "TEMP:-%d C", 0 - val);
}

void Azimuth(int16_t y, int16_t x, char *str_d)
{
    double angle = atan2((double)y, (double)x) * 180 / PI;
//    PRINTF("The angle is %.2lf degrees. \r\n", angle);
    if(-135.0 <= angle && angle < -45.0) sprintf(str_d, "W");
    else if(-45.0 <= angle && angle < 45.0) sprintf(str_d, "N");
    else if(45.0 <= angle && angle < 135.0) sprintf(str_d, "E");
    else sprintf(str_d, "S");
}

/* Sensor Hub task */
void SensorHubTask(void *pvParameters)
{
    int16_t *p_data;
    char str_x[MAX_XYZ_SIZE], str_y[MAX_XYZ_SIZE], str_z[MAX_XYZ_SIZE];
    //FXOS8700CQ is DNP due to EoL, LSM6SDO has no magnetometer data
    //char str_d[MAX_D_SIZE];
    char str_p[MAX_P_SIZE], str_t[MAX_T_SIZE];
    lpm_power_mode_t targetPowerMode;
    uint8_t ch;

    PRINTF("Build Time: %s--%s \r\n", __DATE__, __TIME__);

    //init gpio module
    ModuleInit();

    //init pwm module
    Init_Backlight();

    //init lcd module
    LCD_1in28_test();

    if(!Sensor_Init()){
        PRINTF("Sensor init failed.\r\n");
    }

    InitLptmr();
    
    s_wakeupTimeout = 3;
    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
    LPTMR_SetTimerPeriod(LPTMR1, (1000UL * s_wakeupTimeout / 16U));
    EnableIRQ(LPTMR1_IRQn); 
    LPTMR_StartTimer(LPTMR1);

    PRINTF("Type any key to start sensor hub demo(except enter key)...\r\n");
    /* Wait for user response */
    do
    {
        ch = GETCHAR();
    } while ((ch == '\r') || (ch == '\n'));

    LimitVoltage();

    targetPowerMode = LPM_PowerModeDeepSleep;
    
    for (;;)
    {

        {
            //read data from sensors
            Set_Backlight();
            PRINTF(" Refresh the sensor data;\r\n");
            p_data = Sensor_ReadData();
            Set_String_XYZ('x', *p_data, str_x);
            Set_String_XYZ('y', *(p_data + 1), str_y);
            Set_String_XYZ('z', *(p_data + 2), str_z);
            //FXOS8700CQ is DNP due to EoL, LSM6SDO has no magnetometer data
            //Azimuth(*(p_data + 4), *(p_data + 3), str_d);
            sprintf(str_p, "PRESS: %d kPa", *(p_data + 6));
            Set_String_T(*(p_data + 7), str_t);

            //update data in panel
            //FXOS8700CQ is DNP due to EoL, LSM6SDO has no magnetometer data
            //LCD_1IN28_Update_Block(110, 10, str_d, &Font24, BLACK, LBBLUE, MAX_D_SIZE - 1);
            LCD_1IN28_Update_Block(25, 140, str_x, &Font12, BLACK, BRRED, MAX_XYZ_SIZE - 1);
            LCD_1IN28_Update_Block(95, 140, str_y, &Font12, BLACK, BRRED, MAX_XYZ_SIZE - 1);
            LCD_1IN28_Update_Block(165, 140, str_z, &Font12, BLACK, BRRED, MAX_XYZ_SIZE - 1);
            LCD_1IN28_Update_Block(45, 170, str_p, &Font16, BLACK, YELLOW, MAX_P_SIZE - 1);
            LCD_1IN28_Update_Block(65, 200, str_t, &Font16, BLACK, CYAN, MAX_T_SIZE - 1);

            s_wakeupSource = kAPP_WakeupSourcePin;
                      
            if (sleep_signal == 1){ //receive sleep signal
            
            LPM_SetPowerMode(targetPowerMode);
            APP_SetWakeupConfig(targetPowerMode);
            Set_Backlight_Off();
            xSemaphoreTake(s_wakeupSig, portMAX_DELAY);
            PRINTF(" *******************************************\r\n");
            PRINTF("Receive the signal, wake up.\r\n");
            PRINTF(" *******************************************\r\n");
            /* The call might be blocked by SRTM dispatcher task. Must be called after power mode reset. */
            APP_ClearWakeupConfig(targetPowerMode);
            
            /* Enable timer again */
            s_wakeupTimeout = 3;
            LPTMR_EnableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
            LPTMR_SetTimerPeriod(LPTMR1, (1000UL * s_wakeupTimeout / 16U));
            EnableIRQ(LPTMR1_IRQn); 
            LPTMR_StartTimer(LPTMR1);
            
            sleep_signal = 0;
            }

        }

        PRINTF("\r\nSensorHubTask Next loop\r\n");
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

    CLOCK_SetIpSrcDiv(kCLOCK_Tpm0, kCLOCK_Pcc1BusIpSrcCm33Bus, 1U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c1, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    /* Use Pll1Pfd2Div clock source 12.288MHz. */
    CLOCK_SetIpSrc(kCLOCK_Sai0, kCLOCK_Cm33SaiClkSrcPll1Pfd2Div);

    CLOCK_EnableClock(kCLOCK_Dma0Ch16);
    CLOCK_EnableClock(kCLOCK_Dma0Ch17);
    CLOCK_EnableClock(kCLOCK_RgpioA);
    CLOCK_EnableClock(kCLOCK_RgpioB);
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
        BOARD_SetTrdcAfterApdReset();
        MU_Init(MU0_MUA);
        MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
    }
    else
    {
        APP_SRTM_StartCommunication();
    }

    /* Force APD to take LPAV ownership to workaround PD/DPD issue under dual/lp boot  mode */
    SIM_SEC->SYSCTRL0 |= SIM_SEC_SYSCTRL0_LPAV_MASTER_CTRL(1);

    LPM_Init();

    s_wakeupSig = xSemaphoreCreateBinary();

    xTaskCreate(SensorHubTask, "Sensor Hub Task", 512U, NULL, tskIDLE_PRIORITY + 1U, NULL);
    xTaskCreate(WorkingTask, "Working Task", configMINIMAL_STACK_SIZE, (void *)1, tskIDLE_PRIORITY + 2U, NULL);

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Application should never reach this point. */
    for (;;)
    {
    }
}
