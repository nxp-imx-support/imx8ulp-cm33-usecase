/*
 * Copyright 2021-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _APP_SRTM_H_
#define _APP_SRTM_H_

#include "rpmsg_lite.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*
 * AD: Application Domain
 * LP: Low Power
 * Low Power Modes for Application Domain is indroduced in AD_PMCTRL of CMC1:
 * Active,
 * Sleep,
 * Deep Sleep,
 * Partial Active,
 * Power Down(PD),
 * Deep Power Down(DPD),
 * Hold
 */
typedef enum
{
    AD_UNKOWN,
    AD_ACT, /* Note: linux is in idle state(Switch between Active mode and Sleep Mode of APD) */
    AD_PD,  /* Application Domain enter Power Down Mode when linux execute suspend command(echo mem > /sys/power/state,
                 suspend to ram) */
    AD_DPD, /* Application Domian enter Deep Power Down Mode when linux execute poweroff command */
} lpm_ad_power_mode_e;

typedef enum
{
    APP_SRTM_StateRun = 0x0U,
    APP_SRTM_StateLinkedUp,
    APP_SRTM_StateReboot,
    APP_SRTM_StateShutdown,
} app_srtm_state_t;

#define APP_SRTM_SAI      (SAI0)
#define APP_SRTM_SAI_IRQn SAI0_IRQn

#define APP_MS2TICK(ms)       ((ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)
#define APP_DMA_IRQN(channel) (IRQn_Type)((uint32_t)DMA0_0_IRQn + channel)


/* Task priority definition, bigger number stands for higher priority */
#define APP_SRTM_MONITOR_TASK_PRIO    (4U)
#define APP_SRTM_DISPATCHER_TASK_PRIO (3U)

/* IRQ handler priority definition, bigger number stands for lower priority */
#define APP_LPI2C_IRQ_PRIO      (5U)
#define APP_SAI_TX_DMA_IRQ_PRIO (5U)
#define APP_SAI_RX_DMA_IRQ_PRIO (5U)
#define APP_SAI_IRQ_PRIO        (5U)
#define APP_GPIO_IRQ_PRIO       (5U)
#define APP_WUU_IRQ_PRIO        (5U)
#define APP_CMC1_IRQ_PRIO       (5U)
#define APP_BBNSM_IRQ_PRIO      (5U)

/* Define the timeout ms to polling the A Core link up status */
#define APP_LINKUP_TIMER_PERIOD_MS (10U)

/* Define the timeout ms to refresh s400 watchdog timer to keep s400 alive(23 hours) */
#define APP_REFRESH_S400_WDG_TIMER_PERIOD_MS (23 * 60 * 60 * 1000U)

/* Define the timeout ms to send rtc alarm event */
#define APP_RTC_ALM_EVT_TIMER_PERIOD_MS (50U)

/* Define the timeout ms to send sensor tilt wakeup event */
#define APP_SENSOR_TILT_WAKEUP_EVT_TIMER_PERIOD_MS (50U)

#define APP_SENSOR_MAX_IRQ_CHECK_TIMER_PERIOD_MS    (1000U)

#define RPMSG_LITE_SRTM_SHMEM_BASE (VDEV0_VRING_BASE)
#define RPMSG_LITE_SRTM_LINK_ID    (RL_PLATFORM_IMX8ULP_M33_A35_SRTM_LINK_ID)

#define APP_SRTM_I2C_CHANNEL_NAME    "rpmsg-i2c-channel"
#define APP_SRTM_AUDIO_CHANNEL_NAME  "rpmsg-audio-channel"
#define APP_SRTM_KEYPAD_CHANNEL_NAME "rpmsg-keypad-channel"
#define APP_SRTM_IO_CHANNEL_NAME     "rpmsg-io-channel"
#define APP_SRTM_PWM_CHANNEL_NAME    "rpmsg-pwm-channel"
#define APP_SRTM_RTC_CHANNEL_NAME    "rpmsg-rtc-channel"
#define APP_SRTM_LFCL_CHANNEL_NAME   "rpmsg-life-cycle-channel"
#define APP_SRTM_SENSOR_CHANNEL_NAME "rpmsg-sensor-channel"

#define PEER_CORE_ID (1U)

/* I2C service */
#define LPI2C0_BAUDRATE              (400000)
#define I2C_SOURCE_CLOCK_FREQ_LPI2C0 CLOCK_GetIpFreq(kCLOCK_Lpi2c0)

#define LPI2C1_BAUDRATE              (400000)
#define I2C_SOURCE_CLOCK_FREQ_LPI2C1 CLOCK_GetIpFreq(kCLOCK_Lpi2c1)

#define I2C_SWITCH_NONE 1

/* Audio service */
#define APP_SAI_TX_DMA_CHANNEL (16U)
#define APP_SAI_RX_DMA_CHANNEL (17U)

/* Sensor service */
#define APP_PEDOMETER_POLL_DELAY_MIN (500U)                              /* Half second. */
#define APP_PEDOMETER_POLL_DELAY_MAX (3600000U)                          /* 1 hour. */
#define APP_PEDOMETER_SAMPLE_RATE    (50U)                               /* sample 50 times per second. */
#define APP_PEDOMETER_SAMPLE_WINDOW  (1000U / APP_PEDOMETER_SAMPLE_RATE) /* sample every 20ms. */

/* Keypad index */
/* These definition are from linux include/uapi/linux/input-event-codes.h */
#define APP_KEYPAD_INDEX_RESERVED  (0U)
#define APP_KEYPAD_INDEX_VOL_MINUS (114U)
#define APP_KEYPAD_INDEX_VOL_PLUS  (115U)
#define APP_KEYPAD_INDEX_POWER     (116U)

/* WUU module index */
#define WUU_MODULE_LPTMR0 (0U)
#define WUU_MODULE_LPTMR1 (1U)
#define WUU_MODULE_CMP0   (2U)
#define WUU_MODULE_CMP1   (3U)
#define WUU_MODULE_UPOWER (4U)
#define WUU_MODULE_TAMPER (5U)
#define WUU_MODULE_NSRTC  (6U)
#define WUU_MODULE_SRTC   (7U)

/* GPIO */
/* GPIOA_INT0_IRQn, GPIOA_INT1_IRQn, GPIOA_INT2_IRQn, GPIOA_INT3_IRQn */
#define GPIO_INT_NUM (4U)

#define PIN_CANNOT_USE_AS_GPIO  0, 0, 0, 0, 0
#define PIN_FUNC_ID_SIZE        (5U)
#define GPIO_MODULE_NUM         (3U)  /* GPIOA, GPIOB, GPIOC */
#define PIN_NUM_PER_GPIO_MODULE (32U) /* There are 32 pins per gpio module(such as: GPIOA include 32 pins */
#define TOTAL_PINS              (GPIO_MODULE_NUM * PIN_NUM_PER_GPIO_MODULE)

/* according to interrupt vector assignment(Table 7.) in RM */
#define APP_GPIO_INT_SEL (kHAL_GpioInt2)

/* io */
#define IO_AS_INPUT_SHIFT  (0U)
#define IO_AS_INPUT_MASK   (1U << IO_AS_INPUT_SHIFT)
#define IO_AS_OUTPUT_SHIFT (1U)
#define IO_AS_OUTPUT_MASK  (1U << IO_AS_OUTPUT_SHIFT)
#define IO_AS_BUTTON_SHIFT (2U)
#define IO_AS_BUTTON_MASK  ((1U << IO_AS_BUTTON_SHIFT) | IO_AS_INPUT_MASK)

#define IO_AS_INPUT  (IO_AS_INPUT_MASK)
#define IO_AS_OUTPUT (IO_AS_OUTPUT_MASK)
#define IO_AS_BUTTON (IO_AS_BUTTON_MASK)
#define IO_UNKOWN    (0xFFU)

/* use as button(input) */
#define APP_PIN_PTA4         (0x0004U) /* PTA4, ONOFF button */
#define APP_PIN_ONOFF_BTN    (APP_PIN_PTA4)
#define APP_PIN_PTA6         (0x0006U) /* PTA6, VOL+ button */
#define APP_PIN_VOLPLUS_BTN  (APP_PIN_PTA6)
#define APP_PIN_PTA7         (0x0007U) /* PTA7, VOL- button */
#define APP_PIN_VOLMINUS_BTN (APP_PIN_PTA7)

/* use as interrupt(input) */
#define APP_PIN_PTB4             (0x0104) /* PTB4, touch interrupt */
#define APP_PIN_TOUCH_INT        (APP_PIN_PTB4)
#define APP_PIN_PTB13            (0x010D) /* PTB13, lsm6dso int1 interrupt */
#define APP_PIN_LSM6DSO_INT1     (APP_PIN_PTB13)
#define APP_PIN_PTB14            (0x010E) /* PTB14, lsm6dso int2 interrupt */
#define APP_PIN_LSM6DSO_INT2     (APP_PIN_PTB14)
#define APP_PIN_PTA15            (0x0011) /* PTA15, gnss interrupt */
#define APP_PIN_GNSS_INT         (APP_PIN_PTA15)
#define APP_PIN_PTA17            (0x0011) /* PTA17, nfc interrupt */
#define APP_PIN_NFC_INT          (APP_PIN_PTA17)
#define APP_PIN_PTB6             (0x0106) /* PTB6, charger interrupt */
#define APP_PIN_CHARGER_INT      (APP_PIN_PTB6)
#define APP_PIN_PTB5             (0x0105) /* PTB5, charger alert interrupt */
#define APP_PIN_CHARGER_ALRT_INT (APP_PIN_PTB5)
#define APP_PIN_PTB12            (0x010C) /* PTB12, sensor MAX30101 interrupt */
#define APP_PIN_MAX30101_INT     (APP_PIN_PTB12)
#define APP_PIN_PTA16            (0x0010) /* PTA16, sensor MPL3115 interrupt */
#define APP_PIN_MPL3115_INT      (APP_PIN_PTA16)
#define APP_PIN_PTA24            (0x0018) /* PTA24, sensor TSL25403 interrupt */
#define APP_PIN_TSL25403_INT     (APP_PIN_PTA24)
#define APP_PIN_PTC11            (0x020B) /* PTC11, mipi-panel TE interrupt */
#define APP_PIN_AMOLED_TE_INT    (APP_PIN_PTC11)

/* use as output */
#define APP_PIN_PTA19            (0x0013) /* PTA19, Reset GNSS */
#define APP_PIN_RST_GNSS         (APP_PIN_PTA19)
#define APP_PIN_PTC10            (0x020A) /* PTC10, Reset AMOLED Panel */
#define APP_PIN_RST_AMOLED_PANEL (APP_PIN_PTC10)
#define APP_PIN_PTC12            (0x020C) /* PTC12, Reset Touch Panel */
#define APP_PIN_RST_TOUCH_PANEL  (APP_PIN_PTC12)
#define APP_PIN_PTC13            (0x020D) /* PTC13, output to NFC_DWL_REQ of NFC */
#define APP_PIN_NFC_DWL_REQ      (APP_PIN_PTC13)
#define APP_PIN_PTC14            (0x020E) /* PTC14, output to VEN of NFC */
#define APP_PIN_NFC_VEN          (APP_PIN_PTC14)
#define APP_PIN_PTC15            (0x020F) /* PTC15, output to NFC_GPIO2_A0 of NFC */
#define APP_PIN_NFC_GPIO2_A0     (APP_PIN_PTC15)

extern int32_t RPMsg_MU0_A_IRQHandler(void);

typedef void (*app_rpmsg_monitor_t)(struct rpmsg_lite_instance *rpmsgHandle, bool ready, void *param);
typedef void (*app_irq_handler_t)(IRQn_Type irq, void *param);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/* Initialize SRTM contexts */
void APP_SRTM_Init(void);

/* Create RPMsg channel and start SRTM communication */
void APP_SRTM_StartCommunication(void);

/* Set RPMsg channel init/deinit monitor */
void APP_SRTM_SetRpmsgMonitor(app_rpmsg_monitor_t monitor, void *param);

uint8_t APP_Read_I2C_Register(uint8_t busID, uint16_t slaveAddr, uint8_t regIndex);
uint8_t APP_Write_I2C_Register(uint8_t busID, uint16_t slaveAddr, uint8_t regIndex, uint8_t value);

/* Set IRQ handler for application */
void APP_SRTM_SetIRQHandler(app_irq_handler_t handler, void *param);

/* Enable or disable wakeup pin
 * event[7:0]: llwu_external_pin_mode_t
 * event[8]: LLWU wakeup enable
 */
void APP_SRTM_SetWakeupPin(uint16_t ioId, uint16_t event);

/* Enable or disable LLWU wakeup module */
void APP_SRTM_SetWakeupModule(uint32_t module, bool enable);

void APP_SRTM_Suspend(void);
void APP_SRTM_Resume(bool resume);

/* Enable/Disable LPAV DDR function*/
void APP_SRTM_PreCopyDRAMCallback(void);
void APP_SRTM_PostCopyDRAMCallback(void);
void APP_SRTM_DisableLPAV(void);
void APP_SRTM_EnableLPAV(void);

/* Sensor test functions */
void APP_ShowPedometer(void);
void APP_ShowHeartRate(void);
void APP_ShowSpO2(void);
void APP_ShowTemperature(void);
void APP_DumpMAX30101Regs(void);

#if defined(__cplusplus)
}
#endif

#endif /* _APP_SRTM_H_ */
