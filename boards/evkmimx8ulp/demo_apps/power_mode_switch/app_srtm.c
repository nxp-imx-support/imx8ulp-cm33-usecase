/*
 * Copyright 2021-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "fsl_lpi2c_freertos.h"

#include "srtm_dispatcher.h"
#include "srtm_peercore.h"
#include "srtm_pwm_adapter.h"
#include "srtm_pwm_service.h"
#include "srtm_message.h"
#include "srtm_rpmsg_endpoint.h"
#include "srtm_i2c_service.h"

#include "srtm_sensor_service.h"

#include "srtm_sai_edma_adapter.h"
#include "srtm_io_service.h"
#include "srtm_keypad_service.h"
#include "srtm_lfcl_service.h"
#include "srtm_rtc_service.h"
#include "srtm_rtc_adapter.h"

#include "app_srtm.h"
#include "board.h"
#include "fsl_mu.h"
#include "fsl_debug_console.h"

#include "fsl_wuu.h"
#include "fsl_upower.h"
#include "fsl_iomuxc.h"
#include "rsc_table.h"
#include "fsl_bbnsm.h"
#include "fsl_sentinel.h"
#include "fsl_lsm.h"
#include "fsl_max.h"
#include "fsl_adapter_gpio.h"
#include "max_alg.h"
#include "max_cfg.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
void APP_SRTM_WakeupCA35(void);

typedef struct
{
    uint16_t io_id;
    TimerHandle_t timer; /* GPIO glitch detect timer */
    srtm_io_event_t event;
    bool wakeup;
    bool overridden;    /* Means the CA35 pin configuration is overridden by CM33 wakeup pin. */
    uint8_t io_state;   /* as input(can use as button) or output*/
    uint8_t keypad_idx; /* when use as keypad(button), will use the field */
    uint8_t value;
    IRQn_Type irq; /* such as: GPIO1_0_IRQn */
} app_io_t;

/* NOTE: CM33 DRIVERS DON'T SUPPORT SAVE CONTEXT FOR RESUME, BUT CA35 LINUX DRIVERS DO.
 * WHEN CM33 CORE RUNS INTO VLLS MODE, MOST PERIPHERALS STATE WILL BE LOST. HERE PROVIDES
 * AN EXAMPLE TO SAVE DEVICE STATE BY APPLICATION IN A SUSPEND CONTEXT LOCATING IN TCM
 * WHICH CAN KEEP DATA IN VLLS MODE.
 */
typedef struct
{
    struct
    {
        app_io_t data[TOTAL_PINS];
    } io;
    struct
    {
        uint32_t CR;
    } mu;
} app_suspend_ctx_t;

typedef enum
{
    CORE_ACT  = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x0U),
    CORE_STDB = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x1U),
    CORE_PD   = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x3U),
} core_low_power_mode_t; /* A35 core0/1 low power mode */

typedef struct
{
    uint32_t cnt; /* step counter now. */
} app_pedometer_t;

typedef struct
{
    uint32_t beats;
} app_heartrate_t;

typedef struct
{
    uint32_t rate;
} app_spo2_t;

typedef struct
{
    uint32_t temp_val;
} app_temp_t;

typedef struct
{
    bool stateEnabled;
    bool dataEnabled;
    uint32_t pollDelay;
    app_pedometer_t pedometer;
} app_lsm_sensor_t;

typedef struct
{
    bool stateEnabled;
    bool dataEnabled;
    app_heartrate_t heartrate;
    app_spo2_t      spo2;
    app_temp_t      temp;
} app_max_sensor_t;

#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
#define BUFFER_LEN (128 * 1024)
#if (defined(__ICCARM__))
static uint8_t g_buffer[BUFFER_LEN] @"AudioBuf";
#else
static uint8_t g_buffer[BUFFER_LEN] __attribute__((section("AudioBuf,\"w\",%nobits @")));
#endif
static srtm_sai_edma_local_buf_t g_local_buf = {
    .buf       = (uint8_t *)&g_buffer,
    .bufSize   = BUFFER_LEN,
    .periods   = SRTM_SAI_EDMA_MAX_LOCAL_BUF_PERIODS,
    .threshold = 1,

};
#endif

struct io_struct
{
    uint16_t io_id;
    uint8_t io_state;
    uint8_t keypad_idx; /* it will be used when io is used as button */
};

/* Pls add new pin in the table */
const struct io_struct io_id_table[] = {
    {APP_PIN_ONOFF_BTN, IO_AS_BUTTON, APP_KEYPAD_INDEX_POWER},
    {APP_PIN_VOLPLUS_BTN, IO_AS_BUTTON, APP_KEYPAD_INDEX_VOL_PLUS},
    {APP_PIN_VOLMINUS_BTN, IO_AS_BUTTON, APP_KEYPAD_INDEX_VOL_MINUS},

    {APP_PIN_TOUCH_INT, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_LSM6DSO_INT1, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_LSM6DSO_INT2, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_NFC_INT, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_GNSS_INT, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_CHARGER_INT, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_CHARGER_ALRT_INT, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_MAX30101_INT, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_MPL3115_INT, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_TSL25403_INT, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_AMOLED_TE_INT, IO_AS_INPUT, APP_KEYPAD_INDEX_RESERVED},

    {APP_PIN_RST_GNSS, IO_AS_OUTPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_RST_AMOLED_PANEL, IO_AS_OUTPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_RST_TOUCH_PANEL, IO_AS_OUTPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_NFC_DWL_REQ, IO_AS_OUTPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_NFC_VEN, IO_AS_OUTPUT, APP_KEYPAD_INDEX_RESERVED},
    {APP_PIN_NFC_GPIO2_A0, IO_AS_OUTPUT, APP_KEYPAD_INDEX_RESERVED},
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static srtm_status_t APP_SRTM_I2C_Read(srtm_i2c_adapter_t adapter,
                                       uint32_t base_addr,
                                       srtm_i2c_type_t type,
                                       uint16_t slaveAddr,
                                       uint8_t *buf,
                                       uint16_t len,
                                       uint16_t flags);

static srtm_status_t APP_SRTM_I2C_Write(srtm_i2c_adapter_t adapter,
                                        uint32_t base_addr,
                                        srtm_i2c_type_t type,
                                        uint16_t slaveAddr,
                                        uint8_t *buf,
                                        uint16_t len,
                                        uint16_t flags);

static srtm_status_t APP_SRTM_I2C_SwitchChannel(srtm_i2c_adapter_t adapter,
                                                uint32_t base_addr,
                                                srtm_i2c_type_t type,
                                                uint16_t slaveAddr,
                                                srtm_i2c_switch_channel channel);

static srtm_status_t APP_IO_ConfIEvent(
    srtm_service_t service, srtm_peercore_t core, uint16_t io_id, srtm_io_event_t event, bool wakeup);

static void APP_HandleGPIOHander(void *param);
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile app_srtm_state_t srtmState;
bool option_v_boot_flag          = false;
static bool need_reset_peer_core = false;

pca9460_buck3ctrl_t buck3_ctrl;
pca9460_ldo1_cfg_t ldo1_cfg;

/* For CMC1_IRQHandler */
static int64_t apd_boot_cnt = 0; /* it's cold boot when apd_boot_cnt(Application Domain, A Core) == 1 */

static const uint16_t wuuPins[] = {
    0x0000U, /* WUU_P0 PTA0 */
    0x0003U, /* WUU_P1 PTA3 */
    0x0004U, /* WUU_P2 PTA4 */
    0x0006U, /* WUU_P3 PTA6 */
    0x0007U, /* WUU_P4 PTA7 */
    0x0008U, /* WUU_P5 PTA8 */
    0x0009U, /* WUU_P6 PTA9 */
    0x000AU, /* WUU_P7 PTA10 */
    0x000BU, /* WUU_P8 PTA11 */
    0x000CU, /* WUU_P9 PTA12 */
    0x000DU, /* WUU_P10 PTA13 */
    0x000EU, /* WUU_P11 PTA14 */
    0x000FU, /* WUU_P12 PTA15 */
    0x0010U, /* WUU_P13 PTA16 */
    0x0011U, /* WUU_P14 PTA17 */
    0x0012U, /* WUU_P15 PTA18 */
    0x0018U, /* WUU_P16 PTA24 */

    0x0100U, /* WUU_P17 PTB0 */
    0x0101U, /* WUU_P18 PTB1 */
    0x0102U, /* WUU_P19 PTB2 */
    0x0103U, /* WUU_P20 PTB3 */
    0x0104U, /* WUU_P21 PTB4 */
    0x0105U, /* WUU_P22 PTB5 */
    0x0106U, /* WUU_P23 PTB6 */
    0x010CU, /* WUU_P24 PTB12 */
    0x010DU, /* WUU_P25 PTB13 */
    0x010EU, /* WUU_P26 PTB14 */
    0x010FU, /* WUU_P27 PTB15 */
};

static const srtm_io_event_t IoEvents[] = {
    SRTM_IoEventNone,        /* SRTM_KeypadEventNone */
    SRTM_IoEventRisingEdge,  /* SRTM_KeypadEventPress */
    SRTM_IoEventFallingEdge, /* SRTM_KeypadEventRelease */
    SRTM_IoEventEitherEdge   /* SRTM_KeypadEventPressOrRelease */
};

static const srtm_io_event_t wuuPinModeEvents[] = {
    SRTM_IoEventNone,        /* kWUU_ExternalPinDisable */
    SRTM_IoEventRisingEdge,  /* kWUU_ExternalPinRisingEdge */
    SRTM_IoEventFallingEdge, /* kWUU_ExternalPinFallingEdge */
    SRTM_IoEventEitherEdge   /* kWUU_ExternalPinAnyEdge */
};

static srtm_status_t APP_SRTM_Sensor_EnableStateDetector(srtm_sensor_adapter_t adapter,
                                                         srtm_sensor_type_t type,
                                                         uint8_t index,
                                                         bool enable);
static srtm_status_t APP_SRTM_Sensor_EnableDataReport(srtm_sensor_adapter_t adapter,
                                                      srtm_sensor_type_t type,
                                                      uint8_t index,
                                                      bool enable);
static srtm_status_t APP_SRTM_Sensor_SetPollDelay(srtm_sensor_adapter_t adapter,
                                                  srtm_sensor_type_t type,
                                                  uint8_t index,
                                                  uint32_t millisec);

static struct _srtm_sensor_adapter sensorAdapter =
{
    .enableStateDetector = APP_SRTM_Sensor_EnableStateDetector,
    .enableDataReport    = APP_SRTM_Sensor_EnableDataReport,
    .setPollDelay        = APP_SRTM_Sensor_SetPollDelay
};

static app_lsm_sensor_t lsm_sensor =
{
    .stateEnabled  = false,
    .dataEnabled   = false,
    .pollDelay     = 1000, /* 1 sec by default. */
    .pedometer.cnt = 0
};

static app_max_sensor_t max_sensor =
{
    .stateEnabled  = false,
    .dataEnabled   = false,
    .heartrate.beats = 0,
    .spo2.rate = 0,
    .temp.temp_val = 0
};

static srtm_dispatcher_t disp;
static srtm_peercore_t core;
static srtm_sai_adapter_t saiAdapter;
static srtm_service_t audioService;
static srtm_service_t pwmService;
static srtm_service_t rtcService;
static srtm_rtc_adapter_t rtcAdapter;
static srtm_service_t i2cService;
static srtm_service_t ioService;
static srtm_service_t keypadService;
static SemaphoreHandle_t monSig;
static struct rpmsg_lite_instance *rpmsgHandle;
static app_rpmsg_monitor_t rpmsgMonitor;
static void *rpmsgMonitorParam;
static TimerHandle_t linkupTimer;
static TimerHandle_t refreshS400WdgTimer;
static TimerHandle_t rtcAlarmEventTimer; /* It is used to send alarm event to acore after acore(acore entered power down
                                            mode) is waken by rtc alarm(Avoid losting a rtc alarm event) */
static TimerHandle_t restoreRegValOfMuTimer; /* use the timer to restore register's value of mu(To make sure that
                                                register's value of mu is restored if cmc1 interrupt is not comming) */

static lsm_handle_t lsmHandle;
static max_handle_t maxHandle;
static srtm_service_t sensorService;
static bool lsmSensorReady            = false;
static bool maxSensorReady            = false;
static srtm_procedure_t lsmSensorProc = NULL;
static srtm_procedure_t maxSensorProc = NULL;
static TimerHandle_t
    sensorTiltWakeupEventTimer = NULL; /* It is used to send sensor tilt wakeup event to acore after acore(acore entered power
                           down mode) is waken by sensor(tilt interrupt)(Avoid losting a sensor tilt wakeup event) */
static TimerHandle_t
    sensorMaxFlushFiFoTimer = NULL;    /* Used to flush FIFO if max FIFO overflows. */

static app_irq_handler_t irqHandler;
static void *irqHandlerParam;

static int32_t hr_spo2_irq_count = 0;

static HAL_PWM_HANDLE_DEFINE(pwmHandle0);

static HAL_RTC_HANDLE_DEFINE(rtcHandle);

const uint8_t g_lsm_sensor_address[] = {
    LSM6DSO_SLAVE_ADDRESS_WHEN_SA0_PIN_IS_LOW,
    LSM6DSO_SLAVE_ADDRESS_WHEN_SA0_PIN_IS_HIGH
};

lpm_ad_power_mode_e AD_CurrentMode   = AD_UNKOWN;
lpm_ad_power_mode_e AD_WillEnterMode = AD_UNKOWN;

/* pwmHandles must strictly follow TPM instances. If you don't provide service for some TPM instance,
 * set the corresponding handle to NULL. */
static hal_pwm_handle_t pwmHandles[2] = {(hal_pwm_handle_t)pwmHandle0, NULL};

/* GPIOA */
static GPIO_HANDLE_DEFINE(gpioHandle0_0);
static GPIO_HANDLE_DEFINE(gpioHandle0_1);
static GPIO_HANDLE_DEFINE(gpioHandle0_2);
static GPIO_HANDLE_DEFINE(gpioHandle0_3);
static GPIO_HANDLE_DEFINE(gpioHandle0_4);
static GPIO_HANDLE_DEFINE(gpioHandle0_5);
static GPIO_HANDLE_DEFINE(gpioHandle0_6);
static GPIO_HANDLE_DEFINE(gpioHandle0_7);
static GPIO_HANDLE_DEFINE(gpioHandle0_8);
static GPIO_HANDLE_DEFINE(gpioHandle0_9);
static GPIO_HANDLE_DEFINE(gpioHandle0_10);
static GPIO_HANDLE_DEFINE(gpioHandle0_11);
static GPIO_HANDLE_DEFINE(gpioHandle0_12);
static GPIO_HANDLE_DEFINE(gpioHandle0_13);
static GPIO_HANDLE_DEFINE(gpioHandle0_14);
static GPIO_HANDLE_DEFINE(gpioHandle0_15);
static GPIO_HANDLE_DEFINE(gpioHandle0_16);
static GPIO_HANDLE_DEFINE(gpioHandle0_17);
static GPIO_HANDLE_DEFINE(gpioHandle0_18);
static GPIO_HANDLE_DEFINE(gpioHandle0_19);
static GPIO_HANDLE_DEFINE(gpioHandle0_20);
static GPIO_HANDLE_DEFINE(gpioHandle0_21);
static GPIO_HANDLE_DEFINE(gpioHandle0_22);
static GPIO_HANDLE_DEFINE(gpioHandle0_23);
static GPIO_HANDLE_DEFINE(gpioHandle0_24);

/* GPIOB */
static GPIO_HANDLE_DEFINE(gpioHandle1_0);
static GPIO_HANDLE_DEFINE(gpioHandle1_1);
static GPIO_HANDLE_DEFINE(gpioHandle1_2);
static GPIO_HANDLE_DEFINE(gpioHandle1_3);
static GPIO_HANDLE_DEFINE(gpioHandle1_4);
static GPIO_HANDLE_DEFINE(gpioHandle1_5);
static GPIO_HANDLE_DEFINE(gpioHandle1_6);
static GPIO_HANDLE_DEFINE(gpioHandle1_7);
static GPIO_HANDLE_DEFINE(gpioHandle1_8);
static GPIO_HANDLE_DEFINE(gpioHandle1_9);
static GPIO_HANDLE_DEFINE(gpioHandle1_10);
static GPIO_HANDLE_DEFINE(gpioHandle1_11);
static GPIO_HANDLE_DEFINE(gpioHandle1_12);
static GPIO_HANDLE_DEFINE(gpioHandle1_13);
static GPIO_HANDLE_DEFINE(gpioHandle1_14);
static GPIO_HANDLE_DEFINE(gpioHandle1_15);

/* GPIOC */
static GPIO_HANDLE_DEFINE(gpioHandle2_0);
static GPIO_HANDLE_DEFINE(gpioHandle2_1);
static GPIO_HANDLE_DEFINE(gpioHandle2_2);
static GPIO_HANDLE_DEFINE(gpioHandle2_3);
static GPIO_HANDLE_DEFINE(gpioHandle2_4);
static GPIO_HANDLE_DEFINE(gpioHandle2_5);
static GPIO_HANDLE_DEFINE(gpioHandle2_6);
static GPIO_HANDLE_DEFINE(gpioHandle2_7);
static GPIO_HANDLE_DEFINE(gpioHandle2_8);
static GPIO_HANDLE_DEFINE(gpioHandle2_9);
static GPIO_HANDLE_DEFINE(gpioHandle2_10);
static GPIO_HANDLE_DEFINE(gpioHandle2_11);
static GPIO_HANDLE_DEFINE(gpioHandle2_12);
static GPIO_HANDLE_DEFINE(gpioHandle2_13);
static GPIO_HANDLE_DEFINE(gpioHandle2_14);
static GPIO_HANDLE_DEFINE(gpioHandle2_15);
static GPIO_HANDLE_DEFINE(gpioHandle2_16);
static GPIO_HANDLE_DEFINE(gpioHandle2_17);
static GPIO_HANDLE_DEFINE(gpioHandle2_18);
static GPIO_HANDLE_DEFINE(gpioHandle2_19);
static GPIO_HANDLE_DEFINE(gpioHandle2_20);
static GPIO_HANDLE_DEFINE(gpioHandle2_21);
static GPIO_HANDLE_DEFINE(gpioHandle2_22);
static GPIO_HANDLE_DEFINE(gpioHandle2_23);

static uint32_t inputMask  = IOMUXC_PCR_IBE_MASK;
static uint32_t outputMask = IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK | IOMUXC_PCR_OBE_MASK;

static IRQn_Type gpio_IRQn[][GPIO_INT_NUM] = {
    {NotAvail_IRQn, NotAvail_IRQn, GPIOA_INT0_IRQn,
     GPIOA_INT1_IRQn}, /* GPIOA, gpio_IRQn[0~2][0~1] are used by acore, refer to chapter 3(GIC interrupt vector and NVIC
                          interrupt vector) of RM  */
    {NotAvail_IRQn, NotAvail_IRQn, GPIOB_INT0_IRQn, GPIOB_INT1_IRQn}, /* GPIOB */
    {NotAvail_IRQn, NotAvail_IRQn, GPIOC_INT0_IRQn, GPIOC_INT1_IRQn}, /* GPIOC */
};

static max_config_t g_maxConfig = { 0 };
static max_sample_buf_t g_max_sensor_data_buf;
static int32_t g_is_got_max_irq = 0;

static hal_gpio_handle_t gpioHandles[GPIO_MODULE_NUM][PIN_NUM_PER_GPIO_MODULE] = {
    /* GPIOA */
    {(hal_gpio_handle_t)gpioHandle0_0,
     (hal_gpio_handle_t)gpioHandle0_1,
     (hal_gpio_handle_t)gpioHandle0_2,
     (hal_gpio_handle_t)gpioHandle0_3,
     (hal_gpio_handle_t)gpioHandle0_4,
     (hal_gpio_handle_t)gpioHandle0_5,
     (hal_gpio_handle_t)gpioHandle0_6,
     (hal_gpio_handle_t)gpioHandle0_7,
     (hal_gpio_handle_t)gpioHandle0_8,
     (hal_gpio_handle_t)gpioHandle0_9,
     (hal_gpio_handle_t)gpioHandle0_10,
     (hal_gpio_handle_t)gpioHandle0_11,
     (hal_gpio_handle_t)gpioHandle0_12,
     (hal_gpio_handle_t)gpioHandle0_13,
     (hal_gpio_handle_t)gpioHandle0_14,
     (hal_gpio_handle_t)gpioHandle0_15,
     (hal_gpio_handle_t)gpioHandle0_16,
     (hal_gpio_handle_t)gpioHandle0_17,
     (hal_gpio_handle_t)gpioHandle0_18,
     (hal_gpio_handle_t)gpioHandle0_19,
     (hal_gpio_handle_t)gpioHandle0_20,
     (hal_gpio_handle_t)gpioHandle0_21,
     (hal_gpio_handle_t)gpioHandle0_22,
     (hal_gpio_handle_t)gpioHandle0_23,
     (hal_gpio_handle_t)gpioHandle0_24,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL},

    /* GPIOB */
    {(hal_gpio_handle_t)gpioHandle1_0,
     (hal_gpio_handle_t)gpioHandle1_1,
     (hal_gpio_handle_t)gpioHandle1_2,
     (hal_gpio_handle_t)gpioHandle1_3,
     (hal_gpio_handle_t)gpioHandle1_4,
     (hal_gpio_handle_t)gpioHandle1_5,
     (hal_gpio_handle_t)gpioHandle1_6,
     (hal_gpio_handle_t)gpioHandle1_7,
     (hal_gpio_handle_t)gpioHandle1_8,
     (hal_gpio_handle_t)gpioHandle1_9,
     (hal_gpio_handle_t)gpioHandle1_10,
     (hal_gpio_handle_t)gpioHandle1_11,
     (hal_gpio_handle_t)gpioHandle1_12,
     (hal_gpio_handle_t)gpioHandle1_13,
     (hal_gpio_handle_t)gpioHandle1_14,
     (hal_gpio_handle_t)gpioHandle1_15,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL},

    /* GPIOC */
    {(hal_gpio_handle_t)gpioHandle2_0,
     (hal_gpio_handle_t)gpioHandle2_1,
     (hal_gpio_handle_t)gpioHandle2_2,
     (hal_gpio_handle_t)gpioHandle2_3,
     (hal_gpio_handle_t)gpioHandle2_4,
     (hal_gpio_handle_t)gpioHandle2_5,
     (hal_gpio_handle_t)gpioHandle2_6,
     (hal_gpio_handle_t)gpioHandle2_7,
     (hal_gpio_handle_t)gpioHandle2_8,
     (hal_gpio_handle_t)gpioHandle2_9,
     (hal_gpio_handle_t)gpioHandle2_10,
     (hal_gpio_handle_t)gpioHandle2_11,
     (hal_gpio_handle_t)gpioHandle2_12,
     (hal_gpio_handle_t)gpioHandle2_13,
     (hal_gpio_handle_t)gpioHandle2_14,
     (hal_gpio_handle_t)gpioHandle2_15,
     (hal_gpio_handle_t)gpioHandle2_16,
     (hal_gpio_handle_t)gpioHandle2_17,
     (hal_gpio_handle_t)gpioHandle2_18,
     (hal_gpio_handle_t)gpioHandle2_19,
     (hal_gpio_handle_t)gpioHandle2_20,
     (hal_gpio_handle_t)gpioHandle2_21,
     (hal_gpio_handle_t)gpioHandle2_22,
     (hal_gpio_handle_t)gpioHandle2_23,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL,
     NULL},
};

const static uint32_t gpioFuncId[TOTAL_PINS][PIN_FUNC_ID_SIZE] = {
    /* PTA0 ~ PTA24 */
    {IOMUXC_PTA0_PTA0},
    {IOMUXC_PTA1_PTA1},
    {IOMUXC_PTA2_PTA2},
    {IOMUXC_PTA3_PTA3},
    {IOMUXC_PTA4_PTA4},
    {IOMUXC_PTA5_PTA5},
    {IOMUXC_PTA6_PTA6},
    {IOMUXC_PTA7_PTA7},
    {IOMUXC_PTA8_PTA8},
    {IOMUXC_PTA9_PTA9},
    {IOMUXC_PTA10_PTA10},
    {IOMUXC_PTA11_PTA11},
    {IOMUXC_PTA12_PTA12},
    {IOMUXC_PTA13_PTA13},
    {IOMUXC_PTA14_PTA14},
    {IOMUXC_PTA15_PTA15},
    {IOMUXC_PTA16_PTA16},
    {IOMUXC_PTA17_PTA17},
    {IOMUXC_PTA18_PTA18},
    {IOMUXC_PTA19_PTA19},
    {IOMUXC_PTA20_PTA20},
    {IOMUXC_PTA21_PTA21},
    {IOMUXC_PTA22_PTA22},
    {IOMUXC_PTA23_PTA23},
    {IOMUXC_PTA24_PTA24},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},

    /* PTB0 ~ PTB15 */
    {IOMUXC_PTB0_PTB0},
    {IOMUXC_PTB1_PTB1},
    {IOMUXC_PTB2_PTB2},
    {IOMUXC_PTB3_PTB3},
    {IOMUXC_PTB4_PTB4},
    {IOMUXC_PTB5_PTB5},
    {IOMUXC_PTB6_PTB6},
    {IOMUXC_PTB7_PTB7},
    {IOMUXC_PTB8_PTB8},
    {IOMUXC_PTB9_PTB9},
    {IOMUXC_PTB10_PTB10},
    {IOMUXC_PTB11_PTB11},
    {IOMUXC_PTB12_PTB12},
    {IOMUXC_PTB13_PTB13},
    {IOMUXC_PTB14_PTB14},
    {IOMUXC_PTB15_PTB15},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},

    /* PTC0 ~ PTC23 */
    {IOMUXC_PTC0_PTC0},
    {IOMUXC_PTC1_PTC1},
    {IOMUXC_PTC2_PTC2},
    {IOMUXC_PTC3_PTC3},
    {IOMUXC_PTC4_PTC4},
    {IOMUXC_PTC5_PTC5},
    {IOMUXC_PTC6_PTC6},
    {IOMUXC_PTC7_PTC7},
    {IOMUXC_PTC8_PTC8},
    {IOMUXC_PTC9_PTC9},
    {IOMUXC_PTC10_PTC10},
    {IOMUXC_PTC11_PTC11},
    {IOMUXC_PTC12_PTC12},
    {IOMUXC_PTC13_PTC13},
    {IOMUXC_PTC14_PTC14},
    {IOMUXC_PTC15_PTC15},
    {IOMUXC_PTC16_PTC16},
    {IOMUXC_PTC17_PTC17},
    {IOMUXC_PTC18_PTC18},
    {IOMUXC_PTC19_PTC19},
    {IOMUXC_PTC20_PTC20},
    {IOMUXC_PTC21_PTC21},
    {IOMUXC_PTC22_PTC22},
    {IOMUXC_PTC23_PTC23},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
    {PIN_CANNOT_USE_AS_GPIO},
};

static struct _i2c_bus platform_i2c_buses[] = {
    {.bus_id         = 0,
     .base_addr      = LPI2C0_BASE,
     .type           = SRTM_I2C_TYPE_LPI2C,
     .switch_idx     = I2C_SWITCH_NONE,
     .switch_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED},
    {.bus_id         = 1,
     .base_addr      = LPI2C1_BASE,
     .type           = SRTM_I2C_TYPE_LPI2C,
     .switch_idx     = I2C_SWITCH_NONE,
     .switch_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED},
};

static struct _srtm_i2c_adapter i2c_adapter = {.read          = APP_SRTM_I2C_Read,
                                               .write         = APP_SRTM_I2C_Write,
                                               .switchchannel = APP_SRTM_I2C_SwitchChannel,
                                               .bus_structure = {
                                                   .buses      = platform_i2c_buses,
                                                   .bus_num    = sizeof(platform_i2c_buses) / sizeof(struct _i2c_bus),
                                                   .switch_num = 0,
                                               }};

static app_suspend_ctx_t suspendContext;

static MU_Type mu0_mua;

struct dram_cfg *dram_timing_cfg;
uint32_t dram_class;

/*******************************************************************************
 * Code
 ******************************************************************************/

void MU0_MUA_Save(void)
{
    /* Make sure the clock is on */
    MU_Init(MU0_MUA);
    mu0_mua.RCR   = MU0_MUA->RCR;
    mu0_mua.CIER0 = MU0_MUA->CIER0;
}

void MU0_MUA_Restore(void)
{
    /* Make sure the clock is on */
    MU_Init(MU0_MUA);
    if (mu0_mua.RCR != 0)
    {
        MU0_MUA->RCR = mu0_mua.RCR;
    }
    if (mu0_mua.CIER0 != 0)
    {
        MU0_MUA->CIER0 = mu0_mua.CIER0;
    }
}

/* Real Time Domain save context */
void rtdCtxSave(void)
{
    MU0_MUA_Save();
}

/* Real Time Domain restore context */
void rtdCtxRestore(void)
{
    MU0_MUA_Restore();
}

bool APP_IO_WhetherMatchIoId(uint16_t io_id, int32_t *io_table_idx)
{
    int32_t i = 0;

    while (i < ARRAY_SIZE(io_id_table))
    {
        if (io_id_table[i].io_id == io_id)
        {
            *io_table_idx = i;
            return true;
        }
        i++;
    }
    return false;
}

static uint8_t APP_IO_GetWUUPinByIoId(uint16_t io_id)
{
    uint8_t i;

    for (i = 0; i < ARRAY_SIZE(wuuPins); i++)
    {
        if (wuuPins[i] == io_id)
        {
            break;
        }
    }

    return i;
}

static uint8_t APP_IO_GetIoIndexByIoId(uint16_t io_id)
{
    uint8_t i;

    for (i = 0; i < ARRAY_SIZE(suspendContext.io.data); i++)
    {
        if (suspendContext.io.data[i].io_id == io_id)
        {
            break;
        }
    }

    return i;
}

static uint8_t APP_IO_GetIoIndexByTimerHandle(TimerHandle_t timer)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(suspendContext.io.data); i++)
    {
        if (suspendContext.io.data[i].timer == timer)
        {
            break;
        }
    }

    return i;
}

static uint8_t APP_Keypad_GetIoIndex(uint8_t key_idx)
{
    uint8_t io_idx;

    for (io_idx = 0; io_idx < ARRAY_SIZE(suspendContext.io.data); io_idx++)
    {
        if (suspendContext.io.data[io_idx].keypad_idx == key_idx)
        {
            break;
        }
    }

    return io_idx;
}

static srtm_io_event_t APP_Keypad_GetIoEvent(uint8_t keyIdx, srtm_keypad_event_t event)
{
    switch (keyIdx)
    {
        case APP_KEYPAD_INDEX_VOL_PLUS:
        case APP_KEYPAD_INDEX_VOL_MINUS:
        case APP_KEYPAD_INDEX_POWER:
            return IoEvents[event];
        default:
            assert(false);
            break;
    }

    return SRTM_IoEventNone;
}

void APP_WakeupACore(void)
{
    /*
     * For case: when RTD is the ower of LPAV and APD enter PD/DPD, Mcore don't enter lp mode, but wakes up directly.
     * RTD need restore related settings.
     */
    if (BOARD_IsLPAVOwnedByRTD())
    {
        UPOWER_ReadPmicReg(PCA9460_BUCK3CTRL_ADDR, &(buck3_ctrl.val));
        UPOWER_ReadPmicReg(PCA9460_LDO1_CFG_ADDR, &(ldo1_cfg.val));

        if (AD_CurrentMode == AD_PD)
        {
            APP_SRTM_EnableLPAV();
        }
        else if (AD_CurrentMode == AD_DPD && !ldo1_cfg.reg.L1_ENMODE && !buck3_ctrl.reg.B3_ENMODE)
        {
            /* B3_ENMODE = 0x1, L1_ENMODE, ON at RUN State(default) */
            buck3_ctrl.reg.B3_ENMODE = 0x1;
            ldo1_cfg.reg.L1_ENMODE   = 0x1;
            UPOWER_SetPmicReg(PCA9460_LDO1_CFG_ADDR, ldo1_cfg.val);
            UPOWER_SetPmicReg(PCA9460_BUCK3CTRL_ADDR, buck3_ctrl.val);
        }
    }
    else /* owner of lpav is AD */
    {
        /* For RTD hold lpav, sai low power audio demo, we need enable lpav before wakeup APD */
#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
        if (R32(DDR_IN_SELFREFRESH_BASE))
        {
            DisableIRQ(BBNSM_IRQn);
            DisableIRQ(GPIOA_INT0_IRQn);
            DisableIRQ(GPIOA_INT1_IRQn);
            DisableIRQ(GPIOB_INT0_IRQn);
            DisableIRQ(GPIOB_INT1_IRQn);
            DisableIRQ(GPIOB_INT1_IRQn);
            DisableIRQ(WUU0_IRQn);

            PRINTF("Acore will enter avtive, Put ddr into active\r\n");
            /* ddr in retention state, need put ddr exit retention */
            APP_SRTM_EnableLPAV();

            W32(DDR_IN_SELFREFRESH_BASE, 0);

            EnableIRQ(GPIOA_INT0_IRQn);
            EnableIRQ(GPIOA_INT1_IRQn);
            EnableIRQ(GPIOB_INT0_IRQn);
            EnableIRQ(GPIOB_INT1_IRQn);
            EnableIRQ(BBNSM_IRQn);
            EnableIRQ(WUU0_IRQn);
        }
#endif
    }
    UPOWER_PowerOnADInPDMode();
}

static void APP_ResetSRTM(app_srtm_state_t state)
{
    srtmState = state;
    /* Wake up monitor to reinitialize the SRTM communication with CA35 */
    xSemaphoreGive(monSig);
}

static void APP_SRTM_ControlCA35(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    app_srtm_state_t state = (app_srtm_state_t)(uint32_t)param1;

    switch (state)
    {
        case APP_SRTM_StateRun:
            /* Fresh power up: Need SRTM monitor to prepare communication */
            srtmState = APP_SRTM_StateRun;
            xSemaphoreGive(monSig);
            break;
        case APP_SRTM_StateReboot:
            /* Only when CA35 is active, we can reboot it. */
            if (!core || AD_CurrentMode != AD_ACT)
            {
                PRINTF("CA35 is not active, cannot reboot!\r\n");
            }
            else
            {
                /* Now prepare reboot */
                need_reset_peer_core = true; /* set a flag to check whether need reset peer core(don't need reset peer
                                                core when peer core is in reset) */
                APP_ResetSRTM(APP_SRTM_StateReboot);
            }
            break;
        case APP_SRTM_StateShutdown:
            /* Only when CA35 goes into DPD, we can shutdown it. */
            if (core && AD_CurrentMode == AD_DPD)
            {
                /* Now prepare shutdown */
                APP_ResetSRTM(APP_SRTM_StateShutdown);
            }
            else
            {
                PRINTF("CA35 isn't in PD mode, cannot shutdown!\r\n");
            }
            break;
        default:
            break;
    }
}

static void APP_SRTM_SetLPAV(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    lpm_ad_power_mode_e state = (lpm_ad_power_mode_e)(uint32_t)param1;

    switch (state)
    {
        case AD_UNKOWN:
            break;
        case AD_ACT:
            break;
        case AD_PD:
            if (BOARD_IsLPAVOwnedByRTD())
            {
                /* Power down lpav domain, put ddr into retention, poweroff LDO1, set BUCK3 to 0.73V */
                APP_SRTM_DisableLPAV();
#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
                W32(DDR_IN_SELFREFRESH_BASE, 1);
                /* In dualboot mode, RTD hold LPAV domain, set LPAV ownership to APD let ddr into retention */
                SIM_SEC->SYSCTRL0               = 0xA70480;
                SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0x7F;
                SIM_SEC->LPAV_SLAVE_ALLOC_CTRL  = 0x1F;
#endif
            }
            break;
        case AD_DPD:
            if (BOARD_IsLPAVOwnedByRTD())
            {
                /*
                 * APD default hold LPAV domain device and needs change the ownership of the LPAV device
                 * from APD to RTD
                 */
                SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0;
                SIM_SEC->LPAV_SLAVE_ALLOC_CTRL  = 0;

                UPOWER_PowerOffSwitches((upower_ps_mask_t)(
                    kUPOWER_PS_GPU3D | kUPOWER_PS_HIFI4 | kUPOWER_PS_DDRC | kUPOWER_PS_PXP_EPDC | kUPOWER_PS_MIPI_DSI |
                    kUPOWER_PS_MIPI_CSI | kUPOWER_PS_AV_NIC | kUPOWER_PS_FUSION_AO));

                UPOWER_PowerOffMemPart(
                    (uint32_t)(kUPOWER_MP0_DCNANO_A | kUPOWER_MP0_DCNANO_B | kUPOWER_MP0_EPDC_A | kUPOWER_MP0_EPDC_B |
                               kUPOWER_MP0_DMA2 | kUPOWER_MP0_GPU2D_A | kUPOWER_MP0_GPU2D_B | kUPOWER_MP0_GPU3D_A |
                               kUPOWER_MP0_GPU3D_B | kUPOWER_MP0_HIFI4 | kUPOWER_MP0_ISI | kUPOWER_MP0_MIPI_CSI |
                               kUPOWER_MP0_MIPI_DSI | kUPOWER_MP0_PXP | kUPOWER_MP0_AV_SYSTEM),
                    0U);

                /*
                 * Workaround: After Mcore hold lpav in dualboot mode, Mcore can't resume from PD/DPD modes in some
                 * boards. Restore lpav master and slave devices can fix it.
                 */
                SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0x7F;
                SIM_SEC->LPAV_SLAVE_ALLOC_CTRL  = 0x1F;

                /* Configure BUCK3CTRL and LDO1_CFG according to PCA9460  */
                buck3_ctrl.reg.B3_ENMODE = 0x0; /* 00-OFF */
                buck3_ctrl.reg.B3_FPWM   = 0x0; /* Automatic PFM and PWM mode transition (default) */
                buck3_ctrl.reg.B3_AD     = 0x1; /* Enable Active discharge resistor when regulator is OFF (default) */
                buck3_ctrl.reg.B3_LPMODE = 0x3; /* Normal mode (default) */
                buck3_ctrl.reg.B3_RAMP   = 0x1; /* 25 mV (default) */

                ldo1_cfg.reg.L1_ENMODE = 0x0; /* 00-OFF */
                ldo1_cfg.reg.L1_LPMODE = 0x3; /* Normal mode (default) */
                ldo1_cfg.reg.L1_LLSEL  = 0x1; /* 15 mw (default) */
                ldo1_cfg.reg.L1_CSEL   = 0x2; /* Auto Cout detection (default) */

                /* Poweroff BUCK3 */
                UPOWER_SetPmicReg(PCA9460_BUCK3CTRL_ADDR, buck3_ctrl.val);
                /* Poweroff LDO1 */
                UPOWER_SetPmicReg(PCA9460_LDO1_CFG_ADDR, ldo1_cfg.val);
            }
            break;
    }
}

void APP_RebootCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateReboot, NULL);
    PRINTF("M33 reboot A35\r\n");
    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

void APP_SRTM_ShutdownCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateShutdown, NULL);

    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

/* WUU interrupt handler. */
void WUU0_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(WUU0_IRQn, irqHandlerParam);
    }
}

static void sensorTiltWakeupEventTimer_Callback(TimerHandle_t xTimer)
{
    if (AD_CurrentMode == AD_ACT && sensorAdapter.updateState && sensorAdapter.service)
    {
        sensorAdapter.updateState(sensorAdapter.service, SRTM_SensorTypeTilt, 0);
    }
    xTimerStop(sensorTiltWakeupEventTimer, portMAX_DELAY);
}

static void APP_MAX_FlushFIFO_Dispatcher(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    MAX_FlushFIFO(&maxHandle);
}

static void sensorMaxFlushFiFoTimer_Callback(TimerHandle_t xTimer)
{
    if (g_is_got_max_irq)
    {
        xTimerStop(sensorMaxFlushFiFoTimer, portMAX_DELAY);
    }
    else
    {
        srtm_procedure_t proc = SRTM_Procedure_Create(APP_MAX_FlushFIFO_Dispatcher, NULL, NULL);

        assert(proc);

        SRTM_Dispatcher_PostProc(disp, proc);
    }
}

static void APP_CheckLsmSensorInterrupt(void)
{
    status_t result;
    lsm_emb_func_status_t val;
    BaseType_t reschedule = pdFALSE;

    result = LSM_GetVal(&lsmHandle, LSM_EMB_FUNC_STATUS_REG, (uint8_t *)&val, LSM_EMBEDDED_FUNC_BANK, LSM_USER_BANK);
    if (result == kStatus_Success)
    {
        if (val.is_tilt && AD_CurrentMode == AD_PD)
        {
            /* Wakeup A Core(CA35) when the tilt interrupt is comming */
            APP_WakeupACore();

            /* Send tilt wakeup event(notification) to A Core in timer */
            if (xPortIsInsideInterrupt() == pdTRUE)
            {
                xTimerStartFromISR(sensorTiltWakeupEventTimer, &reschedule);
            }
            else
            {
                xTimerStart(sensorTiltWakeupEventTimer, pdMS_TO_TICKS(10));
            }
        }

        if (val.is_step_det)
        {
            uint16_t stepCnt = 0;

            LSM_GetPedometerCnt(&lsmHandle, &stepCnt);
            lsm_sensor.pedometer.cnt = stepCnt;
            if (lsm_sensor.stateEnabled)
            {
                /* Step detected, then update peer core. */
                assert(sensorAdapter.updateState && sensorAdapter.service);

                if (AD_CurrentMode == AD_ACT)
                {
                    sensorAdapter.updateState(sensorAdapter.service, SRTM_SensorTypePedometer, 0);
                }
            }

            if (lsm_sensor.dataEnabled)
            {
                assert(sensorAdapter.reportData && sensorAdapter.service);

                if (AD_CurrentMode == AD_ACT)
                {
                    sensorAdapter.reportData(sensorAdapter.service, SRTM_SensorTypePedometer, 0,
                                             (uint8_t *)(&lsm_sensor.pedometer.cnt), sizeof(lsm_sensor.pedometer.cnt));
                }
            }
        }
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

static void APP_CheckLsmSensorInterrupt_Dispatcher(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    APP_CheckLsmSensorInterrupt();
}

static void APP_CheckMaxSensorInterrupt(void)
{
    status_t result;
    BaseType_t reschedule = pdFALSE;
    uint8_t int_status_1 = 0, int_status_2 = 0;

    if (kStatus_Success != MAX_ReadReg(&maxHandle, INT_STATUS_REG1, &int_status_1, 1))
    {
        PRINTF("Read MAX Register 1 failed!\r\n");
        return;
    }

    if (kStatus_Success != MAX_ReadReg(&maxHandle, INT_STATUS_REG2, &int_status_2, 1))
    {
        PRINTF("Read MAX Register 2 failed!\r\n");
        return;
    }

    if (int_status_1 & MAXIM_EN_IRQ_FIFO_ALMOST_FULL_MASK)
    {
        g_is_got_max_irq = 1;

        result = MAX_FillSensorDataToBuffer(&maxHandle, &g_max_sensor_data_buf);
        /* If result is not kStatus_Success here, the algorithm calculation should be running */
        if ((kStatus_Success == result) && (g_max_sensor_data_buf.sample_count >= MAX_CFG_TOTAL_SAMPLES))
        {
            MAX_Alg_Start_HR_SpO2_Cal();
        }

        if (hr_spo2_irq_count++ > MAX_CFG_IRQ_NUM_IN_ONE_SEC)
        {
            MAX_GetHeartRate(&maxHandle, &max_sensor.heartrate.beats);
            MAX_GetSpO2(&maxHandle, &max_sensor.spo2.rate);
            PRINTF("Get Hr SpO2 data: %d,hr:%d,spo2:%d\r\n", MAX_CFG_IRQ_NUM_IN_ONE_SEC, max_sensor.heartrate.beats, max_sensor.spo2.rate);
            if (max_sensor.dataEnabled)
            {
                assert(sensorAdapter.reportData && sensorAdapter.service);

                if (AD_CurrentMode == AD_ACT)
                {
                    sensorAdapter.reportData(sensorAdapter.service, SRTM_SensorTypeHeartRate, 0,
                                            (uint8_t *)(&max_sensor.heartrate.beats), sizeof(max_sensor.heartrate.beats));
                    sensorAdapter.reportData(sensorAdapter.service, SRTM_SensorTypeSpO2, 0,
                                            (uint8_t *)(&max_sensor.spo2.rate), sizeof(max_sensor.spo2.rate));
                }
            }
            hr_spo2_irq_count = 0;
        }
    }

    if (int_status_2 & MAXIM_EN_IRQ_INT_TEMP_RDY_MASK)
    {
        int32_t temp_deg = 0;

        /* As temp power should be turned on already, we only need to read temp here */
        MAX_ReadTemp(&maxHandle, &temp_deg);
        max_sensor.temp.temp_val = temp_deg;
        /* Got temperature and report */
        assert(sensorAdapter.updateState && sensorAdapter.service);
        if (AD_CurrentMode == AD_ACT)
        {
            sensorAdapter.updateState(sensorAdapter.service, SRTM_SensorTypeTemperature, 0);
        }

        assert(sensorAdapter.reportData && sensorAdapter.service);
        if (AD_CurrentMode == AD_ACT)
        {
            sensorAdapter.reportData(sensorAdapter.service, SRTM_SensorTypeTemperature, 0,
                                     (uint8_t *)(&max_sensor.temp.temp_val), sizeof(max_sensor.temp.temp_val));
        }
        /* The temp en bit is a self-clearing bit which, when set, initiates a single temperature reading from the
         * temperature sensor. This bit clears automatically back to zero at the conclusion of the temperature
         * reading when the bit is set to one.
         */
 
        /* If heart rate and spo2 is not ON, we need to turn off power. */
        if (!max_sensor.stateEnabled)
        {
            MAX_Enable_Temp_Power(&maxHandle, false);
        }
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

static void APP_CheckMaxSensorInterrupt_Dispatcher(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    APP_CheckMaxSensorInterrupt();
}

static void APP_CheckSensorInterruptCallback(srtm_procedure_t *sensor_proc)
{
    /* Read and process sensor data in SRTM task context */
    if (*sensor_proc)
    {
        SRTM_Dispatcher_PostProc(disp, *sensor_proc);
        *sensor_proc = NULL;
    }
}

static void APP_HandleGPIOHander(void *param)
{
    BaseType_t reschedule = pdFALSE;
    uint8_t port_idx      = 0U;
    uint8_t pin_idx       = 0U;
    uint16_t io_id        = (uint16_t)param;
    int32_t io_idx        = 0U;

    port_idx = GPIO_PORT_IDX(io_id);
    pin_idx  = GPIO_PIN_IDX(io_id);
    io_idx   = port_idx * PIN_NUM_PER_GPIO_MODULE + pin_idx;

    switch (io_id)
    {
    case APP_PIN_LSM6DSO_INT1:
    case APP_PIN_LSM6DSO_INT2:
        /* Read sensor data in dispatcher task context */
        APP_CheckSensorInterruptCallback(&lsmSensorProc);
        break;
    case APP_PIN_MAX30101_INT:
        /* Read sensor data in dispatcher task context */
        APP_CheckSensorInterruptCallback(&maxSensorProc);
        break;
    default:
        if (suspendContext.io.data[io_idx].io_state == IO_AS_BUTTON)
        {
            /* When the gpio is used as button, disable gpio interrupt, then enable gpio interrupt in io timer to
             * receive another gpio interrupt next time(press button, release button) */
            HAL_GpioSetTriggerMode(gpioHandles[port_idx][pin_idx], kHAL_GpioInterruptDisable);
        }
        else
        {
            /*
             * Ignore the interrrupt of gpio to avoid interrupt storm when not used as button(when will enable interrupt
             * of gpio? linux will send command to mcore to enable gpio interrupt after linux get reply from mcore)
             */
            APP_IO_ConfIEvent(NULL, NULL, io_id, SRTM_IoEventNone, false);
        }


        HAL_GpioGetInput(gpioHandles[port_idx][pin_idx], (uint8_t *)&suspendContext.io.data[io_idx].value);
        if ((AD_CurrentMode == AD_PD) && suspendContext.io.data[io_idx].wakeup)
        {
            /* Wakeup A Core(CA35) when A Core is in Power Down Mode */
            //APP_WakeupACore();
            APP_SRTM_WakeupCA35();
        }
        if (suspendContext.io.data[io_idx].timer)
        {
            xTimerStartFromISR(suspendContext.io.data[io_idx].timer, &reschedule);
        }
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

void GPIOA_INT0_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(GPIOA_INT0_IRQn, irqHandlerParam);
    }
    HAL_GpioInterruptHandle(0);
}

void GPIOA_INT1_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(GPIOA_INT1_IRQn, irqHandlerParam);
    }
    HAL_GpioInterruptHandle(0);
}

void GPIOB_INT0_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(GPIOB_INT0_IRQn, irqHandlerParam);
    }
    HAL_GpioInterruptHandle(1);
}

void GPIOB_INT1_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(GPIOB_INT1_IRQn, irqHandlerParam);
    }
    HAL_GpioInterruptHandle(1);
}

void GPIOC_INT0_IRQHandler(void)
{
    if (irqHandler)
    {
        irqHandler(GPIOC_INT0_IRQn, irqHandlerParam);
    }
    HAL_GpioInterruptHandle(2);
}

void GPIOC_INT1_IRQHandler(void)
{
    if (irqHandler)
    {
        irqHandler(GPIOC_INT1_IRQn, irqHandlerParam);
    }
    HAL_GpioInterruptHandle(2);
}

static void rtcAlarmEventTimer_Callback(TimerHandle_t xTimer)
{
    uint32_t status = MU_GetCoreStatusFlags(MU0_MUA);

    if (status & kMU_OtherSideEnterRunFlag) /* A Core in run mode */
    {
        /* Send rpmsg to A Core when Application Domain in active mode(Make sure that DDR is working) */
        SRTM_RtcAdapter_NotifyAlarm(
            rtcAdapter); /* The function SRTM_RtcAdapter_NotifyAlarm will clear alarm interrupt flag */

        xTimerStop(rtcAlarmEventTimer, portMAX_DELAY);
    }
    else
    {
        xTimerStart(rtcAlarmEventTimer, portMAX_DELAY);
    }
}

void BBNSM_IRQHandler(void)
{
    BaseType_t reschedule = pdFALSE;
    uint32_t status       = BBNSM_GetStatusFlags(BBNSM);

    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(BBNSM_IRQn, irqHandlerParam);
    }

    /*
     * Process RTC alarm if present.
     * BBNSM IRQ enable is done in RTC service initialization. So rtcAdapter must be ready.
     */
    if (status & kBBNSM_RTC_AlarmInterruptFlag)
    {
        if (AD_CurrentMode == AD_PD) /* Application Domain is in Power Down Mode */
        {
            /* disable rtc alarm interrupt(when will clear rtc alarm interrupt flag? it will be cleared in
             * rtcAlarmEventTimer) */
            SRTM_RtcAdapter_DisableAlarmInt(rtcAdapter);
            /* Wakeup A Core (A35) */
            APP_WakeupACore();
            /* Send rtc alarm event in timer */
            xTimerStartFromISR(rtcAlarmEventTimer, &reschedule);
        }
        else if (AD_CurrentMode == AD_ACT)
        {
            /* Send rpmsg to A Core when Application Domain in active mode(Make sure that DDR is working) */
            SRTM_RtcAdapter_NotifyAlarm(
                rtcAdapter); /* The function SRTM_RtcAdapter_NotifyAlarm will clear alarm interrupt flag */
        }
    }

    if (status & kBBNSM_EMG_OFF_InterruptFlag)
    {
        /* Clear emergency power off interrupt flag */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_EMG_OFF_InterruptFlag);
    }

    if (status & kBBNSM_PWR_OFF_InterruptFlag)
    {
        if (AD_CurrentMode == AD_DPD) /* Application Domain is in Deep Power Down Mode/Power Down Mode */
        {
            /* Wakeup A Core (A35) */
            APP_SRTM_WakeupCA35();
        }
        else if (AD_CurrentMode == AD_PD)
        {
            /* Wakeup A Core (A35) */
            // APP_WakeupACore();
        }
        /* Clear BBNSM button off interrupt */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_PWR_OFF_InterruptFlag);
        if (keypadService && srtmState == APP_SRTM_StateLinkedUp) /* keypad service is created and linux is ready */
        {
        }
    }
    else if (status & kBBNSM_PWR_ON_InterruptFlag)
    {
        /* Clear BBNSM button on interrupt */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_PWR_ON_InterruptFlag);
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

/*
 * @brief Set pad control register
 * @param asInput    use gpio as input, unless use as output
 */
static void APP_IO_SetPinConfig(uint16_t io_id, bool as_input)
{
    int io_idx       = 0;
    uint8_t io_state = (as_input == true) ? (IO_AS_INPUT) : (IO_AS_OUTPUT);

    io_idx = APP_IO_GetIoIndexByIoId(io_id);
    if (!memcmp(gpioFuncId[io_idx], 0, sizeof(gpioFuncId[io_idx])) &&
        (suspendContext.io.data[io_idx].io_state == IO_UNKOWN ||
         (suspendContext.io.data[io_idx].io_state & io_state) == 0U)) /* Avoid to setup iomux repeatly */
    {
        IOMUXC_SetPinConfig(gpioFuncId[io_idx][0], gpioFuncId[io_idx][1], gpioFuncId[io_idx][2], gpioFuncId[io_idx][3],
                            gpioFuncId[io_idx][4], as_input ? (inputMask) : (outputMask));
    }
}

static srtm_status_t APP_IO_ConfOutput(uint16_t io_id, srtm_io_value_t io_value)
{
    uint8_t port_idx                = GPIO_PORT_IDX(io_id);
    uint8_t pin_idx                 = GPIO_PIN_IDX(io_id);
    uint8_t io_idx                  = APP_IO_GetIoIndexByIoId(io_id);
    hal_gpio_pin_config_t pinConfig = {
        kHAL_GpioDirectionOut, io_value, port_idx, pin_idx, APP_GPIO_INT_SEL, NotAvail_IRQn,
    };

    assert(port_idx < GPIO_MODULE_NUM); /* We only support GPIOA and GPIOB */
    assert(pin_idx < PIN_NUM_PER_GPIO_MODULE);

    /* setup pin as gpio function */
    APP_IO_SetPinConfig(io_id, false);

    /* set gpio as output */
    pinConfig.irq = suspendContext.io.data[io_idx].irq;
    if ((suspendContext.io.data[io_idx].io_state & IO_AS_OUTPUT_MASK) == 0U)
    {
        HAL_GpioDeinit(gpioHandles[port_idx][pin_idx]);
        suspendContext.io.data[io_idx].io_state &= ~IO_AS_INPUT_MASK;
        suspendContext.io.data[io_idx].io_state |= IO_AS_OUTPUT;
    }
    HAL_GpioInit(gpioHandles[port_idx][pin_idx], &pinConfig);
    HAL_GpioSetOutput(gpioHandles[port_idx][pin_idx], (uint8_t)io_value);

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_SetOutput(srtm_service_t service,
                                      srtm_peercore_t core,
                                      uint16_t io_id,
                                      srtm_io_value_t ioValue)
{
    uint8_t index = APP_IO_GetIoIndexByIoId(io_id);

    assert(index < TOTAL_PINS);

    suspendContext.io.data[index].value = (uint8_t)ioValue;

    return APP_IO_ConfOutput(io_id, ioValue);
}

static srtm_status_t APP_IO_GetInput(srtm_service_t service,
                                     srtm_peercore_t core,
                                     uint16_t io_id,
                                     srtm_io_value_t *pIoValue)
{
    uint8_t port_idx = GPIO_PORT_IDX(io_id);
    uint8_t pin_idx  = GPIO_PIN_IDX(io_id);

    assert(port_idx < GPIO_MODULE_NUM); /* support GPIOA, GPIOB and GPIOC */
    assert(pin_idx < PIN_NUM_PER_GPIO_MODULE);
    assert(pIoValue);

    HAL_GpioGetInput(gpioHandles[port_idx][pin_idx], (uint8_t *)pIoValue);

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_ConfInput(uint16_t io_id, srtm_io_event_t event, bool wakeup)
{
    uint8_t port_idx = GPIO_PORT_IDX(io_id);
    uint8_t pin_idx  = GPIO_PIN_IDX(io_id);
    uint8_t wuu_idx  = APP_IO_GetWUUPinByIoId(io_id);
    uint8_t io_idx   = APP_IO_GetIoIndexByIoId(io_id);
    wuu_external_wakeup_pin_config_t config;
    hal_gpio_pin_config_t pinConfig = {
        kHAL_GpioDirectionIn, 0U, port_idx, pin_idx, APP_GPIO_INT_SEL, NotAvail_IRQn,
    };

    assert(port_idx < GPIO_MODULE_NUM);
    assert(pin_idx < PIN_NUM_PER_GPIO_MODULE);
    assert(wuu_idx <= ARRAY_SIZE(wuuPins)); /* When wuuIdx == ARRAY_SIZE(wuuPins),
                                              it means there's no WUU pin for io_id. */
    config.event = kWUU_ExternalPinInterrupt;
    config.mode  = kWUU_ExternalPinActiveAlways;

    /* set pin as gpio function */
    APP_IO_SetPinConfig(io_id, true);

    /* set gpio as input */
    pinConfig.irq = suspendContext.io.data[io_idx].irq;
    if ((suspendContext.io.data[io_idx].io_state & IO_AS_INPUT_MASK) == 0U)
    {
        HAL_GpioDeinit(gpioHandles[port_idx][pin_idx]);
        suspendContext.io.data[io_idx].io_state &= ~IO_AS_OUTPUT_MASK;
        suspendContext.io.data[io_idx].io_state |= IO_AS_INPUT;
    }
    HAL_GpioInit(gpioHandles[port_idx][pin_idx], &pinConfig);
    switch (event)
    {
        case SRTM_IoEventRisingEdge:
            HAL_GpioSetTriggerMode(gpioHandles[port_idx][pin_idx], kHAL_GpioInterruptRisingEdge);
            if (wakeup)
            {
                assert(wuu_idx < ARRAY_SIZE(wuuPins));
                config.edge = kWUU_ExternalPinRisingEdge;
                WUU_SetExternalWakeUpPinsConfig(WUU0, wuu_idx, &config);
            }
            break;
        case SRTM_IoEventFallingEdge:
            HAL_GpioSetTriggerMode(gpioHandles[port_idx][pin_idx], kHAL_GpioInterruptFallingEdge);
            if (wakeup)
            {
                assert(wuu_idx < ARRAY_SIZE(wuuPins));
                config.edge = kWUU_ExternalPinFallingEdge;
                WUU_SetExternalWakeUpPinsConfig(WUU0, wuu_idx, &config);
            }
            break;
        case SRTM_IoEventEitherEdge:
            HAL_GpioSetTriggerMode(gpioHandles[port_idx][pin_idx], kHAL_GpioInterruptEitherEdge);
            if (wakeup)
            {
                assert(wuu_idx < ARRAY_SIZE(wuuPins));
                config.edge = kWUU_ExternalPinAnyEdge;
                WUU_SetExternalWakeUpPinsConfig(WUU0, wuu_idx, &config);
            }
            break;
        case SRTM_IoEventLowLevel:
            HAL_GpioSetTriggerMode(gpioHandles[port_idx][pin_idx], kHAL_GpioInterruptLogicZero);
            /* Power level cannot trigger wakeup */
            assert(!wakeup);
            break;
        case SRTM_IoEventHighLevel:
            HAL_GpioSetTriggerMode(gpioHandles[port_idx][pin_idx], kHAL_GpioInterruptLogicOne);
            /* Power level cannot trigger wakeup */
            assert(!wakeup);
            break;
        default:
            HAL_GpioSetTriggerMode(gpioHandles[port_idx][pin_idx], kHAL_GpioInterruptDisable);
            break;
    }
    if (!wakeup && wuu_idx < ARRAY_SIZE(wuuPins))
    {
        config.edge = kWUU_ExternalPinDisable;
        WUU_SetExternalWakeUpPinsConfig(WUU0, wuu_idx, &config);
    }

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_ConfIEvent(
    srtm_service_t service, srtm_peercore_t core, uint16_t io_id, srtm_io_event_t event, bool wakeup)
{
    uint8_t io_idx = APP_IO_GetIoIndexByIoId(io_id);

    assert(io_idx < TOTAL_PINS);

    suspendContext.io.data[io_idx].event  = event;
    suspendContext.io.data[io_idx].wakeup = wakeup;

    return APP_IO_ConfInput(io_id, event, wakeup);
}

static srtm_status_t APP_IO_ConfKEvent(
    srtm_service_t service, srtm_peercore_t core, uint8_t key_idx, srtm_keypad_event_t event, bool wakeup)
{
    uint8_t io_idx = APP_Keypad_GetIoIndex(key_idx);

    assert(io_idx < TOTAL_PINS);

    suspendContext.io.data[io_idx].event  = APP_Keypad_GetIoEvent(key_idx, event);
    suspendContext.io.data[io_idx].wakeup = wakeup;

    return APP_IO_ConfInput(io_idx, suspendContext.io.data[io_idx].event, wakeup);
}

static void APP_SRTM_DoWakeup(void *param)
{
    APP_WakeupACore();
}

static void APP_SRTM_DoWakeupCA35(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    if (!core || (core && SRTM_PeerCore_GetState(core) == SRTM_PeerCore_State_Deactivated))
    {
        APP_SRTM_DoWakeup(param1);
        APP_SRTM_StartCommunication();
    }
}

static void APP_SRTM_PollLinkup(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    if (srtmState == APP_SRTM_StateRun)
    {
        if (rpmsg_lite_is_link_up(rpmsgHandle))
        {
            srtmState = APP_SRTM_StateLinkedUp;
            xSemaphoreGive(monSig);
        }
        else
        {
            /* Start timer to poll linkup status. */
            xTimerStart(linkupTimer, portMAX_DELAY);
        }
    }
}

static void APP_RefreshS400WdgTimerCallback(TimerHandle_t xTimer)
{
    SENTINEL_Ping();
    PRINTF("\r\n %s: %d ping s400 wdg timer ok\r\n", __func__, __LINE__);
    xTimerStart(refreshS400WdgTimer, portMAX_DELAY);
}

static void APP_RestoreRegValOfMuTimerCallback(TimerHandle_t xTimer)
{
    rtdCtxRestore();
    xTimerStart(restoreRegValOfMuTimer, portMAX_DELAY);
}

static void APP_LinkupTimerCallback(TimerHandle_t xTimer)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_PollLinkup, NULL, NULL);

    if (proc)
    {
        SRTM_Dispatcher_PostProc(disp, proc);
    }
}

static void APP_SRTM_NotifyPeerCoreReady(struct rpmsg_lite_instance *rpmsgHandle, bool ready)
{
    /* deinit and init app task(str_echo/pingpong rpmsg) in APP_SRTM_StateReboot only */
    if (rpmsgMonitor && (srtmState == APP_SRTM_StateReboot))
    {
        rpmsgMonitor(rpmsgHandle, ready, rpmsgMonitorParam);
    }
}

static void APP_SRTM_Linkup(void)
{
    srtm_channel_t chan;
    srtm_rpmsg_endpoint_config_t rpmsgConfig;

    /* Inform upower that m33 is using the ddr */
    UPOWER_SetRtdUseDdr(true);

    /* Create SRTM peer core */
    core = SRTM_PeerCore_Create(PEER_CORE_ID);
    /* Set peer core state to activated */
    SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Activated);

    /* Common RPMsg channel config */
    rpmsgConfig.localAddr   = RL_ADDR_ANY;
    rpmsgConfig.peerAddr    = RL_ADDR_ANY;
    rpmsgConfig.rpmsgHandle = rpmsgHandle;

    /* Create and add SRTM I2C channel to peer core*/
    rpmsgConfig.epName = APP_SRTM_I2C_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM AUDIO channel to peer core*/
    rpmsgConfig.epName = APP_SRTM_AUDIO_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);
    assert((audioService != NULL) && (saiAdapter != NULL));
    SRTM_AudioService_BindChannel(audioService, saiAdapter, chan);

    /* Create and add SRTM Keypad channel to peer core */
    rpmsgConfig.epName = APP_SRTM_KEYPAD_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM IO channel to peer core */
    rpmsgConfig.epName = APP_SRTM_IO_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM PWM channel to peer core */
    rpmsgConfig.epName = APP_SRTM_PWM_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM RTC channel to peer core */
    rpmsgConfig.epName = APP_SRTM_RTC_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM Life Cycle channel to peer core */
    rpmsgConfig.epName = APP_SRTM_LFCL_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM Sensor channel to peer core */
    if (lsmSensorReady && maxSensorReady)
    {
        rpmsgConfig.epName = APP_SRTM_SENSOR_CHANNEL_NAME;
        chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
        SRTM_PeerCore_AddChannel(core, chan);
    }

    SRTM_Dispatcher_AddPeerCore(disp, core);
}

static void APP_SRTM_InitPeerCore(void)
{
    copyResourceTable();

    rpmsgHandle = rpmsg_lite_remote_init((void *)RPMSG_LITE_SRTM_SHMEM_BASE, RPMSG_LITE_SRTM_LINK_ID, RL_NO_FLAGS);
    assert(rpmsgHandle);

    /* save context, such as: MU0_MUA[RCR] */
    rtdCtxSave();

    APP_SRTM_NotifyPeerCoreReady(rpmsgHandle, true);

    if (rpmsg_lite_is_link_up(rpmsgHandle))
    {
        APP_SRTM_Linkup();
    }
    else
    {
        /* Start timer to poll linkup status. */
        xTimerStart(linkupTimer, portMAX_DELAY);
    }
}

static srtm_status_t APP_SRTM_Lsm_Sensor_Init(void)
{
    /*
     * Create a timer to send sensor tilt wakeup event to A Core after A Core is waken by (avoid losting
     * sensor tilt wakeup interrupt)
     */
    LSM_InitUserBank_NoRst(&lsmHandle);

    if (sensorTiltWakeupEventTimer == NULL)
    {
        sensorTiltWakeupEventTimer =
            xTimerCreate("sensorTiltWakeupEventTimer", APP_MS2TICK(APP_SENSOR_TILT_WAKEUP_EVT_TIMER_PERIOD_MS), pdFALSE,
                         NULL, sensorTiltWakeupEventTimer_Callback);
        assert(sensorTiltWakeupEventTimer);
    }

    return SRTM_Status_Success;
}

static srtm_status_t APP_SRTM_Lsm_Sensor_Deinit(void)
{
    return (kStatus_Success == LSM_Deinit(&lsmHandle)) ? SRTM_Status_Success : SRTM_Status_Error;
}

static srtm_status_t APP_SRTM_Max_Sensor_Init(void)
{
    /* Init timer */
    if (NULL == sensorMaxFlushFiFoTimer)
    {
        sensorMaxFlushFiFoTimer =
            xTimerCreate("sensorMaxFlushFiFoTimer", APP_MS2TICK(APP_SENSOR_MAX_IRQ_CHECK_TIMER_PERIOD_MS), pdTRUE,
                         NULL, sensorMaxFlushFiFoTimer_Callback);
        assert(sensorMaxFlushFiFoTimer);
    }
    /* As MAX should have been initialized, we use NULL for config here. */
    if (kStatus_Success == MAX_Init(&maxHandle, NULL))
    {
        return SRTM_Status_Success;
    }
    else
    {
        return SRTM_Status_Error;
    }
}

static srtm_status_t APP_SRTM_Max_Sensor_Deinit(void)
{
    if (kStatus_Success == MAX_Deinit(&maxHandle))
    {
        return SRTM_Status_Success;
    }
    else
    {
        return SRTM_Status_Error;
    }
}

static srtm_status_t APP_SRTM_Max_Sensor_Init_Temp(void)
{
    if (kStatus_Success == MAX_InitTemp(&maxHandle))
    {
        return SRTM_Status_Success;
    }
    else
    {
        return SRTM_Status_Error;
    }
}

static srtm_status_t APP_SRTM_Max_Sensor_Deinit_Temp(void)
{
    if (kStatus_Success == MAX_DeinitTemp(&maxHandle))
    {
        return SRTM_Status_Success;
    }
    else
    {
        return SRTM_Status_Error;
    }
}

static srtm_status_t APP_SRTM_Sensor_EnableStateDetector(srtm_sensor_adapter_t adapter,
                                                         srtm_sensor_type_t type,
                                                         uint8_t index,
                                                         bool enable)
{
    srtm_status_t status = SRTM_Status_Success;

    switch (type)
    {
    case SRTM_SensorTypePedometer:
    case SRTM_SensorTypeTilt:
        if (enable)
        {
            if (!lsm_sensor.stateEnabled && !lsm_sensor.dataEnabled)
            {
                status = APP_SRTM_Lsm_Sensor_Init();
            }
            if (status == SRTM_Status_Success)
            {
                lsm_sensor.stateEnabled = true;
            }
        }
        else if (lsm_sensor.stateEnabled)
        {
            lsm_sensor.stateEnabled = false;
            if (!lsm_sensor.dataEnabled)
            {
                status = APP_SRTM_Lsm_Sensor_Deinit();
            }
        }
        break;
    case SRTM_SensorTypeHeartRate:
    case SRTM_SensorTypeSpO2:
        if (enable)
        {
            if (!max_sensor.stateEnabled && !max_sensor.dataEnabled)
            {
                status = APP_SRTM_Max_Sensor_Init();
            }
            if (status == SRTM_Status_Success)
            {
                max_sensor.stateEnabled = true;
            }
        }
        else if (max_sensor.stateEnabled)
        {
            max_sensor.stateEnabled = false;
            if (!max_sensor.dataEnabled)
            {
                status = APP_SRTM_Max_Sensor_Deinit();
            }
        }
        break;
    case SRTM_SensorTypeTemperature:
        if (enable)
        {
            status = APP_SRTM_Max_Sensor_Init_Temp();
        }
        else
        {
            status = APP_SRTM_Max_Sensor_Deinit_Temp();
        }
        break;
    default:
        return SRTM_Status_InvalidParameter;
    }

    return status;
}

static srtm_status_t APP_SRTM_Sensor_EnableDataReport(srtm_sensor_adapter_t adapter,
                                                      srtm_sensor_type_t type,
                                                      uint8_t index,
                                                      bool enable)
{
    srtm_status_t status = SRTM_Status_Success;

    switch (type)
    {
    case SRTM_SensorTypePedometer:
        if (enable && !lsm_sensor.dataEnabled)
        {
            if (!lsm_sensor.stateEnabled)
            {
                /* Initialize Pedometer. */
                status = APP_SRTM_Lsm_Sensor_Init();
            }
            if (status == SRTM_Status_Success)
            {
                uint16_t stepCnt;

                LSM_GetPedometerCnt(&lsmHandle, &stepCnt);
                lsm_sensor.dataEnabled   = true;
                lsm_sensor.pedometer.cnt = stepCnt;
            }
        }
        else if (!enable && lsm_sensor.dataEnabled)
        {
            lsm_sensor.dataEnabled = false;
            if (!lsm_sensor.stateEnabled)
            {
                status = APP_SRTM_Lsm_Sensor_Deinit();
            }
        }
        break;
    case SRTM_SensorTypeHeartRate:
    case SRTM_SensorTypeSpO2:
        if (enable && !max_sensor.dataEnabled)
        {
            if (!max_sensor.stateEnabled)
            {
                /* Initialize HrSpO2. */
                status = APP_SRTM_Max_Sensor_Init();
            }
            if (status == SRTM_Status_Success)
            {
                memset(&g_max_sensor_data_buf, 0, sizeof(max_sample_buf_t));
                if (kStatus_Success == MAX_Start_HrSpO2(&maxHandle, &g_maxConfig, true))
                {
                    max_sensor.dataEnabled   = true;
                }
                xTimerStart(sensorMaxFlushFiFoTimer, pdMS_TO_TICKS(10));
            }
        }
        else if (!enable && max_sensor.dataEnabled)
        {
            if (kStatus_Success == MAX_Start_HrSpO2(&maxHandle, &g_maxConfig, false))
            {
                max_sensor.dataEnabled = false;
            }
            if (!max_sensor.stateEnabled)
            {
                status = APP_SRTM_Max_Sensor_Deinit();
            }
        }
        break;
    case SRTM_SensorTypeTemperature:
        if (enable)
        {
            MAX_StartTemp(&maxHandle);
        }
        else
        {
            status = APP_SRTM_Max_Sensor_Deinit_Temp();
        }
        break;
    case SRTM_SensorTypeTilt:
    default:
        return SRTM_Status_InvalidParameter;
    }

    return status;
}

static srtm_status_t APP_SRTM_Sensor_SetPollDelay(srtm_sensor_adapter_t adapter,
                                                  srtm_sensor_type_t type,
                                                  uint8_t index,
                                                  uint32_t millisec)
{
    if (type != SRTM_SensorTypePedometer)
    {
        /* Only support pedometer now. */
        return SRTM_Status_InvalidParameter;
    }

    if (millisec > APP_PEDOMETER_POLL_DELAY_MAX || millisec < APP_PEDOMETER_POLL_DELAY_MIN)
    {
        return SRTM_Status_InvalidParameter;
    }

    lsm_sensor.pollDelay = millisec;

    return SRTM_Status_Success;
}

static void APP_SRTM_GpioReset(void)
{
    int32_t i;

    /* First disable all GPIO interrupts configured by CA35 */
    for (i = 0; i < ARRAY_SIZE(suspendContext.io.data); i++)
    {
        if (suspendContext.io.data[i].timer)
        {
            xTimerStop(suspendContext.io.data[i].timer, portMAX_DELAY);
            if (suspendContext.io.data[i].overridden)
            {
                /* The IO is configured by CM33 instead of CA35, don't reset HW configuration. */
                suspendContext.io.data[i].event  = SRTM_IoEventNone;
                suspendContext.io.data[i].wakeup = false;
            }
            else
            {
                APP_IO_ConfIEvent(NULL, NULL, suspendContext.io.data[i].io_id, SRTM_IoEventNone, false);
            }
        }
    }

    /* Output pin value doesn't change. */

    /* Then reset IO service */
    SRTM_IoService_Reset(ioService, core);
    SRTM_KeypadService_Reset(keypadService, core);
}

static void APP_SRTM_ResetServices(void)
{
    /* When CA35 resets, we need to avoid async event to send to CA35. Audio and IO services have async events. */
    SRTM_AudioService_Reset(audioService, core);
    SRTM_RtcService_Reset(rtcService, core);
    if (sensorService != NULL)
    {
        SRTM_SensorService_Reset(sensorService, core);
    }
    APP_SRTM_GpioReset();
}

static void APP_SRTM_DeinitPeerCore(void)
{
    /* Stop linkupTimer if it's started. */
    xTimerStop(linkupTimer, portMAX_DELAY);

    /* Notify application for the peer core disconnection. */
    APP_SRTM_NotifyPeerCoreReady(rpmsgHandle, false);

    if (core)
    {
        /* Need to let services know peer core is now down. */
        APP_SRTM_ResetServices();

        SRTM_Dispatcher_RemovePeerCore(disp, core);
        SRTM_PeerCore_Destroy(core);
        core = NULL;
    }

    if (rpmsgHandle)
    {
        rpmsg_lite_deinit(rpmsgHandle);
        rpmsgHandle = NULL;
    }

    /* Inform upower that m33 is not using the ddr(it's ready to reset ddr of lpavd) */
    UPOWER_SetRtdUseDdr(false);
}

static void APP_SRTM_InitAudioDevice(void)
{
    edma_config_t dmaConfig;

    /* Initialize DMA0 for SAI */
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(DMA0, &dmaConfig);

    /* Initialize DMAMUX for SAI */
    EDMA_SetChannelMux(DMA0, APP_SAI_TX_DMA_CHANNEL, kDmaRequestMux0SAI0Tx);
    EDMA_SetChannelMux(DMA0, APP_SAI_RX_DMA_CHANNEL, kDmaRequestMux0SAI0Rx);
}

/* APP_SRTM_EnableLPAV and APP_SRTM_DisableLPAV function logic is for dualboot mode, RTD hold LPAV domain */
void APP_SRTM_EnableLPAV()
{
    UPOWER_ReadPmicReg(PCA9460_BUCK3CTRL_ADDR, &(buck3_ctrl.val));
    UPOWER_ReadPmicReg(PCA9460_LDO1_CFG_ADDR, &(ldo1_cfg.val));

    if (!(ldo1_cfg.reg.L1_ENMODE))
    {
        UPOWER_SetRtdUseDdr(true);

        if (!BOARD_IsLPAVOwnedByRTD())
        {
            /* Set LPAV ownership to RTD */
            SIM_SEC->SYSCTRL0 = 0;
        }

        /* APD default hold LPAV domain device and needs change the ownership of the LPAV device from APD to RTD */
        SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0;
        SIM_SEC->LPAV_SLAVE_ALLOC_CTRL  = 0;

        /* Power on LPAV domain */
        UPOWER_PowerOnMemPart(
            (uint32_t)(kUPOWER_MP0_DCNANO_A | kUPOWER_MP0_DCNANO_B | kUPOWER_MP0_EPDC_A | kUPOWER_MP0_EPDC_B |
                       kUPOWER_MP0_DMA2 | kUPOWER_MP0_GPU2D_A | kUPOWER_MP0_GPU2D_B | kUPOWER_MP0_GPU3D_A |
                       kUPOWER_MP0_GPU3D_B | kUPOWER_MP0_HIFI4 | kUPOWER_MP0_ISI | kUPOWER_MP0_MIPI_CSI |
                       kUPOWER_MP0_MIPI_DSI | kUPOWER_MP0_PXP | kUPOWER_MP0_AV_SYSTEM),
            0U);

        UPOWER_PowerOnSwitches((upower_ps_mask_t)(kUPOWER_PS_GPU3D | kUPOWER_PS_HIFI4 | kUPOWER_PS_DDRC |
                                                  kUPOWER_PS_PXP_EPDC | kUPOWER_PS_MIPI_DSI | kUPOWER_PS_MIPI_CSI |
                                                  kUPOWER_PS_AV_NIC | kUPOWER_PS_FUSION_AO));

        /* Restore BUCK3 voltage to 1.1V, please refer to PCA9460 for the specific data */
        UPOWER_SetPmicReg(PCA9460_BUCK3OUT_DVS0_ADDR, 0x28);

        ldo1_cfg.reg.L1_ENMODE = 0x1; /* 00-ON */
        ldo1_cfg.reg.L1_LPMODE = 0x3; /* Normal mode (default) */
        ldo1_cfg.reg.L1_LLSEL  = 0x1; /* 15 mw (default) */
        ldo1_cfg.reg.L1_CSEL   = 0x2; /* Auto Cout detection (default) */

        /* Poweron LDO1 */
        UPOWER_SetPmicReg(PCA9460_LDO1_CFG_ADDR, ldo1_cfg.val);

        /* Init LPAV clocks */
        BOARD_LpavInit();

        /* Workaround: If DDR controller register read failed, need to toggle pS7, PS8, PS13, PS14 */
        while (!LPDDR->DENALI_CTL[0])
        {
            /* Toggle pS7, PS8, PS13, PS14 */
            UPOWER_PowerOffSwitches(
                (upower_ps_mask_t)(kUPOWER_PS_GPU3D | kUPOWER_PS_HIFI4 | kUPOWER_PS_AV_NIC | kUPOWER_PS_FUSION_AO));
            UPOWER_PowerOnSwitches(
                (upower_ps_mask_t)(kUPOWER_PS_GPU3D | kUPOWER_PS_HIFI4 | kUPOWER_PS_AV_NIC | kUPOWER_PS_FUSION_AO));
        }

        /* Control DRAM exit from self-refesh */
        BOARD_DramExitRetention(dram_class, dram_timing_cfg);
    }
}

void APP_SRTM_DisableLPAV()
{
    UPOWER_ReadPmicReg(PCA9460_BUCK3CTRL_ADDR, &(buck3_ctrl.val));
    UPOWER_ReadPmicReg(PCA9460_LDO1_CFG_ADDR, &(ldo1_cfg.val));

    if (ldo1_cfg.reg.L1_ENMODE)
    {
        /* Save lpav context */
        BOARD_LpavSave();

        /* Save DDR controller registers */
        BOARD_DdrSave();

        /* DDR will enter retention state when LPAV master domain enter Power down */
        BOARD_DramEnterRetention();

        ldo1_cfg.reg.L1_ENMODE = 0x0; /* 00-OFF */
        ldo1_cfg.reg.L1_LPMODE = 0x3; /* Normal mode (default) */
        ldo1_cfg.reg.L1_LLSEL  = 0x1; /* 15 mw (default) */
        ldo1_cfg.reg.L1_CSEL   = 0x2; /* Auto Cout detection (default) */

        /* Poweroff LDO1 */
        UPOWER_SetPmicReg(PCA9460_LDO1_CFG_ADDR, ldo1_cfg.val);

#ifndef SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
        /* For sai_low_power_audio, reduce buck3 power will impact fucntion */
        /* Set BUCK3 voltage to 0.7375V, please refer to PCA9460 for the specific data */
        UPOWER_SetPmicReg(PCA9460_BUCK3OUT_DVS0_ADDR, 0x0B);
#endif

        if (!BOARD_IsLPAVOwnedByRTD())
        {
            /* Set LPAV ownership to RTD */
            SIM_SEC->SYSCTRL0 = 0;
        }

        /* APD default hold LPAV domain device and needs change the ownership of the LPAV device from APD to RTD */
        SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0;
        SIM_SEC->LPAV_SLAVE_ALLOC_CTRL  = 0;

        UPOWER_PowerOffSwitches((upower_ps_mask_t)(kUPOWER_PS_GPU3D | kUPOWER_PS_HIFI4 | kUPOWER_PS_DDRC |
                                                   kUPOWER_PS_PXP_EPDC | kUPOWER_PS_MIPI_DSI | kUPOWER_PS_MIPI_CSI |
                                                   kUPOWER_PS_AV_NIC | kUPOWER_PS_FUSION_AO));

        UPOWER_PowerOffMemPart(
            (uint32_t)(kUPOWER_MP0_DCNANO_A | kUPOWER_MP0_DCNANO_B | kUPOWER_MP0_EPDC_A | kUPOWER_MP0_EPDC_B |
                       kUPOWER_MP0_DMA2 | kUPOWER_MP0_GPU2D_A | kUPOWER_MP0_GPU2D_B | kUPOWER_MP0_GPU3D_A |
                       kUPOWER_MP0_GPU3D_B | kUPOWER_MP0_HIFI4 | kUPOWER_MP0_ISI | kUPOWER_MP0_MIPI_CSI |
                       kUPOWER_MP0_MIPI_DSI | kUPOWER_MP0_PXP | kUPOWER_MP0_AV_SYSTEM),
            0U);

        /*
         * Workaround: After Mcore hold lpav in dualboot mode, Mcore can't resume from PD/DPD modes in some boards.
         * Restore lpav master and slave devices can fix it.
         */
        SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0x7F;
        SIM_SEC->LPAV_SLAVE_ALLOC_CTRL  = 0x1F;
        UPOWER_SetRtdUseDdr(false);
    }
}

#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
/*
 * For sai low power audio demo, when Acore enter Power down mode, Mcore will
 * put ddr into self-refresh-->transfer data by dma and play audio-->put ddr exit self-refresh-->memcpy data from ddr to
 * ssram
 */
void APP_SRTM_PreCopyCallback()
{
    if (R32(DDR_IN_SELFREFRESH_BASE) || (SRTM_PeerCore_GetState(core) == SRTM_PeerCore_State_Deactivated))
    {
        /* Disable GAIO BBNSM IRQ before operate LPAV */
        DisableIRQ(BBNSM_IRQn);
        DisableIRQ(GPIOA_INT0_IRQn);
        DisableIRQ(GPIOA_INT1_IRQn);
        DisableIRQ(GPIOB_INT0_IRQn);
        DisableIRQ(GPIOB_INT1_IRQn);
        DisableIRQ(WUU0_IRQn);

        /* Poweron LPAV domain, ddr exit retention */
        APP_SRTM_EnableLPAV();
        W32(DDR_IN_SELFREFRESH_BASE, 0);

        EnableIRQ(GPIOA_INT0_IRQn);
        EnableIRQ(GPIOA_INT1_IRQn);
        EnableIRQ(GPIOB_INT0_IRQn);
        EnableIRQ(GPIOB_INT1_IRQn);
        EnableIRQ(BBNSM_IRQn);
        EnableIRQ(WUU0_IRQn);
    }
}

void APP_SRTM_PostCopyCallback()
{
    if (SRTM_PeerCore_GetState(core) == SRTM_PeerCore_State_Deactivated)
    {
        /* Disable GAIO BBNSM IRQ before operate LPAV */
        DisableIRQ(BBNSM_IRQn);
        DisableIRQ(GPIOA_INT0_IRQn);
        DisableIRQ(GPIOA_INT1_IRQn);
        DisableIRQ(GPIOB_INT0_IRQn);
        DisableIRQ(GPIOB_INT1_IRQn);
        DisableIRQ(WUU0_IRQn);

        /* Poweroff LPAV domain, put ddr into retention */
        APP_SRTM_DisableLPAV();

        /* In dualboot mode, RTD hold LPAV domain, set LPAV ownership to APD let ddr into retention */
        SIM_SEC->SYSCTRL0               = 0xA70480;
        SIM_SEC->LPAV_MASTER_ALLOC_CTRL = 0x7F;
        SIM_SEC->LPAV_SLAVE_ALLOC_CTRL  = 0x1F;

        W32(DDR_IN_SELFREFRESH_BASE, 1);

        EnableIRQ(GPIOA_INT0_IRQn);
        EnableIRQ(GPIOA_INT1_IRQn);
        EnableIRQ(GPIOB_INT0_IRQn);
        EnableIRQ(GPIOB_INT1_IRQn);
        EnableIRQ(BBNSM_IRQn);
        EnableIRQ(WUU0_IRQn);
    }
}
#endif

static void APP_SRTM_InitAudioService(void)
{
    srtm_sai_edma_config_t saiTxConfig;
    srtm_sai_edma_config_t saiRxConfig;

    APP_SRTM_InitAudioDevice();

    memset(&saiTxConfig, 0, sizeof(saiTxConfig));
    memset(&saiRxConfig, 0, sizeof(saiRxConfig));

    /*  Set SAI DMA IRQ Priority. */
    NVIC_SetPriority(APP_DMA_IRQN(APP_SAI_TX_DMA_CHANNEL), APP_SAI_TX_DMA_IRQ_PRIO);
    NVIC_SetPriority(APP_DMA_IRQN(APP_SAI_RX_DMA_CHANNEL), APP_SAI_RX_DMA_IRQ_PRIO);
    NVIC_SetPriority(APP_SRTM_SAI_IRQn, APP_SAI_IRQ_PRIO);

    /* Create SAI EDMA adapter */
    SAI_GetClassicI2SConfig(&saiTxConfig.config, kSAI_WordWidth16bits, kSAI_Stereo, kSAI_Channel0Mask);
    saiTxConfig.config.syncMode           = kSAI_ModeSync; /* Tx in Sync mode */
    saiTxConfig.config.fifo.fifoWatermark = FSL_FEATURE_SAI_FIFO_COUNTn(APP_SRTM_SAI) - 1;
    saiTxConfig.mclk                      = CLOCK_GetIpFreq(kCLOCK_Sai0);
#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
    saiTxConfig.stopOnSuspend = false; /* Keep playing audio on APD suspend. */
#else
    saiTxConfig.stopOnSuspend = true;
#endif
    saiTxConfig.threshold = 1U; /* Every period transmitted triggers periodDone message to A core. */
    saiTxConfig.guardTime =
        1000; /* Unit:ms. This is a lower limit that M core should reserve such time data to wakeup A core. */
    saiTxConfig.dmaChannel = APP_SAI_TX_DMA_CHANNEL;

    SAI_GetClassicI2SConfig(&saiRxConfig.config, kSAI_WordWidth16bits, kSAI_Stereo, kSAI_Channel0Mask);
    saiRxConfig.config.syncMode           = kSAI_ModeAsync; /* Rx in async mode */
    saiRxConfig.config.fifo.fifoWatermark = 1;
    saiRxConfig.mclk                      = saiTxConfig.mclk;
#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
    saiRxConfig.stopOnSuspend = false; /* Keep recording data on APD suspend. */
#else
    saiRxConfig.stopOnSuspend = true;
#endif
    saiRxConfig.threshold  = UINT32_MAX; /* Every period received triggers periodDone message to A core. */
    saiRxConfig.dmaChannel = APP_SAI_RX_DMA_CHANNEL;

    saiAdapter = SRTM_SaiEdmaAdapter_Create(SAI0, DMA0, &saiTxConfig, &saiRxConfig);
    assert(saiAdapter);

#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
    SRTM_SaiEdmaAdapter_SetTxLocalBuf(saiAdapter, &g_local_buf);
    SRTM_SaiEdmaAdapter_SetTxPreCopyCallback(saiAdapter, APP_SRTM_PreCopyCallback);
    SRTM_SaiEdmaAdapter_SetTxPostCopyCallback(saiAdapter, APP_SRTM_PostCopyCallback);
#endif

    /* Create and register audio service */
    audioService = SRTM_AudioService_Create(saiAdapter, NULL);
    SRTM_Dispatcher_RegisterService(disp, audioService);
}

static void APP_SRTM_InitPwmDevice(void)
{
    HAL_PwmInit(pwmHandles[0], 0U, CLOCK_GetTpmClkFreq(0U));
}

static void APP_SRTM_InitPwmService(void)
{
    srtm_pwm_adapter_t pwmAdapter;

    APP_SRTM_InitPwmDevice();
    pwmAdapter = SRTM_PwmAdapter_Create(pwmHandles, ARRAY_SIZE(pwmHandles));
    assert(pwmAdapter);

    /* Create and register pwm service */
    pwmService = SRTM_PwmService_Create(pwmAdapter);
    SRTM_Dispatcher_RegisterService(disp, pwmService);
}

static void APP_SRTM_InitI2CDevice(void)
{
    lpi2c_master_config_t masterConfig;

    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = LPI2C0_BAUDRATE;
    LPI2C_MasterInit(LPI2C0, &masterConfig, I2C_SOURCE_CLOCK_FREQ_LPI2C0);
    masterConfig.baudRate_Hz = LPI2C1_BAUDRATE;
    LPI2C_MasterInit(LPI2C1, &masterConfig, I2C_SOURCE_CLOCK_FREQ_LPI2C1);
}

static void APP_SRTM_InitI2CService(void)
{
    APP_SRTM_InitI2CDevice();
    i2cService = SRTM_I2CService_Create(&i2c_adapter);
    SRTM_Dispatcher_RegisterService(disp, i2cService);
}

static void APP_CheckSensorInterrupt_RecycleMessage(srtm_message_t msg, void *param)
{
    srtm_procedure_t *sensorProc = (srtm_procedure_t *)param;

    assert(*sensorProc == NULL);

    *sensorProc = msg;
}

static status_t APP_SRTM_InitLsmSensorDevice(void)
{
    lsm_config_t config = {0};
    status_t result;
    uint8_t i               = 0;
    uint8_t array_addr_size = 0;
    uint8_t port_idx        = 0;
    uint8_t pin_idx         = 0;

    config.I2C_SendFunc    = BOARD_Accel_I2C_Send;
    config.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;

    array_addr_size = sizeof(g_lsm_sensor_address) / sizeof(g_lsm_sensor_address[0]);
    for (i = 0; i < array_addr_size; i++)
    {
        config.slaveAddress = g_lsm_sensor_address[i];
        result              = LSM_Init(&lsmHandle, &config);
        if (result == kStatus_Success)
        {
            /* Setup gpio interrupt trigger type */
            port_idx = GPIO_PORT_IDX(APP_PIN_LSM6DSO_INT1);
            pin_idx  = GPIO_PIN_IDX(APP_PIN_LSM6DSO_INT1);
            HAL_GpioSetTriggerMode(gpioHandles[port_idx][pin_idx], kHAL_GpioInterruptRisingEdge);
            port_idx = GPIO_PORT_IDX(APP_PIN_LSM6DSO_INT2);
            pin_idx  = GPIO_PIN_IDX(APP_PIN_LSM6DSO_INT2);
            HAL_GpioSetTriggerMode(gpioHandles[port_idx][pin_idx], kHAL_GpioInterruptRisingEdge);

            break;
        }
    }
    if (result != kStatus_Success)
    {
        PRINTF("\r\nLSM Sensor device initialize failed!\r\n");
    }
    else
    {
        lsmSensorReady = true;
        lsmSensorProc  = SRTM_Procedure_Create(APP_CheckLsmSensorInterrupt_Dispatcher, NULL, NULL);
        assert(lsmSensorProc);
        SRTM_Message_SetFreeFunc(lsmSensorProc, APP_CheckSensorInterrupt_RecycleMessage, (void *)&lsmSensorProc);
    }

    return result;
}

static status_t APP_SRTM_InitMaxSensorDevice(void)
{
    status_t result;
    uint8_t port_idx        = 0;
    uint8_t pin_idx         = 0;

    g_maxConfig.I2C_SendFunc    = BOARD_Accel_I2C_Send;
    g_maxConfig.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;
    g_maxConfig.mode            = MAX_CFG_MODE;
    g_maxConfig.pulsewidth      = MAX_CFG_PULSEWIDTH;
    g_maxConfig.samples         = MAX_CFG_SAMPLERATE;
    g_maxConfig.adcRange        = MAX_CFG_ADCRANGE;
    g_maxConfig.sampleavg       = MAX_CFG_SMP_AVE;

    result              = MAX_Init(&maxHandle, &g_maxConfig);
    if (result == kStatus_Success)
    {
        /* Setup gpio interrupt trigger type */
        port_idx = GPIO_PORT_IDX(APP_PIN_MAX30101_INT);
        pin_idx  = GPIO_PIN_IDX(APP_PIN_MAX30101_INT);
        HAL_GpioSetTriggerMode(gpioHandles[port_idx][pin_idx], kHAL_GpioInterruptFallingEdge);
    }

    if (result != kStatus_Success)
    {
        PRINTF("\r\nMAX Sensor device initialize failed!\r\n");

        return result;
    }
    else
    {
        maxSensorReady = true;
        maxSensorProc  = SRTM_Procedure_Create(APP_CheckMaxSensorInterrupt_Dispatcher, NULL, NULL);
        assert(maxSensorProc);
        SRTM_Message_SetFreeFunc(maxSensorProc, APP_CheckSensorInterrupt_RecycleMessage, (void *)&maxSensorProc);
    }

    result = MAX_Alg_Init(&g_maxConfig, &g_max_sensor_data_buf);
    if (result != kStatus_Success)
    {
        PRINTF("\r\nMAX Sensor Alg initialize failed!\r\n");
    }
    return result;
}

static void APP_SRTM_InitSensorService(void)
{
    if ((APP_SRTM_InitLsmSensorDevice() == kStatus_Success) && (APP_SRTM_InitMaxSensorDevice() == kStatus_Success))
    {
        sensorService = SRTM_SensorService_Create(&sensorAdapter);
        SRTM_Dispatcher_RegisterService(disp, sensorService);
    }
}

static void APP_SRTM_IO_TimerCallback(TimerHandle_t xTimer)
{
    uint8_t io_idx   = APP_IO_GetIoIndexByTimerHandle(xTimer);
    uint16_t io_id   = suspendContext.io.data[io_idx].io_id;
    uint8_t port_idx = GPIO_PORT_IDX(io_id);
    uint8_t pin_idx  = GPIO_PIN_IDX(io_id);
    srtm_keypad_value_t value;
    uint8_t pinState;

    HAL_GpioGetInput(gpioHandles[port_idx][pin_idx], (uint8_t *)&pinState);
    if (suspendContext.io.data[io_idx].io_state == IO_AS_BUTTON)
    {
        if (pinState == suspendContext.io.data[io_idx].value)
        {
            /* No glitch, a valid user operation */
            if (AD_CurrentMode == AD_ACT)
            {
                /* When A Core(CA35) is running, notify the event to A Core(CA35). */
                value = suspendContext.io.data[io_idx].value ? SRTM_KeypadValueReleased : SRTM_KeypadValuePressed;
                SRTM_KeypadService_NotifyKeypadEvent(keypadService, suspendContext.io.data[io_idx].keypad_idx, value);
            }
        }

        /* Restore pin detection interrupt */
        APP_IO_ConfInput(io_idx, suspendContext.io.data[io_idx].event, false);
    }
    else
    {
        if (pinState == suspendContext.io.data[io_idx].value)
        {
            if (AD_CurrentMode == AD_ACT)
            {
                SRTM_IoService_NotifyInputEvent(ioService, io_id);
            }
        }
    }
}

void APP_SRTM_IO_CreateTimer(uint16_t io_id)
{
    if (!suspendContext.io.data[APP_IO_GetIoIndexByIoId(io_id)].timer)
    {
        char io_timer_name[20] = {0};
        uint8_t io_idx         = APP_IO_GetIoIndexByIoId(io_id);
        TickType_t interval;
        if (suspendContext.io.data[io_idx].io_state == IO_AS_BUTTON)
            interval = APP_MS2TICK(50);
        else
            interval = APP_MS2TICK(2);

        snprintf(io_timer_name, sizeof(io_timer_name), "%s-0x%x", "io_tmr", io_id);
        suspendContext.io.data[APP_IO_GetIoIndexByIoId(io_id)].timer =
            xTimerCreate(io_timer_name, interval, pdFALSE, NULL, APP_SRTM_IO_TimerCallback);
        assert(suspendContext.io.data[APP_IO_GetIoIndexByIoId(io_id)].timer);
    }
}

static void APP_SRTM_InitIoKeyDevice(void)
{
    int32_t io_table_idx = -1;
    uint16_t io_id;
    uint32_t io_idx;
    hal_gpio_pin_config_t pinConfig = {
        kHAL_GpioDirectionIn, 0U, 0U, 0U, APP_GPIO_INT_SEL, NotAvail_IRQn,
    };

    /* setup gpios */
    for (io_idx = 0; io_idx < TOTAL_PINS; io_idx++)
    {
        pinConfig.port = io_idx / PIN_NUM_PER_GPIO_MODULE;
        pinConfig.pin  = io_idx % PIN_NUM_PER_GPIO_MODULE;
        io_id          = GPIO_IO_ID(pinConfig.port, pinConfig.pin);
        pinConfig.irq  = gpio_IRQn[pinConfig.port][pinConfig.sel];
        memset(&suspendContext.io.data[io_idx], 0U, sizeof(app_io_t));
        suspendContext.io.data[io_idx].io_id    = io_id;
        suspendContext.io.data[io_idx].irq      = pinConfig.irq;
        suspendContext.io.data[io_idx].io_state = IO_UNKOWN;
        if (true == APP_IO_WhetherMatchIoId(io_id, &io_table_idx) && gpioHandles[pinConfig.port][pinConfig.pin] != NULL)
        {
            switch (io_id_table[io_table_idx].io_state)
            {
                case IO_AS_INPUT:
                case IO_AS_BUTTON:
                    pinConfig.direction = kHAL_GpioDirectionIn;
                    HAL_GpioInit(gpioHandles[pinConfig.port][pinConfig.pin], &pinConfig);
                    if (!suspendContext.io.data[io_idx].overridden)
                    {
                        APP_IO_ConfInput(io_id, suspendContext.io.data[io_idx].event,
                                         suspendContext.io.data[io_idx].wakeup);
                    }
                    break;
                case IO_AS_OUTPUT:
                    pinConfig.direction = kHAL_GpioDirectionOut;
                    HAL_GpioInit(gpioHandles[pinConfig.port][pinConfig.pin], &pinConfig);
                    APP_IO_ConfOutput(io_id, SRTM_IoValueLow);
                    break;
            }

            suspendContext.io.data[io_idx].keypad_idx = io_id_table[io_table_idx].keypad_idx;
            suspendContext.io.data[io_idx].io_state   = io_id_table[io_table_idx].io_state;
            APP_SRTM_IO_CreateTimer(suspendContext.io.data[io_idx].io_id);
            HAL_GpioInstallCallback(gpioHandles[pinConfig.port][pinConfig.pin], APP_HandleGPIOHander,
                                    (void *)suspendContext.io.data[io_idx].io_id);
        }
    }
}

static void APP_SRTM_InitIoKeyService(void)
{
    int32_t idx;

    APP_SRTM_InitIoKeyDevice();

    ioService     = SRTM_IoService_Create();
    keypadService = SRTM_KeypadService_Create();

    for (idx = 0; idx < ARRAY_SIZE(io_id_table); idx++)
    {
        if (io_id_table[idx].io_state == IO_AS_BUTTON) /* use the gpio as button */
        {
            SRTM_KeypadService_RegisterKey(keypadService, io_id_table[idx].keypad_idx, APP_IO_ConfKEvent, NULL);
        }
        else
        {
            SRTM_IoService_RegisterPin(ioService, io_id_table[idx].io_id, APP_IO_SetOutput, APP_IO_GetInput,
                                       APP_IO_ConfIEvent, NULL);
        }
    }
    SRTM_Dispatcher_RegisterService(disp, ioService);
    SRTM_Dispatcher_RegisterService(disp, keypadService);
}

static void APP_SRTM_A35ResetHandler(void)
{
    portBASE_TYPE taskToWake = pdFALSE;

    /* disable interrupt */
    MU_DisableInterrupts(MU0_MUA, kMU_ResetAssertInterruptEnable);

    srtmState = APP_SRTM_StateReboot;

    /* Wake up monitor to reinitialize the SRTM communication with CA35 */
    if (pdPASS == xSemaphoreGiveFromISR(monSig, &taskToWake))
    {
        portYIELD_FROM_ISR(taskToWake);
    }
}

/* Make sure that XRDC has setup access permission for M Core(M Core can access registers of CMC_AD), unless M Core will
 * get hardfault(CMC_AD is belongs to Application Domain) */
static void CMC_ADClrAD_PSDORF(CMC_AD_Type *base, uint32_t flag)
{
    base->AD_PSDORF = flag; /* Write 1 to clear flag */
}

/*
 * MU Interrrupt RPMsg handler
 */
#ifdef MU0_A_IRQHandler
#undef MU0_A_IRQHandler
#endif

int32_t MU0_A_IRQHandler(void)
{
    uint32_t status = MU_GetStatusFlags(MU0_MUA);

    if (status & kMU_OtherSideEnterPowerDownInterruptFlag) /* PD/DPD mode */
    {
        SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Deactivated);

        PRINTF("AD entered PD(linux suspend to ram)/DPD(linux shutdown) mode\r\n");
        MU_ClearStatusFlags(MU0_MUA, (uint32_t)kMU_OtherSideEnterPowerDownInterruptFlag);

        if (AD_WillEnterMode == AD_DPD)
        {
            AD_CurrentMode = AD_WillEnterMode; /* AD entered Deep Power Down Mode */
            NVIC_ClearPendingIRQ(CMC1_IRQn);
            EnableIRQ(CMC1_IRQn);
            /* Help A35 to setup TRDC after A35 entered deep power down moade */
            BOARD_SetTrdcAfterApdReset();
            /*
             *  When RTD is the ower of LPAV and APD enter PD mode, RTD need poweroff LDO1, reduce BUCK3 to 0.73V,
             *  Set flag to put ddr into retention when RTD enter PD mode.
             *  otherwise APD side is responsible to control them in PD mode.
             */
            srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_SetLPAV, (void *)AD_DPD, NULL);

            assert(proc);
            SRTM_Dispatcher_PostProc(disp, proc);
        }
        else
        {
            /* Relase A Core */
            MU_BootOtherCore(
                MU0_MUA,
                (mu_core_boot_mode_t)0); /* Delete the code after linux supported sending suspend rpmsg to M Core */
            AD_CurrentMode = AD_PD;      /* AD entered Power Down Mode */
            /*
             *  When RTD is the ower of LPAV and APD enter PD mode, RTD need poweroff LDO1 and BUCK3
             *  otherwise APD side is responsible to control them in DPD mode
             */
            srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_SetLPAV, (void *)AD_PD, NULL);

            assert(proc);
            SRTM_Dispatcher_PostProc(disp, proc);
        }
        AD_WillEnterMode = AD_ACT;
    }

    return RPMsg_MU0_A_IRQHandler();
}

// clang-format off
/*
 * [AD_CurrentMode, AD_WillEnterMode] state machine:
 *       +----(uboot reset)---+
 *       |                    |
 *       |                    |
 *       v                    |
 * [AD_UNKOWN, AD_UNKOWN](A Core in uboot)<--+
 *    ^      |                               |
 *    |      |                        (linux reboot)
 *    |    (boot from uboot to linux)        |        +------------(linux resume from suspend)--+
 *    |      |                               |        |                                         |
 *    |      |                               |        v                                         |
 *    |      +--------------------->[AD_ACT, AD_UNKOWN] -----(linux suspend to mem)---->[AD_PD, AD_ACT]
 *    |                                        |
 *    |                                        |
 *    |                                  (linux poweroff)
 *    |                                        |
 *    |                                        v
 *    |                                [AD_DPD, AD_ACT]
 *    |                                        |
 *    |                                        |
 *    +---(A Core is woken by wakeup source)---+
 */   /* Note: When AD is [AD_DPD, AD_ACT],option V will not enter APP_SRTM_A35ResetHandler,execute reset logic,
       * but keep the process of boot A Core*/
// clang-format on
void CMC1_IRQHandler(void)
{
    apd_boot_cnt++;
    DisableIRQ(CMC1_IRQn);
    NVIC_ClearPendingIRQ(CMC1_IRQn);
    rtdCtxRestore();

    if ((AD_CurrentMode == AD_DPD && AD_WillEnterMode == AD_ACT && !option_v_boot_flag) ||
        (apd_boot_cnt > 1 && AD_CurrentMode == AD_UNKOWN && AD_WillEnterMode == AD_UNKOWN) ||
        (AD_CurrentMode == AD_ACT && AD_WillEnterMode == AD_UNKOWN))
    {
        APP_SRTM_A35ResetHandler();
    }
    if (AD_CurrentMode == AD_DPD && AD_WillEnterMode == AD_ACT)
    {
        AD_CurrentMode   = AD_UNKOWN;
        AD_WillEnterMode = AD_UNKOWN;
    }
    if (AD_CurrentMode == AD_PD && AD_WillEnterMode == AD_ACT)
    {
        PRINTF("\r\nAD resume from Power Down Mode\r\n");

        /* hold A core for next reboot */
        MU_HoldOtherCoreReset(MU0_MUA);
    }
}

static void APP_SRTM_InitRtcDevice(void)
{
    HAL_RtcInit(rtcHandle, 0);
    NVIC_ClearPendingIRQ(BBNSM_IRQn);
    NVIC_SetPriority(BBNSM_IRQn, APP_BBNSM_IRQ_PRIO);
    EnableIRQ(BBNSM_IRQn);
}

static void APP_SRTM_InitRtcService(void)
{
    APP_SRTM_InitRtcDevice();
    rtcAdapter = SRTM_RtcAdapter_Create(rtcHandle);
    assert(rtcAdapter);

    rtcService = SRTM_RtcService_Create(rtcAdapter);
    SRTM_Dispatcher_RegisterService(disp, rtcService);
}

static srtm_status_t APP_SRTM_LfclEventHandler(
    srtm_service_t service, srtm_peercore_t core, srtm_lfcl_event_t event, void *eventParam, void *userParam)
{
    switch (event)
    {
        case SRTM_Lfcl_Event_ShutdownReq: /* Notify M Core that Application Domain will enter Deep Power Down Mode */
            AD_WillEnterMode = AD_DPD;
            /* Relase A Core */
            MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
            PRINTF("\r\nAD will enter Deep Power Down Mode\r\n");
            break;
        case SRTM_Lfcl_Event_SuspendReq: /* Notify M Core that Application Domain will enter Power Down Mode */
            AD_WillEnterMode = AD_PD;
            /* Save context(such as: MU0_MUA[RCR]) */
            rtdCtxSave();
            /* Relase A Core */
            MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
            PRINTF("\r\nAD will enter Power Down Mode\r\n");
            break;
        case SRTM_Lfcl_Event_WakeupReq:
            /* If already deactivated, power on CA35, else CA35 will not power off,
               and wakeup will defer until CA35 enter Power Down */
            APP_SRTM_DoWakeupCA35(NULL, NULL, NULL);
            break;
        case SRTM_Lfcl_Event_Running: /* Notify M Core that Application Domain entered Active Mode */
            /* enable CMC1 IRQ */
            CMC_ADClrAD_PSDORF(
                CMC_AD, CMC_AD_AD_PSDORF_AD_PERIPH(
                            1)); /* need clear it, unless A Core cannot reboot after A Core suspend and resume back */
            NVIC_ClearPendingIRQ(CMC1_IRQn);
            EnableIRQ(CMC1_IRQn);
            rtdCtxRestore();
            AD_CurrentMode   = AD_ACT;
            AD_WillEnterMode = AD_UNKOWN;
            PRINTF("\r\nAD entered active mode\r\n");
            break;
        default:
            PRINTF("\r\n%s: %d unsupported event: 0x%x\r\n", __func__, __LINE__, event);
            break;
    }

    return SRTM_Status_Success;
}

static void APP_SRTM_InitLfclService(void)
{
    srtm_service_t service;

    /* Create and register Life Cycle service */
    service = SRTM_LfclService_Create();
    SRTM_LfclService_Subscribe(service, APP_SRTM_LfclEventHandler, NULL);
    SRTM_Dispatcher_RegisterService(disp, service);
}

static void APP_SRTM_InitServices(void)
{
    APP_SRTM_InitIoKeyService();
    APP_SRTM_InitI2CService();
    APP_SRTM_InitSensorService();
    APP_SRTM_InitAudioService();
    APP_SRTM_InitPwmService();
    APP_SRTM_InitRtcService();
    APP_SRTM_InitLfclService();
}

void APP_PowerOffCA35(void)
{
    UPOWER_PowerOffSwitches((upower_ps_mask_t)(kUPOWER_PS_A35_0 | kUPOWER_PS_A35_1 | kUPOWER_PS_L2_CACHE |
                                               kUPOWER_PS_AD_NIC | kUPOWER_PS_AD_PER));

    AD_CurrentMode = AD_DPD;
}

static void APP_PowerOnCA35(void)
{
    MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
    UPOWER_PowerOnSwitches((upower_ps_mask_t)(kUPOWER_PS_A35_0 | kUPOWER_PS_A35_1 | kUPOWER_PS_L2_CACHE |
                                              kUPOWER_PS_AD_NIC | kUPOWER_PS_AD_PER));
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void SRTM_MonitorTask(void *pvParameters)
{
    app_srtm_state_t state = APP_SRTM_StateShutdown;

    /* Initialize services and add to dispatcher */
    APP_SRTM_InitServices();

    /* Start SRTM dispatcher */
    SRTM_Dispatcher_Start(disp);

    /* Monitor peer core state change */
    while (true)
    {
        xSemaphoreTake(monSig, portMAX_DELAY);

        if (state == srtmState)
        {
            continue;
        }

        switch (srtmState)
        {
            case APP_SRTM_StateRun:
                assert(state == APP_SRTM_StateShutdown);
                PRINTF("Start SRTM communication\r\n");
                SRTM_Dispatcher_Stop(disp);
                /* Power On A Core when SoC is in low power boot type or option_v_boot_flag=true
                 * The purpose of using option_v_boot_flag is to avoid entering the APP_SRTM_A35ResetHandler
                 * in the CMC1_IRQHandler and interrupt the startup process during the process of starting Acore. */
                if (BOARD_IsLowPowerBootType() || option_v_boot_flag)
                {
                    DisableIRQ(CMC1_IRQn);
                    MU_Init(MU0_MUA);
                    if (option_v_boot_flag)
                    {
                        APP_WakeupACore();
                    }
                    else
                    {
                        APP_PowerOnCA35();
                    }
                }
                BOARD_PeripheralReset();

                /*
                 * Need handshake with uboot when SoC in all boot type(single boot type, dual boot type, low power boot
                 * type);
                 * M Core will boot A Core with APP_PowerOnCA35() API when SoC in low power boot type.
                 */
                if (BOARD_HandshakeWithUboot() == true)
                {
                    /* CMC1(CMC_AD) is belongs to Application Domain, so if want to access these registers of CMC1, pls
                     * make sure that mcore can access CMC1(mcore can access CMC1 after BOARD_HandshakeWithUboot) */
                    CMC_ADClrAD_PSDORF(
                        CMC_AD,
                        CMC_AD_AD_PSDORF_AD_PERIPH(
                            1)); /* need clear it, unless A Core cannot reboot after A Core suspend and resume back */
                }

                /* enable CMC1 interrupt after handshake with uboot(M Core cannot access CMC1 that belongs to
                 * Application Domain when Power On Reset; M Core can access CMC1 after uboot(running on A Core) enable
                 * accessing permission of CMC1 by XRDC) */
                EnableIRQ(CMC1_IRQn);

                APP_SRTM_InitPeerCore();
                SRTM_Dispatcher_Start(disp);

                NVIC_ClearPendingIRQ(CMC1_IRQn);
                EnableIRQ(CMC1_IRQn);
                option_v_boot_flag = false;
                state              = APP_SRTM_StateRun;
                break;

            case APP_SRTM_StateLinkedUp:
                if (state == APP_SRTM_StateRun)
                {
                    PRINTF("Handle Peer Core Linkup\r\n\r\n");
                    SRTM_Dispatcher_Stop(disp);
                    APP_SRTM_Linkup();
                    AD_CurrentMode   = AD_ACT;
                    AD_WillEnterMode = AD_UNKOWN;
                    SRTM_Dispatcher_Start(disp);
                }
                break;
            case APP_SRTM_StateShutdown:
                PRINTF("#### Shutdown CA35 ####\r\n");
                assert(state == APP_SRTM_StateRun);

                SRTM_Dispatcher_Stop(disp);
                /* Remove peer core from dispatcher */
                APP_SRTM_DeinitPeerCore();
                /* dispatcher can still handle proc message during peer core shutdown */
                SRTM_Dispatcher_Start(disp);

                /* Shutdown CA35 domain power */
                PRINTF("#### Power off CA35 ####\r\n");
                APP_PowerOffCA35();
                state = APP_SRTM_StateShutdown;
                break;
            case APP_SRTM_StateReboot:
                assert(state == APP_SRTM_StateRun);

                PRINTF("Handle Peer Core Reboot\r\n");

                SRTM_Dispatcher_Stop(disp);
                /* Remove peer core from dispatcher */
                APP_SRTM_DeinitPeerCore();

                BOARD_PeripheralReset();

                /* enable clock of MU0_MUA before accessing registers of MU0_MUA */
                MU_Init(MU0_MUA);

                /* Force peer core in reset */
                if (need_reset_peer_core)
                {
                    MU_HardwareResetOtherCore(MU0_MUA, true, true, kMU_CoreBootFromAddr0);
                    need_reset_peer_core = false;
                }

                /* Help A35 to setup TRDC before release A35 */
                BOARD_SetTrdcAfterApdReset();

                /* Release peer core from reset */
                MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);

                if (BOARD_HandshakeWithUboot() == true)
                {
                    /* CMC1(CMC_AD) is belongs to Application Domain, so if want to access these registers of CMC1, pls
                     * make sure that mcore can access CMC1(mcore can access CMC1 after BOARD_HandshakeWithUboot) */
                    CMC_ADClrAD_PSDORF(
                        CMC_AD,
                        CMC_AD_AD_PSDORF_AD_PERIPH(
                            1)); /* need clear it, unless A Core cannot reboot after A Core suspend and resume back */
                }
                /* Initialize peer core and add to dispatcher */
                APP_SRTM_InitPeerCore();

                /* Restore srtmState to Run. */
                srtmState = APP_SRTM_StateRun;

                SRTM_Dispatcher_Start(disp);

                NVIC_ClearPendingIRQ(CMC1_IRQn);
                EnableIRQ(CMC1_IRQn);

                /* hold A core for next reboot */
                MU_HoldOtherCoreReset(MU0_MUA);

                /* Do not need to change state. It's still Run. */
                break;

            default:
                assert(false);
                break;
        }
    }
}

void APP_ShutdownCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateShutdown, NULL);

    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

void APP_BootCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateRun, NULL);

    assert(proc);
    /* Fresh power up: Need SRTM monitor to prepare communication */
    SRTM_Dispatcher_PostProc(disp, proc);
}

static void SRTM_DispatcherTask(void *pvParameters)
{
    SRTM_Dispatcher_Run(disp);
}

static void APP_SRTM_InitPeriph(bool resume)
{
}

void APP_SRTM_Init(void)
{
    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_DMA0);

    APP_SRTM_InitPeriph(false);

    monSig = xSemaphoreCreateBinary();
    assert(monSig);

    /* Create a rtc alarm event timer to send rtc alarm event to A Core after A Core is waken by rtc alarm(avoid losting
     * rtc alarm event) */
    rtcAlarmEventTimer = xTimerCreate("rtcAlarmEventTimer", APP_MS2TICK(APP_RTC_ALM_EVT_TIMER_PERIOD_MS), pdFALSE, NULL,
                                      rtcAlarmEventTimer_Callback);
    assert(rtcAlarmEventTimer);

    /* Note: Create a task to refresh S400(sentinel) watchdog timer to keep S400 alive, the task will be removed after
     * the bug is fixed in soc A1 */
    SENTINEL_Init();
    refreshS400WdgTimer = xTimerCreate("refreshS400WdgTimer", APP_MS2TICK(APP_REFRESH_S400_WDG_TIMER_PERIOD_MS),
                                       pdFALSE, NULL, APP_RefreshS400WdgTimerCallback);
    assert(refreshS400WdgTimer);
    xTimerStart(refreshS400WdgTimer, portMAX_DELAY);

    restoreRegValOfMuTimer =
        xTimerCreate("restoreRegValOfMuTimer", APP_MS2TICK(100), pdFALSE, NULL, APP_RestoreRegValOfMuTimerCallback);
    assert(restoreRegValOfMuTimer);
    xTimerStart(restoreRegValOfMuTimer, portMAX_DELAY);

    linkupTimer =
        xTimerCreate("Linkup", APP_MS2TICK(APP_LINKUP_TIMER_PERIOD_MS), pdFALSE, NULL, APP_LinkupTimerCallback);
    assert(linkupTimer);

    /* Create SRTM dispatcher */
    disp = SRTM_Dispatcher_Create();

    NVIC_SetPriority(CMC1_IRQn, APP_CMC1_IRQ_PRIO);

    MU_Init(MU0_MUA);
    MU_EnableInterrupts(MU0_MUA, kMU_OtherSideEnterPowerDownInterruptEnable);
    rtdCtxSave(); /* try to save CIRE0 */

    /* hold A core for next reboot */
    MU_HoldOtherCoreReset(MU0_MUA);

    xTaskCreate(SRTM_MonitorTask, "SRTM monitor", 256U, NULL, APP_SRTM_MONITOR_TASK_PRIO, NULL);
    xTaskCreate(SRTM_DispatcherTask, "SRTM dispatcher", 512U, NULL, APP_SRTM_DISPATCHER_TASK_PRIO, NULL);
}

void APP_SRTM_StartCommunication(void)
{
    srtmState = APP_SRTM_StateRun;
    xSemaphoreGive(monSig);
}

void APP_SRTM_Suspend(void)
{
}

void APP_SRTM_Resume(bool resume)
{
    APP_SRTM_InitI2CDevice();
    APP_SRTM_InitAudioDevice();
    /*
     * IO has restored in APP_Resume(), so don't need init io again in here.
     * For RTD Power Down Mode(Wakeup through WUU), it's okay to initialize io again(use WUU to wakeup cortex-M33, the
     * WUU interrupt will not lost) For RTD Deep Sleep Mode(Wakeup through WIC), the GPIO interrupt will lost after
     * initialize io with APP_SRTM_InitIoKeyDevice().
     */
    /* APP_SRTM_InitIoKeyDevice(); */
    APP_SRTM_InitPwmDevice();
    HAL_RtcInit(rtcHandle, 0);
    if (resume)
    {
    }
}

void APP_SRTM_SetRpmsgMonitor(app_rpmsg_monitor_t monitor, void *param)
{
    rpmsgMonitor      = monitor;
    rpmsgMonitorParam = param;
}

static srtm_status_t APP_SRTM_I2C_SwitchChannel(srtm_i2c_adapter_t adapter,
                                                uint32_t base_addr,
                                                srtm_i2c_type_t type,
                                                uint16_t slaveAddr,
                                                srtm_i2c_switch_channel channel)
{
    uint8_t txBuff[1];
    assert(channel < SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED);
    txBuff[0] = 1 << (uint8_t)channel;
    return adapter->write(adapter, base_addr, type, slaveAddr, txBuff, sizeof(txBuff),
                          SRTM_I2C_FLAG_NEED_STOP); // APP_SRTM_I2C_Write
}

static srtm_status_t APP_SRTM_I2C_Write(srtm_i2c_adapter_t adapter,
                                        uint32_t base_addr,
                                        srtm_i2c_type_t type,
                                        uint16_t slaveAddr,
                                        uint8_t *buf,
                                        uint16_t len,
                                        uint16_t flags)
{
    status_t retVal   = kStatus_Fail;
    uint32_t needStop = (flags & SRTM_I2C_FLAG_NEED_STOP) ? kLPI2C_TransferDefaultFlag : kLPI2C_TransferNoStopFlag;

    switch (type)
    {
        case SRTM_I2C_TYPE_LPI2C:
            retVal = BOARD_LPI2C_Send((LPI2C_Type *)base_addr, slaveAddr, 0, 0, buf, len, needStop);
            break;
        default:
            break;
    }
    return (retVal == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}

static srtm_status_t APP_SRTM_I2C_Read(srtm_i2c_adapter_t adapter,
                                       uint32_t base_addr,
                                       srtm_i2c_type_t type,
                                       uint16_t slaveAddr,
                                       uint8_t *buf,
                                       uint16_t len,
                                       uint16_t flags)
{
    status_t retVal   = kStatus_Fail;
    uint32_t needStop = (flags & SRTM_I2C_FLAG_NEED_STOP) ? kLPI2C_TransferDefaultFlag : kLPI2C_TransferNoStopFlag;

    switch (type)
    {
        case SRTM_I2C_TYPE_LPI2C:
            retVal = BOARD_LPI2C_Receive((LPI2C_Type *)base_addr, slaveAddr, 0, 0, buf, len, needStop);
            break;
        default:
            break;
    }
    return (retVal == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}

uint8_t APP_Read_I2C_Register(uint8_t busID, uint16_t slaveAddr, uint8_t regIndex)
{
    uint8_t value;
    SRTM_I2C_RequestBusWrite(i2cService, busID, slaveAddr, &regIndex, 1, 0);
    SRTM_I2C_RequestBusRead(i2cService, busID, slaveAddr, &value, 1);
    return value;
}

uint8_t APP_Write_I2C_Register(uint8_t busID, uint16_t slaveAddr, uint8_t regIndex, uint8_t value)
{
    uint8_t write_content[2];
    write_content[0] = regIndex;
    write_content[1] = value;
    SRTM_I2C_RequestBusWrite(i2cService, busID, slaveAddr, write_content, 2, 1);
    return value;
}

void APP_SRTM_HandlePeerReboot(void)
{
    if (srtmState != APP_SRTM_StateShutdown)
    {
        srtmState = APP_SRTM_StateReboot;
        xSemaphoreGive(monSig);
    }
}

void APP_SRTM_SetIRQHandler(app_irq_handler_t handler, void *param)
{
    irqHandler      = handler;
    irqHandlerParam = param;
}

void APP_SRTM_WakeupCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_DoWakeupCA35, NULL, NULL);

    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

static void APP_SRTM_DoSetWakeupModule(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    uint32_t module                          = (uint32_t)param1;
    wuu_internal_wakeup_module_event_t event = (wuu_internal_wakeup_module_event_t)(uint32_t)param2;

    WUU_SetInternalWakeUpModulesConfig(WUU0, module, event);
}

void APP_SRTM_SetWakeupModule(uint32_t module, bool enable)
{
    srtm_procedure_t proc;

    proc = SRTM_Procedure_Create(APP_SRTM_DoSetWakeupModule, (void *)module, (void *)(uint32_t)enable);
    assert(proc);

    SRTM_Dispatcher_CallProc(disp, proc, SRTM_WAIT_FOR_EVER);
    SRTM_Procedure_Destroy(proc);
}

static void APP_SRTM_DoSetWakeupPin(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    uint16_t io_id  = (uint32_t)param1;
    uint16_t event  = (uint32_t)param2;
    uint8_t io_idx  = APP_IO_GetIoIndexByIoId(io_id);
    bool wakeup     = (bool)(event >> 8);
    uint8_t pinMode = (uint8_t)event;

    assert(pinMode < ARRAY_SIZE(wuuPinModeEvents));
    if (io_idx < ARRAY_SIZE(suspendContext.io.data))
    {
        if (wuuPinModeEvents[pinMode] != SRTM_IoEventNone)
        {
            APP_IO_ConfInput(io_idx, wuuPinModeEvents[pinMode], wakeup);
        }
        else
        {
            /* Restore CA35 settings */
            APP_IO_ConfInput(io_idx, suspendContext.io.data[io_idx].event, suspendContext.io.data[io_idx].wakeup);
        }
    }
}

void APP_SRTM_SetWakeupPin(uint16_t io_id, uint16_t event)
{
    srtm_procedure_t proc;

    proc = SRTM_Procedure_Create(APP_SRTM_DoSetWakeupPin, (void *)(uint32_t)io_id, (void *)(uint32_t)event);
    assert(proc);

    SRTM_Dispatcher_CallProc(disp, proc, SRTM_WAIT_FOR_EVER);
    SRTM_Procedure_Destroy(proc);
}

/* Show Podemeter values */
void APP_ShowPedometer(void)
{
    uint16_t stepCnt = 0;
    lsm_emb_func_status_t val;

    PRINTF("%s\r\n", __func__);
    LSM_GetVal(&lsmHandle, LSM_EMB_FUNC_STATUS_REG, (uint8_t *)&val, LSM_EMBEDDED_FUNC_BANK, LSM_USER_BANK);
    PRINTF("LSM_EMB_FUNC_STATUS_REG: 0x%02x, is_tilt: %d, is_step_det: %d\r\n", val, val.is_tilt, val.is_step_det);

    if (kStatus_Success == LSM_GetPedometerCnt(&lsmHandle, &stepCnt))
    {
        PRINTF("Current Pedometer Count: %d\r\n", stepCnt);
    }
    else
    {
        PRINTF("Error reading Pedometer Count\r\n");
    }
}

void APP_ShowTemperature(void)
{
    int32_t temp_val = 0;

    PRINTF("%s\r\n", __func__);

    if (kStatus_Success == MAX_ReadTemp(&maxHandle, &temp_val))
    {
        PRINTF("Current Temperature: %d\r\n", temp_val);
    }
    else
    {
        PRINTF("Error reading Temperature\r\n");
    }
}

void APP_ShowHeartRate(void)
{
    uint32_t heartrate_val = 0;

    PRINTF("%s\r\n", __func__);

    if (kStatus_Success == MAX_GetHeartRate(&maxHandle, &heartrate_val))
    {
        PRINTF("Current HeartRate: %d\r\n", heartrate_val);
    }
    else
    {
        PRINTF("Error reading HeartRate\r\n");
    }
}

void APP_ShowSpO2(void)
{
    uint32_t spo2_val = 0;

    PRINTF("%s\r\n", __func__);

    if (kStatus_Success == MAX_GetSpO2(&maxHandle, &spo2_val))
    {
        PRINTF("Current SpO2: %d\r\n", spo2_val);
    }
    else
    {
        PRINTF("Error reading SpO2\r\n");
    }
}

void APP_DumpMAX30101Regs(void)
{
    MAX_DumpAllRegs(&maxHandle);
}
