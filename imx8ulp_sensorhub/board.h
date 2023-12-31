/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_rgpio.h"
#if defined(BOARD_USE_PCA6416A) && BOARD_USE_PCA6416A
#include "fsl_pca6416a.h"
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME        "MIMX8ULP-EVK"
#define MANUFACTURER_NAME "NXP"

/* SOC infomation*/
#define IMX8ULP_SOC_REV_A0 0xA000
#define IMX8ULP_SOC_REV_A1 0xA100
#define IMX8ULP_SOC_ID     0x084D

/* The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE     kSerialPort_Uart
#define BOARD_DEBUG_UART_BAUDRATE 115200u
#define BOARD_DEBUG_UART_BASEADDR LPUART1_BASE
#define BOARD_DEBUG_UART_INSTANCE 1U
#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetLpuartClkFreq(BOARD_DEBUG_UART_INSTANCE)
#define BOARD_DEBUG_UART_IP_NAME  kCLOCK_Lpuart1
#define BOARD_DEBUG_UART_CLKSRC   kCLOCK_Pcc1BusIpSrcCm33Bus
#define BOARD_DEBUG_UART_RESET    kRESET_Lpuart1
#define BOARD_UART_IRQ            LPUART1_IRQn
#define BOARD_UART_IRQ_HANDLER    LPUART1_IRQHandler

/* Board accelerator sensor configuration */
#define BOARD_ACCEL_I2C_BASEADDR   LPI2C0
#define BOARD_ACCEL_I2C_CLOCK_FREQ CLOCK_GetLpi2cClkFreq(0)

#define BOARD_CODEC_I2C_BASEADDR   LPI2C0
#define BOARD_CODEC_I2C_CLOCK_FREQ CLOCK_GetLpi2cClkFreq(0)
#define BOARD_CODEC_I2C_INSTANCE   0U

/* Board mipi to hdmi bridge ic(IT6161) */
#define BOARD_DISPLAY_I2C_BASEADDR   LPI2C0
#define BOARD_DISPLAY_I2C_CLOCK_FREQ CLOCK_GetLpi2cClkFreq(0)

#define BOARD_FLEXSPI_PSRAM FLEXSPI1

#define BOARD_SW8_GPIO        GPIOB
#define BOARD_SW8_GPIO_PIN    12U
#define BOARD_SW8_IRQ         GPIOB_INT0_IRQn
#define BOARD_SW8_IRQ_HANDLER GPIOB_INT0_IRQHandler
#define BOARD_SW8_NAME        "SW8"

#define BOARD_SW7_GPIO        GPIOB
#define BOARD_SW7_GPIO_PIN    13U
#define BOARD_SW7_IRQ         GPIOB_INT0_IRQn
#define BOARD_SW7_IRQ_HANDLER GPIOB_INT0_IRQHandler
#define BOARD_SW7_NAME        "SW7"

#define LED_INIT()
#define LED_ON()
#define LED_TOGGLE()

#define VDEV0_VRING_BASE (0xAFF00000U)
#define VDEV1_VRING_BASE (0xAFF10000U)

/* MIPI panel. */
/* RST pin. */
#define BOARD_MIPI_RST_GPIO GPIOC
#define BOARD_MIPI_RST_PIN  23
/* Backlight pin. */
#define BOARD_MIPI_BL_GPIO GPIOA
#define BOARD_MIPI_BL_PIN  3
/* TE pin. */
#define BOARD_MIPI_TE_GPIO GPIOE
#define BOARD_MIPI_TE_PIN  17

#define BOARD_IS_XIP_FLEXSPI0()                                                                                 \
    ((((uint32_t)BOARD_InitDebugConsole >= 0x04000000U) && ((uint32_t)BOARD_InitDebugConsole < 0x0C000000U)) || \
     (((uint32_t)BOARD_InitDebugConsole >= 0x14000000U) && ((uint32_t)BOARD_InitDebugConsole < 0x1C000000U)))

/* PCA6416A (U27) */
#define BOARD_PCA6416A_I2C            LPI2C0
#define BOARD_PCA6416A_I2C_ADDR       (0x20U)
#define BOARD_PCA6416A_I2C_CLOCK_FREQ (CLOCK_GetLpi2cClkFreq(0))

#define BOARD_PCA6416A_CPU_POWER_MODE0 0U
#define BOARD_PCA6416A_CPU_POWER_MODE1 1U
#define BOARD_PCA6416A_CPU_POWER_MODE2 2U
#define BOARD_PCA6416A_CAN0_STDBY      4U
#define BOARD_PCA6416A_TOUCH_RESET     5U
#define BOARD_PCA6416A_CAMERA_RESET    7U
#define BOARD_PCA6416A_CAMERA_PWDN     (8U + 0U)
#define BOARD_PCA6416A_MIPI_SWITCH     (8U + 1U)
#define BOARD_PCA6416A_EPDC_SWITCH     (8U + 2U)
#define BOARD_PCA6416A_BATT_ADC_ENABLE (8U + 3U)
#define BOARD_PCA6416A_CHG_OK          (8U + 4U)
#define BOARD_PCA6416A_AC_OK           (8U + 5U)

#define TPM0_CH2 (2UL)
/* 500 Hz */
#define TPM0_CH2_PWM_FREQ (500UL)
#define FULL_DUTY_CYCLE   (100UL)

/* IT6161(U10) */
#define BOARD_IT6161_I2C            LPI2C0
#define BOARD_IT6161_I2C_ADDR       (0x6CU)
#define BOARD_IT6161_I2C_CLOCK_FREQ (CLOCK_GetLpi2cClkFreq(0))

#define TRDC_MBC_ACCESS_CONTROL_POLICY_ALL_INDEX (0)
#define TRDC_MRC_ACCESS_CONTROL_POLICY_ALL_INDEX (0)
#define TRDC_M33_DOMAIN_ID                       (6)
#define TRDC_POWERQUAD_DOMAIN_ID                 (0)
#define TRDC_POWERQUAD_MASTER_ID                 (4)
#define TRDC_DMA0_DOMAIN_ID                      (0)
#define TRDC_DMA0_MASTER_ID                      (2)
#define TRDC_MDAC2_INDEX                         (2) /* T-MADC2 */
#define TRDC_MDAC0_INDEX                         (0) /* T-MADC0 */
#define TRDC_MRC0_INDEX                          (0) /* T-MRC0 */
#define TRDC_MRC1_INDEX                          (1) /* T-MRC1 */

#define TRDC_MBC3_INDEX    (3) /* T-MBC3(controll access of DSP Domain) */
#define TRDC_MBC_INDEX_NUM (4)

/* FSB */
#define FUSE_BANKS           (64)
#define FUSE_WORDS_PER_BANKS (8)
#define FSB_OTP_SHADOW       (0x800)
#define FSB_BASE_ADDR        (0x27010000)

/* handshake with uboot */
/* Define the timeout ms to do handshake with uboot with mu flag */
#define BOARD_HANDSHAKE_WITH_UBOOT_TIMEOUT_MS (10000U)

/*
 * Get a proper sequence to recovery RCR of MU0_MUA from uboot(a35)
 * at the same time, imply that io and pwm of mipi is ready for uboot
 */
#define BOARD_MU0_MUB_F0_INIT_SRTM_COMMUNICATION_FLG (0x1UL)
/* 1 ms */
#define BOARD_WAIT_MU0_MUB_F0_FLG_FROM_UBOOT_MS (0x1U)

#define CTL_NUM      680
#define PI_NUM       298
#define PHY_NUM      1654
#define PHY_DIFF_NUM 49

struct dram_cfg
{
    uint32_t ctl_cfg[CTL_NUM];
    uint32_t pi_cfg[PI_NUM];
    uint32_t phy_full[PHY_NUM];
    uint32_t phy_diff[PHY_DIFF_NUM];
};

/*
 * m33 core frequency
 * OD: OverDrive
 * NM: Norminal
 * UD: UnderDrive
 * RBB: Reverse Body Bias
 */
#define M33_CORE_MAX_FREQ_OD_1_1V     (216 * 1000 * 1000)
#define M33_CORE_MAX_FREQ_NM_1_0V     (172 * 1000 * 1000)
#define M33_CORE_MAX_FREQ_UD_0_9V     (80 * 1000 * 1000)
#define M33_CORE_MAX_FREQ_UD_RBB_0_9V (40 * 1000 * 1000)

/* boot type */
enum BOOT_TYPE
{
    SINGLE_BOOT_TYPE,
    DUAL_BOOT_TYPE,
    LOW_POWER_BOOT_TYPE,
};

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

void BOARD_InitDebugConsole(void);
bool BOARD_IsLowPowerBootType(void);
bool BOARD_IsSingleBootType(void);
const char *BOARD_GetBootTypeName(void);

/* TRDC */
bool BOARD_GetReleaseFlagOfTrdc(void);
void BOARD_SetReleaseFlagOfTrdc(bool flag);
void BOARD_ReleaseTRDC(void);
void BOARD_SetTrdcGlobalConfig(void);
/* Setup TRDC configuration before executing rom code of A35(A35 rom will access FSB, S400 MUAP A-Side, SIM0-S with
 * secure state, so m33 help a35 to configure TRDC) */
void BOARD_SetTrdcAfterApdReset(void);

/*
 * return the handshake result(fail or success):
 * true: succeeded to handshake with uboot; false: failed to handshake with uboot
 */
bool BOARD_HandshakeWithUboot(void);

void BOARD_ConfigMPU(void);
status_t BOARD_InitPsRam(void);
void BOARD_FlexspiClockSafeConfig(void);
AT_QUICKACCESS_SECTION_CODE(
    void BOARD_SetFlexspiClock(FLEXSPI_Type *base, uint32_t src, uint8_t divValue, uint8_t fracValue));
AT_QUICKACCESS_SECTION_CODE(void BOARD_DeinitXip(FLEXSPI_Type *base));
AT_QUICKACCESS_SECTION_CODE(void BOARD_InitXip(FLEXSPI_Type *base));
#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
void BOARD_LPI2C_Init(LPI2C_Type *base, uint32_t clkSrc_Hz);
status_t BOARD_LPI2C_Send(LPI2C_Type *base,
                          uint8_t deviceAddress,
                          uint32_t subAddress,
                          uint8_t subaddressSize,
                          uint8_t *txBuff,
                          uint16_t txBuffSize,
                          uint32_t flags);
status_t BOARD_LPI2C_Receive(LPI2C_Type *base,
                             uint8_t deviceAddress,
                             uint32_t subAddress,
                             uint8_t subaddressSize,
                             uint8_t *rxBuff,
                             uint16_t rxBuffSize,
                             uint32_t flags);
void BOARD_Accel_I2C_Init(void);
status_t BOARD_Accel_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint32_t txBuff, uint32_t flags);
status_t BOARD_Accel_I2C_Receive(uint8_t deviceAddress,
                                 uint32_t subAddress,
                                 uint8_t subaddressSize,
                                 uint8_t *rxBuff,
                                 uint8_t rxBuffSize,
                                 uint32_t flags);
void BOARD_Codec_I2C_Init(void);
status_t BOARD_Codec_I2C_Send(uint8_t deviceAddress,
                              uint32_t subAddress,
                              uint8_t subAddressSize,
                              const uint8_t *txBuff,
                              uint8_t txBuffSize,
                              uint32_t flags);
status_t BOARD_Codec_I2C_Receive(uint8_t deviceAddress,
                                 uint32_t subAddress,
                                 uint8_t subAddressSize,
                                 uint8_t *rxBuff,
                                 uint8_t rxBuffSize,
                                 uint32_t flags);

void BOARD_Display_I2C_Init(void);
status_t BOARD_Display_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize);
status_t BOARD_Display_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize);

#if defined(BOARD_USE_PCA6416A) && BOARD_USE_PCA6416A
void BOARD_PCA6416A_I2C_Init(void);
status_t BOARD_PCA6416A_I2C_Send(uint8_t deviceAddress,
                                 uint32_t subAddress,
                                 uint8_t subAddressSize,
                                 const uint8_t *txBuff,
                                 uint8_t txBuffSize,
                                 uint32_t flags);
status_t BOARD_PCA6416A_I2C_Receive(uint8_t deviceAddress,
                                    uint32_t subAddress,
                                    uint8_t subAddressSize,
                                    uint8_t *rxBuff,
                                    uint8_t rxBuffSize,
                                    uint32_t flags);

/* PCA6416A U27. */
extern pca6416a_handle_t g_pca6416aHandle;

void BOARD_InitPCA6416A(pca6416a_handle_t *handle);

/* MIPI DSI */
void BOARD_InitMipiDsiPins(void);
void BOARD_EnableMipiDsiBacklight(void);
#endif /* BOARD_USE_PCA6416A */

#endif /* SDK_I2C_BASED_COMPONENT_USED */

/* LPAV DDR */
void BOARD_LpavInit();
void BOARD_DramEnterRetention(void);
void BOARD_DramExitRetention(uint32_t dram_class, struct dram_cfg *dram_timing_cfg);

int32_t BOARD_UpdateM33CoreFreq(cgc_rtd_sys_clk_config_t config);
#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
