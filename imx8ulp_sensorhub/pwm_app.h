/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* PWM application header */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_reset.h"
#include "fsl_tpm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* define instance */
#define BOARD_TPM_BASEADDR              TPM0
#define BOARD_TPM_CHANNEL               kTPM_Chnl_3

/* Interrupt to enable and flag to read; depends on the TPM channel used */
#define TPM_CHANNEL_INTERRUPT_ENABLE    kTPM_Chnl3InterruptEnable
#define TPM_CHANNEL_FLAG                kTPM_Chnl3Flag

/* Interrupt number and interrupt handler for the TPM instance used */
#define TPM_INTERRUPT_NUMBER            TPM0_IRQn
#define TPM_LED_HANDLER                 TPM0_IRQHandler

/* Get source clock for TPM driver */
#define TPM_SOURCE_CLOCK                (CLOCK_GetTpmClkFreq(0))
#ifndef TPM_LED_ON_LEVEL
#define TPM_LED_ON_LEVEL                kTPM_HighTrue
#endif
#ifndef DEMO_PWM_FREQUENCY
#define DEMO_PWM_FREQUENCY              (24000U)
#endif

#define SETUP_DUTY_CYCLE                10
#define RUN_DUTY_CYCLE                  80
#define DEEPSLEEP_DUTY_CYCLE            00
/*******************************************************************************
 * API
 ******************************************************************************/
int Init_Backlight(void);
int Set_Backlight(void);
int Set_Backlight_Off(void);