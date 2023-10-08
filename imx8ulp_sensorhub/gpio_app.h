/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* GPIO application header */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_rgpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_DC_RGPIO          GPIOA
#define BOARD_DC_RGPIO_PIN      6U
#define BOARD_CS_RGPIO          GPIOA
#define BOARD_CS_RGPIO_PIN      15U
#define BOARD_RST_RGPIO         GPIOA
#define BOARD_RST_RGPIO_PIN     24U

/*******************************************************************************
 * API
 ******************************************************************************/
void ModuleEnable(void);

void ModuleDisable(void);

void ModuleInit(void);