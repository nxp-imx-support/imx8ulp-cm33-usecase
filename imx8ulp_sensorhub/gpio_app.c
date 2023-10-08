/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* GPIO application source */
#include "gpio_app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Define the init structure for the 3 output pins*/
rgpio_pin_config_t dc_config = {
    kRGPIO_DigitalOutput,
    0,
};
rgpio_pin_config_t cs_config = {
    kRGPIO_DigitalOutput,
    0,
};
rgpio_pin_config_t rst_config = {
    kRGPIO_DigitalOutput,
    0,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
void ModuleEnable(void)
{
    RGPIO_PinWrite(BOARD_DC_RGPIO, BOARD_DC_RGPIO_PIN, 1);
    RGPIO_PinWrite(BOARD_CS_RGPIO, BOARD_CS_RGPIO_PIN, 1);
    RGPIO_PinWrite(BOARD_RST_RGPIO, BOARD_RST_RGPIO_PIN, 1);
}

void ModuleDisable(void)
{
    RGPIO_PinWrite(BOARD_DC_RGPIO, BOARD_DC_RGPIO_PIN, 0);
    RGPIO_PinWrite(BOARD_CS_RGPIO, BOARD_CS_RGPIO_PIN, 0);
    RGPIO_PinWrite(BOARD_RST_RGPIO, BOARD_RST_RGPIO_PIN, 0);
}

void ModuleInit(void)
{
    /* Init 3 output GPIOs. */
    RGPIO_PinInit(BOARD_DC_RGPIO, BOARD_DC_RGPIO_PIN, &dc_config);
    RGPIO_PinInit(BOARD_CS_RGPIO, BOARD_CS_RGPIO_PIN, &cs_config);
    RGPIO_PinInit(BOARD_RST_RGPIO, BOARD_RST_RGPIO_PIN, &rst_config);

    ModuleEnable();
}