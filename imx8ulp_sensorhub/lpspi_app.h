/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* LPSPI application header */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_reset.h"
#include "fsl_lpspi.h"

#include "gui_paint.h"
#include "fonts.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//Change from LPSPI1 to LPSPI0 due to A1 rework difficulty
#define EXAMPLE_LPSPI_MASTER_BASEADDR         LPSPI0
#define EXAMPLE_LPSPI_MASTER_CLOCK_NAME       kCLOCK_Lpspi0
#define LPSPI_MASTER_CLK_FREQ                 (CLOCK_GetIpFreq(EXAMPLE_LPSPI_MASTER_CLOCK_NAME))
#define EXAMPLE_LPSPI_MASTER_CLOCK_SOURCE     (kCLOCK_Pcc0PlatIpSrcSysOscDiv1)
#define EXAMPLE_LPSPI_MASTER_PCS_FOR_INIT     kLPSPI_Pcs0
#define EXAMPLE_LPSPI_MASTER_PCS_FOR_TRANSFER kLPSPI_MasterPcs0

#define TRANSFER_SIZE     1U      /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 30000000U /*! Transfer baudrate - 30m */

#define LCD_1IN28_DC_1          RGPIO_PinWrite(BOARD_DC_RGPIO, BOARD_DC_RGPIO_PIN, 1)
#define LCD_1IN28_CS_1          RGPIO_PinWrite(BOARD_CS_RGPIO, BOARD_CS_RGPIO_PIN, 1)
#define LCD_1IN28_RST_1         RGPIO_PinWrite(BOARD_RST_RGPIO, BOARD_RST_RGPIO_PIN, 1)
#define LCD_1IN28_DC_0          RGPIO_PinWrite(BOARD_DC_RGPIO, BOARD_DC_RGPIO_PIN, 0)
#define LCD_1IN28_CS_0          RGPIO_PinWrite(BOARD_CS_RGPIO, BOARD_CS_RGPIO_PIN, 0)
#define LCD_1IN28_RST_0         RGPIO_PinWrite(BOARD_RST_RGPIO, BOARD_RST_RGPIO_PIN, 0)

#define UBYTE                   uint8_t
#define UWORD                   uint16_t
#define UDOUBLE                 uint32_t

#define LCD_1IN28_HEIGHT        240
#define LCD_1IN28_WIDTH         240
#define WRITE_BLOCK_NUM         4

/*******************************************************************************
 * API
 ******************************************************************************/
void LPSPI_Init(void);

void DEV_SPI_WRITE(uint8_t val);

void DEV_SPI_BLOCK_WRITE(uint8_t *val, uint32_t size);

void LCD_1IN28_Reset(void);

void LCD_1IN28_SendCommand(UBYTE Reg);

void LCD_1IN28_SendData_8Bit(UBYTE Data);

void LCD_1IN28_SendData_16Bit(UWORD Data);

void LCD_1IN28_InitReg(void);

void LCD_1IN28_SetAttributes();

void LCD_1IN28_Init();

void LCD_1IN28_SetWindows(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend);

void LCD_1IN28_DrawPaint(UWORD x, UWORD y, UWORD Color);

void LCD_1IN28_Update_Block(UWORD Xstart, UWORD Ystart, const char * pString, sFONT* Font,
                            UWORD Color_Background, UWORD Color_Foreground, UBYTE Max_Char);

void LCD_1IN28_Clear(UWORD Color);

void LCD_1IN28_Clear_Block(UWORD Color);

void LCD_1in28_test();