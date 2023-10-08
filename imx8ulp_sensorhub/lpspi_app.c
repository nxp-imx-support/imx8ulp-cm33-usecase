/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* LPSPI application shource */
#include "gpio_app.h"
#include "lpspi_app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
lpspi_transfer_t masterXfer;
uint8_t *arr;

/*******************************************************************************
 * Code
 ******************************************************************************/
void LPSPI_Init(void)
{
    uint32_t srcClock_Hz;
    lpspi_master_config_t masterConfig;

    /*Master config*/
    LPSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate = TRANSFER_BAUDRATE;
    masterConfig.whichPcs = EXAMPLE_LPSPI_MASTER_PCS_FOR_INIT;

    srcClock_Hz = LPSPI_MASTER_CLK_FREQ;
    //PRINTF("lpspi src clock freq is: %d \r\n", srcClock_Hz);
    LPSPI_MasterInit(EXAMPLE_LPSPI_MASTER_BASEADDR, &masterConfig, srcClock_Hz);
}

void DEV_SPI_WRITE(uint8_t val)
{
    uint8_t array_val[1] = {val};
    /* Start master transfer, send data to slave */
    masterXfer.txData   = array_val;
    masterXfer.rxData   = NULL;
    masterXfer.dataSize = TRANSFER_SIZE;
    masterXfer.configFlags =
        EXAMPLE_LPSPI_MASTER_PCS_FOR_TRANSFER | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;

    LPSPI_MasterTransferBlocking(EXAMPLE_LPSPI_MASTER_BASEADDR, &masterXfer);
}

void DEV_SPI_BLOCK_WRITE(uint8_t *val, uint32_t size)
{
    /* Start master transfer, send data to slave */
    masterXfer.txData   = val;
    masterXfer.rxData   = NULL;
    masterXfer.dataSize = size;
    masterXfer.configFlags =
        EXAMPLE_LPSPI_MASTER_PCS_FOR_TRANSFER | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;

    LPSPI_MasterTransferBlocking(EXAMPLE_LPSPI_MASTER_BASEADDR, &masterXfer);
}

void LCD_1IN28_Reset(void)
{
    LCD_1IN28_RST_1;
//    DEV_Delay_ms(100);
    SDK_DelayAtLeastUs(100000, SystemCoreClock);
    LCD_1IN28_RST_0;
//    DEV_Delay_ms(100);
    SDK_DelayAtLeastUs(100000, SystemCoreClock);
    LCD_1IN28_RST_1;
//    DEV_Delay_ms(100);
    SDK_DelayAtLeastUs(100000, SystemCoreClock);
}

void LCD_1IN28_SendCommand(UBYTE Reg)
{
    LCD_1IN28_DC_0;
    LCD_1IN28_CS_0;
    DEV_SPI_WRITE(Reg);
}

void LCD_1IN28_SendData_8Bit(UBYTE Data)
{
    LCD_1IN28_DC_1;
    LCD_1IN28_CS_0;
    DEV_SPI_WRITE(Data);
    LCD_1IN28_CS_1;
}

void LCD_1IN28_SendData_16Bit(UWORD Data)
{
    LCD_1IN28_DC_1;
    LCD_1IN28_CS_0;
    DEV_SPI_WRITE(Data >> 8);
    DEV_SPI_WRITE(Data);
    LCD_1IN28_CS_1;
}

void LCD_1IN28_InitReg(void)
{
    LCD_1IN28_SendCommand(0xEF);
    LCD_1IN28_SendCommand(0xEB);
    LCD_1IN28_SendData_8Bit(0x14);

    LCD_1IN28_SendCommand(0xFE);
    LCD_1IN28_SendCommand(0xEF);

    LCD_1IN28_SendCommand(0xEB);
    LCD_1IN28_SendData_8Bit(0x14);

    LCD_1IN28_SendCommand(0x84);
    LCD_1IN28_SendData_8Bit(0x40);

    LCD_1IN28_SendCommand(0x85);
    LCD_1IN28_SendData_8Bit(0xFF);

    LCD_1IN28_SendCommand(0x86);
    LCD_1IN28_SendData_8Bit(0xFF);

    LCD_1IN28_SendCommand(0x87);
    LCD_1IN28_SendData_8Bit(0xFF);

    LCD_1IN28_SendCommand(0x88);
    LCD_1IN28_SendData_8Bit(0x0A);

    LCD_1IN28_SendCommand(0x89);
    LCD_1IN28_SendData_8Bit(0x21);

    LCD_1IN28_SendCommand(0x8A);
    LCD_1IN28_SendData_8Bit(0x00);

    LCD_1IN28_SendCommand(0x8B);
    LCD_1IN28_SendData_8Bit(0x80);

    LCD_1IN28_SendCommand(0x8C);
    LCD_1IN28_SendData_8Bit(0x01);

    LCD_1IN28_SendCommand(0x8D);
    LCD_1IN28_SendData_8Bit(0x01);

    LCD_1IN28_SendCommand(0x8E);
    LCD_1IN28_SendData_8Bit(0xFF);

    LCD_1IN28_SendCommand(0x8F);
    LCD_1IN28_SendData_8Bit(0xFF);


    LCD_1IN28_SendCommand(0xB6);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0x20);

    LCD_1IN28_SendCommand(0x36);
    LCD_1IN28_SendData_8Bit(0x08);//Set as vertical screen

    LCD_1IN28_SendCommand(0x3A);
    LCD_1IN28_SendData_8Bit(0x05);


    LCD_1IN28_SendCommand(0x90);
    LCD_1IN28_SendData_8Bit(0x08);
    LCD_1IN28_SendData_8Bit(0x08);
    LCD_1IN28_SendData_8Bit(0x08);
    LCD_1IN28_SendData_8Bit(0x08);

    LCD_1IN28_SendCommand(0xBD);
    LCD_1IN28_SendData_8Bit(0x06);

    LCD_1IN28_SendCommand(0xBC);
    LCD_1IN28_SendData_8Bit(0x00);

    LCD_1IN28_SendCommand(0xFF);
    LCD_1IN28_SendData_8Bit(0x60);
    LCD_1IN28_SendData_8Bit(0x01);
    LCD_1IN28_SendData_8Bit(0x04);

    LCD_1IN28_SendCommand(0xC3);
    LCD_1IN28_SendData_8Bit(0x13);
    LCD_1IN28_SendCommand(0xC4);
    LCD_1IN28_SendData_8Bit(0x13);

    LCD_1IN28_SendCommand(0xC9);
    LCD_1IN28_SendData_8Bit(0x22);

    LCD_1IN28_SendCommand(0xBE);
    LCD_1IN28_SendData_8Bit(0x11);

    LCD_1IN28_SendCommand(0xE1);
    LCD_1IN28_SendData_8Bit(0x10);
    LCD_1IN28_SendData_8Bit(0x0E);

    LCD_1IN28_SendCommand(0xDF);
    LCD_1IN28_SendData_8Bit(0x21);
    LCD_1IN28_SendData_8Bit(0x0c);
    LCD_1IN28_SendData_8Bit(0x02);

    LCD_1IN28_SendCommand(0xF0);
    LCD_1IN28_SendData_8Bit(0x45);
    LCD_1IN28_SendData_8Bit(0x09);
    LCD_1IN28_SendData_8Bit(0x08);
    LCD_1IN28_SendData_8Bit(0x08);
    LCD_1IN28_SendData_8Bit(0x26);
    LCD_1IN28_SendData_8Bit(0x2A);

    LCD_1IN28_SendCommand(0xF1);
    LCD_1IN28_SendData_8Bit(0x43);
    LCD_1IN28_SendData_8Bit(0x70);
    LCD_1IN28_SendData_8Bit(0x72);
    LCD_1IN28_SendData_8Bit(0x36);
    LCD_1IN28_SendData_8Bit(0x37);
    LCD_1IN28_SendData_8Bit(0x6F);


    LCD_1IN28_SendCommand(0xF2);
    LCD_1IN28_SendData_8Bit(0x45);
    LCD_1IN28_SendData_8Bit(0x09);
    LCD_1IN28_SendData_8Bit(0x08);
    LCD_1IN28_SendData_8Bit(0x08);
    LCD_1IN28_SendData_8Bit(0x26);
    LCD_1IN28_SendData_8Bit(0x2A);

    LCD_1IN28_SendCommand(0xF3);
    LCD_1IN28_SendData_8Bit(0x43);
    LCD_1IN28_SendData_8Bit(0x70);
    LCD_1IN28_SendData_8Bit(0x72);
    LCD_1IN28_SendData_8Bit(0x36);
    LCD_1IN28_SendData_8Bit(0x37);
    LCD_1IN28_SendData_8Bit(0x6F);

    LCD_1IN28_SendCommand(0xED);
    LCD_1IN28_SendData_8Bit(0x1B);
    LCD_1IN28_SendData_8Bit(0x0B);

    LCD_1IN28_SendCommand(0xAE);
    LCD_1IN28_SendData_8Bit(0x77);

    LCD_1IN28_SendCommand(0xCD);
    LCD_1IN28_SendData_8Bit(0x63);


    LCD_1IN28_SendCommand(0x70);
    LCD_1IN28_SendData_8Bit(0x07);
    LCD_1IN28_SendData_8Bit(0x07);
    LCD_1IN28_SendData_8Bit(0x04);
    LCD_1IN28_SendData_8Bit(0x0E);
    LCD_1IN28_SendData_8Bit(0x0F);
    LCD_1IN28_SendData_8Bit(0x09);
    LCD_1IN28_SendData_8Bit(0x07);
    LCD_1IN28_SendData_8Bit(0x08);
    LCD_1IN28_SendData_8Bit(0x03);

    LCD_1IN28_SendCommand(0xE8);
    LCD_1IN28_SendData_8Bit(0x34);

    LCD_1IN28_SendCommand(0x62);
    LCD_1IN28_SendData_8Bit(0x18);
    LCD_1IN28_SendData_8Bit(0x0D);
    LCD_1IN28_SendData_8Bit(0x71);
    LCD_1IN28_SendData_8Bit(0xED);
    LCD_1IN28_SendData_8Bit(0x70);
    LCD_1IN28_SendData_8Bit(0x70);
    LCD_1IN28_SendData_8Bit(0x18);
    LCD_1IN28_SendData_8Bit(0x0F);
    LCD_1IN28_SendData_8Bit(0x71);
    LCD_1IN28_SendData_8Bit(0xEF);
    LCD_1IN28_SendData_8Bit(0x70);
    LCD_1IN28_SendData_8Bit(0x70);

    LCD_1IN28_SendCommand(0x63);
    LCD_1IN28_SendData_8Bit(0x18);
    LCD_1IN28_SendData_8Bit(0x11);
    LCD_1IN28_SendData_8Bit(0x71);
    LCD_1IN28_SendData_8Bit(0xF1);
    LCD_1IN28_SendData_8Bit(0x70);
    LCD_1IN28_SendData_8Bit(0x70);
    LCD_1IN28_SendData_8Bit(0x18);
    LCD_1IN28_SendData_8Bit(0x13);
    LCD_1IN28_SendData_8Bit(0x71);
    LCD_1IN28_SendData_8Bit(0xF3);
    LCD_1IN28_SendData_8Bit(0x70);
    LCD_1IN28_SendData_8Bit(0x70);

    LCD_1IN28_SendCommand(0x64);
    LCD_1IN28_SendData_8Bit(0x28);
    LCD_1IN28_SendData_8Bit(0x29);
    LCD_1IN28_SendData_8Bit(0xF1);
    LCD_1IN28_SendData_8Bit(0x01);
    LCD_1IN28_SendData_8Bit(0xF1);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0x07);

    LCD_1IN28_SendCommand(0x66);
    LCD_1IN28_SendData_8Bit(0x3C);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0xCD);
    LCD_1IN28_SendData_8Bit(0x67);
    LCD_1IN28_SendData_8Bit(0x45);
    LCD_1IN28_SendData_8Bit(0x45);
    LCD_1IN28_SendData_8Bit(0x10);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0x00);

    LCD_1IN28_SendCommand(0x67);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0x3C);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0x01);
    LCD_1IN28_SendData_8Bit(0x54);
    LCD_1IN28_SendData_8Bit(0x10);
    LCD_1IN28_SendData_8Bit(0x32);
    LCD_1IN28_SendData_8Bit(0x98);

    LCD_1IN28_SendCommand(0x74);
    LCD_1IN28_SendData_8Bit(0x10);
    LCD_1IN28_SendData_8Bit(0x85);
    LCD_1IN28_SendData_8Bit(0x80);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(0x4E);
    LCD_1IN28_SendData_8Bit(0x00);

    LCD_1IN28_SendCommand(0x98);
    LCD_1IN28_SendData_8Bit(0x3e);
    LCD_1IN28_SendData_8Bit(0x07);

    LCD_1IN28_SendCommand(0x35);
    LCD_1IN28_SendCommand(0x21);

    LCD_1IN28_SendCommand(0x11);
//    DEV_Delay_ms(120);
    SDK_DelayAtLeastUs(120000, SystemCoreClock);
    LCD_1IN28_SendCommand(0x29);
//    DEV_Delay_ms(20);
    SDK_DelayAtLeastUs(20000, SystemCoreClock);
}

void LCD_1IN28_SetAttributes()
{
    // Set the read / write scan direction of the frame memory
    LCD_1IN28_SendCommand(0x36);        //MX, MY, RGB mode
    LCD_1IN28_SendData_8Bit(0X68);	//0x08 set RGB, 0X68 is vertical scan dir
}

void LCD_1IN28_Init()
{
    //Hardware reset
    LCD_1IN28_Reset();

    //Set the resolution and scanning method of the screen
    LCD_1IN28_SetAttributes();

    //Set the initialization register
    LCD_1IN28_InitReg();
}

void LCD_1IN28_SetWindows(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend)
{
    //set the X coordinates
    LCD_1IN28_SendCommand(0x2A);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(Xstart);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(Xend);

    //set the Y coordinates
    LCD_1IN28_SendCommand(0x2B);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(Ystart);
    LCD_1IN28_SendData_8Bit(0x00);
    LCD_1IN28_SendData_8Bit(Yend);

    LCD_1IN28_SendCommand(0X2C);
}

void LCD_1IN28_DrawPaint(UWORD x, UWORD y, UWORD Color)
{
    LCD_1IN28_SetWindows(x,y,x,y);
    LCD_1IN28_SendData_16Bit(Color);
}

void LCD_1IN28_Update_Block(UWORD Xstart, UWORD Ystart, const char * pString, sFONT* Font,
                            UWORD Color_Background, UWORD Color_Foreground, UBYTE Max_Char)
{
    UWORD i, j;
    UWORD Xpoint = Xstart;
    UWORD Ypoint = Ystart;
    UDOUBLE Xoffset = 0;

    if (Xstart > Paint.Width || Ystart > Paint.Height) {
        PRINTF("Input exceeds the normal display range!\r\n");
        return;
    }

    UBYTE *arr = (UBYTE *)malloc(Max_Char * (Font->Width) * (Font->Height) * 2 * sizeof(uint8_t));

    if (FONT_BACKGROUND == Color_Background) {
        //default background is black
        memset(arr, 0x0, Max_Char * (Font->Width) * (Font->Height) * 2 * sizeof(uint8_t));
    }

    while (* pString != '\0') {
        //if X direction filled , reposition to(Xstart,Ypoint),Ypoint is Y direction plus the Height of the character
        if ((Xpoint + Font->Width ) > Paint.Width ) {
            Xpoint = Xstart;
            Ypoint += Font->Height;
        }

        // If the Y direction is full, reposition to(Xstart, Ystart)
        if ((Ypoint  + Font->Height ) > Paint.Height ) {
            Xpoint = Xstart;
            Ypoint = Ystart;
        }

        uint32_t Char_Offset = (* pString - ' ') * Font->Height * (Font->Width / 8 + (Font->Width % 8 ? 1 : 0));
        const unsigned char *ptr = &Font->table[Char_Offset];

        for (i = 0; i < Font->Height; i++) {
            for (j = 0; j < Font->Width * 2; j+=2) {

                //To determine whether the font background color and screen background color is consistent
                if (FONT_BACKGROUND == Color_Background) { //this process is to speed up the scan
                    if (*ptr & (0x80 >> (j / 2 % 8))) {
                        arr[i * (Font->Width) * Max_Char * 2 + j + Xoffset * 2] = Color_Foreground >> 8;
                        arr[i * (Font->Width) * Max_Char * 2 + j + Xoffset * 2 + 1] = Color_Foreground;
                    }
                }
                else {
                    if (*ptr & (0x80 >> (j / 2 % 8))) {
                        arr[i * (Font->Width) * Max_Char * 2 + j + Xoffset * 2] = Color_Foreground >> 8;
                        arr[i * (Font->Width) * Max_Char * 2 + j + Xoffset * 2 + 1] = Color_Foreground;
                    }
                    else {
                        arr[i * (Font->Width) * Max_Char * 2 + j + Xoffset * 2] = Color_Background >> 8;
                        arr[i * (Font->Width) * Max_Char * 2 + j + Xoffset * 2 + 1] = Color_Background;
                    }
                }
                //One pixel is 8 bits
                if (j / 2 % 8 == 7)
                    ptr++;
            }// Write a line
            if (Font->Width % 8 != 0)
                ptr++;
        }// Write all

        //The next character of the address
        pString++;

        //The next word of the abscissa increases the font of the broadband
        Xpoint += Font->Width;

        Xoffset += Font->Width;
    }

    LCD_1IN28_SetWindows(Xstart, Ystart, Xstart + (Font->Width) * Max_Char - 1, Ystart + Font->Height - 1);

    LCD_1IN28_DC_1;

    DEV_SPI_BLOCK_WRITE(arr, (Max_Char * (Font->Width) * (Font->Height) * 2 * sizeof(uint8_t)));
    free(arr);
}

void LCD_1IN28_Clear(UWORD Color)
{
    UWORD i,j;
    LCD_1IN28_SetWindows(0, 0, LCD_1IN28_WIDTH - 1, LCD_1IN28_HEIGHT - 1);

    LCD_1IN28_DC_1;
    for(i = 0; i < LCD_1IN28_WIDTH; i++){
        for(j = 0; j < LCD_1IN28_HEIGHT; j++){
            DEV_SPI_WRITE(Color>>8);
            DEV_SPI_WRITE(Color);
        }
    }
}

void LCD_1IN28_Clear_Block(UWORD Color)
{
    UWORD i,j;
    UBYTE *arr = (UBYTE *)malloc(LCD_1IN28_WIDTH * LCD_1IN28_HEIGHT * 2 * sizeof(uint8_t) / WRITE_BLOCK_NUM);

    if (FONT_BACKGROUND == Color) {
        //default background is black
        memset(arr, 0x0, LCD_1IN28_WIDTH * LCD_1IN28_HEIGHT * 2 * sizeof(uint8_t) / WRITE_BLOCK_NUM);
    }
    else {
        for(i = 0; i < LCD_1IN28_WIDTH; i++){
            for(j = 0; j < LCD_1IN28_HEIGHT * 2 / WRITE_BLOCK_NUM; j+=2){
                arr[i * LCD_1IN28_HEIGHT * 2 / WRITE_BLOCK_NUM + j] = Color>>8;
                arr[i * LCD_1IN28_HEIGHT * 2 / WRITE_BLOCK_NUM + j + 1] = Color;
            }
        }
    }

    LCD_1IN28_SetWindows(0, 0, LCD_1IN28_WIDTH - 1, LCD_1IN28_HEIGHT - 1);
    LCD_1IN28_DC_1;

    for(i = 0; i < WRITE_BLOCK_NUM; i++){
        DEV_SPI_BLOCK_WRITE(arr, LCD_1IN28_WIDTH * LCD_1IN28_HEIGHT * 2 * sizeof(uint8_t) / WRITE_BLOCK_NUM);
    }

    free(arr);
}

void LCD_1in28_test()
{
    /*Set clock source for LPSPI and get master clock source*/
    CLOCK_SetIpSrc(EXAMPLE_LPSPI_MASTER_CLOCK_NAME, EXAMPLE_LPSPI_MASTER_CLOCK_SOURCE);
    RESET_PeripheralReset(kRESET_Lpspi0);

    LPSPI_Init();

    PRINTF("LCD_1IN28 Init and Clear...\r\n");
    LCD_1IN28_Init();
    /* Write pixel one by one */
    //LCD_1IN28_Clear(BLACK);
    /* Write pixel with block */
    LCD_1IN28_Clear_Block(BLACK);

    PRINTF("Paint_NewImage\r\n");
    Paint_NewImage(LCD_1IN28_WIDTH, LCD_1IN28_HEIGHT, 0, BLACK);

    PRINTF("Set Clear and Display Funtion\r\n");
    Paint_SetClearFuntion(LCD_1IN28_Clear_Block);
    Paint_SetDisplayFuntion(LCD_1IN28_DrawPaint);

    PRINTF("Drawing...\r\n");
    LCD_1IN28_Update_Block(40, 45, "04.05 FRI", &Font24, BLACK, LIGHTGREEN, 9);
    LCD_1IN28_Update_Block(50, 80, "10:28", &Font24, BLACK, WHITE, 5);
}
