/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_max.h"
#include "max_alg.h"
//#include "fsl_timer.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
#define MAX_RED_CURRENT_MA      (7000U)
#define MAX_IR_CURRENT_MA       (7000U)
#define MAX_GREEN_CURRENT_MA    (7000U)
#define MAX_PROXY_CURRENT_MA    (0U)

static int32_t max_initialized = 0;

/******************************************************************************
 * Code
 ******************************************************************************/
status_t MAX_Enable_Temp_Power(max_handle_t *max_handle, bool en)
{
    uint8_t tmp = 0;

    if (MAX_ReadReg(max_handle, MODE_CFG_REG, &tmp, 1) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    tmp = en ? (tmp & ~MODE_CFG_SHUTDOWN_MASK) : (tmp | MODE_CFG_SHUTDOWN_MASK);

    return MAX_WriteReg(max_handle, MODE_CFG_REG, tmp);
}

static status_t MAX_Enable_Mode_With_Power(max_handle_t *max_handle, uint8_t mode, bool en)
{
    uint8_t reg_val = mode;
    status_t result = kStatus_Success;

    if (en)
    {
        if (kMAX_HrSpo2Mode == mode)
        {
            /* SpO2 can be calibrated using temperature, so we will read temperature first. */
            MAX_StartTemp(max_handle);
        }
    }
    else
    {
        reg_val |= MODE_CFG_SHUTDOWN_MASK;
    }

    result = MAX_WriteReg(max_handle, MODE_CFG_REG, reg_val);

    return result;
}

static status_t MAX_Get_Power_Status(max_handle_t *max_handle, int32_t *is_power_on)
{
    uint8_t tmp = 0;
    status_t result = kStatus_Success;

    if (MAX_ReadReg(max_handle, MODE_CFG_REG, &tmp, 1) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    *is_power_on = (tmp & MODE_CFG_SHUTDOWN_MASK) ? 0 : 1;

    return result;
}

#define MAX_LED_MA_2_INDEX(led_cur_ma, led_cur_index) { \
    led_cur_index = led_cur_ma / 200;   \
    if (led_cur_index > 0xff)   \
        return kStatus_OutOfRange;  \
}

static status_t MAX_Led_Init(max_handle_t *max_handle, uint32_t led_red_ma, uint32_t led_ir_ma, uint32_t led_green_ma, uint32_t led_proxy_ma)
{
    int32_t reg_val = 0;

    MAX_LED_MA_2_INDEX(led_red_ma, reg_val);
    /* Set Red LED Reg */
    if (MAX_WriteReg(max_handle, LED_RED_PA_REG, reg_val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    /* Set IR LED Reg */
    MAX_LED_MA_2_INDEX(led_ir_ma, reg_val);
    if (MAX_WriteReg(max_handle, LED_IR_PA_REG, reg_val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    /* Set Green LED Reg */
    MAX_LED_MA_2_INDEX(led_green_ma, reg_val);
    if (MAX_WriteReg(max_handle, LED_GREEN_PA_REG, reg_val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    /* Set Proxy LED Reg */
    MAX_LED_MA_2_INDEX(led_proxy_ma, reg_val);
    if (MAX_WriteReg(max_handle, LED_PROXY_PA_REG, reg_val) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t MAX_Reset(max_handle_t *max_handle)
{
    uint8_t tmp = 0;
    status_t result;

    /* Reset */
    result = MAX_WriteReg(max_handle, MODE_CFG_REG, MODE_CFG_RST_MASK);
    if (result != kStatus_Success)
    {
        return result;
    }

    /* Poll the RESET bit until it's cleared by hardware.*/
    while (1)
    {
        result = MAX_ReadReg(max_handle, MODE_CFG_REG, &tmp, 1);
        if (result != kStatus_Success)
        {
            return result;
        }

        if (!(tmp & MODE_CFG_RST_MASK))
        {
            break;
        }
    }

    return kStatus_Success;
}

status_t MAX_Start_HrSpO2(max_handle_t *max_handle, max_config_t *configure, bool en)
{
    max_mode_t mode = kMAX_None_Mode;

    if (configure)
    {
        mode = configure->mode;
    }
    /* Set mode and power on */
    return MAX_Enable_Mode_With_Power(max_handle, mode, en);
}

status_t MAX_Init(max_handle_t *max_handle, max_config_t *configure)
{
    uint8_t tmp = 0;

    assert(max_handle);

    if (!max_initialized)
    {
        assert(configure);

        /* Initialize the I2C access function. */
        max_handle->I2C_SendFunc    = configure->I2C_SendFunc;
        max_handle->I2C_ReceiveFunc = configure->I2C_ReceiveFunc;

        if (MAX_Reset(max_handle) != kStatus_Success)
        {
            return kStatus_Fail;
        }

        if (MAX_ReadReg(max_handle, ID_PART_REG, &tmp, 1) != kStatus_Success)
        {
            return kStatus_Fail;
        }
        
        if (tmp != kMAX_WHO_AM_I_Device_ID)
        {
            return kStatus_Fail;
        }
        PRINTF("max3010x part id %02x\r\n", tmp);
        
        if (MAX_ReadReg(max_handle, ID_REV_REG, &tmp, 1) != kStatus_Success)
        {
            return kStatus_Fail;
        }
        PRINTF("max3010x revision %02x\r\n", tmp);

        /* Clear mode setting and shutdown chip */
        if (MAX_Enable_Mode_With_Power(max_handle, 0, false) != kStatus_Success)
        {
            return kStatus_Fail;
        }

        /* Set LED Reg */
        MAX_Led_Init(max_handle, MAX_RED_CURRENT_MA, MAX_IR_CURRENT_MA, MAX_GREEN_CURRENT_MA, MAX_PROXY_CURRENT_MA);

        /* FIFO, average by 4 samples and generate FIFO interrupt when unread data samples reach 31 */
        if (MAX_WriteReg(max_handle, FIFO_CFG_REG, (configure->sampleavg << FIFO_SMP_AVE_SHIFT) | 0x1) != kStatus_Success)
        {
            return kStatus_Fail;
        }

        /* Configure HR, SpO2 SampleRate */
        tmp = (configure->adcRange << SPO2_ADC_RAGE_SHIFT) | (configure->samples << SPO2_SR_SHIFT) | configure->pulsewidth;
        if (MAX_WriteReg(max_handle, SPO2_CFG_REG, tmp) != kStatus_Success)
        {
            return kStatus_Fail;
        }

        if (MAX_WriteReg(max_handle, FIFO_WRPTR_REG, 0) != kStatus_Success)
        {
            return kStatus_Fail;
        }

        if (MAX_WriteReg(max_handle, FIFO_OVPTR_REG, 0) != kStatus_Success)
        {
            return kStatus_Fail;
        }

        if (MAX_WriteReg(max_handle, FIFO_RDPTR_REG, 0) != kStatus_Success)
        {
            return kStatus_Fail;
        }

        /* Enable */
        if (MAX_WriteReg(max_handle, INT_ENABLE_REG1, MAXIM_EN_IRQ_FIFO_ALMOST_FULL_MASK) != kStatus_Success)
        {
            return kStatus_Fail;
        }

        /* Configure mode */
        if (kMAX_MultiLedMode == configure->mode)
        {
            /* Slot1 for RED, Slot2 for IR */
            if (MAX_WriteReg(max_handle, MULTI_MODE_REG1, 0x21) != kStatus_Success)
            {
                return kStatus_Fail;
            }
            /* Slot3 for Green, Slot4 for None */
            if (MAX_WriteReg(max_handle, MULTI_MODE_REG2, 0x3) != kStatus_Success)
            {
                return kStatus_Fail;
            }
        }

        max_initialized = 1;
    }

    /* Set mode and power on */
    return kStatus_Success;
}

status_t MAX_Deinit(max_handle_t *max_handle)
{
    return MAX_Enable_Mode_With_Power(max_handle, 0, false);
}

status_t MAX_ReadSensorData(max_handle_t *max_handle, uint8_t *sampleData, uint8_t *sampleNum)
{
    status_t status   = kStatus_Success;
    uint8_t fifoWrPtr = 0;
    uint8_t fifoRdPtr = 0;
    uint8_t fifoOvPtr = 0;
    /* Temporary buffer to read samples from the FIFO buffer.*/
    uint8_t tmpBuf[MAXIM_FIFO_DEPTH * MAXIM_BYTES_PER_ADC_VALUE];
    uint8_t numAvailSam;

    /* Read FIFO pointers. */
    if (MAX_ReadReg(max_handle, FIFO_WRPTR_REG, &fifoWrPtr, 1) != kStatus_Success)
    {
        status = kStatus_Fail;
    }

    if (MAX_ReadReg(max_handle, FIFO_OVPTR_REG, &fifoOvPtr, 1) != kStatus_Success)
    {
        status = kStatus_Fail;
    }

    if (MAX_ReadReg(max_handle, FIFO_RDPTR_REG, &fifoRdPtr, 1) != kStatus_Success)
    {
        status = kStatus_Fail;
    }

    if (fifoOvPtr > 0)
    {
        MAX_ReadReg(max_handle, FIFO_DATA_REG, tmpBuf, MAXIM_FIFO_DEPTH * MAXIM_BYTES_PER_ADC_VALUE);

        numAvailSam = 0;
    }
    else
    {
        if (fifoWrPtr > fifoRdPtr)
        {
            numAvailSam = fifoWrPtr - fifoRdPtr;
        }
        else if (fifoWrPtr < fifoRdPtr)
        {
            numAvailSam = fifoWrPtr + MAXIM_FIFO_DEPTH - fifoRdPtr;
        }
        else
        {
            numAvailSam = 0;
        }

        if (numAvailSam > 0)
        {
            MAX_ReadReg(max_handle, FIFO_DATA_REG, &tmpBuf[0], numAvailSam * MAXIM_BYTES_PER_ADC_VALUE);

            if (sampleData != NULL)
            {
                memcpy(sampleData, tmpBuf, numAvailSam * MAXIM_BYTES_PER_ADC_VALUE);
            }
        }
    }

    if (sampleNum != NULL)
    {
        *sampleNum = numAvailSam;
    }

    return status;
}

status_t MAX_FillSensorDataToBuffer(max_handle_t *handle, max_sample_buf_t *sample_buf)
{
    uint32_t cur_sample_cnt = sample_buf->sample_count;
    uint8_t out_sample_num = 0;
    status_t status   = kStatus_Success;

    if (cur_sample_cnt < MAX_CFG_TOTAL_SAMPLES)
    {
        max_sample_t *sample_buf_ptr = &sample_buf->sample_buf[cur_sample_cnt];

        status = MAX_ReadSensorData(handle, (uint8_t *)sample_buf_ptr, &out_sample_num);
        sample_buf->sample_count = cur_sample_cnt + out_sample_num;

        return status;
    }
    else
    {
        /* Doing dummy read */
        MAX_ReadSensorData(handle, NULL, &out_sample_num);
        /* Should be in algorithm calculation */
        return kStatus_Busy;
    }
}

status_t MAX_ReadReg(max_handle_t *handle, uint8_t reg, uint8_t *val, uint16_t bytesNumber)
{
    assert(handle);
    assert(val);

    if (!handle->I2C_ReceiveFunc)
    {
        return kStatus_Fail;
    }

    return handle->I2C_ReceiveFunc(MAX_I2C_ADDRESS, reg, 1, val, bytesNumber, 0);
}

status_t MAX_WriteReg(max_handle_t *handle, uint8_t reg, uint8_t val)
{
    assert(handle);

    if (!handle->I2C_SendFunc)
    {
        return kStatus_Fail;
    }

    return handle->I2C_SendFunc(MAX_I2C_ADDRESS, reg, 1, val, 0);
}

status_t MAX_InitTemp(max_handle_t *handle)
{
    if (!max_initialized)
    {
        return kStatus_Fail;
    }

    /* Enable temperature interrupt */
    return MAX_WriteReg(handle, INT_ENABLE_REG2, MAXIM_EN_IRQ_INT_TEMP_RDY_MASK);
}

status_t MAX_DeinitTemp(max_handle_t *handle)
{
    /* Disable temperature interrupt */
    return MAX_WriteReg(handle, INT_ENABLE_REG2, 0);
}

status_t MAX_ReadTemp(max_handle_t *handle, int32_t *temp_deg)
{
    uint8_t temp_int = 0, temp_frac = 0;

    if (MAX_ReadReg(handle, TEMP_INT_REG, &temp_int, 1) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    *temp_deg = temp_int << 4;
    if (MAX_ReadReg(handle, TEMP_FRAC_REG, &temp_frac, 1) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    *temp_deg |= temp_frac & 0xf;
    /* Convert to signed value */
    *temp_deg = (int32_t)(*temp_deg << (31 - 11)) >> (31 - 11);

    return kStatus_Success;
}

status_t MAX_GetHeartRate(max_handle_t *handle, uint32_t *heartrate_beats)
{
    *heartrate_beats = MAX_Alg_Read_HeartRate();;

    return kStatus_Success;
}

status_t MAX_GetSpO2(max_handle_t *handle, uint32_t *spo2_rate)
{
    *spo2_rate = MAX_Alg_Read_SpO2();;

    return kStatus_Success;
}

status_t MAX_StartTemp(max_handle_t *handle)
{
    status_t result = kStatus_Success;
    int32_t pwr_origin_on = 0;
    //uint8_t tmp = 0;

    result = MAX_Get_Power_Status(handle, &pwr_origin_on);
    if (kStatus_Success != result)
    {
        return result;
    }

    /* If power is turned on by temp, we should also turn it off. */
    if (!pwr_origin_on)
    {
        result = MAX_Enable_Temp_Power(handle, true);
        if (kStatus_Success != result)
        {
            return result;
        }
    }

    /* start acquisition */
    result = MAX_WriteReg(handle, TEMP_CONFIG_REG, TEMP_CONFIG_TEMP_EN_MASK);

    return result;
}

status_t MAX_FlushFIFO(max_handle_t *handle)
{
    uint8_t tmp;

    MAX_ReadSensorData(handle, NULL, &tmp);

    if (kStatus_Success != MAX_ReadReg(handle, INT_STATUS_REG1, &tmp, 1))
    {
        PRINTF("Read MAX Register 1 failed!\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

#define MAX_DUMP_REG(hdl, reg)  {   \
    uint8_t temp = 0;   \
    MAX_ReadReg(hdl, reg, &temp, 1);   \
    PRINTF(#reg" = 0x%02x\r\n", temp);    \
     }

status_t MAX_DumpAllRegs(max_handle_t *handle)
{
    assert(handle);

    if (!handle->I2C_SendFunc)
    {
        return kStatus_Fail;
    }

    PRINTF("========================== MAX Regs Start===========================\r\n");

    MAX_DUMP_REG(handle, INT_STATUS_REG1);
    MAX_DUMP_REG(handle, INT_STATUS_REG2);
    MAX_DUMP_REG(handle, INT_ENABLE_REG1);
    MAX_DUMP_REG(handle, INT_ENABLE_REG2);
    MAX_DUMP_REG(handle, FIFO_WRPTR_REG);
    MAX_DUMP_REG(handle, FIFO_OVPTR_REG);
    MAX_DUMP_REG(handle, FIFO_RDPTR_REG);
    MAX_DUMP_REG(handle, FIFO_DATA_REG);
    MAX_DUMP_REG(handle, FIFO_CFG_REG);
    MAX_DUMP_REG(handle, MODE_CFG_REG);
    MAX_DUMP_REG(handle, SPO2_CFG_REG);
    MAX_DUMP_REG(handle, LED_RED_PA_REG);
    MAX_DUMP_REG(handle, LED_IR_PA_REG);
    MAX_DUMP_REG(handle, LED_GREEN_PA_REG);
    MAX_DUMP_REG(handle, LED_PROXY_PA_REG);
    MAX_DUMP_REG(handle, MULTI_MODE_REG1);
    MAX_DUMP_REG(handle, MULTI_MODE_REG2);
    MAX_DUMP_REG(handle, TEMP_INT_REG);
    MAX_DUMP_REG(handle, TEMP_FRAC_REG);
    MAX_DUMP_REG(handle, TEMP_CONFIG_REG);
    MAX_DUMP_REG(handle, ID_REV_REG);
    MAX_DUMP_REG(handle, ID_PART_REG);

    PRINTF("========================== MAX Regs End ===========================\r\n");

    MAX_FlushFIFO(handle);

    return kStatus_Success;
}
