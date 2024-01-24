/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_max.h"
#include "max_alg.h"
#include "max_cfg.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
#define APP_HRSPO2_ALG_TASK_PRIO    (2U)

typedef struct _max_alg_config_t
{
    max_mode_t mode;
    max_samplerate_t samples;
    max_sample_buf_t *sample_buf;
    max_alg_result_t cal_result;
} max_alg_config_t;

static max_alg_config_t g_alg_config;
static SemaphoreHandle_t sem_start_cal;
/******************************************************************************
 * Code
 ******************************************************************************/
static status_t MAX_Alg_Cal_HeartRate(max_alg_config_t *config, uint32_t *hr_beats)
{
    /* Fake code to test */
    *hr_beats = 60;
    int32_t timeout_alg = 100000;

    PRINTF("\r\nhr alg begin\r\n");
    while (timeout_alg--)
    {
        __asm("nop");
    }

    PRINTF("\r\nhr alg end\r\n");

    return kStatus_Success;
}

static status_t MAX_Alg_Cal_SpO2(max_alg_config_t *config, uint32_t *spo2_rate)
{
    /* Fake code to test */
    *spo2_rate = 98;

    return kStatus_Success;
}

uint32_t MAX_Alg_Read_HeartRate(void)
{
    return g_alg_config.cal_result.hr_beats;
}

uint32_t MAX_Alg_Read_SpO2(void)
{
    return g_alg_config.cal_result.spo2_rate;
}

status_t MAX_Alg_Start_HR_SpO2_Cal(void)
{
    xSemaphoreGive(sem_start_cal);

    return kStatus_Success;
}

/* Read measurement data and do calculation. */
void MAX_Alg_HR_SpO2_Cal_Task(void *pvParameters)
{
    max_alg_config_t *config = pvParameters;
    status_t result = kStatus_Success;

    while (1)
    {
        if (pdTRUE == xSemaphoreTake(sem_start_cal, portMAX_DELAY))
        {
            result = MAX_Alg_Cal_HeartRate(config, &config->cal_result.hr_beats);
            if (kStatus_Success != result)
            {
                PRINTF("HeartRate Cal failed!\r\n");
                return;
            }

            result = MAX_Alg_Cal_SpO2(config, &config->cal_result.spo2_rate);
            if (kStatus_Success != result)
            {
                PRINTF("SpO2 Cal failed!\r\n");
                return;
            }

            config->sample_buf->sample_count = 0;
        }
    }
}

status_t MAX_Alg_Init(max_config_t *configure, max_sample_buf_t *input_samples)
{
    memset(&g_alg_config, 0, sizeof(max_alg_config_t));

    g_alg_config.mode = configure->mode;
    g_alg_config.samples = configure->samples;
    g_alg_config.sample_buf = input_samples;

    sem_start_cal = xSemaphoreCreateBinary();
    if (sem_start_cal == NULL)
    {
        PRINTF("Alg Cal semaphore creation failed.\r\n");
        return kStatus_Fail;
    }

    if (pdPASS != xTaskCreate(MAX_Alg_HR_SpO2_Cal_Task, "HRSpO2 Alg", 256U, &g_alg_config, APP_HRSPO2_ALG_TASK_PRIO, NULL))
    {
        PRINTF("HrSpO2 Alg Task creation failed!.\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}
