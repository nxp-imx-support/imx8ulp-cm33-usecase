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
#define DC_REMOVE_ALPHA              0.5

static max_alg_config_t g_alg_config;
static SemaphoreHandle_t sem_start_cal;

uint8_t no_signal_counter = 0;
uint8_t signal_counter = 0;
uint8_t start_signal = 0;
uint8_t last_start_signal;
uint8_t beatcounter = 0;
uint32_t last_sample_index = 0;

/*
NCI Due to the different echo intensities between the fingertips and fingerpulps, 
if the position of the fingers may not be correct, and the signal may not be detected. 
The next step is to add a function that automatically sets the threshold.
*/ 

float heartrate_counter[3] = {0,0,0};
float w[2] = {0.0, 0.0};
float last_sample = 0;
float dx_buffer[4] = {0,0,0,0};
float sum_BPM = 0;
float sum_sample = 0;
float last_w = 0;
float heartrate_sample_last;
/******************************************************************************
 * Code
 ******************************************************************************/

uint32_t MAX_Sample_Adapter(max_sample_t *raw_sample)
{
    uint32_t spliced_data=0;
    uint8_t temp[3];

    temp[2] = *(raw_sample->sample_data);
    temp[1] = *(raw_sample->sample_data+1);
    temp[0] = *(raw_sample->sample_data+2);

    spliced_data = (((temp[2] << 16) | (temp[1] << 8) | temp[0]) & 0x3FFFF);

    return spliced_data;
}

float MAX_Alg_Remove_DC(float rawdata)
{
    float result;
    float curr_w;

    curr_w = rawdata + DC_REMOVE_ALPHA * last_w;
    result = curr_w - last_w;
    last_w = curr_w;

    return result;
}

/**
 * http://www.schwietering.com/jayduino/filtuino/
 * Low pass chebyshev filter order=1 alpha1=0.1
 * Fs=100Hz, Fc=10Hz
 */
float MAX_Alg_Lowbus_Filter(float x)
{

    w[0] = w[1];
    w[1] = (2.456770461833230612e-1 * x) + (0.50864590763335382206 * w[0]);

    return (w[0] + w[1]);
}

float MAX_Alg_Diff_Avg(float sample)
{
    float sample_dx;
    float result;
    float buffer_sum;
    
    sample_dx = (sample - last_sample) * 2;
    last_sample = sample;

    dx_buffer[0] = sample_dx;

    buffer_sum = dx_buffer[0] + dx_buffer[1] + dx_buffer[2] + dx_buffer[3];

    dx_buffer[3] = dx_buffer[2];
    dx_buffer[2] = dx_buffer[1];
    dx_buffer[1] = dx_buffer[0];
    result = buffer_sum / 4;
    
    return result;
}

bool MAX_Alg_Checkbeat(float sample)
{

    float heartrate_sample_current;
    bool heart_beated = false;

    heartrate_sample_current = sample;

    if(((heartrate_sample_last < 0) & (heartrate_sample_current >= 0)) | ((heartrate_sample_last <= 0) & (heartrate_sample_current > 0))) // find the positive corss-zero point
    {
        sum_sample = 0;
        start_signal++;
    }

    if(((heartrate_sample_last > 0)& (heartrate_sample_current <= 0)) | ((heartrate_sample_last >= 0) & (heartrate_sample_current < 0))) // find the negative corss-zero point
    {
        sum_sample = 0;
        start_signal++;
    }

    sum_sample = sum_sample + sample;

    if ((sum_sample >= 200) & (sum_sample < 1000) & (start_signal != last_start_signal))
    {
        heart_beated = true;

        last_start_signal = start_signal;
    }

    heartrate_sample_last = heartrate_sample_current;

    // PRINTF("{P1|GREEN AC|0,255,0|%f}\r\n", sum_sample); // Used for viewing the wave

    return heart_beated;
}

status_t MAX_Alg_Cal_HeartRate(max_alg_config_t *config, uint32_t *hr_beats)
{
    /* Fake code to test */
    uint32_t sample_value;
    float BPM;
    no_signal_counter++;
    for (uint32_t sample_index=0; sample_index < config->sample_buf->sample_count; sample_index++)
    {
        {
            float ac_result=0;
            float dx_result=0;
            float lp_result=0;
            bool check_heart_beat = false;
            sample_value = MAX_Sample_Adapter((config->sample_buf->sample_buf + sample_index));

            ac_result = MAX_Alg_Remove_DC((float)sample_value);
            dx_result = MAX_Alg_Diff_Avg(ac_result);
            lp_result = MAX_Alg_Lowbus_Filter(dx_result);
            // PRINTF("{P0|RED RAW|255,0,0|%f}\r\n", dx_result);
            check_heart_beat = MAX_Alg_Checkbeat(lp_result);

            if (check_heart_beat)
            {

                signal_counter++;
                no_signal_counter = 0;
                if (signal_counter >= 5)
                { 
                    BPM = 60/((sample_index - last_sample_index)*(float)MAX_CFG_SAMPLING_TIME/config->sample_buf->sample_count);
                    
                    // PRINTF("current_timestamp: %d\r\n", sample_index);
                    // PRINTF("last_sample_index: %d\r\n", last_sample_index);
                    // PRINTF("BPM: %f\r\n", BPM);
                    last_sample_index = sample_index;

                    if (BPM > 0 & BPM < 150)
                    {
                        beatcounter++;
                        sum_BPM = BPM + sum_BPM;
                        // PRINTF("BPM: %f\r\n", BPM);
                        // PRINTF("beatcounter: %d\r\n", beatcounter);
                        heartrate_counter[0] = (sum_BPM / beatcounter);  
                    }
                }
            }
        }
    }
    sum_BPM = 0;
    beatcounter = 0;
    last_sample_index = 0;

    *hr_beats = (uint8_t)((heartrate_counter[0] + heartrate_counter[1] + heartrate_counter[2]) /3);
    // PRINTF("-------------------------------------------------------\r\n");
    // PRINTF("\r\nbeatreate0 is %f\r\n", heartrate_counter[0]);
    // PRINTF("\r\nbeatreate1 is %f\r\n", heartrate_counter[1]);
    // PRINTF("\r\nbeatreate2 is %f\r\n", heartrate_counter[2]);
    // PRINTF("-------------------------------------------------------\r\n");   
    
    heartrate_counter[2] = heartrate_counter[1];
    heartrate_counter[1] = heartrate_counter[0];
    if (no_signal_counter > 2)
    {
        heartrate_counter[0] = 0;
        heartrate_counter[1] = 0;
        heartrate_counter[2] = 0;
        no_signal_counter = 0;
        signal_counter = 0;
    }
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
