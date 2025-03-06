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
#define MIN_INTERVAL 0.3f  
#define MAX_INTERVAL 1.5f  
#define sample_rate 101

static max_alg_config_t g_alg_config;
static SemaphoreHandle_t sem_start_cal;

uint8_t no_signal_counter = 0;
uint8_t sum_sample = 0;

/*
NCI Due to the different echo intensities between the fingertips and fingerpulps, 
if the position of the fingers may not be correct, and the signal may not be detected. 
The next step is to add a function that automatically sets the threshold.
*/ 

float w[2] = {0.0, 0.0};
float v[3] = {0.0, 0.0, 0.0};
float w2[2] = {0.0, 0.0};
float dx_buffer[4] = {0,0,0,0};
float last_w = 0;
float heartrate_sample_last;

bool is_positive_corsszero_point = false;
bool is_negative_corsszero_point = false;
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
    // w[1] = (0.13700924619520193914 * x) + (0.72598150760959612171 * w[0]);
    // w[1] = (0.05932306989923252089 * x) + (0.88135386020153494435 * w[0]); // Fs=200, Fc=4
    w[1] = (0.03053896819732337953 * x) + (0.93892206360535324094 * w[0]); // Fs=200, Fc=2

    return (w[0] + w[1]);
}

int compare(const void* a, const void* b) {
    return (*(uint8_t*)a - *(uint8_t*)b);
}

float MAX_Alg_Lowbus_Filter2(float x)
{

    w2[0] = w2[1];
    w2[1] = (0.13700924619520193914 * x) + (0.72598150760959612171 * w2[0]); // Fs=200, Fc=2

    return (w2[0] + w2[1]);
}


bool MAX_Alg_Checkbeat(float sample, float rawdata )
{

    float heartrate_sample_current;
    bool heart_beated = false;

    heartrate_sample_current = sample;

    if (rawdata < 4000) // if no finger, skip
    {
        return false;
    }

    if(((heartrate_sample_last < 0) && (heartrate_sample_current >= 0)) || ((heartrate_sample_last <= 0) && (heartrate_sample_current > 0))) // find the positive corss-zero point
    {
        is_positive_corsszero_point = true;
    }

    if(((heartrate_sample_last > 0)&& (heartrate_sample_current <= 0)) || ((heartrate_sample_last >= 0) && (heartrate_sample_current < 0))) // find the negative corss-zero point
    {
        is_negative_corsszero_point = true;
    }

    sum_sample++;

    if (is_positive_corsszero_point && is_negative_corsszero_point)
    {
        if((sum_sample > 50) && (sum_sample < 180))
        {
            heart_beated = true;
        }
        sum_sample = 0;
        is_positive_corsszero_point = false;
        is_negative_corsszero_point = false;
    }

    heartrate_sample_last = heartrate_sample_current;

    return heart_beated;
}

status_t MAX_Alg_Cal_HeartRate(max_alg_config_t *config, uint32_t *hr_beats)
{

    uint32_t sample_value;

    uint32_t a[15] = {0};
    uint8_t valid_intervals[18] = {0};
    uint8_t valid_count = 0;
    const int min_interval_samples = (int)(MIN_INTERVAL * sample_rate);
    const int max_interval_samples = (int)(MAX_INTERVAL * sample_rate);
    uint8_t median_interval = 0;
    uint8_t beat_index = 0;
    for (uint32_t sample_index=0; sample_index < config->sample_buf->sample_count; sample_index++)
    {
        
        float ac_result=0;
        float lp_result=0;
        float lp_result2=0;
        bool check_heart_beat = false;
        sample_value = MAX_Sample_Adapter((config->sample_buf->sample_buf + sample_index));
        
        lp_result = MAX_Alg_Lowbus_Filter((float)sample_value);
        ac_result = MAX_Alg_Remove_DC(lp_result);
        lp_result2 = MAX_Alg_Lowbus_Filter2(ac_result);
        check_heart_beat = MAX_Alg_Checkbeat(lp_result2, (float)sample_value);
        // PRINTF("{P0|SAMPLE|0,0,255|%f}\r\n", (float)sample_value);
        // PRINTF("{P1|LP|255,0,0|%f}\r\n", lp_result);
        // PRINTF("{P2|AC|0,255,0|%f}\r\n", ac_result);
        // PRINTF("{P3|LP2|255,0,0|%f}\r\n", lp_result2);            
        // PRINTF("{P3|BEAT|255,0,255|%d}\r\n", check_heart_beat);
        if (check_heart_beat)
        {
            a[beat_index] = sample_index;
            
            if(beat_index > 0)
            {
                if ((a[beat_index] - a[beat_index-1]) > min_interval_samples && (a[beat_index] - a[beat_index-1]) < max_interval_samples)
                {
                    valid_intervals[valid_count] = a[beat_index] - a[beat_index-1];
                    valid_count++; 
                }

            }
            beat_index++;
        }
        
    }

    if (valid_count < 1) 
    {
        no_signal_counter++;
    }
    else
    {
        qsort(valid_intervals, valid_count, sizeof(uint8_t), compare);
        median_interval = valid_count % 2 ? valid_intervals[valid_count/2] : (valid_intervals[valid_count/2-1] + valid_intervals[valid_count/2])/2;
        
        *hr_beats = (float)60 * 10000 / (median_interval) / (config->sample_buf->sample_count / MAX_CFG_SAMPLING_TIME);  
        // PRINTF("*hr_beats: %d\r\n", *hr_beats);    
        no_signal_counter = 0;          
    }

    if(no_signal_counter == 1)
    {
        *hr_beats = 0;
        no_signal_counter = 0;
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
