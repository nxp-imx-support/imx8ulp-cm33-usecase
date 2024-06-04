/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _MAX_ALG_H_
#define _MAX_ALG_H_

#include "fsl_max.h"
#include "fsl_common.h"

typedef struct _max_alg_result_t
{
    uint32_t hr_beats;
    uint32_t spo2_rate;
} max_alg_result_t;    

typedef struct _max_alg_config_t
{
    max_mode_t mode;
    max_samplerate_t samples;
    max_sample_buf_t *sample_buf;
    max_alg_result_t cal_result;
} max_alg_config_t;
/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

bool MAX_Alg_Checkbeat(float sample);
float MAX_Alg_Remove_DC(float rawdata);
float MAX_Alg_Lowbus_Filter(float x);
float MAX_Alg_Diff_Avg(float sample);
status_t MAX_Alg_Cal_HeartRate(max_alg_config_t *config, uint32_t *hr_beats);
status_t MAX_Alg_Init(max_config_t *configure, max_sample_buf_t *input_samples);
status_t MAX_Alg_Start_HR_SpO2_Cal(void);
uint32_t MAX_Sample_Adapter(max_sample_t *raw_sample);
uint32_t MAX_Alg_Read_HeartRate(void);
uint32_t MAX_Alg_Read_SpO2(void);
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_MAX_H_ */
