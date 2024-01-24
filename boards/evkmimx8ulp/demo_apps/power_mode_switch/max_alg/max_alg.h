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

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

status_t MAX_Alg_Init(max_config_t *configure, max_sample_buf_t *input_samples);
status_t MAX_Alg_Start_HR_SpO2_Cal(void);
uint32_t MAX_Alg_Read_HeartRate(void);
uint32_t MAX_Alg_Read_SpO2(void);

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_MAX_H_ */
