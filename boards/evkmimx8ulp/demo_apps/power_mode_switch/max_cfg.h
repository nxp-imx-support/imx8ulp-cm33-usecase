/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _MAX_CFG_H_
#define _MAX_CFG_H_

#include "fsl_max.h"
#include "fsl_common.h"

/* MAX Configurations */
#define MAX_CFG_MODE                (kMAX_HrSpo2Mode)
#define MAX_CFG_PULSEWIDTH          (kMAX_PW_411US_18Bits)
#define MAX_CFG_SAMPLERATE          (kMAX_SR_400Hz)
#define MAX_CFG_ADCRANGE            (kMAX_AdcRge_01)
#define MAX_CFG_SMP_AVE             (kMAX_SMP_AVE_4)

/* Sample time to input to alg */
#define MAX_CFG_SAMPLING_TIME       (3U)

#define MAX_CFG_BIT_VAL(x, pos)     ((x >> (pos - 1)) & 0x1)
#define MAX_CFG_HRSPO2MODE_2_CHANNELS(x)    (MAX_CFG_BIT_VAL(x, 1) + MAX_CFG_BIT_VAL(x, 2) + MAX_CFG_BIT_VAL(x, 3))
/* Warning: The conversion is ok when MAX_CFG_SAMPLERATE < kMAX_SR_1000Hz */
#define MAX_CFG_SR_2_SMP_PER_SEC(x) (50 * (1 << x))

/* As we don't need to read hr and spo2 results very frequently, here we try to calculate irq numbers in one sec. */
/* irqs in one sec = samples_in_one_sec / 31 samples, suppose we read 31 samples per sec. */
#define MAX_CFG_IRQ_NUM_IN_ONE_SEC  (MAX_CFG_SR_2_SMP_PER_SEC(MAX_CFG_SAMPLERATE) >> 5)

/* Total samples needed for a one-shot calculation of HR or SpO2
 Total samples = channels * sample_rate * sample_time * bytes_per_sample
 */
#define MAX_CFG_TOTAL_SAMPLES               ((MAX_CFG_HRSPO2MODE_2_CHANNELS(MAX_CFG_MODE) * MAX_CFG_SR_2_SMP_PER_SEC(MAX_CFG_SAMPLERATE) >> MAX_CFG_SMP_AVE) * MAX_CFG_SAMPLING_TIME)
#define MAX_CFG_TOTAL_SAMPLES_BYTES         (MAX_CFG_TOTAL_SAMPLES * MAXIM_BYTES_PER_ADC_VALUE)

#endif /* _MAX_CFG_H_ */
