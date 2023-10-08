/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Sensor application header */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_lpi2c.h"

#include "fsl_rgpio.h"
#include "fsl_iomuxc.h"
#include "fsl_reset.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LPI2C_CLOCK_FREQUENCY           CLOCK_GetIpFreq(kCLOCK_Lpi2c0)
#define BOARD_SENSOR_I2C_BASEADDR       LPI2C0

#define I2C_RELEASE_SDA_GPIO            GPIOA
#define I2C_RELEASE_SDA_PIN             9U
#define I2C_RELEASE_SCL_GPIO            GPIOA
#define I2C_RELEASE_SCL_PIN             8U
#define I2C_RELEASE_BUS_COUNT           100U
#define I2C_BAUDRATE                    100000U
#define FXOS8700_WHOAMI                 0xC7U
#define LSM6DSO_WHOAMI                  0x6CU
#define FXOS8700_STATUS                 0x00U
#define FXOS8700_XYZ_DATA_CFG           0x0EU
#define FXOS8700_CTRL_REG1              0x2AU
#define FXOS8700_M_CTRL_REG1            0x5BU
#define FXOS8700_M_CTRL_REG2            0x5CU
#define FXOS8700_WHOAMI_REG             0x0DU
#define LSM6DSO_WHOAMI_REG              0x0FU
#define ACCEL_READ_TIMES                1U
#define SENSOR_MODEL_NUMBERS            2U
#define READ_SEQ_COMMAND_NUMBERS        2U
#define MPL3115_STATUS                  0x00U
#define MPL3115_WHOAMI                  0xC4U
#define MPL3115_WHOAMI_REG              0x0CU
#define BARO_READ_TIMES                 1U

typedef enum _sensor_model
{
    FXOS8700 = 0U,
    LSM6DSO  = 1U,
} sensor_model;

typedef enum _read_seq_command
{
    READ_STATUS = 0U,
    READ_DATA   = 1U,
} read_seq_command;

/*!
 * @brief This structure defines the Write command List.
 */
typedef struct _regList
{
    uint8_t reg; /* Register Address where the value is wrote to */
    uint8_t val;
    uint8_t size; /* read size from register */
} regList_t;

/*******************************************************************************
 * API
 ******************************************************************************/
void i2c_release_bus_delay(void);

void BOARD_I2C_ReleaseBus(void);

void lpi2c_master_callback(LPI2C_Type *base, lpi2c_master_handle_t *handle, status_t status, void *userData);

bool LPI2C_ReadAccelWhoAmI(void);

bool LPI2C_ReadBaroWhoAmI(void);

bool LPI2C_WriteSensorReg(LPI2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);

bool LPI2C_ReadSensorRegs(
    LPI2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);

void LPI2C_InitSensor(uint8_t model);

void LPI2C_InitBaro();

bool Sensor_Init();

int16_t* Sensor_ReadData(void);