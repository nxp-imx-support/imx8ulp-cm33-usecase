/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Sensor application shource */
#include "sensor_app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
int16_t package_data[8] = {0};
bool isThereAccel = false;
bool isThereBaro = false;
uint8_t g_whoami_reg_addr[SENSOR_MODEL_NUMBERS] = {FXOS8700_WHOAMI_REG, LSM6DSO_WHOAMI_REG};
uint8_t b_whoami_reg_addr = MPL3115_WHOAMI_REG;

/*
 * device address:
 * FXOS8700(0x1c, 0x1d, 0x1e, 0x1f),
 * LSM6SDO(0x6a, 0x6b).
 */
const uint8_t g_accel_address[SENSOR_MODEL_NUMBERS][4] = {
    {0x1CU, 0x1DU, 0x1EU, 0x1FU}, /* FXOS8700 */
    {0x6AU, 0x6BU, 0x00U, 0x00U}  /* LSM6SDO */
};

/*
 * device address:
 * MPL3115(0x60)
 */
const uint8_t b_baro_address = 0x60U; /* MPL3115 */

/* Each entry in a regWriteList is composed of: register address, value to write, bit-mask to apply to write */
regList_t FXOS8700InitSeq[] = {
    /* for FXOS8700 */
    /*  write 0000 0000 = 0x00 to accelerometer control register 1 */
    /*  standby */
    /*  [7-1] = 0000 000 */
    /*  [0]: active=0 */
    {FXOS8700_CTRL_REG1, 0x00, 0x01},
    /* write 0001 1111 = 0x1F to magnetometer control register 1 */
    /* [7]: m_acal=0: auto calibration disabled */
    /* [6]: m_rst=0: no one-shot magnetic reset */
    /* [5]: m_ost=0: no one-shot magnetic measurement */
    /* [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise */
    /* [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active */
    {FXOS8700_M_CTRL_REG1, 0x1f, 0x01},
    /* write 0010 0000 = 0x20 to magnetometer control register 2 */
    /* [7]: reserved */
    /* [6]: reserved */
    /* [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the accelerometer registers */
    /* [4]: m_maxmin_dis=0 to retain default min/max latching even though not used */
    /* [3]: m_maxmin_dis_ths=0 */
    /* [2]: m_maxmin_rst=0 */
    /* [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle */
    {FXOS8700_M_CTRL_REG2, 0x20, 0x01},
    /*  write 0000 0001= 0x01 to XYZ_DATA_CFG register */
    /*  [7]: reserved */
    /*  [6]: reserved */
    /*  [5]: reserved */
    /*  [4]: hpf_out=0 */
    /*  [3]: reserved */
    /*  [2]: reserved */
    /*  [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB */
    /*  databyte = 0x01; */
    {FXOS8700_XYZ_DATA_CFG, 0x01, 0x01},
    /*  write 0000 1101 = 0x0D to accelerometer control register 1 */
    /*  [7-6]: aslp_rate=00 */
    /*  [5-3]: dr=001 for 200Hz data rate (when in hybrid mode) */
    /*  [2]: lnoise=1 for low noise mode */
    /*  [1]: f_read=0 for normal 16 bit reads */
    /*  [0]: active=1 to take the part out of standby and enable sampling */
    /*   databyte = 0x0D; */
    {FXOS8700_CTRL_REG1, 0x0d, 0x01},
};

regList_t LSM6DSOInitSeq[] = {
    /* for LSM6DSO */
    /*  write 0000 0001 = 0x01 to CTRL3_C(0x12) */
    /*  software reset */
    {0x12, 0x01, 0x01},

    /*  write 0000 1000 = 0x08 to CTRL4_C(0x13) */
    /*   CTRL4_C[3] = 1,  enable data available */
    {0x13, 0x08, 0x00},

    /*  write 1011 0000 = 0xb0 to CTRL1_XL(0x10) */
    /*   CTRL1_XL[7:4] = 1011,  12.5 Hz(high performance) */
    {0x10, 0xb0, 0x01},
};

regList_t MPL3115InitSeq[] = {
    /* for MPL3115 */
    /* Set to Barometer with an OSR = 0 */
    {0x26, 0x38, 0x01},

    /* Enable Data Flags in PT_DATA_CFG */
    {0x13, 0x07, 0x01},

    /* Set INT to Active Low Open Drain */
    {0x28, 0x11, 0x01},

   /* Enable DRDY Interrupt */
    {0x29, 0x80, 0x01},

    /* Set Active */
    {0x26, 0x39, 0x01},
};

/*  SENSOR_MODEL_NUMBERS * READ_SEQ_COMMAND_NUMBERS */
regList_t g_readSeq[] = {
    /* for FXOS8700 */
    /* read status register */
    {0x00, 0xff, 0x01}, /* READ_STATUS */
    /* read acceleration value from registers */
    {0x01, 0x00, 0x0c}, /* READ_ACCEL_DATA */

    /* for LSM6DSO */
    /* read STATUS_REG(0x1E) */
    {0x1e, 0x05, 0x01}, /* READ_STATUS */
    /* read acceleration value from OUTX_L_A, OUTX_H_A, OUTY_L_A, OUTY_H_A, OUTZ_L_A, OUTZ_H_A */
    {0x28, 0x00, 0x06}, /* READ_ACCEL_DATA */
};

regList_t b_readSeq[] = {
    /* for MPL3115 */
    /* read status register */
    {0x06, 0x0e, 0x01}, /* READ_STATUS */
    /* read pressure and temperature value from registers */
    {0x01, 0x00, 0x05}, /* READ_BARO_DATA */
};

lpi2c_master_handle_t m_handle;

uint8_t g_accel_addr_found = 0x00U;
uint8_t g_model            = SENSOR_MODEL_NUMBERS;

uint8_t b_baro_addr_found = 0x00U;

volatile bool completionFlag = false;
volatile bool nakFlag        = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    rgpio_pin_config_t pin_config;

    pin_config.pinDirection = kRGPIO_DigitalOutput;
    pin_config.outputLogic  = 1U;

    /* Initialize PTA8/PTA9 as GPIO */
    IOMUXC_SetPinMux(IOMUXC_PTA8_PTA8, 0);
    IOMUXC_SetPinMux(IOMUXC_PTA9_PTA9, 0);
    IOMUXC_SetPinConfig(IOMUXC_PTA8_PTA8, IOMUXC_PCR_OBE_MASK);
    IOMUXC_SetPinConfig(IOMUXC_PTA9_PTA9, IOMUXC_PCR_OBE_MASK);

    CLOCK_EnableClock(kCLOCK_RgpioA);

    RGPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    RGPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    RGPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        RGPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        RGPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        RGPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    RGPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    RGPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    RGPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    RGPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}

void lpi2c_master_callback(LPI2C_Type *base, lpi2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if (status == kStatus_LPI2C_Nak)
    {
        nakFlag = true;
    }
}

bool LPI2C_ReadAccelWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */
    uint8_t who_am_i_value         = 0x00;
    uint8_t device_addr_array_size = 0x00;
    bool result                    = false;
    uint8_t i                      = 0U;
    uint8_t model                  = 0;
    uint8_t device_addr_offset     = 0;
    status_t reVal                 = kStatus_Fail;

    lpi2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress   = g_accel_address[0][0];
    masterXfer.direction      = kLPI2C_Read;
    masterXfer.subaddress     = g_whoami_reg_addr[0];
    masterXfer.subaddressSize = 1;
    masterXfer.data           = &who_am_i_value;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kLPI2C_TransferDefaultFlag;

    device_addr_array_size = sizeof(g_accel_address) / sizeof(g_accel_address[0][0]);

    for (i = 0; i < device_addr_array_size; i++)
    {
        model                   = i / sizeof(g_accel_address[0]);
        device_addr_offset      = i % sizeof(g_accel_address[0]);
        masterXfer.slaveAddress = g_accel_address[model][device_addr_offset];
        masterXfer.subaddress   = g_whoami_reg_addr[model];

        reVal = LPI2C_MasterTransferNonBlocking(BOARD_SENSOR_I2C_BASEADDR, &m_handle, &masterXfer);
        if (reVal != kStatus_Success)
        {
            continue;
        }
        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            g_accel_addr_found = masterXfer.slaveAddress;
            g_model            = model;
            break;
        }

        /* Wait to make sure the bus is idle. */
        while ((LPI2C_MasterGetStatusFlags(BOARD_SENSOR_I2C_BASEADDR) & (uint32_t)kLPI2C_MasterBusBusyFlag) != 0U)
        {
        }
    }

    if (completionFlag)
    {
        completionFlag = false;
        if (who_am_i_value == FXOS8700_WHOAMI)
        {
            PRINTF("Found an FXOS8700 on board , the device address is 0x%x. \r\n", masterXfer.slaveAddress);
            result = true;
        }
        else if (who_am_i_value == LSM6DSO_WHOAMI)
        {
            PRINTF("Found a LSDM6DSO on board, the device address is 0x%02X. \r\n", masterXfer.slaveAddress);
            result = true;
        }
        else
        {
            PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
            PRINTF("It's not FXOS8700 or LSM6DSO. \r\n");
            PRINTF("The device address is 0x%x. \r\n", masterXfer.slaveAddress);
            result = false;
        }
    }
    else
    {
        PRINTF("\r\n Do not find an accelerometer device ! \r\n");
        result = false;
    }
    return result;
}

bool LPI2C_ReadBaroWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */
    uint8_t who_am_i_value         = 0x00;
    uint8_t device_addr_array_size = 0x00;
    bool result                    = false;
    uint8_t i                      = 0U;
    status_t reVal                 = kStatus_Fail;

    lpi2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress   = b_baro_address;
    masterXfer.direction      = kLPI2C_Read;
    masterXfer.subaddress     = b_whoami_reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = &who_am_i_value;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kLPI2C_TransferDefaultFlag;

    device_addr_array_size = 1;

    for (i = 0; i < device_addr_array_size; i++)
    {
        masterXfer.slaveAddress = b_baro_address;
        masterXfer.subaddress   = b_whoami_reg_addr;

        reVal = LPI2C_MasterTransferNonBlocking(BOARD_SENSOR_I2C_BASEADDR, &m_handle, &masterXfer);
        if (reVal != kStatus_Success)
        {
            continue;
        }
        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            b_baro_addr_found = masterXfer.slaveAddress;
            break;
        }

        /* Wait to make sure the bus is idle. */
        while ((LPI2C_MasterGetStatusFlags(BOARD_SENSOR_I2C_BASEADDR) & (uint32_t)kLPI2C_MasterBusBusyFlag) != 0U)
        {
        }
    }

    if (completionFlag)
    {
        completionFlag = false;
        if (who_am_i_value == MPL3115_WHOAMI)
        {
            PRINTF("Found an MPL3115 on board , the device address is 0x%x. \r\n", masterXfer.slaveAddress);
            result = true;
        }
        else
        {
            PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
            PRINTF("It's not MPL3115. \r\n");
            PRINTF("The device address is 0x%x. \r\n", masterXfer.slaveAddress);
            result = false;
        }
    }
    else
    {
        PRINTF("\r\n Do not find a barometer device ! \r\n");
        result = false;
    }
    return result;
}

bool LPI2C_WriteSensorReg(LPI2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    lpi2c_master_transfer_t masterXfer;
    status_t reVal = kStatus_Fail;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress   = device_addr;
    masterXfer.direction      = kLPI2C_Write;
    masterXfer.subaddress     = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = &value;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kLPI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    reVal = LPI2C_MasterTransferNonBlocking(BOARD_SENSOR_I2C_BASEADDR, &m_handle, &masterXfer);
    if (reVal != kStatus_Success)
    {
        return false;
    }

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool LPI2C_ReadSensorRegs(
    LPI2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    lpi2c_master_transfer_t masterXfer;
    status_t reVal = kStatus_Fail;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = device_addr;
    masterXfer.direction      = kLPI2C_Read;
    masterXfer.subaddress     = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = rxBuff;
    masterXfer.dataSize       = rxSize;
    masterXfer.flags          = kLPI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    reVal = LPI2C_MasterTransferNonBlocking(BOARD_SENSOR_I2C_BASEADDR, &m_handle, &masterXfer);
    if (reVal != kStatus_Success)
    {
        return false;
    }
    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

void LPI2C_InitAccel(uint8_t model)
{
    int8_t commandNums  = 0;
    regList_t *pRegList = NULL;
    int8_t i            = 0;

    if (model == FXOS8700)
    {
        commandNums = sizeof(FXOS8700InitSeq) / sizeof(FXOS8700InitSeq[0]);
        pRegList    = FXOS8700InitSeq;
    }
    else if (model == LSM6DSO)
    {
//        commandNums = sizeof(LSM6DSOInitSeq) / sizeof(LSM6DSOInitSeq[0]);
//        pRegList    = LSM6DSOInitSeq;
    }
    else
    {
        PRINTF("\r\n Failed to initialize accelerometer\r\n");
        return;
    }

    for (i = 0; i < commandNums; i++)
    {
        LPI2C_WriteSensorReg(BOARD_SENSOR_I2C_BASEADDR, g_accel_addr_found, pRegList[i].reg, pRegList[i].val);
    }
}

void LPI2C_InitBaro()
{
    int8_t commandNums  = 0;
    regList_t *pRegList = NULL;
    int8_t i            = 0;

    commandNums = sizeof(MPL3115InitSeq) / sizeof(MPL3115InitSeq[0]);
    pRegList    = MPL3115InitSeq;

    for (i = 0; i < commandNums; i++)
    {
        LPI2C_WriteSensorReg(BOARD_SENSOR_I2C_BASEADDR, b_baro_addr_found, pRegList[i].reg, pRegList[i].val);
    }
}

bool Sensor_Init()
{
    lpi2c_master_config_t masterConfig;

    //BOARD_I2C_ReleaseBus();
    //BOARD_I2C_ConfigurePins();

    CLOCK_SetIpSrc(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcLpo); /* Choose LPO1MHz */
    RESET_PeripheralReset(kRESET_Lpi2c0);

    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
     * masterConfig.baudRate_Hz = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Hz = I2C_BAUDRATE;

    LPI2C_MasterInit(BOARD_SENSOR_I2C_BASEADDR, &masterConfig, LPI2C_CLOCK_FREQUENCY);
    LPI2C_MasterTransferCreateHandle(BOARD_SENSOR_I2C_BASEADDR, &m_handle, lpi2c_master_callback, NULL);
    isThereAccel = LPI2C_ReadAccelWhoAmI();
    isThereBaro = LPI2C_ReadBaroWhoAmI();
    return isThereAccel && isThereBaro;
}

int16_t* Sensor_ReadData(void)
{
    /*  read the accel xyz value if there is accel device on board */
    if (true == isThereAccel)
    {
        uint8_t readBuff[12] = {0};
        int16_t ax, ay, az;
        int16_t mx, my, mz;
        uint8_t status0_value = 0;
        uint32_t i            = 0U;
        bool reTrans          = false;

        LPI2C_InitAccel(g_model);

        //PRINTF("The accel values:\r\n");
        for (i = 0; i < ACCEL_READ_TIMES; i++)
        {
            status0_value = 0;
            /*  wait for new data are ready. */
            while (status0_value != g_readSeq[g_model * 2 + READ_STATUS].val)
            {
                reTrans = LPI2C_ReadSensorRegs(BOARD_SENSOR_I2C_BASEADDR, g_accel_addr_found,
                                              g_readSeq[g_model * 2 + READ_STATUS].reg, &status0_value,
                                              g_readSeq[g_model * 2 + READ_STATUS].size);
                if (reTrans == false)
                {
                    PRINTF("F4\n");
                    //return -1;
                }
                else
                {
                }
            }

            /*  Multiple-byte Read from acceleration registers */
            reTrans = LPI2C_ReadSensorRegs(BOARD_SENSOR_I2C_BASEADDR, g_accel_addr_found,
                                          g_readSeq[g_model * 2 + READ_DATA].reg, readBuff,
                                          g_readSeq[g_model * 2 + READ_DATA].size);
            if (reTrans == false)
            {
                PRINTF("F5\n");
                //return -1;
            }

            ax = ((int16_t)(((readBuff[0] << 8U) | readBuff[1]))) >> ((g_model == LSM6DSO) ? (0U) : (2U));
            ay = ((int16_t)(((readBuff[2] << 8U) | readBuff[3]))) >> ((g_model == LSM6DSO) ? (0U) : (2U));
            az = ((int16_t)(((readBuff[4] << 8U) | readBuff[5]))) >> ((g_model == LSM6DSO) ? (0U) : (2U));

            //PRINTF("status_reg = 0x%x , ax = %5d , ay = %5d , az = %5d \r\n", status0_value, ax, ay, az);
            package_data[0] = ax;
            package_data[1] = ay;
            package_data[2] = az;

            /* Add magnetometer data read */
            if(g_model == FXOS8700)
            {
                mx = (readBuff[6] << 8) | readBuff[7];
                my = (readBuff[8] << 8) | readBuff[9];
                mz = (readBuff[10] << 8) | readBuff[11];

                //PRINTF("                    mx = %5d , my = %5d , mz = %5d \r\n", mx, my, mz);
                package_data[3] = mx;
                package_data[4] = my;
                package_data[5] = mz;
            }
            else
            {
                //FXOS8700CQ is DNP due to EoL, LSM6SDO has no magnetometer data
                package_data[3] = 0;
                package_data[4] = 0;
                package_data[5] = 0;
            }
        }
    }

        /*  read the baro pressure and temperature value if there is barometer device on board */
    if (true == isThereBaro)
    {
        uint8_t readBuff[5] = {0};
        int32_t p;
        int16_t t;
        uint8_t status0_value = 0;
        uint32_t i            = 0U;
        bool reTrans          = false;

        LPI2C_InitBaro();

        //PRINTF("The baro values:\r\n");
        for (i = 0; i < BARO_READ_TIMES; i++)
        {
            status0_value = 0;
            /*  wait for new data are ready. */
            while (status0_value != b_readSeq[READ_STATUS].val)
            {
                reTrans = LPI2C_ReadSensorRegs(BOARD_SENSOR_I2C_BASEADDR, b_baro_addr_found,
                                              b_readSeq[READ_STATUS].reg, &status0_value,
                                              b_readSeq[READ_STATUS].size);
                if (reTrans == false)
                {
                    PRINTF("F6\n");
                    //return -1;
                }
                else
                {
                }
            }

            /*  Multiple-byte Read from acceleration registers */
            reTrans = LPI2C_ReadSensorRegs(BOARD_SENSOR_I2C_BASEADDR, b_baro_addr_found,
                                          b_readSeq[READ_DATA].reg, readBuff,
                                          b_readSeq[READ_DATA].size);
            if (reTrans == false)
            {
                PRINTF("F7\n");
                //return -1;
            }

            /* Bits 5 to 4 of OUT_P_LSB contain the fractional component */
            p = readBuff[0] << 10U | readBuff[1] << 2 | readBuff[2] >> 2;
            /* readBuff[4] is value in OUT_T_LSB (05h) register contains the fractional part */
            t = readBuff[3];

            //PRINTF("status_reg = 0x%x , p = %5d , t = %5d \r\n", status0_value, p, t);
            package_data[6] = (int16_t)(p / 1000);
            package_data[7] = t;
        }
    }

    return package_data;
}