/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_


/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitLpuartPins(void);                           /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitLpi2cPins(void);                            /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPmicI2cPins(void);                          /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitTouchPanelPins(void);                       /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPmicModePins(void);                         /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitButtonPins(void);                           /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitSensorsPins(void);                          /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitGnssPins(void);                             /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitNfcPins(void);                              /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitMipiPanelPins(void);                        /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitChargerPins(void);                          /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitMotorPins(void);                            /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitMicPins(void);                              /*!< Function assigned for the core: Cortex-M33[cm33] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitSpeakerPins(void);                          /*!< Function assigned for the core: Cortex-M33[cm33] */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
