/*
 * Copyright (c) 2022 ITE.
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MIPIRX_H_
#define _MIPIRX_H_

#define MIPI_RX_SOFT_RESET_REG05                     (0x05U)
#define MIPI_RX_SOFT_RESET_REG05_RegSoftORst_SHIFT   (0x00U)
#define MIPI_RX_SOFT_RESET_REG05_RegSoftORst_MASK    (MIPI_RX_SOFT_RESET_REG05_RegSoftORst(0x01))
#define MIPI_RX_SOFT_RESET_REG05_RegSoftORst(N)      ((N) << MIPI_RX_SOFT_RESET_REG05_RegSoftORst_SHIFT)
#define MIPI_RX_SOFT_RESET_REG05_RegSoftMRst_SHIFT   (0x01U)
#define MIPI_RX_SOFT_RESET_REG05_RegSoftMRst_MASK    (MIPI_RX_SOFT_RESET_REG05_RegSoftMRst(0x01))
#define MIPI_RX_SOFT_RESET_REG05_RegSoftMRst(N)      ((N) << MIPI_RX_SOFT_RESET_REG05_RegSoftMRst_SHIFT)
#define MIPI_RX_SOFT_RESET_REG05_RegMPSoftPRst_SHIFT (0x02U)
#define MIPI_RX_SOFT_RESET_REG05_RegMPSoftPRst_MASK  (MIPI_RX_SOFT_RESET_REG05_RegMPSoftPRst(0x01))
#define MIPI_RX_SOFT_RESET_REG05_RegMPSoftPRst(N)    ((N) << MIPI_RX_SOFT_RESET_REG05_RegMPSoftPRst_SHIFT)
#define MIPI_RX_SOFT_RESET_REG05_RefSoftREFRst_SHIFT (0x03U)
#define MIPI_RX_SOFT_RESET_REG05_RefSoftREFRst_MASK  (MIPI_RX_SOFT_RESET_REG05_RefSoftREFRst(0x01))
#define MIPI_RX_SOFT_RESET_REG05_RefSoftREFRst(N)    ((N) << MIPI_RX_SOFT_RESET_REG05_RefSoftREFRst_SHIFT)

#define MIPI_RX_INT_MASK_REG09                     (0x09U)
#define MIPI_RX_INT_MASK_REG09_REnPPSMVidStbChgInt (1U << 0)
#define MIPI_RX_INT_MASK_REG09_REnPPSPVidStbChgInt (1U << 4)

#define MIPI_RX_INT_MASK_REG0A                    (0x0AU)
#define MIPI_RX_INT_MASK_REG0A_REnPPGVidStbChgInt (1U << 0)
#define MIPI_RX_INT_MASK_REG0A_REnPPSDByteErrInt  (1U << 1)
#define MIPI_RX_INT_MASK_REG0A_REnCMOffInt        (1U << 2)
#define MIPI_RX_INT_MASK_REG0A_REnCMOnInt         (1U << 3)
#define MIPI_RX_INT_MASK_REG0A_REnShutDoneInt     (1U << 4)
#define MIPI_RX_INT_MASK_REG0A_REnTurnOnInt       (1U << 5)
#define MIPI_RX_INT_MASK_REG0A_REnFIFOOvRdInt     (1U << 6)
#define MIPI_RX_INT_MASK_REG0A_REnFIFOOvWrInt     (1U << 7)

#define MIPI_RX_INT_MASK_REG0B                   (0x0BU)
#define MIPI_RX_INT_MASK_REG0B_REnECC1bErrInt    (1U << 0)
#define MIPI_RX_INT_MASK_REG0B_REnECC2bErrInt    (1U << 1)
#define MIPI_RX_INT_MASK_REG0B_REnLMFIFOErrInt   (1U << 2)
#define MIPI_RX_INT_MASK_REG0B_REnCRCErrInt      (1U << 3)
#define MIPI_RX_INT_MASK_REG0B_REnMCLKOffInt     (1U << 4)
#define MIPI_RX_INT_MASK_REG0B_REnPPIFifoOvWrInt (1U << 5)
#define MIPI_RX_INT_MASK_REG0B_REnTimerInt       (1U << 6)

#define MIPI_RX_SYS_CONF_REG0C                     (0x0CU)
#define MIPI_RX_SYS_CONF_REG0C_RegLaneNum_SHIFT    (0x0U)
#define MIPI_RX_SYS_CONF_REG0C_RegLaneNum_MASK     (0x3U)
#define MIPI_RX_SYS_CONF_REG0C_RegLaneNum(N)       ((N - 1) << MIPI_RX_SYS_CONF_REG0C_RegLaneNum_SHIFT)
#define MIPI_RX_SYS_CONF_REG0C_RegEnPNSwap_SHIFT   (0x2U)
#define MIPI_RX_SYS_CONF_REG0C_RegEnPNSwap_MASK    (0x1U << 2U)
#define MIPI_RX_SYS_CONF_REG0C_RegEnPNSwap(N)      ((N) << MIPI_RX_SYS_CONF_REG0C_RegEnPNSwap_SHIFT)
#define MIPI_RX_SYS_CONF_REG0C_RegEnLaneSwap_SHIFT (0x3U)
#define MIPI_RX_SYS_CONF_REG0C_RegEnLaneSwap_MASK  (0x1U << 3U)
#define MIPI_RX_SYS_CONF_REG0C_RegEnLaneSwap(N)    ((N) << MIPI_RX_SYS_CONF_REG0C_RegEnLaneSwap_SHIFT)

#define MIPI_RX_SYS_CONF_REG0D                 (0x0DU)
#define MIPI_RX_SYS_CONF_REG0D_REGINTPOL_SHIFT (0x01U)
#define MIPI_RX_SYS_CONF_REG0D_REGINTPOL_MASK  (MIPI_RX_SYS_CONF_REG0D_REGINTPOL(0x01U))
#define MIPI_RX_SYS_CONF_REG0D_REGINTPOL(N)    ((N) << MIPI_RX_SYS_CONF_REG0D_REGINTPOL_SHIFT)

#define MIPI_RX_SYS_STATUS_REG0F (0x0FU)

#define MIPI_RX_CLKBUF_CTRL_REG10                   (0x10U)
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGateOCLK_SHIFT (0x0U)
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGateOCLK_MASK  (MIPI_RX_CLKBUF_CTRL_REG10_RegGateOCLK(0x01U))
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGateOCLK(N)    ((N) << MIPI_RX_CLKBUF_CTRL_REG10_RegGateOCLK_SHIFT)
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGateMCLK_SHIFT (0x01U)
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGateMCLK_MASK  (MIPI_RX_CLKBUF_CTRL_REG10_RegGateMCLK(0x01U))
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGateMCLK(N)    ((N) << MIPI_RX_CLKBUF_CTRL_REG10_RegGateMCLK_SHIFT)
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGatePCLK_SHIFT (0x02U)
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGatePCLK_MASK  (MIPI_RX_CLKBUF_CTRL_REG10_RegGatePCLK(0x01U))
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGatePCLK(N)    ((N) << MIPI_RX_CLKBUF_CTRL_REG10_RegGatePCLK_SHIFT)
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGateRCLK_SHIFT (0x03U)
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGateRCLK_MASK  (MIPI_RX_CLKBUF_CTRL_REG10_RegGateRCLK(0x01U))
#define MIPI_RX_CLKBUF_CTRL_REG10_RegGateRCLK(N)    ((N) << MIPI_RX_CLKBUF_CTRL_REG10_RegGateRCLK_SHIFT)

#define MIPI_RX_CLKBUF_CTRL_REG11                    (0x11U)
#define MIPI_RX_CLKBUF_CTRL_REG11_RegInvMCLK_SHIFT   (0x0U)
#define MIPI_RX_CLKBUF_CTRL_REG11_RegInvMCLK_MASK    (MIPI_RX_CLKBUF_CTRL_REG11_RegInvMCLK(0x01U))
#define MIPI_RX_CLKBUF_CTRL_REG11_RegInvMCLK(N)      ((N) << MIPI_RX_CLKBUF_CTRL_REG11_RegInvMCLK_SHIFT)
#define MIPI_RX_CLKBUF_CTRL_REG11_RegInvPCLK_SHIFT   (0x1U)
#define MIPI_RX_CLKBUF_CTRL_REG11_RegInvPCLK_MASK    (MIPI_RX_CLKBUF_CTRL_REG11_RegInvPCLK(0x01U))
#define MIPI_RX_CLKBUF_CTRL_REG11_RegInvPCLK(N)      ((N) << MIPI_RX_CLKBUF_CTRL_REG11_RegInvPCLK_SHIFT)
#define MIPI_RX_CLKBUF_CTRL_REG11_RegEnStandby_SHIFT (0x2U)
#define MIPI_RX_CLKBUF_CTRL_REG11_RegEnStandby_MASK  (MIPI_RX_CLKBUF_CTRL_REG11_RegEnStandby(0x01U))
#define MIPI_RX_CLKBUF_CTRL_REG11_RegEnStandby(N)    ((N) << MIPI_RX_CLKBUF_CTRL_REG11_RegEnStandby_SHIFT)
#define MIPI_RX_CLKBUF_CTRL_REG11_RegEnStb2Rst_SHIFT (0x4U)
#define MIPI_RX_CLKBUF_CTRL_REG11_RegEnStb2Rst_MASK  (MIPI_RX_CLKBUF_CTRL_REG11_RegEnStb2Rst(0x01U))
#define MIPI_RX_CLKBUF_CTRL_REG11_RegEnStb2Rst(N)    ((N) << MIPI_RX_CLKBUF_CTRL_REG11_RegEnStb2Rst_SHIFT)
#define MIPI_RX_CLKBUF_CTRL_REG11_RegEnIDDQ_SHIFT    (0x5U)
#define MIPI_RX_CLKBUF_CTRL_REG11_RegEnIDDQ_MASK     (MIPI_RX_CLKBUF_CTRL_REG11_RegEnIDDQ(0x01U))
#define MIPI_RX_CLKBUF_CTRL_REG11_RegEnIDDQ(N)       ((N) << MIPI_RX_CLKBUF_CTRL_REG11_RegEnIDDQ_SHIFT)

#define MIPI_RX_CLKBUF_CTRL_REG12                   (0x12U)
#define MIPI_RX_CLKBUF_CTRL_REG12_RegPDREFCLK_SHIFT (0x00U)
#define MIPI_RX_CLKBUF_CTRL_REG12_RegPDREFCLK_MASK  (MIPI_RX_CLKBUF_CTRL_REG12_RegPDREFCLK(0x01U))
#define MIPI_RX_CLKBUF_CTRL_REG12_RegPDREFCLK(N)    ((N) << MIPI_RX_CLKBUF_CTRL_REG12_RegPDREFCLK_SHIFT)
#define MIPI_RX_CLKBUF_CTRL_REG12_RegPDREFCNT_SHIFT (0x01U)
#define MIPI_RX_CLKBUF_CTRL_REG12_RegPDREFCNT_MASK  (MIPI_RX_CLKBUF_CTRL_REG12_RegPDREFCNT(0x03U))
#define MIPI_RX_CLKBUF_CTRL_REG12_RegPDREFCNT(N)    ((N) << MIPI_RX_CLKBUF_CTRL_REG12_RegPDREFCNT_SHIFT)

/* PPI: PHY-Protocol Interface */
#define MIPI_RX_PPI_REG18                    (0x18U)
#define MIPI_RX_PPI_REG18_RegHSSetNum_SHIFT  (0x00U)
#define MIPI_RX_PPI_REG18_RegHSSetNum_MASK   (MIPI_RX_PPI_REG18_RegHSSetNum(0x07U))
#define MIPI_RX_PPI_REG18_RegHSSetNum(N)     ((N) << MIPI_RX_PPI_REG18_RegHSSetNum_SHIFT)
#define MIPI_RX_PPI_REG18_RegSkipStg_SHIFT   (0x04U)
#define MIPI_RX_PPI_REG18_RegSkipStg_MASK    (MIPI_RX_PPI_REG18_RegSkipStg(0x07U))
#define MIPI_RX_PPI_REG18_RegSkipStg(N)      ((N) << MIPI_RX_PPI_REG18_RegSkipStg_SHIFT)
#define MIPI_RX_PPI_REG18_RegEnSyncErr_SHIFT (0x07U)
#define MIPI_RX_PPI_REG18_RegEnSyncErr_MASK  (MIPI_RX_PPI_REG18_RegEnSyncErr(0x01U))
#define MIPI_RX_PPI_REG18_RegEnSyncErr(N)    ((N) << MIPI_RX_PPI_REG18_RegEnSyncErr_SHIFT)

#define MIPI_RX_PPI_REG19                    (0x19U)
#define MIPI_RX_PPI_REG19_RegEnDeSkew_SHIFT  (0x00U)
#define MIPI_RX_PPI_REG19_RegEnDeSkew_MASK   (MIPI_RX_PPI_REG19_RegEnDeSkew(0x01U))
#define MIPI_RX_PPI_REG19_RegEnDeSkew(N)     ((N) << MIPI_RX_PPI_REG19_RegEnDeSkew_SHIFT)
#define MIPI_RX_PPI_REG19_RegEnContCK_SHIFT  (0x01U)
#define MIPI_RX_PPI_REG19_RegEnContCK_MASK   (MIPI_RX_PPI_REG19_RegEnContCK(0x01U))
#define MIPI_RX_PPI_REG19_RegEnContCK(N)     ((N) << MIPI_RX_PPI_REG19_RegEnContCK_SHIFT)
#define MIPI_RX_PPI_REG19_RegPPIDbgSel_SHIFT (0x04U)
#define MIPI_RX_PPI_REG19_RegPPIDbgSel_MASK  (MIPI_RX_PPI_REG19_RegPPIDbgSel(0x0FU))
#define MIPI_RX_PPI_REG19_RegPPIDbgSel(N)    ((N) << MIPI_RX_PPI_REG19_RegPPIDbgSel_SHIFT)

/* Lane Merge & Packet Decoder */
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20                   (0x20U)
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegIgnrNull_SHIFT (0x00U)
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegIgnrNull_MASK \
    (MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegIgnrNull(0x01U))
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegIgnrNull(N) \
    ((N) << MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegIgnrNull_SHIFT)
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegIgnrBlk_SHIFT (0x01U)
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegIgnrBlk_MASK \
    (MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegIgnrBlk(0x01U))
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegIgnrBlk(N) \
    ((N) << MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegIgnrBlk_SHIFT)
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegEnDummyECC_SHIFT (0x02U)
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegEnDummyECC_MASK \
    (MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegEnDummyECC(0x01U))
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegEnDummyECC(N) \
    ((N) << MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegEnDummyECC_SHIFT)
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegSelEOTP_SHIFT (0x02U)
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegSelEOTP_MASK \
    (MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegSelEOTP(0x01U))
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegSelEOTP(N) \
    ((N) << MIPI_RX_LANE_MERGE_PACKET_DECODER_REG20_RegSelEOTP_SHIFT)

#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG21                   (0x21U)
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG21_RegSelLMDbg_SHIFT (0x00U)
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG21_RegSelLMDbg_MASK \
    (MIPI_RX_LANE_MERGE_PACKET_DECODER_REG21_RegSelLMDbg(0x07U))
#define MIPI_RX_LANE_MERGE_PACKET_DECODER_REG21_RegSelLMDbg(N) \
    ((N) << MIPI_RX_LANE_MERGE_PACKET_DECODER_REG21_RegSelLMDbg_SHIFT)

/* Packed pixel stream, timing generator and pattern generation */
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG31                   (0x31U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG31_RegEnUsrHFP_SHIFT (0x07U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG31_RegEnUsrHFP_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG31_RegEnUsrHFP(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG31_RegEnUsrHFP(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG31_RegEnUsrHFP_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG33                   (0x33U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG33_RegEnUsrHSW_SHIFT (0x07U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG33_RegEnUsrHSW_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG33_RegEnUsrHSW(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG33_RegEnUsrHSW(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG33_RegEnUsrHSW_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG35                   (0x35U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG35_RegEnUsrHBP_SHIFT (0x07U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG35_RegEnUsrHBP_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG35_RegEnUsrHBP(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG35_RegEnUsrHBP(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG35_RegEnUsrHBP_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG37                    (0x37U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG37_RegEnUsrHDEW_SHIFT (0x07U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG37_RegEnUsrHDEW_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG37_RegEnUsrHDEW(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG37_RegEnUsrHDEW(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG37_RegEnUsrHDEW_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG39                      (0x39U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG39_RegEnUsrHVR2nd_SHIFT (0x07U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG39_RegEnUsrHVR2nd_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG39_RegEnUsrHVR2nd(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG39_RegEnUsrHVR2nd(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG39_RegEnUsrHVR2nd_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3A                   (0x3AU)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3A_RegMipi_VFP_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3A_RegMipi_VFP_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3A_RegMipi_VFP(0xFFU))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3A_RegMipi_VFP(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3A_RegMipi_VFP_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3C                   (0x3CU)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3C_RegMipi_VSW_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3C_RegMipi_VSW_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3C_RegMipi_VSW(0xFFU))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3C_RegMipi_VSW(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3C_RegMipi_VSW_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3E                   (0x3EU)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3E_RegMipi_VBP_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3E_RegMipi_VBP_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3E_RegMipi_VBP(0xFFU))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3E_RegMipi_VBP(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG3E_RegMipi_VBP_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG41                    (0x41U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG41_RegEnUsrVDEW_SHIFT (0x07U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG41_RegEnUsrVDEW_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG41_RegEnUsrVDEW(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG41_RegEnUsrVDEW(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG41_RegEnUsrVDEW_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG43                      (0x43U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG43_RegEnUsrVFP2nd_SHIFT (0x07U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG43_RegEnUsrVFP2nd_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG43_RegEnUsrVFP2nd(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG43_RegEnUsrVFP2nd(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG43_RegEnUsrVFP2nd_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44                    (0x44U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegAutoSyncF_SHIFT (0x01U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegAutoSyncF_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegAutoSyncF(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegAutoSyncF(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegAutoSyncF_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegMipi_Interlaced_SHIFT (0x02U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegMipi_Interlaced_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegMipi_Interlaced(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegMipi_Interlaced(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegMipi_Interlaced_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegSEModeUdef_SHIFT (0x03U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegSEModeUdef_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegSEModeUdef(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegSEModeUdef(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegSEModeUdef_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegPRec_UPdate_SHIFT (0x04U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegPRec_UPdate_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegPRec_UPdate(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegPRec_UPdate(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegPRec_UPdate_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegMRec_UPdate_SHIFT (0x05U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegMRec_UPdate_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegMRec_UPdate(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegMRec_UPdate(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG44_RegMRec_UPdate_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B                   (0x4BU)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegVREnhSel_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegVREnhSel_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegVREnhSel(0x07U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegVREnhSel(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegVREnhSel_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegVREnh_SHIFT (0x03U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegVREnh_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegVREnh(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegVREnh(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegVREnh_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegFReSyncEn_SHIFT (0x04U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegFReSyncEn_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegFReSyncEn(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegFReSyncEn(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4B_RegFReSyncEn_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4C                  (0x4CU)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4C_RegFFRdStg_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4C_RegFFRdStg_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4C_RegFFRdStg(0xFFU))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4C_RegFFRdStg(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4C_RegFFRdStg_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4D                  (0x4DU)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4D_RegFFRdStg_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4D_RegFFRdStg_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4D_RegFFRdStg(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4D_RegFFRdStg(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4D_RegFFRdStg_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E                     (0x4EU)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMipi_HSPol_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMipi_HSPol_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMipi_HSPol(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMipi_HSPol(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMipi_HSPol_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMipi_VSPol_SHIFT (0x01U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMipi_VSPol_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMipi_VSPol(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMipi_VSPol(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMipi_VSPol_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegHReSyncEn_SHIFT (0x02U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegHReSyncEn_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegHReSyncEn(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegHReSyncEn(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegHReSyncEn_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegVReSyncEn_SHIFT (0x03U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegVReSyncEn_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegVReSyncEn(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegVReSyncEn(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegVReSyncEn_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForceMCLKOn_SHIFT (0x04U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForceMCLKOn_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForceMCLKOn(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForceMCLKOn(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForceMCLKOn_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMPFFRst_SHIFT (0x05U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMPFFRst_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMPFFRst(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMPFFRst(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegMPFFRst_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForcePPSPStb_SHIFT (0x06U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForcePPSPStb_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForcePPSPStb(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForcePPSPStb(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForcePPSPStb_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForcePPSMStb_SHIFT (0x07U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForcePPSMStb_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForcePPSMStb(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForcePPSMStb(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4E_RegForcePPSMStb_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F                       (0x4FU)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegPPSFFAutoRst_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegPPSFFAutoRst_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegPPSFFAutoRst(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegPPSFFAutoRst(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegPPSFFAutoRst_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegEnPPSFFOv2Rst_SHIFT (0x01U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegEnPPSFFOv2Rst_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegEnPPSFFOv2Rst(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegEnPPSFFOv2Rst(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegEnPPSFFOv2Rst_SHIFT)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegDBGPPSSel_SHIFT (0x04U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegDBGPPSSel_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegDBGPPSSel(0x07U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegDBGPPSSel(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG4F_RegDBGPPSSel_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG70                 (0x70U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG70_RegEnMAvg_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG70_RegEnMAvg_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG70_RegEnMAvg(0x01U))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG70_RegEnMAvg(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG70_RegEnMAvg_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG72                 (0x72U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG72_RegMShift_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG72_RegMShift_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG72_RegMShift(0xFFU))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG72_RegMShift(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG72_RegMShift_SHIFT)

#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG73                 (0x73U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG73_RegPShift_SHIFT (0x00U)
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG73_RegPShift_MASK \
    (MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG73_RegPShift(0xFFU))
#define MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG73_RegPShift(N) \
    ((N) << MIPI_RX_PACKED_PIXEL_STREAM_TIMING_GENERATOR_AND_PATTERN_GENERATION_REG73_RegPShift_SHIFT)

/* AFE: Analog Front End */
#define MIPI_RX_AFE_REG80                    (0x80U)
#define MIPI_RX_AFE_REG80_RegEnExtPCLK_SHIFT (0x05U)
#define MIPI_RX_AFE_REG80_RegEnExtPCLK_MASK  (MIPI_RX_AFE_REG80_RegEnExtPCLK(0x01U))
#define MIPI_RX_AFE_REG80_RegEnExtPCLK(N)    ((N) << MIPI_RX_AFE_REG80_RegEnExtPCLK_SHIFT)

#define MIPI_RX_AFE_REG84                (0x84U)
#define MIPI_RX_AFE_REG84_REGHSAMP_SHIFT (0x00U)
#define MIPI_RX_AFE_REG84_REGHSAMP_MASK  (MIPI_RX_AFE_REG84_REGHSAMP(0x0FU))
#define MIPI_RX_AFE_REG84_REGHSAMP(N)    ((N) << MIPI_RX_AFE_REG84_REGHSAMP_SHIFT)
#define MIPI_RX_AFE_REG84_REGRTERM_SHIFT (0x04U)
#define MIPI_RX_AFE_REG84_REGRTERM_MASK  (MIPI_RX_AFE_REG84_REGRTERM(0x07U))
#define MIPI_RX_AFE_REG84_REGRTERM(N)    ((N) << MIPI_RX_AFE_REG84_REGRTERM_SHIFT)
#define MIPI_RX_AFE_REG84_REGHSCS_SHIFT  (0x07U)
#define MIPI_RX_AFE_REG84_REGHSCS_MASK   (MIPI_RX_AFE_REG84_REGHSCS(0x01U))
#define MIPI_RX_AFE_REG84_REGHSCS(N)     ((N) << MIPI_RX_AFE_REG84_REGHSCS_SHIFT)

/* TX Pattern Conternt */
#define MIPI_RX_TX_PATTERN_CONTENT_REGA0               (0xA0U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA0_RegMBPM_SHIFT (0x00U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA0_RegMBPM_MASK  (MIPI_RX_TX_PATTERN_CONTENT_REGA0_RegMBPM(0x01U))
#define MIPI_RX_TX_PATTERN_CONTENT_REGA0_RegMBPM(N)    ((N) << MIPI_RX_TX_PATTERN_CONTENT_REGA0_RegMBPM_SHIFT)

#define MIPI_RX_TX_PATTERN_CONTENT_REGA1                   (0xA1U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA1_RegMBPM_HFP_SHIFT (0x00U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA1_RegMBPM_HFP_MASK  (MIPI_RX_TX_PATTERN_CONTENT_REGA1_RegMBPM_HFP(0x7FU))
#define MIPI_RX_TX_PATTERN_CONTENT_REGA1_RegMBPM_HFP(N)    ((N) << MIPI_RX_TX_PATTERN_CONTENT_REGA1_RegMBPM_HFP_SHIFT)

#define MIPI_RX_TX_PATTERN_CONTENT_REGA2                   (0xA2U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA2_RegMBPM_VFP_SHIFT (0x00U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA2_RegMBPM_VFP_MASK  (MIPI_RX_TX_PATTERN_CONTENT_REGA2_RegMBPM_VFP(0x7FU))
#define MIPI_RX_TX_PATTERN_CONTENT_REGA2_RegMBPM_VFP(N)    ((N) << MIPI_RX_TX_PATTERN_CONTENT_REGA2_RegMBPM_VFP_SHIFT)

#define MIPI_RX_TX_PATTERN_CONTENT_REGA3                   (0xA3U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA3_RegMBPM_HSW_SHIFT (0x00U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA3_RegMBPM_HSW_MASK  (MIPI_RX_TX_PATTERN_CONTENT_REGA3_RegMBPM_HSW(0xFFU))
#define MIPI_RX_TX_PATTERN_CONTENT_REGA3_RegMBPM_HSW(N)    ((N) << MIPI_RX_TX_PATTERN_CONTENT_REGA3_RegMBPM_HSW_SHIFT)

#define MIPI_RX_TX_PATTERN_CONTENT_REGA4                   (0xA4U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA4_RegMBPM_HSW_SHIFT (0x00U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA4_RegMBPM_HSW_MASK  (MIPI_RX_TX_PATTERN_CONTENT_REGA4_RegMBPM_HSW(0x3FU))
#define MIPI_RX_TX_PATTERN_CONTENT_REGA4_RegMBPM_HSW(N)    ((N) << MIPI_RX_TX_PATTERN_CONTENT_REGA4_RegMBPM_HSW_SHIFT)

#define MIPI_RX_TX_PATTERN_CONTENT_REGA5                   (0xA5U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA5_RegMBPM_VSW_SHIFT (0x00U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA5_RegMBPM_VSW_MASK  (MIPI_RX_TX_PATTERN_CONTENT_REGA5_RegMBPM_VSW(0xFFU))
#define MIPI_RX_TX_PATTERN_CONTENT_REGA5_RegMBPM_VSW(N)    ((N) << MIPI_RX_TX_PATTERN_CONTENT_REGA5_RegMBPM_VSW_SHIFT)

#define MIPI_RX_TX_PATTERN_CONTENT_REGA6                   (0xA6U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA6_RegMBPM_VSW_SHIFT (0x00U)
#define MIPI_RX_TX_PATTERN_CONTENT_REGA6_RegMBPM_VSW_MASK  (MIPI_RX_TX_PATTERN_CONTENT_REGA6_RegMBPM_VSW(0x3FU))
#define MIPI_RX_TX_PATTERN_CONTENT_REGA6_RegMBPM_VSW(N)    ((N) << MIPI_RX_TX_PATTERN_CONTENT_REGA6_RegMBPM_VSW_SHIFT)

/* CRC */
#define MIPI_RX_CRC_REGC0                      (0xC0U)
#define MIPI_RX_CRC_REGC0_RegTTLTxCRCNum_SHIFT (0x00U)
#define MIPI_RX_CRC_REGC0_RegTTLTxCRCNum_MASK  (MIPI_RX_CRC_REGC0_RegTTLTxCRCNum(0x7FU))
#define MIPI_RX_CRC_REGC0_RegTTLTxCRCNum(N)    ((N) << MIPI_RX_CRC_REGC0_RegTTLTxCRCNum_SHIFT)
#define MIPI_RX_CRC_REGC0_RegEnTTLTxCRC_SHIFT  (0x07U)
#define MIPI_RX_CRC_REGC0_RegEnTTLTxCRC_MASK   (MIPI_RX_CRC_REGC0_RegEnTTLTxCRC(0x01U))
#define MIPI_RX_CRC_REGC0_RegEnTTLTxCRC(N)     ((N) << MIPI_RX_CRC_REGC0_RegEnTTLTxCRC_SHIFT)

#define IT6161_VENDER_ID_REG     (0x0)
#define IT6161_DEVICE_ID_REG     (0x2)
#define IT6161_VENDER_ID_VALUE_L (0x54)
#define IT6161_VENDER_ID_VALUE_H (0x49)
#define IT6161_DEVICE_ID_VALUE_L (0x61)
#define IT6161_DEVICE_ID_VALUE_H (0x61)

#define MIPIRX_Debug_message 1
#define DEBUG_MIPIRX

//////////////////////////////////////////////////////////////////////
// reference: MIPI Alliance Specification for DSI Ch8.7 Table 16 Data Types for Processor-sourced Packets
#define RGB_24b   0x3E
#define RGB_30b   0x0D
#define RGB_36b   0x1D
#define RGB_18b   0x1E
#define RGB_18b_L 0x2E
#define YCbCr_16b 0x2C
#define YCbCr_20b 0x0C
#define YCbCr_24b 0x1C

#define FrmPkt  0
#define SbSFull 3
#define TopBtm  6
#define SbSHalf 8

#define DDC75K  0
#define DDC125K 1
#define DDC312K 2

#define PICAR_NO  0
#define PICAR4_3  1
#define PICAR16_9 2

#define ACTAR_PIC 8
#define ACTAR4_3  9
#define ACTAR16_9 10
#define ACTAR14_9 11

#define LMDbgSel (0x00U)

/* for PatGen */
#define EnRxPatGen (false)

/* MP PtGen option */
#define MPVidType RGB_24b /* RGB_24b , RGB_18b, RGB_18b_L */

#define InvMCLK    (true)
#define InvPCLK    (false)
#define MPLaneNum  (MIPIRX_LANE_NUM - 1) /* 0: 1-lane, 1: 2-lane, 2: 3-lane, 3: 4-lane */
#define EnMPx1PCLK (false)
/* system misc control */
#define PDREFCLK    (false) /* False :div1(20M) */
#define PDREFCNT    (0x00U) /* when PDREFCLK=(true), 0:div2, 1:div4, 2:div8, 3:divg16 */
#define EnIntWakeU3 (false)
#define EnIOIDDQ    (false)
#define EnStb2Rst   (false)
#define EnExtStdby  (false)
#define EnStandby   (false)
#define MPLaneSwap  (false)
#define MPPNSwap    (false) /* (true): MTK , (false): Solomon */

#define DisPHSyncErr (false)
#define DisECCErr    (false)

// PPI option
#define EnContCK      (true)
#define HSSetNum      (0x3U)
#define SkipStg       (0x4U)
#define EnDeSkew      (true)
#define PPIDbgSel     (0x00U)
#define RegIgnrNull   (0x01U)
#define RegIgnrBlk    (0x01U)
#define RegEnDummyECC (0x00U)
#define RegEnSyncErr  (false)

// LM option
#define EOTPSel (0x00U)

// PPS option
#define EnMBPM      (false) /* enable MIPI Bypass Mode */
#define PREC_Update (false) /* enable P-timing update */
#define MREC_Update (false) /* enable M-timing update */
#define REGSELDEF   (false)
#define EnMPExtPCLK (false)
#define MPForceStb  (false)
#define EnHReSync   (false)
#define EnVReSync   (false)
#define EnFReSync   (false)
#define EnVREnh     (false)
#define EnVREnhSel  (0x01U) /* 0:Div2, 1:Div4, 2:Div8, 3:Div16, 4:Div32 */
#define EnMAvg      (true)
#define MShift      (0x04U)
#define PShift      (0x03U)
#define EnFFAutoRst (true)
#define PPSFFRdStg  (0x04)

#define EnTxCRC  (true)
#define TxCRCnum (0x00U)

void MIPIRX_DumpRegs(display_handle_t *handle);
void MIPIRX_CalRclk(display_handle_t *handle);
void MIPIRX_AfeCfg(display_handle_t *handle);
void MIPIRX_CalMclk(display_handle_t *handle);
void MIPIRX_CalPclk(display_handle_t *handle);
void MIPIRX_ShowMRec(display_handle_t *handle);
void MIPIRX_ResetPDomain(display_handle_t *handle);
void HDMITX_GenerateBlankTiming(display_handle_t *handle);
void MIPIRX_ShowPrec(display_handle_t *handle);
void MIPIRX_Reg06_Process(display_handle_t *handle, uint8_t Reg06);
void MIPIRX_Reg07_Process(display_handle_t *handle, uint8_t Reg07);
void MIPIRX_Reg08_Process(display_handle_t *handle, uint8_t Reg08);
void MIPIRX_DevLoopProc(display_handle_t *handle);
#endif // _MIPIRX_H_
