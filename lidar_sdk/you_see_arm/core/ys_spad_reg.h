/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-01-06 03:24:50
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-01-17 08:11:07
 * @FilePath     : /you_see_arm/core/ys_spad_reg.h
 * @Description  :
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */
#ifndef YS_SPAD_REG_H
#define YS_SPAD_REG_H

#include <stdint.h>

typedef struct {
	uint16_t addr; // 寄存器地址
	uint32_t data; // 寄存器数据
} reg_config_t;

// IMX459 Device Status Related Registers
/*
Device Status
0: Initial
1: Standby
2: Initial2
3: Active
4: Terminate
5: DIG-TEST
6: ANA-TEST
*/
#define DEVSTS 0x1A0C
/*
Device state control
0x0: Unstandby(Transition from Standby to Initial 2)
0x1: Standby transition(Transition from Active to Standby)
0x3: Temperature measurement while standby(No state transition)
0x5: DIG-TEST boot(Transition from Standby to DIG-TEST)
0x9: ANA-TEST activation(Transition from Standby to ANA-TEST)
Other than the above, use prohibited
*/
#define STANDBY 0x2D20

#define SRCDIV 0x0005
#define INCK_MODE 0x0006
#define REFCK_MODE 0x0007
#define PL_FC_VT 0x000C
#define PL_RC_VT 0x000E
#define PL_FC_OP 0x0010
#define PL_RC_OP 0x0012
#define DUCK_DIV2EN 0x0014
#define LANE_MODE 0x0015
#define MS_SEL 0x0016
#define OUTPUT_PIX_START 0x00AC
#define OUTPUT_PIX_END 0x00AE
#define OUTIF_TRG_O0 0x00CC
#define OUTIF_TRG_O1 0x00CD
#define FB_PERIOD 0x00D4
#define EBD_OUT_EN 0x00DC
#define STATS_OUT_EN 0x00DD
#define AMBI_OUT_EN 0x012A
#define B_STRM_HIST_TIME_START 0x0148
#define B_STRM_HIST_TIME_END 0x014A
#define DATA_IN_SIZE_MODE 0x0150
#define ARRAY_MODE 0x0300
#define REVERSE_MODE 0x0301
#define LD_NUM 0x0303
#define OUT_DATA_MODE 0x0304
#define H_PIX_MODE 0x0305
#define V_ACT_STEP_EL 0x0306
#define V_ACT_STEP_SP 0x0307
#define SEL_AREA0 0x081C
#define SEL_AREA1 0x081E
#define SEL_AREA2 0x0820
#define SEL_AREA3 0x0822
#define SEL_AREA4 0x0824
#define SEL_AREA5 0x0826
#define SEL_AREA6 0x0828
#define SEL_AREA7 0x082A
#define SEL_AREA8 0x082C
#define SEL_AREA9 0x082E
#define SEL_AREA10 0x0830
#define SEL_AREA11 0x0832
#define V_ACT_START_EL 0x0850
#define V_ROI_OFFSET_EL 0x0852
#define V_ROI_OFFSET_SP 0x0853
#define V_ACT_WIDTH_EL 0x0854
#define V_ROI_WIDTH_EL 0x0856
#define V_ROI_WIDTH_SP 0x0857
#define H_ROI_OFFSET_EL 0x085A
#define V_ROI_ADJ_EN0 0x085C
#define V_ROI_ADJ_EN1 0x085D
#define V_ROI_ADJ_EN2 0x085E
#define V_ROI_ADJ_EN3 0x085F
#define H_ROI_ADJ_STEP_EL0 0x0860
#define V_ROI_ADJ_WIDTH_EL0 0x0861
#define V_ROI_ADJ_WIDTH_SP0 0x0862
#define H_ROI_ADJ_STEP_EL1 0x0864
#define V_ROI_ADJ_WIDTH_EL1 0x0865
#define V_ROI_ADJ_WIDTH_SP1 0x0866
#define V_ROI_ADJ_OFFSET_EL1 0x0868
#define V_ROI_ADJ_OFFSET_SP1 0x0869
#define H_ROI_ADJ_OFFSET_EL1 0x086A
#define H_ROI_ADJ_STEP_EL2 0x086C
#define V_ROI_ADJ_WIDTH_EL2 0x086D
#define V_ROI_ADJ_WIDTH_SP2 0x086E
#define V_ROI_ADJ_OFFSET_EL2 0x0870
#define V_ROI_ADJ_OFFSET_SP2 0x0871
#define H_ROI_ADJ_OFFSET_EL2 0x0872
#define H_ROI_ADJ_STEP_EL3 0x0874
#define V_ROI_ADJ_WIDTH_EL3 0x0875
#define V_ROI_ADJ_WIDTH_SP3 0x0876
#define V_ROI_ADJ_OFFSET_EL3 0x0878
#define V_ROI_ADJ_OFFSET_SP3 0x0879
#define H_ROI_ADJ_OFFSET_EL3 0x087A
#define V_ACT_END_EL 0x087C
#define TRG_O0_OFFSET 0x0898
#define TRG_O0_WIDTH 0x089A
#define TRG_O1_OFFSET 0x089C
#define TRG_O1_WIDTH 0x089E
#define SPAD_REC_START 0x08A6
#define SPAD_REC_PERIOD 0x08A8
#define SPAD_WAKEUP 0x08BC
#define TRG_O0 0x08C4
#define TRG_O1 0x08C6
#define FS_AST_WIDTH 0x08FA
#define SS_AST_WIDTH 0x0904
#define G_OUTPUT_NUM_ECHO 0x0A34
#define G_OUTPUT_HEAD_0 0x0A38
#define G_OUTPUT_HEAD_1 0x0A3C
#define G_OUTPUT_HEAD_2 0x0A40
#define G_OUTPUT_HEAD_3 0x0A44
#define G_OUTPUT_HEAD_4 0x0A48
#define G_OUTPUT_NUMBIN_MODE 0x0A58
#define G_OUTPUT_NUMBIN_VAL 0x0A5A
#define G_OUTPUT_HIST_TIMEINFO 0x0A5C
#define G_MIPI_PACK 0x0A60
#define B_MEAN_MODE 0x0B18
#define B_MEAN_SFT 0x0B19
#define B_MEAN_AMP 0x0B1A
#define B_MEAN_OFFSET 0x0B1C
#define B_MEAN_BIAS 0x0B1E
#define B_BIAS 0x0B24
#define B_LOG_MODE 0x0B26
#define B_SUB_MEAN 0x0B27
#define B_FIR_MODE 0x0B38
#define B_M 0x0B3C
#define B_N_SHIFT 0x0B3E
#define B_VALLEY_TH 0x0B40
#define B_ECHO_ORDER 0x0B42
#define B_X 0x0B43
#define B_JDG_AMP1 0x0B44
#define B_JDG_OFFSET1 0x0B46
#define B_JDG_AMP2 0x0B48
#define B_JDG_OFFSET2 0x0B4A
#define B_JDG_POS 0x0B4C
#define B_JDG_WIDTH 0x0B4E
#define B_DET_AMP3 0x0B50
#define B_DET_SHIFT3 0x0B51
#define B_DET_OFFSET3 0x0B52
#define B_FIR_COEF_0 0x0B5C
#define B_FIR_COEF_1 0x0B5D
#define B_FIR_COEF_2 0x0B5E
#define B_FIR_COEF_3 0x0B5F
#define B_FIR_COEF_4 0x0B60
#define B_FIR_COEF_5 0x0B61
#define B_FIR_COEF_6 0x0B62
#define B_FIR_COEF_7 0x0B63
#define B_FIR_COEF_8 0x0B64
#define B_FIR_COEF_9 0x0B65
#define B_FIR_COEF_10 0x0B66
#define B_FIR_COEF_11 0x0B67
#define B_FIR_COEF_12 0x0B68
#define B_FIR_COEF_13 0x0B69
#define B_FIR_COEF_14 0x0B6A
#define B_FIR_COEF_15 0x0B6B
#define B_FIR_COEF_16 0x0B6C
#define B_FIR_COEF_17 0x0B6D
#define B_FIR_COEF_18 0x0B6E
#define B_FIR_COEF_19 0x0B6F
#define B_FIR_COEF_20 0x0B70
#define B_FIR_COEF_21 0x0B71
#define B_FIR_COEF_22 0x0B72
#define B_FIR_COEF_23 0x0B73
#define B_FIR_COEF_24 0x0B74
#define B_FIR_COEF_25 0x0B75
#define B_FIR_COEF_26 0x0B76
#define B_FIR_COEF_27 0x0B77
#define B_FIR_COEF_28 0x0B78
#define B_FIR_COEF_29 0x0B79
#define B_FIR_COEF_30 0x0B7A
#define B_FIR_COEF_31 0x0B7B

static const reg_config_t spad_original_histogram_reg[] = {
    {SRCDIV, 0x2},
    {INCK_MODE, 0x14},
    {REFCK_MODE, 0x14},
    {PL_FC_VT, 0xC8},
    {PL_RC_VT, 0x2},
    {PL_FC_OP, 0xC8},
    {PL_RC_OP, 0x1},
    {DUCK_DIV2EN, 0x0},
    {LANE_MODE, 0x1},
    {MS_SEL, 0x0},
    {OUTPUT_PIX_START, 0x0},
    {OUTPUT_PIX_END, 0x5F},
    {OUTIF_TRG_O0, 0x0},
    {OUTIF_TRG_O1, 0x0},
    {FB_PERIOD, 0x2710},
    {EBD_OUT_EN, 0x0},
    {STATS_OUT_EN, 0x0},
    {AMBI_OUT_EN, 0x0},
    {B_STRM_HIST_TIME_START, 0x0},
    {B_STRM_HIST_TIME_END, 0x3E7},
    {DATA_IN_SIZE_MODE, 0x1},
    {ARRAY_MODE, 0x1},
    {REVERSE_MODE, 0x1},
    {LD_NUM, 0x0},
    {OUT_DATA_MODE, 0x3},
    {H_PIX_MODE, 0x2},
    {V_ACT_STEP_EL, 0x1},
    {V_ACT_STEP_SP, 0x0},
    {SEL_AREA0, 0x1},
    {SEL_AREA1, 0x2},
    {SEL_AREA2, 0x4},
    {SEL_AREA3, 0x8},
    {SEL_AREA4, 0x10},
    {SEL_AREA5, 0x20},
    {SEL_AREA6, 0x40},
    {SEL_AREA7, 0x80},
    {SEL_AREA8, 0x100},
    {SEL_AREA9, 0x200},
    {SEL_AREA10, 0x400},
    {SEL_AREA11, 0x800},
    {V_ACT_START_EL, 0x0},
    {V_ROI_OFFSET_EL, 0x0},
    {V_ROI_OFFSET_SP, 0x0},
    {V_ACT_WIDTH_EL, 0x3},
    {V_ROI_WIDTH_EL, 0x3},
    {V_ROI_WIDTH_SP, 0x0},
    {H_ROI_OFFSET_EL, 0x0},
    {V_ROI_ADJ_EN0, 0x0},
    {V_ROI_ADJ_EN1, 0x0},
    {V_ROI_ADJ_EN2, 0x0},
    {V_ROI_ADJ_EN3, 0x0},
    {H_ROI_ADJ_STEP_EL0, 0x0},
    {V_ROI_ADJ_WIDTH_EL0, 0x0},
    {V_ROI_ADJ_WIDTH_SP0, 0x0},
    {H_ROI_ADJ_STEP_EL1, 0x0},
    {V_ROI_ADJ_WIDTH_EL1, 0x0},
    {V_ROI_ADJ_WIDTH_SP1, 0x0},
    {V_ROI_ADJ_OFFSET_EL1, 0x0},
    {V_ROI_ADJ_OFFSET_SP1, 0x0},
    {H_ROI_ADJ_OFFSET_EL1, 0x0},
    {H_ROI_ADJ_STEP_EL2, 0x0},
    {V_ROI_ADJ_WIDTH_EL2, 0x0},
    {V_ROI_ADJ_WIDTH_SP2, 0x0},
    {V_ROI_ADJ_OFFSET_EL2, 0x0},
    {V_ROI_ADJ_OFFSET_SP2, 0x0},
    {H_ROI_ADJ_OFFSET_EL2, 0x0},
    {H_ROI_ADJ_STEP_EL3, 0x0},
    {V_ROI_ADJ_WIDTH_EL3, 0x0},
    {V_ROI_ADJ_WIDTH_SP3, 0x0},
    {V_ROI_ADJ_OFFSET_EL3, 0x0},
    {V_ROI_ADJ_OFFSET_SP3, 0x0},
    {H_ROI_ADJ_OFFSET_EL3, 0x0},
    {V_ACT_END_EL, 0x39},
    {TRG_O0_OFFSET, 0x1C8},
    {TRG_O0_WIDTH, 0x19},
    {TRG_O1_OFFSET, 0x1CD},
    {TRG_O1_WIDTH, 0x19},
    {SPAD_REC_START, 0x34},
    {SPAD_REC_PERIOD, 0xFA},
    {SPAD_WAKEUP, 0x14},
    {TRG_O0, 0xFFF},
    {TRG_O1, 0xFFF},
    {FS_AST_WIDTH, 0x38},
    {SS_AST_WIDTH, 0x64},
    {G_OUTPUT_NUM_ECHO, 0x3},
    {G_OUTPUT_HEAD_0, 0x1},
    {G_OUTPUT_HEAD_1, 0x1},
    {G_OUTPUT_HEAD_2, 0x1},
    {G_OUTPUT_HEAD_3, 0x1},
    {G_OUTPUT_HEAD_4, 0x1},
    {G_OUTPUT_NUMBIN_MODE, 0x0},
    {G_OUTPUT_NUMBIN_VAL, 0x22},
    {G_OUTPUT_HIST_TIMEINFO, 0x1},
    {G_MIPI_PACK, 0x0},
    {B_MEAN_MODE, 0x0},
    {B_MEAN_SFT, 0xA},
    {B_MEAN_AMP, 0x103},
    {B_MEAN_OFFSET, 0x0},
    {B_MEAN_BIAS, 0x0},
    {B_BIAS, 0x0},
    {B_LOG_MODE, 0x0},
    {B_SUB_MEAN, 0x0},
    {B_FIR_MODE, 0x1},
    {B_M, 0x24},
    {B_N_SHIFT, 0x0},
    {B_VALLEY_TH, 0xA},
    {B_ECHO_ORDER, 0x0},
    {B_X, 0x8},
    {B_JDG_AMP1, 0x8},
    {B_JDG_OFFSET1, 0x0},
    {B_JDG_AMP2, 0x3},
    {B_JDG_OFFSET2, 0x0},
    {B_JDG_POS, 0x64},
    {B_JDG_WIDTH, 0x0},
    {B_DET_AMP3, 0x0},
    {B_DET_SHIFT3, 0x0},
    {B_DET_OFFSET3, 0x1},
    {B_FIR_COEF_0, 0x0},
    {B_FIR_COEF_1, 0x0},
    {B_FIR_COEF_2, 0x0},
    {B_FIR_COEF_3, 0x0},
    {B_FIR_COEF_4, 0x0},
    {B_FIR_COEF_5, 0x0},
    {B_FIR_COEF_6, 0x0},
    {B_FIR_COEF_7, 0x0},
    {B_FIR_COEF_8, 0x0},
    {B_FIR_COEF_9, 0x0},
    {B_FIR_COEF_10, 0x0},
    {B_FIR_COEF_11, 0x0},
    {B_FIR_COEF_12, 0x0},
    {B_FIR_COEF_13, 0x0},
    {B_FIR_COEF_14, 0x0},
    {B_FIR_COEF_15, 0x0},
    {B_FIR_COEF_16, 0x40},
    {B_FIR_COEF_17, 0x0},
    {B_FIR_COEF_18, 0x0},
    {B_FIR_COEF_19, 0x0},
    {B_FIR_COEF_20, 0x0},
    {B_FIR_COEF_21, 0x0},
    {B_FIR_COEF_22, 0x0},
    {B_FIR_COEF_23, 0x0},
    {B_FIR_COEF_24, 0x0},
    {B_FIR_COEF_25, 0x0},
    {B_FIR_COEF_26, 0x0},
    {B_FIR_COEF_27, 0x0},
    {B_FIR_COEF_28, 0x0},
    {B_FIR_COEF_29, 0x0},
    {B_FIR_COEF_30, 0x0},
    {B_FIR_COEF_31, 0x0},
};

static const reg_config_t spad_point_cloud_reg[] = {
    {SRCDIV, 0x1},
    {INCK_MODE, 0x14},
    {REFCK_MODE, 0x14},
    {PL_FC_VT, 0xC8},
    {PL_RC_VT, 0x2},
    {PL_FC_OP, 0xC8},
    {PL_RC_OP, 0x1},
    {DUCK_DIV2EN, 0x0},
    {LANE_MODE, 0x1},
    {MS_SEL, 0x0},
    {OUTPUT_PIX_START, 0x0},
    {OUTPUT_PIX_END, 0x5F},
    {OUTIF_TRG_O0, 0x0},
    {OUTIF_TRG_O1, 0x0},
    {FB_PERIOD, 0x2710},
    {EBD_OUT_EN, 0x1},
    {STATS_OUT_EN, 0x1},
    {AMBI_OUT_EN, 0x1},
    {B_STRM_HIST_TIME_START, 0x0},
    {B_STRM_HIST_TIME_END, 0x3E7},
    {DATA_IN_SIZE_MODE, 0x1},
    {ARRAY_MODE, 0x0},
    {REVERSE_MODE, 0x1},
    {LD_NUM, 0x0},
    {OUT_DATA_MODE, 0x1},
    {H_PIX_MODE, 0x2},
    {V_ACT_STEP_EL, 0x1},
    {V_ACT_STEP_SP, 0x0},
    {SEL_AREA0, 0x1},
    {SEL_AREA1, 0x2},
    {SEL_AREA2, 0x4},
    {SEL_AREA3, 0x8},
    {SEL_AREA4, 0x10},
    {SEL_AREA5, 0x20},
    {SEL_AREA6, 0x40},
    {SEL_AREA7, 0x80},
    {SEL_AREA8, 0x100},
    {SEL_AREA9, 0x200},
    {SEL_AREA10, 0x400},
    {SEL_AREA11, 0x800},
    {V_ACT_START_EL, 0x0},
    {V_ROI_OFFSET_EL, 0x0},
    {V_ROI_OFFSET_SP, 0x0},
    {V_ACT_WIDTH_EL, 0xE},
    {V_ROI_WIDTH_EL, 0x7},
    {V_ROI_WIDTH_SP, 0x0},
    {H_ROI_OFFSET_EL, 0x0},
    {V_ROI_ADJ_EN0, 0x1},
    {V_ROI_ADJ_EN1, 0x1},
    {V_ROI_ADJ_EN2, 0x0},
    {V_ROI_ADJ_EN3, 0x0},
    {H_ROI_ADJ_STEP_EL0, 0x30},
    {V_ROI_ADJ_WIDTH_EL0, 0x4},
    {V_ROI_ADJ_WIDTH_SP0, 0x0},
    {H_ROI_ADJ_STEP_EL1, 0x0},
    {V_ROI_ADJ_WIDTH_EL1, 0x0},
    {V_ROI_ADJ_WIDTH_SP1, 0x0},
    {V_ROI_ADJ_OFFSET_EL1, 0x0},
    {V_ROI_ADJ_OFFSET_SP1, 0x0},
    {H_ROI_ADJ_OFFSET_EL1, 0x60},
    {H_ROI_ADJ_STEP_EL2, 0x0},
    {V_ROI_ADJ_WIDTH_EL2, 0x0},
    {V_ROI_ADJ_WIDTH_SP2, 0x0},
    {V_ROI_ADJ_OFFSET_EL2, 0x0},
    {V_ROI_ADJ_OFFSET_SP2, 0x0},
    {H_ROI_ADJ_OFFSET_EL2, 0xc0},
    {H_ROI_ADJ_STEP_EL3, 0x0},
    {V_ROI_ADJ_WIDTH_EL3, 0x0},
    {V_ROI_ADJ_WIDTH_SP3, 0x0},
    {V_ROI_ADJ_OFFSET_EL3, 0x0},
    {V_ROI_ADJ_OFFSET_SP3, 0x0},
    {H_ROI_ADJ_OFFSET_EL3, 0xc0},
    {V_ACT_END_EL, 0x37},
    {TRG_O0_OFFSET, 0x1C8},
    {TRG_O0_WIDTH, 0x19},
    {TRG_O1_OFFSET, 0x1CE},
    {TRG_O1_WIDTH, 0x19},
    {SPAD_REC_START, 0x34},
    {SPAD_REC_PERIOD, 0x15E},
    {SPAD_WAKEUP, 0x14},
    {TRG_O0, 0xFFF},
    {TRG_O1, 0xFFF},
    {FS_AST_WIDTH, 0x320},
    {SS_AST_WIDTH, 0xD},
    {G_OUTPUT_NUM_ECHO, 0x3},
    {G_OUTPUT_HEAD_0, 0x1},
    {G_OUTPUT_HEAD_1, 0x1},
    {G_OUTPUT_HEAD_2, 0x1},
    {G_OUTPUT_HEAD_3, 0x1},
    {G_OUTPUT_HEAD_4, 0x1},
    {G_OUTPUT_NUMBIN_MODE, 0x0},
    {G_OUTPUT_NUMBIN_VAL, 0x18},
    {G_OUTPUT_HIST_TIMEINFO, 0x1},
    {G_MIPI_PACK, 0x0},
    {B_MEAN_MODE, 0x0},
    {B_MEAN_SFT, 0x9},
    {B_MEAN_AMP, 0x106},
    {B_MEAN_OFFSET, 0x0},
    {B_MEAN_BIAS, 0x0},
    {B_BIAS, 0x0},
    {B_LOG_MODE, 0x0},
    {B_SUB_MEAN, 0x0},
    {B_FIR_MODE, 0x1},
    {B_M, 0x24},
    {B_N_SHIFT, 0x0},
    {B_VALLEY_TH, 0xA},
    {B_ECHO_ORDER, 0x0},
    {B_X, 0x8},
    {B_JDG_AMP1, 0x8},
    {B_JDG_OFFSET1, 0xC8},
    {B_JDG_AMP2, 0x3},
    {B_JDG_OFFSET2, 0x0},
    {B_JDG_POS, 0x64},
    {B_JDG_WIDTH, 0x0},
    {B_DET_AMP3, 0x0},
    {B_DET_SHIFT3, 0x0},
    {B_DET_OFFSET3, 0x1},
    {B_FIR_COEF_0, 0x0},
    {B_FIR_COEF_1, 0x0},
    {B_FIR_COEF_2, 0x0},
    {B_FIR_COEF_3, 0x0},
    {B_FIR_COEF_4, 0x0},
    {B_FIR_COEF_5, 0x0},
    {B_FIR_COEF_6, 0x0},
    {B_FIR_COEF_7, 0x0},
    {B_FIR_COEF_8, 0x0},
    {B_FIR_COEF_9, 0x0},
    {B_FIR_COEF_10, 0x0},
    {B_FIR_COEF_11, 0x0},
    {B_FIR_COEF_12, 0x0},
    {B_FIR_COEF_13, 0x0},
    {B_FIR_COEF_14, 0x0},
    {B_FIR_COEF_15, 0x0},
    {B_FIR_COEF_16, 0x40},
    {B_FIR_COEF_17, 0x0},
    {B_FIR_COEF_18, 0x0},
    {B_FIR_COEF_19, 0x0},
    {B_FIR_COEF_20, 0x0},
    {B_FIR_COEF_21, 0x0},
    {B_FIR_COEF_22, 0x0},
    {B_FIR_COEF_23, 0x0},
    {B_FIR_COEF_24, 0x0},
    {B_FIR_COEF_25, 0x0},
    {B_FIR_COEF_26, 0x0},
    {B_FIR_COEF_27, 0x0},
    {B_FIR_COEF_28, 0x0},
    {B_FIR_COEF_29, 0x0},
    {B_FIR_COEF_30, 0x0},
    {B_FIR_COEF_31, 0x0},
};

#endif // YS_SPAD_REG_H
