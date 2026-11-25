#ifndef __FL9103_REG_DEFINE_V01_00_00_H__
#define __FL9103_REG_DEFINE_V01_00_00_H__

//==================================================
// version info
//==================================================

#define FL9103_REG_DEFINE_MAJOR_VER    0x01
#define FL9103_REG_DEFINE_MINOR_VER    0x00
#define FL9103_REG_DEFINE_PATCH_VER    0x00

//==================================================
// reg_define
//==================================================
#define SYSMSC_CHIP_ID                  0x00001000
#define SYSMSC_ERR_STATUS               0x00001004
#define SYSMSC_ERR_MASK                 0x00001008
#define SYSMSC_ERR_CLEAR                0x0000100C
#define SYSMSC_ANA_STATUS               0x00001010
#define SYSMSC_ERR_ANA_DEBOUNCE         0x00001014
#define SYSMSC_DEBUG                    0x0000101C
#define SYSMSC_CLK_MONITOR              0x00001020
#define SYSMSC_MONITOR0_CNT0            0x00001024
#define SYSMSC_MONITOR0_CNT1            0x00001028
#define SYSMSC_MONITOR1_CNT0            0x0000102C
#define SYSMSC_MONITOR1_CNT1            0x00001030
#define SYSMSC_DBG_TRIG_GEN             0x00001034

#define HOST_UI_BUS_GATE_EN             0x00000014
#define HOST_UI_MOSI_POS                0x00000018
#define HOST_UI_CHECK_MODE              0x0000001C
#define HOST_UI_WCHK_STOR               0x00000020
#define HOST_UI_RCHK_STOR               0x00000024
#define HOST_UI_INT                     0x00000030
#define HOST_UI_INT_CLR                 0x00000034
#define HOST_UI_INT_EN                  0x00000038

#define DRVCTR_FUNC_CONFIG              0x00003000
#define DRVCTR_MODE_CONFIG1             0x00003004
#define DRVCTR_MODE_CONFIG0             0x00003008
#define DRVCTR_POWER_CONFIG_HL_CH23_16  0x0000300C
#define DRVCTR_POWER_CONFIG_HL_CH15_8   0x00003010
#define DRVCTR_POWER_CONFIG_HL_CH7_0    0x00003014
#define DRVCTR_POWER_CONFIG_SEL         0x00003018
#define DRVCTR_POWER_CONFIG_PWP         0x0000301C
#define DRVCTR_CH_D2A_RB_HL             0x00003020
#define DRVCTR_CH_D2A_RB_OE             0x00003024
#define DRVCTR_DET_DAC_DONFIG           0x00003028
#define DRVCTR_DET_TIME_CONFIG          0x0000302C
#define DRVCTR_DET_MODE_TRIG            0x00003030
#define DRVCTR_ERROR                    0x00003034
#define DRVCTR_DET_RESULT               0x00003038

#define OTPCTR_TRIG_MODE_CTRL           0x00002000
#define OTPCTR_CFG0                     0x0000200C
#define OTPCTR_DEBUG0                   0x00002018
#define OTPCTR_DEBUG1                   0x0000201C
#define OTPCTR_DEBUG2                   0x00002020
#define OTPCTR_SHADOW_TRIM              0x00002024

#define TRGCNT_MODE_CONFIG              0x00004000
#define TRGCNT_TEST_MODE                0x00004004
#define TRGCNT_MODE_EN                  0x00004008
#define TRGCNT_LDMOS_OPEN_DET           0x0000400C
#define TRGCNT_TRGCNT_A1                0x00004010
#define TRGCNT_TRGCNT_A0                0x00004014
#define TRGCNT_TRGCNT_B1                0x00004018
#define TRGCNT_TRGCNT_B0                0x0000401C
#define TRGCNT_SLOTCNT_A                0x00004020
#define TRGCNT_SLOTCNT_B                0x00004024
#define TRGCNT_PWRCNT_A                 0x00004028
#define TRGCNT_PWRCNT_B                 0x0000402C
#define TRGCNT_FRAME_WD                 0x00004030
#define TRGCNT_CHCNT_OUT                0x00004034
#define TRGCNT_TRGCNT_OUT               0x00004038
#define TRGCNT_SLOTCNT_OUT              0x0000403C
#define TRGCNT_PWRCNT_OUT               0x00004040
#define TRGCNT_STATUS_ERROR             0x00004044



//==================================================
// sig_define
//==================================================
//--------------------------------
// module: sysmsc
//--------------------------------
// SYSMSC_CHIP_ID bit def
// chip_id
#define SYSMSC_RO_DEF_CHIP_ID_CHIP_ID                  0x8331         // 28-bit
#define SYSMSC_RO_MASK_CHIP_ID_CHIP_ID                 ((0x0FFFFFFFU) <<  4U)
#define SYSMSC_RO_CHIP_ID_CHIP_ID(var)                 (((var) & 0x0FFFFFFFU) <<  4U)
#define SYSMSC_RO_SFT_CHIP_ID_CHIP_ID                   4
#define SYSMSC_RO_GET_CHIP_ID_CHIP_ID(var)             (((var) & SYSMSC_RO_MASK_CHIP_ID_CHIP_ID) >>  4U)
// VERSION_ID
#define SYSMSC_RO_DEF_CHIP_ID_VERSION_ID               0x0            // 4-bit
#define SYSMSC_RO_MASK_CHIP_ID_VERSION_ID              ((0x0000000FU) <<  0U)
#define SYSMSC_RO_CHIP_ID_VERSION_ID(var)              (((var) & 0x0000000FU) <<  0U)
#define SYSMSC_RO_SFT_CHIP_ID_VERSION_ID                0
#define SYSMSC_RO_GET_CHIP_ID_VERSION_ID(var)          (((var) & SYSMSC_RO_MASK_CHIP_ID_VERSION_ID) >>  0U)

// SYSMSC_ERR_STATUS bit def
// err_status
#define SYSMSC_RO_DEF_ERR_STATUS_ERR_STATUS               0x0            // 32-bit
#define SYSMSC_RO_MASK_ERR_STATUS_ERR_STATUS              ((0xFFFFFFFFU) <<  0U)
#define SYSMSC_RO_ERR_STATUS_ERR_STATUS(var)              (((var) & 0xFFFFFFFFU) <<  0U)
#define SYSMSC_RO_SFT_ERR_STATUS_ERR_STATUS                0
#define SYSMSC_RO_GET_ERR_STATUS_ERR_STATUS(var)          (((var) & SYSMSC_RO_MASK_ERR_STATUS_ERR_STATUS) >>  0U)

// SYSMSC_ERR_MASK bit def
// err_mask
#define SYSMSC_RW_DEF_ERR_MASK_ERR_MASK                 0x0            // 32-bit
#define SYSMSC_RW_MASK_ERR_MASK_ERR_MASK                ((0xFFFFFFFFU) <<  0U)
#define SYSMSC_RW_ERR_MASK_ERR_MASK(var)                (((var) & 0xFFFFFFFFU) <<  0U)
#define SYSMSC_RW_SFT_ERR_MASK_ERR_MASK                  0
#define SYSMSC_RW_GET_ERR_MASK_ERR_MASK(var)            (((var) & SYSMSC_RW_MASK_ERR_MASK_ERR_MASK) >>  0U)

// SYSMSC_ERR_CLEAR bit def
// err_clr
#define SYSMSC_WO_DEF_ERR_CLEAR_ERR_CLR                  0x0            // 32-bit
#define SYSMSC_WO_MASK_ERR_CLEAR_ERR_CLR                 ((0xFFFFFFFFU) <<  0U)
#define SYSMSC_WO_ERR_CLEAR_ERR_CLR(var)                 (((var) & 0xFFFFFFFFU) <<  0U)
#define SYSMSC_WO_SFT_ERR_CLEAR_ERR_CLR                   0
#define SYSMSC_WO_GET_ERR_CLEAR_ERR_CLR(var)             (((var) & SYSMSC_WO_MASK_ERR_CLEAR_ERR_CLR) >>  0U)

// SYSMSC_ANA_STATUS bit def
// OTP_TRIG_150_A2D
#define SYSMSC_RO_DEF_ANA_STATUS_OTP_TRIG_150_A2D         0x0            // 1-bit
#define SYSMSC_RO_MASK_ANA_STATUS_OTP_TRIG_150_A2D        ((0x00000001U) <<  5U)
#define SYSMSC_RO_ANA_STATUS_OTP_TRIG_150_A2D(var)        (((var) & 0x00000001U) <<  5U)
#define SYSMSC_RO_SFT_ANA_STATUS_OTP_TRIG_150_A2D          5
#define SYSMSC_RO_GET_ANA_STATUS_OTP_TRIG_150_A2D(var)    (((var) & SYSMSC_RO_MASK_ANA_STATUS_OTP_TRIG_150_A2D) >>  5U)
// OTP_TRIG_125_A2D
#define SYSMSC_RO_DEF_ANA_STATUS_OTP_TRIG_125_A2D         0x0            // 1-bit
#define SYSMSC_RO_MASK_ANA_STATUS_OTP_TRIG_125_A2D        ((0x00000001U) <<  4U)
#define SYSMSC_RO_ANA_STATUS_OTP_TRIG_125_A2D(var)        (((var) & 0x00000001U) <<  4U)
#define SYSMSC_RO_SFT_ANA_STATUS_OTP_TRIG_125_A2D          4
#define SYSMSC_RO_GET_ANA_STATUS_OTP_TRIG_125_A2D(var)    (((var) & SYSMSC_RO_MASK_ANA_STATUS_OTP_TRIG_125_A2D) >>  4U)
// OVP55P_A2D
#define SYSMSC_RO_DEF_ANA_STATUS_OVP55P_A2D               0x0            // 1-bit
#define SYSMSC_RO_MASK_ANA_STATUS_OVP55P_A2D              ((0x00000001U) <<  3U)
#define SYSMSC_RO_ANA_STATUS_OVP55P_A2D(var)              (((var) & 0x00000001U) <<  3U)
#define SYSMSC_RO_SFT_ANA_STATUS_OVP55P_A2D                3
#define SYSMSC_RO_GET_ANA_STATUS_OVP55P_A2D(var)          (((var) & SYSMSC_RO_MASK_ANA_STATUS_OVP55P_A2D) >>  3U)
// POR_AOKP_A2D
#define SYSMSC_RO_DEF_ANA_STATUS_POR_AOKP_A2D             0x0            // 1-bit
#define SYSMSC_RO_MASK_ANA_STATUS_POR_AOKP_A2D            ((0x00000001U) <<  2U)
#define SYSMSC_RO_ANA_STATUS_POR_AOKP_A2D(var)            (((var) & 0x00000001U) <<  2U)
#define SYSMSC_RO_SFT_ANA_STATUS_POR_AOKP_A2D              2
#define SYSMSC_RO_GET_ANA_STATUS_POR_AOKP_A2D(var)        (((var) & SYSMSC_RO_MASK_ANA_STATUS_POR_AOKP_A2D) >>  2U)
// OVP18P_A2D
#define SYSMSC_RO_DEF_ANA_STATUS_OVP18P_A2D               0x0            // 1-bit
#define SYSMSC_RO_MASK_ANA_STATUS_OVP18P_A2D              ((0x00000001U) <<  1U)
#define SYSMSC_RO_ANA_STATUS_OVP18P_A2D(var)              (((var) & 0x00000001U) <<  1U)
#define SYSMSC_RO_SFT_ANA_STATUS_OVP18P_A2D                1
#define SYSMSC_RO_GET_ANA_STATUS_OVP18P_A2D(var)          (((var) & SYSMSC_RO_MASK_ANA_STATUS_OVP18P_A2D) >>  1U)
// OSC32K_READY_A2D
#define SYSMSC_RO_DEF_ANA_STATUS_OSC32K_READY_A2D         0x0            // 1-bit
#define SYSMSC_RO_MASK_ANA_STATUS_OSC32K_READY_A2D        ((0x00000001U) <<  0U)
#define SYSMSC_RO_ANA_STATUS_OSC32K_READY_A2D(var)        (((var) & 0x00000001U) <<  0U)
#define SYSMSC_RO_SFT_ANA_STATUS_OSC32K_READY_A2D          0
#define SYSMSC_RO_GET_ANA_STATUS_OSC32K_READY_A2D(var)    (((var) & SYSMSC_RO_MASK_ANA_STATUS_OSC32K_READY_A2D) >>  0U)

// SYSMSC_ERR_ANA_DEBOUNCE bit def
// ana_err_debounce_en
#define SYSMSC_RW_DEF_ERR_ANA_DEBOUNCE_ANA_ERR_DEBOUNCE_EN      0x1F           // 5-bit
#define SYSMSC_RW_MASK_ERR_ANA_DEBOUNCE_ANA_ERR_DEBOUNCE_EN     ((0x0000001FU) << 24U)
#define SYSMSC_RW_ERR_ANA_DEBOUNCE_ANA_ERR_DEBOUNCE_EN(var)     (((var) & 0x0000001FU) << 24U)
#define SYSMSC_RW_SFT_ERR_ANA_DEBOUNCE_ANA_ERR_DEBOUNCE_EN      24
#define SYSMSC_RW_GET_ERR_ANA_DEBOUNCE_ANA_ERR_DEBOUNCE_EN(var) (((var) & SYSMSC_RW_MASK_ERR_ANA_DEBOUNCE_ANA_ERR_DEBOUNCE_EN) >> 24U)
// otp_err_time
#define SYSMSC_RW_DEF_ERR_ANA_DEBOUNCE_OTP_ERR_TIME             0x20           // 12-bit
#define SYSMSC_RW_MASK_ERR_ANA_DEBOUNCE_OTP_ERR_TIME            ((0x00000FFFU) << 12U)
#define SYSMSC_RW_ERR_ANA_DEBOUNCE_OTP_ERR_TIME(var)            (((var) & 0x00000FFFU) << 12U)
#define SYSMSC_RW_SFT_ERR_ANA_DEBOUNCE_OTP_ERR_TIME             12
#define SYSMSC_RW_GET_ERR_ANA_DEBOUNCE_OTP_ERR_TIME(var)        (((var) & SYSMSC_RW_MASK_ERR_ANA_DEBOUNCE_OTP_ERR_TIME) >> 12U)
// ovp_err_time
#define SYSMSC_RW_DEF_ERR_ANA_DEBOUNCE_OVP_ERR_TIME             0x20           // 12-bit
#define SYSMSC_RW_MASK_ERR_ANA_DEBOUNCE_OVP_ERR_TIME            ((0x00000FFFU) <<  0U)
#define SYSMSC_RW_ERR_ANA_DEBOUNCE_OVP_ERR_TIME(var)            (((var) & 0x00000FFFU) <<  0U)
#define SYSMSC_RW_SFT_ERR_ANA_DEBOUNCE_OVP_ERR_TIME              0
#define SYSMSC_RW_GET_ERR_ANA_DEBOUNCE_OVP_ERR_TIME(var)        (((var) & SYSMSC_RW_MASK_ERR_ANA_DEBOUNCE_OVP_ERR_TIME) >>  0U)

// SYSMSC_DEBUG bit def
// otp150_shutdwn_mask
#define SYSMSC_RW_DEF_DEBUG_OTP150_SHUTDWN_MASK      0x0            // 1-bit
#define SYSMSC_RW_MASK_DEBUG_OTP150_SHUTDWN_MASK     ((0x00000001U) <<  1U)
#define SYSMSC_RW_DEBUG_OTP150_SHUTDWN_MASK(var)     (((var) & 0x00000001U) <<  1U)
#define SYSMSC_RW_SFT_DEBUG_OTP150_SHUTDWN_MASK       1
#define SYSMSC_RW_GET_DEBUG_OTP150_SHUTDWN_MASK(var) (((var) & SYSMSC_RW_MASK_DEBUG_OTP150_SHUTDWN_MASK) >>  1U)
// pad_err_pseudo_err
#define SYSMSC_RW_DEF_DEBUG_PAD_ERR_PSEUDO_ERR       0x0            // 1-bit
#define SYSMSC_RW_MASK_DEBUG_PAD_ERR_PSEUDO_ERR      ((0x00000001U) <<  0U)
#define SYSMSC_RW_DEBUG_PAD_ERR_PSEUDO_ERR(var)      (((var) & 0x00000001U) <<  0U)
#define SYSMSC_RW_SFT_DEBUG_PAD_ERR_PSEUDO_ERR        0
#define SYSMSC_RW_GET_DEBUG_PAD_ERR_PSEUDO_ERR(var)  (((var) & SYSMSC_RW_MASK_DEBUG_PAD_ERR_PSEUDO_ERR) >>  0U)

// SYSMSC_CLK_MONITOR bit def
// clk_monitor_en
#define SYSMSC_RW_DEF_CLK_MONITOR_CLK_MONITOR_EN           0x0            // 2-bit
#define SYSMSC_RW_MASK_CLK_MONITOR_CLK_MONITOR_EN          ((0x00000003U) <<  0U)
#define SYSMSC_RW_CLK_MONITOR_CLK_MONITOR_EN(var)          (((var) & 0x00000003U) <<  0U)
#define SYSMSC_RW_SFT_CLK_MONITOR_CLK_MONITOR_EN            0
#define SYSMSC_RW_GET_CLK_MONITOR_CLK_MONITOR_EN(var)      (((var) & SYSMSC_RW_MASK_CLK_MONITOR_CLK_MONITOR_EN) >>  0U)

// SYSMSC_MONITOR0_CNT0 bit def
// monitor0_cnt0_max
#define SYSMSC_RW_DEF_MONITOR0_CNT0_MONITOR0_CNT0_MAX        0xC3500        // 32-bit
#define SYSMSC_RW_MASK_MONITOR0_CNT0_MONITOR0_CNT0_MAX       ((0xFFFFFFFFU) <<  0U)
#define SYSMSC_RW_MONITOR0_CNT0_MONITOR0_CNT0_MAX(var)       (((var) & 0xFFFFFFFFU) <<  0U)
#define SYSMSC_RW_SFT_MONITOR0_CNT0_MONITOR0_CNT0_MAX         0
#define SYSMSC_RW_GET_MONITOR0_CNT0_MONITOR0_CNT0_MAX(var)   (((var) & SYSMSC_RW_MASK_MONITOR0_CNT0_MONITOR0_CNT0_MAX) >>  0U)

// SYSMSC_MONITOR0_CNT1 bit def
// monitor0_cnt1_max
#define SYSMSC_RW_DEF_MONITOR0_CNT1_MONITOR0_CNT1_MAX        0x4B0          // 32-bit
#define SYSMSC_RW_MASK_MONITOR0_CNT1_MONITOR0_CNT1_MAX       ((0xFFFFFFFFU) <<  0U)
#define SYSMSC_RW_MONITOR0_CNT1_MONITOR0_CNT1_MAX(var)       (((var) & 0xFFFFFFFFU) <<  0U)
#define SYSMSC_RW_SFT_MONITOR0_CNT1_MONITOR0_CNT1_MAX         0
#define SYSMSC_RW_GET_MONITOR0_CNT1_MONITOR0_CNT1_MAX(var)   (((var) & SYSMSC_RW_MASK_MONITOR0_CNT1_MONITOR0_CNT1_MAX) >>  0U)

// SYSMSC_MONITOR1_CNT0 bit def
// monitor1_cnt0_max
#define SYSMSC_RW_DEF_MONITOR1_CNT0_MONITOR1_CNT0_MAX        0x384          // 32-bit
#define SYSMSC_RW_MASK_MONITOR1_CNT0_MONITOR1_CNT0_MAX       ((0xFFFFFFFFU) <<  0U)
#define SYSMSC_RW_MONITOR1_CNT0_MONITOR1_CNT0_MAX(var)       (((var) & 0xFFFFFFFFU) <<  0U)
#define SYSMSC_RW_SFT_MONITOR1_CNT0_MONITOR1_CNT0_MAX         0
#define SYSMSC_RW_GET_MONITOR1_CNT0_MONITOR1_CNT0_MAX(var)   (((var) & SYSMSC_RW_MASK_MONITOR1_CNT0_MONITOR1_CNT0_MAX) >>  0U)

// SYSMSC_MONITOR1_CNT1 bit def
// monitor1_cnt1_max
#define SYSMSC_RW_DEF_MONITOR1_CNT1_MONITOR1_CNT1_MAX        0xC3500        // 32-bit
#define SYSMSC_RW_MASK_MONITOR1_CNT1_MONITOR1_CNT1_MAX       ((0xFFFFFFFFU) <<  0U)
#define SYSMSC_RW_MONITOR1_CNT1_MONITOR1_CNT1_MAX(var)       (((var) & 0xFFFFFFFFU) <<  0U)
#define SYSMSC_RW_SFT_MONITOR1_CNT1_MONITOR1_CNT1_MAX         0
#define SYSMSC_RW_GET_MONITOR1_CNT1_MONITOR1_CNT1_MAX(var)   (((var) & SYSMSC_RW_MASK_MONITOR1_CNT1_MONITOR1_CNT1_MAX) >>  0U)

// SYSMSC_DBG_TRIG_GEN bit def
// power_sel_int
#define SYSMSC_RW_DEF_DBG_TRIG_GEN_POWER_SEL_INT            0x0            // 2-bit
#define SYSMSC_RW_MASK_DBG_TRIG_GEN_POWER_SEL_INT           ((0x00000003U) <<  6U)
#define SYSMSC_RW_DBG_TRIG_GEN_POWER_SEL_INT(var)           (((var) & 0x00000003U) <<  6U)
#define SYSMSC_RW_SFT_DBG_TRIG_GEN_POWER_SEL_INT             6
#define SYSMSC_RW_GET_DBG_TRIG_GEN_POWER_SEL_INT(var)       (((var) & SYSMSC_RW_MASK_DBG_TRIG_GEN_POWER_SEL_INT) >>  6U)
// slot_sel_int
#define SYSMSC_RW_DEF_DBG_TRIG_GEN_SLOT_SEL_INT             0x0            // 2-bit
#define SYSMSC_RW_MASK_DBG_TRIG_GEN_SLOT_SEL_INT            ((0x00000003U) <<  4U)
#define SYSMSC_RW_DBG_TRIG_GEN_SLOT_SEL_INT(var)            (((var) & 0x00000003U) <<  4U)
#define SYSMSC_RW_SFT_DBG_TRIG_GEN_SLOT_SEL_INT              4
#define SYSMSC_RW_GET_DBG_TRIG_GEN_SLOT_SEL_INT(var)        (((var) & SYSMSC_RW_MASK_DBG_TRIG_GEN_SLOT_SEL_INT) >>  4U)
// frame_sync_int
#define SYSMSC_RW_DEF_DBG_TRIG_GEN_FRAME_SYNC_INT           0x0            // 1-bit
#define SYSMSC_RW_MASK_DBG_TRIG_GEN_FRAME_SYNC_INT          ((0x00000001U) <<  3U)
#define SYSMSC_RW_DBG_TRIG_GEN_FRAME_SYNC_INT(var)          (((var) & 0x00000001U) <<  3U)
#define SYSMSC_RW_SFT_DBG_TRIG_GEN_FRAME_SYNC_INT            3
#define SYSMSC_RW_GET_DBG_TRIG_GEN_FRAME_SYNC_INT(var)      (((var) & SYSMSC_RW_MASK_DBG_TRIG_GEN_FRAME_SYNC_INT) >>  3U)
// trig_bypass_power_sel
#define SYSMSC_RW_DEF_DBG_TRIG_GEN_TRIG_BYPASS_POWER_SEL        0x0            // 1-bit
#define SYSMSC_RW_MASK_DBG_TRIG_GEN_TRIG_BYPASS_POWER_SEL       ((0x00000001U) <<  2U)
#define SYSMSC_RW_DBG_TRIG_GEN_TRIG_BYPASS_POWER_SEL(var)       (((var) & 0x00000001U) <<  2U)
#define SYSMSC_RW_SFT_DBG_TRIG_GEN_TRIG_BYPASS_POWER_SEL         2
#define SYSMSC_RW_GET_DBG_TRIG_GEN_TRIG_BYPASS_POWER_SEL(var)   (((var) & SYSMSC_RW_MASK_DBG_TRIG_GEN_TRIG_BYPASS_POWER_SEL) >>  2U)
// trig_bypass_slot_sel
#define SYSMSC_RW_DEF_DBG_TRIG_GEN_TRIG_BYPASS_SLOT_SEL         0x0            // 1-bit
#define SYSMSC_RW_MASK_DBG_TRIG_GEN_TRIG_BYPASS_SLOT_SEL        ((0x00000001U) <<  1U)
#define SYSMSC_RW_DBG_TRIG_GEN_TRIG_BYPASS_SLOT_SEL(var)        (((var) & 0x00000001U) <<  1U)
#define SYSMSC_RW_SFT_DBG_TRIG_GEN_TRIG_BYPASS_SLOT_SEL          1
#define SYSMSC_RW_GET_DBG_TRIG_GEN_TRIG_BYPASS_SLOT_SEL(var)    (((var) & SYSMSC_RW_MASK_DBG_TRIG_GEN_TRIG_BYPASS_SLOT_SEL) >>  1U)
// trig_bypass_frame_sync
#define SYSMSC_RW_DEF_DBG_TRIG_GEN_TRIG_BYPASS_FRAME_SYNC       0x0            // 1-bit
#define SYSMSC_RW_MASK_DBG_TRIG_GEN_TRIG_BYPASS_FRAME_SYNC      ((0x00000001U) <<  0U)
#define SYSMSC_RW_DBG_TRIG_GEN_TRIG_BYPASS_FRAME_SYNC(var)      (((var) & 0x00000001U) <<  0U)
#define SYSMSC_RW_SFT_DBG_TRIG_GEN_TRIG_BYPASS_FRAME_SYNC        0
#define SYSMSC_RW_GET_DBG_TRIG_GEN_TRIG_BYPASS_FRAME_SYNC(var)  (((var) & SYSMSC_RW_MASK_DBG_TRIG_GEN_TRIG_BYPASS_FRAME_SYNC) >>  0U)


//--------------------------------
// module: host_ui
//--------------------------------
// HOST_UI_BUS_GATE_EN bit def
// bus_gate_en
#define HOST_UI_RW_DEF_BUS_GATE_EN_BUS_GATE_EN         0x0            // 1-bit
#define HOST_UI_RW_MASK_BUS_GATE_EN_BUS_GATE_EN        ((0x00000001U) <<  0U)
#define HOST_UI_RW_BUS_GATE_EN_BUS_GATE_EN(var)        (((var) & 0x00000001U) <<  0U)
#define HOST_UI_RW_SFT_BUS_GATE_EN_BUS_GATE_EN          0
#define HOST_UI_RW_GET_BUS_GATE_EN_BUS_GATE_EN(var)    (((var) & HOST_UI_RW_MASK_BUS_GATE_EN_BUS_GATE_EN) >>  0U)

// HOST_UI_MOSI_POS bit def
// miso_pos_en
#define HOST_UI_RW_DEF_MOSI_POS_MISO_POS_EN         0x0            // 1-bit
#define HOST_UI_RW_MASK_MOSI_POS_MISO_POS_EN        ((0x00000001U) <<  0U)
#define HOST_UI_RW_MOSI_POS_MISO_POS_EN(var)        (((var) & 0x00000001U) <<  0U)
#define HOST_UI_RW_SFT_MOSI_POS_MISO_POS_EN          0
#define HOST_UI_RW_GET_MOSI_POS_MISO_POS_EN(var)    (((var) & HOST_UI_RW_MASK_MOSI_POS_MISO_POS_EN) >>  0U)

// HOST_UI_CHECK_MODE bit def
// check_mode
#define HOST_UI_RW_DEF_CHECK_MODE_CHECK_MODE          0x1            // 1-bit
#define HOST_UI_RW_MASK_CHECK_MODE_CHECK_MODE         ((0x00000001U) <<  0U)
#define HOST_UI_RW_CHECK_MODE_CHECK_MODE(var)         (((var) & 0x00000001U) <<  0U)
#define HOST_UI_RW_SFT_CHECK_MODE_CHECK_MODE           0
#define HOST_UI_RW_GET_CHECK_MODE_CHECK_MODE(var)     (((var) & HOST_UI_RW_MASK_CHECK_MODE_CHECK_MODE) >>  0U)

// HOST_UI_WCHK_STOR bit def
// wcrc_stor
#define HOST_UI_RO_DEF_WCHK_STOR_WCRC_STOR           0x0            // 16-bit
#define HOST_UI_RO_MASK_WCHK_STOR_WCRC_STOR          ((0x0000FFFFU) << 16U)
#define HOST_UI_RO_WCHK_STOR_WCRC_STOR(var)          (((var) & 0x0000FFFFU) << 16U)
#define HOST_UI_RO_SFT_WCHK_STOR_WCRC_STOR           16
#define HOST_UI_RO_GET_WCHK_STOR_WCRC_STOR(var)      (((var) & HOST_UI_RO_MASK_WCHK_STOR_WCRC_STOR) >> 16U)
// wadd_stor
#define HOST_UI_RO_DEF_WCHK_STOR_WADD_STOR           0x0            // 16-bit
#define HOST_UI_RO_MASK_WCHK_STOR_WADD_STOR          ((0x0000FFFFU) <<  0U)
#define HOST_UI_RO_WCHK_STOR_WADD_STOR(var)          (((var) & 0x0000FFFFU) <<  0U)
#define HOST_UI_RO_SFT_WCHK_STOR_WADD_STOR            0
#define HOST_UI_RO_GET_WCHK_STOR_WADD_STOR(var)      (((var) & HOST_UI_RO_MASK_WCHK_STOR_WADD_STOR) >>  0U)

// HOST_UI_RCHK_STOR bit def
// rcrc_stor
#define HOST_UI_RO_DEF_RCHK_STOR_RCRC_STOR           0x0            // 16-bit
#define HOST_UI_RO_MASK_RCHK_STOR_RCRC_STOR          ((0x0000FFFFU) << 16U)
#define HOST_UI_RO_RCHK_STOR_RCRC_STOR(var)          (((var) & 0x0000FFFFU) << 16U)
#define HOST_UI_RO_SFT_RCHK_STOR_RCRC_STOR           16
#define HOST_UI_RO_GET_RCHK_STOR_RCRC_STOR(var)      (((var) & HOST_UI_RO_MASK_RCHK_STOR_RCRC_STOR) >> 16U)
// radd_stor
#define HOST_UI_RO_DEF_RCHK_STOR_RADD_STOR           0x0            // 16-bit
#define HOST_UI_RO_MASK_RCHK_STOR_RADD_STOR          ((0x0000FFFFU) <<  0U)
#define HOST_UI_RO_RCHK_STOR_RADD_STOR(var)          (((var) & 0x0000FFFFU) <<  0U)
#define HOST_UI_RO_SFT_RCHK_STOR_RADD_STOR            0
#define HOST_UI_RO_GET_RCHK_STOR_RADD_STOR(var)      (((var) & HOST_UI_RO_MASK_RCHK_STOR_RADD_STOR) >>  0U)

// HOST_UI_INT bit def
// check_int
#define HOST_UI_RO_DEF_INT_CHECK_INT           0x0            // 1-bit
#define HOST_UI_RO_MASK_INT_CHECK_INT          ((0x00000001U) <<  2U)
#define HOST_UI_RO_INT_CHECK_INT(var)          (((var) & 0x00000001U) <<  2U)
#define HOST_UI_RO_SFT_INT_CHECK_INT            2
#define HOST_UI_RO_GET_INT_CHECK_INT(var)      (((var) & HOST_UI_RO_MASK_INT_CHECK_INT) >>  2U)
// hresp_int
#define HOST_UI_RO_DEF_INT_HRESP_INT           0x0            // 1-bit
#define HOST_UI_RO_MASK_INT_HRESP_INT          ((0x00000001U) <<  1U)
#define HOST_UI_RO_INT_HRESP_INT(var)          (((var) & 0x00000001U) <<  1U)
#define HOST_UI_RO_SFT_INT_HRESP_INT            1
#define HOST_UI_RO_GET_INT_HRESP_INT(var)      (((var) & HOST_UI_RO_MASK_INT_HRESP_INT) >>  1U)
// hready_int
#define HOST_UI_RO_DEF_INT_HREADY_INT          0x0            // 1-bit
#define HOST_UI_RO_MASK_INT_HREADY_INT         ((0x00000001U) <<  0U)
#define HOST_UI_RO_INT_HREADY_INT(var)         (((var) & 0x00000001U) <<  0U)
#define HOST_UI_RO_SFT_INT_HREADY_INT           0
#define HOST_UI_RO_GET_INT_HREADY_INT(var)     (((var) & HOST_UI_RO_MASK_INT_HREADY_INT) >>  0U)

// HOST_UI_INT_CLR bit def
// check_int_clr
#define HOST_UI_WO_DEF_INT_CLR_CHECK_INT_CLR       0x0            // 1-bit
#define HOST_UI_WO_MASK_INT_CLR_CHECK_INT_CLR      ((0x00000001U) <<  2U)
#define HOST_UI_WO_INT_CLR_CHECK_INT_CLR(var)      (((var) & 0x00000001U) <<  2U)
#define HOST_UI_WO_SFT_INT_CLR_CHECK_INT_CLR        2
#define HOST_UI_WO_GET_INT_CLR_CHECK_INT_CLR(var)  (((var) & HOST_UI_WO_MASK_INT_CLR_CHECK_INT_CLR) >>  2U)
// hresp_int_clr
#define HOST_UI_WO_DEF_INT_CLR_HRESP_INT_CLR       0x0            // 1-bit
#define HOST_UI_WO_MASK_INT_CLR_HRESP_INT_CLR      ((0x00000001U) <<  1U)
#define HOST_UI_WO_INT_CLR_HRESP_INT_CLR(var)      (((var) & 0x00000001U) <<  1U)
#define HOST_UI_WO_SFT_INT_CLR_HRESP_INT_CLR        1
#define HOST_UI_WO_GET_INT_CLR_HRESP_INT_CLR(var)  (((var) & HOST_UI_WO_MASK_INT_CLR_HRESP_INT_CLR) >>  1U)
// hready_int_clr
#define HOST_UI_WO_DEF_INT_CLR_HREADY_INT_CLR      0x0            // 1-bit
#define HOST_UI_WO_MASK_INT_CLR_HREADY_INT_CLR     ((0x00000001U) <<  0U)
#define HOST_UI_WO_INT_CLR_HREADY_INT_CLR(var)     (((var) & 0x00000001U) <<  0U)
#define HOST_UI_WO_SFT_INT_CLR_HREADY_INT_CLR       0
#define HOST_UI_WO_GET_INT_CLR_HREADY_INT_CLR(var) (((var) & HOST_UI_WO_MASK_INT_CLR_HREADY_INT_CLR) >>  0U)

// HOST_UI_INT_EN bit def
// check_int_en
#define HOST_UI_RW_DEF_INT_EN_CHECK_INT_EN        0x1            // 1-bit
#define HOST_UI_RW_MASK_INT_EN_CHECK_INT_EN       ((0x00000001U) <<  2U)
#define HOST_UI_RW_INT_EN_CHECK_INT_EN(var)       (((var) & 0x00000001U) <<  2U)
#define HOST_UI_RW_SFT_INT_EN_CHECK_INT_EN         2
#define HOST_UI_RW_GET_INT_EN_CHECK_INT_EN(var)   (((var) & HOST_UI_RW_MASK_INT_EN_CHECK_INT_EN) >>  2U)
// hresp_int_en
#define HOST_UI_RW_DEF_INT_EN_HRESP_INT_EN        0x1            // 1-bit
#define HOST_UI_RW_MASK_INT_EN_HRESP_INT_EN       ((0x00000001U) <<  1U)
#define HOST_UI_RW_INT_EN_HRESP_INT_EN(var)       (((var) & 0x00000001U) <<  1U)
#define HOST_UI_RW_SFT_INT_EN_HRESP_INT_EN         1
#define HOST_UI_RW_GET_INT_EN_HRESP_INT_EN(var)   (((var) & HOST_UI_RW_MASK_INT_EN_HRESP_INT_EN) >>  1U)
// hready_int_en
#define HOST_UI_RW_DEF_INT_EN_HREADY_INT_EN       0x1            // 1-bit
#define HOST_UI_RW_MASK_INT_EN_HREADY_INT_EN      ((0x00000001U) <<  0U)
#define HOST_UI_RW_INT_EN_HREADY_INT_EN(var)      (((var) & 0x00000001U) <<  0U)
#define HOST_UI_RW_SFT_INT_EN_HREADY_INT_EN        0
#define HOST_UI_RW_GET_INT_EN_HREADY_INT_EN(var)  (((var) & HOST_UI_RW_MASK_INT_EN_HREADY_INT_EN) >>  0U)


//--------------------------------
// module: drvctr
//--------------------------------
// DRVCTR_FUNC_CONFIG bit def
// ch_en
#define DRVCTR_RW_DEF_FUNC_CONFIG_CH_EN                       0x1            // 24-bit
#define DRVCTR_RW_MASK_FUNC_CONFIG_CH_EN                      ((0x00FFFFFFU) <<  8U)
#define DRVCTR_RW_FUNC_CONFIG_CH_EN(var)                      (((var) & 0x00FFFFFFU) <<  8U)
#define DRVCTR_RW_SFT_FUNC_CONFIG_CH_EN                        8
#define DRVCTR_RW_GET_FUNC_CONFIG_CH_EN(var)                  (((var) & DRVCTR_RW_MASK_FUNC_CONFIG_CH_EN) >>  8U)
// ovt150_frame_sync
#define DRVCTR_RW_DEF_FUNC_CONFIG_OVT150_FRAME_SYNC           0x0            // 1-bit
#define DRVCTR_RW_MASK_FUNC_CONFIG_OVT150_FRAME_SYNC          ((0x00000001U) <<  5U)
#define DRVCTR_RW_FUNC_CONFIG_OVT150_FRAME_SYNC(var)          (((var) & 0x00000001U) <<  5U)
#define DRVCTR_RW_SFT_FUNC_CONFIG_OVT150_FRAME_SYNC            5
#define DRVCTR_RW_GET_FUNC_CONFIG_OVT150_FRAME_SYNC(var)      (((var) & DRVCTR_RW_MASK_FUNC_CONFIG_OVT150_FRAME_SYNC) >>  5U)
// reg_comp_check
#define DRVCTR_RW_DEF_FUNC_CONFIG_REG_COMP_CHECK              0x1            // 1-bit
#define DRVCTR_RW_MASK_FUNC_CONFIG_REG_COMP_CHECK             ((0x00000001U) <<  4U)
#define DRVCTR_RW_FUNC_CONFIG_REG_COMP_CHECK(var)             (((var) & 0x00000001U) <<  4U)
#define DRVCTR_RW_SFT_FUNC_CONFIG_REG_COMP_CHECK               4
#define DRVCTR_RW_GET_FUNC_CONFIG_REG_COMP_CHECK(var)         (((var) & DRVCTR_RW_MASK_FUNC_CONFIG_REG_COMP_CHECK) >>  4U)
// tdet_from_slotsel_0or1
#define DRVCTR_RW_DEF_FUNC_CONFIG_TDET_FROM_SLOTSEL_0OR1      0x1            // 1-bit
#define DRVCTR_RW_MASK_FUNC_CONFIG_TDET_FROM_SLOTSEL_0OR1     ((0x00000001U) <<  3U)
#define DRVCTR_RW_FUNC_CONFIG_TDET_FROM_SLOTSEL_0OR1(var)     (((var) & 0x00000001U) <<  3U)
#define DRVCTR_RW_SFT_FUNC_CONFIG_TDET_FROM_SLOTSEL_0OR1       3
#define DRVCTR_RW_GET_FUNC_CONFIG_TDET_FROM_SLOTSEL_0OR1(var) (((var) & DRVCTR_RW_MASK_FUNC_CONFIG_TDET_FROM_SLOTSEL_0OR1) >>  3U)
// det_out_polarity
#define DRVCTR_RW_DEF_FUNC_CONFIG_DET_OUT_POLARITY            0x0            // 1-bit
#define DRVCTR_RW_MASK_FUNC_CONFIG_DET_OUT_POLARITY           ((0x00000001U) <<  2U)
#define DRVCTR_RW_FUNC_CONFIG_DET_OUT_POLARITY(var)           (((var) & 0x00000001U) <<  2U)
#define DRVCTR_RW_SFT_FUNC_CONFIG_DET_OUT_POLARITY             2
#define DRVCTR_RW_GET_FUNC_CONFIG_DET_OUT_POLARITY(var)       (((var) & DRVCTR_RW_MASK_FUNC_CONFIG_DET_OUT_POLARITY) >>  2U)
// det_en
#define DRVCTR_RW_DEF_FUNC_CONFIG_DET_EN                      0x0            // 1-bit
#define DRVCTR_RW_MASK_FUNC_CONFIG_DET_EN                     ((0x00000001U) <<  1U)
#define DRVCTR_RW_FUNC_CONFIG_DET_EN(var)                     (((var) & 0x00000001U) <<  1U)
#define DRVCTR_RW_SFT_FUNC_CONFIG_DET_EN                       1
#define DRVCTR_RW_GET_FUNC_CONFIG_DET_EN(var)                 (((var) & DRVCTR_RW_MASK_FUNC_CONFIG_DET_EN) >>  1U)
// drvctr_en
#define DRVCTR_RW_DEF_FUNC_CONFIG_DRVCTR_EN                   0x0            // 1-bit
#define DRVCTR_RW_MASK_FUNC_CONFIG_DRVCTR_EN                  ((0x00000001U) <<  0U)
#define DRVCTR_RW_FUNC_CONFIG_DRVCTR_EN(var)                  (((var) & 0x00000001U) <<  0U)
#define DRVCTR_RW_SFT_FUNC_CONFIG_DRVCTR_EN                    0
#define DRVCTR_RW_GET_FUNC_CONFIG_DRVCTR_EN(var)              (((var) & DRVCTR_RW_MASK_FUNC_CONFIG_DRVCTR_EN) >>  0U)

// DRVCTR_MODE_CONFIG1 bit def
// ch23_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG1_CH23_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG1_CH23_SLOT_SEL                ((0x00000003U) <<  14U)
#define DRVCTR_RW_MODE_CONFIG1_CH23_SLOT_SEL(var)                (((var) & 0x00000003U) <<  14U)
#define DRVCTR_RW_SFT_MODE_CONFIG1_CH23_SLOT_SEL                  14
#define DRVCTR_RW_GET_MODE_CONFIG1_CH23_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG1_CH23_SLOT_SEL) >>  14U)
// ch22_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG1_CH22_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG1_CH22_SLOT_SEL                ((0x00000003U) <<  12U)
#define DRVCTR_RW_MODE_CONFIG1_CH22_SLOT_SEL(var)                (((var) & 0x00000003U) <<  12U)
#define DRVCTR_RW_SFT_MODE_CONFIG1_CH22_SLOT_SEL                  12
#define DRVCTR_RW_GET_MODE_CONFIG1_CH22_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG1_CH22_SLOT_SEL) >>  12U)
// ch21_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG1_CH21_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG1_CH21_SLOT_SEL                ((0x00000003U) << 10U)
#define DRVCTR_RW_MODE_CONFIG1_CH21_SLOT_SEL(var)                (((var) & 0x00000003U) << 10U)
#define DRVCTR_RW_SFT_MODE_CONFIG1_CH21_SLOT_SEL                 10
#define DRVCTR_RW_GET_MODE_CONFIG1_CH21_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG1_CH21_SLOT_SEL) >> 10U)
// ch20_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG1_CH20_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG1_CH20_SLOT_SEL                ((0x00000003U) <<  8U)
#define DRVCTR_RW_MODE_CONFIG1_CH20_SLOT_SEL(var)                (((var) & 0x00000003U) <<  8U)
#define DRVCTR_RW_SFT_MODE_CONFIG1_CH20_SLOT_SEL                  8
#define DRVCTR_RW_GET_MODE_CONFIG1_CH20_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG1_CH20_SLOT_SEL) >>  8U)
// ch19_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG1_CH19_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG1_CH19_SLOT_SEL                ((0x00000003U) <<  6U)
#define DRVCTR_RW_MODE_CONFIG1_CH19_SLOT_SEL(var)                (((var) & 0x00000003U) <<  6U)
#define DRVCTR_RW_SFT_MODE_CONFIG1_CH19_SLOT_SEL                  6
#define DRVCTR_RW_GET_MODE_CONFIG1_CH19_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG1_CH19_SLOT_SEL) >>  6U)
// ch18_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG1_CH18_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG1_CH18_SLOT_SEL                ((0x00000003U) <<  4U)
#define DRVCTR_RW_MODE_CONFIG1_CH18_SLOT_SEL(var)                (((var) & 0x00000003U) <<  4U)
#define DRVCTR_RW_SFT_MODE_CONFIG1_CH18_SLOT_SEL                  4
#define DRVCTR_RW_GET_MODE_CONFIG1_CH18_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG1_CH18_SLOT_SEL) >>  4U)
// ch17_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG1_CH17_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG1_CH17_SLOT_SEL                ((0x00000003U) <<  2U)
#define DRVCTR_RW_MODE_CONFIG1_CH17_SLOT_SEL(var)                (((var) & 0x00000003U) <<  2U)
#define DRVCTR_RW_SFT_MODE_CONFIG1_CH17_SLOT_SEL                  2
#define DRVCTR_RW_GET_MODE_CONFIG1_CH17_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG1_CH17_SLOT_SEL) >>  2U)
// ch16_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG1_CH16_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG1_CH16_SLOT_SEL                ((0x00000003U) <<  0U)
#define DRVCTR_RW_MODE_CONFIG1_CH16_SLOT_SEL(var)                (((var) & 0x00000003U) <<  0U)
#define DRVCTR_RW_SFT_MODE_CONFIG1_CH16_SLOT_SEL                  0
#define DRVCTR_RW_GET_MODE_CONFIG1_CH16_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG1_CH16_SLOT_SEL) >>  0U)

// DRVCTR_MODE_CONFIG0 bit def
// ch15_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH15_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH15_SLOT_SEL                ((0x00000003U) <<  30U)
#define DRVCTR_RW_MODE_CONFIG0_CH15_SLOT_SEL(var)                (((var) & 0x00000003U) <<  30U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH15_SLOT_SEL                  30
#define DRVCTR_RW_GET_MODE_CONFIG0_CH15_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH15_SLOT_SEL) >>  30U)
// ch14_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH14_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH14_SLOT_SEL                ((0x00000003U) <<  28U)
#define DRVCTR_RW_MODE_CONFIG0_CH14_SLOT_SEL(var)                (((var) & 0x00000003U) <<  28U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH14_SLOT_SEL                  28
#define DRVCTR_RW_GET_MODE_CONFIG0_CH14_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH14_SLOT_SEL) >>  28U)
// ch13_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH13_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH13_SLOT_SEL                ((0x00000003U) <<  26U)
#define DRVCTR_RW_MODE_CONFIG0_CH13_SLOT_SEL(var)                (((var) & 0x00000003U) <<  26U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH13_SLOT_SEL                  26
#define DRVCTR_RW_GET_MODE_CONFIG0_CH13_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH13_SLOT_SEL) >>  26U)
// ch12_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH12_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH12_SLOT_SEL                ((0x00000003U) <<  24U)
#define DRVCTR_RW_MODE_CONFIG0_CH12_SLOT_SEL(var)                (((var) & 0x00000003U) <<  24U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH12_SLOT_SEL                  24
#define DRVCTR_RW_GET_MODE_CONFIG0_CH12_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH12_SLOT_SEL) >>  24U)
// ch11_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH11_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH11_SLOT_SEL                ((0x00000003U) <<  22U)
#define DRVCTR_RW_MODE_CONFIG0_CH11_SLOT_SEL(var)                (((var) & 0x00000003U) <<  22U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH11_SLOT_SEL                  22
#define DRVCTR_RW_GET_MODE_CONFIG0_CH11_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH11_SLOT_SEL) >>  22U)
// ch10_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH10_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH10_SLOT_SEL                ((0x00000003U) <<  20U)
#define DRVCTR_RW_MODE_CONFIG0_CH10_SLOT_SEL(var)                (((var) & 0x00000003U) <<  20U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH10_SLOT_SEL                  20
#define DRVCTR_RW_GET_MODE_CONFIG0_CH10_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH10_SLOT_SEL) >>  20U)
// ch09_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH09_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH09_SLOT_SEL                ((0x00000003U) <<  18U)
#define DRVCTR_RW_MODE_CONFIG0_CH09_SLOT_SEL(var)                (((var) & 0x00000003U) <<  18U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH09_SLOT_SEL                  18
#define DRVCTR_RW_GET_MODE_CONFIG0_CH09_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH09_SLOT_SEL) >>  18U)
// ch08_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH08_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH08_SLOT_SEL                ((0x00000003U) <<  16U)
#define DRVCTR_RW_MODE_CONFIG0_CH08_SLOT_SEL(var)                (((var) & 0x00000003U) <<  16U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH08_SLOT_SEL                  16
#define DRVCTR_RW_GET_MODE_CONFIG0_CH08_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH08_SLOT_SEL) >>  16U)
// ch07_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH07_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH07_SLOT_SEL                ((0x00000003U) <<  14U)
#define DRVCTR_RW_MODE_CONFIG0_CH07_SLOT_SEL(var)                (((var) & 0x00000003U) <<  14U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH07_SLOT_SEL                  14
#define DRVCTR_RW_GET_MODE_CONFIG0_CH07_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH07_SLOT_SEL) >>  14U)
// ch06_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH06_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH06_SLOT_SEL                ((0x00000003U) <<  12U)
#define DRVCTR_RW_MODE_CONFIG0_CH06_SLOT_SEL(var)                (((var) & 0x00000003U) <<  12U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH06_SLOT_SEL                  12
#define DRVCTR_RW_GET_MODE_CONFIG0_CH06_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH06_SLOT_SEL) >>  12U)
// ch05_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH05_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH05_SLOT_SEL                ((0x00000003U) <<  10U)
#define DRVCTR_RW_MODE_CONFIG0_CH05_SLOT_SEL(var)                (((var) & 0x00000003U) <<  10U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH05_SLOT_SEL                  10
#define DRVCTR_RW_GET_MODE_CONFIG0_CH05_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH05_SLOT_SEL) >>  10U)
// ch04_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH04_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH04_SLOT_SEL                ((0x00000003U) <<  8U)
#define DRVCTR_RW_MODE_CONFIG0_CH04_SLOT_SEL(var)                (((var) & 0x00000003U) <<  8U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH04_SLOT_SEL                  8
#define DRVCTR_RW_GET_MODE_CONFIG0_CH04_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH04_SLOT_SEL) >>  8U)
// ch03_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH03_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH03_SLOT_SEL                ((0x00000003U) <<  6U)
#define DRVCTR_RW_MODE_CONFIG0_CH03_SLOT_SEL(var)                (((var) & 0x00000003U) <<  6U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH03_SLOT_SEL                  6
#define DRVCTR_RW_GET_MODE_CONFIG0_CH03_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH03_SLOT_SEL) >>  6U)
// ch02_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH02_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH02_SLOT_SEL                ((0x00000003U) <<  4U)
#define DRVCTR_RW_MODE_CONFIG0_CH02_SLOT_SEL(var)                (((var) & 0x00000003U) <<  4U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH02_SLOT_SEL                  4
#define DRVCTR_RW_GET_MODE_CONFIG0_CH02_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH02_SLOT_SEL) >>  4U)
// ch01_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH01_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH01_SLOT_SEL                ((0x00000003U) <<  2U)
#define DRVCTR_RW_MODE_CONFIG0_CH01_SLOT_SEL(var)                (((var) & 0x00000003U) <<  2U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH01_SLOT_SEL                  2
#define DRVCTR_RW_GET_MODE_CONFIG0_CH01_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH01_SLOT_SEL) >>  2U)
// ch00_slot_sel
#define DRVCTR_RW_DEF_MODE_CONFIG0_CH00_SLOT_SEL                 0x0            // 2-bit
#define DRVCTR_RW_MASK_MODE_CONFIG0_CH00_SLOT_SEL                ((0x00000003U) <<  0U)
#define DRVCTR_RW_MODE_CONFIG0_CH00_SLOT_SEL(var)                (((var) & 0x00000003U) <<  0U)
#define DRVCTR_RW_SFT_MODE_CONFIG0_CH00_SLOT_SEL                  0
#define DRVCTR_RW_GET_MODE_CONFIG0_CH00_SLOT_SEL(var)            (((var) & DRVCTR_RW_MASK_MODE_CONFIG0_CH00_SLOT_SEL) >>  0U)

// DRVCTR_POWER_CONFIG_HL_CH23_16 bit def
// ch_power_hl_cfg
#define DRVCTR_RW_DEF_POWER_CONFIG_HL_CH23_16_CH_POWER_HL_CFG             0x0            // 32-bit
#define DRVCTR_RW_MASK_POWER_CONFIG_HL_CH23_16_CH_POWER_HL_CFG            ((0xFFFFFFFFU) <<  0U)
#define DRVCTR_RW_POWER_CONFIG_HL_CH23_16_CH_POWER_HL_CFG(var)            (((var) & 0xFFFFFFFFU) <<  0U)
#define DRVCTR_RW_SFT_POWER_CONFIG_HL_CH23_16_CH_POWER_HL_CFG              0
#define DRVCTR_RW_GET_POWER_CONFIG_HL_CH23_16_CH_POWER_HL_CFG(var)        (((var) & DRVCTR_RW_MASK_POWER_CONFIG_HL_CH23_16_CH_POWER_HL_CFG) >>  0U)

// DRVCTR_POWER_CONFIG_HL_CH15_8 bit def
// ch_power_hl_cfg
#define DRVCTR_RW_DEF_POWER_CONFIG_HL_CH15_8_CH_POWER_HL_CFG             0x0            // 32-bit
#define DRVCTR_RW_MASK_POWER_CONFIG_HL_CH15_8_CH_POWER_HL_CFG            ((0xFFFFFFFFU) <<  0U)
#define DRVCTR_RW_POWER_CONFIG_HL_CH15_8_CH_POWER_HL_CFG(var)            (((var) & 0xFFFFFFFFU) <<  0U)
#define DRVCTR_RW_SFT_POWER_CONFIG_HL_CH15_8_CH_POWER_HL_CFG              0
#define DRVCTR_RW_GET_POWER_CONFIG_HL_CH15_8_CH_POWER_HL_CFG(var)        (((var) & DRVCTR_RW_MASK_POWER_CONFIG_HL_CH15_8_CH_POWER_HL_CFG) >>  0U)

// DRVCTR_POWER_CONFIG_HL_CH7_0 bit def
// ch_power_hl_cfg
#define DRVCTR_RW_DEF_POWER_CONFIG_HL_CH7_0_CH_POWER_HL_CFG             0x0            // 32-bit
#define DRVCTR_RW_MASK_POWER_CONFIG_HL_CH7_0_CH_POWER_HL_CFG            ((0xFFFFFFFFU) <<  0U)
#define DRVCTR_RW_POWER_CONFIG_HL_CH7_0_CH_POWER_HL_CFG(var)            (((var) & 0xFFFFFFFFU) <<  0U)
#define DRVCTR_RW_SFT_POWER_CONFIG_HL_CH7_0_CH_POWER_HL_CFG              0
#define DRVCTR_RW_GET_POWER_CONFIG_HL_CH7_0_CH_POWER_HL_CFG(var)        (((var) & DRVCTR_RW_MASK_POWER_CONFIG_HL_CH7_0_CH_POWER_HL_CFG) >>  0U)

// DRVCTR_POWER_CONFIG_SEL bit def
// ch_power_hl_cfg_uni
#define DRVCTR_RW_DEF_POWER_CONFIG_SEL_CH_POWER_HL_CFG_UNI         0x0            // 4-bit
#define DRVCTR_RW_MASK_POWER_CONFIG_SEL_CH_POWER_HL_CFG_UNI        ((0x0000000FU) << 24U)
#define DRVCTR_RW_POWER_CONFIG_SEL_CH_POWER_HL_CFG_UNI(var)        (((var) & 0x0000000FU) << 24U)
#define DRVCTR_RW_SFT_POWER_CONFIG_SEL_CH_POWER_HL_CFG_UNI         24
#define DRVCTR_RW_GET_POWER_CONFIG_SEL_CH_POWER_HL_CFG_UNI(var)    (((var) & DRVCTR_RW_MASK_POWER_CONFIG_SEL_CH_POWER_HL_CFG_UNI) >> 24U)
// ch_power_sel
#define DRVCTR_RW_DEF_POWER_CONFIG_SEL_CH_POWER_SEL                0x0            // 24-bit
#define DRVCTR_RW_MASK_POWER_CONFIG_SEL_CH_POWER_SEL               ((0x00FFFFFFU) <<  0U)
#define DRVCTR_RW_POWER_CONFIG_SEL_CH_POWER_SEL(var)               (((var) & 0x00FFFFFFU) <<  0U)
#define DRVCTR_RW_SFT_POWER_CONFIG_SEL_CH_POWER_SEL                 0
#define DRVCTR_RW_GET_POWER_CONFIG_SEL_CH_POWER_SEL(var)           (((var) & DRVCTR_RW_MASK_POWER_CONFIG_SEL_CH_POWER_SEL) >>  0U)

// DRVCTR_POWER_CONFIG_PWP bit def
// ch_pw_cfg
#define DRVCTR_RW_DEF_POWER_CONFIG_PWP_CH_PW_CFG                   0x0            // 2-bit
#define DRVCTR_RW_MASK_POWER_CONFIG_PWP_CH_PW_CFG                  ((0x00000003U) <<  0U)
#define DRVCTR_RW_POWER_CONFIG_PWP_CH_PW_CFG(var)                  (((var) & 0x00000003U) <<  0U)
#define DRVCTR_RW_SFT_POWER_CONFIG_PWP_CH_PW_CFG                    0
#define DRVCTR_RW_GET_POWER_CONFIG_PWP_CH_PW_CFG(var)              (((var) & DRVCTR_RW_MASK_POWER_CONFIG_PWP_CH_PW_CFG) >>  0U)

// DRVCTR_CH_D2A_RB_HL bit def
// ch_rb_data_hl
#define DRVCTR_RO_DEF_CH_D2A_RB_HL_CH_RB_DATA_HL               0x0            // 24-bit
#define DRVCTR_RO_MASK_CH_D2A_RB_HL_CH_RB_DATA_HL              ((0x00FFFFFFU) <<  0U)
#define DRVCTR_RO_CH_D2A_RB_HL_CH_RB_DATA_HL(var)              (((var) & 0x00FFFFFFU) <<  0U)
#define DRVCTR_RO_SFT_CH_D2A_RB_HL_CH_RB_DATA_HL                0
#define DRVCTR_RO_GET_CH_D2A_RB_HL_CH_RB_DATA_HL(var)          (((var) & DRVCTR_RO_MASK_CH_D2A_RB_HL_CH_RB_DATA_HL) >>  0U)

// DRVCTR_CH_D2A_RB_OE bit def
// ch_rb_data_oe
#define DRVCTR_RO_DEF_CH_D2A_RB_OE_CH_RB_DATA_OE               0x0            // 24-bit
#define DRVCTR_RO_MASK_CH_D2A_RB_OE_CH_RB_DATA_OE              ((0x00FFFFFFU) <<  0U)
#define DRVCTR_RO_CH_D2A_RB_OE_CH_RB_DATA_OE(var)              (((var) & 0x00FFFFFFU) <<  0U)
#define DRVCTR_RO_SFT_CH_D2A_RB_OE_CH_RB_DATA_OE                0
#define DRVCTR_RO_GET_CH_D2A_RB_OE_CH_RB_DATA_OE(var)          (((var) & DRVCTR_RO_MASK_CH_D2A_RB_OE_CH_RB_DATA_OE) >>  0U)

// DRVCTR_DET_DAC_DONFIG bit def
// DET_DAC
#define DRVCTR_RW_DEF_DET_DAC_DONFIG_DET_DAC                     0x0            // 6-bit
#define DRVCTR_RW_MASK_DET_DAC_DONFIG_DET_DAC                    ((0x0000003FU) <<  0U)
#define DRVCTR_RW_DET_DAC_DONFIG_DET_DAC(var)                    (((var) & 0x0000003FU) <<  0U)
#define DRVCTR_RW_SFT_DET_DAC_DONFIG_DET_DAC                      0
#define DRVCTR_RW_GET_DET_DAC_DONFIG_DET_DAC(var)                (((var) & DRVCTR_RW_MASK_DET_DAC_DONFIG_DET_DAC) >>  0U)

// DRVCTR_DET_TIME_CONFIG bit def
// tdet_vcsel
#define DRVCTR_RW_DEF_DET_TIME_CONFIG_TDET_VCSEL                  0x1400         // 16-bit
#define DRVCTR_RW_MASK_DET_TIME_CONFIG_TDET_VCSEL                 ((0x0000FFFFU) << 16U)
#define DRVCTR_RW_DET_TIME_CONFIG_TDET_VCSEL(var)                 (((var) & 0x0000FFFFU) << 16U)
#define DRVCTR_RW_SFT_DET_TIME_CONFIG_TDET_VCSEL                  16
#define DRVCTR_RW_GET_DET_TIME_CONFIG_TDET_VCSEL(var)             (((var) & DRVCTR_RW_MASK_DET_TIME_CONFIG_TDET_VCSEL) >> 16U)
// tdet_ldmos
#define DRVCTR_RW_DEF_DET_TIME_CONFIG_TDET_LDMOS                  0x1400         // 16-bit
#define DRVCTR_RW_MASK_DET_TIME_CONFIG_TDET_LDMOS                 ((0x0000FFFFU) <<  0U)
#define DRVCTR_RW_DET_TIME_CONFIG_TDET_LDMOS(var)                 (((var) & 0x0000FFFFU) <<  0U)
#define DRVCTR_RW_SFT_DET_TIME_CONFIG_TDET_LDMOS                   0
#define DRVCTR_RW_GET_DET_TIME_CONFIG_TDET_LDMOS(var)             (((var) & DRVCTR_RW_MASK_DET_TIME_CONFIG_TDET_LDMOS) >>  0U)

// DRVCTR_DET_MODE_TRIG bit def
// single_det_trig
#define DRVCTR_WO_DEF_DET_MODE_TRIG_SINGLE_DET_TRIG             0x0            // 1-bit
#define DRVCTR_WO_MASK_DET_MODE_TRIG_SINGLE_DET_TRIG            ((0x00000001U) <<  0U)
#define DRVCTR_WO_DET_MODE_TRIG_SINGLE_DET_TRIG(var)            (((var) & 0x00000001U) <<  0U)
#define DRVCTR_WO_SFT_DET_MODE_TRIG_SINGLE_DET_TRIG              0
#define DRVCTR_WO_GET_DET_MODE_TRIG_SINGLE_DET_TRIG(var)        (((var) & DRVCTR_WO_MASK_DET_MODE_TRIG_SINGLE_DET_TRIG) >>  0U)

// DRVCTR_ERROR bit def
// err_ch_oe
#define DRVCTR_RO_DEF_ERROR_ERR_CH_OE                   0x0            // 1-bit
#define DRVCTR_RO_MASK_ERROR_ERR_CH_OE                  ((0x00000001U) <<  2U)
#define DRVCTR_RO_ERROR_ERR_CH_OE(var)                  (((var) & 0x00000001U) <<  2U)
#define DRVCTR_RO_SFT_ERROR_ERR_CH_OE                    2
#define DRVCTR_RO_GET_ERROR_ERR_CH_OE(var)              (((var) & DRVCTR_RO_MASK_ERROR_ERR_CH_OE) >>  2U)
// err_reg_fusa
#define DRVCTR_RO_DEF_ERROR_ERR_REG_FUSA                0x0            // 1-bit
#define DRVCTR_RO_MASK_ERROR_ERR_REG_FUSA               ((0x00000001U) <<  1U)
#define DRVCTR_RO_ERROR_ERR_REG_FUSA(var)               (((var) & 0x00000001U) <<  1U)
#define DRVCTR_RO_SFT_ERROR_ERR_REG_FUSA                 1
#define DRVCTR_RO_GET_ERROR_ERR_REG_FUSA(var)           (((var) & DRVCTR_RO_MASK_ERROR_ERR_REG_FUSA) >>  1U)
// err_det
#define DRVCTR_RO_DEF_ERROR_ERR_DET                     0x0            // 1-bit
#define DRVCTR_RO_MASK_ERROR_ERR_DET                    ((0x00000001U) <<  0U)
#define DRVCTR_RO_ERROR_ERR_DET(var)                    (((var) & 0x00000001U) <<  0U)
#define DRVCTR_RO_SFT_ERROR_ERR_DET                      0
#define DRVCTR_RO_GET_ERROR_ERR_DET(var)                (((var) & DRVCTR_RO_MASK_ERROR_ERR_DET) >>  0U)

// DRVCTR_DET_RESULT bit def
// row_cnt
#define DRVCTR_RO_DEF_DET_RESULT_ROW_CNT                     0x3F           // 6-bit
#define DRVCTR_RO_MASK_DET_RESULT_ROW_CNT                    ((0x0000003FU) << 24U)
#define DRVCTR_RO_DET_RESULT_ROW_CNT(var)                    (((var) & 0x0000003FU) << 24U)
#define DRVCTR_RO_SFT_DET_RESULT_ROW_CNT                     24
#define DRVCTR_RO_GET_DET_RESULT_ROW_CNT(var)                (((var) & DRVCTR_RO_MASK_DET_RESULT_ROW_CNT) >> 24U)
// err_det_ch
#define DRVCTR_RO_DEF_DET_RESULT_ERR_DET_CH                  0x0            // 24-bit
#define DRVCTR_RO_MASK_DET_RESULT_ERR_DET_CH                 ((0x00FFFFFFU) <<  0U)
#define DRVCTR_RO_DET_RESULT_ERR_DET_CH(var)                 (((var) & 0x00FFFFFFU) <<  0U)
#define DRVCTR_RO_SFT_DET_RESULT_ERR_DET_CH                   0
#define DRVCTR_RO_GET_DET_RESULT_ERR_DET_CH(var)             (((var) & DRVCTR_RO_MASK_DET_RESULT_ERR_DET_CH) >>  0U)


//--------------------------------
// module: otpctr
//--------------------------------
// OTPCTR_TRIG_MODE_CTRL bit def
// start_read
#define OTPCTR_WO_DEF_TRIG_MODE_CTRL_START_READ               0x0            // 1-bit
#define OTPCTR_WO_MASK_TRIG_MODE_CTRL_START_READ              ((0x00000001U) <<  0U)
#define OTPCTR_WO_TRIG_MODE_CTRL_START_READ(var)              (((var) & 0x00000001U) <<  0U)
#define OTPCTR_WO_SFT_TRIG_MODE_CTRL_START_READ                0
#define OTPCTR_WO_GET_TRIG_MODE_CTRL_START_READ(var)          (((var) & OTPCTR_WO_MASK_TRIG_MODE_CTRL_START_READ) >>  0U)

// OTPCTR_CFG0 bit def
// use_double_bit_mode
#define OTPCTR_RW_DEF_CFG0_USE_DOUBLE_BIT_MODE        0x0            // 2-bit
#define OTPCTR_RW_MASK_CFG0_USE_DOUBLE_BIT_MODE       ((0x00000003U) <<  3U)
#define OTPCTR_RW_CFG0_USE_DOUBLE_BIT_MODE(var)       (((var) & 0x00000003U) <<  3U)
#define OTPCTR_RW_SFT_CFG0_USE_DOUBLE_BIT_MODE         3
#define OTPCTR_RW_GET_CFG0_USE_DOUBLE_BIT_MODE(var)   (((var) & OTPCTR_RW_MASK_CFG0_USE_DOUBLE_BIT_MODE) >>  3U)
// use_ecc
#define OTPCTR_RW_DEF_CFG0_USE_ECC                    0x1            // 1-bit
#define OTPCTR_RW_MASK_CFG0_USE_ECC                   ((0x00000001U) <<  2U)
#define OTPCTR_RW_CFG0_USE_ECC(var)                   (((var) & 0x00000001U) <<  2U)
#define OTPCTR_RW_SFT_CFG0_USE_ECC                     2
#define OTPCTR_RW_GET_CFG0_USE_ECC(var)               (((var) & OTPCTR_RW_MASK_CFG0_USE_ECC) >>  2U)
// no_trim
#define OTPCTR_RW_DEF_CFG0_NO_TRIM                    0x0            // 1-bit
#define OTPCTR_RW_MASK_CFG0_NO_TRIM                   ((0x00000001U) <<  1U)
#define OTPCTR_RW_CFG0_NO_TRIM(var)                   (((var) & 0x00000001U) <<  1U)
#define OTPCTR_RW_SFT_CFG0_NO_TRIM                     1
#define OTPCTR_RW_GET_CFG0_NO_TRIM(var)               (((var) & OTPCTR_RW_MASK_CFG0_NO_TRIM) >>  1U)
// efuse_cs
#define OTPCTR_RW_DEF_CFG0_EFUSE_CS                   0x0            // 1-bit
#define OTPCTR_RW_MASK_CFG0_EFUSE_CS                  ((0x00000001U) <<  0U)
#define OTPCTR_RW_CFG0_EFUSE_CS(var)                  (((var) & 0x00000001U) <<  0U)
#define OTPCTR_RW_SFT_CFG0_EFUSE_CS                    0
#define OTPCTR_RW_GET_CFG0_EFUSE_CS(var)              (((var) & OTPCTR_RW_MASK_CFG0_EFUSE_CS) >>  0U)

// OTPCTR_DEBUG0 bit def
// ecc_multi_error
#define OTPCTR_RO_DEF_DEBUG0_ECC_MULTI_ERROR          0x0            // 1-bit
#define OTPCTR_RO_MASK_DEBUG0_ECC_MULTI_ERROR         ((0x00000001U) << 17U)
#define OTPCTR_RO_DEBUG0_ECC_MULTI_ERROR(var)         (((var) & 0x00000001U) << 17U)
#define OTPCTR_RO_SFT_DEBUG0_ECC_MULTI_ERROR          17
#define OTPCTR_RO_GET_DEBUG0_ECC_MULTI_ERROR(var)     (((var) & OTPCTR_RO_MASK_DEBUG0_ECC_MULTI_ERROR) >> 17U)
// ecc_single_error
#define OTPCTR_RO_DEF_DEBUG0_ECC_SINGLE_ERROR         0x0            // 1-bit
#define OTPCTR_RO_MASK_DEBUG0_ECC_SINGLE_ERROR        ((0x00000001U) << 16U)
#define OTPCTR_RO_DEBUG0_ECC_SINGLE_ERROR(var)        (((var) & 0x00000001U) << 16U)
#define OTPCTR_RO_SFT_DEBUG0_ECC_SINGLE_ERROR         16
#define OTPCTR_RO_GET_DEBUG0_ECC_SINGLE_ERROR(var)    (((var) & OTPCTR_RO_MASK_DEBUG0_ECC_SINGLE_ERROR) >> 16U)
// rd_done
#define OTPCTR_RO_DEF_DEBUG0_RD_DONE                  0x0            // 1-bit
#define OTPCTR_RO_MASK_DEBUG0_RD_DONE                 ((0x00000001U) <<  9U)
#define OTPCTR_RO_DEBUG0_RD_DONE(var)                 (((var) & 0x00000001U) <<  9U)
#define OTPCTR_RO_SFT_DEBUG0_RD_DONE                   9
#define OTPCTR_RO_GET_DEBUG0_RD_DONE(var)             (((var) & OTPCTR_RO_MASK_DEBUG0_RD_DONE) >>  9U)
// rd_busy
#define OTPCTR_RO_DEF_DEBUG0_RD_BUSY                  0x0            // 1-bit
#define OTPCTR_RO_MASK_DEBUG0_RD_BUSY                 ((0x00000001U) <<  8U)
#define OTPCTR_RO_DEBUG0_RD_BUSY(var)                 (((var) & 0x00000001U) <<  8U)
#define OTPCTR_RO_SFT_DEBUG0_RD_BUSY                   8
#define OTPCTR_RO_GET_DEBUG0_RD_BUSY(var)             (((var) & OTPCTR_RO_MASK_DEBUG0_RD_BUSY) >>  8U)

// OTPCTR_DEBUG1 bit def
// read_value
#define OTPCTR_RO_DEF_DEBUG1_READ_VALUE               0x0            // 32-bit
#define OTPCTR_RO_MASK_DEBUG1_READ_VALUE              ((0xFFFFFFFFU) << 0U)
#define OTPCTR_RO_DEBUG1_READ_VALUE(var)              (((var) & 0xFFFFFFFFU) << 0U)
#define OTPCTR_RO_SFT_DEBUG1_READ_VALUE               0
#define OTPCTR_RO_GET_DEBUG1_READ_VALUE(var)          (((var) & OTPCTR_RO_MASK_DEBUG1_READ_VALUE) >> 0U)

// OTPCTR_DEBUG2 bit def
// read_value
#define OTPCTR_RO_DEF_DEBUG2_READ_VALUE               0x0            // 32-bit
#define OTPCTR_RO_MASK_DEBUG2_READ_VALUE              ((0xFFFFFFFFU) << 0U)
#define OTPCTR_RO_DEBUG2_READ_VALUE(var)              (((var) & 0xFFFFFFFFU) << 0U)
#define OTPCTR_RO_SFT_DEBUG2_READ_VALUE               0
#define OTPCTR_RO_GET_DEBUG2_READ_VALUE(var)          (((var) & OTPCTR_RO_MASK_DEBUG2_READ_VALUE) >> 0U)

// OTPCTR_SHADOW_TRIM bit def
// default_trim
#define OTPCTR_RO_DEF_SHADOW_TRIM_DEFAULT_TRIM        0x0            // 32-bit
#define OTPCTR_RO_MASK_SHADOW_TRIM_DEFAULT_TRIM       ((0xFFFFFFFFU) << 0U)
#define OTPCTR_RO_SHADOW_TRIM_DEFAULT_TRIM(var)       (((var) & 0xFFFFFFFFU) << 0U)
#define OTPCTR_RO_SFT_SHADOW_TRIM_DEFAULT_TRIM        0
#define OTPCTR_RO_GET_SHADOW_TRIM_DEFAULT_TRIM(var)   (((var) & OTPCTR_RO_MASK_SHADOW_TRIM_DEFAULT_TRIM) >> 0U)


//--------------------------------
// module: trgcnt
//--------------------------------
// TRGCNT_MODE_CONFIG bit def
// trgcnt_ch_out_sel
#define TRGCNT_RW_DEF_MODE_CONFIG_TRGCNT_CH_OUT_SEL                0x0            // 5-bit
#define TRGCNT_RW_MASK_MODE_CONFIG_TRGCNT_CH_OUT_SEL               ((0x0000001FU) <<  8U)
#define TRGCNT_RW_MODE_CONFIG_TRGCNT_CH_OUT_SEL(var)               (((var) & 0x0000001FU) <<  8U)
#define TRGCNT_RW_SFT_MODE_CONFIG_TRGCNT_CH_OUT_SEL                 8
#define TRGCNT_RW_GET_MODE_CONFIG_TRGCNT_CH_OUT_SEL(var)           (((var) & TRGCNT_RW_MASK_MODE_CONFIG_TRGCNT_CH_OUT_SEL) >>  8U)
// slot_power_sel_filter_width
#define TRGCNT_RW_DEF_MODE_CONFIG_SLOT_POWER_SEL_FILTER_WIDTH      0x3            // 3-bit
#define TRGCNT_RW_MASK_MODE_CONFIG_SLOT_POWER_SEL_FILTER_WIDTH     ((0x00000007U) <<  4U)
#define TRGCNT_RW_MODE_CONFIG_SLOT_POWER_SEL_FILTER_WIDTH(var)     (((var) & 0x00000007U) <<  4U)
#define TRGCNT_RW_SFT_MODE_CONFIG_SLOT_POWER_SEL_FILTER_WIDTH       4
#define TRGCNT_RW_GET_MODE_CONFIG_SLOT_POWER_SEL_FILTER_WIDTH(var) (((var) & TRGCNT_RW_MASK_MODE_CONFIG_SLOT_POWER_SEL_FILTER_WIDTH) >>  4U)
// ab_frame
#define TRGCNT_RW_DEF_MODE_CONFIG_AB_FRAME                         0x1            // 1-bit
#define TRGCNT_RW_MASK_MODE_CONFIG_AB_FRAME                        ((0x00000001U) <<  0U)
#define TRGCNT_RW_MODE_CONFIG_AB_FRAME(var)                        (((var) & 0x00000001U) <<  0U)
#define TRGCNT_RW_SFT_MODE_CONFIG_AB_FRAME                          0
#define TRGCNT_RW_GET_MODE_CONFIG_AB_FRAME(var)                    (((var) & TRGCNT_RW_MASK_MODE_CONFIG_AB_FRAME) >>  0U)

// TRGCNT_TEST_MODE bit def
// test_mode_en
#define TRGCNT_RW_DEF_TEST_MODE_TEST_MODE_EN                     0x0            // 1-bit
#define TRGCNT_RW_MASK_TEST_MODE_TEST_MODE_EN                    ((0x00000001U) <<  0U)
#define TRGCNT_RW_TEST_MODE_TEST_MODE_EN(var)                    (((var) & 0x00000001U) <<  0U)
#define TRGCNT_RW_SFT_TEST_MODE_TEST_MODE_EN                      0
#define TRGCNT_RW_GET_TEST_MODE_TEST_MODE_EN(var)                (((var) & TRGCNT_RW_MASK_TEST_MODE_TEST_MODE_EN) >>  0U)

// TRGCNT_MODE_EN bit def
// slotcnt_en
#define TRGCNT_RW_DEF_MODE_EN_SLOTCNT_EN                       0x0            // 1-bit
#define TRGCNT_RW_MASK_MODE_EN_SLOTCNT_EN                      ((0x00000001U) << 26U)
#define TRGCNT_RW_MODE_EN_SLOTCNT_EN(var)                      (((var) & 0x00000001U) << 26U)
#define TRGCNT_RW_SFT_MODE_EN_SLOTCNT_EN                       26
#define TRGCNT_RW_GET_MODE_EN_SLOTCNT_EN(var)                  (((var) & TRGCNT_RW_MASK_MODE_EN_SLOTCNT_EN) >> 26U)
// pwrcnt_en
#define TRGCNT_RW_DEF_MODE_EN_PWRCNT_EN                        0x0            // 1-bit
#define TRGCNT_RW_MASK_MODE_EN_PWRCNT_EN                       ((0x00000001U) << 25U)
#define TRGCNT_RW_MODE_EN_PWRCNT_EN(var)                       (((var) & 0x00000001U) << 25U)
#define TRGCNT_RW_SFT_MODE_EN_PWRCNT_EN                        25
#define TRGCNT_RW_GET_MODE_EN_PWRCNT_EN(var)                   (((var) & TRGCNT_RW_MASK_MODE_EN_PWRCNT_EN) >> 25U)
// framewd_en
#define TRGCNT_RW_DEF_MODE_EN_FRAMEWD_EN                       0x0            // 1-bit
#define TRGCNT_RW_MASK_MODE_EN_FRAMEWD_EN                      ((0x00000001U) << 24U)
#define TRGCNT_RW_MODE_EN_FRAMEWD_EN(var)                      (((var) & 0x00000001U) << 24U)
#define TRGCNT_RW_SFT_MODE_EN_FRAMEWD_EN                       24
#define TRGCNT_RW_GET_MODE_EN_FRAMEWD_EN(var)                  (((var) & TRGCNT_RW_MASK_MODE_EN_FRAMEWD_EN) >> 24U)
// trgcnt_en
#define TRGCNT_RW_DEF_MODE_EN_TRGCNT_EN                        0x0            // 24-bit
#define TRGCNT_RW_MASK_MODE_EN_TRGCNT_EN                       ((0x00FFFFFFU) <<  0U)
#define TRGCNT_RW_MODE_EN_TRGCNT_EN(var)                       (((var) & 0x00FFFFFFU) <<  0U)
#define TRGCNT_RW_SFT_MODE_EN_TRGCNT_EN                         0
#define TRGCNT_RW_GET_MODE_EN_TRGCNT_EN(var)                   (((var) & TRGCNT_RW_MASK_MODE_EN_TRGCNT_EN) >>  0U)

// TRGCNT_LDMOS_OPEN_DET bit def
// ch_out_det_en
#define TRGCNT_RW_DEF_LDMOS_OPEN_DET_CH_OUT_DET_EN                    0x0            // 24-bit
#define TRGCNT_RW_MASK_LDMOS_OPEN_DET_CH_OUT_DET_EN                   ((0x00FFFFFFU) <<  0U)
#define TRGCNT_RW_LDMOS_OPEN_DET_CH_OUT_DET_EN(var)                   (((var) & 0x00FFFFFFU) <<  0U)
#define TRGCNT_RW_SFT_LDMOS_OPEN_DET_CH_OUT_DET_EN                     0
#define TRGCNT_RW_GET_LDMOS_OPEN_DET_CH_OUT_DET_EN(var)               (((var) & TRGCNT_RW_MASK_LDMOS_OPEN_DET_CH_OUT_DET_EN) >>  0U)

// TRGCNT_TRGCNT_A1 bit def
// trgcnt_min_afrm
#define TRGCNT_RW_DEF_TRGCNT_A1_TRGCNT_MIN_AFRM                  0x0            // 18-bit
#define TRGCNT_RW_MASK_TRGCNT_A1_TRGCNT_MIN_AFRM                 ((0x0003FFFFU) <<  0U)
#define TRGCNT_RW_TRGCNT_A1_TRGCNT_MIN_AFRM(var)                 (((var) & 0x0003FFFFU) <<  0U)
#define TRGCNT_RW_SFT_TRGCNT_A1_TRGCNT_MIN_AFRM                   0
#define TRGCNT_RW_GET_TRGCNT_A1_TRGCNT_MIN_AFRM(var)             (((var) & TRGCNT_RW_MASK_TRGCNT_A1_TRGCNT_MIN_AFRM) >>  0U)

// TRGCNT_TRGCNT_A0 bit def
// trgcnt_max_afrm
#define TRGCNT_RW_DEF_TRGCNT_A0_TRGCNT_MAX_AFRM                  0x3FFFF        // 18-bit
#define TRGCNT_RW_MASK_TRGCNT_A0_TRGCNT_MAX_AFRM                 ((0x0003FFFFU) <<  0U)
#define TRGCNT_RW_TRGCNT_A0_TRGCNT_MAX_AFRM(var)                 (((var) & 0x0003FFFFU) <<  0U)
#define TRGCNT_RW_SFT_TRGCNT_A0_TRGCNT_MAX_AFRM                   0
#define TRGCNT_RW_GET_TRGCNT_A0_TRGCNT_MAX_AFRM(var)             (((var) & TRGCNT_RW_MASK_TRGCNT_A0_TRGCNT_MAX_AFRM) >>  0U)

// TRGCNT_TRGCNT_B1 bit def
// trgcnt_min_bfrm
#define TRGCNT_RW_DEF_TRGCNT_B1_TRGCNT_MIN_BFRM                  0x0            // 18-bit
#define TRGCNT_RW_MASK_TRGCNT_B1_TRGCNT_MIN_BFRM                 ((0x0003FFFFU) <<  0U)
#define TRGCNT_RW_TRGCNT_B1_TRGCNT_MIN_BFRM(var)                 (((var) & 0x0003FFFFU) <<  0U)
#define TRGCNT_RW_SFT_TRGCNT_B1_TRGCNT_MIN_BFRM                   0
#define TRGCNT_RW_GET_TRGCNT_B1_TRGCNT_MIN_BFRM(var)             (((var) & TRGCNT_RW_MASK_TRGCNT_B1_TRGCNT_MIN_BFRM) >>  0U)

// TRGCNT_TRGCNT_B0 bit def
// trgcnt_max_bfrm
#define TRGCNT_RW_DEF_TRGCNT_B0_TRGCNT_MAX_BFRM                  0x3FFFF        // 18-bit
#define TRGCNT_RW_MASK_TRGCNT_B0_TRGCNT_MAX_BFRM                 ((0x0003FFFFU) <<  0U)
#define TRGCNT_RW_TRGCNT_B0_TRGCNT_MAX_BFRM(var)                 (((var) & 0x0003FFFFU) <<  0U)
#define TRGCNT_RW_SFT_TRGCNT_B0_TRGCNT_MAX_BFRM                   0
#define TRGCNT_RW_GET_TRGCNT_B0_TRGCNT_MAX_BFRM(var)             (((var) & TRGCNT_RW_MASK_TRGCNT_B0_TRGCNT_MAX_BFRM) >>  0U)

// TRGCNT_SLOTCNT_A bit def
// slotcnt_min_afrm
#define TRGCNT_RW_DEF_SLOTCNT_A_SLOTCNT_MIN_AFRM                 0x0            // 16-bit
#define TRGCNT_RW_MASK_SLOTCNT_A_SLOTCNT_MIN_AFRM                ((0x0000FFFFU) << 16U)
#define TRGCNT_RW_SLOTCNT_A_SLOTCNT_MIN_AFRM(var)                (((var) & 0x0000FFFFU) << 16U)
#define TRGCNT_RW_SFT_SLOTCNT_A_SLOTCNT_MIN_AFRM                 16
#define TRGCNT_RW_GET_SLOTCNT_A_SLOTCNT_MIN_AFRM(var)            (((var) & TRGCNT_RW_MASK_SLOTCNT_A_SLOTCNT_MIN_AFRM) >> 16U)
// slotcnt_max_afrm
#define TRGCNT_RW_DEF_SLOTCNT_A_SLOTCNT_MAX_AFRM                 0xFFFF         // 16-bit
#define TRGCNT_RW_MASK_SLOTCNT_A_SLOTCNT_MAX_AFRM                ((0x0000FFFFU) <<  0U)
#define TRGCNT_RW_SLOTCNT_A_SLOTCNT_MAX_AFRM(var)                (((var) & 0x0000FFFFU) <<  0U)
#define TRGCNT_RW_SFT_SLOTCNT_A_SLOTCNT_MAX_AFRM                  0
#define TRGCNT_RW_GET_SLOTCNT_A_SLOTCNT_MAX_AFRM(var)            (((var) & TRGCNT_RW_MASK_SLOTCNT_A_SLOTCNT_MAX_AFRM) >>  0U)

// TRGCNT_SLOTCNT_B bit def
// slotcnt_min_bfrm
#define TRGCNT_RW_DEF_SLOTCNT_B_SLOTCNT_MIN_BFRM                 0x0            // 16-bit
#define TRGCNT_RW_MASK_SLOTCNT_B_SLOTCNT_MIN_BFRM                ((0x0000FFFFU) << 16U)
#define TRGCNT_RW_SLOTCNT_B_SLOTCNT_MIN_BFRM(var)                (((var) & 0x0000FFFFU) << 16U)
#define TRGCNT_RW_SFT_SLOTCNT_B_SLOTCNT_MIN_BFRM                 16
#define TRGCNT_RW_GET_SLOTCNT_B_SLOTCNT_MIN_BFRM(var)            (((var) & TRGCNT_RW_MASK_SLOTCNT_B_SLOTCNT_MIN_BFRM) >> 16U)
// slotcnt_max_bfrm
#define TRGCNT_RW_DEF_SLOTCNT_B_SLOTCNT_MAX_BFRM                 0xFFFF         // 16-bit
#define TRGCNT_RW_MASK_SLOTCNT_B_SLOTCNT_MAX_BFRM                ((0x0000FFFFU) <<  0U)
#define TRGCNT_RW_SLOTCNT_B_SLOTCNT_MAX_BFRM(var)                (((var) & 0x0000FFFFU) <<  0U)
#define TRGCNT_RW_SFT_SLOTCNT_B_SLOTCNT_MAX_BFRM                  0
#define TRGCNT_RW_GET_SLOTCNT_B_SLOTCNT_MAX_BFRM(var)            (((var) & TRGCNT_RW_MASK_SLOTCNT_B_SLOTCNT_MAX_BFRM) >>  0U)

// TRGCNT_PWRCNT_A bit def
// pwrcnt_min_afrm
#define TRGCNT_RW_DEF_PWRCNT_A_PWRCNT_MIN_AFRM                  0x0            // 16-bit
#define TRGCNT_RW_MASK_PWRCNT_A_PWRCNT_MIN_AFRM                 ((0x0000FFFFU) << 16U)
#define TRGCNT_RW_PWRCNT_A_PWRCNT_MIN_AFRM(var)                 (((var) & 0x0000FFFFU) << 16U)
#define TRGCNT_RW_SFT_PWRCNT_A_PWRCNT_MIN_AFRM                  16
#define TRGCNT_RW_GET_PWRCNT_A_PWRCNT_MIN_AFRM(var)             (((var) & TRGCNT_RW_MASK_PWRCNT_A_PWRCNT_MIN_AFRM) >> 16U)
// pwrcnt_max_afrm
#define TRGCNT_RW_DEF_PWRCNT_A_PWRCNT_MAX_AFRM                  0xFFFF         // 16-bit
#define TRGCNT_RW_MASK_PWRCNT_A_PWRCNT_MAX_AFRM                 ((0x0000FFFFU) <<  0U)
#define TRGCNT_RW_PWRCNT_A_PWRCNT_MAX_AFRM(var)                 (((var) & 0x0000FFFFU) <<  0U)
#define TRGCNT_RW_SFT_PWRCNT_A_PWRCNT_MAX_AFRM                   0
#define TRGCNT_RW_GET_PWRCNT_A_PWRCNT_MAX_AFRM(var)             (((var) & TRGCNT_RW_MASK_PWRCNT_A_PWRCNT_MAX_AFRM) >>  0U)

// TRGCNT_PWRCNT_B bit def
// pwrcnt_min_bfrm
#define TRGCNT_RW_DEF_PWRCNT_B_PWRCNT_MIN_BFRM                  0x0            // 16-bit
#define TRGCNT_RW_MASK_PWRCNT_B_PWRCNT_MIN_BFRM                 ((0x0000FFFFU) << 16U)
#define TRGCNT_RW_PWRCNT_B_PWRCNT_MIN_BFRM(var)                 (((var) & 0x0000FFFFU) << 16U)
#define TRGCNT_RW_SFT_PWRCNT_B_PWRCNT_MIN_BFRM                  16
#define TRGCNT_RW_GET_PWRCNT_B_PWRCNT_MIN_BFRM(var)             (((var) & TRGCNT_RW_MASK_PWRCNT_B_PWRCNT_MIN_BFRM) >> 16U)
// pwrcnt_max_bfrm
#define TRGCNT_RW_DEF_PWRCNT_B_PWRCNT_MAX_BFRM                  0xFFFF         // 16-bit
#define TRGCNT_RW_MASK_PWRCNT_B_PWRCNT_MAX_BFRM                 ((0x0000FFFFU) <<  0U)
#define TRGCNT_RW_PWRCNT_B_PWRCNT_MAX_BFRM(var)                 (((var) & 0x0000FFFFU) <<  0U)
#define TRGCNT_RW_SFT_PWRCNT_B_PWRCNT_MAX_BFRM                   0
#define TRGCNT_RW_GET_PWRCNT_B_PWRCNT_MAX_BFRM(var)             (((var) & TRGCNT_RW_MASK_PWRCNT_B_PWRCNT_MAX_BFRM) >>  0U)

// TRGCNT_FRAME_WD bit def
// framewd_max
#define TRGCNT_RW_DEF_FRAME_WD_FRAMEWD_MAX                      0x6422C40      // 32-bit
#define TRGCNT_RW_MASK_FRAME_WD_FRAMEWD_MAX                     ((0xFFFFFFFFU) <<  0U)
#define TRGCNT_RW_FRAME_WD_FRAMEWD_MAX(var)                     (((var) & 0xFFFFFFFFU) <<  0U)
#define TRGCNT_RW_SFT_FRAME_WD_FRAMEWD_MAX                       0
#define TRGCNT_RW_GET_FRAME_WD_FRAMEWD_MAX(var)                 (((var) & TRGCNT_RW_MASK_FRAME_WD_FRAMEWD_MAX) >>  0U)

// TRGCNT_CHCNT_OUT bit def
// trgoutcnt_result
#define TRGCNT_RO_DEF_CHCNT_OUT_TRGOUTCNT_RESULT                 0x0            // 18-bit
#define TRGCNT_RO_MASK_CHCNT_OUT_TRGOUTCNT_RESULT                ((0x0003FFFFU) <<  0U)
#define TRGCNT_RO_CHCNT_OUT_TRGOUTCNT_RESULT(var)                (((var) & 0x0003FFFFU) <<  0U)
#define TRGCNT_RO_SFT_CHCNT_OUT_TRGOUTCNT_RESULT                  0
#define TRGCNT_RO_GET_CHCNT_OUT_TRGOUTCNT_RESULT(var)            (((var) & TRGCNT_RO_MASK_CHCNT_OUT_TRGOUTCNT_RESULT) >>  0U)

// TRGCNT_TRGCNT_OUT bit def
// trgcnt_result
#define TRGCNT_RO_DEF_TRGCNT_OUT_TRGCNT_RESULT                    0x0            // 18-bit
#define TRGCNT_RO_MASK_TRGCNT_OUT_TRGCNT_RESULT                   ((0x0003FFFFU) <<  0U)
#define TRGCNT_RO_TRGCNT_OUT_TRGCNT_RESULT(var)                   (((var) & 0x0003FFFFU) <<  0U)
#define TRGCNT_RO_SFT_TRGCNT_OUT_TRGCNT_RESULT                     0
#define TRGCNT_RO_GET_TRGCNT_OUT_TRGCNT_RESULT(var)               (((var) & TRGCNT_RO_MASK_TRGCNT_OUT_TRGCNT_RESULT) >>  0U)

// TRGCNT_SLOTCNT_OUT bit def
// slotcnt_result
#define TRGCNT_RO_DEF_SLOTCNT_OUT_SLOTCNT_RESULT                   0x0            // 16-bit
#define TRGCNT_RO_MASK_SLOTCNT_OUT_SLOTCNT_RESULT                  ((0x0000FFFFU) <<  0U)
#define TRGCNT_RO_SLOTCNT_OUT_SLOTCNT_RESULT(var)                  (((var) & 0x0000FFFFU) <<  0U)
#define TRGCNT_RO_SFT_SLOTCNT_OUT_SLOTCNT_RESULT                    0
#define TRGCNT_RO_GET_SLOTCNT_OUT_SLOTCNT_RESULT(var)              (((var) & TRGCNT_RO_MASK_SLOTCNT_OUT_SLOTCNT_RESULT) >>  0U)

// TRGCNT_PWRCNT_OUT bit def
// pwrcnt_result
#define TRGCNT_RO_DEF_PWRCNT_OUT_PWRCNT_RESULT                    0x0            // 16-bit
#define TRGCNT_RO_MASK_PWRCNT_OUT_PWRCNT_RESULT                   ((0x0000FFFFU) <<  0U)
#define TRGCNT_RO_PWRCNT_OUT_PWRCNT_RESULT(var)                   (((var) & 0x0000FFFFU) <<  0U)
#define TRGCNT_RO_SFT_PWRCNT_OUT_PWRCNT_RESULT                     0
#define TRGCNT_RO_GET_PWRCNT_OUT_PWRCNT_RESULT(var)               (((var) & TRGCNT_RO_MASK_PWRCNT_OUT_PWRCNT_RESULT) >>  0U)

// TRGCNT_STATUS_ERROR bit def
// ch_err_type
#define TRGCNT_RO_DEF_STATUS_ERROR_CH_ERR_TYPE                      0x0            // 2-bit
#define TRGCNT_RO_MASK_STATUS_ERROR_CH_ERR_TYPE                     ((0x00000003U) << 28U)
#define TRGCNT_RO_STATUS_ERROR_CH_ERR_TYPE(var)                     (((var) & 0x00000003U) << 28U)
#define TRGCNT_RO_SFT_STATUS_ERROR_CH_ERR_TYPE                      28
#define TRGCNT_RO_GET_STATUS_ERROR_CH_ERR_TYPE(var)                 (((var) & TRGCNT_RO_MASK_STATUS_ERROR_CH_ERR_TYPE) >> 28U)
// slotcnt_err
#define TRGCNT_RO_DEF_STATUS_ERROR_SLOTCNT_ERR                      0x0            // 1-bit
#define TRGCNT_RO_MASK_STATUS_ERROR_SLOTCNT_ERR                     ((0x00000001U) << 26U)
#define TRGCNT_RO_STATUS_ERROR_SLOTCNT_ERR(var)                     (((var) & 0x00000001U) << 26U)
#define TRGCNT_RO_SFT_STATUS_ERROR_SLOTCNT_ERR                      26
#define TRGCNT_RO_GET_STATUS_ERROR_SLOTCNT_ERR(var)                 (((var) & TRGCNT_RO_MASK_STATUS_ERROR_SLOTCNT_ERR) >> 26U)
// pwrcnt_err
#define TRGCNT_RO_DEF_STATUS_ERROR_PWRCNT_ERR                       0x0            // 1-bit
#define TRGCNT_RO_MASK_STATUS_ERROR_PWRCNT_ERR                      ((0x00000001U) << 25U)
#define TRGCNT_RO_STATUS_ERROR_PWRCNT_ERR(var)                      (((var) & 0x00000001U) << 25U)
#define TRGCNT_RO_SFT_STATUS_ERROR_PWRCNT_ERR                       25
#define TRGCNT_RO_GET_STATUS_ERROR_PWRCNT_ERR(var)                  (((var) & TRGCNT_RO_MASK_STATUS_ERROR_PWRCNT_ERR) >> 25U)
// framewd_err
#define TRGCNT_RO_DEF_STATUS_ERROR_FRAMEWD_ERR                      0x0            // 1-bit
#define TRGCNT_RO_MASK_STATUS_ERROR_FRAMEWD_ERR                     ((0x00000001U) << 24U)
#define TRGCNT_RO_STATUS_ERROR_FRAMEWD_ERR(var)                     (((var) & 0x00000001U) << 24U)
#define TRGCNT_RO_SFT_STATUS_ERROR_FRAMEWD_ERR                      24
#define TRGCNT_RO_GET_STATUS_ERROR_FRAMEWD_ERR(var)                 (((var) & TRGCNT_RO_MASK_STATUS_ERROR_FRAMEWD_ERR) >> 24U)
// trgcnt_err_ch
#define TRGCNT_RO_DEF_STATUS_ERROR_TRGCNT_ERR_CH                    0x0            // 24-bit
#define TRGCNT_RO_MASK_STATUS_ERROR_TRGCNT_ERR_CH                   ((0x00FFFFFFU) <<  0U)
#define TRGCNT_RO_STATUS_ERROR_TRGCNT_ERR_CH(var)                   (((var) & 0x00FFFFFFU) <<  0U)
#define TRGCNT_RO_SFT_STATUS_ERROR_TRGCNT_ERR_CH                     0
#define TRGCNT_RO_GET_STATUS_ERROR_TRGCNT_ERR_CH(var)               (((var) & TRGCNT_RO_MASK_STATUS_ERROR_TRGCNT_ERR_CH) >>  0U)



#endif /* __FL9103_REG_DEFINE_V01_00_00_H__ */
