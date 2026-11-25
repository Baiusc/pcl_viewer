#include <stdio.h>
#include <string.h>

#include "fs_err.h"
#include "fs_log.h"
#include "fs_util.h"
#include "fs_hw.h"
#include "mcu_protocol.h"
#include "FL9103.h"
#include "FL9103_reg_define.h"

#undef  LOG_TAG
#define LOG_TAG "soc.FL9103"

#define FL9103_DEFAULT_TRIM_CODE         0x003C4420

#define FS_PROTO_HEAD(cmd)               (uint16_t)(((cmd & 0xFF) << 8) | ((~cmd) & 0xFF))

static int16_t m_dev_id[FS_SENSOR_TX_LS_MAX_DEV_CNT];
static uint8_t m_ch_map[FS_SENSOR_TX_LS_MAX_DEV_CNT][FS_SENSOR_TX_LS_MAX_CH_CNT];

static uint32_t FL9103_compose_addr(fs_sensor_tx_rx_e idx, uint32_t addr)
{
    uint32_t a = addr & 0x0000FFFFu;
    uint32_t hb;

    // 如果是广播模式，直接使用 0x5A
    if (idx == FS_TX_FL9103_ALL) {
        hb = 0x5Au;
        // FS_LOGD("FL9103_compose_addr: broadcast mode");
    } else {
        // 根据当前设备索引获取对应的设备编号
        hb = (idx == FS_TX_FL9103_1) ? 0 : 1;
    }

    a |= (hb << 16);
    return a;
}

static int32_t FL9103_get_reg(fs_sensor_tx_rx_e idx, uint32_t addr, uint32_t *data)
{
    return mcu_prot_get_fl9103_reg(idx - FS_TX_FL9103_1, addr, data);
}

static int32_t FL9103_set_reg(fs_sensor_tx_rx_e idx, uint32_t addr, uint32_t data)
{
    return mcu_prot_set_fl9103_reg(idx - FS_TX_FL9103_1, addr, data);
}

int32_t FL9103_read_register(fs_sensor_tx_rx_e idx, uint32_t addr, uint32_t *data, uint32_t *dlen)
{
    int32_t err = FS_SUCCESS;
    uint32_t reg_cnt;
    int32_t i;

    if ((data == NULL) || (dlen == NULL) || (*dlen == 0)) {
        return FS_ERR_BAD_PARAMS;
    }

    reg_cnt = (*dlen);
    *dlen = 0;

    for (i = 0; i < (int32_t)reg_cnt; i++) {
        BREAK_ON_ERR(err = FL9103_get_reg(idx, addr + i * sizeof(uint32_t), &data[i]));
    }

    *dlen = i;
    return err;
}

int32_t FL9103_write_register(fs_sensor_tx_rx_e idx, uint32_t addr, const uint32_t *data, uint32_t  dlen)
{
    if (data == NULL || dlen == 0) {
        return FS_ERR_BAD_PARAMS;
    }
    return FL9103_set_reg(idx, addr, *data);
}

static int32_t FL9103_read_buffer(fs_sensor_tx_rx_e idx, uint32_t addr, void *data, uint32_t *dlen)
{
    int32_t err = FS_SUCCESS;
    return err;
}

static int32_t FL9103_write_buffer(fs_sensor_tx_rx_e idx, uint32_t addr, const void *data, uint32_t  dlen)
{
    int32_t err = FS_SUCCESS;
    return err;
}

static int32_t FL9103_decode_SN(uint32_t SN[2], char *sn_buf, uint32_t buf_size)
{
    int32_t err = FS_SUCCESS;
    char lot[6];
    uint8_t ver, wa, abc;
    bool mass;
    CHECK_PTR(SN);
    CHECK_PTR(sn_buf);
    sn_buf[0] = '\0';

    if (buf_size < 64) {
        FS_LOGE("Invalid buffer size(%u) for SN decoding.", buf_size);
        return FS_ERR_BAD_PARAMS;
    }

    // test v2.0: SN=DPK272-05_127-157(B-Y2026-W14-08642)
    // SN[0] = 0x057021C2;
    // SN[1] = 0x66D00BB4;
    // test v3.0: SN=DPK272-06_136-165(B)
    // SN[0] = 0x065021C2;
    // SN[1] = 0x0B583590;

    if ((SN[0] == 0) && (SN[1] == 0)) {
        FS_LOGE("Invalid SN read from OTP: empty!");
        return FS_ERR_NOT_FOUND;
    }

    mass = ((SN[1] >> 31) & 0x01);
    ver  = ((SN[1] >> 29) & 0x03);

    switch (ver) {
        case 0x00: {
            ver = 3;
            break;
        }

        case 0x01: {
            ver = 4;
            break;
        }

        case 0x02: {
            ver = 5;
            break;
        }

        case 0x03: {
            ver = 2;
            break;
        }

        default: {
            ver = 3;
            break;
        }
    }

    if (ver == 2) {
        lot[0] = ('D' - 'A' + 10);
        lot[1] = ('P' - 'A' + 10);
        lot[2] = ('K' - 'A' + 10);
    }
    else {
        if (ver != 3) {
            FS_LOGW("Unsupported SN version: %d, using as v3", ver);
            // return FS_ERR_UNSUPPORT;
        }

        lot[0] = (SN[1] >> 10) & 0x3F;
        lot[1] = (SN[1] >>  4) & 0x3F;
        lot[2] = (SN[0] >> 18) & 0x3F;
    }

    lot[3] = (SN[0] >> 12) & 0x3F;
    lot[4] = (SN[0] >>  6) & 0x3F;
    lot[5] = (SN[0] >>  0) & 0x3F;

    for (int32_t i = 0; i < (int32_t)(sizeof(lot) / sizeof(lot[0])); i++) {
        if (lot[i] <= 9) {
            lot[i] = lot[i] -  0 + '0';
        }
        else if (lot[i] <= 35) {
            lot[i] = lot[i] - 10 + 'A';
        }
        else {
            FS_LOGE("Invalid lot[%d] decoded from SN: %d", i, lot[i]);
            err = FS_ERR_CHECK_CRC;
        }
    }

    if (err) {
        return err;
    }

    wa = (SN[0] >> 24) & 0x1F;

    if (!mass) {
        if ((lot[3] == '2') && (lot[4] == '7') && (lot[5] == '2')) {
            if ((wa == 2) || (wa == 3) || (wa == 4)) {
                if ((ver == 2) && ((wa == 2) || (wa == 3))) {
                    abc = 'A';
                }
                else if ((ver == 3) && (wa == 4)) {
                    abc = 'A';
                }
                else { // 理论上不存在这种情况
                    // abc = ((SN[0] >> 29) & 0x07) + 'A' - 1;
                    abc = 'A';
                }
            }
            else if ((wa == 5) || (wa == 6)) {
                abc = 'B';
            }
            else if ((wa == 7) || (wa == 8)) {
                abc = 'C';
            }
            else { // 理论上不存在这种情况
                abc = ((SN[0] >> 29) & 0x07) + 'A' - 1;
            }
        }
        else {
            abc = ((SN[0] >> 29) & 0x07) + 'A' - 1;
        }

        if ((ver == 2) && ((lot[3] != '2') || (lot[4] != '7') || (lot[5] != '2'))) {
            lot[0] = '*';
            lot[1] = '*';
            lot[2] = '*';
        }
    }
    else {
        abc = ((SN[0] >> 29) & 0x07) + 'A' - 1;
    }

    if (ver == 2) {
        uint8_t w;
        w = ((SN[0] >> 18) & 0x3F);
        w = (w > 14) ? (w - 14) : 0;

        snprintf(sn_buf, buf_size, "%c%c%c%c%c%c-%02u_%03u-%03u(%c-Y%4u-W%2u-%05u)",
                lot[0], lot[1], lot[2], lot[3], lot[4], lot[5], wa, ((SN[1] >> 23) & 0x1F) + 114, ((SN[1] >> 16) & 0x7F) + 77,
                abc, ((SN[1] >> 28) & 0x0F) + 2020, w, (SN[0] >> 0) & 0xFFFF);
    }
    else {
        snprintf(sn_buf, buf_size, "%c%c%c%c%c%c-%02u_%03u-%03u(%c)",
                lot[0], lot[1], lot[2], lot[3], lot[4], lot[5], wa, ((SN[1] >> 23) & 0x1F) + 114, ((SN[1] >> 16) & 0x7F) + 77,
                abc);
    }

    return err;
}

static int32_t FL9103_load_trim(fs_sensor_tx_rx_e idx, char *sn_buf, uint32_t buf_size)
{
    int32_t err = FS_SUCCESS;

    do {
        uint32_t trim, val, SN[2];
        char lot[6];
        uint8_t ver, wa, abc;
        bool mass;
        // load SN
        BREAK_ON_ERR(err = FL9103_set_reg(idx, OTPCTR_CFG0, OTPCTR_RW_CFG0_USE_DOUBLE_BIT_MODE(0) | OTPCTR_RW_CFG0_USE_ECC(0) | OTPCTR_RW_CFG0_NO_TRIM(1) | OTPCTR_RW_CFG0_EFUSE_CS(1)));
        BREAK_ON_ERR(err = FL9103_set_reg(idx, OTPCTR_TRIG_MODE_CTRL, OTPCTR_WO_TRIG_MODE_CTRL_START_READ(1)));
        BREAK_ON_ERR(err = FL9103_get_reg(idx, OTPCTR_DEBUG0, &val));
        fs_hw_msleep(2);

        if (OTPCTR_RO_GET_DEBUG0_RD_DONE(val) != 1) {
            FS_LOGE("Can't wait OTP read SN done, val=0x%08X", val);
            err = FS_ERR_INTERNAL;
            break;
        }

        val = 2;
        BREAK_ON_ERR(err = FL9103_read_register(idx, OTPCTR_DEBUG1, SN, &val));

        if ((SN[0] == 0) && (SN[1] == 0)) {
            FS_LOGE("Invalid SN read from OTP: empty!");
            err = FS_ERR_NOT_FOUND;
            break;
        }

        if (sn_buf && buf_size > 0) {
            BREAK_ON_ERR(err = FL9103_decode_SN(SN, sn_buf, buf_size));
            FS_LOGD("FL9103[%d] SN=%s", idx - FS_TX_FL9103_1, sn_buf);
        }

        // load trim
        BREAK_ON_ERR(err = FL9103_set_reg(idx, OTPCTR_CFG0, OTPCTR_RW_CFG0_USE_DOUBLE_BIT_MODE(1) | OTPCTR_RW_CFG0_USE_ECC(1) | OTPCTR_RW_CFG0_NO_TRIM(0) | OTPCTR_RW_CFG0_EFUSE_CS(0)));
        BREAK_ON_ERR(err = FL9103_set_reg(idx, OTPCTR_TRIG_MODE_CTRL, OTPCTR_WO_TRIG_MODE_CTRL_START_READ(1)));
        BREAK_ON_ERR(err = FL9103_get_reg(idx, OTPCTR_DEBUG0, &val));
        fs_hw_msleep(2);

        if (OTPCTR_RO_GET_DEBUG0_RD_DONE(val) != 1) {
            FS_LOGE("Can't wait OTP read SN done, val=0x%08X", val);
            err = FS_ERR_INTERNAL;
            break;
        }

        BREAK_ON_ERR(err = FL9103_get_reg(idx, OTPCTR_DEBUG2, &trim));
        FS_LOGD("Read trim from OTP: 0x%08X", trim);
    }
    while(false);

    if (err) {
        FS_LOGE("load trim for FL9103[%d] fail, using: 0x%08X", idx - FS_TX_FL9103_1, FL9103_DEFAULT_TRIM_CODE);
        FL9103_set_reg(idx, OTPCTR_CFG0, OTPCTR_RW_CFG0_USE_DOUBLE_BIT_MODE(1) | OTPCTR_RW_CFG0_USE_ECC(1) | OTPCTR_RW_CFG0_NO_TRIM(1) | OTPCTR_RW_CFG0_EFUSE_CS(0));
        FL9103_set_reg(idx, OTPCTR_SHADOW_TRIM, FL9103_DEFAULT_TRIM_CODE); // use default trim
    }

    return err;
}

static int32_t FL9103_check_alive(fs_sensor_tx_rx_e idx)
{
    int32_t err = FS_SUCCESS;
    return err;
}

static int32_t FL9103_get_status(fs_sensor_tx_rx_e idx, void *status)
{
    int32_t err = FS_SUCCESS;
    return err;
}

int32_t FL9103_reset(fs_sensor_tx_rx_e idx, dev_reset_type_t soft_reset)
{
    int32_t err = FS_SUCCESS;
    uint32_t addr;
    fs_sensor_tx_rx_e rst_list[2];
    uint32_t rst_cnt = 0;

    if (idx == FS_TX_FL9103_ALL) {
        for (int32_t i = 0; i < (int32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0])); i++) {
            if (m_dev_id[i] < 0) {
                continue;
            }

            rst_list[rst_cnt] = FS_TX_FL9103_1 + m_dev_id[i];
            rst_cnt++;
        }

        if (rst_cnt == 0) {
            FS_LOGE("No valid FL9103 devices found");
            return FS_ERR_NO_DEVICE;
        }
    }
    else {
        rst_list[0] = idx;
        rst_cnt = 1;
    }

    if (soft_reset == DEV_RESET_TYPE_SW) {
        for (uint32_t i = 0; i < rst_cnt; i++) {
            addr = DRVCTR_FUNC_CONFIG;
            err = FL9103_set_reg(rst_list[i], addr, (  DRVCTR_RW_FUNC_CONFIG_CH_EN                 (0x000000)
                                                     | DRVCTR_RW_FUNC_CONFIG_OVT150_FRAME_SYNC     (0)
                                                     | DRVCTR_RW_FUNC_CONFIG_REG_COMP_CHECK        (0)
                                                     | DRVCTR_RW_FUNC_CONFIG_TDET_FROM_SLOTSEL_0OR1(0)
                                                     | DRVCTR_RW_FUNC_CONFIG_DET_OUT_POLARITY      (0)
                                                     | DRVCTR_RW_FUNC_CONFIG_DET_EN                (0)
                                                     | DRVCTR_RW_FUNC_CONFIG_DRVCTR_EN             (0)));

            if (err) {
                FS_LOGE("set reg(0x%04x) failed(%d), idx=%d.", addr, err, rst_list[i]);
                return err;
            }

            addr = DRVCTR_POWER_CONFIG_SEL;
            err = FL9103_set_reg(rst_list[i], addr, (  DRVCTR_RW_POWER_CONFIG_SEL_CH_POWER_HL_CFG_UNI(0)
                                                     | DRVCTR_RW_POWER_CONFIG_SEL_CH_POWER_SEL       (0x000000)));
            if (err) {
                FS_LOGE("set reg(0x%04x) failed(%d), idx=%d.", addr, err, rst_list[i]);
                return err;
            }
        }
    }
    else {
        err = fs_hw_reset(FS_TX_FL9103_ALL);
    }

    for (uint32_t i = 0; i < rst_cnt; i++) {
        FL9103_load_trim(rst_list[i], NULL, 0);
    }

    return err;
}

/**
 * @brief Set the slot sequence of FL9103.
 * @param idx The index of FL9103.
 * @param len The length of slot sequence.
 * @param sequence The slot sequence. Each element is a 64-bit value(valid bit[47:0])
 * @param lock_slot Whether to lock the slot.
 * @return int32_t The error code.
 */
static int32_t FL9103_set_sequence(fs_sensor_tx_rx_e idx, uint16_t len, const uint64_t sequence[len], bool lock_slot)
{
    int32_t ret = FS_SUCCESS;
    uint8_t  chip_num;
    uint64_t slot_seq[sizeof(m_dev_id) / sizeof(m_dev_id[0])] = { 0, 0, };
    uint32_t channel_en[sizeof(m_dev_id) / sizeof(m_dev_id[0])] = { 0, 0, };
    bool block_mode = false;

    if (sequence == NULL || (!lock_slot && (len != 4)) || (lock_slot && (len != 1)) || idx != FS_TX_FL9103_ALL) {
        FS_LOGE("Invalid parameters: sequence=%p, len=%u, lock_slot=%d, idx=%d", sequence, len, lock_slot, idx);
        return FS_ERR_BAD_PARAMS;
    }

    // 遍历 slot 序列生成 slot_seq
    for (int32_t slot_idx = 0; slot_idx < len; slot_idx++) {
        uint64_t seq = sequence[slot_idx];

        for (int32_t j = 0; j < (int32_t)((sizeof(m_ch_map) / sizeof(m_ch_map[0])) * (sizeof(m_ch_map[0]) / sizeof(m_ch_map[0][0]))); j++) {
            if (((seq >> j) & 0x01) == 0x01) {
                for (int32_t row = 0; row < (int32_t)(sizeof(m_ch_map) / sizeof(m_ch_map[0])); row++) {
                    if (m_dev_id[row] == -1) {
                        continue;
                    }

                    for (int32_t col = 0; col < (int32_t)(sizeof(m_ch_map[0]) / sizeof(m_ch_map[0][0])); col++) {
                        if (m_ch_map[row][col] >= (uint32_t)((sizeof(m_ch_map) / sizeof(m_ch_map[0])) * (sizeof(m_ch_map[0]) / sizeof(m_ch_map[0][0])))) {
                            FS_LOGE("Invalid channel mapping for device[%d]=%d, channel=%d, map=%u", row, m_dev_id[row], col, m_ch_map[row][col]);
                            return FS_ERR_BAD_PARAMS;
                        }

                        if (m_ch_map[row][col] == j) {
                            slot_seq  [row] |= ((uint64_t)(slot_idx & 0x03) << (col * 2));
                            channel_en[row] |=            (1U               <<  col     );
                        }
                    }
                }
            }
        }
    }

    chip_num = 0;
    if (m_dev_id[0] != -1 && m_dev_id[1] != -1) {
        block_mode = true;
        FS_LOGI("Both FL9103 devices are present, using block mode for slot sequence configuration.");
    }

    for (int32_t i = 0; i < (int32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0])); i++) {
        uint32_t val = 0;

        if (m_dev_id[i] == -1) {
            continue;
        }

        if (lock_slot) {
            // 锁定为 slot 0
            val = (  SYSMSC_RW_DBG_TRIG_GEN_POWER_SEL_INT         (0)   // power_sel  internal signal
                   | SYSMSC_RW_DBG_TRIG_GEN_SLOT_SEL_INT          (0)   // slot_sel   internal signal
                   | SYSMSC_RW_DBG_TRIG_GEN_FRAME_SYNC_INT        (0)   // frame_sync internal signal
                   | SYSMSC_RW_DBG_TRIG_GEN_TRIG_BYPASS_POWER_SEL (0)   // bypass power_sel  signal and using internal signal
                   | SYSMSC_RW_DBG_TRIG_GEN_TRIG_BYPASS_SLOT_SEL  (1)   // bypass slot_sel   signal and using internal signal
                   | SYSMSC_RW_DBG_TRIG_GEN_TRIG_BYPASS_FRAME_SYNC(0)); // bypass frame_sync signal and using internal signal
        }
        else {
            // 不锁定 slot
            val = (  SYSMSC_RW_DBG_TRIG_GEN_POWER_SEL_INT         (0)   // power_sel  internal signal
                   | SYSMSC_RW_DBG_TRIG_GEN_SLOT_SEL_INT          (0)   // slot_sel   internal signal
                   | SYSMSC_RW_DBG_TRIG_GEN_FRAME_SYNC_INT        (0)   // frame_sync internal signal
                   | SYSMSC_RW_DBG_TRIG_GEN_TRIG_BYPASS_POWER_SEL (0)   // bypass power_sel  signal and using internal signal
                   | SYSMSC_RW_DBG_TRIG_GEN_TRIG_BYPASS_SLOT_SEL  (0)   // bypass slot_sel   signal and using internal signal
                   | SYSMSC_RW_DBG_TRIG_GEN_TRIG_BYPASS_FRAME_SYNC(0)); // bypass frame_sync signal and using internal signal
        }

        ret = FL9103_write_register(FS_TX_FL9103_1 + m_dev_id[i], SYSMSC_DBG_TRIG_GEN, &val, 1);

        if (ret) {
            FS_LOGE("write reg(0x%04X, 0x%08X) failed(%d).", SYSMSC_DBG_TRIG_GEN, val, ret);
            return ret;
        }

        val = (uint32_t)((slot_seq[i] >> 32) & 0x00FFFF);
        if (block_mode) {
            // block模式下,9103_1 配置成 0xF5F5F5F5, 响应slot 1 & 3,  9103_2 配置成 0x0A0A0A0A, 响应slot 0 & 2
            if(i==0) {
                val = 0x0000F5F5;
            } else {
                val = 0x00000A0A;
            }   
        }
        ret = FL9103_write_register(FS_TX_FL9103_1 + m_dev_id[i], DRVCTR_MODE_CONFIG1, &val, 1);

        if (ret) {
            FS_LOGE("write reg(0x%04X, 0x%08X) failed(%d).", DRVCTR_MODE_CONFIG1, val, ret);
            return ret;
        }

        val = (uint32_t)(slot_seq[i] & 0xFFFFFFFF);
        if (block_mode) {
            // block模式下,9103_1 配置成 0xF5F5F5F5, 响应slot 1 & 3,  9103_2 配置成 0x0A0A0A0A, 响应slot 0 & 2
            if(i==0) {
                val = 0xF5F5F5F5;
            } else {
                val = 0x0A0A0A0A;
            } 
        }
        ret = FL9103_write_register(FS_TX_FL9103_1 + m_dev_id[i], DRVCTR_MODE_CONFIG0, &val, 1);

        if (ret) {
            FS_LOGE("write reg(0x%04X, 0x%08X) failed(%d).", DRVCTR_MODE_CONFIG0, val, ret);
            return ret;
        }

        val = (  DRVCTR_RW_POWER_CONFIG_SEL_CH_POWER_HL_CFG_UNI(15)
               | DRVCTR_RW_POWER_CONFIG_SEL_CH_POWER_SEL       (0x000000));
        ret = FL9103_write_register(FS_TX_FL9103_1 + m_dev_id[i], DRVCTR_POWER_CONFIG_SEL, &val, 1);

        if (ret) {
            FS_LOGE("write reg(0x%04X, 0x%08X) failed(%d).", DRVCTR_POWER_CONFIG_SEL, val, ret);
            return ret;
        }

        val = (  DRVCTR_RW_FUNC_CONFIG_CH_EN                 (channel_en[i] & 0x00FFFFFFu)
               | DRVCTR_RW_FUNC_CONFIG_OVT150_FRAME_SYNC     (0)
               | DRVCTR_RW_FUNC_CONFIG_REG_COMP_CHECK        (1)
               | DRVCTR_RW_FUNC_CONFIG_TDET_FROM_SLOTSEL_0OR1(0)
               | DRVCTR_RW_FUNC_CONFIG_DET_OUT_POLARITY      (0)
               | DRVCTR_RW_FUNC_CONFIG_DET_EN                (0)
               | DRVCTR_RW_FUNC_CONFIG_DRVCTR_EN             (1));
        ret = FL9103_write_register(FS_TX_FL9103_1 + m_dev_id[i], DRVCTR_FUNC_CONFIG, &val, 1);

        if (ret) {
            FS_LOGE("write reg(0x%04X, 0x%08X) failed(%d).", DRVCTR_FUNC_CONFIG, val, ret);
            return ret;
        }

        chip_num++;
    }

    if (chip_num == 0) {
        FS_LOGE("No valid FL9103 devices found.");
        return FS_ERR_NOT_FOUND;
    }

    return ret;
}

static int32_t FL9103_start(fs_sensor_tx_rx_e idx)
{
    int32_t err = FS_SUCCESS;
    return err;
}

static int32_t FL9103_stop(fs_sensor_tx_rx_e idx)
{
    int32_t err = FS_SUCCESS;
    return err;
}

static int32_t FL9103_detect_device(fs_sensor_tx_rx_e idx)
{
    int32_t err = FS_SUCCESS;
    int32_t try_cnt = 3;

    do {
        uint32_t chip_id = 0x00;
        CONTINUE_ON_ERR(err = FL9103_get_reg(idx, SYSMSC_CHIP_ID, &chip_id));

        if (SYSMSC_RO_GET_CHIP_ID_CHIP_ID(chip_id) == SYSMSC_RO_DEF_CHIP_ID_CHIP_ID) {
            err = FS_SUCCESS;
            break;
        }

        err = FS_ERR_NO_DEVICE;
    }
    while (--try_cnt > 0);

    return err;
}

int32_t FL9103_deinit(fs_tx_t *tx)
{
    int32_t err = FS_SUCCESS;
    return err;
}

int32_t FL9103_disable_device(fs_sensor_tx_rx_e idx, bool disable)
{
    if (idx - FS_TX_FL9103_1 >= (int32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0]))) {
        FS_LOGE("Invalid device index: idx=%d", idx);
        return FS_ERR_BAD_PARAMS;
    }
    static int16_t last_fl9103_dev_id[sizeof(m_dev_id) / sizeof(m_dev_id[0])] = {-2, -2};
    if (last_fl9103_dev_id[idx - FS_TX_FL9103_1] == -2) {
        // 初始化时记录第一个被禁用的设备编号，确保同一设备被多次禁用/启用时状态一致
        last_fl9103_dev_id[idx - FS_TX_FL9103_1] = m_dev_id[idx - FS_TX_FL9103_1];
    }
    if (disable) {
        m_dev_id[idx - FS_TX_FL9103_1] = -1;
    } else {
        m_dev_id[idx - FS_TX_FL9103_1] = last_fl9103_dev_id[idx - FS_TX_FL9103_1];
    }
    return FS_SUCCESS;
}
int32_t FL9103_init(fs_tx_t *tx, const int16_t *dev_id, uint8_t dev_cnt, uint8_t ch_cnt, const uint8_t ch_map[dev_cnt][ch_cnt])
{
    int32_t err = FS_SUCCESS;
    CHECK_PTR(tx);
    // FL9103 寄存器的位宽
    tx->reg_width = FS_SENSOR_REG_WIDTH_32;
    // 对外提供的函数接口
    tx->op.check_alive    = FL9103_check_alive;
    tx->op.get_status     = FL9103_get_status;
    tx->op.reset          = FL9103_reset;
    tx->op.set_sequence   = FL9103_set_sequence;
    tx->op.get_sequence   = NULL;
    tx->op.start_sequence = FL9103_start;
    tx->op.stop_sequence  = FL9103_stop;
    tx->op.read_register  = FL9103_read_register;
    tx->op.write_register = FL9103_write_register;
    tx->op.read_buffer    = FL9103_read_buffer;
    tx->op.write_buffer   = FL9103_write_buffer;
    tx->op.disable_device = FL9103_disable_device;

    for (int32_t i = 0; i < (int32_t)(sizeof(tx->dev_id) / sizeof(tx->dev_id[0])); i++) {
        tx->dev_id[i] = -1;
        snprintf(tx->SN[i], sizeof(tx->SN[i]), "%s", "unknown");

        for (int32_t j = 0; j < (int32_t)(sizeof(tx->ch_map[0]) / sizeof(tx->ch_map[0][0])); j++) {
            tx->ch_map[i][j] = 0xFF;
        }
    }

    if ((dev_id != NULL) && (dev_cnt > 0)) {
        int32_t detected = 0;

        if (dev_cnt > sizeof(tx->dev_id) / sizeof(tx->dev_id[0])) {
            FS_LOGE("dev_cnt is too large: dev_cnt(%d) > %d", dev_cnt, (int32_t)(sizeof(tx->dev_id) / sizeof(tx->dev_id[0])));
            return FS_ERR_BAD_PARAMS;
        }
        else if ((ch_map != NULL) && (ch_cnt > 0) && (ch_cnt > sizeof(tx->ch_map[0]) / sizeof(tx->ch_map[0][0]))) {
            FS_LOGE("ch_cnt is too large: ch_cnt(%d) > %d", ch_cnt, (int32_t)(sizeof(tx->ch_map[0]) / sizeof(tx->ch_map[0][0])));
            return FS_ERR_BAD_PARAMS;
        }

        for (int32_t i = 0; i < dev_cnt; i++) {
            int32_t ret;

            if (dev_id[i] < 0) {
                FS_LOGE("invalid dev_id: dev_id[%d] = %d", i, dev_id[i]);
                return FS_ERR_BAD_PARAMS;
            }

            ret = FL9103_detect_device(FS_TX_FL9103_1 + dev_id[i]);

            if (ret != FS_SUCCESS) {
                FS_LOGE("Failed to detect FL9103: dev_id[%d] = %d", i, dev_id[i]);
                continue;
            }

            FS_LOGD("Successfully detected FL9103: dev_id[%d] = %d", i, dev_id[i]);
            FL9103_load_trim(FS_TX_FL9103_1 + dev_id[i], tx->SN[detected], sizeof(tx->SN[0])); // allow to fail
            tx->dev_id[detected++] = dev_id[i];

            if ((ch_map != NULL) && (ch_cnt > 0)) {
                for (int32_t j = 0; j < ch_cnt; j++) {
                    tx->ch_map[detected - 1][j] = ch_map[i][j];
                }
            }
        }

        if (detected == 0) {
            FS_LOGE("No valid FL9103 devices found");
            return FS_ERR_NO_DEVICE;
        }
        // else {
        //     FS_LOGD("Successfully detected FL9103 devices count: %d", detected);
        // }
    }

    for (int32_t i = 0; i < (int32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0])); i++) {
        if (i < (int32_t)(sizeof(tx->dev_id) / sizeof(tx->dev_id[0]))) {
            m_dev_id[i] = tx->dev_id[i];

            for (int32_t j = 0; j < (int32_t)(sizeof(m_ch_map[0]) / sizeof(m_ch_map[0][0])); j++) {
                if (j < (int32_t)(sizeof(tx->ch_map[0]) / sizeof(tx->ch_map[0][0]))) {
                    m_ch_map[i][j] = tx->ch_map[i][j];
                }
                else {
                    m_ch_map[i][j] = 0xFF;
                }
            }
        }
        else {
            m_dev_id[i] = -1;

            for (int32_t j = 0; j < (int32_t)(sizeof(m_ch_map[0]) / sizeof(m_ch_map[0][0])); j++) {
                m_ch_map[i][j] = 0xFF;
            }
        }
    }

    return err;
}
