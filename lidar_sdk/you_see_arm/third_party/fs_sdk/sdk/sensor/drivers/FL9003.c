#include <stdio.h>
#include <string.h>

#include "fs_err.h"
#include "fs_hw.h"
#include "mcu_protocol.h"
#include "FL9003.h"

#undef  LOG_TAG
#define LOG_TAG "soc.FL9003"

extern fs_sensor_t *g_hub;  // 声明外部全局变量

#define FL9003_MAX_PKG_ITEMS            8       // 每包最多写寄存器数量

static int16_t m_dev_id[FS_SENSOR_TX_HS_MAX_DEV_CNT];
static uint8_t m_ch_map[FS_SENSOR_TX_HS_MAX_DEV_CNT][FS_SENSOR_TX_HS_MAX_CH_CNT];

static int32_t FL9003_stop_sequence(fs_sensor_tx_rx_e idx);

/**
 * @param idx   设备索引 (FS_TX_FL9003_ALL 或其他设备ID)
 * @param addr  起始寄存器地址 (8位)
 * @param data  32位数据数组指针
 * @param dlen  数据项数量 (32位数据个数)
 * @return int32_t 执行结果
 * @note 固定单个寄存器的数据长度为32位。限制一次最多写8个寄存器
 */
static int32_t FL9003_write_register(fs_sensor_tx_rx_e idx, uint32_t addr, const uint32_t *data, uint32_t dlen)
{
    int32_t err = FS_SUCCESS;
    int32_t i;

    for (i = 0; i < (int32_t)dlen; i++) {
        err = mcu_prot_set_fl9003_reg(idx - FS_TX_FL9003_1, addr + i, data[i]);

        if (err != FS_SUCCESS) {
            break;
        }
    }

    return FS_SUCCESS;
}

static int32_t FL9003_read_register(fs_sensor_tx_rx_e idx, uint32_t addr, uint32_t *data, uint32_t *dlen)
{
    int32_t err = FS_SUCCESS;
    int32_t i;

    for (i = 0; i < (int32_t)(*dlen); i++) {
        err = mcu_prot_get_fl9003_reg(idx - FS_TX_FL9003_1, addr + i, &data[i]);

        if (err != FS_SUCCESS) {
            break;
        }
    }

    return FS_SUCCESS;
}

/**
 * @param idx   设备索引
 * @param addr  起始寄存器地址 (8位)
 * @param data  32位数据数组指针
 * @param dlen  数据项数量(BYTE)
 * @return int32_t 执行结果
 * @note 固定单个寄存器的数据长度不固定，通过初始化定义
 */
static int32_t FL9003_write_buffer(fs_sensor_tx_rx_e idx, uint32_t addr, const void *data, uint32_t dlen)
{
    int32_t err = FS_SUCCESS;
    return FS_SUCCESS;
}

static int32_t FL9003_read_buffer(fs_sensor_tx_rx_e idx, uint32_t addr, void *data, uint32_t *dlen)
{
    int32_t err = FS_SUCCESS;
    return FS_SUCCESS;
}

static int32_t FL9003_check_alive(fs_sensor_tx_rx_e idx)
{
    int32_t err = FS_SUCCESS;
    uint32_t read_val = 0;
    uint32_t dlen = 1;

    // 1. 读取原始OPSIZE值
    err = FL9003_read_register(idx, FL9003_OPSIZE, &read_val, &dlen);
    if (err != FS_SUCCESS) return FS_ERR_SPI_READ;
    uint32_t original_val = read_val & 0xFF;

    // 2. 写入测试值0x16
    uint32_t  test_val = 0x16;
    err = FL9003_write_register(idx, FL9003_OPSIZE, &test_val, 1);
    if (err != FS_SUCCESS) return FS_ERR_SPI_WRITE;

    // 3. 验证测试值
    err = FL9003_read_register(idx, FL9003_OPSIZE, &read_val, &dlen);
    if (err != FS_SUCCESS || (read_val & 0xFF) != test_val) {
        FL9003_write_register(idx, FL9003_OPSIZE, &original_val, 1);  // 尝试恢复
        return FS_ERR_SPI_WRITE;
    }

    // 4. 恢复原始终值
    err = FL9003_write_register(idx, FL9003_OPSIZE,&original_val, 1);
    if (err != FS_SUCCESS) return FS_ERR_SPI_WRITE;

    return err;
}

static int32_t FL9003_get_status(fs_sensor_tx_rx_e idx, void *status)
{
    if (status == NULL) {
        FS_LOGE("FL9003_get_status: status is NULL");
        return FS_ERR_BAD_PARAMS;
    }

    uint32_t reg_val = 0;
    uint32_t dlen = 1;  // 读取1字节
    int32_t err = FL9003_read_register(idx, FL9003_STATUS, &reg_val, &dlen);
    if (err != FS_SUCCESS) {
        FS_LOGE("Get status failed (idx=%d, err=%d)", idx, err);
        return err;
    }

    return FS_SUCCESS;
}

static int32_t FL9003_reset(fs_sensor_tx_rx_e idx, dev_reset_type_t reset_type)
{
    int32_t err = FS_SUCCESS;

    if (reset_type == DEV_RESET_TYPE_SW) {        // 软复位，复位寄存器置一
        uint32_t val = 0xAA;
        err = FL9003_write_register(idx, FL9003_CTRL, &val, 1);   // 置复位信号
        if (err != FS_SUCCESS) return err;
        fs_hw_msleep(5);

        val = 0xA8;
        err =FL9003_write_register(idx, FL9003_CTRL, &val, 1);  // 清除复位信
        if (err != FS_SUCCESS) return err;

        err = FL9003_stop_sequence(idx);  // 停止当前序列
        if (err != FS_SUCCESS) return err;

        return err;
    }
    else if (reset_type == DEV_RESET_TYPE_HW) {    // 硬复位：GPIO控制
        return fs_hw_reset(FS_TX_FL9003_ALL);
    }
    else {
        FS_LOGE("FL9003_reset: invalid reset_type %u", reset_type);
        return FS_ERR_BAD_PARAMS;
    }

    return err;
}

/**
 * @brief 获取指定芯片的掩码
 * @param chip_idx 芯片索引 (0, 1, 2)
 * @param mask 芯片通道的掩码指针
 * @return 成功返回 0, 失败返回非 0
 */
static int32_t FL9003_get_mask(uint8_t chip_idx, uint64_t *mask)
{
    int32_t err = FS_SUCCESS;
    CHECK_PTR(mask);

    if ((chip_idx >= (uint32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0]))) || (m_dev_id[chip_idx] == -1)) {
        FS_LOGE("invalid chip_idx %u", chip_idx);
        return FS_ERR_BAD_PARAMS;
    }

    *mask = 0;

    for (int32_t i = 0; i < (int32_t)(sizeof(m_ch_map[chip_idx]) / sizeof(m_ch_map[chip_idx][0])); i++) {
        if ((m_ch_map[chip_idx][i] != 0xFF) && (m_ch_map[chip_idx][i] < 64)) {
            *mask |= (1ULL << m_ch_map[chip_idx][i]);
        }
    }

    return err;
}

/**
 * @brief 获取指定序列中的通道数量
 * @param sequence 序列
 * @param ch_cnt 通道数量指针
 * @return 成功返回 0, 失败返回非 0
 */
static int32_t FL9003_get_seq_channel_cnt(uint64_t sequence, uint8_t *ch_cnt)
{
    int32_t err = FS_SUCCESS;
    CHECK_PTR(ch_cnt);
    *ch_cnt = 0;

    for (int32_t i = 0; i < 64; i++) {
        if (sequence & (1ULL << i)) {
            bool matched = false;

            for (int32_t j = 0; j < (int32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0])); j++) {
                if (m_dev_id[j] != -1) {
                    for (int32_t k = 0; k < (int32_t)(sizeof(m_ch_map[j]) / sizeof(m_ch_map[j][0])); k++) {
                        if ((m_ch_map[j][k] != 0xFF) && (m_ch_map[j][k] == i)) {
                            matched = true;
                            break;
                        }
                    }
                }
            }

            if (matched) {
                (*ch_cnt)++;
            }
            else {
                FS_LOGE("Channel %d is not mapped to any device", i);
                return FS_ERR_BAD_PARAMS;
            }
        }
    }

    return err;
}

/**
 * @brief 获取指定芯片的序列
 * @param chip_idx 芯片索引 (0, 1, 2)
 * @param sequence 完整序列
 * @param chip_seq 指定芯片序列指针
 * @return 成功返回 0, 失败返回非 0
 */
static int32_t FL9003_get_chip_seq(uint8_t chip_idx, uint64_t sequence, uint64_t *chip_seq)
{
    int32_t err = FS_SUCCESS;
    CHECK_PTR(chip_seq);
    *chip_seq = 0;

    if ((chip_idx >= (uint32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0]))) || (m_dev_id[chip_idx] == -1)) {
        FS_LOGE("invalid chip_idx %u", chip_idx);
        return FS_ERR_BAD_PARAMS;
    }

    for (int32_t i = 0; i < 64; i++) {
        if (sequence & (1ULL << i)) {
            for (int32_t k = 0; k < (int32_t)(sizeof(m_ch_map[chip_idx]) / sizeof(m_ch_map[chip_idx][0])); k++) {
                if ((m_ch_map[chip_idx][k] != 0xFF) && (m_ch_map[chip_idx][k] == i)) {
                    *chip_seq |= (1ULL << k);
                    break;
                }
            }
        }
    }

    return err;
}

/**
 * @brief 查找指定芯片的操作序列起始位置
 * @param chip_idx 芯片索引 (0, 1, 2)
 * @param sequence 序列数组指针
 * @param len 序列长度
 * @param has_trp1 是否包含 TRP1 信号
 * @param mask 芯片通道的掩码
 * @param opstart 操作起始位置指针
 * @param opsize 操作大小指针
 * @return 成功返回 0, 失败返回非 0
 */
static int32_t FL9003_find_op_cfg(uint8_t chip_idx, const uint64_t *sequence, uint16_t len, bool has_trp1, uint64_t mask,
                                  uint16_t *opstart, uint16_t *opsize)
{
    int32_t err = FS_SUCCESS;
    uint16_t start = 0xFFFF;
    uint16_t size = 0;
    uint16_t seq_idx;

    if ((chip_idx >= (uint32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0]))) || (m_dev_id[chip_idx] == -1)
        || sequence == NULL || len == 0
        || opstart == NULL || opsize == NULL) {
        FS_LOGE("invalid params: (chip_idx=%u, sequence=%p, len=%u, opstart=%p, opsize=%p)",
                chip_idx, sequence, len, opstart, opsize);
        return FS_ERR_BAD_PARAMS;
    }

    *opstart = 0xFFFF;
    *opsize  = 0;

    for (seq_idx = 0; seq_idx < len; seq_idx++) {
        uint64_t seq_data;
        seq_data = sequence[seq_idx] & mask;

        // 第一次找到非零序列, 记录起始位置
        if (seq_data && (start == 0xFFFF)) {
            start = seq_idx;
        }

        // 从起始位置开始，统计连续序列的长度
        if (start != 0xFFFF) {
            bool check = true;

            if (has_trp1) {
                // 2 颗 FL9003 交替工作模式下, 要求起始位置和序列索引的奇偶性一致时才能进行统计, 不一致时无须统计, 以节省 tx 时序资源
                if ((start % 2) != (seq_idx % 2)) {
                    check = false;

                    if (seq_data) {
                        // 如果在交替工作模式下，起始位置和序列索引的奇偶性不一致，且序列数据非零，则报错, 交替工作模式下不支持连续 2 个序列都让同 1 颗 9003 工作
                        FS_LOGE("Invalid sequence data at index %u for chip %u", seq_idx, m_dev_id[chip_idx]);
                        err = FS_ERR_BAD_PROTOCOL;
                        break;
                    }
                }
                else if ((sequence[seq_idx] & (~mask)) != 0x00) {
                    // 如果在交替工作模式下，同时要求点亮 trp0 和 trp1 的 9003 芯片, 则报错, 不支持这种模式
                    FS_LOGE("Invalid sequence data at index %u for chip %u", seq_idx, m_dev_id[chip_idx]);
                    err = FS_ERR_BAD_PROTOCOL;
                    break;
                }
            }

            if (check) {
                size++;

                // 如果序列的长度超过最大限制后还存在非零序列，则停止统计并报错
                if (seq_data && (size > FL9003_MAX_SEQUENCE_PER_CHIP)) {
                    FS_LOGE("sequence length exceeds max limit (%d) for chip %u, start=%u, size=%u", FL9003_MAX_SEQUENCE_PER_CHIP, m_dev_id[chip_idx], start, size);
                    err = FS_ERR_BAD_PROTOCOL;
                    break;
                }
            }
        }
    }

    if (err != FS_SUCCESS) {
        return err;
    }

    *opstart = start;
    *opsize  = size;
    return err;
}

static int32_t FL9003_set_sequence(fs_sensor_tx_rx_e idx, uint16_t len, const uint64_t sequence[len], bool has_trp1)
{
    int32_t err = FS_SUCCESS;
    int32_t i;
    uint8_t  chip_num;
    uint16_t op_start[sizeof(m_dev_id) / sizeof(m_dev_id[0])] = { 0xFFFF, 0xFFFF, 0xFFFF, };
    uint16_t op_size [sizeof(m_dev_id) / sizeof(m_dev_id[0])] = { 0, 0, 0, };
    uint64_t mask[sizeof(m_dev_id) / sizeof(m_dev_id[0])];

    // 1. 参数校验
    if (sequence == NULL || len == 0 || idx != FS_TX_FL9003_ALL) {
        FS_LOGE("Invalid params (idx=%d, seq=%p, len=%u)", idx, sequence, len);
        return FS_ERR_BAD_PARAMS;
    }

    chip_num = 0;

    for (i = 0; i < (int32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0])); i++) {
        if (m_dev_id[i] != -1) {
            chip_num++;
        }
    }

    if (chip_num == 0) {
        FS_LOGE("No valid chips found");
        return FS_ERR_BAD_PARAMS;
    }

    // 查找每个芯片的 opstart
    for (i = 0; i < (int32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0])); i++) {
        if (m_dev_id[i] == -1) {
            continue;
        }

        CHECK_ERR(FL9003_get_mask(i, &mask[i]));
        err = FL9003_find_op_cfg(i, sequence, len, has_trp1, mask[i], &op_start[i], &op_size[i]);

        if (err == FS_ERR_BAD_PROTOCOL) {
            return FS_ERR_BAD_PARAMS;
        }
        else if (err != FS_SUCCESS) {
            FS_LOGE("Failed to find op_cfg for chip %u, err=%d", i, err);
            return err;
        }

        // if (op_start[i] == 0xFFFF) {
        //     FS_LOGE("Failed to find opstart for chip %u", i);
        //     return FS_ERR_BAD_PARAMS;
        // }
    }

    // 检查每个序列中的通道数是否超过最大限制(防止同时点亮的通道数太多)
    for (i = 0; i < len; i++) {
        uint8_t chn_cnt = 0;
        CHECK_ERR(FL9003_get_seq_channel_cnt(sequence[i], &chn_cnt));

        if (chn_cnt >= FL9003_MAX_CHANNELS_IN_SEQ) {
            FS_LOGE("Too many channels set in sequence[%u]", i);
            return FS_ERR_BAD_PARAMS;
        }
    }

    // 写入每个芯片的 OPSTART 和 OPSIZE 寄存器
    for (i = 0; i < (int32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0])); i++) {
        uint32_t reg_val;

        if (m_dev_id[i] == -1) {
            continue;
        }

        reg_val = has_trp1 ? (op_start[i] / 2) : op_start[i];
        err = FL9003_write_register(FS_TX_FL9003_1 + m_dev_id[i], FL9003_OPSTART, &reg_val, 1);

        if (err != FS_SUCCESS) {
            FS_LOGE("Failed to set OPSTART for chip %u, err=%d", i, err);
            return err;
        }

        reg_val = op_size[i] - 1; // OPSIZE

        if (reg_val == 0 || reg_val >= FL9003_MAX_SEQUENCE_PER_CHIP) {
            reg_val = 1;
        }

        err = FL9003_write_register(FS_TX_FL9003_1 + m_dev_id[i], FL9003_OPSIZE, &reg_val, 1);

        if (err != FS_SUCCESS) {
            FS_LOGE("Failed to set OPSIZE for chip %u, err=%d", i, err);
            return err;
        }
    }

    for (int32_t j = 0; j < len; j++) {
        uint64_t seq_data = sequence[j];

        // 分别写入每个芯片的序列数据
        for (i = 0; i < (int32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0])); i++) {
            uint64_t chip_seq;
            uint16_t seq_id;

            if (m_dev_id[i] == -1) {
                continue;
            }

            CHECK_ERR(FL9003_get_chip_seq(i, seq_data, &chip_seq));

            if ((chip_seq == 0) && (j < op_start[i])) {
                continue; // 跳过第 1 个工作序列前的空序列
            }

            seq_id = j - op_start[i];

            if (has_trp1) {
                if (seq_id % 2 != 0) {
                    continue;
                }

                seq_id = seq_id / 2;
            }

            uint32_t reg_addr = FL9003_S00A + 3 * seq_id; // 每个序列占3字节寄存器空间
            uint32_t tx_buf[3];
            tx_buf[0] = (chip_seq >> 16) & 0x03;  // 高 2 位
            tx_buf[1] = (chip_seq >>  8) & 0xFF;  // 中 8 位
            tx_buf[2] =  chip_seq        & 0xFF;  // 低 8 位
            err = FL9003_write_register(FS_TX_FL9003_1 + m_dev_id[i], reg_addr, tx_buf, sizeof(tx_buf) / sizeof(tx_buf[0]));

            if (err != FS_SUCCESS) {
                FS_LOGE("failed to write sequence for chip %u at index %u, err=%d", m_dev_id[i], j, err);
                return err;
            }
        }
    }

    return FS_SUCCESS;
}

static int32_t FL9003_get_sequence(fs_sensor_tx_rx_e idx, uint64_t *sequence, uint16_t *len)
{
    int32_t err = FS_SUCCESS;
    int retry = 3;

    // 1. 参数校验
    if (sequence == NULL || len == NULL || *len == 0) {
        FS_LOGE("FL9003_get_sequence: Invalid params (seq=%p, len=%p, *len=%u)", sequence, len, len ? *len : 0);
        return FS_ERR_BAD_PARAMS;
    }

    uint16_t max_seq = *len;    // 最大可读取的序列数
    uint16_t actual_read = 0;   // 实际读取的序列数

    do {
        actual_read = 0;
        err = FS_SUCCESS;

        // 2. 根据idx确定要读取的芯片
        uint8_t target_chips[3] = {0};
        uint8_t chip_count = 0;

        if (idx == FS_TX_FL9003_ALL) {
            target_chips[0] = 0; target_chips[1] = 1; target_chips[2] = 2;
            chip_count = 3;
        } else if (idx >= FS_TX_FL9003_1 && idx <= FS_TX_FL9003_3) {
            target_chips[0] = idx - FS_TX_FL9003_1;
            chip_count = 1;
        } else {
            FS_LOGE("FL9003_get_sequence: Invalid device index %d", idx);
            return FS_ERR_BAD_PARAMS;
        }

        // 3. 读取序列数据
        for (uint16_t seq_idx = 0; seq_idx < max_seq; seq_idx++) {
            uint64_t full_seq = 0;

            //读取每个目标芯片的序列数据
            for (uint8_t chip_idx = 0; chip_idx < chip_count; chip_idx++) {
                uint8_t chip_id = target_chips[chip_idx];

                // 索引芯片寄存器地址
                uint32_t reg_addr = FL9003_S00A + 3 * seq_idx;
                uint32_t rx_buf[3];
                uint32_t dlen = sizeof(rx_buf) / sizeof(rx_buf[0]);

                err = FL9003_read_register(chip_id, reg_addr, rx_buf, &dlen);
                if (err != FS_SUCCESS) {
                    FS_LOGE("FL9003_get_sequence: Read seq reg failed for chip %d, seq %d: err=%d", chip_id, seq_idx, err);
                    memset(rx_buf, 0, sizeof(rx_buf));
                }

                // 3.3 组合16位数据
                uint16_t chip_data = ((uint16_t)(rx_buf[0] & 0x00FF) << 8) | ((uint16_t)(rx_buf[1] & 0x00FF) << 0);
                // 注意：rx_buf[2] 是多余的（原始16位数据只用到高2字节）

                // 根据芯片ID放置数据
                if (idx == FS_TX_FL9003_ALL) {
                    if (chip_id == 0) {
                        full_seq |= chip_data;
                    } else if (chip_id == 1) {
                        full_seq |= (uint64_t)chip_data << 16;
                    } else if (chip_id == 2) {
                        full_seq |= (uint64_t)chip_data << 32;
                    }
                } else {
                    full_seq = (uint64_t)chip_data << (chip_id * 16);
                }
            }

            sequence[actual_read++] = full_seq;
        }

        if (err == FS_SUCCESS) break;
    } while (--retry > 0);

    *len = actual_read;

    if (err != FS_SUCCESS) {
        FS_LOGE("FL9003_get_sequence failed after retries, last error: %d", err);
        return FS_ERR_SPI_READ;
    }

    return FS_SUCCESS;
}

static int32_t FL9003_start_sequence(fs_sensor_tx_rx_e idx)
{
    uint32_t ctrl_val = 0xA9;  // 启动序列控制字
    return FL9003_write_register(idx, FL9003_CTRL, &ctrl_val, 1);
}

static int32_t FL9003_stop_sequence(fs_sensor_tx_rx_e idx)
{
    uint32_t val = 0xA8;  // 停止序列控制字
    int32_t err = FS_SUCCESS;

    err = FL9003_write_register(idx, FL9003_CTRL, &val, 1);
    if (err != FS_SUCCESS) {
        FS_LOGE("FL9003_stop_sequence: Failed to write register, err=%d", err);
        return err;
    }

    val = 0xFF;
    err = FL9003_write_register(idx, FL9003_INTMASK, &val, 1);  // 使能所有中断
    if (err != FS_SUCCESS) return err;

    val = 0xFF;
    err = FL9003_write_register(idx, FL9003_INTFLAG, &val, 1);  // 清除中断标志
    if (err != FS_SUCCESS) return err;

    for (int i = 0; i < FL9003_MAX_SEQUENCE_PER_CHIP; i++) {
        val = 0x00;
        err = FL9003_write_register(idx, FL9003_S00A + i, &val, 1);  // 清除序列寄存器
        if (err != FS_SUCCESS) return err;
    }

    // 最后需要打开序列开启位，使所有的序列清零
    val = 0xA9;
    err = FL9003_write_register(idx, FL9003_CTRL, &val, 1);
    if (err != FS_SUCCESS) {
        FS_LOGE("FL9003_stop_sequence: Failed to write register, err=%d", err);
        return err;
    }

    return FS_SUCCESS;
}

static int32_t FL9003_detect_device(fs_sensor_tx_rx_e idx)
{
    int32_t err = FS_SUCCESS;
    int32_t try_cnt = 3;

    do {
        uint32_t set_val, get_val, bak_val, reg_len;
        bak_val = 0;
        reg_len = 1;
        CONTINUE_ON_ERR(err = FL9003_read_register (idx, FL9003_OPSTART, &bak_val, &reg_len));
        set_val = 21;
        CONTINUE_ON_ERR(err = FL9003_write_register(idx, FL9003_OPSTART, &set_val, 1));
        get_val = 0;
        reg_len = 1;
        CONTINUE_ON_ERR(err = FL9003_read_register (idx, FL9003_OPSTART, &get_val, &reg_len));

        // test
        // if (idx == FS_TX_FL9003_2) {
        //     get_val = set_val;
        // }

        if (get_val == set_val) {
            err = FS_SUCCESS;
            break;
        }

        err = FS_ERR_NO_DEVICE;
    }
    while (--try_cnt > 0);

    return err;
}

int32_t FL9003_deinit(fs_tx_t *tx)
{
    int32_t err = FS_SUCCESS;
    return err;
}

int32_t FL9003_init(fs_tx_t *tx, const int16_t *dev_id, uint8_t dev_cnt, uint8_t ch_cnt, const uint8_t ch_map[dev_cnt][ch_cnt])
{
    int32_t err = FS_SUCCESS;

    CHECK_PTR(tx);
    // FL9003 寄存器的位宽
    tx->reg_width = FS_SENSOR_REG_WIDTH_08;

    // 对外提供的函数接口
    tx->op.check_alive    = FL9003_check_alive;
    tx->op.get_status     = FL9003_get_status;
    tx->op.reset          = FL9003_reset;
    tx->op.set_sequence   = FL9003_set_sequence;
    tx->op.get_sequence   = FL9003_get_sequence;
    tx->op.start_sequence = FL9003_start_sequence;
    tx->op.stop_sequence  = FL9003_stop_sequence;
    tx->op.read_register  = FL9003_read_register;  //用于PC工具直接透传
    tx->op.write_register = FL9003_write_register;
    tx->op.read_buffer    = FL9003_read_buffer;
    tx->op.write_buffer   = FL9003_write_buffer;
    tx->op.disable_device = NULL;  // 暂不支持单独禁用某个设备

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

            ret = FL9003_detect_device(FS_TX_FL9003_1 + dev_id[i]);

            if (ret != FS_SUCCESS) {
                FS_LOGE("Failed to detect FL9003: dev_id[%d] = %d", i, dev_id[i]);
                continue;
            }

            FS_LOGD("Successfully detected FL9003: dev_id[%d] = %d", i, dev_id[i]);
            tx->dev_id[detected++] = dev_id[i];

            if ((ch_map != NULL) && (ch_cnt > 0)) {
                for (int32_t j = 0; j < ch_cnt; j++) {
                    tx->ch_map[detected - 1][j] = ch_map[i][j];
                }
            }
        }

        if (detected == 0) {
            FS_LOGE("No valid FL9003 devices found");
            return FS_ERR_NO_DEVICE;
        }
        // else {
        //     FS_LOGD("Successfully detected FL9003 devices count: %d", detected);
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
