#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "fs_err.h"
#include "fs_util.h"
#include "drivers/FL6X32.h"
#include "drivers/FL9003.h"
#include "drivers/FL9103.h"

#include "fs_hw.h"
#include "fs_sensor_hub.h"

#undef  LOG_TAG
#define LOG_TAG "sensor_hub"

#define TX_TEST_ENABLE 0

#define CHECK_HUB_INIT()                      \
    do {                                    \
        if (!g_hub || !g_hub->is_initialized) { \
            FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub); \
            return FS_ERR_BAD_PARAMS;       \
        }                                   \
    } while (0)

typedef struct {
    uint32_t addr;
    uint32_t data;
} reg_write_t;

// 传感器集线器实例
static fs_sensor_t *g_hub = NULL;

// 中间层接口函数
int32_t fs_sensor_write_register(fs_sensor_tx_rx_e idx, uint32_t addr,
                                 const uint32_t *data, uint32_t dlen)
{
    int32_t err = FS_SUCCESS;

    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    switch (idx) {
        case FS_RX_FL6X32_1:
        case FS_RX_FL6X32_ALL: {
            // 2. 调用 FL6X32 驱动
            if (!g_hub->rx.op.write_register) {
                FS_LOGE("[HUB] Error: FL6X32 write function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->rx.op.write_register(idx, addr, data, dlen);
            break;
        }

        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL: {
            if (!g_hub->tx[0].op.write_register) {
                FS_LOGE("[HUB] Error: FL9003 write function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[0].op.write_register(idx, addr, data, dlen);
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            if (!g_hub->tx[1].op.write_register) {
                FS_LOGE("[HUB] Error: FL9103 write function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[1].op.write_register(idx, addr, data, dlen);
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

int32_t fs_sensor_read_register(fs_sensor_tx_rx_e idx, uint32_t addr,
                                uint32_t *data, uint32_t *dlen)
{
    int32_t err = FS_SUCCESS;

    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    switch (idx) {
        case FS_RX_FL6X32_1:
        case FS_RX_FL6X32_ALL: {
            if (!g_hub->rx.op.read_register) {
                FS_LOGE("[HUB] Error: FL6X32 read function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->rx.op.read_register(idx, addr, data, *dlen);
            break;
        }

        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL: {
            if (!g_hub->tx[0].op.read_register) {
                FS_LOGE("[HUB] Error: FL9003 read function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[0].op.read_register(idx, addr, data, dlen);
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            if (!g_hub->tx[1].op.read_register) {
                FS_LOGE("[HUB] Error: FL9103 read function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[1].op.read_register(idx, addr, data, dlen);
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

// 检查设备工作状态
int32_t fs_sensor_check_alive(fs_sensor_tx_rx_e idx)
{
    int32_t err = FS_SUCCESS;

    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    switch (idx) {
        case FS_RX_FL6X32_1:
        case FS_RX_FL6X32_ALL: {
            if (!g_hub->rx.op.check_alive) {
                FS_LOGE("[HUB] Error: FL6X32 check_alive function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->rx.op.check_alive(&g_hub->rx);
            break;
        }

        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL: {
            if (!g_hub->tx[0].op.check_alive) {
                FS_LOGE("[HUB] Error: FL9003 check_alive function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[0].op.check_alive(idx);
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            if (!g_hub->tx[1].op.check_alive) {
                FS_LOGE("[HUB] Error: FL9103 check_alive function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[1].op.check_alive(idx);
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

// 获取设备状态
int32_t fs_sensor_get_status(fs_sensor_tx_rx_e idx, void *status)
{
    int32_t err = FS_SUCCESS;

    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    switch (idx) {
        case FS_RX_FL6X32_1:
        case FS_RX_FL6X32_ALL: {
            if (!g_hub->rx.op.get_status) {
                FS_LOGE("[HUB] Error: FL6X32 get_status function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->rx.op.get_status(&g_hub->rx, status);
            break;
        }

        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL: {
            if (!g_hub->tx[0].op.get_status) {
                FS_LOGE("[HUB] Error: FL9003 get_status function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            return g_hub->tx[0].op.get_status(idx, status);
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            if (!g_hub->tx[1].op.get_status) {
                FS_LOGE("[HUB] Error: FL9103 get_status function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            return g_hub->tx[1].op.get_status(idx, status);
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

// 获取设备校准参数
int32_t fs_sensor_get_trim(fs_sensor_tx_rx_e idx, void *trim, uint32_t tlen)
{
    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    // 2. 根据传感器索引获取校准参数
    switch (idx) {
        case FS_RX_FL6X32_1:
        case FS_RX_FL6X32_ALL: {
            if (tlen < sizeof(fs_rx_trim_t)) {
                FS_LOGE("[HUB] Error: Trim buffer size(%u) is too small!", tlen);
                return FS_ERR_BAD_PARAMS;
            }

            memcpy(trim, &g_hub->rx.trim[idx - FS_RX_FL6X32_1], sizeof(fs_rx_trim_t));
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            return FS_ERR_BAD_PARAMS;
        }
    }

    return FS_SUCCESS;
}

int32_t fs_sensor_get_sn(fs_sensor_tx_rx_e idx, char *sn, uint32_t sn_buf_len)
{
    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    // 2. 根据传感器索引获取序列号
    switch (idx) {
        case FS_RX_FL6X32_1:
        case FS_RX_FL6X32_ALL: {
            if (sn_buf_len < sizeof(g_hub->rx.SN[idx - FS_RX_FL6X32_1])) {
                FS_LOGE("[HUB] Error: SN buffer size(%u) is too small!", sn_buf_len);
                return FS_ERR_BAD_PARAMS;
            }

            memcpy(sn, g_hub->rx.SN[idx - FS_RX_FL6X32_1], sizeof(g_hub->rx.SN[idx - FS_RX_FL6X32_1]));
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            if (sn_buf_len < sizeof(g_hub->tx[1].SN[idx - FS_TX_FL9103_1])) {
                FS_LOGE("[HUB] Error: SN buffer size(%u) is too small!", sn_buf_len);
                return FS_ERR_BAD_PARAMS;
            }

            memcpy(sn, g_hub->tx[1].SN[idx - FS_TX_FL9103_1], sizeof(g_hub->tx[1].SN[idx - FS_TX_FL9103_1]));
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            return FS_ERR_BAD_PARAMS;
        }
    }

    return FS_SUCCESS;
}

int32_t fs_sensor_get_trx_version(uint32_t *version)
{
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    if (g_hub->trx_ver == 0) { // 无效版本，尝试重新读取
        // 重新读取 TRX 版本
        int32_t trx_version;
        uint8_t cnt = 1;
        CHECK_ERR(fs_hw_get_adc(ADC_TYPE_VER_RX, ADC_DATA_DEF_PARA, &trx_version, &cnt, NULL));
        g_hub->trx_ver = trx_version;

        if (g_hub->trx_ver == 0) {
            FS_LOGE("[HUB] Error: TRX version not detected!");
            return FS_ERR_BAD_PARAMS;
        }
    }

    *version = g_hub->trx_ver;
    return FS_SUCCESS;
}

// 重置设备
int32_t fs_sensor_reset(fs_sensor_tx_rx_e idx, dev_reset_type_t reset_type)
{
    int32_t err = FS_SUCCESS;

    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    switch (idx) {
        case FS_RX_FL6X32_1:
        case FS_RX_FL6X32_ALL: {
            if (!g_hub->rx.op.reset) {
                FS_LOGE("[HUB] Error: FL6X32 reset function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->rx.op.reset(&g_hub->rx, reset_type);
            break;
        }

        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL: {
            // 2. 调用 FL9003 驱动
            if (!g_hub->tx[0].op.reset) {
                FS_LOGE("[HUB] Error: FL9003 reset function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[0].op.reset(idx, reset_type);
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            // 3. 调用 FL9103 驱动
            if (!g_hub->tx[1].op.reset) {
                FS_LOGE("[HUB] Error: FL9103 reset function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[1].op.reset(idx, reset_type);
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

int32_t fs_sensor_set_sequence(fs_sensor_tx_rx_e idx, uint16_t len, const uint64_t sequence[len], bool is_switch)
{
    int32_t err = FS_SUCCESS;

    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    switch (idx) {
        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL: {
            bool has_trp1 = is_switch;

            // 2. 调用 FL9003 驱动
            if (!g_hub->tx[0].op.set_sequence) {
                FS_LOGE("[HUB] Error: FL9003 set_sequence function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[0].op.set_sequence(idx, len, sequence, has_trp1);
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            bool lock_slot = is_switch;

            // 2. 调用 FL9103 驱动
            if (!g_hub->tx[1].op.set_sequence) {
                FS_LOGE("[HUB] Error: FL9103 set_sequence function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[1].op.set_sequence(idx, len, sequence, lock_slot);
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

// 获取序列
int32_t fs_sensor_get_sequence(fs_sensor_tx_rx_e idx, uint64_t *sequence, uint16_t *len)
{
    int32_t err = FS_SUCCESS;

    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    switch (idx) {
        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL: {
            // 2. 调用 FL9003 驱动
            if (!g_hub->tx[0].op.get_sequence) {
                FS_LOGE("[HUB] Error: FL9003 get_sequence function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[0].op.get_sequence(idx, sequence, len);
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            // 2. 调用 FL9103 驱动
            if (!g_hub->tx[1].op.get_sequence) {
                FS_LOGE("[HUB] Error: FL9103 get_sequence function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[1].op.get_sequence(idx, sequence, len);
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

// 开始序列
int32_t fs_sensor_start_sequence(fs_sensor_tx_rx_e idx)
{
    int32_t err = FS_SUCCESS;

    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    switch (idx) {
        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL: {
            // 2. 调用 FL9003 驱动
            if (!g_hub->tx[0].op.start_sequence) {
                FS_LOGE("[HUB] Error: FL9003 start_sequence function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[0].op.start_sequence(idx);
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            // 2. 调用 FL9103 驱动
            if (!g_hub->tx[1].op.start_sequence) {
                FS_LOGE("[HUB] Error: FL9103 start_sequence function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[1].op.start_sequence(idx);
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

int32_t fs_sensor_disable_device(fs_sensor_tx_rx_e idx, bool disable)
{
    int32_t err = FS_SUCCESS;

    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    switch (idx) {
        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL: {
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            // 2. 调用 FL9103 驱动
            if (!g_hub->tx[1].op.disable_device) {
                FS_LOGE("[HUB] Error: FL9103 disable_device function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[1].op.disable_device(idx, disable);
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

// 停止序列
int32_t fs_sensor_stop_sequence(fs_sensor_tx_rx_e idx)
{
    int32_t err = FS_SUCCESS;

    // 1. 检查集线器初始化状态
    if (!g_hub || !g_hub->is_initialized) {
        FS_LOGE("[HUB] Error: Sensor hub not initialized! g_hub=%p", g_hub);
        return FS_ERR_BAD_PARAMS;
    }

    switch (idx) {
        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL: {
            // 2. 调用 FL9003 驱动
            if (!g_hub->tx[0].op.stop_sequence) {
                FS_LOGE("[HUB] Error: FL9003 stop_sequence function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[0].op.stop_sequence(idx);
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            // 2. 调用 FL9103 驱动
            if (!g_hub->tx[1].op.stop_sequence) {
                FS_LOGE("[HUB] Error: FL9103 stop_sequence function not registered!");
                return FS_ERR_BAD_PARAMS;
            }

            err = g_hub->tx[1].op.stop_sequence(idx);
            break;
        }

        default: {
            FS_LOGE("[HUB] Error: Invalid sensor index %d!", idx);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

int32_t fs_sensor_cfg_tx(const trx_hw_cfg_t *hw_cfg, tx_sequence_t *tx_seq, bool is_set_vpp, bool disable_9103_1)
{
    int32_t err = FS_SUCCESS;
    FL9003_deinit(&g_hub->tx[0]);
    FL9103_deinit(&g_hub->tx[1]);
    CHECK_ERR(FL9003_init(&g_hub->tx[0], hw_cfg->tx_hs_devid, hw_cfg->tx_hs_ic_cnt, sizeof(tx_seq->hs_ch_id[0]) / sizeof(tx_seq->hs_ch_id[0][0]), tx_seq->hs_ch_id));
    CHECK_ERR(FL9103_init(&g_hub->tx[1], hw_cfg->tx_ls_devid, hw_cfg->tx_ls_ic_cnt, sizeof(tx_seq->ls_ch_id[0]) / sizeof(tx_seq->ls_ch_id[0][0]), tx_seq->ls_ch_id));
    CHECK_ERR(fs_sensor_disable_device(FS_TX_FL9103_1, disable_9103_1));
    CHECK_ERR(fs_sensor_reset(FS_TX_FL9003_ALL, DEV_RESET_TYPE_HW));
    CHECK_ERR(fs_sensor_reset(FS_TX_FL9103_ALL, DEV_RESET_TYPE_HW));
    CHECK_ERR(fs_sensor_reset(FS_TX_FL9003_ALL, DEV_RESET_TYPE_SW));
    CHECK_ERR(fs_sensor_set_sequence(FS_TX_FL9003_ALL, tx_seq->hs_seq.seq_cnt, tx_seq->hs_seq.sequence, tx_seq->has_trp1));
    CHECK_ERR(fs_sensor_reset(FS_TX_FL9103_ALL, DEV_RESET_TYPE_SW));
    CHECK_ERR(fs_sensor_set_sequence(FS_TX_FL9103_ALL, tx_seq->ls_seq.seq_cnt, tx_seq->ls_seq.sequence, tx_seq->lock_slot));

    if (is_set_vpp) {
        CHECK_ERR(fs_hw_set_vpp_volt(30.0f, NULL));
    }

    return err;
}

int32_t fs_sensor_cfg_rx(const rx_staged_reg_list_t *reg_list, bool is_set_vbd)
{
    int32_t err = FS_SUCCESS;
    CHECK_PTR(reg_list);
    CHECK_HUB_INIT();
    g_hub->rx.cfg = reg_list;

    do {
        int16_t rx_dev_id = 0;
        err = g_hub->rx.op.reset(&g_hub->rx, false);

        if (err) {
            FS_LOGE("Failed to get reset cfg: %d", err);
            break;
        }

        FL6X32_deinit(&g_hub->rx);
        err = FL6X32_init(&g_hub->rx, &rx_dev_id, 1);

        if (err) {
            FS_LOGE("Failed to re-init FL6X32: %d", err);
            break;
        }

        err = g_hub->rx.op.wakeup(&g_hub->rx);

        if (err) {
            FS_LOGE("Failed to get wakeup cfg: %d", err);
            break;
        }

        err = g_hub->rx.op.setup_cfg(&g_hub->rx);

        if (err) {
            FS_LOGE("Failed to get config cfg: %d", err);
            break;
        }
    }
    while (false);

    if (err) {
        return err;
    }

    if (is_set_vbd) {
        CHECK_ERR(fs_hw_set_vbd_volt(g_hub->rx.trim[0].cali_vbd_25C, NULL));
    }

    return err;
}

int32_t fs_sensor_start(void)
{
    int32_t err = FS_SUCCESS;
    CHECK_HUB_INIT();
    CHECK_ERR(fs_sensor_start_sequence(FS_TX_FL9103_ALL));
    CHECK_ERR(fs_sensor_start_sequence(FS_TX_FL9003_ALL));
    CHECK_ERR(g_hub->rx.op.start_scan(&g_hub->rx));
    return err;
}

int32_t fs_sensor_stop(void)
{
    int32_t err = FS_SUCCESS;
    CHECK_HUB_INIT();
    CHECK_ERR(fs_sensor_stop_sequence(FS_TX_FL9003_ALL));
    CHECK_ERR(fs_sensor_stop_sequence(FS_TX_FL9103_ALL));
    CHECK_ERR(g_hub->rx.op.stop_scan(&g_hub->rx));
    return err;
}

void fs_sensor_deinit(void)
{
    if (g_hub) {
        memset(g_hub, 0x00, sizeof(*g_hub));
        g_hub = NULL;
    }

    return;
}

int32_t fs_sensor_init(void)
{
    int32_t err = FS_SUCCESS;
    static fs_sensor_t hub;
    int32_t trx_version;
    uint8_t cnt;

    hub.is_initialized = false;
    memset(&hub, 0, sizeof(fs_sensor_t));
    hub.trx_ver = 0;
    trx_version = 0;
    cnt         = 1;
    CHECK_ERR(fs_hw_get_adc(ADC_TYPE_VER_RX, ADC_DATA_DEF_PARA, &trx_version, &cnt, NULL));
    hub.trx_ver = trx_version;
    FS_LOGD("TRX version: V%02u", hub.trx_ver);

    // 初始化 FL6X32 传感器
    err = FL6X32_init(&hub.rx, NULL, 0);
    if (err != FS_SUCCESS) {
        return err;
    }

    // 初始化 FL9003 传感器
    err = FL9003_init(&hub.tx[0], NULL, 0, 0, NULL);
    if (err != FS_SUCCESS) {
        return err;
    }

    // 初始化 FL9103 传感器
    err = FL9103_init(&hub.tx[1], NULL, 0, 0, NULL);
    if (err != FS_SUCCESS) {
        return err;
    }

    hub.is_initialized = true;
    g_hub = &hub;

    FS_LOGI("All sensors initialized successfully");
    return err;
}
