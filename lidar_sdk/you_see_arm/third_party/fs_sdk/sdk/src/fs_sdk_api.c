#include <stdio.h>
#include <string.h>

#include "fs_sdk_api.h"
#include "fs_util.h"
#include "utils/fs_err.h"
#include "utils/fs_log.h"
#include "utils/fs_version.h"
#include "pinctrl.h"
#include "i2c_master.h"
#include "serdes.h"
#include "mcu_protocol.h"
#include "mcu_upgrade.h"
#include "imu_data_service.h"
#include "soc_time_sync_module.h"
#include "soc_system_manager.h"
#include "soc_sdk_internal.h"
#include "mcu_log_reader.h"
#include "mcu_log_route.h"
#include "fs_sensor_hub.h"
#include "algo.h"
#include "rxdata_service.h"
#include "fs_client_data_process.h"

#undef  LOG_TAG
#define LOG_TAG "sdk_api"

static int fs_sdk_mcu_version_is_placeholder(const char *ver)
{
    return (ver == NULL) ||
           (ver[0] == '\0') ||
           (strcmp(ver, "uart-passive-listen") == 0) ||
           (strcmp(ver, "unknown") == 0);
}

static void fs_sdk_print_mcu_firmware_version(void)
{
    char version_data[FS_VERSION_MAX_LEN * 2] = {0};
    int32_t err;
    const char *mcu_ver = g_version.app_mcu;

    if (fs_sdk_mcu_version_is_placeholder(mcu_ver)) {
        err = mcu_prot_get_mcu_version((uint8_t *)version_data, sizeof(version_data));
        if (err == FS_SUCCESS && version_data[0] != '\0') {
            snprintf(g_version.app_mcu, sizeof(g_version.app_mcu), "%s", version_data);
            mcu_ver = g_version.app_mcu;
        }
    }

    if (fs_sdk_mcu_version_is_placeholder(mcu_ver)) {
        mcu_ver = "unknown";
    }

    FS_LOGI("MCU firmware version: %s", mcu_ver);
    fprintf(stderr, "[soc-app] MCU firmware version: %s\n", mcu_ver);
    fflush(stderr);
}

// 初始化 MIPI 速率, 单位: 100Mbps
#define SERDES_MIPI_RATE_100MBPS    (12)

// MUC 固件路径
#define MCU_FIRMWARE_PATH "/etc/mcu_firmware.bin"

static fs_work_mode_map_t m_sdk_mode_map[FS_SDK_WORK_MODE_CNT];
static fs_work_mode_map_t m_now_mode = {FS_SDK_WORK_MODE_IDLE, RX_LIDAR_MODE_IDLE};

/**
 * @brief 获取 TRX 硬件配置: 根据 TRX 版本获取对应的硬件配置以支持不同版本 TRX 的兼容
 * @param hw_cfg 输出参数, 用于返回 TRX 硬件配置
 * @param trx_version 输出参数, 可选, 用于返回 TRX 版本信息
 * @return 0 表示成功，非 0 表示失败
 */
static int32_t fs_sdk_get_trx_map(trx_hw_cfg_t *hw_cfg, uint32_t *trx_version)
{
    int32_t err = FS_SUCCESS;
    uint32_t version = 0;
    CHECK_PTR(hw_cfg);

    if (fs_sensor_get_trx_version(&version) != FS_SUCCESS) {
        version = 0xFF; // Default to 0xFF if version cannot be retrieved
    }

    if (trx_version != NULL) {
        *trx_version = version;
    }

    if (version == 2) {
        CHECK_ERR(rx_cfg_get_hw_cfg(TX_HW_MODE_1H_2L_R, hw_cfg));
    }
    else if (version == 3) {
        CHECK_ERR(rx_cfg_get_hw_cfg(TX_HW_MODE_2H_2L_TRP01, hw_cfg));
    }
    else { // customer hardware or unknown hardware, using default configuration
        CHECK_ERR(rx_cfg_get_hw_cfg(TX_HW_MODE_DEFAULT, hw_cfg));
    }

    return err;
}

fs_sdk_work_mode_e fs_sdk_get_work_mode(void)
{
    return m_now_mode.work_mode;
}

const char *fs_sdk_get_version(void)
{
    return SOC_SDK_VERSION "_" GIT_VERSION;
}

int32_t fs_sdk_pc_support(const pc_callbacks *cbs)
{
    // 初始化 UDP 服务: 支持发送点云数据
    CHECK_ERR(fs_udp_server_init());
    // 可选: 初始化阜时 PC 工具支持模块
    CHECK_ERR(fspc_support_init(cbs));
    return FS_SUCCESS;
}

void fs_sdk_pc_cleanup(void)
{
    // 清理阜时 PC 工具支持模块
    // fspc_cleanup();
    return;
}

int32_t fs_sdk_mcu_upgrade(void)
{
    int32_t err;
    int32_t restore_err = FS_SUCCESS;
    int32_t current_err;
    fs_sdk_work_mode_e mode_after;
    mcu_version_t target_version = {0};
    mcu_version_t current_version = {0};

    err = soc_mgr_upgrade_enter();
    if (err) {
        return err;
    }

    (void)soc_mgr_time_sync_stop();

    err = mcu_upgrade_load_firmware(MCU_FIRMWARE_PATH, &target_version, NULL, NULL);
    if (err != FS_SUCCESS) {
        FS_LOGW("target_fw_unavailable path=%s err=%d", MCU_FIRMWARE_PATH, err);
        FS_LOGI("upgrade_decision=skip_target_unavailable");
        err = FS_SUCCESS;
        goto exit_upgrade_gate;
    }
    FS_LOGI("target_fw_version=%s", target_version.hrd.version_string);

    current_err = mcu_upgrade_get_hw_version(&current_version);
    if (current_err == FS_SUCCESS) {
        snprintf(g_version.app_mcu, sizeof(g_version.app_mcu), "%s", current_version.hrd.version_string);
        FS_LOGI("current_mcu_version=%s", current_version.hrd.version_string);

        if (mcu_upgrade_compare_version(&current_version, &target_version) == 0) {
            FS_LOGI("upgrade_decision=skip_same");
            err = FS_SUCCESS;
            goto exit_upgrade_gate;
        }

        FS_LOGI("upgrade_decision=upgrade_version_mismatch");
    } else {
        FS_LOGW("current_mcu_version_read_failed err=%d", current_err);
        FS_LOGI("upgrade_decision=upgrade_current_read_failed");
    }

    err = mcu_upgrade_fw(MCU_FIRMWARE_PATH);

exit_upgrade_gate:
    mode_after = fs_sdk_get_work_mode();
    if (mode_after != FS_SDK_WORK_MODE_IDLE) {
        int32_t ts_err = soc_mgr_time_sync_start_for_mode(mode_after);
        if (ts_err) {
            FS_LOGE("restore time sync publisher failed after upgrade: %d", ts_err);
            if (restore_err == FS_SUCCESS) {
                restore_err = ts_err;
            }
        }
    }

    soc_mgr_upgrade_leave(err);

    if (err) {
        return err;
    }
    return restore_err;
}

int32_t fs_sdk_remap_mode(fs_work_mode_map_t *mode_map)
{
    int32_t i, j;
    CHECK_PTR(mode_map);

    for (i = 0; i < (int32_t)(sizeof(m_sdk_mode_map) / sizeof(m_sdk_mode_map[0])); i++) {
        m_sdk_mode_map[i] = mode_map[i];

        for (j = 0; j < i; j++) {
            if (m_sdk_mode_map[j].work_mode == m_sdk_mode_map[i].work_mode) {
                FS_LOGE("Duplicate work_mode in mode_map: %d", m_sdk_mode_map[i].work_mode);
                return FS_ERR_BAD_PARAMS;
            }
        }
    }

    return FS_SUCCESS;
}

int32_t fs_sdk_core_init(fs_work_mode_map_t *mode_map, const char *cfg_load_path)
{
    soc_mgr_init();
    // 映射 SDK 工作模式与 RX 激光雷达模式的关系
    CHECK_ERR(fs_sdk_remap_mode(mode_map));
    // 初始化激光雷达配置模块
    CHECK_ERR(rx_cfg_loader_init(cfg_load_path));
    // 初始化 sensor_hub(TX & RX 抽象层)
    CHECK_ERR(fs_sensor_init());
    // TODO: 初始化 RX 数据接收模块(仅初始化, 还不能工作)
    // TODO: 初始化温飘修正模块(在 sdk 暴露开关)
    // TODO: 初始化健康监测模块
    return FS_SUCCESS;
}

int32_t fs_sdk_setup_mode(fs_sdk_work_mode_e work_mode)
{
    return soc_mgr_setup_mode(work_mode);
}

int32_t fs_sdk_setup_mode_impl(fs_sdk_work_mode_e work_mode)
{
    int32_t err = FS_ERR_NOT_FOUND;
    int32_t i;
    rx_lidar_mode_e lidar_mode;
    static rx_staged_reg_list_t patched_reg_list;
    const rx_staged_reg_list_t *reg_list;
    const rx_parse_cfg_t       *rx_parse;
    trx_hw_cfg_t                hw_cfg;
    tx_sequence_t               tx_seq;
    uint32_t trx_version = 0;

    FS_LOGD("work_mode: %d", work_mode);

    for (i = 0; i < (int32_t)(sizeof(m_sdk_mode_map) / sizeof(m_sdk_mode_map[0])); i++) {
        if (m_sdk_mode_map[i].work_mode == work_mode) {
            lidar_mode = m_sdk_mode_map[i].lidar_mode;
            err = FS_SUCCESS;
            break;
        }
    }

    if (err) {
        FS_LOGE("Work mode %d not found in m_sdk_mode_map", work_mode);
        return err;
    }

    m_now_mode = m_sdk_mode_map[i];
    // (若在工作)停止 sensor, 忽略失败
    fs_sensor_stop();
    rxdata_service_stop();
    algo_deinit();
    rxdata_service_cleanup_csi();
    // 获取对应雷达模式的寄存器配置列表和数据解析配置
    CHECK_ERR(fs_sdk_get_trx_map(&hw_cfg, &trx_version));
    CHECK_ERR(rx_cfg_get_reg_list(lidar_mode, &reg_list));
    CHECK_ERR(rx_cfg_get_rx_parse(lidar_mode, &rx_parse));
    CHECK_ERR(rx_cfg_check_validity(&hw_cfg, rx_parse));
    memset(&patched_reg_list, 0, sizeof(patched_reg_list));
    CHECK_ERR(rx_cfg_patching_reg_list(reg_list, &hw_cfg, &patched_reg_list));
    CHECK_ERR(rx_cfg_gen_tx_seq(&hw_cfg, rx_parse, &tx_seq));
    // CHECK_ERR(serdes_update_rate(xx)); // TODO: 根据 rx_parse 更新 Serdes 速率
    // 根据 rx_parse 重新配置 RX 数据接收模块和算法模块(等待数据状态, 数据一来就可以工作)
    CHECK_ERR(rxdata_service_init(rx_parse));
    CHECK_ERR(algo_init(rx_parse, work_mode));
    // 根据 reg_list 调用 sensor_hub 配置 RX 寄存器
    CHECK_ERR(fs_sensor_cfg_rx(&patched_reg_list, true));
    // 根据 reg_list 调用 sensor_hub 配置 TX 寄存器
    if (m_now_mode.work_mode == FS_SDK_WORK_MODE_GRAY || m_now_mode.lidar_mode == RX_LIDAR_MODE_GRAY_CUSTOM) {
        // 灰度图模式下强制启用 TX_LO_1, 避免AA过程出来干扰
        CHECK_ERR(fs_sensor_cfg_tx(&hw_cfg, &tx_seq, true, true));
    } else {
        CHECK_ERR(fs_sensor_cfg_tx(&hw_cfg, &tx_seq, true, false));
    }
    // 使能 VPP 电压
    CHECK_ERR(fs_hw_enable_vpp(true));
    // 使能 VBD 电压
    CHECK_ERR(fs_hw_enable_vbd(true));
    // 更新版本信息
    g_version.txver = trx_version;
    snprintf(g_version.rx_ref, sizeof(g_version.rx_ref), "V%02X", trx_version);
    snprintf(g_version.tx_ref, sizeof(g_version.tx_ref), "V%02X", trx_version);
    fs_sensor_get_sn(FS_RX_FL6X32_1, g_version.spad_sn, sizeof(g_version.spad_sn));
    fs_sensor_get_sn(FS_TX_FL9103_1, g_version.tx_lo_sn[0], sizeof(g_version.tx_lo_sn[0]));
    fs_sensor_get_sn(FS_TX_FL9103_2, g_version.tx_lo_sn[1], sizeof(g_version.tx_lo_sn[1]));
    FS_LOGD("SPAD SN: %s, TX_LO SN: [%s, %s]", g_version.spad_sn, g_version.tx_lo_sn[0], g_version.tx_lo_sn[1]);

    if (work_mode == FS_SDK_WORK_MODE_GRAY) {
        fs_hw_enable_temp_comp(false); 
    }
    else {
        fs_hw_enable_temp_comp(true); // 温飘修正仅在非灰度图模式下启用
    }

    // 启动测距
    CHECK_ERR(fs_sensor_start());
    return err;
}

int32_t fs_sdk_cfg_tx_mode(tx_seq_mode_e mode, const uint8_t *chns, uint16_t chn_num)
{
    int32_t err = FS_SUCCESS;
    tx_cfg_seq_mode_e tx_mode;
    const rx_parse_cfg_t       *rx_parse;
    trx_hw_cfg_t                hw_cfg;
    tx_sequence_t               tmp_tx_seq;

    static fs_work_mode_map_t last_mode = {FS_SDK_WORK_MODE_IDLE, RX_LIDAR_MODE_IDLE};

    switch (mode) {
        case TX_SEQ_MODE_REVERSE: {
            tx_mode = TX_CFG_SEQ_MODE_REVERSE;
            break;
        }

        case TX_SEQ_MODE_FORWARD: {
            tx_mode = TX_CFG_SEQ_MODE_FORWARD;
            break;
        }

        case TX_SEQ_MODE_SINGLE_Y_CHN: {
            tx_mode = TX_CFG_SEQ_MODE_SINGLE_Y_CHN;
            break;
        }

        case TX_SEQ_MODE_MULTI_Y_CHN: {
            tx_mode = TX_CFG_SEQ_MODE_MULTI_Y_CHN;
            break;
        }

        case TX_SEQ_MODE_SINGLE_X_CHN: {
            tx_mode = TX_CFG_SEQ_MODE_SINGLE_X_CHN;
            break;
        }

        case TX_SEQ_MODE_BLOCK_CHN: {
            tx_mode = TX_CFG_SEQ_MODE_BLOCK_CHN;
            break;
        }

        case TX_SEQ_MODE_MULTI_X_CHN: {
            tx_mode = TX_CFG_SEQ_MODE_MULTI_X_CHN;
            break;
        }

        case TX_SEQ_MODE_CTRL_TX: {
            tx_mode = TX_CFG_SEQ_MODE_CTRL_TX;
            break;
        }

        case TX_SEQ_MODE_TX_RX_MERGE: {
            tx_mode = TX_CFG_SEQ_MODE_TX_RX_MERGE;
            break;
        }

        case TX_SEQ_MODE_TX_SUSPEND: {
            tx_mode = TX_CFG_SEQ_MODE_TX_SUSPEND;
            break;
        }
        
        case TX_SEQ_MODE_TX_RESUME: {
            tx_mode = TX_CFG_SEQ_MODE_TX_RESUME;
            break;
        }

        default: {
            FS_LOGE("Invalid TX sequence mode: %d", mode);
            return FS_ERR_BAD_PARAMS;
        }
    }

    if (TX_SEQ_MODE_CTRL_TX== mode) {
        last_mode.lidar_mode = m_now_mode.lidar_mode;
        last_mode.work_mode = m_now_mode.work_mode;
        m_now_mode.lidar_mode = RX_LIDAR_MODE_GRAY_CUSTOM;
        m_now_mode.work_mode = FS_SDK_WORK_MODE_GRAY_CUSTOM;
    } else {
        if (last_mode.lidar_mode != RX_LIDAR_MODE_IDLE) {
            FS_LOGI("Restoring previous mode: work_mode=%d, lidar_mode=%d", last_mode.work_mode, last_mode.lidar_mode);
            m_now_mode.lidar_mode = last_mode.lidar_mode;
            m_now_mode.work_mode = last_mode.work_mode;
        } else {
            FS_LOGI("No previous mode to restore, keeping current mode: work_mode=%d, lidar_mode=%d", m_now_mode.work_mode, m_now_mode.lidar_mode);
        }
    }

    CHECK_ERR(fs_sdk_get_trx_map(&hw_cfg, NULL));
    CHECK_ERR(rx_cfg_get_rx_parse(m_now_mode.lidar_mode, &rx_parse));
    CHECK_ERR(rx_cfg_check_validity(&hw_cfg, rx_parse));
    CHECK_ERR(rx_cfg_gen_tx_seq(&hw_cfg, rx_parse, &tmp_tx_seq));
    CHECK_ERR(rx_cfg_gen_trx_seq(tx_mode, chns, chn_num, &tmp_tx_seq));
    if (m_now_mode.work_mode == FS_SDK_WORK_MODE_GRAY || m_now_mode.lidar_mode == RX_LIDAR_MODE_GRAY_CUSTOM) {
        // 灰度图模式下强制启用 TX_LO_1, 避免AA过程出来干扰
        CHECK_ERR(fs_sensor_cfg_tx(&hw_cfg, &tmp_tx_seq, false, true));
    } else {
        CHECK_ERR(fs_sensor_cfg_tx(&hw_cfg, &tmp_tx_seq, false, false));
    }

    return err;
}

static int32_t fs_sdk_remap_chip_device(chip_device_e dev_id, fs_sensor_tx_rx_e *idx)
{
    switch (dev_id) {
        case CHIP_DEVICE_RX: {
            *idx = FS_RX_FL6X32_1;
            break;
        }

        case CHIP_DEVICE_TX_H_ALL: {
            *idx = FS_TX_FL9003_ALL;
            break;
        }

        case CHIP_DEVICE_TX_H0: {
            *idx = FS_TX_FL9003_1;
            break;
        }

        case CHIP_DEVICE_TX_H1: {
            *idx = FS_TX_FL9003_2;
            break;
        }

        case CHIP_DEVICE_TX_H2: {
            *idx = FS_TX_FL9003_3;
            break;
        }

        case CHIP_DEVICE_TX_L_ALL: {
            *idx = FS_TX_FL9103_ALL;
            break;
        }

        case CHIP_DEVICE_TX_L0: {
            *idx = FS_TX_FL9103_1;
            break;
        }

        case CHIP_DEVICE_TX_L1: {
            *idx = FS_TX_FL9103_2;
            break;
        }

        default: {
            FS_LOGE("Invalid chip device: %d", dev_id);
            return FS_ERR_BAD_PARAMS;
        }
    }

    return FS_SUCCESS;
}

int32_t fs_sdk_get_reg(chip_device_e dev_id, uint32_t reg_addr, uint32_t *reg_val, uint32_t *reg_cnt)
{
    fs_sensor_tx_rx_e idx;
    CHECK_ERR(fs_sdk_remap_chip_device(dev_id, &idx));
    return fs_sensor_read_register(idx, reg_addr, reg_val, reg_cnt);
}

int32_t fs_sdk_set_reg(chip_device_e dev_id, uint32_t reg_addr, const uint32_t *reg_val, uint32_t reg_cnt)
{
    fs_sensor_tx_rx_e idx;
    CHECK_ERR(fs_sdk_remap_chip_device(dev_id, &idx));
    return fs_sensor_write_register(idx, reg_addr, reg_val, reg_cnt);
}

int32_t fs_sdk_mcu_init(void)
{
    // Ensure log reader is active in case MCU was reset before init.
    mcu_log_reader_resume();
    // 初始化 MCU 协议（含启动时 MCU 固件版本查询）
    CHECK_ERR(mcu_prot_init());
    FS_LOGI("MCU protocol ready, firmware: %s", g_version.app_mcu);
    /* TimeSync publisher is lazily prepared when entering PCL mode (via soc_mgr_setup_mode) */
    CHECK_ERR(imu_service_init());
    fs_sdk_print_mcu_firmware_version();
    return FS_SUCCESS;
}

int32_t fs_sdk_hw_init(void)
{
    int32_t err = FS_SUCCESS;
    snprintf(g_version.framework, sizeof(g_version.framework), "%s", SOC_SDK_VERSION "_" GIT_VERSION);
    snprintf(g_version.app_soc  , sizeof(g_version.app_soc  ), "%s", SOC_SDK_VERSION "_" GIT_VERSION);
    // 初始化 I2C 主控制器
    CHECK_ERR(i2c_master_init());
    // 尝试初始化 serdes，部分平台可能没有该设备，忽略未检测到设备的错误
    err = serdes_init(SERDES_MIPI_RATE_100MBPS);

    if (err == FS_ERR_NO_DEVICE) {
        err = FS_SUCCESS;  // 忽略未检测到 SerDes 设备的错误
    }
    else if (err) {
        FS_LOGE("serdes_init failed: %d", err);
        return err;
    }

    // 初始化引脚控制
    CHECK_ERR(pinctrl_init());
    // Ensure MCU log reader is active before power-on reset to capture early logs.

    if (mcu_log_route_enabled()) {
        mcu_log_reader_start();
        mcu_log_reader_resume();
    }

    // MCU 上电
    CHECK_ERR(mcu_power_on());
    // FIXME: 等待 MCU 初始化完毕
    fs_hw_msleep(2000);
    return FS_SUCCESS;
}

void fs_sdk_cleanup(void)
{
    // 配置 SDK 工作模式为待机模式并忽略错误
    fs_sdk_setup_mode(FS_SDK_WORK_MODE_IDLE);
    memset(m_sdk_mode_map, 0x00, sizeof(m_sdk_mode_map));
    // TODO: 清理健康监测模块
    // TODO: 清理温飘修正模块
    // 清理 RX 数据接收模块和算法模块
    rxdata_service_stop();
    algo_deinit();
    rxdata_service_cleanup_csi();
    rxdata_service_deinit();
    // 清理激光雷达配置模块
    fs_sensor_deinit();
    rx_cfg_loader_cleanup();
    // 清理 PC 工具支持模块的相关资源
    fs_sdk_pc_cleanup();
    // 反初始化 MCU 协议
    (void)soc_mgr_time_sync_stop();
    mcu_prot_deinit();
    // 停止 MCU 日志读取
    if (mcu_log_route_enabled()) {
        mcu_log_reader_stop();
    }
    // 反初始化引脚控制
    pinctrl_deinit();
    // 反初始化 Serdes
    serdes_deinit();
    // 反初始化 I2C 主控制器
    i2c_master_close();
}
