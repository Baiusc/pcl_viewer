#ifndef __FS_SENSOR_HUB_H__
#define __FS_SENSOR_HUB_H__

#include <stdint.h>
#include <stdbool.h>
#include "fs_hw.h"
#include "rx_cfg_loader.h"
// #include "fs_config.h"

#define FS_SENSOR_TX_LS_MAX_CH_CNT  24  // 低边芯片的最大通道数量
#define FS_SENSOR_TX_HS_MAX_CH_CNT  18  // 高边芯片的最大通道数量
#define FS_SENSOR_TX_MAX_CH_CNT     ((FS_SENSOR_TX_LS_MAX_CH_CNT > FS_SENSOR_TX_HS_MAX_CH_CNT) ? FS_SENSOR_TX_LS_MAX_CH_CNT : FS_SENSOR_TX_HS_MAX_CH_CNT)

#define FS_SENSOR_TX_LS_MAX_DEV_CNT  2  // 低边芯片的最大设备数量
#define FS_SENSOR_TX_HS_MAX_DEV_CNT  3  // 高边芯片的最大设备数量
#define FS_SENSOR_TX_MAX_DEV_CNT     ((FS_SENSOR_TX_LS_MAX_DEV_CNT > FS_SENSOR_TX_HS_MAX_DEV_CNT) ? FS_SENSOR_TX_LS_MAX_DEV_CNT : FS_SENSOR_TX_HS_MAX_DEV_CNT)

#define FS_SENSOR_RX_MAX_DEV_CNT     1  // RX 芯片的最大设备数量

typedef enum {
    DEV_RESET_TYPE_HW = 0,
    DEV_RESET_TYPE_SW = 1,
    DEV_RESET_TYPE_NUM,
} dev_reset_type_t;

typedef enum {
    FS_SENSOR_REG_WIDTH_08 =  8,
    FS_SENSOR_REG_WIDTH_16 = 16,
    FS_SENSOR_REG_WIDTH_24 = 24,
    FS_SENSOR_REG_WIDTH_32 = 32,
} fs_sensor_reg_width_e;

typedef enum {
    FS_SENSOR_CMD_UNDEF            =  0x00,  // 无操作

    FS_SENSOR_CMD_DELAY_S          =  0x01,  // 延时: 单位 s
    FS_SENSOR_CMD_DELAY_MS         =  0x02,  // 延时: 单位 ms
    FS_SENSOR_CMD_DELAY_US         =  0x03,  // 延时: 单位 us
    // FS_SENSOR_CMD_DELAY_NS         =  0x04,  // 延时: 单位 ns

    FS_SENSOR_CMD_READ_REG         =  0x05,  // 读取寄存器
    FS_SENSOR_CMD_HW_RESET         =  0x06,  // 硬件复位
    FS_SENSOR_CMD_WAIT_IRQ         =  0x07,  // 等待中断引脚
    FS_SENSOR_CMD_WAIT_FLAG        =  0x08,  // 等待标志位

    FS_SENSOR_CMD_UNKNOWN          =  0xFF,  // 未知操作
} fs_sensor_cmd_e;

typedef struct {
    float temp_vbd_k;   // 温度对 VBD 的补偿系数(带符号), 单位 V/°C
    float cali_vbd_25C; // 从 RX 存储器中读取的校准 VBD 电压(带符号), 单位 V, 已经补偿到 25°C
} fs_rx_trim_t;

typedef struct {
    // read_buffer/write_buffer 读/写数据时, 需按照 reg_width 解析 data;
    fs_sensor_reg_width_e reg_width;
    tx_cfg_seq_mode_e     seq_mode;
    int16_t  dev_id[FS_SENSOR_TX_MAX_DEV_CNT];                          // 设备 ID 列表
    uint8_t  ch_map[FS_SENSOR_TX_MAX_DEV_CNT][FS_SENSOR_TX_MAX_CH_CNT]; // 通道映射表
    char SN[FS_SENSOR_TX_MAX_DEV_CNT][64]; // TX 芯片序列号字符串

    struct {
        int32_t (*check_alive   )(fs_sensor_tx_rx_e idx);                   // 检测设备是否存在
        int32_t (*get_status    )(fs_sensor_tx_rx_e idx, void *status);     // 获取设备状态, TODO: status 应当定义为一个结构体
        int32_t (*reset         )(fs_sensor_tx_rx_e idx, dev_reset_type_t reset_type);  // 重置设备

        int32_t (*set_sequence  )(fs_sensor_tx_rx_e idx, uint16_t len, const uint64_t sequence[len], bool is_switch); // 设置 TX 工作序列
        int32_t (*get_sequence  )(fs_sensor_tx_rx_e idx, uint64_t *sequence, uint16_t *len); // 获取 TX 工作序列
        int32_t (*start_sequence)(fs_sensor_tx_rx_e idx); // 启动 TX 工作序列
        int32_t (*stop_sequence )(fs_sensor_tx_rx_e idx); // 停止 TX 工作序列

        int32_t (*read_register )(fs_sensor_tx_rx_e idx, uint32_t addr,       uint32_t *data, uint32_t *dlen); // 读取寄存器(数据位宽统一 32bit)
        int32_t (*write_register)(fs_sensor_tx_rx_e idx, uint32_t addr, const uint32_t *data, uint32_t  dlen); // 写入寄存器(数据位宽统一 32bit)
        int32_t (*read_buffer   )(fs_sensor_tx_rx_e idx, uint32_t addr,       void     *data, uint32_t *dlen); // 读取缓冲区(数据位宽按芯片实际位宽)
        int32_t (*write_buffer  )(fs_sensor_tx_rx_e idx, uint32_t addr, const void     *data, uint32_t  dlen); // 写入缓冲区(数据位宽按芯片实际位宽)
        int32_t (*disable_device)(fs_sensor_tx_rx_e idx, bool disable); // 禁用设备, 使其进入不可访问状态
    } op;
} fs_tx_t;

// 前向声明，解决结构体自包含问题
typedef struct fs_rx_s fs_rx_t;

struct fs_rx_s {
    fs_sensor_reg_width_e reg_width;
    bool has_load_cfg;
    const rx_staged_reg_list_t *cfg; // RX 配置指针
    int16_t  dev_id[FS_SENSOR_RX_MAX_DEV_CNT]; // 设备 ID 列表
    char SN[FS_SENSOR_RX_MAX_DEV_CNT][64]; // RX 芯片序列号字符串
    fs_rx_trim_t trim[FS_SENSOR_RX_MAX_DEV_CNT]; // RX 出厂校准参数

    struct {
        int32_t (*check_alive   )(fs_rx_t *rx);             // 检测设备是否存在
        int32_t (*get_status    )(fs_rx_t *rx, void *status);     // 获取设备状态, TODO: status 应当定义为一个结构体
        int32_t (*reset         )(fs_rx_t *rx, bool soft_reset);  // 重置设备
        int32_t (*wakeup        )(fs_rx_t *rx);  // 唤醒设备
        int32_t (*sleep         )(fs_rx_t *rx);  // 进入低功耗模式

        int32_t (*setup_cfg     )(fs_rx_t *rx); // 设置 RX 配置
        int32_t (*start_scan    )(fs_rx_t *rx); // 启动 RX 扫描
        int32_t (*stop_scan     )(fs_rx_t *rx); // 停止 RX 扫描

        int32_t (*read_register )(fs_sensor_tx_rx_e idx, uint32_t addr,       uint32_t *data, uint32_t  dlen); // 读取寄存器(数据位宽统一 32bit)
        int32_t (*write_register)(fs_sensor_tx_rx_e idx, uint32_t addr, const uint32_t *data, uint32_t  dlen); // 写入寄存器(数据位宽统一 32bit)
        int32_t (*read_buffer   )(fs_sensor_tx_rx_e idx, uint32_t addr,       void     *data, uint32_t *dlen); // 读取缓冲区(数据位宽按芯片实际位宽)
        int32_t (*write_buffer  )(fs_sensor_tx_rx_e idx, uint32_t addr, const void     *data, uint32_t  dlen); // 写入缓冲区(数据位宽按芯片实际位宽)
        int32_t (*disable_device)(fs_sensor_tx_rx_e idx, bool disable); // 禁用设备, 使其进入不可访问状态
    } op;
};

typedef struct {
    bool is_initialized;
    fs_tx_t tx[2]; // 0: FL9003, 1: FL9103;
    fs_rx_t rx;
    uint32_t trx_ver; // TRX 板硬件版本号
    const rx_parse_cfg_t *cfg_decode; // MIPI 数据解析配置指针
} fs_sensor_t;

#ifdef __cplusplus
extern "C" {
#endif

// 传感器复位
int32_t fs_sensor_reset(fs_sensor_tx_rx_e idx, dev_reset_type_t reset_type);

// 设置传感器配置
int32_t fs_sensor_set_cfg(void);

// 获取传感器配置
int32_t fs_sensor_get_cfg(void);

// 设置传感器模式
int32_t fs_sensor_set_mode(void);

// 获取传感器模式
int32_t fs_sensor_get_mode(void);

// 配置 tx
int32_t fs_sensor_cfg_tx(const trx_hw_cfg_t *hw_cfg, tx_sequence_t *tx_seq, bool is_set_vpp, bool disable_9103_1);

// 配置 rx
int32_t fs_sensor_cfg_rx(const rx_staged_reg_list_t *reg_list, bool is_set_vbd);

/**
 * @brief 传感器启动
 * @return 0 成功, 负值错误码
 */
int32_t fs_sensor_start(void);

/**
 * @brief 传感器停止
 * @return 0 成功, 负值错误码
 */
int32_t fs_sensor_stop(void);

/**
 * @brief 传感器反初始化
 * @return 0 成功, 负值错误码
 */
void fs_sensor_deinit(void);

/**
 * @brief 传感器初始化
 * @return 0 成功, 负值错误码
 */
int32_t fs_sensor_init(void);

int32_t fs_sensor_read_register(fs_sensor_tx_rx_e idx, uint32_t addr,uint32_t *data, uint32_t *dlen);
int32_t fs_sensor_write_register(fs_sensor_tx_rx_e idx, uint32_t addr,const uint32_t *data, uint32_t dlen);
int32_t fs_sensor_set_sequence(fs_sensor_tx_rx_e idx, uint16_t len, const uint64_t sequence[len], bool is_switch);
int32_t fs_sensor_get_sequence(fs_sensor_tx_rx_e idx, uint64_t *sequence, uint16_t *len);
int32_t fs_sensor_start_sequence(fs_sensor_tx_rx_e idx);
int32_t fs_sensor_stop_sequence(fs_sensor_tx_rx_e idx);
int32_t fs_sensor_check_alive(fs_sensor_tx_rx_e idx);
int32_t fs_sensor_get_status(fs_sensor_tx_rx_e idx, void *status);
int32_t fs_sensor_get_trim(fs_sensor_tx_rx_e idx, void *trim, uint32_t tlen);
int32_t fs_sensor_get_sn(fs_sensor_tx_rx_e idx, char *sn, uint32_t sn_buf_len);
int32_t fs_sensor_get_trx_version(uint32_t *version);

int32_t fs_sensor_disable_device(fs_sensor_tx_rx_e idx, bool disable);

#ifdef __cplusplus
}
#endif

#endif /* __FS_SENSOR_HUB_H__ */
