#ifndef __SDK_API_H__
#define __SDK_API_H__

#include <stdint.h>
#include "fspc_support.h"
#include "rx_cfg_loader.h"

typedef struct {
    fs_sdk_work_mode_e work_mode ;
    rx_lidar_mode_e    lidar_mode;
} fs_work_mode_map_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取 SDK 版本信息
 * @return SDK 版本字符串
 */
const char* fs_sdk_get_version(void);

/**
 * @brief 获取当前 SDK 工作模式
 * @return 当前工作模式
 */
fs_sdk_work_mode_e fs_sdk_get_work_mode(void);

/**
 * @brief 使能 PC 工具支持(可选)
 * @param cbs PC 工具回调函数集合, 由调用方提供具体实现以支持 PC 工具的功能需求;
 *            若调用方不需要 PC 工具回调支持, 可传入 NULL;
 *            也可在传入 cbs 时仅实现部分回调函数, 未实现部分填为 NULL;
 * @return 0 表示成功，非 0 表示失败
 */
int32_t fs_sdk_pc_support(const pc_callbacks *cbs);

/**
 * @brief 清理 PC 工具支持模块的资源
 */
void fs_sdk_pc_cleanup(void);

/**
 * @brief 升级 MCU 固件(可选)
 * @return 0 表示成功，非 0 表示失败
 */
int32_t fs_sdk_mcu_upgrade(void);

/**
 * @brief 重新映射 SDK 工作模式与 RX 激光雷达模式的关系(同一个工作模式存在多种 RX 激光雷达模式时, 可以切换映射关系)
 * @param mode_map 工作模式映射表, 用于指定 SDK 工作模式与 RX 激光雷达模式之间的映射关系, 长度必须是 FS_SDK_WORK_MODE_CNT;
 * @return 0 表示成功，非 0 表示失败
 */
int32_t fs_sdk_remap_mode(fs_work_mode_map_t *mode_map);

/**
 * @brief 初始化 SDK 核心功能
 * @param mode_map 工作模式映射表, 用于指定 SDK 工作模式与 RX 激光雷达模式之间的映射关系, 长度必须是 FS_SDK_WORK_MODE_CNT;
 * @param cfg_load_path RX 配置文件存储路径, SDK 将从该路径加载用户自定义的配置文件;
 *                      若传入 NULL, 则仅使用 SDK 内置的预设配置;
 * @return 0 表示成功，非 0 表示失败
 */
int32_t fs_sdk_core_init(fs_work_mode_map_t *mode_map, const char *cfg_load_path);

/**
 * @brief 配置 SDK 工作模式
 * @param work_mode 工作模式, 详见 fs_sdk_work_mode_e 枚举类型
 * @return 0 表示成功，非 0 表示失败
 */
int32_t fs_sdk_setup_mode(fs_sdk_work_mode_e work_mode);

/**
 * @brief 配置 TX 工作模式
 * @param mode     TX 工作模式
 * @param chns     多通道模式下的通道列表
 * @param chn_num  多通道模式下的通道数量
 * @return 0 表示成功，非 0 表示失败
 */
int32_t fs_sdk_cfg_tx_mode(tx_seq_mode_e mode, const uint8_t *chns, uint16_t chn_num);

/**
 * @brief 读取设备寄存器: 支持 PC 工具读取设备寄存器的值(支持连续读取多个寄存器)
 * @param dev_id 设备 ID, 详见 chip_device_e 枚举类型
 * @param reg_addr 寄存器地址
 * @param reg_val 输出参数, 用于返回寄存器值的缓冲区指针
 * @param reg_cnt 输入输出参数, 输入时表示 reg_val 的缓冲区长度, 输出时表示实际读到的寄存器个数
 * @return 0 表示成功，非 0 表示失败
 */
int32_t fs_sdk_get_reg(chip_device_e dev_id, uint32_t reg_addr, uint32_t *reg_val, uint32_t *reg_cnt);

/**
 * @brief 写入设备寄存器: 支持 PC 工具写入设备寄存器的值(支持连续写入多个寄存器)
 * @param dev_id 设备 ID, 详见 chip_device_e 枚举类型
 * @param reg_addr 寄存器地址
 * @param reg_val 寄存器值的缓冲区指针
 * @param reg_cnt 寄存器个数, 表示 reg_val 缓冲区中寄存器值的数量
 * @return 0 表示成功，非 0 表示失败
 */
int32_t fs_sdk_set_reg(chip_device_e dev_id, uint32_t reg_addr, const uint32_t *reg_val, uint32_t reg_cnt);

/**
 * @brief 初始化 MCU
 * @return 0 表示成功，非 0 表示失败
 */
int32_t fs_sdk_mcu_init(void);

/**
 * @brief SDK 初始化硬件
 * @return 0 表示成功，非 0 表示失败
 */
int32_t fs_sdk_hw_init(void);

/**
 * @brief SDK 清理函数
 */
void fs_sdk_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* __SDK_API_H__ */
