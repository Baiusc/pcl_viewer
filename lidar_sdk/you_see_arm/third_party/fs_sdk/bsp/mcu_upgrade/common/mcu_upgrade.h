#ifndef MCU_UPGRADE_API_H
#define MCU_UPGRADE_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 硬件复位 MCU
 * @param duration_us 复位持续时间，单位微秒；如果为 0 或负数，默认为 1 微秒
 * @return 0表示成功，非0表示失败
*/
int32_t hwResetMcu(uint32_t duration_us);

/**
 * @brief 上电 MCU
 * @return 0 表示成功，非 0 表示失败
 */
int32_t mcu_power_on(void);

/**
 * @brief 初始化 MCU 升级模块
 * @return 0表示成功，非0表示失败
 */
int32_t mcu_upgrade_init(void);

/**
 * @brief 清理 MCU 升级模块
 */
void mcu_upgrade_cleanup(void);

/**
 * @brief MCU 固件版本包头
 */
typedef struct {
    uint8_t hrd_ver;
    uint8_t hrd_len;
    char version_string[32];
    char date[16];
    uint16_t fw_offset;
    uint32_t fw_len;
    uint16_t fw_crc;
} mcu_version_hrd_t;

/**
 * @brief MCU 版本信息
 */
typedef struct {
    mcu_version_hrd_t hrd;
    uint32_t major;
    uint32_t minor;
    uint32_t patch;
    char git_commit[9];
    char pipeline_id[16];
} mcu_version_t;

/**
 * @brief 解析版本字符串 "v0.0.1_00000000-00000000"
 * @param version_string 版本字符串
 * @param version 输出版本信息
 * @return 0 表示成功，非 0 表示失败
 */
int32_t mcu_upgrade_parse_version_string(const char *version_string, mcu_version_t *version);

/**
 * @brief 加载 MCU 固件包头并按需读取负载
 * @param firmware_path 固件文件路径
 * @param version 输出版本信息，可为 NULL
 * @param fw_buf_out 输出固件负载，可为 NULL；调用方负责 free
 * @param fw_len_out 输出固件负载长度，可为 NULL
 * @return 0 表示成功，非 0 表示失败
 */
int32_t mcu_upgrade_load_firmware(const char *firmware_path,
                                  mcu_version_t *version,
                                  uint8_t **fw_buf_out,
                                  uint32_t *fw_len_out);

/**
 * @brief 比较 MCU 版本字符串
 * @param current 当前版本
 * @param target 目标版本
 * @return 0 表示版本相同，非 0 表示不同
 */
int mcu_upgrade_compare_version(const mcu_version_t *current, const mcu_version_t *target);

/**
 * @brief 读取当前 MCU 固件版本
 * @param version 输出版本信息
 * @return 0 表示成功，非 0 表示失败
 */
int32_t mcu_upgrade_get_hw_version(mcu_version_t *version);

/**
 * @brief 升级 MCU 固件
 * @param firmware_path 固件文件路径
 * @return 0表示成功，非0表示失败
 */
int32_t mcu_upgrade_fw(const char *firmware_path);

#ifdef __cplusplus
}
#endif

#endif /* MCU_UPGRADE_API_H */