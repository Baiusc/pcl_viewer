#ifndef MCU_LOG_READER_H
#define MCU_LOG_READER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 MCU 日志读取线程
 *        从 /dev/ttyS2 读取 MCU 日志并通过 FS_LOGI 输出
 * @return 0表示成功，非0表示失败
 */
int mcu_log_reader_start(void);

/**
 * @brief 停止 MCU 日志读取线程
 */
void mcu_log_reader_stop(void);

/**
 * @brief 查询日志读取线程是否正在运行
 */
bool mcu_log_reader_is_running(void);

/**
 * @brief 挂起日志读取（释放串口 FD）
 *        MCU 升级前调用，确保升级可以独占使用串口
 * @return 0表示成功，非0表示失败
 */
int mcu_log_reader_suspend(void);

/**
 * @brief 恢复日志读取（重新打开串口）
 *        MCU 升级完成后调用
 * @return 0表示成功，非0表示失败
 */
int mcu_log_reader_resume(void);

#ifdef __cplusplus
}
#endif

#endif /* MCU_LOG_READER_H */
