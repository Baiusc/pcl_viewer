#ifndef FS_UART_H
#define FS_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 打开串口设备
 * @param dev_name 串口设备路径 (如 "/dev/ttyS2")
 * @return 成功返回文件描述符(>=0)，失败返回负数错误码
 */
int32_t fs_uart_open(const char *dev_name);

/**
 * @brief 初始化串口参数 (8N1, 无流控, 原始模式)
 * @param fd 串口文件描述符
 * @param speed 波特率
 * @return 0表示成功，非0表示失败
 */
int32_t fs_uart_init(int32_t fd, uint32_t speed);

/**
 * @brief 打开并初始化串口 (open + init 的便捷组合)
 * @param dev_name 串口设备路径
 * @param speed 波特率
 * @return 成功返回文件描述符(>=0)，失败返回负数错误码
 */
int32_t fs_uart_open_init(const char *dev_name, uint32_t speed);

/**
 * @brief 关闭串口
 * @param fd 串口文件描述符
 * @return 0表示成功，非0表示失败
 */
int32_t fs_uart_close(int32_t fd);

#ifdef __cplusplus
}
#endif

#endif /* FS_UART_H */
