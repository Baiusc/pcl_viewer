#ifndef __I2C_MASTER_H__
#define __I2C_MASTER_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化I2C主机
 * @return 0 成功，非0 失败
 */
int i2c_master_init(void);

/**
 * @brief 关闭I2C主机
 */
void i2c_master_close(void);

/**
 * @brief 发送写命令到I2C从机
 * @param busno I2C主机总线序号
 * @param slave_addr I2C从机地址(8位)
 * @param reg 命令字节
 * @param reg_len 命令长度(1或2)
 * @param data 数据缓冲区
 * @param data_len 数据长度
 * @return 0 成功，非0 失败
 */
int i2c_master_write(uint8_t busno, uint8_t slave_addr, uint16_t reg, uint8_t reg_len, uint8_t *data, uint16_t data_len);

/**
 * @brief 发送写命令到I2C从机
 * @param busno I2C主机总线序号
 * @param slave_addr I2C从机地址(8位)
 * @param reg 命令字节
 * @param reg_len 命令长度(1或2)
 * @param data 读取的数据缓冲区
 * @param data_len 要读取的数据长度
 * @return 0 成功，非0 失败
 */
int i2c_master_read(uint8_t busno, uint8_t slave_addr, uint16_t reg, uint8_t reg_len, uint8_t *data, uint16_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_MASTER_H__ */
