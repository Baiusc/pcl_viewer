#ifndef __SERDES_H__
#define __SERDES_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 Serdes
 * @param mipi_rate_100mbps MIPI 速率，单位为 100Mbps，例如 15 表示 15*100Mbps = 1.5Gbps
 * @return 0 成功，非0 失败
 */
int serdes_init(uint8_t mipi_rate_100mbps);

/**
 * @brief 更新 Serdes 速率
 * @param mipi_rate_100mbps MIPI 速率，单位为 100Mbps，例如 15 表示 15*100Mbps = 1.5Gbps
 * @return 0 成功，非0 失败
 */
int serdes_update_rate(uint8_t mipi_rate_100mbps);

/**
 * @brief 反初始化 Serdes
 * @return void
 */
void serdes_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERDES_H__ */
