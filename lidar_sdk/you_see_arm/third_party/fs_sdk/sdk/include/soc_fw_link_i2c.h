#ifndef SOC_FW_LINK_I2C_H
#define SOC_FW_LINK_I2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int soc_fw_link_i2c_send_timesync_push(const uint8_t *payload, uint16_t payload_len);

#ifdef __cplusplus
}
#endif

#endif /* SOC_FW_LINK_I2C_H */
