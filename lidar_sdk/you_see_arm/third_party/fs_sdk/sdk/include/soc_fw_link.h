#ifndef SOC_FW_LINK_H
#define SOC_FW_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "fw_link.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int (*send_req_wait)(fw_link_channel_t channel,
                         uint8_t tag,
                         uint8_t cmd_type,
                         const uint8_t *data,
                         uint16_t data_len,
                         uint8_t *resp_buf,
                         uint16_t resp_buf_size,
                         uint16_t *resp_len,
                         uint32_t timeout_ms);
    int (*send_push)(fw_link_channel_t channel,
                     uint8_t tag,
                     const uint8_t *payload,
                     uint16_t payload_len);
} soc_fw_link_transport_ops_t;

void soc_fw_link_set_transport_ops(const soc_fw_link_transport_ops_t *ops);
bool soc_fw_link_channel_enabled(fw_link_channel_t channel);
int soc_fw_link_send_req_wait(fw_link_channel_t channel,
                              uint8_t tag,
                              uint8_t cmd_type,
                              const uint8_t *data,
                              uint16_t data_len,
                              uint8_t *resp_buf,
                              uint16_t resp_buf_size,
                              uint16_t *resp_len,
                              uint32_t timeout_ms);
int soc_fw_link_send_push(fw_link_channel_t channel,
                          uint8_t tag,
                          const uint8_t *payload,
                          uint16_t payload_len);

#ifdef __cplusplus
}
#endif

#endif /* SOC_FW_LINK_H */
