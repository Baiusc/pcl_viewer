#include "soc_fw_link.h"

#include <string.h>

#include "fs_top_module_cfg.h"
#include "utils/fs_err.h"

static soc_fw_link_transport_ops_t s_transport_ops;

static fw_link_backend_type_t soc_fw_link_backend_for_channel(fw_link_channel_t channel)
{
    switch (channel) {
    case FW_LINK_CHANNEL_CONTROL:
        return (fw_link_backend_type_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_CONTROL_CORE);
    case FW_LINK_CHANNEL_STORAGE:
        return (fw_link_backend_type_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_CFG_STORAGE);
    case FW_LINK_CHANNEL_UPDATE_BOOT:
        return (fw_link_backend_type_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_UPGRADE);
    case FW_LINK_CHANNEL_DIAGNOSTIC:
        return (fw_link_backend_type_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_DIAGNOSTIC);
    case FW_LINK_CHANNEL_TIME_SYNC:
        return (fw_link_backend_type_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_TIME_SYNC);
    case FW_LINK_CHANNEL_IMU_PUSH:
        return (fw_link_backend_type_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_IMU_PUSH);
    case FW_LINK_CHANNEL_LOG:
        return (fw_link_backend_type_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_LOG);
    case FW_LINK_CHANNEL_OTHER:
        return FW_LINK_BACKEND_UART;
    default:
        return FW_LINK_BACKEND_DISABLED;
    }
}

void soc_fw_link_set_transport_ops(const soc_fw_link_transport_ops_t *ops)
{
    if (ops == NULL) {
        (void)memset(&s_transport_ops, 0, sizeof(s_transport_ops));
        return;
    }

    s_transport_ops = *ops;
}

bool soc_fw_link_channel_enabled(fw_link_channel_t channel)
{
    return soc_fw_link_backend_for_channel(channel) != FW_LINK_BACKEND_DISABLED;
}

int soc_fw_link_send_req_wait(fw_link_channel_t channel,
                              uint8_t tag,
                              uint8_t cmd_type,
                              const uint8_t *data,
                              uint16_t data_len,
                              uint8_t *resp_buf,
                              uint16_t resp_buf_size,
                              uint16_t *resp_len,
                              uint32_t timeout_ms)
{
    fw_link_backend_type_t backend = soc_fw_link_backend_for_channel(channel);

    if (backend == FW_LINK_BACKEND_DISABLED) {
        return FS_ERR_UNSUPPORT;
    }
    if (backend != FW_LINK_BACKEND_UART) {
        return FW_LINK_ERR_NOT_SUPPORTED;
    }
    if (s_transport_ops.send_req_wait == NULL) {
        return FW_LINK_ERR_BACKEND_UNAVAILABLE;
    }

    return s_transport_ops.send_req_wait(channel,
                                         tag,
                                         cmd_type,
                                         data,
                                         data_len,
                                         resp_buf,
                                         resp_buf_size,
                                         resp_len,
                                         timeout_ms);
}

int soc_fw_link_send_push(fw_link_channel_t channel,
                          uint8_t tag,
                          const uint8_t *payload,
                          uint16_t payload_len)
{
    fw_link_backend_type_t backend = soc_fw_link_backend_for_channel(channel);

    if (backend == FW_LINK_BACKEND_DISABLED) {
        return FS_ERR_UNSUPPORT;
    }
    if (backend != FW_LINK_BACKEND_UART) {
        return FW_LINK_ERR_NOT_SUPPORTED;
    }
    if (s_transport_ops.send_push == NULL) {
        return FW_LINK_ERR_BACKEND_UNAVAILABLE;
    }

    return s_transport_ops.send_push(channel, tag, payload, payload_len);
}
