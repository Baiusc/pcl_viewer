#include "fw_link.h"
#include "../project/fs_top_module_cfg.h"
#include <string.h>

#define FW_LINK_BACKEND_SLOT_COUNT 4u

#define FW_LINK_POLICY_NO_DROP(priority_) \
    { (priority_), 0u, 0u, 0u }

#define FW_LINK_POLICY_DROP(priority_) \
    { (priority_), 1u, FW_LINK_LOW_PRIORITY_RESERVE_QUEUE_SLOTS, FW_LINK_LOW_PRIORITY_RESERVE_BYTES }

static const fw_link_channel_config_t s_default_channel_config[FW_LINK_CHANNEL_COUNT] = {
    { (uint8_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_CONTROL_CORE), FW_LINK_POLICY_NO_DROP(FW_LINK_PRIORITY_CONTROL) },
    { (uint8_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_CFG_STORAGE), FW_LINK_POLICY_NO_DROP(FW_LINK_PRIORITY_STORAGE) },
    /* Phase 1 reuses the UART backend for upgrade data only; GPIO boot sideband stays external. */
    { (uint8_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_UPGRADE), FW_LINK_POLICY_NO_DROP(FW_LINK_PRIORITY_UPDATE_BOOT) },
    { (uint8_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_DIAGNOSTIC), FW_LINK_POLICY_NO_DROP(FW_LINK_PRIORITY_DIAGNOSTIC) },
    { (uint8_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_TIME_SYNC), FW_LINK_POLICY_DROP(FW_LINK_PRIORITY_TIME_SYNC) },
    { (uint8_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_IMU_PUSH), FW_LINK_POLICY_DROP(FW_LINK_PRIORITY_IMU_PUSH) },
    { (uint8_t)FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_LOG), FW_LINK_POLICY_DROP(FW_LINK_PRIORITY_LOG) },
    { FW_LINK_BACKEND_UART, FW_LINK_POLICY_DROP(FW_LINK_PRIORITY_OTHER) },
};

/* ---- Backend registry and channel profile ---- */
static const fw_link_backend_ops_t *s_backend_ops[FW_LINK_BACKEND_SLOT_COUNT];
static const fw_link_channel_config_t *s_active_channel_config = s_default_channel_config;
static uint8_t s_active_channel_config_count = (uint8_t)FW_LINK_CHANNEL_COUNT;
static uint8_t s_initialized = 0u;

/* Frame buffer — sized to worst-case frame.
 * Placed at file scope to avoid 2KB on caller stacks. */
static uint8_t s_frame_buf[FW_LINK_FRAME_MAX_TOTAL];

static int fw_link_backend_index(fw_link_backend_type_t type)
{
    if (type <= FW_LINK_BACKEND_DISABLED || type >= FW_LINK_BACKEND_SLOT_COUNT) {
        return -1;
    }
    return (int)type;
}

static uint8_t fw_link_backend_type_valid(fw_link_backend_type_t type)
{
    return (type < FW_LINK_BACKEND_SLOT_COUNT) ? 1u : 0u;
}

static const fw_link_backend_ops_t *fw_link_backend_ops(fw_link_backend_type_t type)
{
    int index = fw_link_backend_index(type);

    if (index < 0) {
        return NULL;
    }
    return s_backend_ops[index];
}

void fw_link_init(void)
{
    (void)memset(s_backend_ops, 0, sizeof(s_backend_ops));
    s_active_channel_config = s_default_channel_config;
    s_active_channel_config_count = (uint8_t)FW_LINK_CHANNEL_COUNT;
    s_initialized = 1u;
}

int fw_link_register_backend(fw_link_backend_type_t type, const fw_link_backend_ops_t *ops)
{
    int index;

    if (ops == NULL
        || ops->init == NULL
        || ops->send == NULL
        || ops->recv_poll == NULL
        || ops->get_diag == NULL) {
        return FW_LINK_ERR_INVALID_ARGUMENT;
    }

    index = fw_link_backend_index(type);
    if (index < 0) {
        return FW_LINK_ERR_INVALID_ARGUMENT;
    }

    if (s_backend_ops[index] != NULL) {
        return FW_LINK_ERR_INVALID_ARGUMENT;
    }

    s_backend_ops[index] = ops;
    return 0;
}

int fw_link_configure_channels(const fw_link_channel_config_t *configs, uint8_t config_count)
{
    uint8_t i;

    if (configs == NULL || config_count > FW_LINK_CHANNEL_COUNT) {
        return FW_LINK_ERR_INVALID_ARGUMENT;
    }

    for (i = 0u; i < config_count; i++) {
        if (fw_link_backend_type_valid((fw_link_backend_type_t)configs[i].backend_type) == 0u) {
            return FW_LINK_ERR_INVALID_ARGUMENT;
        }
    }

    s_active_channel_config = configs;
    s_active_channel_config_count = config_count;
    return 0;
}

fw_link_channel_policy_t fw_link_channel_policy(fw_link_channel_t channel)
{
    if (channel >= FW_LINK_CHANNEL_COUNT || channel >= s_active_channel_config_count) {
        return s_default_channel_config[FW_LINK_CHANNEL_OTHER].policy;
    }
    return s_active_channel_config[channel].policy;
}

fw_link_backend_type_t fw_link_channel_backend(fw_link_channel_t channel)
{
    if (channel >= FW_LINK_CHANNEL_COUNT || channel >= s_active_channel_config_count) {
        return FW_LINK_BACKEND_NONE;
    }
    return (fw_link_backend_type_t)s_active_channel_config[channel].backend_type;
}

void fw_link_poll(void)
{
    uint8_t i;

    if (s_initialized == 0u) {
        return;
    }

    for (i = 1u; i < FW_LINK_BACKEND_SLOT_COUNT; i++) {
        if (s_backend_ops[i] != NULL) {
            s_backend_ops[i]->recv_poll();
        }
    }
}

int fw_link_send(fw_link_channel_t channel,
                 uint8_t msg_type,
                 uint16_t txid,
                 uint8_t flags,
                 const uint8_t *payload,
                 uint16_t payload_len)
{
    uint16_t framed_len = 0u;
    int pack_ret;
    fw_link_backend_type_t backend_type;
    const fw_link_backend_ops_t *ops;

    if (s_initialized == 0u) {
        return FW_LINK_ERR_INVALID_ARGUMENT;
    }

    backend_type = fw_link_channel_backend(channel);
    if (backend_type == FW_LINK_BACKEND_DISABLED) {
        return FW_LINK_ERR_NOT_SUPPORTED;
    }

    ops = fw_link_backend_ops(backend_type);
    if (ops == NULL) {
        return FW_LINK_ERR_BACKEND_UNAVAILABLE;
    }

    pack_ret = fw_link_frame_pack(msg_type, txid, flags, payload, payload_len,
                                  s_frame_buf, sizeof(s_frame_buf), &framed_len);
    if (pack_ret != 0) {
        return FW_LINK_ERR_PACK_FAILED;
    }

    return ops->send(channel, s_frame_buf, framed_len);
}
