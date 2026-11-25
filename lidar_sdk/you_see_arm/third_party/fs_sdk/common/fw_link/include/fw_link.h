#ifndef FW_LINK_H
#define FW_LINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FW_LINK_VERSION 1u

typedef enum {
    FW_LINK_CHANNEL_CONTROL = 0,
    FW_LINK_CHANNEL_STORAGE,
    FW_LINK_CHANNEL_UPDATE_BOOT,
    FW_LINK_CHANNEL_DIAGNOSTIC,
    FW_LINK_CHANNEL_TIME_SYNC,
    FW_LINK_CHANNEL_IMU_PUSH,
    FW_LINK_CHANNEL_LOG,
    FW_LINK_CHANNEL_OTHER,
    FW_LINK_CHANNEL_COUNT,
    FW_LINK_CHANNEL_INVALID = 0xffu
} fw_link_channel_t;

typedef struct {
    uint8_t priority;
    uint8_t drop_allowed;
    uint8_t reserve_queue_slots;
    uint16_t reserve_bytes;
} fw_link_channel_policy_t;

#define FW_LINK_PRIORITY_CONTROL     0u
#define FW_LINK_PRIORITY_STORAGE     1u
#define FW_LINK_PRIORITY_UPDATE_BOOT 2u
#define FW_LINK_PRIORITY_DIAGNOSTIC  3u
#define FW_LINK_PRIORITY_TIME_SYNC   4u
#define FW_LINK_PRIORITY_IMU_PUSH    5u
#define FW_LINK_PRIORITY_LOG         6u
#define FW_LINK_PRIORITY_OTHER       7u

#define FW_LINK_LOW_PRIORITY_RESERVE_QUEUE_SLOTS 2u
#define FW_LINK_LOW_PRIORITY_RESERVE_BYTES       320u

/* =========================================================================
 * Backend abstraction (Phase 4 Link/Transport layer — fw_link_frame.h)
 * ========================================================================= */

#include "fw_link_frame.h"

/**
 * Backend types known to fw_link.
 */
typedef enum {
    FW_LINK_BACKEND_DISABLED = 0,
    FW_LINK_BACKEND_NONE = FW_LINK_BACKEND_DISABLED,
    FW_LINK_BACKEND_UART = 1,
    FW_LINK_BACKEND_I2C  = 2,
    FW_LINK_BACKEND_HYBRID = 3,
} fw_link_backend_type_t;

#define FW_LINK_ERR_INVALID_ARGUMENT      (-1)
#define FW_LINK_ERR_PACK_FAILED           (-2)
#define FW_LINK_ERR_NOT_SUPPORTED         (-3)
#define FW_LINK_ERR_BACKEND_UNAVAILABLE   (-4)

typedef struct {
    uint8_t backend_type;
    fw_link_channel_policy_t policy;
} fw_link_channel_config_t;

/**
 * Opaque diagnostics snapshot — each backend defines its own struct,
 * cast through this pointer. Consumers agree on the concrete type
 * through the backend_type.
 */
typedef void fw_link_diag_t;

/**
 * Per-backend operations table.
 *
 * On MCU (M0+), indirect calls have measurable cost.  If this becomes
 * a profiling issue, replace the ops table with a compile-time
 * switch(backend_type) in fw_link.c.  The interface stays the same.
 */
typedef struct {
    /** One-time hardware init (GPIO, IRQ, baud rate).  Called once at boot. */
    int  (*init)(void);

    /** Non-blocking send of a fully-framed buffer.  Returns 0 on success,
     *  negative on drop/reject.  The backend owns queueing and TX ISR. */
    int  (*send)(fw_link_channel_t channel, const uint8_t *framed_data, uint16_t framed_len);

    /** Main-loop poll: processes received frames and dispatches to protocol layer. */
    void (*recv_poll)(void);

    /** Fill backend-specific diagnostics snapshot.  diag_buf is owned by caller. */
    void (*get_diag)(fw_link_diag_t *diag_buf);
} fw_link_backend_ops_t;

/* ---- fw_link core API ---- */

/**
 * @brief  Initialise the link layer and reset channel routing to defaults.
 */
void fw_link_init(void);

/**
 * @brief  Register one concrete backend type. Different types may be
 *         registered once each.
 *         The ops table must have static lifetime.
 * @return 0 on success, negative on invalid args or duplicate type.
 */
int fw_link_register_backend(fw_link_backend_type_t type, const fw_link_backend_ops_t *ops);

/**
 * @brief  Configure the first config_count logical channels from a profile table.
 *         The table must have static lifetime.
 */
int fw_link_configure_channels(const fw_link_channel_config_t *configs, uint8_t config_count);

/**
 * @brief  Return the currently configured policy for a channel.
 */
fw_link_channel_policy_t fw_link_channel_policy(fw_link_channel_t channel);

/**
 * @brief  Return the currently configured backend for a channel.
 */
fw_link_backend_type_t fw_link_channel_backend(fw_link_channel_t channel);

/**
 * @brief  Poll all registered backends.
 */
void fw_link_poll(void);

/**
 * @brief  Send a payload on the given logical channel.
 *
 *         Internally:
 *         1. Packs a frame via fw_link_frame_pack()
 *         2. Calls backend->send(channel, framed_data, framed_len)
 *
 *         For REQ/RESP transactions with txid management, callers should use
 *         the protocol-level send which wraps this API.
 *
 * @return 0 on success, negative on error.
 */
int fw_link_send(fw_link_channel_t channel,
                 uint8_t msg_type,
                 uint16_t txid,
                 uint8_t flags,
                 const uint8_t *payload,
                 uint16_t payload_len);

/**
 * @brief  Convenience: send a PUSH-type payload (fire-and-forget).
 */
static inline int fw_link_send_push(fw_link_channel_t channel,
                                    const uint8_t *payload,
                                    uint16_t payload_len)
{
    return fw_link_send(channel, FW_LINK_FRAME_MSG_PUSH, 0u, 0u, payload, payload_len);
}

/**
 * @brief  Convenience: send a REQ-type payload.
 */
static inline int fw_link_send_req(fw_link_channel_t channel,
                                   uint16_t txid,
                                   const uint8_t *payload,
                                   uint16_t payload_len)
{
    return fw_link_send(channel, FW_LINK_FRAME_MSG_REQ, txid, 0u, payload, payload_len);
}

/**
 * @brief  Convenience: send a RESP-type payload.
 */
static inline int fw_link_send_resp(fw_link_channel_t channel,
                                    uint16_t txid,
                                    const uint8_t *payload,
                                    uint16_t payload_len)
{
    return fw_link_send(channel, FW_LINK_FRAME_MSG_RESP, txid, 0u, payload, payload_len);
}

#ifdef __cplusplus
}
#endif

#endif /* FW_LINK_H */
