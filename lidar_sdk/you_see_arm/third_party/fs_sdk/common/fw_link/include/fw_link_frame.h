#ifndef FW_LINK_FRAME_H
#define FW_LINK_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Frame layout constants (shared SoC / MCU / Boot) ---- */
#define FW_LINK_FRAME_MAGIC0           (0x46u)  /* 'F' */
#define FW_LINK_FRAME_MAGIC1           (0x53u)  /* 'S' */
#define FW_LINK_FRAME_VERSION          (0x01u)

#define FW_LINK_FRAME_HEAD_LEN         (9u)
#define FW_LINK_FRAME_CRC_LEN          (2u)
#define FW_LINK_FRAME_MAX_PAYLOAD      (2080u)
#define FW_LINK_FRAME_MAX_TOTAL        (FW_LINK_FRAME_HEAD_LEN + FW_LINK_FRAME_MAX_PAYLOAD + FW_LINK_FRAME_CRC_LEN)

/* ---- Message type ---- */
typedef enum {
    FW_LINK_FRAME_MSG_REQ  = 0x01,
    FW_LINK_FRAME_MSG_RESP = 0x02,
    FW_LINK_FRAME_MSG_PUSH = 0x03,
} fw_link_frame_msg_type_t;

/* ---- Parsed frame metadata ---- */
typedef struct {
    uint8_t  version;
    uint8_t  msg_type;
    uint16_t txid;
    uint16_t payload_len;
    uint8_t  flags;
} fw_link_frame_meta_t;

/* ---- Parser state machine (platform-neutral) ---- */
typedef enum {
    FW_LINK_FRAME_PARSE_SYNC0 = 0,
    FW_LINK_FRAME_PARSE_SYNC1,
    FW_LINK_FRAME_PARSE_HEADER,
    FW_LINK_FRAME_PARSE_PAYLOAD,
    FW_LINK_FRAME_PARSE_CRC0,
    FW_LINK_FRAME_PARSE_CRC1,
} fw_link_frame_parse_state_t;

typedef struct {
    fw_link_frame_parse_state_t state;
    uint8_t  header[FW_LINK_FRAME_HEAD_LEN];
    uint16_t header_idx;
    uint16_t payload_idx;
    uint16_t payload_len;
    uint16_t crc_rx;
} fw_link_frame_parser_t;

/* ---- API ---- */

/**
 * @brief  Compute CRC-16 over data (seed=0 starts fresh; pass prior crc to chain).
 */
uint16_t fw_link_frame_crc16(const uint8_t *data, uint16_t len);

/**
 * @brief  Pack a complete frame into out_buf.
 * @return 0 on success, -1 on error (NULL args, payload too large, buffer too small).
 */
int fw_link_frame_pack(uint8_t  msg_type,
                       uint16_t txid,
                       uint8_t  flags,
                       const uint8_t *payload,
                       uint16_t payload_len,
                       uint8_t *out_buf,
                       uint16_t out_buf_size,
                       uint16_t *out_len);

/**
 * @brief  Initialize parser state machine.
 */
void fw_link_frame_parser_init(fw_link_frame_parser_t *parser);

/**
 * @brief  Feed one byte into parser.
 * @return  1 = full frame decoded (meta + payload_len valid)
 *          0 = need more bytes
 *         -1 = frame error (parser auto-resets)
 */
int fw_link_frame_parser_input(fw_link_frame_parser_t *parser,
                               uint8_t byte,
                               uint8_t *payload_buf,
                               uint16_t payload_buf_size,
                               fw_link_frame_meta_t *meta,
                               uint16_t *payload_len);

#ifdef __cplusplus
}
#endif

#endif /* FW_LINK_FRAME_H */
