#ifndef __MCU_UART_SESSION_H__
#define __MCU_UART_SESSION_H__

#include <stdint.h>
#include "fw_link_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Compatibility aliases (map old names to fw_link_frame) ---- */
#define MCU_UART_SESSION_MAGIC0        FW_LINK_FRAME_MAGIC0
#define MCU_UART_SESSION_MAGIC1        FW_LINK_FRAME_MAGIC1
#define MCU_UART_SESSION_VERSION       FW_LINK_FRAME_VERSION
#define MCU_UART_SESSION_HEAD_LEN      FW_LINK_FRAME_HEAD_LEN
#define MCU_UART_SESSION_CRC_LEN       FW_LINK_FRAME_CRC_LEN
#define MCU_UART_SESSION_MAX_PAYLOAD   FW_LINK_FRAME_MAX_PAYLOAD

/* ---- Type aliases (keep old names for caller compatibility) ---- */
typedef fw_link_frame_msg_type_t    mcu_uart_msg_type_t;
typedef fw_link_frame_parse_state_t mcu_uart_parse_state_t;
typedef fw_link_frame_meta_t        mcu_uart_frame_meta_t;
typedef fw_link_frame_parser_t      mcu_uart_parser_t;

#define MCU_UART_MSG_REQ   FW_LINK_FRAME_MSG_REQ
#define MCU_UART_MSG_RESP  FW_LINK_FRAME_MSG_RESP
#define MCU_UART_MSG_PUSH  FW_LINK_FRAME_MSG_PUSH

#define MCU_UART_PARSE_SYNC0   FW_LINK_FRAME_PARSE_SYNC0
#define MCU_UART_PARSE_SYNC1   FW_LINK_FRAME_PARSE_SYNC1
#define MCU_UART_PARSE_HEADER  FW_LINK_FRAME_PARSE_HEADER
#define MCU_UART_PARSE_PAYLOAD FW_LINK_FRAME_PARSE_PAYLOAD
#define MCU_UART_PARSE_CRC0    FW_LINK_FRAME_PARSE_CRC0
#define MCU_UART_PARSE_CRC1    FW_LINK_FRAME_PARSE_CRC1

/* ---- Inline wrappers (delegate to fw_link_frame) ---- */

static inline uint16_t mcu_uart_frame_crc16(const uint8_t *data, uint16_t len)
{
    return fw_link_frame_crc16(data, len);
}

static inline int mcu_uart_frame_pack(uint8_t msg_type,
                                      uint16_t txid,
                                      uint8_t flags,
                                      const uint8_t *payload,
                                      uint16_t payload_len,
                                      uint8_t *out_buf,
                                      uint16_t out_buf_size,
                                      uint16_t *out_len)
{
    return fw_link_frame_pack(msg_type, txid, flags, payload, payload_len,
                              out_buf, out_buf_size, out_len);
}

static inline void mcu_uart_parser_init(mcu_uart_parser_t *parser)
{
    fw_link_frame_parser_init(parser);
}

static inline int mcu_uart_parser_input(mcu_uart_parser_t *parser,
                                        uint8_t byte,
                                        uint8_t *payload_buf,
                                        uint16_t payload_buf_size,
                                        mcu_uart_frame_meta_t *meta,
                                        uint16_t *payload_len)
{
    return fw_link_frame_parser_input(parser, byte, payload_buf, payload_buf_size,
                                      meta, payload_len);
}

#ifdef __cplusplus
}
#endif

#endif /* __MCU_UART_SESSION_H__ */
