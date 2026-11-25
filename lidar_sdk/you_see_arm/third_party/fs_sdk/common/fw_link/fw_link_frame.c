#include "fw_link_frame.h"
#include <string.h>

/* CRC-16/BUYPASS: poly=0x8005, init=seed, refin=false, refout=false, xorout=0x0000.
 * Identical to existing fs_util_crc_16() in mcu/app/sdk/FL6X32.c.
 * Inlined here to keep fw_link_frame self-contained. */
static uint16_t fw_link_frame_crc16_impl(const uint8_t *data, uint16_t len, uint16_t seed)
{
#define FW_LINK_CRC_POLY 0x8005u
    uint16_t crc = seed;
    uint16_t i, j;

    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (j = 0u; j < 8u; j++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ FW_LINK_CRC_POLY);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
#undef FW_LINK_CRC_POLY
}

uint16_t fw_link_frame_crc16(const uint8_t *data, uint16_t len)
{
    return fw_link_frame_crc16_impl(data, len, 0u);
}

/* ---- pack ---- */
enum {
    PK_IDX_MAGIC0 = 0,
    PK_IDX_MAGIC1,
    PK_IDX_VER,
    PK_IDX_TYPE,
    PK_IDX_TXID_H,
    PK_IDX_TXID_L,
    PK_IDX_LEN_H,
    PK_IDX_LEN_L,
    PK_IDX_FLAGS,
};

int fw_link_frame_pack(uint8_t  msg_type,
                       uint16_t txid,
                       uint8_t  flags,
                       const uint8_t *payload,
                       uint16_t payload_len,
                       uint8_t *out_buf,
                       uint16_t out_buf_size,
                       uint16_t *out_len)
{
    uint16_t total_len;
    uint16_t crc;

    if (out_buf == NULL || out_len == NULL) {
        return -1;
    }
    if (payload_len > FW_LINK_FRAME_MAX_PAYLOAD) {
        return -1;
    }
    if ((payload_len != 0u) && (payload == NULL)) {
        return -1;
    }

    total_len = FW_LINK_FRAME_HEAD_LEN + payload_len + FW_LINK_FRAME_CRC_LEN;
    if (out_buf_size < total_len) {
        return -1;
    }

    out_buf[PK_IDX_MAGIC0] = FW_LINK_FRAME_MAGIC0;
    out_buf[PK_IDX_MAGIC1] = FW_LINK_FRAME_MAGIC1;
    out_buf[PK_IDX_VER]    = FW_LINK_FRAME_VERSION;
    out_buf[PK_IDX_TYPE]   = msg_type;
    out_buf[PK_IDX_TXID_H] = (uint8_t)((txid >> 8) & 0xFFu);
    out_buf[PK_IDX_TXID_L] = (uint8_t)(txid & 0xFFu);
    out_buf[PK_IDX_LEN_H]  = (uint8_t)((payload_len >> 8) & 0xFFu);
    out_buf[PK_IDX_LEN_L]  = (uint8_t)(payload_len & 0xFFu);
    out_buf[PK_IDX_FLAGS]  = flags;

    if (payload_len > 0u) {
        (void)memcpy(&out_buf[FW_LINK_FRAME_HEAD_LEN], payload, payload_len);
    }

    crc = fw_link_frame_crc16(out_buf, FW_LINK_FRAME_HEAD_LEN + payload_len);
    out_buf[FW_LINK_FRAME_HEAD_LEN + payload_len]     = (uint8_t)((crc >> 8) & 0xFFu);
    out_buf[FW_LINK_FRAME_HEAD_LEN + payload_len + 1u] = (uint8_t)(crc & 0xFFu);

    *out_len = total_len;
    return 0;
}

/* ---- parser ---- */

void fw_link_frame_parser_init(fw_link_frame_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    (void)memset(parser, 0, sizeof(*parser));
    parser->state = FW_LINK_FRAME_PARSE_SYNC0;
}

static void fw_link_frame_parser_reset(fw_link_frame_parser_t *parser)
{
    parser->state       = FW_LINK_FRAME_PARSE_SYNC0;
    parser->header_idx  = 0u;
    parser->payload_idx = 0u;
    parser->payload_len = 0u;
    parser->crc_rx      = 0u;
}

int fw_link_frame_parser_input(fw_link_frame_parser_t *parser,
                               uint8_t byte,
                               uint8_t *payload_buf,
                               uint16_t payload_buf_size,
                               fw_link_frame_meta_t *meta,
                               uint16_t *payload_len)
{
    uint16_t crc_calc;

    if (parser == NULL || meta == NULL || payload_len == NULL) {
        return -1;
    }

    switch (parser->state) {
    case FW_LINK_FRAME_PARSE_SYNC0:
        if (byte == FW_LINK_FRAME_MAGIC0) {
            parser->state = FW_LINK_FRAME_PARSE_SYNC1;
        }
        break;

    case FW_LINK_FRAME_PARSE_SYNC1:
        if (byte == FW_LINK_FRAME_MAGIC1) {
            parser->state      = FW_LINK_FRAME_PARSE_HEADER;
            parser->header_idx = 0u;
            parser->header[parser->header_idx++] = FW_LINK_FRAME_MAGIC0;
            parser->header[parser->header_idx++] = FW_LINK_FRAME_MAGIC1;
        } else {
            fw_link_frame_parser_reset(parser);
            if (byte == FW_LINK_FRAME_MAGIC0) {
                parser->state = FW_LINK_FRAME_PARSE_SYNC1;
            }
        }
        break;

    case FW_LINK_FRAME_PARSE_HEADER:
        parser->header[parser->header_idx++] = byte;
        if (parser->header_idx >= FW_LINK_FRAME_HEAD_LEN) {
            parser->payload_len = (uint16_t)(((uint16_t)parser->header[PK_IDX_LEN_H] << 8)
                                   | (uint16_t)parser->header[PK_IDX_LEN_L]);
            if (parser->payload_len > FW_LINK_FRAME_MAX_PAYLOAD) {
                fw_link_frame_parser_reset(parser);
                return -1;
            }
            if (parser->payload_len == 0u) {
                parser->state = FW_LINK_FRAME_PARSE_CRC0;
            } else {
                if (payload_buf == NULL || payload_buf_size < parser->payload_len) {
                    fw_link_frame_parser_reset(parser);
                    return -1;
                }
                parser->state       = FW_LINK_FRAME_PARSE_PAYLOAD;
                parser->payload_idx = 0u;
            }
        }
        break;

    case FW_LINK_FRAME_PARSE_PAYLOAD:
        payload_buf[parser->payload_idx++] = byte;
        if (parser->payload_idx >= parser->payload_len) {
            parser->state = FW_LINK_FRAME_PARSE_CRC0;
        }
        break;

    case FW_LINK_FRAME_PARSE_CRC0:
        parser->crc_rx = (uint16_t)((uint16_t)byte << 8);
        parser->state  = FW_LINK_FRAME_PARSE_CRC1;
        break;

    case FW_LINK_FRAME_PARSE_CRC1:
        parser->crc_rx |= (uint16_t)byte;

        crc_calc = fw_link_frame_crc16(parser->header, FW_LINK_FRAME_HEAD_LEN);
        if (parser->payload_len > 0u) {
            crc_calc = fw_link_frame_crc16_impl(payload_buf, parser->payload_len, crc_calc);
        }
        if (crc_calc == parser->crc_rx) {
            meta->version     = parser->header[PK_IDX_VER];
            meta->msg_type    = parser->header[PK_IDX_TYPE];
            meta->txid        = (uint16_t)(((uint16_t)parser->header[PK_IDX_TXID_H] << 8)
                                | (uint16_t)parser->header[PK_IDX_TXID_L]);
            meta->payload_len = parser->payload_len;
            meta->flags       = parser->header[PK_IDX_FLAGS];
            *payload_len      = parser->payload_len;
            fw_link_frame_parser_reset(parser);
            return 1;
        }
        fw_link_frame_parser_reset(parser);
        return -1;

    default:
        fw_link_frame_parser_reset(parser);
        return -1;
    }

    return 0;
}
