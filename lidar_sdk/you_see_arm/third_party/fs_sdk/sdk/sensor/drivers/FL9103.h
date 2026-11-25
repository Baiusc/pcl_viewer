#ifndef __FL9103_H__
#define __FL9103_H__

#include <stdint.h>
#include <stdbool.h>
#include "fs_sensor_hub.h"

#define FL9103_PROTO_HEAD_READ               0x81
#define FL9103_PROTO_HEAD_WRITE              0x61
#define FL9103_PROTO_HEAD_MUL_READ           0x91
#define FL9103_PROTO_HEAD_MUL_WRITE          0x71
#define FL9103_PROTO_HEAD_MUL_WRITE_WITH_CRC 0x5A

#pragma pack(push)
#pragma pack(1)

typedef struct {
    uint16_t hdr;
    uint32_t addr;
    uint16_t wcrc_o;
} FL9103_rreg_t;

typedef struct {
    uint16_t wcrc_i;
    uint32_t data;
    uint16_t rcrc;
} FL9103_rreg_tail_t;

typedef struct {
    uint16_t hdr;
    uint32_t addr;
    uint32_t data;
    uint16_t wcrc_o;
} FL9103_wreg_t;

typedef struct {
    uint16_t wcrc_i;
} FL9103_wreg_tail_t;

typedef struct {
    uint16_t hdr;
    uint32_t addr;
    uint16_t dlen;
    uint16_t wcrc_o;
    uint16_t wcrc_i;
    uint32_t data[0];
} FL9103_rbuf_t;

typedef struct {
    uint16_t rcrc;
} FL9103_rbuf_tail_t;

typedef struct {
    uint16_t hdr;
    uint32_t addr;
    uint16_t dlen;
    uint32_t data[0];
} FL9103_wbuf_t;

typedef struct {
    uint16_t wcrc_o;
    uint16_t wcrc_i;
} FL9103_wbuf_tail_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

int32_t FL9103_deinit(fs_tx_t *tx); // 反初始化设备
int32_t FL9103_init(fs_tx_t *tx, const int16_t *dev_id, uint8_t dev_cnt, uint8_t ch_cnt, const uint8_t ch_map[dev_cnt][ch_cnt]); // 初始化设备

#ifdef __cplusplus
}
#endif

#endif /* __FL9103_H__ */
