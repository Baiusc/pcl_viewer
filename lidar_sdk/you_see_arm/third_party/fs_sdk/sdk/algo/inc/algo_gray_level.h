#ifndef __FS_ALGO_GRAY_LEVEL_H__
#define __FS_ALGO_GRAY_LEVEL_H__

#include <stdint.h>
#include <stdbool.h>

#pragma pack(push)
#pragma pack(1)

/*
 * Gray-Level Pixel Information after unpacketizing
 */
typedef struct {
    uint16_t fid;               // Frame ID
    uint16_t row;               // 物理行坐标
    uint16_t col;               // 物理列坐标
    uint32_t laser_all_cnt;     // 灰度图pixel数据
} gl_pix_info_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

int32_t algo_gray_calc_cfg(algo_context_t* ctx, int32_t ab_idx);
int32_t algo_gray_data_process(algo_context_t* ctx, int32_t vi, uint8_t *data, uint32_t data_len);

#ifdef __cplusplus
}
#endif

#endif // __FS_ALGO_GRAY_LEVEL_H__
