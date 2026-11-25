#ifndef __FL9003_H__
#define __FL9003_H__

#include <stdint.h>
#include <stdbool.h>
#include "fs_sensor_hub.h"

#define FL9003_MAX_CHANNELS_IN_SEQ      ( 8)    // 每个序列最大通道数
#define FL9003_MAX_CHANNELS_PER_CHIP    (18)    // 单芯片最大通道数
#define FL9003_MAX_SEQUENCE_PER_CHIP    (22)    // 最大序列长度

#define FL9003_REG_CHECK_EN 0

#define FL9003_OPSTART      (0x00)
#define FL9003_OPSIZE       (0x01)
#define FL9003_CTRL         (0x02)
#define FL9003_STATUS       (0x03)
#define FL9003_INTMASK      (0x04)
#define FL9003_INTFLAG      (0x05)

#define FL9003_S00A         (0x10)

enum {
    VCSEL_MODE_ONE_CHANNEL = 0, //单通道，保留，减少传参
    VCSEL_MODE_MULT_CHANNEL,    //多通道
    VCSEL_MODE_SEQUENCE,        //顺序点亮
    VCSEL_MODE_REVERT_SEQUEUECE,//逆序点亮
    VCSEL_MODE_READ_EFUSE,      //查AB模式
    VCSEL_MODE_CUSTOM,          //自定义序列

    VCSEL_MODE_MAX_NUM
};

/********************************* 用于自定义序列 ******************************* */
//定义op_start与op_size
typedef struct {
    uint8_t tx_dev;
    uint8_t op_start;
    uint8_t op_size;
} fs_tx_op_t;

//定义序列内容
typedef struct {
    uint8_t tx_dev;
    uint8_t seq_idx;    //序列号,用来索引寄存器
    uint32_t seq_data;
} fs_tx_seq_t;

typedef struct {
    fs_tx_op_t *op;
    uint32_t op_size;
    fs_tx_seq_t *seq;
    uint32_t seq_size;
} fs_tx_seq_cfg_t;

/**************************************************************************** */

#ifdef __cplusplus
extern "C" {
#endif

int32_t FL9003_deinit(fs_tx_t *tx); // 反初始化设备
int32_t FL9003_init(fs_tx_t *tx, const int16_t *dev_id, uint8_t dev_cnt, uint8_t ch_cnt, const uint8_t ch_map[dev_cnt][ch_cnt]); // 初始化设备

#ifdef __cplusplus
}
#endif

#endif /* __FL9003_H__ */

