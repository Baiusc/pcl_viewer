#ifndef YS_PRODUCER_RANGE_MF150_FL6532_H
#define YS_PRODUCER_RANGE_MF150_FL6532_H

#include "../../ys_def.h"

/* 纯测量模式: 只消费统计不组包/不发送, 并禁用 consumer,
 * 用于排除下游影响测量 Range 真实帧率与整帧大小。置 0 恢复发送。 */
#define RANGE_MEASURE_ONLY 0

/**
 * @description  : 退出 Range 生产者线程
 * @return        {}
 */
void producer_exit_range(void);

/**
 * @description  : 启动 Range 生产者线程, 消费 fs_sdk Range(0x2E) 数据并分片写入内存池
 * @param         {udp_t} *udp_instance
 * @param         {int} driver_slot_num
 * @return        {}
 */
void producer_get_range(udp_t *udp_instance, int driver_slot_num);

#endif // YS_PRODUCER_RANGE_MF150_FL6532_H
