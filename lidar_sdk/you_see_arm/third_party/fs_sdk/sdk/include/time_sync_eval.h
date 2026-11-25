#ifndef __TIME_SYNC_EVAL_H__
#define __TIME_SYNC_EVAL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t time_sync_eval_realtime_us(void);
int time_sync_eval_udp_enabled(void);
void time_sync_eval_pack_le56(uint8_t dst[7], uint64_t value);
void time_sync_eval_record_imu_batch(uint16_t sample_count);
void time_sync_eval_record_imu_timestamp(uint64_t timestamp_ns, uint8_t timestamp_quality);
void time_sync_eval_record_pointcloud_packet(uint32_t frame_id,
                                             uint64_t frame_timestamp_us,
                                             uint64_t send_realtime_us,
                                             int frame_start);
void time_sync_eval_record_spad_anchor_miss(void);

#ifdef __cplusplus
}
#endif

#endif
