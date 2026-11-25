#ifndef __SPAD_TIME_ANCHOR_H__
#define __SPAD_TIME_ANCHOR_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPAD_ANCHOR_DEFAULT_CAPACITY 128u

typedef enum {
    SPAD_ANCHOR_SRC_FRAME_SYNC = 0,
    SPAD_ANCHOR_SRC_TRIGGER_FALLBACK = 1,
    SPAD_ANCHOR_SRC_HYBRID_DIAG = 2,
} spad_anchor_source_t;

typedef struct {
    uint32_t insert_count;
    uint32_t match_count;
    uint32_t invalid_frame_id_count;
    uint32_t miss_count;
    uint32_t overwrite_count;
    uint32_t mismatch_count;
    uint32_t latest_sync_seq;
    uint32_t latest_frame_id;
    uint64_t latest_anchor_ns;
    int64_t  trigger_delay_ns;
    uint32_t frame_anchor_insert_count;
    uint32_t pending_fsync_insert_count;
    uint32_t pending_fsync_underflow_count;
    uint32_t latest_fsync_seq;
    uint32_t frame_id_gap_count;
    uint32_t fsync_seq_gap_count;
    uint32_t bind_count;
    uint32_t latest_bound_frame_id;
    uint32_t latest_bound_fsync_seq;
} spad_anchor_diag_t;

void spad_anchor_init(void);
uint32_t spad_anchor_get_generation(void);
void spad_anchor_set_trigger_delay_ns(int64_t delay_ns);
/* Legacy trigger fallback: applies trigger_delay_ns before inserting anchor. */
int spad_anchor_insert(uint32_t sync_seq, uint64_t t_soc_sync_event_ns);
int spad_anchor_insert_pending_fsync(uint32_t fsync_seq, uint64_t t_frame_sync_ns);
int spad_anchor_bind_next_frame(uint32_t frame_id);
int spad_anchor_insert_frame(uint32_t frame_id,
                             uint64_t t_frame_sync_ns,
                             uint32_t fsync_seq);
int spad_anchor_match_frame(uint32_t frame_id, uint64_t *anchor_ns);
void spad_anchor_get_diag(spad_anchor_diag_t *diag);

#ifdef __cplusplus
}
#endif

#endif /* __SPAD_TIME_ANCHOR_H__ */
