#include "spad_time_anchor.h"

#include <limits.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>

#include "utils/fs_err.h"
#include "utils/fs_log.h"

#undef LOG_TAG
#define LOG_TAG "spad.anchor"

#define SPAD_LIDAR_FRAME_ID_MAX 65535u

typedef struct {
    uint32_t fsync_seq;
    uint64_t t_frame_sync_ns;
    uint8_t valid;
    uint8_t consumed;
} spad_pending_fsync_entry_t;

typedef struct {
    uint32_t frame_id;
    uint32_t fsync_seq;
    uint64_t anchor_ns;
    uint8_t valid;
    uint8_t source;
} spad_anchor_entry_t;

static pthread_mutex_t g_anchor_lock = PTHREAD_MUTEX_INITIALIZER;
static spad_anchor_entry_t g_anchor_ring[SPAD_ANCHOR_DEFAULT_CAPACITY];
static spad_pending_fsync_entry_t g_pending_fsync_ring[SPAD_ANCHOR_DEFAULT_CAPACITY];
static uint32_t g_anchor_head;
static uint32_t g_anchor_count;
static uint32_t g_pending_fsync_head;
static uint32_t g_pending_fsync_tail;
static uint32_t g_pending_fsync_count;
static int64_t g_trigger_delay_ns;
static spad_anchor_diag_t g_anchor_diag;
static uint32_t g_latest_bound_frame_id;
static uint32_t g_latest_bound_fsync_seq;
static uint32_t g_anchor_generation;

static uint32_t anchor_find_locked(uint32_t frame_id)
{
    uint32_t i;

    for (i = 0u; i < SPAD_ANCHOR_DEFAULT_CAPACITY; i++) {
        if (g_anchor_ring[i].valid && g_anchor_ring[i].frame_id == frame_id) {
            return i;
        }
    }
    return SPAD_ANCHOR_DEFAULT_CAPACITY;
}

static uint32_t anchor_frame_id_from_sync_seq(uint32_t sync_seq)
{
    return ((sync_seq - 1u) % SPAD_LIDAR_FRAME_ID_MAX) + 1u;
}

static uint32_t frame_next_id(uint32_t frame_id)
{
    if (frame_id == 0u || frame_id >= SPAD_LIDAR_FRAME_ID_MAX) {
        return 1u;
    }
    return frame_id + 1u;
}

static uint32_t fsync_next_seq(uint32_t fsync_seq)
{
    if (fsync_seq == 0u) {
        return 1u;
    }
    return fsync_seq + 1u;
}

static int spad_anchor_insert_frame_with_source(uint32_t frame_id,
                                                uint64_t anchor_ns,
                                                uint32_t fsync_seq,
                                                spad_anchor_source_t source)
{
    spad_anchor_entry_t *entry;
    uint32_t slot;

    if (frame_id == 0u || fsync_seq == 0u) {
        return FS_ERR_BAD_PARAMS;
    }

    pthread_mutex_lock(&g_anchor_lock);

    slot = anchor_find_locked(frame_id);
    if (slot == SPAD_ANCHOR_DEFAULT_CAPACITY) {
        slot = g_anchor_head;
        if (g_anchor_ring[slot].valid && g_anchor_count >= SPAD_ANCHOR_DEFAULT_CAPACITY) {
            g_anchor_diag.overwrite_count++;
        } else if (g_anchor_count < SPAD_ANCHOR_DEFAULT_CAPACITY) {
            g_anchor_count++;
        }
        g_anchor_head = (g_anchor_head + 1u) % SPAD_ANCHOR_DEFAULT_CAPACITY;
    }

    entry = &g_anchor_ring[slot];
    entry->frame_id = frame_id;
    entry->fsync_seq = fsync_seq;
    entry->anchor_ns = anchor_ns;
    entry->valid = 1u;
    entry->source = (uint8_t)source;

    g_anchor_diag.insert_count++;
    g_anchor_diag.frame_anchor_insert_count++;
    g_anchor_diag.latest_sync_seq = fsync_seq;
    g_anchor_diag.latest_frame_id = frame_id;
    g_anchor_diag.latest_anchor_ns = anchor_ns;

    pthread_mutex_unlock(&g_anchor_lock);
    return FS_SUCCESS;
}

void spad_anchor_init(void)
{
    pthread_mutex_lock(&g_anchor_lock);
    memset(g_anchor_ring, 0, sizeof(g_anchor_ring));
    memset(g_pending_fsync_ring, 0, sizeof(g_pending_fsync_ring));
    memset(&g_anchor_diag, 0, sizeof(g_anchor_diag));
    g_anchor_head = 0u;
    g_anchor_count = 0u;
    g_pending_fsync_head = 0u;
    g_pending_fsync_tail = 0u;
    g_pending_fsync_count = 0u;
    g_trigger_delay_ns = 0;
    g_latest_bound_frame_id = 0u;
    g_latest_bound_fsync_seq = 0u;
    g_anchor_generation++;
    if (g_anchor_generation == 0u) {
        g_anchor_generation = 1u;
    }
    g_anchor_diag.trigger_delay_ns = g_trigger_delay_ns;
    pthread_mutex_unlock(&g_anchor_lock);

    FS_LOGI("SPAD anchor table initialized, capacity=%u, trigger_delay_ns=%lld",
            SPAD_ANCHOR_DEFAULT_CAPACITY, (long long)g_trigger_delay_ns);
}

uint32_t spad_anchor_get_generation(void)
{
    uint32_t generation;

    pthread_mutex_lock(&g_anchor_lock);
    generation = g_anchor_generation;
    pthread_mutex_unlock(&g_anchor_lock);

    return generation;
}

void spad_anchor_set_trigger_delay_ns(int64_t delay_ns)
{
    pthread_mutex_lock(&g_anchor_lock);
    g_trigger_delay_ns = delay_ns;
    g_anchor_diag.trigger_delay_ns = delay_ns;
    pthread_mutex_unlock(&g_anchor_lock);

    FS_LOGI("SPAD trigger delay configured: %lld ns", (long long)delay_ns);
}

int spad_anchor_insert_pending_fsync(uint32_t fsync_seq, uint64_t t_frame_sync_ns)
{
    spad_pending_fsync_entry_t *entry;

    if (fsync_seq == 0u) {
        return FS_ERR_BAD_PARAMS;
    }

    pthread_mutex_lock(&g_anchor_lock);
    if (g_pending_fsync_count >= SPAD_ANCHOR_DEFAULT_CAPACITY) {
        g_pending_fsync_tail = (g_pending_fsync_tail + 1u) % SPAD_ANCHOR_DEFAULT_CAPACITY;
        g_pending_fsync_count--;
        g_anchor_diag.overwrite_count++;
    }

    entry = &g_pending_fsync_ring[g_pending_fsync_head];
    entry->fsync_seq = fsync_seq;
    entry->t_frame_sync_ns = t_frame_sync_ns;
    entry->valid = 1u;
    entry->consumed = 0u;
    g_pending_fsync_head = (g_pending_fsync_head + 1u) % SPAD_ANCHOR_DEFAULT_CAPACITY;
    g_pending_fsync_count++;
    g_anchor_diag.pending_fsync_insert_count++;
    g_anchor_diag.latest_fsync_seq = fsync_seq;
    pthread_mutex_unlock(&g_anchor_lock);
    return FS_SUCCESS;
}

int spad_anchor_insert_frame(uint32_t frame_id, uint64_t t_frame_sync_ns, uint32_t fsync_seq)
{
    return spad_anchor_insert_frame_with_source(frame_id,
                                                t_frame_sync_ns,
                                                fsync_seq,
                                                SPAD_ANCHOR_SRC_FRAME_SYNC);
}

int spad_anchor_bind_next_frame(uint32_t frame_id)
{
    spad_pending_fsync_entry_t pending;
    uint32_t expected_frame_id;
    uint32_t expected_fsync_seq;

    if (frame_id == 0u) {
        return FS_ERR_BAD_PARAMS;
    }

    pthread_mutex_lock(&g_anchor_lock);
    if (g_pending_fsync_count == 0u) {
        g_anchor_diag.pending_fsync_underflow_count++;
        pthread_mutex_unlock(&g_anchor_lock);
        return FS_ERR_NOT_FOUND;
    }

    pending = g_pending_fsync_ring[g_pending_fsync_tail];

    if (g_latest_bound_frame_id != 0u) {
        expected_frame_id = frame_next_id(g_latest_bound_frame_id);
        if (frame_id != expected_frame_id) {
            g_anchor_diag.frame_id_gap_count++;
            FS_LOGW("SPAD frame_id gap: expected=%u actual=%u fsync_seq=%u",
                    expected_frame_id, frame_id, pending.fsync_seq);
        }
    }

    if (g_latest_bound_fsync_seq != 0u) {
        expected_fsync_seq = fsync_next_seq(g_latest_bound_fsync_seq);
        if (pending.fsync_seq != expected_fsync_seq) {
            g_anchor_diag.fsync_seq_gap_count++;
            FS_LOGW("SPAD fsync_seq gap: expected=%u actual=%u frame_id=%u",
                    expected_fsync_seq, pending.fsync_seq, frame_id);
        }
    }

    g_pending_fsync_ring[g_pending_fsync_tail].consumed = 1u;
    g_pending_fsync_tail = (g_pending_fsync_tail + 1u) % SPAD_ANCHOR_DEFAULT_CAPACITY;
    g_pending_fsync_count--;

    g_latest_bound_frame_id = frame_id;
    g_latest_bound_fsync_seq = pending.fsync_seq;
    g_anchor_diag.bind_count++;
    g_anchor_diag.latest_bound_frame_id = frame_id;
    g_anchor_diag.latest_bound_fsync_seq = pending.fsync_seq;
    pthread_mutex_unlock(&g_anchor_lock);

    return spad_anchor_insert_frame(frame_id, pending.t_frame_sync_ns, pending.fsync_seq);
}

int spad_anchor_insert(uint32_t sync_seq, uint64_t t_soc_sync_event_ns)
{
    uint32_t frame_id;
    int64_t trigger_delay_ns;
    int64_t adjusted_ns;
    uint64_t adjusted_anchor_ns;

    if (sync_seq == 0u) {
        return FS_ERR_BAD_PARAMS;
    }

    frame_id = anchor_frame_id_from_sync_seq(sync_seq);

    pthread_mutex_lock(&g_anchor_lock);
    trigger_delay_ns = g_trigger_delay_ns;
    pthread_mutex_unlock(&g_anchor_lock);

    if (t_soc_sync_event_ns > (uint64_t)INT64_MAX) {
        adjusted_ns = INT64_MAX;
    } else {
        adjusted_ns = (int64_t)t_soc_sync_event_ns;
    }

    adjusted_ns += trigger_delay_ns;
    if (adjusted_ns < 0) {
        adjusted_ns = 0;
    }

    adjusted_anchor_ns = (uint64_t)adjusted_ns;

    return spad_anchor_insert_frame_with_source(frame_id,
                                                adjusted_anchor_ns,
                                                sync_seq,
                                                SPAD_ANCHOR_SRC_TRIGGER_FALLBACK);
}

int spad_anchor_match_frame(uint32_t frame_id, uint64_t *anchor_ns)
{
    uint32_t slot;

    if (anchor_ns == NULL) {
        return FS_ERR_BAD_PARAMS;
    }
    if (frame_id == 0u) {
        pthread_mutex_lock(&g_anchor_lock);
        g_anchor_diag.invalid_frame_id_count++;
        pthread_mutex_unlock(&g_anchor_lock);
        *anchor_ns = 0u;
        return FS_ERR_BAD_PARAMS;
    }

    pthread_mutex_lock(&g_anchor_lock);

    slot = anchor_find_locked(frame_id);

    g_anchor_diag.latest_frame_id = frame_id;
    if (slot == SPAD_ANCHOR_DEFAULT_CAPACITY) {
        g_anchor_diag.miss_count++;
        if (g_anchor_diag.latest_sync_seq != 0u &&
            frame_id != anchor_frame_id_from_sync_seq(g_anchor_diag.latest_sync_seq)) {
            g_anchor_diag.mismatch_count++;
        }
        pthread_mutex_unlock(&g_anchor_lock);
        *anchor_ns = 0u;
        return FS_ERR_NOT_FOUND;
    }

    *anchor_ns = g_anchor_ring[slot].anchor_ns;
    g_anchor_diag.match_count++;
    pthread_mutex_unlock(&g_anchor_lock);
    return FS_SUCCESS;
}

void spad_anchor_get_diag(spad_anchor_diag_t *diag)
{
    if (diag == NULL) {
        return;
    }

    pthread_mutex_lock(&g_anchor_lock);
    *diag = g_anchor_diag;
    pthread_mutex_unlock(&g_anchor_lock);
}
