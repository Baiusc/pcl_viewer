#include "time_sync_eval.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "fs_log.h"
#include "mcu_protocol.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "time_sync_eval"

#define TIME_SYNC_EVAL_EN_FILE        "/data/fslidar/config/time_sync_eval_en"
#define TIME_SYNC_EVAL_UDP_EN_FILE    "/data/fslidar/config/time_sync_eval_udp_en"
#define TIME_SYNC_EVAL_PERIOD_MS_FILE "/data/fslidar/config/time_sync_eval_period_ms"
#define TIME_SYNC_EVAL_CSV_PATH       "/tmp/fs_time_sync_eval_summary.csv"
#define TIME_SYNC_EVAL_CSV_PATH_OLD   "/tmp/fs_time_sync_eval_summary.csv.1"
#define TIME_SYNC_EVAL_CSV_MAX_BYTES  (16u * 1024u * 1024u)
#define TIME_SYNC_EVAL_CONFIG_TTL_MS   (500u)
#define TIME_SYNC_EVAL_SAMPLE_DEFAULT_MS (5000u)
#define TIME_SYNC_EVAL_SAMPLE_MIN_MS     (1000u)
#define TIME_SYNC_EVAL_SAMPLE_MAX_MS     (60000u)
#define TIME_SYNC_EVAL_SCHEMA_VERSION    (6u)
/* CSV label migration: trigger_irq_d became sync_event_irq_d after sync-event profile work. */
#define TIME_SYNC_EVAL_CSV_HEADER \
    "schema_version,soc_realtime_us,soc_monotonic_ms,sample_period_ms,time_diag_ok,imu_diag_ok," \
    "map_locked,map_anchor_age_ms,map_diag_offset_ms,map_last_residual_us,map_last_residual_signed_us,map_drift_ppb,map_lock_d,map_lost_d,resync_d," \
    "sync_event_irq_d,fsync_pulse_d,sync_event_queue_overflow_d,fsync_overlap_d,soc_sync_rx_d,soc_drop_invalid_d,soc_drop_queue_full_d," \
    "pair_ok_d,pair_drop_d,pair_drop_trigger_old_d,pair_drop_soc_old_d,pair_drop_rx_early_d,pair_drop_rx_late_d,pair_drop_invalid_d,pair_drop_align_mismatch_d,pair_seq_gap_d,pair_cap_gap_d," \
    "pair_rx_age_samples_d,pair_rx_age_avg_us,pair_rx_age_last_us,pair_rx_age_max_us,pair_rx_age_gt50ms_d,pair_rx_age_gt80ms_d," \
    "holdover_active,holdover_enter_d,holdover_stale_d,holdover_good_d,holdover_bad_d,holdover_recover_d,holdover_timeout_d,residual_hold_d,residual_reject_d," \
    "imu_samples,imu_locked_samples,imu_holdover_samples,imu_nonmono_d,imu_period_min_us,imu_period_avg_us,imu_period_max_us," \
    "imu_map_drop_d,imu_ts_drop_d,imu_ts_drop_extend_no_anchor_d,imu_ts_drop_extend_no_candidate_d,imu_ts_drop_map_anchor_d," \
    "imu_ts_drop_plausible_past_d,imu_ts_drop_plausible_future_d,imu_ts_drop_plausible_nonmono_d,imu_ts_drop_plausible_large_step_d," \
    "imu_ts_tracker_reseed_d,imu_ts_tracker_wrap_count,imu_ts_tracker_last_gap_us,imu_queue_overwrite_d,imu_ts_wrap_d,imu_ts_anchor_update_d," \
    "imu_fifo_poll_d,imu_fifo_read_d,imu_fifo_overflow_d,imu_fifo_ts_d,imu_fifo_ts_decode_err_d,imu_fifo_level_last,imu_fifo_level_peak,imu_fifo_level_max,imu_fifo_last_error," \
    "pcl_frames,pcl_anchor_miss_d,pcl_nonmono_d,pcl_period_min_us,pcl_period_avg_us,pcl_period_max_us," \
    "pcl_latency_min_us,pcl_latency_avg_us,pcl_latency_max_us," \
    "imu_ts_drop_fifo_duplicate_d,imu_ts_drop_fifo_large_gap_d,imu_ts_fifo_stream_reseed_d,imu_ts_fifo_forward_gap_us,imu_ts_output_step_us," \
    "imu_ts_anchor_reject_d,imu_ts_anchor_raw_delta_us,imu_ts_anchor_elapsed_us,imu_ts_anchor_valid_wrap_d\n"

typedef struct {
    uint64_t count;
    uint64_t sum;
    uint64_t min;
    uint64_t max;
} eval_u64_stat_t;

typedef struct {
    uint64_t imu_samples;
    uint64_t imu_locked_samples;
    uint64_t imu_holdover_samples;
    uint64_t imu_nonmonotonic;
    uint64_t imu_last_ts_ns;
    uint8_t imu_last_valid;
    eval_u64_stat_t imu_period_us;

    uint64_t pcl_frames;
    uint64_t pcl_anchor_miss;
    uint64_t pcl_nonmonotonic;
    uint64_t pcl_last_frame_ts_us;
    uint8_t pcl_last_frame_valid;
    eval_u64_stat_t pcl_period_us;
    eval_u64_stat_t pcl_latency_us;
} time_sync_eval_state_t;

static pthread_mutex_t g_eval_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_eval_thread;
static int g_eval_thread_started;
static time_sync_eval_state_t g_eval;

static pthread_mutex_t g_eval_cfg_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_eval_en_cache;
static int g_eval_udp_en_cache;
static int g_eval_period_ms_cache;
static uint64_t g_eval_en_cache_ms;
static uint64_t g_eval_udp_en_cache_ms;
static uint64_t g_eval_period_cache_ms;

static void eval_stat_reset(eval_u64_stat_t *s)
{
    s->count = 0u;
    s->sum = 0u;
    s->min = UINT64_MAX;
    s->max = 0u;
}

static void eval_stat_add(eval_u64_stat_t *s, uint64_t v)
{
    if (s->count == 0u) {
        s->min = v;
        s->max = v;
    } else {
        if (v < s->min) {
            s->min = v;
        }
        if (v > s->max) {
            s->max = v;
        }
    }
    s->sum += v;
    s->count++;
}

static uint64_t eval_stat_min_or_zero(const eval_u64_stat_t *s)
{
    return (s->count == 0u) ? 0u : s->min;
}

static uint64_t eval_stat_avg_or_zero(const eval_u64_stat_t *s)
{
    return (s->count == 0u) ? 0u : (s->sum / s->count);
}

static uint64_t eval_stat_max_or_zero(const eval_u64_stat_t *s)
{
    return (s->count == 0u) ? 0u : s->max;
}

static uint64_t monotonic_ms(void)
{
    struct timespec ts;

    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

uint64_t time_sync_eval_realtime_us(void)
{
    struct timespec ts;

    (void)clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static int read_int_file_raw(const char *path, int default_value)
{
    char buf[32];
    int fd;
    ssize_t n;

    fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return default_value;
    }
    n = read(fd, buf, sizeof(buf) - 1u);
    close(fd);
    if (n <= 0) {
        return default_value;
    }
    buf[n] = '\0';
    return atoi(buf);
}

static int read_int_file(const char *path, int default_value)
{
    return (read_int_file_raw(path, default_value) > 0) ? 1 : 0;
}

static int cached_config_enabled(const char *path, int *cache, uint64_t *cache_ms)
{
    uint64_t now_ms = monotonic_ms();
    int value;

    pthread_mutex_lock(&g_eval_cfg_lock);
    if (*cache_ms == 0u || (now_ms - *cache_ms) >= TIME_SYNC_EVAL_CONFIG_TTL_MS) {
        *cache = read_int_file(path, 0);
        *cache_ms = now_ms;
    }
    value = *cache;
    pthread_mutex_unlock(&g_eval_cfg_lock);

    return value;
}

static int eval_enabled(void)
{
    return cached_config_enabled(TIME_SYNC_EVAL_EN_FILE, &g_eval_en_cache, &g_eval_en_cache_ms);
}

int time_sync_eval_udp_enabled(void)
{
    return cached_config_enabled(TIME_SYNC_EVAL_UDP_EN_FILE,
                                 &g_eval_udp_en_cache,
                                 &g_eval_udp_en_cache_ms);
}

static uint32_t eval_sample_period_ms(void)
{
    uint64_t now_ms = monotonic_ms();
    int value;

    pthread_mutex_lock(&g_eval_cfg_lock);
    if (g_eval_period_cache_ms == 0u || (now_ms - g_eval_period_cache_ms) >= TIME_SYNC_EVAL_CONFIG_TTL_MS) {
        g_eval_period_ms_cache = read_int_file_raw(TIME_SYNC_EVAL_PERIOD_MS_FILE,
                                                   (int)TIME_SYNC_EVAL_SAMPLE_DEFAULT_MS);
        g_eval_period_cache_ms = now_ms;
    }
    value = g_eval_period_ms_cache;
    pthread_mutex_unlock(&g_eval_cfg_lock);

    if (value < (int)TIME_SYNC_EVAL_SAMPLE_MIN_MS) {
        return TIME_SYNC_EVAL_SAMPLE_MIN_MS;
    }
    if (value > (int)TIME_SYNC_EVAL_SAMPLE_MAX_MS) {
        return TIME_SYNC_EVAL_SAMPLE_MAX_MS;
    }
    return (uint32_t)value;
}

void time_sync_eval_pack_le56(uint8_t dst[7], uint64_t value)
{
    uint8_t i;

    if (dst == NULL) {
        return;
    }
    for (i = 0u; i < 7u; i++) {
        dst[i] = (uint8_t)((value >> (8u * i)) & 0xffu);
    }
}

static void reset_window_locked(void)
{
    g_eval.imu_samples = 0u;
    g_eval.imu_locked_samples = 0u;
    g_eval.imu_holdover_samples = 0u;
    g_eval.imu_nonmonotonic = 0u;
    eval_stat_reset(&g_eval.imu_period_us);

    g_eval.pcl_frames = 0u;
    g_eval.pcl_anchor_miss = 0u;
    g_eval.pcl_nonmonotonic = 0u;
    eval_stat_reset(&g_eval.pcl_period_us);
    eval_stat_reset(&g_eval.pcl_latency_us);
}

static void snapshot_and_reset(time_sync_eval_state_t *out)
{
    pthread_mutex_lock(&g_eval_lock);
    *out = g_eval;
    reset_window_locked();
    pthread_mutex_unlock(&g_eval_lock);
}

static uint64_t u32_delta(uint32_t now, uint32_t prev, int valid)
{
    if (!valid) {
        return 0u;
    }
    return (uint64_t)(uint32_t)(now - prev);
}

static uint64_t u64_delta(uint64_t now, uint64_t prev, int valid)
{
    if (!valid) {
        return 0u;
    }
    return now - prev;
}

static int csv_needs_header(void)
{
    struct stat st;

    if (stat(TIME_SYNC_EVAL_CSV_PATH, &st) != 0) {
        return 1;
    }
    return (st.st_size == 0) ? 1 : 0;
}

static int csv_header_matches(void)
{
    char buf[sizeof(TIME_SYNC_EVAL_CSV_HEADER)];
    FILE *fp;
    size_t n;

    fp = fopen(TIME_SYNC_EVAL_CSV_PATH, "r");
    if (fp == NULL) {
        return 1;
    }
    n = fread(buf, 1u, sizeof(buf) - 1u, fp);
    fclose(fp);
    buf[n] = '\0';
    return (strcmp(buf, TIME_SYNC_EVAL_CSV_HEADER) == 0) ? 1 : 0;
}

static void rotate_csv_if_needed(void)
{
    struct stat st;

    if (stat(TIME_SYNC_EVAL_CSV_PATH, &st) != 0) {
        return;
    }
    if ((uint64_t)st.st_size <= TIME_SYNC_EVAL_CSV_MAX_BYTES && csv_header_matches()) {
        return;
    }
    (void)unlink(TIME_SYNC_EVAL_CSV_PATH_OLD);
    if (rename(TIME_SYNC_EVAL_CSV_PATH, TIME_SYNC_EVAL_CSV_PATH_OLD) != 0) {
        FS_LOGW("rotate %s failed: %s", TIME_SYNC_EVAL_CSV_PATH, strerror(errno));
    }
}

static uint64_t ns_pair_to_u64(uint32_t hi, uint32_t lo)
{
    return ((uint64_t)hi << 32) | (uint64_t)lo;
}

static void write_csv_row(const time_sync_eval_state_t *win)
{
    static int last_diag_valid;
    static uint32_t last_trigger_irq;
    static uint32_t last_fsync_pulse;
    static uint32_t last_trigger_ring_overflow_count;
    static uint32_t last_fsync_overlap;
    static uint32_t last_soc_sync_rx;
    static uint32_t last_soc_drop_invalid;
    static uint32_t last_soc_drop_queue_full;
    static uint32_t last_pair_ok;
    static uint32_t last_pair_drop_trigger_old;
    static uint32_t last_pair_drop_soc_old;
    static uint32_t last_pair_drop_rx_early;
    static uint32_t last_pair_drop_rx_late;
    static uint32_t last_pair_drop_invalid;
    static uint32_t last_pair_drop_align_mismatch;
    static uint32_t last_pair_seq_gap;
    static uint32_t last_pair_cap_gap;
    static uint32_t last_pair_rx_age_sample_count;
    static uint64_t last_pair_rx_age_sum_us;
    static uint32_t last_pair_rx_age_gt50ms;
    static uint32_t last_pair_rx_age_gt80ms;
    static uint32_t last_map_lock;
    static uint32_t last_resync;
    static uint32_t last_map_lost;
    static uint32_t last_holdover;
    static uint32_t last_holdover_stale;
    static uint32_t last_holdover_good;
    static uint32_t last_holdover_bad;
    static uint32_t last_holdover_recover;
    static uint32_t last_holdover_timeout;
    static uint32_t last_residual_hold;
    static uint32_t last_residual_reject;
    static uint32_t last_imu_ts_wrap;
    static uint32_t last_imu_ts_anchor_update;
    static int last_imu_diag_valid;
    static uint32_t last_imu_map_drop;
    static uint32_t last_imu_ts_drop;
    static uint32_t last_imu_ts_drop_extend_no_anchor;
    static uint32_t last_imu_ts_drop_extend_no_candidate;
    static uint32_t last_imu_ts_drop_map_anchor;
    static uint32_t last_imu_ts_drop_plausible_past;
    static uint32_t last_imu_ts_drop_plausible_future;
    static uint32_t last_imu_ts_drop_plausible_nonmono;
    static uint32_t last_imu_ts_drop_plausible_large_step;
    static uint32_t last_imu_ts_tracker_reseed;
    static uint32_t last_imu_ts_drop_fifo_duplicate;
    static uint32_t last_imu_ts_drop_fifo_large_gap;
    static uint32_t last_imu_ts_fifo_stream_reseed;
    static uint32_t last_imu_ts_anchor_reject;
    static uint32_t last_imu_ts_anchor_valid_wrap;
    static uint32_t last_imu_queue_overwrite;
    static uint32_t last_imu_fifo_poll;
    static uint32_t last_imu_fifo_read;
    static uint32_t last_imu_fifo_overflow;
    static uint32_t last_imu_fifo_ts;
    static uint32_t last_imu_fifo_ts_decode_err;

    mcu_time_sync_diag_t time_diag;
    mcu_imu_push_diag_t imu_diag;
    int time_ret;
    int imu_ret;
    uint64_t now_us;
    uint64_t now_mono_ms;
    uint32_t sample_period_ms;
    uint64_t soc_est_ns = 0u;
    uint64_t anchor_soc_ns = 0u;
    int64_t map_diag_offset_ms = 0;
    int64_t map_anchor_age_ms = 0;
    uint64_t sync_event_irq_d = 0u;
    uint64_t fsync_pulse_d = 0u;
    uint64_t sync_event_queue_overflow_d = 0u;
    uint64_t fsync_overlap_d = 0u;
    uint64_t soc_sync_rx_d = 0u;
    uint64_t soc_drop_invalid_d = 0u;
    uint64_t soc_drop_queue_full_d = 0u;
    uint64_t pair_ok_d = 0u;
    uint64_t pair_drop_d = 0u;
    uint64_t pair_drop_trigger_old_d = 0u;
    uint64_t pair_drop_soc_old_d = 0u;
    uint64_t pair_drop_rx_early_d = 0u;
    uint64_t pair_drop_rx_late_d = 0u;
    uint64_t pair_drop_invalid_d = 0u;
    uint64_t pair_drop_align_mismatch_d = 0u;
    uint64_t pair_seq_gap_d = 0u;
    uint64_t pair_cap_gap_d = 0u;
    uint64_t pair_rx_age_samples_d = 0u;
    uint64_t pair_rx_age_sum_us_d = 0u;
    uint64_t pair_rx_age_avg_us = 0u;
    uint64_t pair_rx_age_gt50ms_d = 0u;
    uint64_t pair_rx_age_gt80ms_d = 0u;
    uint64_t map_lock_d = 0u;
    uint64_t resync_d = 0u;
    uint64_t map_lost_d = 0u;
    uint64_t holdover_enter_d = 0u;
    uint64_t holdover_stale_d = 0u;
    uint64_t holdover_good_d = 0u;
    uint64_t holdover_bad_d = 0u;
    uint64_t holdover_recover_d = 0u;
    uint64_t holdover_timeout_d = 0u;
    uint64_t residual_hold_d = 0u;
    uint64_t residual_reject_d = 0u;
    uint64_t imu_map_drop_d = 0u;
    uint64_t imu_ts_drop_d = 0u;
    uint64_t imu_ts_drop_extend_no_anchor_d = 0u;
    uint64_t imu_ts_drop_extend_no_candidate_d = 0u;
    uint64_t imu_ts_drop_map_anchor_d = 0u;
    uint64_t imu_ts_drop_plausible_past_d = 0u;
    uint64_t imu_ts_drop_plausible_future_d = 0u;
    uint64_t imu_ts_drop_plausible_nonmono_d = 0u;
    uint64_t imu_ts_drop_plausible_large_step_d = 0u;
    uint64_t imu_ts_tracker_reseed_d = 0u;
    uint64_t imu_ts_drop_fifo_duplicate_d = 0u;
    uint64_t imu_ts_drop_fifo_large_gap_d = 0u;
    uint64_t imu_ts_fifo_stream_reseed_d = 0u;
    uint64_t imu_ts_anchor_reject_d = 0u;
    uint64_t imu_ts_anchor_valid_wrap_d = 0u;
    uint64_t imu_queue_overwrite_d = 0u;
    uint64_t imu_ts_wrap_d = 0u;
    uint64_t imu_ts_anchor_update_d = 0u;
    uint64_t imu_fifo_poll_d = 0u;
    uint64_t imu_fifo_read_d = 0u;
    uint64_t imu_fifo_overflow_d = 0u;
    uint64_t imu_fifo_ts_d = 0u;
    uint64_t imu_fifo_ts_decode_err_d = 0u;
    FILE *fp;
    int need_header;

    memset(&time_diag, 0, sizeof(time_diag));
    memset(&imu_diag, 0, sizeof(imu_diag));

    now_us = time_sync_eval_realtime_us();
    now_mono_ms = monotonic_ms();
    sample_period_ms = eval_sample_period_ms();

    time_ret = mcu_prot_get_time_sync_diag(&time_diag);
    if (time_ret == 0) {
        soc_est_ns = ns_pair_to_u64(time_diag.now_soc_est_ns_hi, time_diag.now_soc_est_ns_lo);
        anchor_soc_ns = ns_pair_to_u64(time_diag.anchor_soc_ns_hi, time_diag.anchor_soc_ns_lo);
        if (soc_est_ns != 0u) {
            map_diag_offset_ms = (int64_t)(((int64_t)(now_us * 1000ull) - (int64_t)soc_est_ns) / 1000000ll);
            if (soc_est_ns >= anchor_soc_ns) {
                map_anchor_age_ms = (int64_t)((soc_est_ns - anchor_soc_ns) / 1000000ull);
            }
        }

        sync_event_irq_d = u32_delta(time_diag.trigger_irq_count, last_trigger_irq, last_diag_valid);
        fsync_pulse_d = u32_delta(time_diag.fsync_pulse_count, last_fsync_pulse, last_diag_valid);
        sync_event_queue_overflow_d = u32_delta(time_diag.trigger_ring_overflow_count,
                                                last_trigger_ring_overflow_count,
                                                last_diag_valid);
        fsync_overlap_d = u32_delta(time_diag.fsync_overlap_count,
                                    last_fsync_overlap,
                                    last_diag_valid);
        soc_sync_rx_d = u32_delta(time_diag.soc_sync_rx_count, last_soc_sync_rx, last_diag_valid);
        soc_drop_invalid_d = u32_delta(time_diag.soc_sync_drop_invalid_count,
                                       last_soc_drop_invalid,
                                       last_diag_valid);
        soc_drop_queue_full_d = u32_delta(time_diag.soc_sync_drop_queue_full_count,
                                          last_soc_drop_queue_full,
                                          last_diag_valid);
        pair_ok_d = u32_delta(time_diag.sync_pair_count, last_pair_ok, last_diag_valid);
        pair_drop_trigger_old_d = u32_delta(time_diag.sync_pair_drop_trigger_old_count,
                                            last_pair_drop_trigger_old,
                                            last_diag_valid);
        pair_drop_soc_old_d = u32_delta(time_diag.sync_pair_drop_soc_old_count,
                                        last_pair_drop_soc_old,
                                        last_diag_valid);
        pair_drop_rx_early_d = u32_delta(time_diag.sync_pair_drop_rx_early_count,
                                         last_pair_drop_rx_early,
                                         last_diag_valid);
        pair_drop_rx_late_d = u32_delta(time_diag.sync_pair_drop_rx_late_count,
                                        last_pair_drop_rx_late,
                                        last_diag_valid);
        pair_drop_invalid_d = u32_delta(time_diag.sync_pair_drop_invalid_count,
                                        last_pair_drop_invalid,
                                        last_diag_valid);
        pair_drop_align_mismatch_d = u32_delta(time_diag.sync_pair_drop_align_mismatch_count,
                                               last_pair_drop_align_mismatch,
                                               last_diag_valid);
        pair_drop_d = pair_drop_trigger_old_d
                    + pair_drop_soc_old_d
                    + pair_drop_rx_early_d
                    + pair_drop_rx_late_d
                    + pair_drop_invalid_d
                    + pair_drop_align_mismatch_d;
        pair_seq_gap_d = u32_delta(time_diag.sync_pair_seq_gap, last_pair_seq_gap, last_diag_valid);
        pair_cap_gap_d = u32_delta(time_diag.sync_pair_capture_gap, last_pair_cap_gap, last_diag_valid);
        {
            uint64_t pair_rx_age_sum_us = ns_pair_to_u64(time_diag.pair_rx_age_sum_us_hi,
                                                         time_diag.pair_rx_age_sum_us_lo);
            pair_rx_age_samples_d = u32_delta(time_diag.pair_rx_age_sample_count,
                                              last_pair_rx_age_sample_count,
                                              last_diag_valid);
            pair_rx_age_sum_us_d = u64_delta(pair_rx_age_sum_us,
                                             last_pair_rx_age_sum_us,
                                             last_diag_valid);
            pair_rx_age_avg_us = (pair_rx_age_samples_d > 0u)
                               ? (pair_rx_age_sum_us_d / pair_rx_age_samples_d)
                               : 0u;
            pair_rx_age_gt50ms_d = u32_delta(time_diag.pair_rx_age_gt50ms_count,
                                             last_pair_rx_age_gt50ms,
                                             last_diag_valid);
            pair_rx_age_gt80ms_d = u32_delta(time_diag.pair_rx_age_gt80ms_count,
                                             last_pair_rx_age_gt80ms,
                                             last_diag_valid);
            last_pair_rx_age_sample_count = time_diag.pair_rx_age_sample_count;
            last_pair_rx_age_sum_us = pair_rx_age_sum_us;
            last_pair_rx_age_gt50ms = time_diag.pair_rx_age_gt50ms_count;
            last_pair_rx_age_gt80ms = time_diag.pair_rx_age_gt80ms_count;
        }
        map_lock_d = u32_delta(time_diag.time_map_locked_count, last_map_lock, last_diag_valid);
        resync_d = u32_delta(time_diag.resync_count, last_resync, last_diag_valid);
        map_lost_d = u32_delta(time_diag.time_map_lost_count, last_map_lost, last_diag_valid);
        holdover_enter_d = u32_delta(time_diag.holdover_count, last_holdover, last_diag_valid);
        holdover_stale_d = u32_delta(time_diag.holdover_stale_count,
                                      last_holdover_stale,
                                      last_diag_valid);
        holdover_good_d = u32_delta(time_diag.holdover_good_count,
                                     last_holdover_good,
                                     last_diag_valid);
        holdover_bad_d = u32_delta(time_diag.holdover_bad_count,
                                    last_holdover_bad,
                                    last_diag_valid);
        holdover_recover_d = u32_delta(time_diag.holdover_recover_count,
                                        last_holdover_recover,
                                        last_diag_valid);
        holdover_timeout_d = u32_delta(time_diag.holdover_timeout_count,
                                        last_holdover_timeout,
                                        last_diag_valid);
        residual_hold_d = u32_delta(time_diag.residual_hold_count, last_residual_hold, last_diag_valid);
        residual_reject_d = u32_delta(time_diag.residual_reject_count,
                                      last_residual_reject,
                                      last_diag_valid);
        imu_ts_wrap_d = u32_delta(time_diag.imu_timestamp_epoch_count,
                                  last_imu_ts_wrap,
                                  last_diag_valid);
        imu_ts_anchor_update_d = u32_delta(time_diag.imu_ts_anchor_update_count,
                                           last_imu_ts_anchor_update,
                                           last_diag_valid);
        last_trigger_irq = time_diag.trigger_irq_count;
        last_fsync_pulse = time_diag.fsync_pulse_count;
        last_trigger_ring_overflow_count = time_diag.trigger_ring_overflow_count;
        last_fsync_overlap = time_diag.fsync_overlap_count;
        last_soc_sync_rx = time_diag.soc_sync_rx_count;
        last_soc_drop_invalid = time_diag.soc_sync_drop_invalid_count;
        last_soc_drop_queue_full = time_diag.soc_sync_drop_queue_full_count;
        last_pair_ok = time_diag.sync_pair_count;
        last_pair_drop_trigger_old = time_diag.sync_pair_drop_trigger_old_count;
        last_pair_drop_soc_old = time_diag.sync_pair_drop_soc_old_count;
        last_pair_drop_rx_early = time_diag.sync_pair_drop_rx_early_count;
        last_pair_drop_rx_late = time_diag.sync_pair_drop_rx_late_count;
        last_pair_drop_invalid = time_diag.sync_pair_drop_invalid_count;
        last_pair_drop_align_mismatch = time_diag.sync_pair_drop_align_mismatch_count;
        last_pair_seq_gap = time_diag.sync_pair_seq_gap;
        last_pair_cap_gap = time_diag.sync_pair_capture_gap;
        last_map_lock = time_diag.time_map_locked_count;
        last_resync = time_diag.resync_count;
        last_map_lost = time_diag.time_map_lost_count;
        last_holdover = time_diag.holdover_count;
        last_holdover_stale = time_diag.holdover_stale_count;
        last_holdover_good = time_diag.holdover_good_count;
        last_holdover_bad = time_diag.holdover_bad_count;
        last_holdover_recover = time_diag.holdover_recover_count;
        last_holdover_timeout = time_diag.holdover_timeout_count;
        last_residual_hold = time_diag.residual_hold_count;
        last_residual_reject = time_diag.residual_reject_count;
        last_imu_ts_wrap = time_diag.imu_timestamp_epoch_count;
        last_imu_ts_anchor_update = time_diag.imu_ts_anchor_update_count;
        last_diag_valid = 1;
    }

    imu_ret = mcu_prot_get_imu_push_diag(&imu_diag);
    if (imu_ret == 0) {
        imu_map_drop_d = u32_delta(imu_diag.map_drop, last_imu_map_drop, last_imu_diag_valid);
        imu_ts_drop_d = u32_delta(imu_diag.ts_drop, last_imu_ts_drop, last_imu_diag_valid);
        imu_ts_drop_extend_no_anchor_d = u32_delta(imu_diag.imu_ts_drop_extend_no_anchor,
                                                   last_imu_ts_drop_extend_no_anchor,
                                                   last_imu_diag_valid);
        imu_ts_drop_extend_no_candidate_d = u32_delta(imu_diag.imu_ts_drop_extend_no_candidate,
                                                      last_imu_ts_drop_extend_no_candidate,
                                                      last_imu_diag_valid);
        imu_ts_drop_map_anchor_d = u32_delta(imu_diag.imu_ts_drop_map_anchor,
                                             last_imu_ts_drop_map_anchor,
                                             last_imu_diag_valid);
        imu_ts_drop_plausible_past_d = u32_delta(imu_diag.imu_ts_drop_plausible_past,
                                                 last_imu_ts_drop_plausible_past,
                                                 last_imu_diag_valid);
        imu_ts_drop_plausible_future_d = u32_delta(imu_diag.imu_ts_drop_plausible_future,
                                                   last_imu_ts_drop_plausible_future,
                                                   last_imu_diag_valid);
        imu_ts_drop_plausible_nonmono_d = u32_delta(imu_diag.imu_ts_drop_plausible_nonmono,
                                                    last_imu_ts_drop_plausible_nonmono,
                                                    last_imu_diag_valid);
        imu_ts_drop_plausible_large_step_d = u32_delta(imu_diag.imu_ts_drop_plausible_large_step,
                                                       last_imu_ts_drop_plausible_large_step,
                                                       last_imu_diag_valid);
        imu_ts_tracker_reseed_d = u32_delta(imu_diag.imu_ts_tracker_reseed_count,
                                            last_imu_ts_tracker_reseed,
                                            last_imu_diag_valid);
        imu_ts_drop_fifo_duplicate_d = u32_delta(imu_diag.imu_ts_drop_fifo_duplicate,
                                                 last_imu_ts_drop_fifo_duplicate,
                                                 last_imu_diag_valid);
        imu_ts_drop_fifo_large_gap_d = u32_delta(imu_diag.imu_ts_drop_fifo_large_gap,
                                                 last_imu_ts_drop_fifo_large_gap,
                                                 last_imu_diag_valid);
        imu_ts_fifo_stream_reseed_d = u32_delta(imu_diag.imu_ts_fifo_stream_reseed_count,
                                                last_imu_ts_fifo_stream_reseed,
                                                last_imu_diag_valid);
        imu_ts_anchor_reject_d = u32_delta(imu_diag.imu_ts_anchor_reject_count,
                                           last_imu_ts_anchor_reject,
                                           last_imu_diag_valid);
        imu_ts_anchor_valid_wrap_d = u32_delta(imu_diag.imu_ts_anchor_valid_wrap_count,
                                               last_imu_ts_anchor_valid_wrap,
                                               last_imu_diag_valid);
        imu_queue_overwrite_d = u32_delta(imu_diag.queue_overwrite_drop,
                                          last_imu_queue_overwrite,
                                          last_imu_diag_valid);
        imu_fifo_poll_d = u32_delta(imu_diag.fifo_poll_count, last_imu_fifo_poll, last_imu_diag_valid);
        imu_fifo_read_d = u32_delta(imu_diag.fifo_read_count, last_imu_fifo_read, last_imu_diag_valid);
        imu_fifo_overflow_d = u32_delta(imu_diag.fifo_overflow_count,
                                        last_imu_fifo_overflow,
                                        last_imu_diag_valid);
        imu_fifo_ts_d = u32_delta(imu_diag.fifo_ts_count, last_imu_fifo_ts, last_imu_diag_valid);
        imu_fifo_ts_decode_err_d = u32_delta(imu_diag.fifo_ts_decode_err,
                                             last_imu_fifo_ts_decode_err,
                                             last_imu_diag_valid);
        last_imu_map_drop = imu_diag.map_drop;
        last_imu_ts_drop = imu_diag.ts_drop;
        last_imu_ts_drop_extend_no_anchor = imu_diag.imu_ts_drop_extend_no_anchor;
        last_imu_ts_drop_extend_no_candidate = imu_diag.imu_ts_drop_extend_no_candidate;
        last_imu_ts_drop_map_anchor = imu_diag.imu_ts_drop_map_anchor;
        last_imu_ts_drop_plausible_past = imu_diag.imu_ts_drop_plausible_past;
        last_imu_ts_drop_plausible_future = imu_diag.imu_ts_drop_plausible_future;
        last_imu_ts_drop_plausible_nonmono = imu_diag.imu_ts_drop_plausible_nonmono;
        last_imu_ts_drop_plausible_large_step = imu_diag.imu_ts_drop_plausible_large_step;
        last_imu_ts_tracker_reseed = imu_diag.imu_ts_tracker_reseed_count;
        last_imu_ts_drop_fifo_duplicate = imu_diag.imu_ts_drop_fifo_duplicate;
        last_imu_ts_drop_fifo_large_gap = imu_diag.imu_ts_drop_fifo_large_gap;
        last_imu_ts_fifo_stream_reseed = imu_diag.imu_ts_fifo_stream_reseed_count;
        last_imu_ts_anchor_reject = imu_diag.imu_ts_anchor_reject_count;
        last_imu_ts_anchor_valid_wrap = imu_diag.imu_ts_anchor_valid_wrap_count;
        last_imu_queue_overwrite = imu_diag.queue_overwrite_drop;
        last_imu_fifo_poll = imu_diag.fifo_poll_count;
        last_imu_fifo_read = imu_diag.fifo_read_count;
        last_imu_fifo_overflow = imu_diag.fifo_overflow_count;
        last_imu_fifo_ts = imu_diag.fifo_ts_count;
        last_imu_fifo_ts_decode_err = imu_diag.fifo_ts_decode_err;
        last_imu_diag_valid = 1;
    }

    rotate_csv_if_needed();
    need_header = csv_needs_header();
    fp = fopen(TIME_SYNC_EVAL_CSV_PATH, "a");
    if (fp == NULL) {
        FS_LOGW("open %s failed: %s", TIME_SYNC_EVAL_CSV_PATH, strerror(errno));
        return;
    }
    if (need_header) {
        fprintf(fp, "%s", TIME_SYNC_EVAL_CSV_HEADER);
    }

    fprintf(fp,
            "%u,%llu,%llu,%u,%u,%u,"
            "%u,%lld,%lld,%u,%ld,%d,%llu,%llu,%llu,"
            "%llu,%llu,%llu,%llu,%llu,%llu,%llu,"
            "%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,"
            "%llu,%llu,%u,%u,%llu,%llu,"
            "%u,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,"
            "%llu,%llu,%llu,%llu,%llu,%llu,%llu,"
            "%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%u,%u,%llu,%llu,%llu,"
            "%llu,%llu,%llu,%llu,%llu,%u,%u,%u,%ld,"
            "%llu,%llu,%llu,%llu,%llu,%llu,"
            "%llu,%llu,%llu,%llu,%llu,%llu,%u,%u,%llu,%u,%u,%llu\n",
            TIME_SYNC_EVAL_SCHEMA_VERSION,
            (unsigned long long)now_us,
            (unsigned long long)now_mono_ms,
            sample_period_ms,
            (time_ret == 0) ? 1u : 0u,
            (imu_ret == 0) ? 1u : 0u,
            time_diag.map_locked,
            (long long)map_anchor_age_ms,
            (long long)map_diag_offset_ms,
            time_diag.holdover_last_residual_us,
            (long)time_diag.holdover_last_residual_signed_us,
            time_diag.map_drift_ppb,
            (unsigned long long)map_lock_d,
            (unsigned long long)map_lost_d,
            (unsigned long long)resync_d,
            (unsigned long long)sync_event_irq_d,
            (unsigned long long)fsync_pulse_d,
            (unsigned long long)sync_event_queue_overflow_d,
            (unsigned long long)fsync_overlap_d,
            (unsigned long long)soc_sync_rx_d,
            (unsigned long long)soc_drop_invalid_d,
            (unsigned long long)soc_drop_queue_full_d,
            (unsigned long long)pair_ok_d,
            (unsigned long long)pair_drop_d,
            (unsigned long long)pair_drop_trigger_old_d,
            (unsigned long long)pair_drop_soc_old_d,
            (unsigned long long)pair_drop_rx_early_d,
            (unsigned long long)pair_drop_rx_late_d,
            (unsigned long long)pair_drop_invalid_d,
            (unsigned long long)pair_drop_align_mismatch_d,
            (unsigned long long)pair_seq_gap_d,
            (unsigned long long)pair_cap_gap_d,
            (unsigned long long)pair_rx_age_samples_d,
            (unsigned long long)pair_rx_age_avg_us,
            time_diag.pair_rx_age_last_us,
            time_diag.pair_rx_age_max_us,
            (unsigned long long)pair_rx_age_gt50ms_d,
            (unsigned long long)pair_rx_age_gt80ms_d,
            time_diag.holdover_active,
            (unsigned long long)holdover_enter_d,
            (unsigned long long)holdover_stale_d,
            (unsigned long long)holdover_good_d,
            (unsigned long long)holdover_bad_d,
            (unsigned long long)holdover_recover_d,
            (unsigned long long)holdover_timeout_d,
            (unsigned long long)residual_hold_d,
            (unsigned long long)residual_reject_d,
            (unsigned long long)win->imu_samples,
            (unsigned long long)win->imu_locked_samples,
            (unsigned long long)win->imu_holdover_samples,
            (unsigned long long)win->imu_nonmonotonic,
            (unsigned long long)eval_stat_min_or_zero(&win->imu_period_us),
            (unsigned long long)eval_stat_avg_or_zero(&win->imu_period_us),
            (unsigned long long)eval_stat_max_or_zero(&win->imu_period_us),
            (unsigned long long)imu_map_drop_d,
            (unsigned long long)imu_ts_drop_d,
            (unsigned long long)imu_ts_drop_extend_no_anchor_d,
            (unsigned long long)imu_ts_drop_extend_no_candidate_d,
            (unsigned long long)imu_ts_drop_map_anchor_d,
            (unsigned long long)imu_ts_drop_plausible_past_d,
            (unsigned long long)imu_ts_drop_plausible_future_d,
            (unsigned long long)imu_ts_drop_plausible_nonmono_d,
            (unsigned long long)imu_ts_drop_plausible_large_step_d,
            (unsigned long long)imu_ts_tracker_reseed_d,
            imu_diag.imu_ts_tracker_wrap_count,
            imu_diag.imu_ts_tracker_last_gap_us,
            (unsigned long long)imu_queue_overwrite_d,
            (unsigned long long)imu_ts_wrap_d,
            (unsigned long long)imu_ts_anchor_update_d,
            (unsigned long long)imu_fifo_poll_d,
            (unsigned long long)imu_fifo_read_d,
            (unsigned long long)imu_fifo_overflow_d,
            (unsigned long long)imu_fifo_ts_d,
            (unsigned long long)imu_fifo_ts_decode_err_d,
            imu_diag.fifo_last_level_word,
            imu_diag.fifo_last_peak_level_word,
            imu_diag.fifo_max_level_word,
            (long)imu_diag.fifo_last_error,
            (unsigned long long)win->pcl_frames,
            (unsigned long long)win->pcl_anchor_miss,
            (unsigned long long)win->pcl_nonmonotonic,
            (unsigned long long)eval_stat_min_or_zero(&win->pcl_period_us),
            (unsigned long long)eval_stat_avg_or_zero(&win->pcl_period_us),
            (unsigned long long)eval_stat_max_or_zero(&win->pcl_period_us),
            (unsigned long long)eval_stat_min_or_zero(&win->pcl_latency_us),
            (unsigned long long)eval_stat_avg_or_zero(&win->pcl_latency_us),
            (unsigned long long)eval_stat_max_or_zero(&win->pcl_latency_us),
            (unsigned long long)imu_ts_drop_fifo_duplicate_d,
            (unsigned long long)imu_ts_drop_fifo_large_gap_d,
            (unsigned long long)imu_ts_fifo_stream_reseed_d,
            imu_diag.imu_ts_fifo_forward_gap_us,
            imu_diag.imu_ts_output_step_us,
            (unsigned long long)imu_ts_anchor_reject_d,
            imu_diag.imu_ts_anchor_raw_delta_us,
            imu_diag.imu_ts_anchor_elapsed_us,
            (unsigned long long)imu_ts_anchor_valid_wrap_d);
    fclose(fp);
}

static void *eval_thread_main(void *arg)
{
    time_sync_eval_state_t win;

    (void)arg;
    while (1) {
        usleep(eval_sample_period_ms() * 1000u);
        if (!eval_enabled()) {
            pthread_mutex_lock(&g_eval_lock);
            reset_window_locked();
            g_eval.imu_last_valid = 0u;
            g_eval.pcl_last_frame_valid = 0u;
            pthread_mutex_unlock(&g_eval_lock);
            continue;
        }
        memset(&win, 0, sizeof(win));
        eval_stat_reset(&win.imu_period_us);
        eval_stat_reset(&win.pcl_period_us);
        eval_stat_reset(&win.pcl_latency_us);
        snapshot_and_reset(&win);
        write_csv_row(&win);
    }
    return NULL;
}

static void ensure_eval_thread(void)
{
    pthread_mutex_lock(&g_eval_lock);
    if (!g_eval_thread_started) {
        reset_window_locked();
        if (pthread_create(&g_eval_thread, NULL, eval_thread_main, NULL) == 0) {
            (void)pthread_detach(g_eval_thread);
            g_eval_thread_started = 1;
        } else {
            FS_LOGW("failed to create time sync eval thread");
        }
    }
    pthread_mutex_unlock(&g_eval_lock);
}

void time_sync_eval_record_imu_batch(uint16_t sample_count)
{
    if (sample_count == 0u) {
        return;
    }
    ensure_eval_thread();
}

void time_sync_eval_record_imu_timestamp(uint64_t timestamp_ns, uint8_t timestamp_quality)
{
    ensure_eval_thread();

    pthread_mutex_lock(&g_eval_lock);
    if (g_eval.imu_last_valid != 0u) {
        if (timestamp_ns <= g_eval.imu_last_ts_ns) {
            g_eval.imu_nonmonotonic++;
        } else {
            eval_stat_add(&g_eval.imu_period_us, (timestamp_ns - g_eval.imu_last_ts_ns) / 1000ull);
        }
    }
    g_eval.imu_last_ts_ns = timestamp_ns;
    g_eval.imu_last_valid = 1u;
    g_eval.imu_samples++;
    if (timestamp_quality == MCU_IMU_PUSH_TIMESTAMP_QUALITY_HOLDOVER) {
        g_eval.imu_holdover_samples++;
    } else if (timestamp_quality == MCU_IMU_PUSH_TIMESTAMP_QUALITY_LOCKED) {
        g_eval.imu_locked_samples++;
    }
    pthread_mutex_unlock(&g_eval_lock);
}

void time_sync_eval_record_pointcloud_packet(uint32_t frame_id,
                                             uint64_t frame_timestamp_us,
                                             uint64_t send_realtime_us,
                                             int frame_start)
{
    (void)frame_id;
    ensure_eval_thread();

    pthread_mutex_lock(&g_eval_lock);
    if (frame_start && frame_timestamp_us != 0u) {
        g_eval.pcl_frames++;
        if (g_eval.pcl_last_frame_valid != 0u) {
            if (frame_timestamp_us > g_eval.pcl_last_frame_ts_us) {
                eval_stat_add(&g_eval.pcl_period_us, frame_timestamp_us - g_eval.pcl_last_frame_ts_us);
            } else {
                g_eval.pcl_nonmonotonic++;
            }
        }
        g_eval.pcl_last_frame_ts_us = frame_timestamp_us;
        g_eval.pcl_last_frame_valid = 1u;
        if (send_realtime_us >= frame_timestamp_us) {
            eval_stat_add(&g_eval.pcl_latency_us, send_realtime_us - frame_timestamp_us);
        }
    }
    pthread_mutex_unlock(&g_eval_lock);
}

void time_sync_eval_record_spad_anchor_miss(void)
{
    ensure_eval_thread();

    pthread_mutex_lock(&g_eval_lock);
    g_eval.pcl_anchor_miss++;
    pthread_mutex_unlock(&g_eval_lock);
}
