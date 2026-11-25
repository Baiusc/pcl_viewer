#include "mcu_time_sync_pub.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "fs_pointcloud_timesync_caps.h"
#include "mcu_protocol.h"
#include "utils/fs_err.h"
#include "utils/fs_log.h"
#include "drivers/fs_frame_sync_ioctl.h"
#include "drivers/fs_gpio_trig_ioctl.h"
#include "spad_time_anchor.h"

#undef LOG_TAG
#define LOG_TAG "mcu.time_sync_pub"

/* PointCloud-only timestamp publisher. */
#define SOC_SYNC_STATE_LOCKED 1u

static int g_trig_fd = -1;
static int g_frame_sync_fd = -1;
static uint8_t g_soc_sync_state = 0u;
static uint32_t g_expected_trig_seq = 1u;
static uint32_t g_expected_fsync_seq = 1u;
static uint8_t g_frame_sync_fallback_warned = 0u;
static uint32_t g_frame_sync_fallback_log_suppressed = 0u;
static uint32_t g_trig_events_since_last_fsync = 0u;
static fs_pointcloud_timesync_caps_t g_pointcloud_caps;

#define FRAME_SYNC_SILENCE_TRIG_THRESHOLD 10u

static bool pcl_internal_trigger_enabled(void)
{
    return g_pointcloud_caps.trigger_mode == FS_PCL_TRIGGER_INTERNAL;
}

static void log_frame_sync_event_flags(uint32_t fsync_seq, uint32_t flags)
{
    if ((flags & FS_FRAME_SYNC_EVENT_F_OVERFLOW) != 0u) {
        FS_LOGW("frame_sync overflow: fsync_seq=%u flags=0x%08x", fsync_seq, flags);
    }
    if ((flags & FS_FRAME_SYNC_EVENT_F_SEQ_GAP) != 0u) {
        FS_LOGW("frame_sync seq gap flag: fsync_seq=%u flags=0x%08x", fsync_seq, flags);
    }
    if ((flags & FS_FRAME_SYNC_EVENT_F_IRQ_MISSED) != 0u) {
        FS_LOGW("frame_sync irq missed: fsync_seq=%u flags=0x%08x", fsync_seq, flags);
    }
}

static int handle_frame_sync_event_for_anchor(const struct fs_frame_sync_event *event)
{
    if (event == NULL || event->fsync_seq == 0u) {
        return FS_ERR_BAD_PROTOCOL;
    }

    if (event->fsync_seq != g_expected_fsync_seq) {
        FS_LOGW("frame_sync anchor seq gap: expected=%u actual=%u flags=0x%08x",
                g_expected_fsync_seq,
                event->fsync_seq,
                event->flags);
    }

    log_frame_sync_event_flags(event->fsync_seq, event->flags);
    g_trig_events_since_last_fsync = 0u;

    g_expected_fsync_seq = event->fsync_seq + 1u;
    if (g_expected_fsync_seq == 0u) {
        g_expected_fsync_seq = 1u;
    }

    if (g_soc_sync_state != SOC_SYNC_STATE_LOCKED) {
        return FS_SUCCESS;
    }

    return spad_anchor_insert_pending_fsync(event->fsync_seq, event->t_frame_sync_ns);
}

static int frame_sync_anchor_open_optional(void)
{
    if (g_frame_sync_fd >= 0) {
        return FS_SUCCESS;
    }

    g_frame_sync_fd = open(FS_FRAME_SYNC_DEVICE_PATH, O_RDWR | O_NONBLOCK);
    if (g_frame_sync_fd < 0) {
        FS_LOGW("frame_sync device %s unavailable (errno=%d); external profile will "
                "use trigger fallback for frame anchor when locked",
                FS_FRAME_SYNC_DEVICE_PATH, errno);
        return FS_ERR_IO;
    }

    FS_LOGI("frame_sync anchor capture opened on fd=%d (nonblock)", g_frame_sync_fd);
    return FS_SUCCESS;
}

static int frame_sync_anchor_start(void)
{
    if (g_frame_sync_fd < 0) {
        return FS_ERR_CHECK_INIT;
    }

    if (ioctl(g_frame_sync_fd, FS_FRAME_SYNC_IOCTL_START) < 0) {
        FS_LOGE("FS_FRAME_SYNC_IOCTL_START failed for frame_sync anchor fd=%d", g_frame_sync_fd);
        return FS_ERR_IO;
    }

    return FS_SUCCESS;
}

static void frame_sync_anchor_close(void)
{
    if (g_frame_sync_fd >= 0) {
        (void)ioctl(g_frame_sync_fd, FS_FRAME_SYNC_IOCTL_STOP);
        close(g_frame_sync_fd);
        g_frame_sync_fd = -1;
    }
}

static void disable_frame_sync_anchor_for_fallback(const char *reason)
{
    if (!g_pointcloud_caps.pointcloud_true_fsync_anchor_supported && g_frame_sync_fd < 0) {
        return;
    }

    FS_LOGW("frame_sync anchor disabled (%s); using trigger fallback", reason);
    frame_sync_anchor_close();
    g_pointcloud_caps.pointcloud_true_fsync_anchor_supported = false;
    g_trig_events_since_last_fsync = 0u;
    g_frame_sync_fallback_warned = 0u;
    g_frame_sync_fallback_log_suppressed = 0u;
}

static void log_frame_sync_fallback_rate_limited(uint32_t seq)
{
    if (g_frame_sync_fallback_warned == 0u) {
        FS_LOGW("frame_sync unavailable; using trigger fallback for frame anchor, first seq=%u",
                seq);
        g_frame_sync_fallback_warned = 1u;
        g_frame_sync_fallback_log_suppressed = 0u;
        return;
    }

    g_frame_sync_fallback_log_suppressed++;
    if ((g_frame_sync_fallback_log_suppressed % 100u) == 0u) {
        FS_LOGW("frame_sync fallback still active; suppressed=%u latest_seq=%u",
                g_frame_sync_fallback_log_suppressed, seq);
    }
}

static int drain_frame_sync_anchors_nonblock(void)
{
    int processed = 0;

    if (g_frame_sync_fd < 0) {
        return 0;
    }

    for (;;) {
        struct fs_frame_sync_event event;
        ssize_t n;

        do {
            n = read(g_frame_sync_fd, &event, sizeof(event));
        } while (n < 0 && errno == EINTR);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            FS_LOGW("frame_sync anchor drain read failed: errno=%d", errno);
            break;
        }
        if (n == 0) {
            break;
        }
        if ((size_t)n != sizeof(event)) {
            FS_LOGW("frame_sync anchor drain short read: n=%zd", n);
            break;
        }
        if (event.fsync_seq == 0u) {
            FS_LOGW("frame_sync anchor drain ignored fsync_seq=0");
            break;
        }

        if (handle_frame_sync_event_for_anchor(&event) != FS_SUCCESS) {
            FS_LOGW("frame_sync anchor insert failed: fsync_seq=%u", event.fsync_seq);
        }
        processed++;
    }

    return processed;
}

static int process_gpio_trig_event(const struct fs_gpio_trig_event *event)
{
    int ret;

    if (event == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    if (event->seq == 0u) {
        FS_LOGE("invalid fs_gpio_trig event seq=0");
        return FS_ERR_BAD_PROTOCOL;
    }

    if (event->seq != g_expected_trig_seq) {
        FS_LOGW("fs_gpio_trig seq gap: expected=%u actual=%u flags=0x%08x",
                g_expected_trig_seq, event->seq, event->flags);
    }

    if ((event->flags & FS_GPIO_TRIG_EVENT_F_OVERFLOW) != 0u) {
        FS_LOGW("fs_gpio_trig overflow flag set: seq=%u flags=0x%08x",
                event->seq, event->flags);
    }

    g_expected_trig_seq = event->seq + 1u;
    if (g_expected_trig_seq == 0u) {
        g_expected_trig_seq = 1u;
    }

    if (g_pointcloud_caps.pointcloud_true_fsync_anchor_supported && g_frame_sync_fd >= 0) {
        g_trig_events_since_last_fsync++;
        if (g_trig_events_since_last_fsync >= FRAME_SYNC_SILENCE_TRIG_THRESHOLD) {
            disable_frame_sync_anchor_for_fallback("no fsync events");
        }
    }

    if (g_soc_sync_state == SOC_SYNC_STATE_LOCKED &&
        !g_pointcloud_caps.pointcloud_true_fsync_anchor_supported) {
        log_frame_sync_fallback_rate_limited(event->seq);

        ret = spad_anchor_insert(event->seq, event->t_trig_ns);
        if (ret != FS_SUCCESS) {
            return ret;
        }
    }

    return mcu_prot_send_soc_sync_pub(event->seq,
                                      event->t_trig_ns,
                                      g_soc_sync_state);
}

static int poll_gpio_trig_and_frame_sync_once(void)
{
    if (g_trig_fd < 0 && g_frame_sync_fd < 0) {
        return MCU_TIME_SYNC_PUB_STOPPED;
    }

    for (;;) {
        struct pollfd fds[2];
        nfds_t nfds = 0u;
        int trig_idx = -1;
        int fsync_idx = -1;
        int pr;

        if (g_trig_fd >= 0) {
            trig_idx = (int)nfds;
            fds[nfds].fd = g_trig_fd;
            fds[nfds].events = POLLIN | POLLERR | POLLHUP;
            fds[nfds].revents = 0;
            nfds++;
        }

        if (g_frame_sync_fd >= 0) {
            fsync_idx = (int)nfds;
            fds[nfds].fd = g_frame_sync_fd;
            fds[nfds].events = POLLIN | POLLERR | POLLHUP;
            fds[nfds].revents = 0;
            nfds++;
        }

        if (nfds == 0u) {
            return MCU_TIME_SYNC_PUB_STOPPED;
        }

        do {
            pr = poll(fds, nfds, -1);
        } while (pr < 0 && errno == EINTR);

        if (pr < 0) {
            FS_LOGE("time sync poll failed: errno=%d", errno);
            return FS_ERR_IO;
        }

        if (fsync_idx >= 0) {
            short revents = fds[fsync_idx].revents;

            if ((revents & POLLIN) != 0) {
                (void)drain_frame_sync_anchors_nonblock();
            }

            if ((revents & (POLLERR | POLLHUP)) != 0) {
                FS_LOGW("frame_sync anchor fd error/hup revents=0x%x; entering trigger fallback",
                        (unsigned int)revents);
                frame_sync_anchor_close();
            }
        }

        if (trig_idx >= 0) {
            short revents = fds[trig_idx].revents;

            if ((revents & POLLIN) != 0) {
                struct fs_gpio_trig_event event;
                ssize_t n;

                do {
                    n = read(g_trig_fd, &event, sizeof(event));
                } while (n < 0 && errno == EINTR);

                if (n == 0) {
                    return MCU_TIME_SYNC_PUB_STOPPED;
                }

                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        continue;
                    }
                    if (errno == EBUSY) {
                        return FS_ERR_BUSY;
                    }
                    return FS_ERR_IO;
                }

                if ((size_t)n != sizeof(event)) {
                    return FS_ERR_BAD_PROTOCOL;
                }

                (void)drain_frame_sync_anchors_nonblock();

                return process_gpio_trig_event(&event);
            }

            if ((revents & POLLHUP) != 0) {
                return MCU_TIME_SYNC_PUB_STOPPED;
            }

            if ((revents & POLLERR) != 0) {
                FS_LOGE("gpio trigger fd poll error revents=0x%x",
                        (unsigned int)revents);
                return FS_ERR_IO;
            }
        }
    }
}

/* ---- Public API (pointcloud-only, external trigger) ---- */

int mcu_time_sync_pub_prepare(const fs_pointcloud_timesync_caps_t *caps)
{
    if (caps == NULL) return FS_ERR_BAD_PARAMS;
    g_pointcloud_caps = *caps;

    if (!pcl_internal_trigger_enabled()) {
        /* external trigger mode: gpio_trig is required */
        if (g_trig_fd < 0) {
            g_trig_fd = open(FS_GPIO_TRIG_DEVICE_PATH, O_RDWR);
            if (g_trig_fd < 0) {
                FS_LOGE("failed to open %s (errno=%d)", FS_GPIO_TRIG_DEVICE_PATH, errno);
                g_pointcloud_caps.pointcloud_timesync_supported = false;
                return FS_ERR_IO;
            }
        }
    } else if (g_trig_fd >= 0) {
        /* internal trigger mode does not use /dev/fs_gpio_trig */
        close(g_trig_fd);
        g_trig_fd = -1;
    }

    /* open frame_sync anchor (optional) */
    if (g_pointcloud_caps.pointcloud_true_fsync_anchor_supported && g_frame_sync_fd < 0) {
        g_frame_sync_fd = open(FS_FRAME_SYNC_DEVICE_PATH, O_RDWR | O_NONBLOCK);
        if (g_frame_sync_fd < 0) {
            FS_LOGW("frame_sync device %s unavailable (errno=%d); will use trigger fallback",
                    FS_FRAME_SYNC_DEVICE_PATH, errno);
            g_pointcloud_caps.pointcloud_true_fsync_anchor_supported = false;
        } else {
            FS_LOGI("frame_sync anchor capture opened on fd=%d (nonblock)", g_frame_sync_fd);
        }
    }

    spad_anchor_set_trigger_delay_ns(g_pointcloud_caps.trigger_to_spad_fsync_delay_ns);
    FS_LOGI("time sync publisher prepared: trig_fd=%d fsync_fd=%d delay_ns=%lld",
            g_trig_fd, g_frame_sync_fd, (long long)g_pointcloud_caps.trigger_to_spad_fsync_delay_ns);
    return FS_SUCCESS;
}

int mcu_time_sync_pub_start_anchor(void)
{
    if (!g_pointcloud_caps.pointcloud_true_fsync_anchor_supported) return FS_SUCCESS;
    if (g_frame_sync_fd < 0) return FS_SUCCESS;

    if (ioctl(g_frame_sync_fd, FS_FRAME_SYNC_IOCTL_START) < 0) {
        FS_LOGW("FS_FRAME_SYNC_IOCTL_START failed (errno=%d); switching to trigger fallback", errno);
        close(g_frame_sync_fd);
        g_frame_sync_fd = -1;

        if (pcl_internal_trigger_enabled()) {
            FS_LOGE("internal trigger mode requires frame_sync anchor");
            return FS_ERR_IO;
        }

        g_pointcloud_caps.pointcloud_true_fsync_anchor_supported = false;
        return FS_SUCCESS;  /* not fatal */
    }
    FS_LOGI("frame_sync anchor started on fd=%d", g_frame_sync_fd);
    return FS_SUCCESS;
}

int mcu_time_sync_pub_start_trigger(void)
{
    struct fs_gpio_trig_config cfg;

    g_expected_trig_seq = 1u;
    g_expected_fsync_seq = 1u;
    g_frame_sync_fallback_warned = 0u;
    g_frame_sync_fallback_log_suppressed = 0u;
    g_trig_events_since_last_fsync = 0u;
    spad_anchor_init();
    g_soc_sync_state = SOC_SYNC_STATE_LOCKED;

    if (pcl_internal_trigger_enabled()) {
        FS_LOGI("time sync session started: internal trigger mode, frame_sync-only");
        return FS_SUCCESS;
    }

    if (g_trig_fd < 0) {
        FS_LOGE("mcu_time_sync_pub_start_trigger: gpio_trig not prepared");
        return FS_ERR_CHECK_INIT;
    }

    cfg.period_ms     = 100u;
    cfg.pulse_high_ms = 10u;
    cfg.ts_point      = FS_GPIO_TRIG_TS_BEFORE;
    cfg.reserved[0]   = 0u;
    cfg.reserved[1]   = 0u;
    cfg.reserved[2]   = 0u;
    cfg.reserved[3]   = 0u;
    cfg.reserved[4]   = 0u;

    if (ioctl(g_trig_fd, FS_GPIO_TRIG_IOCTL_SET_CONFIG, &cfg) < 0) {
        FS_LOGE("FS_GPIO_TRIG_IOCTL_SET_CONFIG failed");
        return FS_ERR_IO;
    }
    if (ioctl(g_trig_fd, FS_GPIO_TRIG_IOCTL_START) < 0) {
        FS_LOGE("FS_GPIO_TRIG_IOCTL_START failed");
        return FS_ERR_IO;
    }

    FS_LOGI("time sync trigger started: 10 Hz on fd=%d", g_trig_fd);
    return FS_SUCCESS;
}

int mcu_time_sync_pub_stop(void)
{
    g_soc_sync_state = 0u;
    g_trig_events_since_last_fsync = 0u;

    if (g_trig_fd >= 0) {
        (void)ioctl(g_trig_fd, FS_GPIO_TRIG_IOCTL_STOP);
    }
    if (g_frame_sync_fd >= 0) {
        (void)ioctl(g_frame_sync_fd, FS_FRAME_SYNC_IOCTL_STOP);
    }
    spad_anchor_init();
    FS_LOGI("time sync publisher stopped (fds preserved)");
    return FS_SUCCESS;
}

int mcu_time_sync_pub_poll_once(void)
{
    return poll_gpio_trig_and_frame_sync_once();
}

void mcu_time_sync_pub_set_soc_state(uint8_t soc_sync_state)
{
    g_soc_sync_state = soc_sync_state;
    if (soc_sync_state != SOC_SYNC_STATE_LOCKED) {
        spad_anchor_init();
    }
}

void mcu_time_sync_pub_deinit(void)
{
    mcu_time_sync_pub_stop();
    if (g_frame_sync_fd >= 0) {
        close(g_frame_sync_fd);
        g_frame_sync_fd = -1;
    }
    if (g_trig_fd >= 0) {
        close(g_trig_fd);
        g_trig_fd = -1;
    }
}
