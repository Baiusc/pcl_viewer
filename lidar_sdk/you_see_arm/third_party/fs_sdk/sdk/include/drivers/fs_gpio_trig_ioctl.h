/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * fs_gpio_trig_ioctl.h – userspace-visible UAPI for the fs_gpio_trig driver.
 *
 * Kernel-canonical path: include/uapi/linux/fs_gpio_trig_ioctl.h
 * This copy is kept in sync for soc-app consumption.
 *
 */
#ifndef _FS_GPIO_TRIG_IOCTL_H
#define _FS_GPIO_TRIG_IOCTL_H

#include <stdint.h>
#include <sys/ioctl.h>

#define FS_GPIO_TRIG_DEVICE_PATH  "/dev/fs_gpio_trig"
#define FS_GPIO_TRIG_IOCTL_MAGIC  'T'

/* Timestamp sample point. Product default and required app setting: BEFORE. */
#define FS_GPIO_TRIG_TS_BEFORE  0  /* sampled immediately before GPIO asserts HIGH */
#define FS_GPIO_TRIG_TS_AFTER   1  /* sampled immediately after GPIO de-asserts LOW */

/* Event flags. Existing apps should still treat seq gaps as source of truth. */
#define FS_GPIO_TRIG_EVENT_F_OVERFLOW  (1U << 0)

/*
 * Idle level is fixed LOW.  On each hrtimer tick the GPIO is driven HIGH
 * for pulse_high_ms milliseconds, then returned LOW.
 *
 * Constraint: pulse_high_ms must be strictly less than period_ms.
 * Struct size is 32 bytes; reserved[] pads to keep size stable.
 */
struct fs_gpio_trig_config {
    uint32_t period_ms;      /* trigger period in ms,         [1, 10000]           */
    uint32_t pulse_high_ms;  /* high pulse width in ms,       [1, period_ms - 1]   */
    uint32_t ts_point;       /* see FS_GPIO_TRIG_TS_*                               */
    uint32_t reserved[5];    /* must be zero                                        */
};

/*
 * One event is produced per emitted trigger edge.
 *
 * t_trig_ns is CLOCK_REALTIME ns at the configured timestamp point.
 * seq is kernel-owned and resets on START; first delivered event is 1.
 * seq 0 is reserved and must never be delivered as a valid event.
 */
struct fs_gpio_trig_event {
    uint64_t t_trig_ns;
    uint32_t seq;
    uint32_t flags;
};

#define FS_GPIO_TRIG_IOCTL_SET_CONFIG \
    _IOW(FS_GPIO_TRIG_IOCTL_MAGIC, 1, struct fs_gpio_trig_config)
#define FS_GPIO_TRIG_IOCTL_GET_CONFIG \
    _IOR(FS_GPIO_TRIG_IOCTL_MAGIC, 2, struct fs_gpio_trig_config)
#define FS_GPIO_TRIG_IOCTL_START  _IO(FS_GPIO_TRIG_IOCTL_MAGIC, 3)
#define FS_GPIO_TRIG_IOCTL_STOP   _IO(FS_GPIO_TRIG_IOCTL_MAGIC, 4)

#endif /* _FS_GPIO_TRIG_IOCTL_H */
