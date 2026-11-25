/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _FS_FRAME_SYNC_IOCTL_H
#define _FS_FRAME_SYNC_IOCTL_H

#include <stdint.h>
#include <sys/ioctl.h>

#define FS_FRAME_SYNC_DEVICE_PATH "/dev/fs_frame_sync"
#define FS_FRAME_SYNC_IOCTL_MAGIC 'F'

#define FS_FRAME_SYNC_EVENT_F_OVERFLOW   (1U << 0)
#define FS_FRAME_SYNC_EVENT_F_SEQ_GAP    (1U << 1)
#define FS_FRAME_SYNC_EVENT_F_IRQ_MISSED (1U << 2)

struct fs_frame_sync_event {
    uint64_t t_frame_sync_ns;
    uint32_t fsync_seq;
    uint32_t flags;
    uint32_t level;
    uint32_t reserved[4];
};

#define FS_FRAME_SYNC_IOCTL_START _IO(FS_FRAME_SYNC_IOCTL_MAGIC, 1)
#define FS_FRAME_SYNC_IOCTL_STOP  _IO(FS_FRAME_SYNC_IOCTL_MAGIC, 2)

#endif /* _FS_FRAME_SYNC_IOCTL_H */
