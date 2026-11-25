#ifndef FS_POINTCLOUD_TIMESYNC_CAPS_H
#define FS_POINTCLOUD_TIMESYNC_CAPS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FS_PCL_TRIGGER_EXTERNAL = 0,
    FS_PCL_TRIGGER_INTERNAL = 1,
} fs_pcl_trigger_mode_t;

typedef struct {
    bool    pointcloud_timesync_supported;
    bool    pointcloud_true_fsync_anchor_supported;
    int64_t trigger_to_spad_fsync_delay_ns;
    fs_pcl_trigger_mode_t trigger_mode;
} fs_pointcloud_timesync_caps_t;

#ifdef __cplusplus
}
#endif

#endif /* FS_POINTCLOUD_TIMESYNC_CAPS_H */
