#ifndef FS_TIMESYNC_CTRL_H
#define FS_TIMESYNC_CTRL_H

#include <stdint.h>

#ifndef FS_SOC_TIMESYNC_CTRL_TOPIC
#define FS_SOC_TIMESYNC_CTRL_TOPIC 0x21u
#endif

typedef enum {
    FS_TIMESYNC_CTRL_DISABLE = 0,
    FS_TIMESYNC_CTRL_ENABLE_POINTCLOUD = 1,
} fs_timesync_ctrl_t;

#pragma pack(push)
#pragma pack(1)
typedef struct {
    uint8_t topic;
    uint8_t ctrl;
    uint8_t reserved[2];
} fs_timesync_ctrl_payload_t;
#pragma pack(pop)

#endif /* FS_TIMESYNC_CTRL_H */
