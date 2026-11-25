#ifndef __FS_CSI_INTERFACE_H__
#define __FS_CSI_INTERFACE_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <pthread.h>

// #ifdef SOC_T527
// #include "sunxiMemInterface.h"
// #endif

#ifdef __cplusplus
extern "C" {
#endif

#define VIN_NPLANES_NUM         3
#define CAPTURE_FRAME_NUM       (2 * 3)
#define CAPTURE_RATE_FPS        30

typedef struct csi_buffers {
    void *start[VIN_NPLANES_NUM];
    uint32_t length[VIN_NPLANES_NUM];
    uint32_t nplanes;
    struct v4l2_buffer *v4l_buf;
    int video_fd;
// // #ifdef SOC_T527
//     dma_mem_des_t vbuf;
//     void *phy_addr[VIN_NPLANES_NUM];
//     unsigned int fd[VIN_NPLANES_NUM];
//     volatile struct csi_buffers *buffers;
// // #endif
} csi_buffers_t;

typedef struct {
    int vi;
    int fd;
    int subdev_fd;
    uint32_t width;
    uint32_t height;
    uint32_t nplanes;
    uint32_t nbuffers;
    uint32_t buffer_type;
// #ifdef SOC_T527
    // dma_mem_des_t mem;
// #endif
    volatile csi_buffers_t *buffers;
    pthread_mutex_t buffers_lock; // 保护 buffers 的并发访问
    char subdev_path[64];
} csi_info_t;

int csi_interface_init(csi_info_t *csi);
int csi_interface_deinit(csi_info_t *csi);
int csi_interface_fmt_set(csi_info_t *csi, uint32_t width, uint32_t height, uint32_t pix_fmt);
int csi_interface_request_buffers(csi_info_t *csi);
int csi_interface_free_buffers(csi_info_t *csi);
int csi_interface_stream_enable(csi_info_t *csi, bool enable);
int csi_interface_stream_monitor(csi_info_t *csi, uint32_t timeout);
int csi_interface_read_frame(csi_info_t *csi, csi_buffers_t *frame);
int csi_interface_release_frame(csi_buffers_t* f);

#ifdef __cplusplus
}
#endif

#endif // __FS_CSI_INTERFACE_H__
