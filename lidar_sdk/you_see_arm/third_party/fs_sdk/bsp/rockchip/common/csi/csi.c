#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <getopt.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <assert.h>

#include <linux/types.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <pthread.h>

#include "csi.h"
#ifdef SOC_T527
static csi_info_t csi_info[2] = {
    {
        .vi = 8,
        .fd = -1,
        .width = 0,
        .height = 0,
        .nplanes = 0,
        .nbuffers = CAPTURE_FRAME_NUM,
        .buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        .buffers = NULL,
    },
    {
        .vi = 12,
        .fd = -1,
        .width = 0,
        .height = 0,
        .nplanes = 0,
        .nbuffers = CAPTURE_FRAME_NUM,
        .buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        .buffers = NULL,
    }
};
#else
static csi_info_t csi_info[1] = {
    {
        .vi = 0,
        .fd = -1,
        .subdev_fd = -1,
        .width = 0,
        .height = 0,
        .nplanes = 0,
        .nbuffers = CAPTURE_FRAME_NUM,
        .buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        .buffers = NULL,
    },
};
#endif
int csi_initialize(int vi, uint32_t width, uint32_t height, uint32_t pix_fmt)
{
    csi_info_t* csi;

    if (vi < 0 || vi > 1) {
        printf("csi_initialize: invalid vi %d\n", vi);
        return -1;
    }

    csi = &csi_info[vi];
    csi->width = width;
    csi->height = height;

    if (csi_interface_init(csi) < 0) {
        printf("csi_interface_init failed for vi %d\n", csi->vi);
        return -1;
    }

    if (csi_interface_fmt_set(csi, width, height, pix_fmt) < 0) {
        printf("csi_interface_init failed for vi %d\n", csi->vi);
        return -1;
    }

    if (csi_interface_request_buffers(csi) < 0) {
        printf("csi_interface_request_buffers failed for vi %d\n", csi->vi);
        csi_interface_deinit(csi);
        return -1;
    }

    if (csi_interface_stream_enable(csi, true) < 0) {
        printf("csi_interface_stream_enable failed for vi %d\n", csi->vi);
        csi_interface_free_buffers(csi);
        csi_interface_deinit(csi);
        return -1;
    }

    return 0;
}

int csi_deinit(int vi)
{
    if (vi < 0 || vi > 1) {
        printf("csi_deinit: invalid vi %d\n", vi);
        return -1;
    }

    if (csi_info[vi].fd >= 0) {
        csi_interface_deinit(&csi_info[vi]);
    }

    return 0;
}

int csi_stream_monitor(int vi, uint32_t timeout)
{
    csi_info_t* csi;

    if (vi < 0 || vi > 1) {
        printf("csi_stream_monitor: invalid vi %d\n", vi);
        return -1;
    }

    csi = &csi_info[vi];
    return csi_interface_stream_monitor(csi, timeout);
}

int csi_read_frame(int vi, csi_buffers_t* frame)
{
    csi_info_t* csi;

    if (vi < 0 || vi > 1) {
        printf("csi_read_frame: invalid vi %d\n", vi);
        return -1;
    }

    csi = &csi_info[vi];
    return csi_interface_read_frame(csi, frame);
}

int csi_release_frame(csi_buffers_t* frame)
{
    return csi_interface_release_frame(frame);
}

int csi_stream_enable(int vi, bool en)
{
    if (vi < 0 || vi > 1) {
        printf("csi_stream_enable: invalid vi %d\n", vi);
        return -1;
    }

    if (csi_info[vi].fd < 0) {
        // 未初始化或已关闭，无需操作
        return 0;
    }

    return csi_interface_stream_enable(&csi_info[vi], en);
}
