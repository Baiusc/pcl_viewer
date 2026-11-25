#ifndef __FS_CSI_H__
#define __FS_CSI_H__

#include <stdint.h>
#include <stdbool.h>
#include "csi_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

int csi_initialize(int vi, uint32_t width, uint32_t height, uint32_t pix_fmt);
int csi_deinit(int vi);
int csi_stream_monitor(int vi, uint32_t timeout);
int csi_read_frame(int vi, csi_buffers_t* frame);
int csi_release_frame(csi_buffers_t* frame);

// 控制指定通道的 CSI 流开关，用于在停止接收前快速解除阻塞
int csi_stream_enable(int vi, bool enable);

#ifdef __cplusplus
}
#endif

#endif // __FS_CSI_H__
