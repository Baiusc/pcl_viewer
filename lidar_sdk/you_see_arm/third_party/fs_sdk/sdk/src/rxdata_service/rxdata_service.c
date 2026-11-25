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

#include "fs_err.h"
#include "fs_util.h"
#include "csi.h"
#include "drivers/serdes/serdes.h"
#include "rxdata_service.h"
#include "algo.h"

typedef struct {
    int vi;
    int ready;
    pthread_t thread_id;
    uint32_t width;
    uint32_t height;
    uint32_t pix_fmt;
} rxdata_service_info_t;

static int rxdata_service_run_flag = 1;
static rxdata_cfg_t rxdata_cfg = {0};
static int rxdata_cfg_flag = 0;
static int rxdata_service_init_flag = 0;  // 追踪初始化状态
static rxdata_service_info_t rxdata_info_vc0;
static rxdata_service_info_t rxdata_info_vc1;

int rxdata_service_release_frame(void* frame)
{
    if (!frame) {
        return FS_ERR_BAD_PARAMS;
    }

    return csi_release_frame((csi_buffers_t *)frame);
}

static void *rxdata_service_run(void *arg)
{

    rxdata_service_info_t *info = (rxdata_service_info_t *)arg;
    int ret;
    int vi = info->vi;

    // 初始化CSI并开始接收数据
    if (csi_initialize(vi, info->width, info->height, V4L2_PIX_FMT_SBGGR12) < 0) {
        printf("RX Data Service: CSI initialization failed for vi %d\n", vi);
        info->ready = -1;  // 标记失败，避免主线程死锁
        return NULL;
    }

    info->ready = 1;

    while (rxdata_service_run_flag) {
        // 处理接收的数据
        ret = csi_stream_monitor(vi, 2000);
        if (ret < 0) {
            printf("RX Data Service: Stream monitor error for vi %d\n", vi);
            continue;
        } else if (ret == 0) {
            // 超时无数据
            // printf("RX Data Service: Stream monitor timeout for vi %d\n", vi);
            continue;
        }

        // 再次检查运行标志，避免停流后继续读取
        if (!rxdata_service_run_flag) {
            break;
        }

        csi_buffers_t frame;
        if (csi_read_frame(vi, &frame) < 0) {
            printf("RX Data Service: Failed to read frame for vi %d\n", vi);
            continue;
        }
        // 将接收到的数据放入队列
        algo_data_queue_t *data_queue = (algo_data_queue_t *)malloc(sizeof(algo_data_queue_t));
        if (data_queue) {
            data_queue->frame = malloc(sizeof(csi_buffers_t));

            if (!data_queue->frame) {
                free(data_queue);
                continue;
            }

            data_queue->data_len = frame.length[0];
            data_queue->data = (uint8_t*)frame.start[0];
            memcpy(data_queue->frame, &frame, sizeof(csi_buffers_t));
            /* Point-cloud frame time is resolved later from spad_anchor by frame_id. */
            data_queue->timestamp = fs_timestamp_us();
            data_queue->vi = vi;

            data_queue->timestamp = data_queue->timestamp - 98 * 1000; // 时间戳兜底方案
            if (algo_queue_push_data(data_queue) < 0) {
                printf("RX Data Service: Failed to push data to algo queue for vi %d\n", vi);
                free(data_queue->frame);
                free(data_queue);
            }
        } else {
            printf("RX Data Service: Failed to allocate memory for algo data queue for vi %d\n", vi);
        }

        // 注意：不要在这里释放frame，应该在处理完数据后由调用者释放
    }

    // 注意：不在此处清理 CSI，因为算法线程可能还在处理队列中的帧数据（指向 CSI buffers）
    // CSI 资源由主线程在 join 后统一清理

    return NULL;
}

int rxdata_service_set_cfg(bool is_double_vc, uint32_t vc0_w, uint32_t vc0_h, uint32_t vc1_w, uint32_t vc1_h)
{
    rxdata_cfg.is_doubel_vc = is_double_vc;
    rxdata_cfg.vc0_w = vc0_w;
    rxdata_cfg.vc0_h = vc0_h;
    rxdata_cfg.vc1_w = vc1_w;
    rxdata_cfg.vc1_h = vc1_h;

    rxdata_cfg_flag = 1;

    return FS_SUCCESS;
}

int rxdata_service_init(const rx_parse_cfg_t *rx_parse)
{
    bool is_double_vc = false;
    int32_t ret;

    // 每次初始化前确保运行标志为开启
    rxdata_service_run_flag = 1;
    rxdata_service_set_cfg(rx_parse->is_ab_frame, rx_parse->frm_cfg[0].width, rx_parse->frm_cfg[0].height,
                           rx_parse->frm_cfg[1].width, rx_parse->frm_cfg[1].height);

    if (rxdata_cfg_flag == 0) {
        FS_LOGE("cfg not set");
        return FS_ERR_INTERNAL;
    }

    memset(&rxdata_info_vc0, 0, sizeof(rxdata_service_info_t));
    memset(&rxdata_info_vc1, 0, sizeof(rxdata_service_info_t));

    rxdata_info_vc0.vi = 0;
    rxdata_info_vc0.pix_fmt = V4L2_PIX_FMT_SBGGR12;
    rxdata_info_vc0.width = rxdata_cfg.vc0_w;
    rxdata_info_vc0.height = rxdata_cfg.vc0_h;

    rxdata_info_vc1.vi = 1;
    rxdata_info_vc1.pix_fmt = V4L2_PIX_FMT_SBGGR12;
    rxdata_info_vc1.width = rxdata_cfg.vc1_w;
    rxdata_info_vc1.height = rxdata_cfg.vc1_h;

    is_double_vc = rxdata_cfg.is_doubel_vc;

    pthread_create(&rxdata_info_vc0.thread_id, NULL, rxdata_service_run, &rxdata_info_vc0);

    // 如果是AB帧模式，则启动第二个接收线程
    if (is_double_vc) {
        pthread_create(&rxdata_info_vc1.thread_id, NULL, rxdata_service_run, &rxdata_info_vc1);
    }

    if (is_double_vc) {
        while (!rxdata_info_vc0.ready || !rxdata_info_vc1.ready) {
            // 检测初始化失败
            if (rxdata_info_vc0.ready < 0 || rxdata_info_vc1.ready < 0) {
                FS_LOGE("CSI initialization failed in worker thread");
                rxdata_service_run_flag = 0;
                if (rxdata_info_vc0.thread_id) pthread_join(rxdata_info_vc0.thread_id, NULL);
                if (rxdata_info_vc1.thread_id) pthread_join(rxdata_info_vc1.thread_id, NULL);
                return FS_ERR_INTERNAL;
            }
            usleep(10000);
        }
    } else {
        while (!rxdata_info_vc0.ready) {
            if (rxdata_info_vc0.ready < 0) {
                FS_LOGE("CSI initialization failed in worker thread");
                rxdata_service_run_flag = 0;
                pthread_join(rxdata_info_vc0.thread_id, NULL);
                return FS_ERR_INTERNAL;
            }
            usleep(10000);
        }
    }

    rxdata_service_init_flag = 1;  // 标记初始化成功

    return 0;
}

int rxdata_service_deinit(void)
{
    // 仅在成功初始化时才进行清理
    if (!rxdata_service_init_flag) {
        return 0;
    }

    rxdata_service_run_flag = 0;

    // 优先关闭视频流以解除线程中 select/DQBUF 的阻塞
    if (rxdata_info_vc0.vi >= 0) {
        csi_stream_enable(rxdata_info_vc0.vi, false);
    }
    if (rxdata_cfg.is_doubel_vc && rxdata_info_vc1.vi >= 0) {
        csi_stream_enable(rxdata_info_vc1.vi, false);
    }

    // 等待工作线程退出
    pthread_t self = pthread_self();
    if (rxdata_info_vc0.thread_id && !pthread_equal(rxdata_info_vc0.thread_id, self)) {
        pthread_join(rxdata_info_vc0.thread_id, NULL);
        rxdata_info_vc0.thread_id = 0;
    }
    if (rxdata_cfg.is_doubel_vc && rxdata_info_vc1.thread_id && !pthread_equal(rxdata_info_vc1.thread_id, self)) {
        pthread_join(rxdata_info_vc1.thread_id, NULL);
        rxdata_info_vc1.thread_id = 0;
    }

    // 所有工作线程已退出，现在可以关闭流（如需重新初始化）
    // 但不清理 CSI 资源，因为算法线程可能还在处理队列中的帧数据（指向 CSI buffers）
    // CSI 资源需要在 algo_deinit() 之后由外部显式清理

    rxdata_service_init_flag = 0;

    return 0;
}

int rxdata_service_cleanup_csi(void)
{
    // 单独的 CSI 清理函数，必须在 algo_deinit() 之后调用
    // 以确保没有线程再访问 CSI buffers
    if (rxdata_info_vc0.vi >= 0) {
        csi_deinit(rxdata_info_vc0.vi);
    }
    if (rxdata_cfg.is_doubel_vc && rxdata_info_vc1.vi >= 0) {
        csi_deinit(rxdata_info_vc1.vi);
    }

    return 0;
}

int rxdata_service_stop(void)
{
    // 直接复用 deinit 逻辑以保证资源释放
    return rxdata_service_deinit();
}

int rxdata_service_restart(const rx_parse_cfg_t *rx_parse)
{
    int ret = rxdata_service_stop();
    if (ret != FS_SUCCESS) {
        return ret;
    }

    return rxdata_service_init(rx_parse);
}
