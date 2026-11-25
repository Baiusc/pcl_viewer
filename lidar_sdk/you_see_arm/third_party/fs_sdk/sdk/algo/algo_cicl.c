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
#include <pthread.h>
#include <time.h>
#include <linux/types.h>
#include <sys/mman.h>

#include "fs_err.h"
#include "fs_util.h"
#include "fs_version.h"
#include "fs_file.h"
#include "algo.h"
#include "algo_cicl.h"
#include "Lidar_FL6x32_algo.h"
#include "fs_sdk_api.h"
#include "mcu_protocol.h"
#include "concurrent_queue.h"
#include "spad_time_anchor.h"
#include "time_sync_eval.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "AlgoCicl"

#define ALGO_THREAD_CNT       4   // 0: 纯单线程接口; 1~4: 多线程接口的线程数
#define ALGO_PCL_DOUBLE_ECHO  0   // 点云是否输出双回波: 0 - 否，1 - 是

#define RANGE_ONE_PIXEL_INFO_LEN(peak_num) (sizeof(FL6x32_range_pix_info_t) + (peak_num) * sizeof(FL6x32_algo_result_t))
#define CICL_LIDAR_FRAME_ID_MAX 65535u

#pragma pack(push)
#pragma pack(1) // one byte-aligned

typedef struct fs_pcl_xyz_info {
    uint16_t distance;     /* 当前回波的距离值    : 单位 - 1/N cm, N 值取决于点云数据单位 */
    int16_t  x;            /* 当前回波的空间坐标 X: 单位 - 1/N cm, N 值取决于点云数据单位 */
    int16_t  y;            /* 当前回波的空间坐标 Y: 单位 - 1/N cm, N 值取决于点云数据单位 */
    int16_t  z;            /* 当前回波的空间坐标 Z: 单位 - 1/N cm, N 值取决于点云数据单位 */
    uint8_t  reflectivity; /* 反射率 */
    uint8_t  tag_label;    /* target label, 保留，跟点云测量到的实际物体有关，如雨雾、灰尘等*/
} fs_pcl_xyz_info_t;

typedef struct fs_pcl_dist_xyz_info {
    uint32_t timestamp_offset; // 当前 pixel 相对于帧同步时刻(timestamp_base)的时间戳 offset
    uint16_t col;
    uint16_t row;
    fs_pcl_xyz_info_t pcl_info[0]; /* 包含距离、空间坐标和反射率等信息 */
} fs_pcl_dist_xyz_info_t;

#pragma pack(pop)

typedef struct {
    bool     algo_init;
    bool     algo_env_init;
    uint32_t calib_cnt;  // float 个数
    float   *calib_data;

    struct {
        fs_lidar_data_info_t data_info;
        FL6x32_para          algo_para;

        struct {
            FL6x32_range_pix_info_t  *range_info;
            FL6x32_range_pix_info_t  *range_buf[2];
            uint32_t                 *timestamp_info;
            uint32_t                 *timestamp_buf[2];
            FL6x32_algo_pcl_output_t *algo_pcl;
            FL6x32_algo_pcl_output_t *algo_pcl_thread_buf[ALGO_THREAD_CNT];
            fs_pcl_dist_xyz_info_t   *fs_pcl;
            uint8_t                   write_buf_idx;   // 当前写入缓冲区索引
            uint8_t                   buf_in_use[2];   // 缓冲区是否正在被 PCL 线程使用
            uint32_t                  range_data_len;  // 本 ROI 固定缓冲区大小（init 时计算）
            uint16_t                  updated_frame_idx; // 最近一次更新该 ROI 的帧号
            uint8_t                   updated_sub_frame_idx; // 最近一次更新该 ROI 的子帧索引
            uint16_t                  enqueued_frame_idx; // 最近一次入队的帧号(去重)
            uint8_t                   enqueued_sub_frame_idx; // 最近一次入队的子帧索引(去重)
        } ROI_ctx[4];
    } ab_ctx[2];
} algo_pcl_ctx_t;

static algo_pcl_ctx_t g_algo_pcl_ctx;

static lidar_frm_info_t cicl_lidar_frm_info = {
    .cur_lidar_frm_idx  = 1,
    .cur_mipi_frm_idx   = 0,
    .lidar_frm_ready    = false,
    .find_start_frm     = false,
    .pixel_count        = 0,
    .pixel_len          = 0,
    .total_len          = 0,
};

// static int cicl_lidar_frm_id = 0;
// PCL异步处理线程相关
typedef struct {
    uint8_t *range_data;        // Range原始数据副本
    uint32_t range_data_len;    // 数据长度
    uint8_t  range_buf_idx;     // 指向 ROI 双缓冲中的哪一个
    uint16_t frame_index;       // 帧索引
    uint8_t  ab_frame_idx;      // AB 帧标记
    uint8_t  max_sub_frame_cnt; // 最大子帧数
    uint8_t  roi_index;         // 当前数据所属的 roi 区域索引值
    uint16_t vir_group_id;      // 虚拟 Group ID
    uint16_t phy_group_id;      // 物理 Group ID
    uint8_t  sub_frame_cnt;     // 子帧数
    uint8_t  sub_frame_idx;     // 子帧索引
    uint16_t group_cnt;
    uint64_t frame_base_timestamp;
} pcl_process_task_t;

static pthread_t g_pcl_thread_id = 0;
static volatile uint8_t g_pcl_thread_run_flag = 0;
static ConcurrentQueue *g_pcl_task_queue = NULL;

static uint32_t g_pixel_timestamp_offset_data[4][144][192] = {0}; // 最多支持4个ROI，每个ROI最大5120个像素点(80*64)，每个像素点一个时间戳偏移值

#define PCL_TASK_QUEUE_SIZE  8   // 最多缓存8帧待处理数据

#define LIDAR_CALIB_DATA_PATH "/data/fslidar/storage/calib_data.bin"


typedef struct {
    bool            initialized;
    bool            stop_requested;
    bool            job_ready;
    uint32_t        job_seq;
    uint32_t        completed_cnt;
    int32_t         job_ret;
    int             per_thread_output_len[ALGO_THREAD_CNT];
    pthread_t       worker_thread_ids[ALGO_THREAD_CNT];
    pthread_mutex_t lock;
    pthread_cond_t  job_cond;
    pthread_cond_t  done_cond;
    struct {
        uint32_t seq;
        int32_t  ab_frame_idx;
        uint8_t  roi_index;
        FL6x32_para *algo_para;
        uint8_t  *range_data;
        uint32_t  range_data_len;
    } job;
} algo_mt_pool_t;

static algo_mt_pool_t g_algo_mt_pool = {0};
static bool g_pcl_merge = true; // 是否合并同一帧的多个 ROI 数据后再发送给上层，默认合并 当前逻辑只适用于 2个ROI区域且有重叠区域的情况

/* 任务处理完成后释放任务对象并归还双缓冲状态 */
static inline void pcl_task_release(pcl_process_task_t *task)
{
    if (task) {
        if (task->ab_frame_idx < 2 && task->roi_index < 4 && task->range_buf_idx < 2) {
            __atomic_store_n(
                &g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].ROI_ctx[task->roi_index].buf_in_use[task->range_buf_idx],
                0,
                __ATOMIC_RELEASE);
        }
        free(task);
    }
}

static void *algo_mt_worker_thread(void *arg)
{
#if (ALGO_THREAD_CNT == 0)
    (void)arg;
    return NULL;
#else
    const unsigned short multithread_idx = (unsigned short)(uintptr_t)arg;
    uint32_t seen_seq = 0;

    while (true) {
        int32_t ret;
        int output_len = 0;
        int32_t ab_frame_idx;
        uint8_t roi_index;
        FL6x32_para *algo_para;
        uint8_t *range_data;
        uint32_t range_data_len;
        FL6x32_algo_pcl_output_t *output_cloud_data;

        pthread_mutex_lock(&g_algo_mt_pool.lock);
        while (!g_algo_mt_pool.stop_requested && (!g_algo_mt_pool.job_ready || g_algo_mt_pool.job.seq == seen_seq)) {
            pthread_cond_wait(&g_algo_mt_pool.job_cond, &g_algo_mt_pool.lock);
        }

        if (g_algo_mt_pool.stop_requested) {
            pthread_mutex_unlock(&g_algo_mt_pool.lock);
            break;
        }

        seen_seq = g_algo_mt_pool.job.seq;
        ab_frame_idx = g_algo_mt_pool.job.ab_frame_idx;
        roi_index = g_algo_mt_pool.job.roi_index;
        algo_para = g_algo_mt_pool.job.algo_para;
        range_data = g_algo_mt_pool.job.range_data;
        range_data_len = g_algo_mt_pool.job.range_data_len;
        output_cloud_data = g_algo_pcl_ctx.ab_ctx[ab_frame_idx].ROI_ctx[roi_index].algo_pcl_thread_buf[multithread_idx];
        pthread_mutex_unlock(&g_algo_mt_pool.lock);

        ret = Algo_Get_CloudData_multithread(algo_para,
                                             multithread_idx,
                                             range_data,
                                             range_data_len,
                                             (float *)output_cloud_data,
                                             &output_len);

        pthread_mutex_lock(&g_algo_mt_pool.lock);
        if (ret != FS_SUCCESS && g_algo_mt_pool.job_ret == FS_SUCCESS) {
            g_algo_mt_pool.job_ret = ret;
        }
        if (output_len < 0) {
            output_len = 0;
            if (g_algo_mt_pool.job_ret == FS_SUCCESS) {
                g_algo_mt_pool.job_ret = FS_ERR_INTERNAL;
            }
        }
        g_algo_mt_pool.per_thread_output_len[multithread_idx] = output_len;
        g_algo_mt_pool.completed_cnt++;
        if (g_algo_mt_pool.completed_cnt >= ALGO_THREAD_CNT) {
            pthread_cond_signal(&g_algo_mt_pool.done_cond);
        }
        pthread_mutex_unlock(&g_algo_mt_pool.lock);
    }

    return NULL;
#endif
}

static int32_t algo_mt_start_workers(void)
{
#if (ALGO_THREAD_CNT == 0)
    return FS_SUCCESS;
#else
    int i;

    if (g_algo_mt_pool.initialized) {
        return FS_SUCCESS;
    }

    if (pthread_mutex_init(&g_algo_mt_pool.lock, NULL) != 0) {
        FS_LOGE("Failed to init PCL multithread mutex");
        return FS_ERR_INTERNAL;
    }

    if (pthread_cond_init(&g_algo_mt_pool.job_cond, NULL) != 0) {
        FS_LOGE("Failed to init PCL multithread job cond");
        pthread_mutex_destroy(&g_algo_mt_pool.lock);
        return FS_ERR_INTERNAL;
    }

    if (pthread_cond_init(&g_algo_mt_pool.done_cond, NULL) != 0) {
        FS_LOGE("Failed to init PCL multithread done cond");
        pthread_cond_destroy(&g_algo_mt_pool.job_cond);
        pthread_mutex_destroy(&g_algo_mt_pool.lock);
        return FS_ERR_INTERNAL;
    }

    g_algo_mt_pool.stop_requested = false;
    g_algo_mt_pool.job_ready = false;
    g_algo_mt_pool.job_seq = 0;
    g_algo_mt_pool.completed_cnt = 0;
    g_algo_mt_pool.job_ret = FS_SUCCESS;

    for (i = 0; i < ALGO_THREAD_CNT; i++) {
        if (pthread_create(&g_algo_mt_pool.worker_thread_ids[i], NULL, algo_mt_worker_thread, (void *)(uintptr_t)i) != 0) {
            FS_LOGE("Failed to create persistent algo worker %d", i);
            g_algo_mt_pool.stop_requested = true;
            pthread_cond_broadcast(&g_algo_mt_pool.job_cond);
            while (i-- > 0) {
                pthread_join(g_algo_mt_pool.worker_thread_ids[i], NULL);
            }
            pthread_cond_destroy(&g_algo_mt_pool.done_cond);
            pthread_cond_destroy(&g_algo_mt_pool.job_cond);
            pthread_mutex_destroy(&g_algo_mt_pool.lock);
            memset(&g_algo_mt_pool, 0, sizeof(g_algo_mt_pool));
            return FS_ERR_INTERNAL;
        }
    }

    g_algo_mt_pool.initialized = true;
    FS_LOGI("Persistent PCL multithread pool started with %d workers", ALGO_THREAD_CNT);
    return FS_SUCCESS;
#endif
}

static void algo_mt_stop_workers(void)
{
#if (ALGO_THREAD_CNT == 0)
    return;
#else
    int i;

    if (!g_algo_mt_pool.initialized) {
        return;
    }

    pthread_mutex_lock(&g_algo_mt_pool.lock);
    g_algo_mt_pool.stop_requested = true;
    pthread_cond_broadcast(&g_algo_mt_pool.job_cond);
    pthread_cond_broadcast(&g_algo_mt_pool.done_cond);
    pthread_mutex_unlock(&g_algo_mt_pool.lock);

    for (i = 0; i < ALGO_THREAD_CNT; i++) {
        pthread_join(g_algo_mt_pool.worker_thread_ids[i], NULL);
    }

    pthread_cond_destroy(&g_algo_mt_pool.done_cond);
    pthread_cond_destroy(&g_algo_mt_pool.job_cond);
    pthread_mutex_destroy(&g_algo_mt_pool.lock);
    memset(&g_algo_mt_pool, 0, sizeof(g_algo_mt_pool));
    FS_LOGI("Persistent PCL multithread pool stopped");
#endif
}

static int32_t algo_mt_process_cloud_data(int32_t ab_frame_idx,
                                          uint8_t roi_index,
                                          FL6x32_para *algo_para,
                                          uint8_t *range_data,
                                          uint32_t range_data_len,
                                          float *output_cloud_data,
                                          int *output_data_length)
{
#if (ALGO_THREAD_CNT == 0)
    (void)ab_frame_idx;
    (void)roi_index;
    (void)algo_para;
    (void)range_data;
    (void)range_data_len;
    (void)output_cloud_data;
    (void)output_data_length;
    return FS_ERR_BAD_PARAMS;
#else
    int i;
    int32_t ret;
    uint32_t total_output_len = 0;

    (void)output_cloud_data;

    CHECK_PTR(algo_para);
    CHECK_PTR(range_data);
    CHECK_PTR(output_data_length);

    if (!g_algo_mt_pool.initialized) {
        FS_LOGE("Persistent multithread pool is not initialized");
        return FS_ERR_BAD_PARAMS;
    }

    pthread_mutex_lock(&g_algo_mt_pool.lock);
    g_algo_mt_pool.job.seq++;
    g_algo_mt_pool.job.ab_frame_idx = ab_frame_idx;
    g_algo_mt_pool.job.roi_index = roi_index;
    g_algo_mt_pool.job.algo_para = algo_para;
    g_algo_mt_pool.job.range_data = range_data;
    g_algo_mt_pool.job.range_data_len = range_data_len;
    g_algo_mt_pool.completed_cnt = 0;
    g_algo_mt_pool.job_ret = FS_SUCCESS;
    *output_data_length = 0;
    for (i = 0; i < ALGO_THREAD_CNT; i++) {
        g_algo_mt_pool.per_thread_output_len[i] = 0;
    }
    g_algo_mt_pool.job_ready = true;
    pthread_cond_broadcast(&g_algo_mt_pool.job_cond);

    while (!g_algo_mt_pool.stop_requested && g_algo_mt_pool.completed_cnt < ALGO_THREAD_CNT) {
        pthread_cond_wait(&g_algo_mt_pool.done_cond, &g_algo_mt_pool.lock);
    }

    ret = g_algo_mt_pool.job_ret;
    if (ret == FS_SUCCESS) {
        for (i = 0; i < ALGO_THREAD_CNT; i++) {
            int thread_output_len = g_algo_mt_pool.per_thread_output_len[i];
            if (thread_output_len > 0) {
                total_output_len += (uint32_t)thread_output_len;
            }
        }
        *output_data_length = (int)total_output_len;
    }

    g_algo_mt_pool.job_ready = false;
    pthread_mutex_unlock(&g_algo_mt_pool.lock);

    return ret;
#endif
}

static int32_t load_calib_data_from_mcu(float **calib_data, uint32_t *calib_cnt)
{
    int32_t ret;
    uint8_t *raw_data = NULL;
    uint32_t raw_len = 0u;
    uint32_t float_count;

    if (calib_data == NULL || calib_cnt == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    ret = mcu_prot_recv_flash_data(MCU_PROT_FLASH_DT_CALI, &raw_data, &raw_len);
    if (ret != FS_SUCCESS) {
        FS_LOGW("MCU fallback read CALI failed: ret=%d", ret);
        return ret;
    }

    if (raw_data == NULL || raw_len == 0u || (raw_len % sizeof(float)) != 0u) {
        FS_LOGW("MCU fallback CALI invalid length: %u", raw_len);
        free(raw_data);
        return FS_ERR_BAD_PARAMS;
    }

    float_count = raw_len / (uint32_t)sizeof(float);

    if (*calib_data != NULL) {
        free(*calib_data);
        *calib_data = NULL;
        *calib_cnt = 0u;
    }

    *calib_data = (float *)malloc((size_t)float_count * sizeof(float));
    if (*calib_data == NULL) {
        free(raw_data);
        return FS_ERR_NOMEM;
    }

    memcpy(*calib_data, raw_data, raw_len);
    *calib_cnt = float_count;
    free(raw_data);

    FS_LOGI("Loaded calib data from MCU fallback: %u floats", *calib_cnt);
    return FS_SUCCESS;
}

static uint64_t cicl_get_system_time_us(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        return 0u;
    }

    return ((uint64_t)ts.tv_sec * 1000000ull)
         + ((uint64_t)ts.tv_nsec / 1000ull);
}


static uint64_t cicl_resolve_frame_timestamp_us(uint32_t frame_id)
{
#if FS_CFG_FEATURE_MCU_TIMESYNC
    uint64_t anchor_ns = 0u;
    int ret = spad_anchor_match_frame(frame_id, &anchor_ns);

    if (ret != FS_SUCCESS) {
        static uint32_t s_last_warn_frame_id = 0u;

        if (frame_id != s_last_warn_frame_id) {
            FS_LOGW("SPAD anchor miss: frame_id=%u, ret=%d", frame_id, ret);
            s_last_warn_frame_id = frame_id;
        }
        time_sync_eval_record_spad_anchor_miss();
        return 0u;
    }

    return anchor_ns / 1000u;
#else
    (void)frame_id;
    return cicl_get_system_time_us();
#endif
}

static uint32_t cicl_next_frame_index(uint32_t frame_id)
{
    return (frame_id >= CICL_LIDAR_FRAME_ID_MAX) ? 1u : (frame_id + 1u);
}

// PCL异步处理线程函数
static void *pcl_process_thread(void *arg)
{
    (void)arg;

    FS_LOGI("PCL process thread started");

    while (g_pcl_thread_run_flag) {
        static uint32_t roi0_pcl_count = 0;
        static uint8_t  roi0_max_sub_frame_cnt = 0;
        static uint16_t roi0_phy_group_id = 0;
        pcl_process_task_t *task = NULL;
        int32_t ret;
        int32_t pix_idx, peak_idx;
        FL6x32_para *algo_para;
        fs_pcl_dist_xyz_info_t *fs_pcl;
        fs_lidar_data_info_t pcl_data_info;
        fs_lidar_data_info_t *data_copy = NULL;
        uint8_t *data_buffer = NULL;
        uint32_t data_size;
        int32_t pcl_count;
        uint32_t point_offset = 0;
        uint8_t peak_num;
        uint8_t pcl_unit_u;
        float   pcl_unit_f;

        // 从队列获取任务（100ms 超时）
        task = (pcl_process_task_t *)queue_pop_timeout(g_pcl_task_queue, 100);

        // 检查线程退出标志
        if (!g_pcl_thread_run_flag) {
            if (task) {
                pcl_task_release(task);
            }
            break;
        }

        if (!task) {
            continue;  // 超时或队列为空，继续等待
        }

        // 检查初始化状态
        if (!g_algo_pcl_ctx.algo_init) {
            FS_LOGE("PCL algo not initialized, dropping frame %u", task->frame_index);
            pcl_task_release(task);
            continue;
        }

        // 检查任务提供的 Range 数据
        if (task->range_data_len == 0 || task->range_data == NULL) {
            FS_LOGE("range_data_len(%u) is 0 of range_data(%p) is NULL", task->range_data_len, task->range_data);
            pcl_task_release(task);
            continue;
        }

        // if (task->roi_index == 0) {
        //     free(task->range_data);
        //     free(task);
        //     continue;
        // }

        // FS_LOGD("PCL sub_frame_idx=%u", task->sub_frame_idx);
        algo_para = &g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].algo_para;
        algo_para->ROI_idx = task->roi_index;
        algo_para->ROI_info[task->roi_index].roll_idx = (task->sub_frame_cnt == 1) ? 0 : task->sub_frame_idx;

        // 调用算法处理
#if (ALGO_THREAD_CNT == 0)
        ret = Algo_GetRawCloudData(algo_para, task->range_data, task->range_data_len, g_algo_pcl_ctx.calib_data, g_algo_pcl_ctx.calib_cnt);
        if (ret != FS_SUCCESS) {
            FS_LOGE("Algo_GetRawCloudData failed: %d, frame=%u, roi_index=%u, range_data_len=%u", ret, task->frame_index, task->roi_index, task->range_data_len);
            pcl_task_release(task);
            continue;
        }
#endif
#if (ALGO_THREAD_CNT == 0)
        ret = Algo_GetOutputCloudData(algo_para, task->range_data, task->range_data_len, (float *)g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].ROI_ctx[task->roi_index].algo_pcl, &pcl_count);
        if (ret != FS_SUCCESS) {
            FS_LOGE("Algo_GetOutputCloudData failed: %d, frame=%u", ret, task->frame_index);
            pcl_task_release(task);
            continue;
        }
#else
        ret = algo_mt_process_cloud_data(task->ab_frame_idx,
                                         task->roi_index,
                                         algo_para,
                                         task->range_data,
                                         task->range_data_len,
                                         (float *)g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].ROI_ctx[task->roi_index].algo_pcl,
                                         &pcl_count);
        if (ret != FS_SUCCESS) {
            FS_LOGE("Algo_Get_CloudData_multithread failed: %d, frame=%u, roi_index=%u", ret, task->frame_index, task->roi_index);
            pcl_task_release(task);
            continue;
        }
#endif

        pcl_count /= (sizeof(FL6x32_algo_pcl_output_t) / sizeof(float));

        // 边界检查：区分“算法输出点数”和“像素数”，避免多峰时按点数遍历像素导致越界
        uint32_t max_pixel_count = algo_para->ROI_info[task->roi_index].col_size * algo_para->ROI_info[task->roi_index].row_size;
        int32_t output_point_count = pcl_count;

        if (algo_para->output_data_size != (sizeof(FL6x32_algo_pcl_output_t) / sizeof(float))) { // 多峰点云
            peak_num = algo_para->ROI_info[task->roi_index].peak_num;
        }
        else { // 单峰点云
            peak_num = 1;
        }

        if (output_point_count < 0) {
            FS_LOGW("Invalid output point count %d, frame=%u", output_point_count, task->frame_index);
            output_point_count = 0;
        }

        {
            uint32_t max_output_point_count = max_pixel_count * peak_num;
            if ((uint32_t)output_point_count > max_output_point_count) {
                FS_LOGW("Output point count %d exceeds capacity %u, truncating, peak_num=%u", output_point_count, max_output_point_count, peak_num);
                output_point_count = (int32_t)max_output_point_count;
            }
        }

        if ((output_point_count % peak_num) != 0) {
            FS_LOGW("Output point count %d is not divisible by peak_num=%u, truncating tail", output_point_count, peak_num);
            output_point_count = (output_point_count / peak_num) * peak_num;
        }

        pcl_count = output_point_count / peak_num;

        if (g_pcl_merge) {
            fs_pcl     = g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].ROI_ctx[0].fs_pcl;
            pcl_unit_u = g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.ROI_info[0].pcl_unit;

            for (int i = 1; i < g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.roi_cnt; i++) {
                if (pcl_unit_u < g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.ROI_info[i].pcl_unit) {
                    pcl_unit_u = g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.ROI_info[i].pcl_unit;
                }
            }
        }
        else {
            fs_pcl     = g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].ROI_ctx[task->roi_index].fs_pcl;
            pcl_unit_u = g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.ROI_info[task->roi_index].pcl_unit;
        }

        // 获取 pcl_unit 解码后的实际单位值 (cm)
        pcl_unit_f = fs_decode_pcl_unit(pcl_unit_u);

        if (pcl_unit_f <= 0.0f) {
            pcl_unit_f = 1.0f;  // 防止除零
        }

        // 转换格式到 fs_pcl_dist_xyz_info_t
        if (g_pcl_merge && g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.roi_cnt > 1 && task->roi_index == g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.roi_cnt - 1) {
            for (int thread_idx = 0; thread_idx < ALGO_THREAD_CNT; thread_idx++) {
                FL6x32_algo_pcl_output_t *thread_output = g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].ROI_ctx[task->roi_index].algo_pcl_thread_buf[thread_idx];
                int thread_output_len = g_algo_mt_pool.per_thread_output_len[thread_idx] / (int)(sizeof(FL6x32_algo_pcl_output_t) / sizeof(float));
                thread_output_len /= peak_num;

                for (pix_idx = 0; pix_idx < thread_output_len; pix_idx++) {
                    fs_pcl_dist_xyz_info_t *pcl;
                    int32_t  src_idx   = pix_idx * peak_num;
                    uint16_t col = (uint16_t)thread_output[src_idx].col_idx;
                    uint16_t row = (uint16_t)thread_output[src_idx].row_idx;
                    uint32_t roi0_idx = 0;
                    int32_t  offset;
                    float roi0_max_peak_dist;

                    if (row == algo_para->ROI_info[task->roi_index].row_start || row == (algo_para->ROI_info[task->roi_index].row_start + algo_para->ROI_info[task->roi_index].row_size - 1)) {
                        // roi1的第一行和最后一行不处理，因为没有roi0的数据可以参考，直接丢弃掉，避免误判为被遮挡
                        continue;
                    }

                    roi0_idx = (row - algo_para->ROI_info[0].row_start) * algo_para->ROI_info[0].col_size + col;
                    offset = roi0_idx * (sizeof(fs_pcl_dist_xyz_info_t) + sizeof(fs_pcl_xyz_info_t) * peak_num);
                    pcl    = (fs_pcl_dist_xyz_info_t *)((uint8_t *)g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].ROI_ctx[0].fs_pcl + offset);
                    roi0_max_peak_dist = pcl->pcl_info[0].distance * pcl_unit_f;

                    if (((roi0_max_peak_dist > 0) && (roi0_max_peak_dist < 1400)) && thread_output[src_idx].distance < 1400) {
                        continue; // 如果roi0已经有点云数据了，并且距离小于1400cm了，则认为roi1的点云数据是被roi0遮挡的，丢弃掉
                    }

                    pcl = (fs_pcl_dist_xyz_info_t *)((uint8_t *)fs_pcl + offset);
                    pcl->col                      = col;
                    pcl->row                      = row;
                    pcl->timestamp_offset         = (uint32_t)((g_pixel_timestamp_offset_data[task->roi_index][pcl->row][pcl->col] * 40u) / 1000u);

                    for (peak_idx = 0; peak_idx < peak_num; peak_idx++) {
                        if ((peak_idx >= 1) && ((thread_output[src_idx + peak_idx].col_idx != pcl->col) || (thread_output[src_idx + peak_idx].row_idx != pcl->row))) {
                            FS_LOGW("Pixel index mismatch at thread %d, pix_idx %d, peak_idx %d: expected (%u, %u), got (%u, %u), frame=%u, roi_index=%u",
                                    thread_idx, pix_idx, peak_idx,
                                    pcl->col, pcl->row,
                                    (uint16_t)thread_output[src_idx + peak_idx].col_idx,
                                    (uint16_t)thread_output[src_idx + peak_idx].row_idx,
                                    task->frame_index, task->roi_index);
                        }

                        // 使用 pcl_unit_f 压缩数据到 16bit 以内(四舍五入)
                        pcl->pcl_info[peak_idx].distance     = (uint16_t)(thread_output[src_idx + peak_idx].distance / pcl_unit_f + 0.5f);
                        pcl->pcl_info[peak_idx].x            = ( int16_t)(thread_output[src_idx + peak_idx].x        / pcl_unit_f + 0.5f);
                        pcl->pcl_info[peak_idx].y            = ( int16_t)(thread_output[src_idx + peak_idx].y        / pcl_unit_f + 0.5f);
                        pcl->pcl_info[peak_idx].z            = ( int16_t)(thread_output[src_idx + peak_idx].z        / pcl_unit_f + 0.5f);
                        pcl->pcl_info[peak_idx].reflectivity = (uint8_t ) thread_output[src_idx + peak_idx].refl;
                        pcl->pcl_info[peak_idx].tag_label    = 0;
                    }

                    if ((pcl->timestamp_offset == 0) && (pcl->pcl_info[0].distance != 0)) {
                        FS_LOGW("PCL pixel timestamp warning: roi_index=%u, point(%u, %u) timestamp_offset is 0", task->roi_index, pcl->col, pcl->row);
                    }
                }

                point_offset += (uint32_t)thread_output_len;
            }
        }
        else {
            roi0_pcl_count         = pcl_count;
            roi0_max_sub_frame_cnt = task->max_sub_frame_cnt;
            roi0_phy_group_id      = task->phy_group_id;

            for (int thread_idx = 0; thread_idx < ALGO_THREAD_CNT; thread_idx++) {
                FL6x32_algo_pcl_output_t *thread_output = g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].ROI_ctx[task->roi_index].algo_pcl_thread_buf[thread_idx];
                int thread_output_len = g_algo_mt_pool.per_thread_output_len[thread_idx] / (int)(sizeof(FL6x32_algo_pcl_output_t) / sizeof(float));
                thread_output_len /= peak_num;

                for (pix_idx = 0; pix_idx < thread_output_len; pix_idx++) {
                    int32_t  src_idx   = pix_idx * peak_num;
                    uint32_t local_idx = point_offset + (uint32_t)pix_idx;
                    int32_t  offset    = local_idx * (sizeof(fs_pcl_dist_xyz_info_t) + sizeof(fs_pcl_xyz_info_t) * peak_num);
                    fs_pcl_dist_xyz_info_t *pcl = (fs_pcl_dist_xyz_info_t *)((uint8_t *)fs_pcl + offset);
                    pcl->col                      = (uint16_t)thread_output[src_idx].col_idx;
                    pcl->row                      = (uint16_t)thread_output[src_idx].row_idx;
                    pcl->timestamp_offset         = (uint32_t)((g_pixel_timestamp_offset_data[task->roi_index][pcl->row][pcl->col] * 40u) / 1000u);

                    for (peak_idx = 0; peak_idx < peak_num; peak_idx++) {
                        if ((peak_idx >= 1) && ((thread_output[src_idx + peak_idx].col_idx != pcl->col) || (thread_output[src_idx + peak_idx].row_idx != pcl->row))) {
                            FS_LOGW("Pixel index mismatch at thread %d, pix_idx %d, peak_idx %d: expected (%u, %u), got (%u, %u), frame=%u, roi_index=%u",
                                    thread_idx, pix_idx, peak_idx,
                                    pcl->col, pcl->row,
                                    (uint16_t)thread_output[src_idx + peak_idx].col_idx,
                                    (uint16_t)thread_output[src_idx + peak_idx].row_idx,
                                    task->frame_index, task->roi_index);
                        }

                        // 使用 pcl_unit_f 压缩数据到 16bit 以内(四舍五入)
                        pcl->pcl_info[peak_idx].distance     = (uint16_t)(thread_output[src_idx + peak_idx].distance / pcl_unit_f + 0.5f);
                        pcl->pcl_info[peak_idx].x            = ( int16_t)(thread_output[src_idx + peak_idx].x        / pcl_unit_f + 0.5f);
                        pcl->pcl_info[peak_idx].y            = ( int16_t)(thread_output[src_idx + peak_idx].y        / pcl_unit_f + 0.5f);
                        pcl->pcl_info[peak_idx].z            = ( int16_t)(thread_output[src_idx + peak_idx].z        / pcl_unit_f + 0.5f);
                        pcl->pcl_info[peak_idx].reflectivity = (uint8_t ) thread_output[src_idx + peak_idx].refl;
                        pcl->pcl_info[peak_idx].tag_label    = 0;
                    }

                    if ((pcl->timestamp_offset == 0) && (pcl->pcl_info[0].distance != 0)) {
                        FS_LOGW("PCL pixel timestamp warning: roi_index=%u, point(%u, %u) timestamp_offset is 0", task->roi_index, pcl->col, pcl->row);
                    }
                }

                point_offset += (uint32_t)thread_output_len;
            }
        }

        if (g_pcl_merge && g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.roi_cnt > 1 && task->roi_index < g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.roi_cnt - 1) {
            // 如果有多个 ROI，且当前处理的是其中一个 ROI，则继续等待同一帧的其他 ROI 数据，以便合并发送
            pcl_task_release(task);
            continue;
        }

        // 准备发送数据
        if (g_pcl_merge) {
            // 将roi1的点云数据合并到roi0中一起发送，简化上层处理
            memset(&pcl_data_info, 0, sizeof(fs_lidar_data_info_t));
            pcl_data_info.data_type = DATA_TYPE_POINTCLOUD;
            pcl_data_info.max_sub_frame_cnt        = roi0_max_sub_frame_cnt;
            pcl_data_info.group_info.vir_group_idx = 0;  // 为了制造 PC 可接受的序列, 只能这样设置
            pcl_data_info.group_info.phy_group_idx = roi0_phy_group_id;
            pcl_data_info.group_info.pixel_len     = sizeof(fs_pcl_dist_xyz_info_t) + sizeof(fs_pcl_xyz_info_t) * peak_num;
            pcl_data_info.group_info.pixel_count   = roi0_pcl_count/*  / task->sub_frame_cnt */; // FIXME: 真实点数应当使用当前 group 的子帧数量计算
            pcl_data_info.group_info.roi_index     = 0;

            pcl_data_info.data                     = (uint8_t *)fs_pcl;
            pcl_data_info.frm_id                   = task->frame_index;
            pcl_data_info.ab_frame_idx             = task->ab_frame_idx;
            pcl_data_info.timestamp_base           = task->frame_base_timestamp;
            pcl_data_info.ROI_info[0].width         = algo_para->ROI_info[0].col_size;
            pcl_data_info.ROI_info[0].height        = algo_para->ROI_info[0].row_size;
            pcl_data_info.ROI_info[0].sub_frame_cnt = task->sub_frame_cnt;
            pcl_data_info.ROI_info[0].sub_frame_idx = task->sub_frame_idx;
            pcl_data_info.ROI_info[0].pcl_unit      = pcl_unit_u;
            memcpy(pcl_data_info.peak_type, g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.peak_type, sizeof(pcl_data_info.peak_type));
            pcl_data_info.max_bin_cnt              = g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.max_bin_cnt;
            pcl_data_info.group_cnt                = task->group_cnt;
        }
        else {
            // 如果不合并，则每个 ROI 单独发送，当前只发送 roi_index
            memset(&pcl_data_info, 0, sizeof(fs_lidar_data_info_t));
            pcl_data_info.data_type = DATA_TYPE_POINTCLOUD;
            pcl_data_info.max_sub_frame_cnt        = task->max_sub_frame_cnt;
            pcl_data_info.group_info.vir_group_idx = task->roi_index;  // 为了制造 PC 可接受的序列, 只能这样设置
            pcl_data_info.group_info.phy_group_idx = task->phy_group_id;
            pcl_data_info.group_info.pixel_len     = sizeof(fs_pcl_dist_xyz_info_t) + sizeof(fs_pcl_xyz_info_t) * peak_num;
            pcl_data_info.group_info.pixel_count   = (uint32_t)(pcl_count)/*  / task->sub_frame_cnt */; // FIXME: 真实点数应当使用当前 group 的子帧数量计算
            pcl_data_info.group_info.roi_index     = task->roi_index;

            pcl_data_info.data                     = (uint8_t *)fs_pcl;
            pcl_data_info.frm_id                   = task->frame_index;
            pcl_data_info.ab_frame_idx             = task->ab_frame_idx;
            pcl_data_info.timestamp_base           = task->frame_base_timestamp;
            pcl_data_info.ROI_info[task->roi_index].width         = algo_para->ROI_info[task->roi_index].col_size;
            pcl_data_info.ROI_info[task->roi_index].height        = algo_para->ROI_info[task->roi_index].row_size;
            pcl_data_info.ROI_info[task->roi_index].sub_frame_cnt = task->sub_frame_cnt;
            pcl_data_info.ROI_info[task->roi_index].sub_frame_idx = task->sub_frame_idx;
            pcl_data_info.ROI_info[task->roi_index].pcl_unit      = pcl_unit_u;
            memcpy(pcl_data_info.peak_type, g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.peak_type, sizeof(pcl_data_info.peak_type));
            pcl_data_info.max_bin_cnt              = g_algo_pcl_ctx.ab_ctx[task->ab_frame_idx].data_info.max_bin_cnt;
            pcl_data_info.group_cnt                = task->group_cnt;
        }

        // 深拷贝数据到新分配的内存
        data_size = pcl_data_info.group_info.pixel_count * pcl_data_info.group_info.pixel_len;

        data_copy = (fs_lidar_data_info_t *)malloc(sizeof(fs_lidar_data_info_t));
        if (!data_copy) {
            FS_LOGE("Failed to allocate PCL data_copy for frame %u", task->frame_index);
            pcl_task_release(task);
            continue;
        }

        data_buffer = (uint8_t *)malloc(data_size);
        if (!data_buffer) {
            FS_LOGE("Failed to allocate PCL data_buffer, size=%u, frame=%u", data_size, task->frame_index);
            free(data_copy);
            pcl_task_release(task);
            continue;
        }

        memcpy(data_copy, &pcl_data_info, sizeof(fs_lidar_data_info_t));
        memcpy(data_buffer, pcl_data_info.data, data_size);
        data_copy->data = data_buffer;

        // 推送到发送队列
        ret = algo_pixel_queue_push_data(data_copy);
        if (ret != FS_SUCCESS) {
            FS_LOGE("Failed to push PCL data for frame %u, ret=%d", task->frame_index, ret);
            free(data_buffer);
            free(data_copy);
        }

        // 释放任务并归还双缓冲状态
        pcl_task_release(task);
        task = NULL;
    }

    FS_LOGI("PCL process thread exited");
    return NULL;
}

static void init_algo_pcl_para(fs_lidar_data_info_t *info, FL6x32_para *algo_para)
{
    FL6x32_para para = {
        .input_data_version = 0,    // 0 - range , 1 - echo
        .calib_version      = 2,
#if ALGO_PCL_DOUBLE_ECHO
        .output_data_size   = (sizeof(FL6x32_algo_pcl_output_t) / sizeof(float)) * info->peak_num,
#else
        .output_data_size   = (sizeof(FL6x32_algo_pcl_output_t) / sizeof(float)),
#endif

        .ROI_cnt            = info->roi_cnt,
        .ROI_idx            = 0,
        .ROI_info           = {
            { .group_start = info->ROI_info[0].group_start, .piece_start = 0, .row_start = ((info->ROI_info[0].row_unit > 0) ? (info->ROI_info[0].group_start * info->group_row_unit / info->ROI_info[0].row_unit) : 0), .col_start = 0, .row_size = info->ROI_info[0].height, .col_size = info->ROI_info[0].width, .peak_num = info->peak_num, .range_data_size = RANGE_ONE_PIXEL_INFO_LEN(info->peak_num), .binning_model = { info->ROI_info[0].col_unit, info->ROI_info[0].row_unit }, .roll_mode = ((info->ROI_info[0].sub_frame_cnt == 1) ? 0 : info->ROI_info[0].sub_frame_cnt), .roll_idx = 0, },
            { .group_start = info->ROI_info[1].group_start, .piece_start = 0, .row_start = ((info->ROI_info[1].row_unit > 0) ? (info->ROI_info[1].group_start * info->group_row_unit / info->ROI_info[1].row_unit) : 0), .col_start = 0, .row_size = info->ROI_info[1].height, .col_size = info->ROI_info[1].width, .peak_num = info->peak_num, .range_data_size = RANGE_ONE_PIXEL_INFO_LEN(info->peak_num), .binning_model = { info->ROI_info[1].col_unit, info->ROI_info[1].row_unit }, .roll_mode = ((info->ROI_info[1].sub_frame_cnt == 1) ? 0 : info->ROI_info[1].sub_frame_cnt), .roll_idx = 0, },
            { .group_start = info->ROI_info[2].group_start, .piece_start = 0, .row_start = ((info->ROI_info[2].row_unit > 0) ? (info->ROI_info[2].group_start * info->group_row_unit / info->ROI_info[2].row_unit) : 0), .col_start = 0, .row_size = info->ROI_info[2].height, .col_size = info->ROI_info[2].width, .peak_num = info->peak_num, .range_data_size = RANGE_ONE_PIXEL_INFO_LEN(info->peak_num), .binning_model = { info->ROI_info[2].col_unit, info->ROI_info[2].row_unit }, .roll_mode = ((info->ROI_info[2].sub_frame_cnt == 1) ? 0 : info->ROI_info[2].sub_frame_cnt), .roll_idx = 0, },
            { .group_start = info->ROI_info[3].group_start, .piece_start = 0, .row_start = ((info->ROI_info[3].row_unit > 0) ? (info->ROI_info[3].group_start * info->group_row_unit / info->ROI_info[3].row_unit) : 0), .col_start = 0, .row_size = info->ROI_info[3].height, .col_size = info->ROI_info[3].width, .peak_num = info->peak_num, .range_data_size = RANGE_ONE_PIXEL_INFO_LEN(info->peak_num), .binning_model = { info->ROI_info[3].col_unit, info->ROI_info[3].row_unit }, .roll_mode = ((info->ROI_info[3].sub_frame_cnt == 1) ? 0 : info->ROI_info[3].sub_frame_cnt), .roll_idx = 0, },
        },
        .TX_temperature        = 30,
        .RX_temperature        = 30,
        .multithread_num       = ((ALGO_THREAD_CNT == 0) ? 1 : ALGO_THREAD_CNT),
        .neighb_denoise_switch = 1,
        .backfill_switch       = 1,
        .dist_smooth_switch    = 1,
        .refl_smooth_switch    = 1,
    };
    *algo_para = para;
    return;
}

static void show_algo_pcl_para(FL6x32_para *algo_para, int32_t ab_idx)
{
    FS_LOGI("=== algo pcl para Configuration[%d] ===", ab_idx);

    FS_LOGI("input_data_version    = %d", algo_para->input_data_version);
    FS_LOGI("calib_version         = %d", algo_para->calib_version);
    FS_LOGI("output_data_size      = %d", algo_para->output_data_size);

    FS_LOGI("ROI_cnt               = %d", algo_para->ROI_cnt);
    FS_LOGI("ROI_idx               = %d", algo_para->ROI_idx);
    for (int32_t i = 0; i < algo_para->ROI_cnt && i < 4; i++) {
        FS_LOGI("ROI_info[%d]: group_start=%d, piece_start=%d, row_start=%d, col_start=%d",
                i,
                algo_para->ROI_info[i].group_start,
                algo_para->ROI_info[i].piece_start,
                algo_para->ROI_info[i].row_start,
                algo_para->ROI_info[i].col_start);
        FS_LOGI("ROI_info[%d]: row_size=%d, col_size=%d, peak_num=%d, range_data_size=%u",
                i,
                algo_para->ROI_info[i].row_size,
                algo_para->ROI_info[i].col_size,
                algo_para->ROI_info[i].peak_num,
                algo_para->ROI_info[i].range_data_size);
        FS_LOGI("ROI_info[%d]: binning_model=[%d, %d], roll_mode=%d, roll_idx=%d",
                i,
                algo_para->ROI_info[i].binning_model[0],
                algo_para->ROI_info[i].binning_model[1],
                algo_para->ROI_info[i].roll_mode,
                algo_para->ROI_info[i].roll_idx);
    }

    FS_LOGI("TX_temperature        = %.3f", algo_para->TX_temperature);
    FS_LOGI("RX_temperature        = %.3f", algo_para->RX_temperature);
    FS_LOGI("multithread_num       = %d", algo_para->multithread_num);
    FS_LOGI("neighb_denoise_switch = %d", algo_para->neighb_denoise_switch);
    FS_LOGI("backfill_switch       = %d", algo_para->backfill_switch);
    FS_LOGI("dist_smooth_switch    = %d", algo_para->dist_smooth_switch);
    FS_LOGI("refl_smooth_switch    = %d", algo_para->refl_smooth_switch);
    FS_LOGI("=================================");
}

static int32_t load_calib_data(float **calib_data, uint32_t *calib_cnt, int32_t roi_cnt)
{
    int32_t err = FS_SUCCESS;
    FILE *file = NULL;
    long file_size = 0;
    size_t float_count = 0;
    size_t read_size = 0;

    for (int32_t i = 0; i < roi_cnt; i++) {
        // FIXME: AB 帧如何适配 ab_ctx[0] 和 ab_ctx[1]
        float_count += (3 + g_algo_pcl_ctx.ab_ctx[0].data_info.ROI_info[i].width * g_algo_pcl_ctx.ab_ctx[0].data_info.ROI_info[i].height * 2);
    }

    // 打开文件
    file = fopen(LIDAR_CALIB_DATA_PATH, "rb");

    if (file == NULL) {
        FS_LOGE("Failed to open file: %s", LIDAR_CALIB_DATA_PATH);
        err = load_calib_data_from_mcu(calib_data, calib_cnt);

        if (err == FS_SUCCESS) {
            // 将读出来的校准文件同步保存到 LIDAR_CALIB_DATA_PATH 以加速下次开机时间(即使读出来的文件是错误的也要缓存, 避免每次都从 MCU 读影响开机时间)
            FILE *calib_file = fopen(LIDAR_CALIB_DATA_PATH, "wb");

            if (calib_file == NULL && errno == ENOENT) {
                FS_LOGD("Parent directory does not exist for %s, attempting to create it", LIDAR_CALIB_DATA_PATH);
                int32_t mkdir_ret = fs_file_mkdir(fs_file_dirname(LIDAR_CALIB_DATA_PATH));

                if (mkdir_ret != FS_SUCCESS) {
                    FS_LOGW("Failed to create parent directory for %s, ret=%d", LIDAR_CALIB_DATA_PATH, mkdir_ret);
                }
                else {
                    calib_file = fopen(LIDAR_CALIB_DATA_PATH, "wb");
                }
            }

            if (calib_file != NULL) {
                size_t write_count = fwrite(*calib_data, sizeof(float), *calib_cnt, calib_file);

                if (write_count == *calib_cnt) {
                    FS_LOGI("Saved calib data to %s: %u floats", LIDAR_CALIB_DATA_PATH, *calib_cnt);
                }
                else {
                    FS_LOGW("Failed to save all calib data to %s: expected %u floats, wrote %zu", LIDAR_CALIB_DATA_PATH, *calib_cnt, write_count);
                }

                fclose(calib_file);
            }
            else {
                FS_LOGW("Failed to open calib data file for writing: %s", LIDAR_CALIB_DATA_PATH);
            }

            if (*calib_cnt != float_count) {
                FS_LOGE("Calib data count mismatch: expected %u floats, got %u", (uint32_t)float_count, *calib_cnt);
                free(*calib_data);
                *calib_data = NULL;
                *calib_cnt = 0;
            }
            else {
                return FS_SUCCESS;
            }
        }

        *calib_data = (float *)malloc(float_count * sizeof(float));
        *calib_cnt = (uint32_t)float_count;
        return FS_ERR_NOT_FOUND;
    }

    // 获取文件大小
    if (fseek(file, 0, SEEK_END) != 0) {
        FS_LOGE("Failed to seek file: %s", LIDAR_CALIB_DATA_PATH);
        fclose(file);
        return FS_ERR_INTERNAL;
    }

    file_size = ftell(file);
    if (file_size < 0) {
        FS_LOGE("Failed to get file size: %s", LIDAR_CALIB_DATA_PATH);
        fclose(file);
        return FS_ERR_INTERNAL;
    }

    // 检查文件大小是否为 sizeof(float) 的倍数
    if (file_size == 0 || (file_size % sizeof(float)) != 0) {
        FS_LOGE("Invalid calib data file size: %ld, not a multiple of sizeof(float)=%zu", file_size, sizeof(float));
        fclose(file);
        return FS_ERR_BAD_PARAMS;
    }

    // 判断文件中的 float 数量是否满足预期
    if (float_count != (size_t)file_size / sizeof(float)) {
        FS_LOGE("Calib data count mismatch: expected %zu floats, file has %zu floats", float_count, (size_t)file_size / sizeof(float));
        *calib_data = (float *)malloc(float_count * sizeof(float));
        *calib_cnt = (uint32_t)float_count;
        fclose(file);
        return FS_ERR_NOT_FOUND;
    }

    float_count = (size_t)file_size / sizeof(float);

    // 回到文件开头
    if (fseek(file, 0, SEEK_SET) != 0) {
        FS_LOGE("Failed to seek file to beginning: %s", LIDAR_CALIB_DATA_PATH);
        fclose(file);
        return FS_ERR_INTERNAL;
    }

    // 释放旧的内存（如果存在）
    if (*calib_data != NULL) {
        free(*calib_data);
        *calib_data = NULL;
        *calib_cnt = 0;
    }

    // 分配新的内存
    *calib_data = (float *)malloc(float_count * sizeof(float));
    if (*calib_data == NULL) {
        FS_LOGE("Failed to allocate memory for calib data, size=%zu", float_count * sizeof(float));
        fclose(file);
        return FS_ERR_NOMEM;
    }

    // 读取文件内容
    read_size = fread(*calib_data, sizeof(float), float_count, file);
    if (read_size != float_count) {
        FS_LOGE("Failed to read calib data file: %s, expected %zu floats, got %zu", LIDAR_CALIB_DATA_PATH, float_count, read_size);
        free(*calib_data);
        *calib_data = NULL;
        fclose(file);
        return FS_ERR_NOT_FOUND;
    }

    fclose(file);

    // TODO: 检查 LIDAR_CALIB_DATA_PATH 文件的 CRC 是否与 MCU 外置 Flash 中的校准数据一致，如果不一致则重新从 MCU 加载并覆盖文件

    *calib_cnt = (uint32_t)float_count;
    FS_LOGI("Loaded calib data: %u floats from %s", *calib_cnt, LIDAR_CALIB_DATA_PATH);
    return FS_SUCCESS;
}

/**
 * @brief 重置 PCL 相关全局变量
 *
 * 此函数在 algo_init() 中调用，确保新一轮初始化时 PCL 线程相关的全局变量处于干净状态
 * 防止旧线程未完全清理导致的线程泄漏问题
 */
void algo_pcl_reset_globals(void)
{
    g_pcl_thread_id = 0;
    g_pcl_thread_run_flag = 0;
    g_pcl_task_queue = NULL;
    memset(&g_algo_pcl_ctx, 0, sizeof(algo_pcl_ctx_t));
    FS_LOGD("PCL globals reset to initial state");
}

int32_t algo_pcl_uninit(int32_t ab_idx)
{
    int err = FS_SUCCESS;
    int32_t i;

    // ============ 只在 ab_idx == 0 时清理全局共享资源 ============
    if (ab_idx == 0) {
        // 停止 PCL 处理线程（全局共享，只清理一次）
        if (g_pcl_thread_id != 0) {
            g_pcl_thread_run_flag = 0;  // 设置退出标志
            pthread_join(g_pcl_thread_id, NULL);  // 等待线程退出
            g_pcl_thread_id = 0;
            FS_LOGI("PCL process thread stopped");
        }

        algo_mt_stop_workers();

        // 销毁任务队列（全局共享，只清理一次）
        if (g_pcl_task_queue) {
            // 清空队列中剩余的任务，并归还双缓冲状态
            pcl_process_task_t *task = NULL;
            while ((task = (pcl_process_task_t *)queue_pop_timeout(g_pcl_task_queue, 0)) != NULL) {
                pcl_task_release(task);
            }
            queue_destroy(g_pcl_task_queue);
            g_pcl_task_queue = NULL;
            FS_LOGI("PCL task queue destroyed");
        }

        // 仅在算法环境完成初始化后执行清理，避免重复释放库内全局资源。
        if (g_algo_pcl_ctx.algo_env_init) {
            err = Clean_Algo_Env();
            if (err) {
                FS_LOGE(" Clean Algo Env failed...");
            }
            g_algo_pcl_ctx.algo_env_init = false;
        }

        // 释放校准数据（全局共享，只清理一次）
        if (g_algo_pcl_ctx.calib_data) {
            free(g_algo_pcl_ctx.calib_data);
            g_algo_pcl_ctx.calib_data = NULL;
            g_algo_pcl_ctx.calib_cnt = 0;
        }

        g_algo_pcl_ctx.algo_init = false;
    }

    // ============ 清理每个 AB 帧独立的资源 ============
    for (i = 0; i < g_algo_pcl_ctx.ab_ctx[ab_idx].data_info.roi_cnt; i++) {
        if (g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[0]) {
            free(g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[0]);
            g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[0] = NULL;
        }

        if (g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[1]) {
            free(g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[1]);
            g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[1] = NULL;
        }

        if (g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[0]) {
            free(g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[0]);
            g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[0] = NULL;
        }

        if (g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[1]) {
            free(g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[1]);
            g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[1] = NULL;
        }

        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_info = NULL;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_info = NULL;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].write_buf_idx = 0;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].buf_in_use[0] = 0;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].buf_in_use[1] = 0;

        if (g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].algo_pcl) {
            free(g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].algo_pcl);
            g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].algo_pcl = NULL;
        }

        if (g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].fs_pcl) {
            free(g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].fs_pcl);
            g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].fs_pcl = NULL;
        }

        for (int t = 0; t < ALGO_THREAD_CNT; t++) {
            if (g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].algo_pcl_thread_buf[t]) {
                free(g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].algo_pcl_thread_buf[t]);
                g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].algo_pcl_thread_buf[t] = NULL;
            }
        }
    }

    return err;

}

int32_t algo_pcl_init_param(int32_t ab_idx, int32_t roi_cnt)
{
    int32_t err = FS_SUCCESS;
    fs_lidar_data_info_t *data_info = &g_algo_pcl_ctx.ab_ctx[ab_idx].data_info;
    FL6x32_para          *algo_para = &g_algo_pcl_ctx.ab_ctx[ab_idx].algo_para;
    int32_t i;

    snprintf(g_version.algo_ver, sizeof(g_version.algo_ver), "%s", Obtain_Version());

    if (ab_idx == 0) {
        FS_LOGI("algo version: %s", Obtain_Version());
    }

    // 初始化算法参数
    init_algo_pcl_para(data_info, algo_para);
    show_algo_pcl_para(algo_para, ab_idx);

    if (ab_idx == 0) {
        err = load_calib_data(&g_algo_pcl_ctx.calib_data, &g_algo_pcl_ctx.calib_cnt, roi_cnt);

        if (err == FS_ERR_NOT_FOUND) {
            FS_LOGW("Calib data file not found!");
            err = FS_SUCCESS;  // 校准文件不存在不是致命错误，当作无校准表降级处理即可(点云效果会比较差)
        }
        else if (err != FS_SUCCESS) {
            FS_LOGE("Failed to load calib data, err=%d", err);
            algo_pcl_uninit(ab_idx);
            return err;
        }
    }

    for (i = 0; i < data_info->roi_cnt; i++) {
        uint32_t range_buf_size = data_info->ROI_info[i].width * data_info->ROI_info[i].height *
                                  RANGE_ONE_PIXEL_INFO_LEN(data_info->peak_num) /
                                  data_info->ROI_info[i].sub_frame_cnt;
        uint32_t ts_count = (data_info->ROI_info[i].width * data_info->ROI_info[i].height) /
                            data_info->ROI_info[i].sub_frame_cnt;

        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[0]   = (FL6x32_range_pix_info_t  *)malloc(range_buf_size);
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[1]   = (FL6x32_range_pix_info_t  *)malloc(range_buf_size);
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[0]= (uint32_t *)calloc(ts_count, sizeof(uint32_t));
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[1]= (uint32_t *)calloc(ts_count, sizeof(uint32_t));
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].algo_pcl       = (FL6x32_algo_pcl_output_t *)malloc(data_info->ROI_info[i].width * data_info->ROI_info[i].height *  data_info->peak_num * sizeof(FL6x32_algo_pcl_output_t));
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].fs_pcl         = (fs_pcl_dist_xyz_info_t   *)malloc(data_info->ROI_info[i].width * data_info->ROI_info[i].height * (data_info->peak_num * sizeof(fs_pcl_xyz_info_t) + sizeof(fs_pcl_dist_xyz_info_t)));
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_data_len = range_buf_size;

        for (int t = 0; t < ALGO_THREAD_CNT; t++) {
            g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].algo_pcl_thread_buf[t] =
                (FL6x32_algo_pcl_output_t *)malloc(data_info->ROI_info[i].width * data_info->ROI_info[i].height * sizeof(FL6x32_algo_pcl_output_t));
        }

        if (!g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[0] ||
            !g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[1] ||
            !g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[0] ||
            !g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[1] ||
            !g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].algo_pcl ||
            !g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].fs_pcl) {
            FS_LOGE("PCL ROI memory alloc failed: ab_idx=%d roi=%d", ab_idx, i);
            algo_pcl_uninit(ab_idx);
            return FS_ERR_NOMEM;
        }

        for (int t = 0; t < ALGO_THREAD_CNT; t++) {
            if (!g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].algo_pcl_thread_buf[t]) {
                FS_LOGE("PCL ROI thread buffer alloc failed: ab_idx=%d roi=%d thread=%d", ab_idx, i, t);
                algo_pcl_uninit(ab_idx);
                return FS_ERR_NOMEM;
            }
        }

        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].write_buf_idx = 0;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].buf_in_use[0] = 0;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].buf_in_use[1] = 0;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].updated_frame_idx = 0;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].updated_sub_frame_idx = 0;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].enqueued_frame_idx = 0;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].enqueued_sub_frame_idx = 0;
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_info =
            g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].range_buf[0];
        g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_info =
            g_algo_pcl_ctx.ab_ctx[ab_idx].ROI_ctx[i].timestamp_buf[0];

        FS_LOGI("PCL range double-buffer initialized: ab_idx=%d, roi=%d, range_buf_size=%u",
                ab_idx, i, range_buf_size);
    }

    if (ab_idx == 0) {
        // FIXME: AB 帧应当各初始化一次, 需要算法支持
        err = Init_Algo_Env(&g_algo_pcl_ctx.ab_ctx[ab_idx].algo_para, g_algo_pcl_ctx.calib_data, g_algo_pcl_ctx.calib_cnt);
        if (err != FS_SUCCESS) {
            FS_LOGE(" Init Alge Env failed..");
            algo_pcl_uninit(ab_idx);
            g_algo_pcl_ctx.algo_init = false;
            return err;
        }

        g_algo_pcl_ctx.algo_env_init = true;

        // 创建 PCL 任务队列
        g_pcl_task_queue = queue_create(PCL_TASK_QUEUE_SIZE);
        if (!g_pcl_task_queue) {
            FS_LOGE("Failed to create PCL task queue");
            algo_pcl_uninit(ab_idx);
            g_algo_pcl_ctx.algo_init = false;
            return FS_ERR_NOMEM;
        }

        // 启动 PCL 处理线程
        g_pcl_thread_run_flag = 1;
        if (pthread_create(&g_pcl_thread_id, NULL, pcl_process_thread, NULL) != 0) {
            FS_LOGE("Failed to create PCL process thread");
            queue_destroy(g_pcl_task_queue);
            g_pcl_task_queue = NULL;
            algo_pcl_uninit(ab_idx);
            g_algo_pcl_ctx.algo_init = false;
            return FS_ERR_INTERNAL;
        }

        err = algo_mt_start_workers();
        if (err != FS_SUCCESS) {
            FS_LOGE("Failed to start persistent PCL multithread workers");
            algo_pcl_uninit(ab_idx);
            g_algo_pcl_ctx.algo_init = false;
            return err;
        }
    }

    g_algo_pcl_ctx.algo_init = true;
    FS_LOGI("PCL async processing initialized successfully");
    return err;
}

int32_t algo_pcl_update_tx_temp(float temp_tx)
{
    int32_t err = FS_SUCCESS;
    int32_t ab_idx;

    for (ab_idx = 0; ab_idx < (int32_t)(sizeof(g_algo_pcl_ctx.ab_ctx) / sizeof(g_algo_pcl_ctx.ab_ctx[0])); ab_idx++) {
        FL6x32_para *algo_para = &g_algo_pcl_ctx.ab_ctx[ab_idx].algo_para;
        algo_para->TX_temperature = temp_tx;
    }

    return err;
}

int32_t algo_pcl_update_rx_temp(const float *temp_rx, uint32_t temp_rx_cnt)
{
    int32_t err = FS_SUCCESS;
    int32_t ab_idx;
    CHECK_PTR(temp_rx);

    if (temp_rx_cnt <= 0) {
        FS_LOGE("temp_rx_cnt(%u) is invalid", temp_rx_cnt);
        return FS_ERR_BAD_PARAMS;
    }

    for (ab_idx = 0; ab_idx < (int32_t)(sizeof(g_algo_pcl_ctx.ab_ctx) / sizeof(g_algo_pcl_ctx.ab_ctx[0])); ab_idx++) {
        FL6x32_para *algo_para = &g_algo_pcl_ctx.ab_ctx[ab_idx].algo_para;
        algo_para->RX_temperature = temp_rx[0];
    }

    return err;
}

static int32_t algo_pcl_push_one_pixel(FL6x32_range_pix_info_t *range_info, cicl_pix_info_t *pix_info, uint16_t row_start, uint16_t width, uint16_t height, uint8_t ROI_sel, algo_result_unpack_t *peak_result)
{
    int err = FS_SUCCESS;
    FL6x32_range_pix_info_t *pix_data;
    uint32_t pix_index;

    pix_index = (pix_info->row - row_start) * width + pix_info->col;
    pix_index /= (1 << pix_info->pkt_info.piece_roll_mode);

    if ((pix_info->row < row_start) || ((pix_info->row - row_start) >= height) || (pix_index >= ((uint32_t)width * height) / (uint32_t)(1 << pix_info->pkt_info.piece_roll_mode)) || pix_info->peak_num > 2) {
        FS_LOGE("Pixel row %d out of bounds [%d, %d), pix_index %u out of bounds [0, %u), peak_num %u not supported", pix_info->row, row_start, row_start + height, pix_index, width * height / (1 << pix_info->pkt_info.piece_roll_mode), pix_info->peak_num);
        return FS_ERR_BAD_PARAMS;
    }
    else {
        UNUSED_VAR(ROI_sel);
    }

    pix_data = (FL6x32_range_pix_info_t *)(((uint8_t *)range_info) + pix_index * RANGE_ONE_PIXEL_INFO_LEN(pix_info->peak_num));
    pix_data->pixel_id         = pix_info->pixel_id;
    pix_data->noise_amb        = pix_info->noise_amb;
    pix_data->amb_num          = pix_info->amb_num;
    pix_data->det_sum          = pix_info->det_sum;
    pix_data->real_tx_num      = pix_info->real_tx_num;
    pix_data->lpwr_real_tx_num = pix_info->lpwr_real_tx_num;
    pix_data->peak_num         = (uint32_t)pix_info->peak_num;
    pix_data->row              = pix_info->row;
    pix_data->col              = pix_info->col;
    memcpy(pix_data->data, peak_result, pix_info->peak_num * sizeof(FL6x32_algo_result_t));

    return err;
}

static const uint8_t s_piece_roll4_0_binning3x3_array[4] = { 0, 5, 10, 15 };
static const uint8_t s_piece_roll4_1_binning3x3_array[4] = { 1, 12, 6, 11 };
static const uint8_t s_piece_roll4_2_binning3x3_array[4] = { 8, 13, 2, 7 };
static const uint8_t s_piece_roll4_3_binning3x3_array[4] = { 4, 9, 3, 14 };
static const uint8_t s_piece_roll2_0_binning3x3_array[8] = { 0, 5, 8, 13, 2, 7, 10, 15 };
static const uint8_t s_piece_roll2_1_binning3x3_array[8] = { 4, 1, 12, 9, 6, 3, 14, 11 };
static const uint8_t s_piece_binning3x3_array[16] = { 0, 4, 1, 5, 8, 12, 9, 13, 2, 6, 3, 7, 10, 14, 11, 15 };
static const uint8_t s_piece_binning2x6_array[12] = { 0, 1, 2, 6, 7, 8, 3, 4, 5, 9, 10, 11 };
static const uint8_t s_piece_binning2x2_array[36] = {
    0, 6, 12, 1, 7, 13, 2, 8, 14, 18, 24, 30,
    19, 25, 31, 20, 26, 32, 3, 9, 15, 4, 10, 16,
    5, 11, 17, 21, 27, 33, 22, 28, 34, 23, 29, 35,
};
static const uint8_t s_piece_binning6x6_array[4] = { 0, 2, 1, 3 };

typedef struct {
    const uint8_t *piece_map;
    uint8_t channel_cnt;
    uint8_t ch_cnt_per_piece;
    uint8_t half_piece_per_ch;
    uint8_t pix_cnt_per_chann;
    uint8_t pix_cnt_per_pie_in_ch;
    uint8_t piece_cnt_per_block;
    uint8_t row_num_in_piece;
    uint8_t col_num_in_piece;
    uint8_t remap_slot_idx;
    uint16_t row_base;
    uint16_t piece_loop_cnt;
} cicl_fast_decode_ctx_t;

static inline const uint8_t *cicl_get_piece_map(uint8_t binning_type, uint8_t piece_roll_mode, uint8_t sub_frame_idx)
{
    switch (binning_type) {
        case RX_BINNING_TYPE_3X3:
            switch (piece_roll_mode) {
                case 1:
                    return (sub_frame_idx == 0) ? s_piece_roll2_0_binning3x3_array : s_piece_roll2_1_binning3x3_array;
                case 2:
                    if (sub_frame_idx == 0) {
                        return s_piece_roll4_0_binning3x3_array;
                    }
                    if (sub_frame_idx == 1) {
                        return s_piece_roll4_1_binning3x3_array;
                    }
                    if (sub_frame_idx == 2) {
                        return s_piece_roll4_2_binning3x3_array;
                    }
                    return s_piece_roll4_3_binning3x3_array;
                case 0:
                case 3:
                default:
                    return s_piece_binning3x3_array;
            }
        case RX_BINNING_TYPE_2X6:
            return s_piece_binning2x6_array;
        case RX_BINNING_TYPE_2X2:
            return s_piece_binning2x2_array;
        case RX_BINNING_TYPE_6X6:
            return s_piece_binning6x6_array;
        default:
            return NULL;
    }
}

static inline bool cicl_decode_pixel_id_fast(const cicl_fast_decode_ctx_t *dctx, uint16_t pixel_id, uint16_t *row, uint16_t *col)
{
    uint32_t pixel_id_in_chann;
    uint32_t channel_id;
    uint32_t piece_id_in_chann;
    uint32_t pix_id_in_pie_in_ch;
    uint32_t pixel_id_in_piece;
    uint32_t piece_id_in_slot;
    uint32_t piece_loop_idx;
    uint32_t piece_loop_off;
    uint32_t phy_piece_id;
    uint32_t piece_pix_id;

    if (dctx->piece_map == NULL || dctx->channel_cnt == 0 || dctx->pix_cnt_per_pie_in_ch == 0 || dctx->row_num_in_piece == 0) {
        return false;
    }

    pixel_id_in_chann = pixel_id / dctx->channel_cnt;
    if (pixel_id_in_chann >= dctx->pix_cnt_per_chann) {
        return false;
    }

    channel_id = pixel_id - pixel_id_in_chann * dctx->channel_cnt;
    piece_id_in_chann = pixel_id_in_chann / dctx->pix_cnt_per_pie_in_ch;
    pix_id_in_pie_in_ch = pixel_id_in_chann - piece_id_in_chann * dctx->pix_cnt_per_pie_in_ch;

    if (dctx->ch_cnt_per_piece > 1) {
        uint32_t half_pix_per_piece;
        uint32_t area_id;
        uint32_t pixel_id_in_area;
        half_pix_per_piece = dctx->pix_cnt_per_pie_in_ch / 2;
        if (half_pix_per_piece == 0) {
            return false;
        }
        area_id = pix_id_in_pie_in_ch / half_pix_per_piece;
        pixel_id_in_area = pix_id_in_pie_in_ch - area_id * half_pix_per_piece;
        pixel_id_in_piece = (area_id * dctx->ch_cnt_per_piece + (channel_id % dctx->ch_cnt_per_piece)) * half_pix_per_piece + pixel_id_in_area;
    } else {
        pixel_id_in_piece = pix_id_in_pie_in_ch;
    }

    piece_id_in_slot = channel_id * dctx->half_piece_per_ch / 2 + piece_id_in_chann;
    piece_loop_idx = piece_id_in_slot / dctx->piece_cnt_per_block;
    piece_loop_off = piece_id_in_slot - piece_loop_idx * dctx->piece_cnt_per_block;
    piece_loop_off += (uint32_t)dctx->remap_slot_idx * dctx->piece_cnt_per_block;
    phy_piece_id = (uint32_t)dctx->piece_loop_cnt * piece_loop_idx + piece_loop_off;
    if (phy_piece_id >= RX_DATA_MAX_PIECE_CNT_PRE_GROUP) {
        return false;
    }

    piece_pix_id = dctx->piece_map[pixel_id_in_piece];
    *row = (uint16_t)(dctx->row_base + piece_pix_id / dctx->row_num_in_piece);
    *col = (uint16_t)(phy_piece_id * dctx->col_num_in_piece + piece_pix_id % dctx->col_num_in_piece);

    return true;
}


int32_t algo_cicl_calc_cfg(algo_context_t* ctx, int32_t ab_idx)
{
    int ret = FS_SUCCESS;

    CHECK_PTR(ctx);
    CHECK_PTR(ctx->parse_cfg);

    const rx_parse_cfg_t* parse_cfg = ctx->parse_cfg;
    fs_lidar_data_info_t *data_info;
    uint16_t peak_cnt;
    uint32_t total_len;
    int32_t i;

    data_info = &g_algo_pcl_ctx.ab_ctx[ab_idx].data_info;
    memset(data_info, 0, sizeof(fs_lidar_data_info_t));

    total_len = 0;
    peak_cnt  = parse_cfg->frm_cfg[ab_idx].pixel_cfg.data_type_cfg.range_cfg.peak_cnt;

    for (i = 0; i < parse_cfg->frm_cfg[ab_idx].group_cnt; i++) {
        rx_ROI_cfg_e ROI_sel;
        uint8_t row_num, col_num;
        uint8_t binning_type;
        uint16_t width;
        uint16_t pixel_len;
        uint16_t echo_bin_len;

        echo_bin_len = 0;
        pixel_len    = sizeof(cicl_pix_info_t);
        ROI_sel      = parse_cfg->frm_cfg[ab_idx].group_cfg[i].ROI_sel;
        binning_type = parse_cfg->frm_cfg[ab_idx].pixel_cfg.ROI_cfg[ROI_sel].binning_type;

        // 计算每个pixel的长度
        for (int32_t j = 0; j < (int32_t)peak_cnt; j++) {
            if (parse_cfg->frm_cfg[ab_idx].pixel_cfg.data_type_cfg.range_cfg.en_echo_bin) {
                uint8_t bin_cnt_l = parse_cfg->frm_cfg[ab_idx].pixel_cfg.ROI_cfg[ROI_sel].echo_cfg[j].bin_cnt_left;
                uint8_t bin_cnt_r = parse_cfg->frm_cfg[ab_idx].pixel_cfg.ROI_cfg[ROI_sel].echo_cfg[j].bin_cnt_right;

                // CICL 模式下的 echo 半峰宽最大支持 22bin
                if (bin_cnt_l > 22) {
                    bin_cnt_l = 22;
                }

                if (bin_cnt_r > 22) {
                    bin_cnt_r = 22;
                }

                echo_bin_len      = (bin_cnt_l + bin_cnt_r + 1) * sizeof(uint16_t);
            } else {
                echo_bin_len = 0;
            }
            pixel_len += (sizeof(algo_result_unpack_t) + echo_bin_len);
        }

        ret = rx_parse_get_row_col_num_in_piece(binning_type, &row_num, &col_num);
        if (ret != FS_SUCCESS) {
            FS_LOGE("Failed to get row/col num in piece, binning_type=%u", binning_type);
            return ret;
        }

        // 计算最大宽度和最大bin长度
        width = parse_cfg->frm_cfg[ab_idx].piece_cnt_per_slot * parse_cfg->frm_cfg[ab_idx].slot_cnt_per_group * col_num;

        if (parse_cfg->frm_cfg[ab_idx].auto_cfg.ROI_cfg[ROI_sel].max_bin_cnt > data_info->max_bin_cnt) {
            data_info->max_bin_cnt = parse_cfg->frm_cfg[ab_idx].auto_cfg.ROI_cfg[ROI_sel].max_bin_cnt;
        }

        // 计算每个group的pixel信息
        ctx->group_cfg[ab_idx].group[i].pixel_len = pixel_len;
        ctx->group_cfg[ab_idx].group[i].pixel_cnt = row_num * width;
        total_len += ctx->group_cfg[ab_idx].group[i].pixel_len * ctx->group_cfg[ab_idx].group[i].pixel_cnt;
    }

    // 计算每个group的addr_base
    uint32_t base_addr = 0;
    for (i = 0; i < parse_cfg->frm_cfg[ab_idx].group_cnt; i++) {
        for (int32_t j = 0; j < parse_cfg->frm_cfg[ab_idx].group_cnt; j++) {
            uint8_t phy_group_id = parse_cfg->frm_cfg[ab_idx].group_cfg[j].group_id - parse_cfg->frm_cfg[ab_idx].start_group;
            if (i == phy_group_id) {
                if (i == 0) {
                    ctx->group_cfg[ab_idx].group[j].addr_base = 0;
                } else {
                    ctx->group_cfg[ab_idx].group[j].addr_base = base_addr;
                }
                base_addr += ctx->group_cfg[ab_idx].group[j].pixel_len * ctx->group_cfg[ab_idx].group[j].pixel_cnt;
                break;
            }
        }
    }

    ctx->pixel_buffer.buffer_size = total_len;

    if (ctx->pixel_buffer.buffer) {
        free(ctx->pixel_buffer.buffer);
        ctx->pixel_buffer.buffer = NULL;
    }

    ctx->pixel_buffer.buffer = (uint8_t*)malloc(ctx->pixel_buffer.buffer_size);
    if (!ctx->pixel_buffer.buffer) {
        FS_LOGE("Failed to malloc pixel buffer, size=%u", ctx->pixel_buffer.buffer_size);
        return FS_ERR_NOMEM;
    }

    FL6x32_algo_roi_info_t info[8];
    FL6x32_algo_roi_info_t *ptr_info;
    uint8_t info_cnt[2] = { sizeof(info) / sizeof(info[0]), 0, };
    uint8_t *ptr_info_cnt;
    CHECK_ERR(algo_get_roi_info(info, info_cnt));

    if (!ab_idx) {
        ptr_info = &info[0];
        ptr_info_cnt = &info_cnt[0];
    }
    else {
        ptr_info = &info[info_cnt[0]];
        ptr_info_cnt = &info_cnt[1];
    }

    if (ctx->work_mode == FS_SDK_WORK_MODE_PCL) {
        data_info->data_type = DATA_TYPE_POINTCLOUD;
    }
    else {
        data_info->data_type = DATA_TYPE_CICL_DATA;
    }

    data_info->ab_frame_idx = (uint8_t)ab_idx;
    data_info->max_sub_frame_cnt = (1 << parse_cfg->frm_cfg[ab_idx].piece_roll_ctrl);
    data_info->peak_num = ptr_info->echo_peak_num;
    data_info->group_cnt = parse_cfg->frm_cfg[ab_idx].group_cnt;
    data_info->group_row_unit = 12;
    data_info->roi_cnt = ptr_info_cnt[0];

    for (i = 0; i < data_info->roi_cnt; i++) {
        data_info->ROI_info[i].width         = ptr_info[i].col_size;
        data_info->ROI_info[i].height        = ptr_info[i].row_size;
        data_info->ROI_info[i].bin_cnt       = ptr_info[i].rw_hist_length;
        data_info->ROI_info[i].col_unit      = ptr_info[i].binning_model[0];
        data_info->ROI_info[i].row_unit      = ptr_info[i].binning_model[1];
        data_info->ROI_info[i].group_start   = ptr_info[i].group_start;
        data_info->ROI_info[i].sub_frame_cnt = ptr_info[i].roll_mode == 0 ? 1 : ptr_info[i].roll_mode;

        if (parse_cfg->frm_cfg[ab_idx].pixel_cfg.ROI_cfg[i].bin_cnt_ns > 0) {
            data_info->ROI_info[i].range_max = parse_cfg->frm_cfg[ab_idx].auto_cfg.ROI_cfg[i].max_bin_cnt * (15.0f / parse_cfg->frm_cfg[ab_idx].pixel_cfg.ROI_cfg[i].bin_cnt_ns);
        }
        else {
            data_info->ROI_info[i].range_max = parse_cfg->frm_cfg[ab_idx].auto_cfg.ROI_cfg[i].max_bin_cnt * 15.0f;
        }

        // 根据 range_max 自动计算 pcl_unit，确保点云数据能压缩到 16bit 以内
        data_info->ROI_info[i].pcl_unit = fs_calc_pcl_unit(data_info->ROI_info[i].range_max, FS_PCL_UNIT_ROUND_CEIL);

        FS_LOGD("CICL Lidar Data Info[%d]: ROI[%d], width=%u, height=%u, max_bin_cnt=%u, range_max=%.0fcm, pcl_unit=0x%02x (%.5fcm)",
             ab_idx,
             i,
             data_info->ROI_info[i].width,
             data_info->ROI_info[i].height,
             data_info->max_bin_cnt,
             data_info->ROI_info[i].range_max,
             data_info->ROI_info[i].pcl_unit,
             fs_decode_pcl_unit(data_info->ROI_info[i].pcl_unit));
    }

    ret = algo_pcl_init_param(ab_idx, data_info->roi_cnt);

    if (ret != FS_SUCCESS) {
        FS_LOGE("Failed to init PCL params, ab_idx=%d, ret=%d", ab_idx, ret);
        return ret;
    }

    memset(data_info->peak_type, 0, sizeof(data_info->peak_type));

    if (parse_cfg->frm_cfg[ab_idx].pixel_cfg.en_pixel_data) {
        uint8_t peak_cnt = parse_cfg->frm_cfg[ab_idx].pixel_cfg.data_type_cfg.range_cfg.peak_cnt;

        if (g_algo_pcl_ctx.ab_ctx[ab_idx].algo_para.output_data_size == (sizeof(FL6x32_algo_pcl_output_t) / sizeof(float))) {
            peak_cnt = 1; // 如果算法输出数据结构是单峰的，则强制设置 peak_cnt 为 1
        }

        for (i = 0; i < peak_cnt; i++) {
            data_info->peak_type[i] = algo_conver_peak_type(parse_cfg->frm_cfg[ab_idx].pixel_cfg.data_type_cfg.range_cfg.algo_result_cfg[i].peak_type);
        }
    }

    return 0;
}

int32_t algo_cicl_analyze_slot_data(algo_context_t *ctx,        int32_t ab_idx,
                                    pkt_info_t     *pkt_info,   uint32_t pkt_cnt,
                                    pixel_data_t   *pixel_data, uint32_t pixel_cnt,
                                    gray_data_t    *gray_data , uint32_t gray_cnt,
                                    ev_info_t      *ev_info,    int32_t flag,
                                    uint64_t  timestamp)
{
    int32_t ret = FS_SUCCESS;
    uint16_t row, col;
    cicl_pix_info_t pix_info;
    rx_ROI_cfg_e ROI_sel;
    uint32_t pixel_len;
    uint32_t echo_bin_len;
    uint32_t vir_group_index, phy_group_index;
    uint8_t *buffer = NULL;
    uint8_t *pixel_buffer = NULL;
    uint32_t offset = 0;
    // uint32_t row_offset = 0;
    uint8_t row_num, col_num;
    uint8_t binning_type;
    cicl_fast_decode_ctx_t decode_ctx;
    bool use_fast_decode;
    bool is_pcl_mode;
    // uint16_t pixel_num_per_row;
    uint16_t pixel_cnt_per_slot;
    fs_lidar_data_info_t *data_info;
    uint16_t row_start;
    uint16_t group_cnt;
    const rx_parse_frm_cfg_t *frm_cfg = &ctx->parse_cfg->frm_cfg[ab_idx];

    UNUSED_VAR(pkt_cnt);
    UNUSED_VAR(gray_data);
    UNUSED_VAR(gray_cnt);
    UNUSED_VAR(ev_info);

    if (ctx->parse_ctx.slot_complete == false) {
        // FS_LOGW("Current slot is not complete, skip processing");
        return FS_SUCCESS;
    }

    data_info = &g_algo_pcl_ctx.ab_ctx[ab_idx].data_info;
    vir_group_index = pkt_info->vir_group_index;
    phy_group_index = pkt_info->phy_group_index;
    is_pcl_mode     = (ctx->work_mode == FS_SDK_WORK_MODE_PCL);
    ROI_sel         = frm_cfg->group_cfg[vir_group_index].ROI_sel;
    binning_type    = frm_cfg->pixel_cfg.ROI_cfg[ROI_sel].binning_type;
    row_start = data_info->ROI_info[ROI_sel].group_start * data_info->group_row_unit / data_info->ROI_info[ROI_sel].row_unit;
    group_cnt = frm_cfg->group_cnt;

    ret = rx_parse_get_row_col_num_in_piece(binning_type, &row_num, &col_num);

    if (ret != FS_SUCCESS) {
        FS_LOGE("Failed to get row/col num in piece, binning_type=%u", binning_type);
        return ret;
    }

    pixel_cnt_per_slot = frm_cfg->piece_cnt_per_slot * col_num * row_num;

    pixel_len    = ctx->group_cfg[ab_idx].group[vir_group_index].pixel_len;
    buffer       = ctx->pixel_buffer.buffer + ctx->group_cfg[ab_idx].group[vir_group_index].addr_base;

    /* ===== 像素循环常量提升：slot 内所有 pixel 共享同一 pkt_info ===== */
    /* 提前填充 pix_info.pkt_info，避免循环内每次通过 pixel_data[i].pkt_info 解引用 */
    pix_info.pkt_info.packet_type     = pkt_info->packet_type;
    pix_info.pkt_info.frame_index     = pkt_info->frame_index;
    pix_info.pkt_info.packet_id       = pkt_info->packet_id;
    pix_info.pkt_info.group_index     = pkt_info->phy_group_index;
    pix_info.pkt_info.slot_index      = pkt_info->slot_index;
    pix_info.pkt_info.timestamp       = pkt_info->timestamp;
    pix_info.pkt_info.ab_frame_sel    = pkt_info->ab_frame_sel;
    pix_info.pkt_info.sub_frame_idx   = pkt_info->sub_frame_idx;
    pix_info.pkt_info.piece_roll_mode = pkt_info->piece_roll_mode;
    pix_info.pkt_info.tdc_reso_mode   = pkt_info->tdc_reso_mode;
    pix_info.binning_type             = binning_type;
    pix_info.en_algo_echo_bin         = frm_cfg->pixel_cfg.data_type_cfg.range_cfg.en_echo_bin;
    /* pixel_buffer 内的 slot 基偏移（= slot_index × pixels_per_slot/roll_div） */
    const uint32_t slot_pix_base_offset =
        (uint32_t)pkt_info->slot_index *
        ((uint32_t)pixel_cnt_per_slot / (1u << (pkt_info->piece_roll_mode & 0x03u)));
    /* 预取 range_info 指针和 ROI 尺寸，避免循环内重复解引用嵌套结构 */
    FL6x32_range_pix_info_t *const roi_range_info =
        g_algo_pcl_ctx.ab_ctx[pkt_info->ab_frame_sel & 0x01].ROI_ctx[ROI_sel].range_info;
    uint32_t *const roi_timestamp_info =
        g_algo_pcl_ctx.ab_ctx[pkt_info->ab_frame_sel & 0x01].ROI_ctx[ROI_sel].timestamp_info;
    const uint16_t roi_width  = data_info->ROI_info[ROI_sel].width;
    const uint16_t roi_height = data_info->ROI_info[ROI_sel].height;
    const uint16_t row_limit = (uint16_t)(row_start + roi_height);
    const uint32_t pkt_timestamp = pkt_info->timestamp;
    const uint32_t range_stride_peak1 = RANGE_ONE_PIXEL_INFO_LEN(1);
    const uint32_t range_stride_peak2 = RANGE_ONE_PIXEL_INFO_LEN(2);
    const uint8_t roll_shift = (uint8_t)(pkt_info->piece_roll_mode & 0x03u);
    const uint32_t roll_div = (1u << roll_shift);
    const uint32_t roi_pixel_capacity = ((uint32_t)roi_width * (uint32_t)roi_height) / roll_div;

    memset(&decode_ctx, 0, sizeof(decode_ctx));
    decode_ctx.piece_map = cicl_get_piece_map(binning_type, pkt_info->piece_roll_mode, pkt_info->sub_frame_idx);
    decode_ctx.channel_cnt = frm_cfg->channel_cnt_per_slot;
    decode_ctx.ch_cnt_per_piece = (frm_cfg->half_piece_per_ch < 2) ? 2 : 1;
    decode_ctx.half_piece_per_ch = frm_cfg->half_piece_per_ch;
    decode_ctx.pix_cnt_per_chann = frm_cfg->pixel_cfg.ROI_cfg[ROI_sel].location_cfg.pix_cnt_per_channel;
    decode_ctx.piece_cnt_per_block = frm_cfg->piece_cnt_per_block;
    decode_ctx.piece_loop_cnt = (uint16_t)(frm_cfg->slot_cnt_per_group * frm_cfg->piece_cnt_per_block);
    decode_ctx.row_base = frm_cfg->group_cfg[vir_group_index].row_offset;

    if (frm_cfg->slot_cnt_per_group > 4) {
        uint8_t slot_loop_cnt = frm_cfg->slot_cnt_per_group / 4;
        decode_ctx.remap_slot_idx = (uint8_t)(((pkt_info->slot_index % slot_loop_cnt) * 4u) + (pkt_info->slot_index / slot_loop_cnt));
    } else {
        decode_ctx.remap_slot_idx = pkt_info->slot_index;
    }

    if (decode_ctx.channel_cnt > 0) {
        uint8_t pie_cnt_per_chann = (uint8_t)((frm_cfg->piece_cnt_per_slot + decode_ctx.channel_cnt - 1) / decode_ctx.channel_cnt);
        if (pie_cnt_per_chann > 0) {
            decode_ctx.pix_cnt_per_pie_in_ch = (uint8_t)(decode_ctx.pix_cnt_per_chann / pie_cnt_per_chann);
        }
    }
    rx_parse_get_row_col_num_in_piece(binning_type, &decode_ctx.row_num_in_piece, &decode_ctx.col_num_in_piece);
    use_fast_decode = (decode_ctx.piece_map != NULL) &&
                      (decode_ctx.channel_cnt > 0) &&
                      (decode_ctx.pix_cnt_per_pie_in_ch > 0) &&
                      (decode_ctx.piece_cnt_per_block > 0) &&
                      (decode_ctx.row_num_in_piece > 0);

    for (uint32_t i = 0; i < pixel_cnt; i++) {
        const pixel_data_t *pd = &pixel_data[i];

        /* 软件预取下一批 pixel_data 元素，隐藏顺序内存访问延迟 */
        if (__builtin_expect(i + 8 < pixel_cnt, 1)) {
            __builtin_prefetch(&pixel_data[i + 8], 0, 0);
        }

        if (use_fast_decode) {
            if (!cicl_decode_pixel_id_fast(&decode_ctx, pd->pixel_info.pixel_id, &row, &col)) {
                continue;
            }
        } else {
            rx_parse_decode_pixel_id(frm_cfg, &pixel_data[i], &row, &col);
        }

        if (row >= row_limit || col >= roi_width) {
            continue;
        }

        if (is_pcl_mode) {
            uint8_t peak_num = pd->peak_cnt;
            uint32_t pix_index = (((uint32_t)(row - row_start) * (uint32_t)roi_width) + (uint32_t)col) >> roll_shift;
            FL6x32_range_pix_info_t *pix_data;
            FL6x32_algo_result_t *dst;
            const range_result_t *r0;
            const range_result_t *r1;

            if (pix_index >= roi_pixel_capacity || peak_num == 0 || peak_num > 2) {
                continue;
            }

            if (peak_num == 1) {
                pix_data = (FL6x32_range_pix_info_t *)(((uint8_t *)roi_range_info) + pix_index * range_stride_peak1);
            } else {
                pix_data = (FL6x32_range_pix_info_t *)(((uint8_t *)roi_range_info) + pix_index * range_stride_peak2);
            }

            pix_data->pixel_id         = pd->pixel_info.pixel_id;
            pix_data->noise_amb        = pd->data.range_data.noise_amb >> 2;
            pix_data->amb_num          = pd->pixel_info.ambient_num;
            pix_data->det_sum          = pd->pixel_info.det_sum;
            pix_data->real_tx_num      = pd->pixel_info.real_tx_num;
            pix_data->lpwr_real_tx_num = pd->pixel_info.lpwr_real_tx_num;
            pix_data->peak_num         = (uint32_t)peak_num;
            pix_data->row              = row;
            pix_data->col              = col;
            roi_timestamp_info[pix_index] = pkt_timestamp;

            g_pixel_timestamp_offset_data[ROI_sel][row][col] = pkt_timestamp;

            dst = (FL6x32_algo_result_t *)pix_data->data;
            r0 = &pd->data.range_data.range_result[0];
            dst[0].th_peak_addr   = r0->peak_info.peak;
            dst[0].crosstalk      = r0->peak_info.crosstalk;
            dst[0].dist_rp        = r0->dist_rp;
            dst[0].fwhm           = r0->fwhm;
            dst[0].conv_intensity = r0->conv_intensity;
            dst[0].dist_cent      = r0->dist_cent;
            dst[0].hist_peak_max  = r0->hist_peak_max;

            if (peak_num == 2) {
                r1 = &pd->data.range_data.range_result[1];
                dst[1].th_peak_addr   = r1->peak_info.peak;
                dst[1].crosstalk      = r1->peak_info.crosstalk;
                dst[1].dist_rp        = r1->dist_rp;
                dst[1].fwhm           = r1->fwhm;
                dst[1].conv_intensity = r1->conv_intensity;
                dst[1].dist_cent      = r1->dist_cent;
                dst[1].hist_peak_max  = r1->hist_peak_max;
            }
            continue;
        }
        else {
            /* pix_info.pkt_info 已在循环外提前填充，此处仅更新 per-pixel 字段 */
            pix_info.pixel_id           = pd->pixel_info.pixel_id;
            pix_info.noise_amb          = pd->data.range_data.noise_amb >> 2;
            pix_info.amb_num            = pd->pixel_info.ambient_num;
            pix_info.det_sum            = pd->pixel_info.det_sum;
            pix_info.real_tx_num        = pd->pixel_info.real_tx_num;
            pix_info.lpwr_real_tx_num   = pd->pixel_info.lpwr_real_tx_num;
            pix_info.peak_num           = pd->peak_cnt;
            pix_info.row                = row;
            pix_info.col                = col;

            g_pixel_timestamp_offset_data[ROI_sel][row][col] = pkt_timestamp;
            echo_bin_len = 0;
            if (pix_info.en_algo_echo_bin) {
                for (int32_t j = 0; j < (int32_t)pix_info.peak_num; j++) {
                    pix_info.echo_sample_l[j] = pd->data.range_data.range_result[j].echo_info.bin_cnt_left;
                    pix_info.echo_sample_r[j] = pd->data.range_data.range_result[j].echo_info.bin_cnt_right;

                    if (pix_info.echo_sample_l[j] > 22) {
                        pix_info.echo_sample_l[j] = 22;
                    }

                    if (pix_info.echo_sample_r[j] > 22) {
                        pix_info.echo_sample_r[j] = 22;
                    }

                    echo_bin_len += (pix_info.echo_sample_l[j] + pix_info.echo_sample_r[j] + 1) * sizeof(uint16_t);
                }
            }

            pixel_buffer = buffer + (slot_pix_base_offset + pix_info.pixel_id) * pixel_len;
            memcpy(pixel_buffer, &pix_info, sizeof(cicl_pix_info_t));
            offset = sizeof(cicl_pix_info_t);

            for (int32_t j = 0; j < (int32_t)pix_info.peak_num; j++) {
                algo_result_unpack_t algo_result;
                algo_result.th_peak_addr = pd->data.range_data.range_result[j].peak_info.peak;
                algo_result.crosstalk = pd->data.range_data.range_result[j].peak_info.crosstalk;
                algo_result.dist_rp = pd->data.range_data.range_result[j].dist_rp;
                algo_result.fwhm = pd->data.range_data.range_result[j].fwhm;
                algo_result.conv_intensity = pd->data.range_data.range_result[j].conv_intensity;
                algo_result.dist_cent = pd->data.range_data.range_result[j].dist_cent;
                algo_result.hist_peak_max = pd->data.range_data.range_result[j].hist_peak_max;
                memcpy(pixel_buffer + offset, &algo_result, sizeof(algo_result_unpack_t));
                offset += sizeof(algo_result_unpack_t);
            }

            ret = algo_pcl_push_one_pixel(roi_range_info,
                                          &pix_info, row_start, roi_width, roi_height, ROI_sel,
                                          (algo_result_unpack_t *)(pixel_buffer + sizeof(pix_info)));

            if (pix_info.en_algo_echo_bin && echo_bin_len > 0) {
                uint32_t actual_buf_len = pd->buf_len;
                uint32_t src_offset = 0;

                for (int32_t j = 0; j < (int32_t)pix_info.peak_num; j++) {
                    uint32_t bin_len     = (pix_info.echo_sample_l[j] + pix_info.echo_sample_r[j] + 1) * sizeof(uint16_t);
                    uint32_t src_bin_len = (pd->data.range_data.range_result[j].echo_info.bin_cnt_left + pd->data.range_data.range_result[j].echo_info.bin_cnt_right + 1);
                    uint32_t skip = (pd->data.range_data.range_result[j].echo_info.bin_cnt_left - pix_info.echo_sample_l[j]);

                    if (src_offset + src_bin_len > actual_buf_len) {
                        FS_LOGE("Echo bin buffer overflow: pixel_id=%u, peak=%d, src_offset=%u, src_bin_len=%u, actual_buf_len=%u (expected=%u)",
                                pix_info.pixel_id, j, src_offset, src_bin_len, actual_buf_len, echo_bin_len);
                        break;
                    }

                    memcpy(pixel_buffer + offset,
                              pd->buffer + src_offset + skip,
                           bin_len);
                    offset += bin_len;
                    src_offset += src_bin_len;
                }
            }
        }
    }

    if (is_pcl_mode) {
        g_algo_pcl_ctx.ab_ctx[pkt_info->ab_frame_sel & 0x01].ROI_ctx[ROI_sel].updated_frame_idx = pkt_info->frame_index;
        g_algo_pcl_ctx.ab_ctx[pkt_info->ab_frame_sel & 0x01].ROI_ctx[ROI_sel].updated_sub_frame_idx = pkt_info->sub_frame_idx;
    }

    if (cicl_lidar_frm_info.find_start_frm) {
        if (ctx->work_mode == FS_SDK_WORK_MODE_PCL) {
            if (flag == FS_ERR_GROUP_CHANGE || flag == FS_SUCCESS) {
                // if (pkt_info->frame_index != cicl_lidar_frm_info.cur_lidar_frm_idx) {
                if ((pkt_info->vir_group_index == data_info->group_cnt - 1) && (pkt_info->slot_index == frm_cfg->slot_cnt_per_group - 1)) {
                    pcl_process_task_t *task = NULL;
                    uint8_t ab_frame_sel;
                    uint32_t range_data_len;
                    int32_t roi_idx;
                    uint8_t frame_index_gap = 0u;

                    if (cicl_next_frame_index(cicl_lidar_frm_info.cur_lidar_frm_idx) != pkt_info->frame_index) {
                        FS_LOGE("Lidar frame index lost, last_idx=%u, new_idx=%u",
                                cicl_lidar_frm_info.cur_lidar_frm_idx, pkt_info->frame_index);
                        frame_index_gap = 1u;
                    }

                    cicl_lidar_frm_info.cur_lidar_frm_idx = pkt_info->frame_index;
                    ab_frame_sel = pkt_info->ab_frame_sel & 0x01;

                    if (!g_algo_pcl_ctx.algo_init || !g_pcl_task_queue) {
                        return FS_ERR_BAD_PARAMS;
                    }

                    data_info = &g_algo_pcl_ctx.ab_ctx[ab_frame_sel].data_info;

#if FS_CFG_FEATURE_MCU_TIMESYNC
                    {
                        static uint32_t s_last_anchor_generation = 0u;
                        static uint32_t s_last_bound_frame_id = 0u;
                        static uint64_t s_last_bound_frame_base_timestamp_us = 0u;

                        uint32_t anchor_generation = spad_anchor_get_generation();
                        uint64_t frame_base_timestamp_us = 0u;

                        if (frame_index_gap != 0u) {
                            FS_LOGW("SPAD frame anchor bind risk: frame index lost before bind, "
                                    "frame_id=%u generation=%u",
                                    pkt_info->frame_index,
                                    anchor_generation);
                        }

                        if (s_last_anchor_generation != anchor_generation ||
                            s_last_bound_frame_id != pkt_info->frame_index) {
                            int bind_ret;

                            bind_ret = spad_anchor_bind_next_frame(pkt_info->frame_index);
                            if (bind_ret == FS_SUCCESS) {
                                s_last_anchor_generation = anchor_generation;
                                s_last_bound_frame_id = pkt_info->frame_index;
                                s_last_bound_frame_base_timestamp_us =
                                    cicl_resolve_frame_timestamp_us(pkt_info->frame_index);
                            } else {
                                uint64_t fallback_ts =
                                    cicl_resolve_frame_timestamp_us(pkt_info->frame_index);

                                if (fallback_ts != 0u) {
                                    s_last_anchor_generation = anchor_generation;
                                    s_last_bound_frame_id = pkt_info->frame_index;
                                    s_last_bound_frame_base_timestamp_us = fallback_ts;
                                } else {
                                    FS_LOGW("SPAD frame anchor bind miss: frame_id=%u ret=%d generation=%u",
                                            pkt_info->frame_index,
                                            bind_ret,
                                            anchor_generation);

                                    /*
                                     * Do not cache a failed bind. This allows a later call for the same
                                     * frame_id to retry if the frame_sync anchor arrives slightly later.
                                     */
                                    s_last_anchor_generation = anchor_generation;
                                    s_last_bound_frame_id = 0u;
                                    s_last_bound_frame_base_timestamp_us = 0u;
                                }
                            }
                        }

                        frame_base_timestamp_us = s_last_bound_frame_base_timestamp_us;
#else
                    {
                        uint64_t frame_base_timestamp_us =
                            cicl_resolve_frame_timestamp_us(pkt_info->frame_index);
#endif

                    for (roi_idx = 0; roi_idx < data_info->roi_cnt; roi_idx++) {
                        uint8_t cur_buf_idx;
                        uint8_t next_buf_idx;
                        uint8_t next_busy;

                        if (g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].updated_frame_idx != pkt_info->frame_index ||
                            g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].updated_sub_frame_idx != pkt_info->sub_frame_idx) {
                            continue;
                        }

                        if (g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].enqueued_frame_idx == pkt_info->frame_index &&
                            g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].enqueued_sub_frame_idx == pkt_info->sub_frame_idx) {
                            continue;
                        }

                        range_data_len = g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].range_data_len;

                        task = (pcl_process_task_t *)malloc(sizeof(pcl_process_task_t));
                        if (!task) {
                            FS_LOGE("Failed to allocate PCL task for frame %u", pkt_info->frame_index);
                            continue;
                        }

                        /* 双缓冲交接：当前缓冲交给消费者，切换到下一个缓冲继续写 */
                        cur_buf_idx = g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].write_buf_idx;
                        next_buf_idx = (uint8_t)(cur_buf_idx ^ 0x01);
                        next_busy = __atomic_load_n(
                            &g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].buf_in_use[next_buf_idx],
                            __ATOMIC_ACQUIRE);
                        if (next_busy) {
                            FS_LOGW("PCL double-buffer busy for ROI %d, dropping frame %u",
                                    roi_idx, pkt_info->frame_index);
                            free(task);
                            continue;
                        }

                        __atomic_store_n(
                            &g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].buf_in_use[cur_buf_idx],
                            1,
                            __ATOMIC_RELEASE);

                        task->range_data = (uint8_t *)g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].range_buf[cur_buf_idx];
                        task->range_data_len    = range_data_len;
                        task->range_buf_idx     = cur_buf_idx;
                        task->frame_index       = pkt_info->frame_index;
                        task->ab_frame_idx      = ab_frame_sel;
                        task->max_sub_frame_cnt = (1 << ctx->parse_cfg->frm_cfg[ab_frame_sel].piece_roll_ctrl);
                        task->roi_index         = roi_idx & 0x03;
                        task->vir_group_id      = vir_group_index;
                        task->phy_group_id      = phy_group_index;
                        task->sub_frame_cnt     = (1 << pkt_info->piece_roll_mode);
                        task->sub_frame_idx     = pkt_info->sub_frame_idx;
                        task->group_cnt         = group_cnt;
                        task->frame_base_timestamp = frame_base_timestamp_us;

                        g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].write_buf_idx = next_buf_idx;
                        g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].range_info =
                            g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].range_buf[next_buf_idx];
                        g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].timestamp_info =
                            g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].timestamp_buf[next_buf_idx];

                        // 提交到队列（非阻塞）
                        ret = queue_push_timeout(g_pcl_task_queue, task, 0);  // 0ms超时=非阻塞
                        if (ret != FS_SUCCESS) {
                            FS_LOGW("PCL task queue full, dropping frame %u", pkt_info->frame_index);
                            __atomic_store_n(
                                &g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].buf_in_use[cur_buf_idx],
                                0,
                                __ATOMIC_RELEASE);
                            free(task);
                            /* 恢复写缓冲，避免后续写入切到忙缓冲 */
                            g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].write_buf_idx = cur_buf_idx;
                            g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].range_info =
                                g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].range_buf[cur_buf_idx];
                            g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].timestamp_info =
                                g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].timestamp_buf[cur_buf_idx];
                        } else {
                            g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].enqueued_frame_idx = pkt_info->frame_index;
                            g_algo_pcl_ctx.ab_ctx[ab_frame_sel].ROI_ctx[roi_idx].enqueued_sub_frame_idx = pkt_info->sub_frame_idx;
                        }
                    }
                    }
                }
            }
        }
        else {
            // FS_LOGD("CICL Data Analyze: frame_index=%5u, group_id=%2u, slot_id=%u, sub_f=%u, roll=%u, flag=%4d",
            //          pkt_info->frame_index, vir_group_index, pkt_info[0].slot_index, pkt_info[0].sub_frame_idx, pkt_info[0].piece_roll_mode, flag);

            // 用静态变量记录当前周期是否发送
            // 每 4 个周期发送 3 个周期（75% 利用率），确保发送连续完整的 sub_frame_idx 0~n-1
            static uint8_t s_cicl_cycle_cnt = 0;     // 周期计数器 (0~3)
            static uint16_t s_last_frame_index = 0;

            // 检测新周期开始：当 sub_frame_idx == 0 且 frame_index 变化时
            if (pkt_info->sub_frame_idx == 0 && pkt_info->frame_index != s_last_frame_index) {
                s_cicl_cycle_cnt = (s_cicl_cycle_cnt + 1) % 4;  // 0, 1, 2, 3 循环
            }
            s_last_frame_index = pkt_info->frame_index;

            // 当 s_cicl_cycle_cnt != 3 时发送（即 0, 1, 2 发送，3 跳过）
            // 这样每 4 个周期发送 3 个，流量约 75%
            if (s_cicl_cycle_cnt != 3) {
                if (flag == FS_ERR_GROUP_CHANGE || flag == FS_SUCCESS) {
                    // 发送UDP数据包
                    data_info->data_type = DATA_TYPE_CICL_DATA;
                    data_info->group_info.vir_group_idx = vir_group_index;
                    data_info->group_info.phy_group_idx = phy_group_index;
                    data_info->group_info.pixel_len = pixel_len;
                    data_info->group_info.pixel_count = ctx->group_cfg[ab_idx].group[vir_group_index].pixel_cnt / (1 << (pkt_info->piece_roll_mode & 0x03)); // 真实点数应当使用当前 group 的子帧数量计算
                    data_info->group_info.roi_index = ROI_sel & 0x03;
                    data_info->ROI_info[ROI_sel].sub_frame_cnt = (1 << (pkt_info->piece_roll_mode & 0x03));
                    data_info->ROI_info[ROI_sel].sub_frame_idx = pkt_info->sub_frame_idx;
                    data_info->data = buffer;
                    data_info->frm_id = pkt_info->frame_index;

                    // 深拷贝数据到新分配的内存，避免多线程竞态
                    do {
                        fs_lidar_data_info_t* data_copy = NULL;
                        uint8_t* data_buffer = NULL;
                        uint32_t data_size = data_info->group_info.pixel_count * data_info->group_info.pixel_len;

                        data_copy = (fs_lidar_data_info_t*)malloc(sizeof(fs_lidar_data_info_t));
                        if (!data_copy) {
                            FS_LOGE("Failed to allocate memory for CICL data_copy");
                            ret = FS_ERR_NOMEM;
                            break;
                        }

                        data_buffer = (uint8_t*)malloc(data_size);
                        if (!data_buffer) {
                            FS_LOGE("Failed to allocate memory for CICL data_buffer, size=%u", data_size);
                            free(data_copy);
                            ret = FS_ERR_NOMEM;
                            break;
                        }

                        memcpy(data_copy, data_info, sizeof(fs_lidar_data_info_t));
                        memcpy(data_buffer, data_info->data, data_size);
                        data_copy->data = data_buffer;

                        ret = algo_pixel_queue_push_data(data_copy);
                        if (ret != FS_SUCCESS) {
                            FS_LOGE("Failed to send CICL data one group, ret=%d", ret);
                            free(data_buffer);
                            free(data_copy);
                            return ret;
                        }
                    } while (0);
                }
            }
        }
    }

    return ret;
}

int32_t algo_cicl_data_analyze(algo_context_t* ctx, int32_t vi, uint8_t *data, uint32_t data_len, uint64_t timestamp)
{
    int32_t ret = FS_SUCCESS;
    uint32_t csi_width = 0;
    uint32_t csi_height = 0;
    const uint16_t* next_ptr = NULL;
    const uint16_t* data_end = NULL;
    uint32_t total_pkt_cnt = 0;
    uint32_t total_pixel_cnt = 0;
    uint32_t total_gray_cnt = 0;

    CHECK_PTR(ctx);
    CHECK_PTR(data);

    csi_width = ctx->parse_cfg->frm_cfg[vi].width;
    csi_height = ctx->parse_cfg->frm_cfg[vi].height;

    if (csi_width * csi_height * sizeof(uint16_t) > data_len) {
        return FS_ERR_BAD_PARAMS;
    }

    next_ptr = (const uint16_t* )data;
    data_end = (const uint16_t*)data + csi_width * csi_height;

    // FS_LOGI("Analyzing CICL data: width=%u, height=%u, data_len=%u", csi_width, csi_height, data_len);

    while (next_ptr < data_end) {
        uint32_t remain_len = (uint32_t)(data_end - next_ptr);
        uint32_t max_pkt_cnt = (csi_width == 0) ? 0 : (remain_len / csi_width);
        if (max_pkt_cnt > csi_height) {
            max_pkt_cnt = csi_height;
        }

        pkt_info_t* pkt_info = NULL;
        uint32_t pkt_cnt = 0;
        pixel_data_t* pixel_data = NULL;
        uint32_t pixel_cnt = 0;
        gray_data_t* gray_data = NULL;
        uint32_t gray_cnt = 0;
        ev_info_t ev_info = {0};

        ret = rx_parse_one_frame(&ctx->parse_ctx, ctx->parse_cfg, next_ptr, remain_len, csi_width, max_pkt_cnt, &pkt_info,
                                 &pkt_cnt, &pixel_data, &pixel_cnt, &gray_data, &gray_cnt, &ev_info, &next_ptr);

        total_pkt_cnt += pkt_cnt;
        total_pixel_cnt += pixel_cnt;
        total_gray_cnt += gray_cnt;

        if (ret == FS_ERR_SLOT_CHANGE || ret == FS_ERR_GROUP_CHANGE) {
            // 继续解析下一个 slot 片段
            if (pixel_data->pkt_info->vir_group_index == 0 && pixel_data->pkt_info->slot_index == 0) {
                if (!cicl_lidar_frm_info.find_start_frm) {
                    cicl_lidar_frm_info.find_start_frm = true;
                }
            }

            if (!cicl_lidar_frm_info.find_start_frm) {
                // 未找到起始帧，丢弃数据
                continue;
            }

            algo_cicl_analyze_slot_data(ctx, pkt_info->ab_frame_sel, pkt_info, pkt_cnt, pixel_data, pixel_cnt, gray_data, gray_cnt, &ev_info, ret, timestamp);
            continue;
        } else if (ret == FS_ERR_FRAME_CHANGE) {
            // 完成一帧
            if (!cicl_lidar_frm_info.find_start_frm) {
                // 未找到起始帧，丢弃数据
                continue;
            }
            break;
        } else if (ret == FS_SUCCESS) {
            // 原始数据解析完成
            if (!cicl_lidar_frm_info.find_start_frm) {
                // 未找到起始帧，丢弃数据
                continue;
            }

            algo_cicl_analyze_slot_data(ctx, pkt_info->ab_frame_sel, pkt_info, pkt_cnt, pixel_data, pixel_cnt, gray_data, gray_cnt, &ev_info, ret, timestamp);
            // FS_LOGI("CICL Data Analyze Completed: pkt_cnt=%u, pixel_cnt=%u, gray_cnt=%u",
            //          pkt_cnt, pixel_cnt, gray_cnt);

            break;
        } else {
            // 其他错误直接失败
            FS_LOGE("Unexpected parse error: %d", ret);
            break;
        }
    }

    return 0;
}

int32_t algo_cicl_data_process(algo_context_t* ctx, int32_t vi, uint8_t *data, uint32_t data_len, uint64_t timestamp)
{
    int32_t ret;

    ret = algo_cicl_data_analyze(ctx, vi, data, data_len, timestamp);
    if (ret < 0) {
        return ret;
    }

    // if (cicl_lidar_frm_info.lidar_frm_ready) {
    //     // FS_LOGI("lidar frame ready..");

    //     if (g_sdk_config.lidar_status == FS_LIDAR_WORK_STATUS_CICL || g_sdk_config.lidar_status == FS_LIDAR_WORK_STATUS_CICL_ECHO) {
    //         if (cicl_lidar_frm_id % 2 == 0) {
    //             ret = algo_cicl_data_udp_send();
    //         }
    //     }
    //     else if (g_sdk_config.lidar_status == FS_LIDAR_WORK_STATUS_PCL) {
    //         // fs_timingcost(false);
    //         ret = algo_pcl_process();
    //         // FS_LOGI("algo cost time : %d ms", fs_timingcost(true));
    //     }
    // }

    return ret;
}
