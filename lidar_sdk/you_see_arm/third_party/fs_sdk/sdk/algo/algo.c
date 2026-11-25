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
#include <sys/mman.h>
#include <pthread.h>

#include "fs_log.h"
#include "fs_err.h"
#include "concurrent_queue.h"
#include "rxdata_service.h"
#include "algo_cicl.h"
#include "algo_gray_level.h"
#include "algo.h"
#include "rx_cfg_loader.h"
#include "fs_hw.h"
#include "Lidar_FL6x32_algo.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "Algo"

#define ALGO_BUFFER_SIZE (56 * 1024 * 1024)

static algo_context_t algo_ctx;

int32_t algo_get_roi_info(FL6x32_algo_roi_info_t *info, uint8_t *info_cnt)
{
    FL6x32_algo_roi_info_t *roi;
    const rx_parse_cfg_t* parse_cfg = algo_ctx.parse_cfg;
    const rx_parse_frm_cfg_t* frm_cfg;
    uint8_t binning_type;
    uint8_t row_spad, col_spad;
    uint8_t row_pixel, col_pixel;
    uint32_t ab_idx;
    uint32_t max_frm_cnt = 1;
    uint8_t roi_cnt[2] = { 0 , 0 };
    uint8_t roi_idx;

    CHECK_PTR(info);
    CHECK_PTR(info_cnt);

    if (parse_cfg->is_ab_frame) {
        max_frm_cnt = 2;
    }

    for (ab_idx = 0; ab_idx < max_frm_cnt; ab_idx++) {
        frm_cfg = &parse_cfg->frm_cfg[ab_idx];

        if (!frm_cfg->pixel_cfg.en_pixel_data) {
            roi_cnt[ab_idx] = 1;
            continue;
        }

        roi_cnt[ab_idx] = frm_cfg->pixel_cfg.ROI_cnt;
    }

    if (info_cnt[0] + info_cnt[1] < roi_cnt[0] + roi_cnt[1]) {
        FS_LOGE("roi info buffer size insufficient!");
        return FS_ERR_BAD_PARAMS;
    }

    info_cnt[0] = 0;
    info_cnt[1] = 0;

    roi_idx = 0;
    for (ab_idx = 0; ab_idx < max_frm_cnt; ab_idx++) {
        frm_cfg = &parse_cfg->frm_cfg[ab_idx];

        for (uint8_t i = 0; i < roi_cnt[ab_idx]; i++) {
            roi = (FL6x32_algo_roi_info_t *)(info + roi_idx);

            binning_type = frm_cfg->pixel_cfg.ROI_cfg[i].binning_type;
            rx_parse_get_pixel_binning(binning_type, &row_spad, &col_spad);
            rx_parse_get_row_col_num_in_piece(binning_type, &row_pixel, &col_pixel);

            roi->piece_start      = 0;
            roi->col_start        = 0;
            roi->binning_model[0] = col_spad;
            roi->binning_model[1] = row_spad;
            roi->col_size         = parse_cfg->frm_cfg[ab_idx].piece_cnt_per_slot * parse_cfg->frm_cfg[ab_idx].slot_cnt_per_group * col_pixel;
            roi->rw_hist_length   = frm_cfg->pixel_cfg.ROI_cfg[i].hist_cfg.main_hist_cnt + frm_cfg->pixel_cfg.ROI_cfg[i].hist_cfg.sub_hist_cnt;
            roi->rw_data_size     = roi->rw_hist_length + 4;

            if (parse_cfg->frm_cfg[ab_idx].pixel_cfg.en_pixel_data) {
                roi->echo_peak_width = 1;

                if (parse_cfg->frm_cfg[ab_idx].pixel_cfg.is_range_data) {
                    roi->echo_peak_num = parse_cfg->frm_cfg[ab_idx].pixel_cfg.data_type_cfg.range_cfg.peak_cnt;

                    if (frm_cfg->pixel_cfg.ROI_cfg[i].echo_cfg[0].bin_cnt_left > 22) {
                        roi->echo_peak_width += 22;
                    }
                    else {
                        roi->echo_peak_width += frm_cfg->pixel_cfg.ROI_cfg[i].echo_cfg[0].bin_cnt_left;
                    }

                    if (frm_cfg->pixel_cfg.ROI_cfg[i].echo_cfg[0].bin_cnt_right > 22) {
                        roi->echo_peak_width += 22;
                    }
                    else {
                        roi->echo_peak_width += frm_cfg->pixel_cfg.ROI_cfg[i].echo_cfg[0].bin_cnt_right;
                    }
                }
                else {
                    roi->echo_peak_num = parse_cfg->frm_cfg[ab_idx].pixel_cfg.data_type_cfg.hist_cfg.peak_cnt;
                    roi->echo_peak_width += frm_cfg->pixel_cfg.ROI_cfg[i].echo_cfg[0].bin_cnt_left;
                    roi->echo_peak_width += frm_cfg->pixel_cfg.ROI_cfg[i].echo_cfg[0].bin_cnt_right;
                }
            }
            else {
                roi->echo_peak_num   = 0;
                roi->echo_peak_width = 0;
            }

            roi->TDC              = frm_cfg->pixel_cfg.ROI_cfg[i].bin_cnt_ns;
            roi->roll_mode        = (frm_cfg->pixel_cfg.ROI_cfg[i].sub_frame_cnt == 1) ? 0 : frm_cfg->pixel_cfg.ROI_cfg[i].sub_frame_cnt;
            roi->roll_idx         = 0;

            roi->row_size = 0;
            for (int32_t j = 0; j < frm_cfg->group_cnt; j++) {
                if (i == frm_cfg->group_cfg[j].ROI_sel) {
                    roi->row_size += row_pixel;
                }
            }

            roi->group_start = 0xFFFF;
            roi->row_start   = 0xFFFF;

            for (int32_t j = 0; j < frm_cfg->group_cnt; j++) {
                if (i == frm_cfg->group_cfg[j].ROI_sel) {
                    uint8_t  group_start;
                    uint32_t row_offset;
                    group_start = frm_cfg->group_cfg[j].group_id - frm_cfg->start_group;
                    row_offset  = frm_cfg->group_cfg[j].row_offset;

                    // 查找当前 ROI 的起始位置
                    if (roi->group_start > group_start) {
                        roi->group_start = group_start;
                        roi->row_start   = (uint16_t)row_offset;
                    }
                }
            }

            roi_idx++;
        }
    }

    info_cnt[0] = roi_cnt[0];
    info_cnt[1] = roi_cnt[1];
    return FS_SUCCESS;
}

/**
 * @brief Get roi msg
 *
 * @param info roi msg
 * @return int32_t error code
 */
int32_t algo_get_roi_msg(FL6x32_algo_roi_msg_t* info, uint32_t *info_size)
{
    uint8_t roi_cnt[2];

    CHECK_PTR(info);
    CHECK_PTR(info_size);

    if (*info_size < sizeof(FL6x32_algo_roi_msg_t)) {
        FS_LOGE("roi msg buffer size insufficient!");
        return FS_ERR_BAD_PARAMS;
    }

    roi_cnt[0] = *info_size / sizeof(FL6x32_algo_roi_info_t);
    roi_cnt[1] = 0;
    *info_size = 0;
    memset(info, 0, sizeof(*info));
    CHECK_ERR(algo_get_roi_info((FL6x32_algo_roi_info_t *)info->data, roi_cnt));
    info->version       = RX_ROI_INFO_VERSION;
    info->roi_cnt_a_frm = roi_cnt[0];
    info->roi_cnt_b_frm = roi_cnt[1];
    *info_size = sizeof(FL6x32_algo_roi_msg_t) + (roi_cnt[0] + roi_cnt[1]) * sizeof(FL6x32_algo_roi_info_t);
    return FS_SUCCESS;
}

static int32_t algo_data_process(algo_context_t* ctx, int32_t vi, uint8_t *data, uint32_t data_len, uint64_t timestamp)
{
    int32_t ret = FS_SUCCESS;
    static uint32_t counter = 0;
    const rx_parse_cfg_t* parse_cfg = ctx->parse_cfg;

    if (parse_cfg->frm_cfg[vi].pixel_cfg.en_pixel_data) {
        if (parse_cfg->frm_cfg[vi].pixel_cfg.is_range_data) {
            if (counter++ % 50 == 0) {
                uint8_t cnt = 1;
                int32_t temp;
                float rx_temp;

                if ((fs_hw_get_adc(ADC_TYPE_TEMP_RX, ADC_DATA_CALIB_PARA, &temp, &cnt, NULL) == FS_SUCCESS) && (cnt == 1)) {
                    rx_temp = temp / 10.0f;
                    // FS_LOGD("Current RX temperature: %.1f C", rx_temp);
                    algo_update_tx_temp(rx_temp);
                    algo_update_rx_temp(&rx_temp, 1);
                }
                else {
                    FS_LOGE("Failed to get RX temperature from ADC");
                }
            }

            ret = algo_cicl_data_process(ctx, vi, data, data_len, timestamp);
        }
    }
    else {
        ret = algo_gray_data_process(ctx, vi, data, data_len);
    }

    return ret;
}

static void *algo_run(void *arg)
{
    algo_data_queue_t* data_queue = NULL;

    (void)arg;

    while (algo_ctx.algo_thread_run_flag) {
        // 使用超时 pop 从辅助函数获取数据，避免被永久阻塞
        algo_queue_pop_data(&data_queue, 100);

        // 检查运行标志
        if (!algo_ctx.algo_thread_run_flag) {
            // 退出前释放刚获取的数据（如果有）
            if (data_queue) {
                rxdata_service_release_frame(data_queue->frame);
                free(data_queue->frame);
                free(data_queue);
            }
            FS_LOGD("algo_run: thread exiting (flag=0)");
            break;
        }

        if (!data_queue) {
            // 超时或被唤醒返回 NULL，继续循环检查 flag
            continue;
        }

        // 处理接收到的帧数据
        algo_data_process(&algo_ctx, data_queue->vi, data_queue->data, data_queue->data_len, data_queue->timestamp);
        rxdata_service_release_frame(data_queue->frame);
        free(data_queue->frame);
        free(data_queue);
    }

    FS_LOGD("algo_run: thread exited");
    return NULL;
}

static void *algo_pixel_run(void *arg)
{
    fs_lidar_data_info_t* data_queue = NULL;

    (void)arg;

    while (algo_ctx.algo_thread_run_flag) {
        // 使用超时 pop 从辅助函数获取数据，避免被永久阻塞
        algo_pixel_queue_pop_data(&data_queue, 100);

        // 检查运行标志
        if (!algo_ctx.algo_thread_run_flag) {
            // 退出前释放刚获取的数据（如果有）
            if (data_queue) {
                if (data_queue->data) {
                    free(data_queue->data);
                }
                free(data_queue);
            }
            FS_LOGD("algo_pixel_run: thread exiting (flag=0)");
            break;
        }

        if (!data_queue) {
            // 超时或被唤醒返回 NULL，继续循环检查 flag
            continue;
        }
        fs_send_pixel_data_one_group(data_queue);

        // 释放由深拷贝分配的内存
        if (data_queue->data) {
            free(data_queue->data);
            data_queue->data = NULL;
        }
        free(data_queue);
        data_queue = NULL;
    }

    FS_LOGD("algo_pixel_run: thread exited");
    return NULL;
}

static void *algo_gray_run(void *arg)
{
    fs_lidar_data_info_t* data_queue = NULL;

    (void)arg;

    while (algo_ctx.algo_thread_run_flag) {
        // 使用超时 pop 从辅助函数获取数据，避免被永久阻塞
        algo_gray_queue_pop_data(&data_queue, 100);

        // 检查运行标志
        if (!algo_ctx.algo_thread_run_flag) {
            // 退出前释放刚获取的数据（如果有）
            if (data_queue) {
                if (data_queue->data) {
                    free(data_queue->data);
                }
                free(data_queue);
            }
            FS_LOGD("algo_gray_run: thread exiting (flag=0)");
            break;
        }

        if (!data_queue) {
            // 超时或被唤醒返回 NULL，继续循环检查 flag
            continue;
        }

        fs_send_gray_data_one_group(data_queue);

        // 释放由深拷贝分配的内存，保持与 pixel 线程一致的生命周期管理
        if (data_queue->data) {
            free(data_queue->data);
            data_queue->data = NULL;
        }
        free(data_queue);
        data_queue = NULL;
    }

    FS_LOGD("algo_gray_run: thread exited");
    return NULL;
}

int32_t algo_init(const rx_parse_cfg_t *rx_parse, fs_sdk_work_mode_e work_mode)
{
    int32_t ret = FS_SUCCESS;

    memset(&algo_ctx, 0, sizeof(algo_context_t));
    algo_ctx.algo_thread_run_flag = 1;

    // ============ 重置 PCL 全局变量，防止旧线程残留 ============
    algo_pcl_reset_globals();

    algo_ctx.work_mode = work_mode;
    algo_ctx.parse_cfg = rx_parse;
    if (!algo_ctx.parse_cfg) {
        FS_LOGE("Failed to get parse configuration");
        return FS_ERR_BAD_PARAMS;
    }

    ret = rx_frame_parse_ctx_init(&algo_ctx.parse_ctx, algo_ctx.parse_cfg);
    if (ret != FS_SUCCESS) {
        FS_LOGE("Failed to initialize frame parse context");
        return ret;
    }

    algo_ctx.algo_queue = queue_create(100);
    if (!algo_ctx.algo_queue) {
        FS_LOGE("Failed to create algorithm queue");
        ret = FS_ERR_NOMEM;
        goto init_fail;
    }

    algo_ctx.pixel_queue = queue_create(100);
    if (!algo_ctx.pixel_queue) {
        FS_LOGE("Failed to create pixel queue");
        ret = FS_ERR_NOMEM;
        goto init_fail;
    }

    algo_ctx.gray_queue = queue_create(100);
    if (!algo_ctx.gray_queue) {
        FS_LOGE("Failed to create gray queue");
        ret = FS_ERR_NOMEM;
        goto init_fail;
    }

    const rx_parse_cfg_t* parse_cfg = algo_ctx.parse_cfg;

    for (int32_t ab_idx = 0; ab_idx < RX_MAX_FRAME_CFG; ab_idx++) {
        if (!parse_cfg->frm_cfg[ab_idx].is_b_frame && ab_idx == 1) {
            continue;
        } else if (parse_cfg->frm_cfg[ab_idx].is_b_frame && ab_idx == 0) {
            continue;
        }

        if (parse_cfg->frm_cfg[ab_idx].pixel_cfg.en_pixel_data) {
            if (parse_cfg->frm_cfg[ab_idx].pixel_cfg.is_range_data) {
                ret = algo_cicl_calc_cfg(&algo_ctx, ab_idx);
            }
        }
        else {
            ret = algo_gray_calc_cfg(&algo_ctx, ab_idx);
        }

        if (ret != FS_SUCCESS) {
            FS_LOGE("Failed to initialize algo config, ab_idx=%d, ret=%d", ab_idx, ret);
            goto init_fail;
        }
    }

    if (pthread_create(&algo_ctx.algo_thread_id, NULL, algo_run, NULL) != 0) {
        FS_LOGE("Failed to create algorithm thread");
        ret = FS_ERR_INTERNAL;
        goto init_fail;
    }

    // if (pthread_create(&algo_ctx.pixel_thread_id, NULL, algo_pixel_run, NULL) != 0) {
    //     FS_LOGE("Failed to create pixel thread");
    //     ret = FS_ERR_INTERNAL;
    //     goto init_fail;
    // }

    if (pthread_create(&algo_ctx.gray_thread_id, NULL, algo_gray_run, NULL) != 0) {
        FS_LOGE("Failed to create gray thread");
        ret = FS_ERR_INTERNAL;
        goto init_fail;
    }

    FS_LOGI("Algorithm thread started");
    return 0;

init_fail:
    algo_deinit();
    return ret;
}

int32_t algo_deinit(void)
{
    algo_ctx.algo_thread_run_flag = 0;

    // 先保存句柄，防止并发访问时读取不一致
    ConcurrentQueue* algo_queue = algo_ctx.algo_queue;
    ConcurrentQueue* pixel_queue = algo_ctx.pixel_queue;
    ConcurrentQueue* gray_queue = algo_ctx.gray_queue;
    pthread_t algo_thread_id = algo_ctx.algo_thread_id;
    pthread_t pixel_thread_id = algo_ctx.pixel_thread_id;
    pthread_t gray_thread_id = algo_ctx.gray_thread_id;

    FS_LOGD("algo_deinit: algo_queue=%p, pixel_queue=%p, gray_queue=%p",
            algo_queue, pixel_queue, gray_queue);

    // 唤醒所有阻塞在队列中的线程：广播条件变量
    if (algo_queue) {
        pthread_mutex_lock(&algo_queue->mutex);
        pthread_cond_broadcast(&algo_queue->not_empty);
        pthread_mutex_unlock(&algo_queue->mutex);
    }
    if (pixel_queue) {
        pthread_mutex_lock(&pixel_queue->mutex);
        pthread_cond_broadcast(&pixel_queue->not_empty);
        pthread_mutex_unlock(&pixel_queue->mutex);
    }
    if (gray_queue) {
        pthread_mutex_lock(&gray_queue->mutex);
        pthread_cond_broadcast(&gray_queue->not_empty);
        pthread_mutex_unlock(&gray_queue->mutex);
    }

    // 等待线程退出（检查线程 ID 是否有效）
    pthread_t self = pthread_self();
    if (algo_thread_id && !pthread_equal(algo_thread_id, self)) {
        FS_LOGD("algo_deinit: joining algo_thread");
        pthread_join(algo_thread_id, NULL);
    }
    if (pixel_thread_id && !pthread_equal(pixel_thread_id, self)) {
        FS_LOGD("algo_deinit: joining pixel_thread");
        pthread_join(pixel_thread_id, NULL);
    }
    if (gray_thread_id && !pthread_equal(gray_thread_id, self)) {
        FS_LOGD("algo_deinit: joining gray_thread");
        pthread_join(gray_thread_id, NULL);
    }

    FS_LOGD("algo_deinit: all threads joined");

    // ============ 步骤 2: 线程全部退出后，再清理 PCL 异步处理线程 ============
    // 避免 algo_run 仍在访问 g_algo_pcl_ctx 时被提前释放导致崩溃
    FS_LOGD("algo_deinit: stopping PCL process thread...");
    algo_pcl_uninit(0);  // 清理 AB 帧 0，同时清理 PCL 线程及任务队列
    algo_pcl_uninit(1);  // 清理 AB 帧 1（如果存在）
    FS_LOGD("algo_deinit: PCL process thread stopped");

    // 清空算法队列并释放所有残留的帧数据
    if (algo_queue) {
        algo_data_queue_t* data_queue = NULL;
        while ((data_queue = (algo_data_queue_t*)queue_pop_timeout(algo_queue, 0)) != NULL) {
            // 释放帧资源
            rxdata_service_release_frame(data_queue->frame);
            free(data_queue->frame);
            free(data_queue);
        }
        queue_destroy(algo_queue);
        FS_LOGD("algo_deinit: algo_queue destroyed");
    }

    // 清空 pixel 队列
    if (pixel_queue) {
        fs_lidar_data_info_t* data = NULL;
        while ((data = (fs_lidar_data_info_t*)queue_pop_timeout(pixel_queue, 0)) != NULL) {
            // pixel_queue 中的数据由其他模块管理
        }
        queue_destroy(pixel_queue);
        FS_LOGD("algo_deinit: pixel_queue destroyed");
    }

    // 清空 gray 队列
    if (gray_queue) {
        fs_lidar_data_info_t* data = NULL;
        while ((data = (fs_lidar_data_info_t*)queue_pop_timeout(gray_queue, 0)) != NULL) {
            // gray_queue 中的数据由其他模块管理
        }
        queue_destroy(gray_queue);
        FS_LOGD("algo_deinit: gray_queue destroyed");
    }

    if (algo_ctx.pixel_buffer.buffer) {
        free(algo_ctx.pixel_buffer.buffer);
        algo_ctx.pixel_buffer.buffer = NULL;
    }

    if (algo_ctx.gray_buffer.buffer) {
        free(algo_ctx.gray_buffer.buffer);
        algo_ctx.gray_buffer.buffer = NULL;
    }

    rx_frame_parse_ctx_destroy(&algo_ctx.parse_ctx);

    // 最后清零句柄，标记 deinit 完成
    algo_ctx.algo_queue = NULL;
    algo_ctx.pixel_queue = NULL;
    algo_ctx.gray_queue = NULL;
    algo_ctx.algo_thread_id = 0;
    algo_ctx.pixel_thread_id = 0;
    algo_ctx.gray_thread_id = 0;

    FS_LOGI("Algorithm thread stopped");
    return 0;
}

int32_t algo_queue_push_data(algo_data_queue_t* data)
{
    if (!data) {
        return FS_ERR_BAD_PARAMS;
    }

    // 检查队列是否已被清零（algo_deinit 中）
    if (!algo_ctx.algo_queue) {
        FS_LOGE("Algo queue is not initialized");
        return FS_ERR_INTERNAL;
    }

    // 将数据推送到处理队列
    if (queue_push(algo_ctx.algo_queue, (void*)data) < 0) {
        FS_LOGE("Failed to push pixel data to algorithm queue");
        return FS_ERR_INTERNAL;
    }

    return 0;
}

int32_t algo_queue_pop_data(algo_data_queue_t** data, int timeout_ms)
{
    if (!data) {
        return FS_ERR_BAD_PARAMS;
    }

    // 检查队列是否已被初始化
    if (!algo_ctx.algo_queue) {
        *data = NULL;
        return FS_ERR_INTERNAL;
    }

    *data = (algo_data_queue_t*)queue_pop_timeout(algo_ctx.algo_queue, timeout_ms);
    return 0;  // 返回 0 表示调用成功，*data 可能为 NULL（超时）
}

int32_t algo_pixel_queue_push_data(fs_lidar_data_info_t* data)
{
    if (!data) {
        return FS_ERR_BAD_PARAMS;
    }

    // 检查队列是否已被清零（algo_deinit 中）
    if (!algo_ctx.pixel_queue) {
        FS_LOGE("Pixel queue is not initialized");
        return FS_ERR_INTERNAL;
    }

    // 将数据推送到处理队列
    if (queue_push(algo_ctx.pixel_queue, (void*)data) < 0) {
        FS_LOGE("Failed to push pixel data to algorithm queue");
        return FS_ERR_INTERNAL;
    }

    return 0;
}

int32_t algo_pixel_queue_pop_data(fs_lidar_data_info_t** data, int timeout_ms)
{
    if (!data) {
        return FS_ERR_BAD_PARAMS;
    }

    // 检查队列是否已被初始化
    if (!algo_ctx.pixel_queue) {
        *data = NULL;
        return FS_ERR_INTERNAL;
    }

    *data = (fs_lidar_data_info_t*)queue_pop_timeout(algo_ctx.pixel_queue, timeout_ms);
    return 0;  // 返回 0 表示调用成功，*data 可能为 NULL（超时）
}

int32_t algo_gray_queue_push_data(fs_lidar_data_info_t* data)
{
    if (!data) {
        return FS_ERR_BAD_PARAMS;
    }

    // 检查队列是否已被清零（algo_deinit 中）
    if (!algo_ctx.gray_queue) {
        FS_LOGE("Gray queue is not initialized");
        return FS_ERR_INTERNAL;
    }

    // 将数据推送到处理队列
    if (queue_push(algo_ctx.gray_queue, (void*)data) < 0) {
        FS_LOGE("Failed to push gray data to algorithm queue");
        return FS_ERR_INTERNAL;
    }

    return 0;
}

int32_t algo_gray_queue_pop_data(fs_lidar_data_info_t** data, int timeout_ms)
{
    if (!data) {
        return FS_ERR_BAD_PARAMS;
    }

    // 检查队列是否已被初始化
    if (!algo_ctx.gray_queue) {
        *data = NULL;
        return FS_ERR_INTERNAL;
    }

    *data = (fs_lidar_data_info_t*)queue_pop_timeout(algo_ctx.gray_queue, timeout_ms);
    return 0;  // 返回 0 表示调用成功，*data 可能为 NULL（超时）
}

int32_t algo_update_tx_temp(float temp_tx)
{
    return algo_pcl_update_tx_temp(temp_tx);
}

int32_t algo_update_rx_temp(const float *temp_rx, uint32_t temp_rx_cnt)
{
    return algo_pcl_update_rx_temp(temp_rx, temp_rx_cnt);
}

uint8_t fs_calc_pcl_unit(float range_max, fs_pcl_unit_round_mode_t mode)
{
    if (range_max <= 0.0f) {
        return 0;
    }

    // 计算每个单位代表的实际距离 (cm)
    float unit = range_max / 32767.0f;

    // 分离整数部分和小数部分
    uint8_t integer_part = (uint8_t)unit;
    float frac = unit - integer_part;

    // 将小数部分编码为 5bit (0.00000 ~ 0.96875, 步进 0.03125)
    uint8_t frac_bits;
    if (mode == FS_PCL_UNIT_ROUND_CEIL) {
        // 进一法：只要有小数就向上取整
        if (frac > 0.0f) {
            frac_bits = (uint8_t)(frac * 32.0f);  // 直接截断
            if (frac_bits < (frac * 32.0f)) {
                frac_bits += 1;  // 向上取整
            }
            if (frac_bits > 31) {
                frac_bits = 0;
                integer_part += 1;
            }
        } else {
            frac_bits = 0;
        }
    }
    else {
        // 四舍五入（默认）
        frac_bits = (uint8_t)(frac * 32.0f + 0.5f);
        if (frac_bits > 31) {
            frac_bits = 0;
            integer_part += 1;
        }
    }

    // 编码：bit[6:5] 整数部分，bit[4:0] 小数部分
    return ((integer_part & 0x03) << 5) | (frac_bits & 0x1F);
}

float fs_decode_pcl_unit(uint8_t pcl_unit)
{
    uint8_t integer_part = (pcl_unit >> 5) & 0x03;
    uint8_t frac_bits = pcl_unit & 0x1F;
    float frac = frac_bits * 0.03125f;
    return (float)integer_part + frac;
}

fs_pcl_peak_type_t algo_conver_peak_type(uint32_t cfg_type)
{
    fs_pcl_peak_type_t peak_type;

    switch (cfg_type) {
        case RX_PEAK_TYPE_1ST_STRONG: {
            peak_type = FS_PCL_PEAK_1ST_STRONG;
            break;
        }

        case RX_PEAK_TYPE_2ND_STRONG: {
            peak_type = FS_PCL_PEAK_2ND_STRONG;
            break;
        }

        case RX_PEAK_TYPE_3RD_STRONG: {
            peak_type = FS_PCL_PEAK_3RD_STRONG;
            break;
        }

        case RX_PEAK_TYPE_4TH_STRONG: {
            peak_type = FS_PCL_PEAK_4TH_STRONG;
            break;
        }

        case RX_PEAK_TYPE_1ST_CLOSE: {
            peak_type = FS_PCL_PEAK_1ST_CLOSE;
            break;
        }

        case RX_PEAK_TYPE_2ND_CLOSE: {
            peak_type = FS_PCL_PEAK_2ND_CLOSE;
            break;
        }

        case RX_PEAK_TYPE_3RD_CLOSE: {
            peak_type = FS_PCL_PEAK_3RD_CLOSE;
            break;
        }

        case RX_PEAK_TYPE_4TH_CLOSE: {
            peak_type = FS_PCL_PEAK_4TH_CLOSE;
            break;
        }

        case RX_PEAK_TYPE_1ST_FAR: {
            peak_type = FS_PCL_PEAK_1ST_FAR;
            break;
        }

        case RX_PEAK_TYPE_UNDEF:
        default: {
            peak_type = FS_PCL_PEAK_NONE;
            break;
        }
    }

    return peak_type;
}
