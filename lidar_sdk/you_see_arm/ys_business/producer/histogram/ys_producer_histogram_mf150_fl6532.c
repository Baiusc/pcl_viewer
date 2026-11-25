/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-07-08 19:10:30
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-15 09:20:00
 * @FilePath     : /you_see_arm/ys_business/producer/histogram/ys_producer_histogram_mf150.c
 * @Description  : 直方图生产者（主动获取+零拷贝）- 双线程架构
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#include "../../../core/ys_base.h"
#include "../../../core/ys_core_system.h"

#include <pthread.h>

#define MF150_RW_PACKET_SKIP_SIZE 4
#define MF150_RW_PACKET_TAIL_SKIP_SIZE 26
#define MF150_RW_PACKET_SIZE 456
#define MF150_RW_PACKET_HEADER_SIZE 17
#define MF150_PIXEL_DATA_SIZE 439
#define MF150_PIXELS_PER_GROUP 32
#define MF150_GROUP_GAP_SIZE 32

// 队列大小（必须是2的幂次方）
// 注意：驱动只有 10 个 buffer，队列不应太大以避免 buffer 全在队列中
#define MF150_QUEUE_SIZE 8

// 单帧原始数据最大尺寸：855组 × (32像素×456字节 + 32字节组间隙) ≈ 12.5MB，取13MB留余量
#define MF150_MAX_RAW_FRAME_SIZE (13 * 1024 * 1024)

// worker线程专用暂存区（单线程使用，无需多份）
// 用于在归还v4l2 buffer之前将数据拷出，避免内存池满时worker线程长期持有v4l2 buffer
static uint8_t g_raw_frame_staging[MF150_MAX_RAW_FRAME_SIZE];

static atomic_uint_fast8_t g_run_flag; // 线程运行标识
static udp_t *g_udp_instance = NULL;

// 大帧frame_id计数与内部小帧状态
static uint32_t g_big_frame_id = 0;
static uint8_t g_small_frame_in_big = 0; // 0=期望SF0(新大帧), 1=期望SF1

// 队列项：存储从驱动获取的buffer信息
typedef struct {
	uint8_t *start; // buffer起始地址
	size_t length; // buffer长度
	uint32_t buf_index; // v4l2 buffer索引（用于put_buffer）
} mf150_queue_item_t;

// 简单的环形队列（单生产者单消费者，无锁）
static mf150_queue_item_t g_data_queue[MF150_QUEUE_SIZE];
static atomic_uint_fast32_t g_queue_head = 0; // 生产者写入位置
static atomic_uint_fast32_t g_queue_tail = 0; // 消费者读取位置

// 工作线程
static pthread_t g_worker_thread;
static atomic_uint_fast8_t g_worker_run_flag;

void producer_exit_histogram()
{
	atomic_store(&g_run_flag, STOPPED);
	atomic_store(&g_worker_run_flag, STOPPED);
	return;
}

#if LOCAL_TEST == 1
static void producer_get_histogram_from_file(udp_t *udp_instance)
{
	while (atomic_load(&(g_run_flag)) == RUNNING) {
		sleep(1);
	}
	return;
}
#else

// 检查队列是否有数据可读
static inline bool queue_has_data()
{
	return atomic_load(&g_queue_head) != atomic_load(&g_queue_tail);
}

// 从队列获取一个item
static bool queue_get(mf150_queue_item_t *item)
{
	uint32_t tail = atomic_load(&g_queue_tail);
	if (tail == atomic_load(&g_queue_head)) {
		return false; // 队列为空
	}
	*item = g_data_queue[tail & (MF150_QUEUE_SIZE - 1)];
	atomic_store(&g_queue_tail, tail + 1);
	return true;
}

// 向队列放入一个item
static bool queue_put(mf150_queue_item_t *item)
{
	uint32_t head = atomic_load(&g_queue_head);
	uint32_t tail = atomic_load(&g_queue_tail);

	// 检查队列是否已满
	if ((head - tail) >= MF150_QUEUE_SIZE) {
		return false; // 队列已满
	}

	g_data_queue[head & (MF150_QUEUE_SIZE - 1)] = *item;
	atomic_store(&g_queue_head, head + 1);
	return true;
}

// ==========================================================
// 核心修复：根据第一个像素的 row_pos 判断 SF 类型
// ==========================================================
// 数据组织方式：
//   SF0: row 范围 0~7, 16~23, 32~39, ... (row_pos/8 是偶数)
//   SF1: row 范围 8~15, 24~31, 40~47, ... (row_pos/8 是奇数)
#if 0
static int determine_actual_sf(uint8_t *data, size_t data_len)
{
	if (data_len < MF150_RW_PACKET_SIZE) {
		return 0; // 默认 SF0
	}

	mf150_rw_packet_t *pkt = (mf150_rw_packet_t *)data;
	if (pkt->data_type != 0x5257) {
		return 0; // 数据异常，默认 SF0
	}

	// 根据 row_pos 判断 SF 类型
	// row_pos / 8 得到组行号，% 2 判断是偶数组(SF0)还是奇数组(SF1)
	int row_group = pkt->row_pos / 8;
	int sf_type = row_group % 2;

	return sf_type; // 0=SF0, 1=SF1
}
#endif

// 工作线程：处理数据搬运到内存池
static void *worker_thread_func(void *arg)
{
	log_info("[MF150] Worker thread started");

	mf150_queue_item_t item;

	while (atomic_load(&g_worker_run_flag) == RUNNING) {
		if (!queue_has_data()) {
			usleep(1000); // 队列为空，等待1ms
			continue;
		}

#if 0
		if (!queue_get(&item)) {
			continue;
		}

		// 安全检查：确保 buffer 有效
		if (item.start == NULL || item.length == 0) {
			if (item.buf_index < 10) { // V4L2_DEV_BUF_NUM = 10
				V4l2Buffer dataBuf;
				dataBuf.start = item.start;
				dataBuf.length = item.length;
				dataBuf.index = item.buf_index;
				v4l2_put_buffer(&dataBuf);
			}
			continue;
		}

		// 关键修复：将 v4l2 buffer 中的原始数据拷贝到本地暂存区后立即归还 buffer。
		// 背景：allocate_memory() 在内存池耗尽时会自旋等待（usleep(1) 循环），
		// 若 worker 在持有 v4l2 buffer 期间进入自旋，会导致驱动 10 帧环形缓冲区耗尽，
		// 进而使 SF1 帧无法被驱动交付（只剩 SF0），sf_type 永远为 0。
		// 提前归还 v4l2 buffer 后，即使后续 allocate_memory 自旋，驱动也不受影响。
		size_t data_len = 0;
		if (item.length > (size_t)(MF150_RW_PACKET_SKIP_SIZE + MF150_RW_PACKET_TAIL_SKIP_SIZE)) {
			data_len = item.length - MF150_RW_PACKET_SKIP_SIZE - MF150_RW_PACKET_TAIL_SKIP_SIZE;
		}
		if (data_len > MF150_MAX_RAW_FRAME_SIZE) {
			data_len = MF150_MAX_RAW_FRAME_SIZE;
		}
		memcpy(g_raw_frame_staging, item.start + MF150_RW_PACKET_SKIP_SIZE, data_len);

		// 立即归还 v4l2 buffer，驱动可继续填充后续帧（SF1 不再被饿死）
		{
			V4l2Buffer dataBuf;
			dataBuf.start = item.start;
			dataBuf.length = item.length;
			dataBuf.index = item.buf_index;
			v4l2_put_buffer(&dataBuf);
		}

		// 后续处理全部基于暂存区，v4l2 buffer 已释放
		uint8_t *data = g_raw_frame_staging;

		// 获取当前 buffer 的真实物理帧类型
		int actual_sf = determine_actual_sf(data, data_len);

		if (actual_sf == 0) {
			// SF0 → 标志着新大帧的开始
			if (g_small_frame_in_big != 0) {
				// 如果之前状态是1（正常的上一大帧结束）或发生丢帧，现在开始全新一帧
				g_big_frame_id++;
			}
			g_small_frame_in_big = 0;
		} else {
			// SF1 → 标志着大帧的下半场
			g_small_frame_in_big = 1;
		}

		log_debug("[MF150] Processing small frame, sf_type=%d, frame_id=%u", actual_sf,
			  (uint16_t)g_big_frame_id);

		uint8_t *cur = data;
		size_t remaining = data_len;
		int packets_in_group = 0;

		while (remaining >= MF150_RW_PACKET_SIZE) {
			if (atomic_load(&g_worker_run_flag) != RUNNING) {
				// v4l2 buffer 已在上方提前归还，此处直接退出
				return NULL;
			}

			// 使用非阻塞版本：内存池耗尽时立即返回 NULL 而非自旋等待，
			// 避免 worker 线程被卡住导致 v4l2 buffer 长期不归还。
			node_t *p_node = (node_t *)try_allocate_memory(g_udp_instance->pool);

			if (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL) {
				// 内存池耗尽，跳过像素
				for (int skip_i = 0;
				     skip_i < MF150_PIXELS_PER_SEND && remaining >= MF150_RW_PACKET_SIZE; skip_i++) {
					cur += MF150_RW_PACKET_SIZE;
					remaining -= MF150_RW_PACKET_SIZE;
					packets_in_group++;

					if (packets_in_group >= MF150_PIXELS_PER_GROUP &&
					    remaining >= MF150_GROUP_GAP_SIZE) {
						cur += MF150_GROUP_GAP_SIZE;
						remaining -= MF150_GROUP_GAP_SIZE;
						packets_in_group = 0;
					}
				}
				continue;
			}

			mf150_histogram_packet_t *buf = (mf150_histogram_packet_t *)(p_node->data->buf);
			buf->sof = 0xAA;
			buf->frame_id = g_big_frame_id;

			int actual_pixels = 0;

			for (int i = 0; i < MF150_PIXELS_PER_SEND && remaining >= MF150_RW_PACKET_SIZE; i++) {
				mf150_rw_packet_t *rw_packet = (mf150_rw_packet_t *)cur;

				if (rw_packet->data_type != 0x5257) {
					break;
				}

				mf150_pixel_info_t *pixel_info =
					(mf150_pixel_info_t *)(buf->value + i * MF150_PIXEL_INFO_SIZE);
				pixel_info->row_pos = rw_packet->row_pos;
				pixel_info->col_pos = rw_packet->col_pos;

				Memcpy(pixel_info->data, cur + MF150_RW_PACKET_HEADER_SIZE, MF150_PIXEL_DATA_SIZE);

				cur += MF150_RW_PACKET_SIZE;
				remaining -= MF150_RW_PACKET_SIZE;
				packets_in_group++;
				actual_pixels++;

				if (packets_in_group >= MF150_PIXELS_PER_GROUP && remaining >= MF150_GROUP_GAP_SIZE) {
					cur += MF150_GROUP_GAP_SIZE;
					remaining -= MF150_GROUP_GAP_SIZE;
					packets_in_group = 0;
				}
			}

			buf->data_len = actual_pixels * MF150_PIXEL_INFO_SIZE;
			p_node->data->len = sizeof(mf150_histogram_packet_t) - 1 + buf->data_len;
			atomic_store(&(p_node->write_ok), READABLE);
		}

		// v4l2 buffer 已在数据拷贝后立即归还，此处无需再次释放
#endif
	}

	log_info("[MF150] Worker thread exiting");
	return NULL;
}

// 生产者线程：从驱动获取数据，放入队列
// 策略：处理连续的2帧（一个完整大帧），跳过8帧
static void producer_get_histogram_from_driver_mf150(udp_t *udp_instance)
{
	log_info("[MF150] Producer thread started (dual-thread mode, 2fps)");

	g_udp_instance = udp_instance;
	g_big_frame_id = 0;
	g_small_frame_in_big = 0;

	// 初始化队列
	atomic_store(&g_queue_head, 0);
	atomic_store(&g_queue_tail, 0);

	// 启动工作线程
	atomic_store(&g_worker_run_flag, RUNNING);
	if (pthread_create(&g_worker_thread, NULL, worker_thread_func, NULL) != 0) {
		log_error("[MF150] Failed to create worker thread");
		return;
	}

#if 0
	V4l2Buffer dataBuf;
	int frame_counter = 0; // 周期计数器：0~9，处理0~1，跳过2~9

	while (atomic_load(&g_run_flag) == RUNNING) {
		// 主动获取数据buffer
		int ret = v4l2_get_buffer(&dataBuf);
		if (ret != 0) {
			usleep(1000);
			continue;
		}

		if (dataBuf.start == NULL || dataBuf.length == 0) {
			v4l2_put_buffer(&dataBuf);
			continue;
		}

		// 周期逻辑：处理帧0和帧1，跳过帧2~9
		if (frame_counter < 2) {
			// 处理帧：放入队列让工作线程处理
			mf150_queue_item_t item;
			item.start = dataBuf.start;
			item.length = dataBuf.length;
			item.buf_index = dataBuf.index;

			if (!queue_put(&item)) {
				log_warn("[MF150] Queue full, dropping frame");
				v4l2_put_buffer(&dataBuf);
			}
			log_debug("[MF150] Processing frame %d", frame_counter);
		} else {
			// 跳过帧：立即归还buffer
			v4l2_put_buffer(&dataBuf);
			log_debug("[MF150] Skipping frame %d", frame_counter);
		}

		// 更新计数器，0~9循环
		frame_counter++;
		if (frame_counter >= 10) {
			frame_counter = 0;
		}
	}
#else
	while (atomic_load(&g_run_flag) == RUNNING) {
		usleep(1000);
	}
#endif

	// 等待工作线程退出
	atomic_store(&g_worker_run_flag, STOPPED);
	pthread_join(g_worker_thread, NULL);

	log_info("[MF150] Producer thread exiting");
}
#endif // LOCAL_TEST

void producer_get_histogram(udp_t *udp_instance)
{
	atomic_init(&g_run_flag, RUNNING);
	atomic_init(&g_worker_run_flag, RUNNING);

	if (udp_instance == NULL) {
		return;
	}

#if LOCAL_TEST == 1
	producer_get_histogram_from_file(udp_instance);
#else
	producer_get_histogram_from_driver_mf150(udp_instance);
#endif

	return;
}
