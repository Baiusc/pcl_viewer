/*
 * @Description  : FL6532 Range(原始测距)数据生产者
 *                 与 point_cloud 生产者平行: 消费 fs_sdk pixel 队列中
 *                 data_type=0x2E(DATA_TYPE_CICL_DATA) 的 Range 数据,
 *                 按 MTU 分片写入内存池, 交由 consumer 重组为 fs 协议帧发送上位机。
 *
 *                 与 PCL 生产者(ys_producer_point_cloud_mf150_fl6532.c)的差异:
 *                 PCL 经算法库合并后每帧仅 push 1 次(单 group/单 sub_frame);
 *                 Range 每帧由 44 组 x 子帧构成多次 push(ROI1 组 4 个 sub_frame),
 *                 因此帧边界/序号状态机按 (vir_group_idx==0 && sub_frame_idx==0)
 *                 + frm_id 变化判定, data_sequence 跨 group/sub_frame 帧内连续递增。
 */

#include "../../../core/ys_base.h"
#include "../../../core/ys_conf.h"
#include "../../../core/ys_compress.h"
#include "../../../core/ys_core_def.h"
#include "../../../core/ys_core_system.h"
#include "../../../core/ys_memory_pool.h"

#include "../../../third_party/fs_sdk/sdk/net_server/inc/fs_net_protocol.h"
#include "../../../third_party/fs_sdk/sdk/src/rx_cfg/rx_cfg_loader.h"
#include "../../../third_party/fs_sdk/sdk/algo/inc/algo.h"

#include <stdbool.h>
#include <time.h>

#include "ys_producer_range_mf150_fl6532.h"

static atomic_uint_fast8_t g_run_flag; // 线程运行标识

#if RANGE_MEASURE_ONLY == 0
/* ---- 跨 push(组/子帧)帧状态(仅发送流程使用) ---- */
static uint16_t g_next_start_sequence = 0; // 本帧下一个包的 data_sequence
static uint16_t g_max_sequence = 0; // 已见过的最大 sequence
static int32_t g_max_group_num = 0; // 一帧虚拟 group 总数(缓存)
static uint16_t s_last_frm_id = 0xFFFF; // 上一个 push 的帧号(帧首判定), 初始无效值保证首帧识别

/* ---- 帧级调试统计 ---- */
static uint32_t dbg_frame_pkt_total = 0; // 本帧产生的 UDP 包总数
static uint32_t dbg_frame_dot_total = 0; // 本帧总点数
static uint32_t g_frame_cnt = 0; // 已收帧计数(低频统计用)

/* 帧统计打印间隔(帧), 10fps 时约每 10s 打印一次, 避免刷屏 */
#define RANGE_FRAME_STAT_INTERVAL 100

/* LZ4 压缩发送参数 */
#define RANGE_TX_LZ4_LEVEL 0 // 压缩级别(RV1126B 实测: level 3 HC 会压垮单核)
#define RANGE_TX_LZ4_BLOCK_KB 256 // LZ4F 内部块大小(KB)
/* 压缩包固定总长: 必须小于以太网 MTU(1500-20-8=1472), 否则 IP 分片会显著放大丢包;
 * 统一长度便于接收端处理, 最后一包不足时补 0, 有效长度见 payload_len */
#define RANGE_TX_LZ4_PKT_SIZE 1400
#endif // RANGE_MEASURE_ONLY == 0

void producer_exit_range()
{
	atomic_store(&g_run_flag, STOPPED);
	return;
}

#if LOCAL_TEST == 0
#if RANGE_MEASURE_ONLY == 0
/**
 * @description  : 判断某 push 是否为完整一帧的最后一个 push(帧尾单元)
 *                 vir_group_idx 达帧末组 且 到达该组最后一个 sub_frame
 */
static bool is_frame_last_push(const fs_lidar_data_info_t *dq, uint8_t roi_index)
{
	uint8_t sub_frame_cnt = dq->ROI_info[roi_index].sub_frame_cnt;

	if (sub_frame_cnt == 0) {
		return false;
	}
	if ((int32_t)dq->group_info.vir_group_idx != (g_max_group_num - 1)) {
		return false;
	}
	if (dq->ROI_info[roi_index].sub_frame_idx != (sub_frame_cnt - 1)) {
		return false;
	}
	return true;
}

/**
 * @description  : 从 fs_sdk 队列中消费 Range 数据并分片
 * @param         {udp_t} *udp_instance
 * @return        {}
 */
static void producer_get_range_from_driver(udp_t *udp_instance)
{
	fs_lidar_data_info_t *data_queue = NULL;
	node_t *p_node;
	mf150_fl6532_raw_group_packet_t *buf;

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		algo_pixel_queue_pop_data(&data_queue, 100);
		if (!data_queue) {
			continue;
		}

		/* 缓存一帧的虚拟 group 总数(与 PCL 生产者一致, Range 使用同一 RX preset) */
		if (g_max_group_num == 0) {
			int32_t all_group_num = 0;
			const rx_parse_cfg_t *parse_cfg;
			rx_cfg_get_rx_parse(RX_LIDAR_MODE_RANGE_01, &parse_cfg);
			uint8_t frame_cfg_cnt = (uint8_t)parse_cfg->is_ab_frame + 1;
			for (int32_t ab_idx = 0; ab_idx < frame_cfg_cnt; ab_idx++) {
				if (!parse_cfg->frm_cfg[ab_idx].is_b_frame && ab_idx == 1) {
					continue;
				} else if (parse_cfg->frm_cfg[ab_idx].is_b_frame && ab_idx == 0) {
					continue;
				}
				all_group_num += parse_cfg->frm_cfg[ab_idx].group_cnt;
			}
			g_max_group_num = all_group_num;
			log_info("[FL6532-RANGE] frame group_cnt=%d", g_max_group_num);
		}

		uint16_t pixel_len = data_queue->group_info.pixel_len;
		uint32_t data_count = data_queue->group_info.pixel_count;
		uint8_t roi_index = data_queue->group_info.roi_index & 0x03;

		/* 新帧判定: 虚拟组回到 0 且子帧回到 0, 且帧号较上一 push 变化
		 * (Range 模式下 ROI-1 组按 4 个 sub_frame 分多次 push, 不能只凭 sub==0
		 *  判帧首; 帧号可能因 SDK 帧级节流而跳变) */
		if ((data_queue->group_info.vir_group_idx == 0) &&
		    (data_queue->ROI_info[roi_index].sub_frame_idx == 0) && (data_queue->frm_id != s_last_frm_id)) {
			/* 低频打印上一帧统计(帧号跳变 = SDK 帧级节流丢帧, 属正常现象) */
			if ((g_frame_cnt % RANGE_FRAME_STAT_INTERVAL) == 0) {
				log_info("[FL6532-RANGE] frame stat: frm=%u pkt_total=%u dot_total=%u (%.1f MB/frame)",
					 s_last_frm_id, dbg_frame_pkt_total, dbg_frame_dot_total,
					 (dbg_frame_pkt_total * (double)(FS_PROTOCOL_UDP_MTU_SIZE + 28)) /
						 (1024.0 * 1024.0));
			}
			g_frame_cnt++;
			g_next_start_sequence = 0;
			g_max_sequence = 0;
			dbg_frame_pkt_total = 0;
			dbg_frame_dot_total = 0;
		}
		s_last_frm_id = data_queue->frm_id;

		int32_t remain_dot_num = (int32_t)data_count;
		int32_t max_dot_num =
			(FS_PROTOCOL_UDP_MTU_SIZE - sizeof(fs_protocol_head_t) - sizeof(fs_lidar_pcl_echo_data_t)) /
			pixel_len;
		if (max_dot_num == 0) {
			free(data_queue->data);
			free(data_queue);
			continue;
		}
		int32_t pkt_num = (data_count % max_dot_num) ? (data_count / max_dot_num + 1) :
							       (data_count / max_dot_num);

		bool frame_last_push = is_frame_last_push(data_queue, roi_index);

		for (int32_t j = 0; j < pkt_num; j++) {
			uint16_t seq = g_next_start_sequence + j;
			uint8_t frame_flag = FS_PCL_FRAME_FLAG_NORMAL;
			if ((0 == j) && (data_queue->group_info.vir_group_idx == 0) &&
			    (data_queue->ROI_info[roi_index].sub_frame_idx == 0)) {
				frame_flag = FS_PCL_FRAME_FLAG_START;
			} else if ((j == (pkt_num - 1)) && frame_last_push) {
				frame_flag = FS_PCL_FRAME_FLAG_END;
			} else if ((j == (pkt_num - 1)) && (seq == g_max_sequence)) {
				/* 兜底: 保留 max_sequence 启发式(未取到帧末 group 信息时) */
				frame_flag = FS_PCL_FRAME_FLAG_END;
			}

			uint16_t dot_num = (remain_dot_num >= max_dot_num) ? (uint16_t)max_dot_num :
									     (uint16_t)remain_dot_num;
			remain_dot_num -= dot_num;

			do {
				p_node = (node_t *)allocate_memory(udp_instance->pool);
				if (atomic_load(&(g_run_flag)) != RUNNING) {
					free(data_queue->data);
					free(data_queue);
					return;
				}
			} while (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL);
			buf = (mf150_fl6532_raw_group_packet_t *)(p_node->data->buf);
			memset(buf, 0, sizeof(mf150_fl6532_raw_group_packet_t));

			buf->sof = 0xAA;
			buf->frame_id = data_queue->frm_id;
			buf->group_id = data_queue->group_info.vir_group_idx;
			buf->data_len =
				sizeof(fs_protocol_head_t) + sizeof(fs_lidar_pcl_echo_data_t) + dot_num * pixel_len;
			buf->timestamp = data_queue->timestamp_base;
			buf->data_type = data_queue->data_type;

			/* === echo 平铺字段 === */
			buf->echo_protocol_ver_minor = PROTOCOL_PCL_VER_MINOR;
			buf->echo_data_sequence = seq;
			buf->echo_frame_cnt = data_queue->frm_id;
			buf->echo_dot_num = dot_num;

			switch ((uint32_t)data_queue->data_type) {
			case DATA_TYPE_POINTCLOUD:
				buf->echo_type = FS_PCL_DIS_XYZ;
				break;
			case DATA_TYPE_CICL_DATA:
				buf->echo_type = FS_CICL_DATA;
				break;
			case DATA_TYPE_GL_DATA:
				buf->echo_type = FS_GL_DATA;
				break;
			default:
				buf->echo_type = FS_CICL_DATA;
				break;
			}
			buf->echo_peak_type_2_1 = ((data_queue->peak_type[1] & 0x0F) << 4) |
						  (data_queue->peak_type[0] & 0x0F);
			buf->echo_frame_flag = frame_flag;
			buf->echo_width = data_queue->ROI_info[roi_index].width;
			buf->echo_height = data_queue->ROI_info[roi_index].height;
			buf->echo_max_bin_cnt = data_queue->max_bin_cnt;
			buf->echo_ab_frame_idx = ((data_queue->ROI_info[roi_index].pcl_unit & 0x7F) << 1) |
						 (data_queue->ab_frame_idx & 0x01);
			buf->echo_max_sub_frame_cnt = data_queue->max_sub_frame_cnt & 0x0F;
			buf->echo_roi_index = roi_index;
			buf->echo_pixel_len = pixel_len;
			buf->echo_data_offset = 0;
			buf->echo_data_len = 0;
			buf->echo_peak_type_4_3 = ((data_queue->peak_type[3] & 0x0F) << 4) |
						  (data_queue->peak_type[2] & 0x0F);
			buf->echo_timestamp_base = data_queue->timestamp_base;
			memcpy(buf->raw_pixels, data_queue->data + (size_t)(j * max_dot_num) * pixel_len,
			       (size_t)dot_num * pixel_len);
			atomic_store(&(p_node->write_ok), READABLE);

			dbg_frame_pkt_total++;
			dbg_frame_dot_total += dot_num;
		}

		/* 跨 push 状态推进 */
		g_next_start_sequence = g_next_start_sequence + (uint16_t)pkt_num;
		if (g_max_sequence < (g_next_start_sequence - 1)) {
			g_max_sequence = g_next_start_sequence - 1;
		}

		free(data_queue->data);
		free(data_queue);
	}
}

#if RANGE_TX_LZ4
/* ============================ Range LZ4 压缩发送 ============================
 *
 * 流程: 帧首 push 开启 LZ4F 流 -> 每个 push 的像素数据在压缩组件内做
 *       字节转置 + LZ4 压缩 -> 压缩输出经 sink 回调按节点容量分片,
 *       写入内存池(每片一个 UDP 包), consumer 原样发送。
 *
 * 分片策略: 尾部不足一包的数据暂存 pending, 待下一段数据凑满或帧尾
 *           flush 时发送, 以保证最后一个包能正确标记 frame_flag=END。
 */
typedef struct {
	udp_t *udp;
	uint16_t frm_id; // 当前帧号
	uint8_t sub_frame_idx; // 当前子帧相位(ROI1: 0~3)
	uint16_t seq; // 帧内包序号
	bool first_sent; // 帧首包(带元信息)是否已发出
	uint8_t pending[sizeof(mf150_fl6532_raw_group_packet_t)]; // 尾部分片暂存
	uint32_t pending_len;
	uint32_t pkt_total; // 本帧已发包数
	uint64_t uncomp_total; // 本帧原始字节累计
	mf150_fl6532_range_lz4_meta_t meta; // 帧元信息(帧首包携带)
} range_lz4_tx_t;

/* 各 ROI 的 group 数(由 RX 配置决定, 首次调用时缓存) */
static uint8_t s_roi_group_cnt[4] = { 0 };

static void range_lz4_load_roi_group_cnt(void)
{
	const rx_parse_cfg_t *parse_cfg;

	if (s_roi_group_cnt[0] != 0) {
		return;
	}
	rx_cfg_get_rx_parse(RX_LIDAR_MODE_RANGE_01, &parse_cfg);
	for (int32_t i = 0; i < parse_cfg->frm_cfg[0].group_cnt && i < RX_MAX_GROUP_CNT; i++) {
		uint8_t roi = parse_cfg->frm_cfg[0].group_cfg[i].ROI_sel & 0x03;

		s_roi_group_cnt[roi]++;
	}
}

/**
 * @description  : 从帧首 push 提取元信息(像素布局/ROI 描述), 供上位机
 *                 解压后按 push 逆转置与按 (roi,row,col) 拼图
 */
static void range_lz4_fill_meta(range_lz4_tx_t *tx, const fs_lidar_data_info_t *dq)
{
	mf150_fl6532_range_lz4_meta_t *m = &tx->meta;
	uint32_t total_pixels = 0;

	memset(m, 0, sizeof(*m));
	range_lz4_load_roi_group_cnt();

	m->pixel_len = dq->group_info.pixel_len;
	m->push_cnt = (uint8_t)dq->group_cnt;
	m->group_cnt = (uint8_t)dq->group_cnt;
	m->roi_cnt = dq->roi_cnt;
	m->max_sub_frame_cnt = dq->max_sub_frame_cnt;
	m->peak_num = dq->peak_num;
	m->group_row_unit = dq->group_row_unit;
	m->max_bin_cnt = dq->max_bin_cnt;

	for (uint8_t r = 0; r < dq->roi_cnt && r < 4; r++) {
		/* 每个 group 覆盖 (group_row_unit / row_unit) 行像素,
		 * 单次 push 的像素数再按子帧数均分 */
		uint32_t rows_per_group =
			(dq->ROI_info[r].row_unit > 0) ? ((uint32_t)dq->group_row_unit / dq->ROI_info[r].row_unit) : 0;
		uint32_t push_pixels = 0;

		if ((dq->ROI_info[r].sub_frame_cnt > 0) && (rows_per_group > 0)) {
			push_pixels = (uint32_t)dq->ROI_info[r].width * rows_per_group / dq->ROI_info[r].sub_frame_cnt;
		}

		m->roi[r].width = dq->ROI_info[r].width;
		m->roi[r].height = dq->ROI_info[r].height;
		m->roi[r].push_pixels = (uint16_t)push_pixels;
		m->roi[r].group_cnt = s_roi_group_cnt[r];
		m->roi[r].group_start = dq->ROI_info[r].group_start;
		m->roi[r].sub_frame_cnt = dq->ROI_info[r].sub_frame_cnt;
		m->roi[r].pcl_unit = dq->ROI_info[r].pcl_unit;
		m->roi[r].range_max = dq->ROI_info[r].range_max;
		total_pixels += (uint32_t)s_roi_group_cnt[r] * push_pixels;
	}
	m->total_pixels = total_pixels;
}

/* 当前可用的负载容量(帧首包要扣除元信息区) */
static uint32_t range_lz4_pkt_cap(const range_lz4_tx_t *tx)
{
	uint32_t hdr_len = tx->first_sent ? sizeof(mf150_fl6532_range_lz4_hdr_t) :
					    sizeof(mf150_fl6532_range_lz4_first_t);

	return (uint32_t)RANGE_TX_LZ4_PKT_SIZE - hdr_len;
}

/**
 * @description  : 把一段压缩数据组装为一个 UDP 包并写入内存池
 * @param         {range_lz4_tx_t} *tx 发送上下文
 * @param         {const uint8_t} *payload 压缩数据片段
 * @param         {uint32_t} len 片段长度
 * @param         {uint8_t} frame_flag 0=START,1=NORMAL,2=END
 * @return        {int32_t} RET_OK / RET_FAIL
 */
static int32_t range_lz4_send_pkt(range_lz4_tx_t *tx, const uint8_t *payload, uint32_t len, uint8_t frame_flag)
{
	node_t *p_node;
	uint8_t *buf;
	uint32_t hdr_len;
	mf150_fl6532_range_lz4_hdr_t *hdr;

	if (!tx->first_sent) {
		hdr_len = sizeof(mf150_fl6532_range_lz4_first_t);
	} else {
		hdr_len = sizeof(mf150_fl6532_range_lz4_hdr_t);
	}

	do {
		p_node = (node_t *)allocate_memory(tx->udp->pool);
		if (atomic_load(&(g_run_flag)) != RUNNING) {
			return RET_FAIL;
		}
	} while (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL);

	buf = p_node->data->buf;
	memset(buf, 0, (size_t)raw_point_cloud_full_size);

	hdr = (mf150_fl6532_range_lz4_hdr_t *)buf;
	hdr->sof = MF150_FL6532_RANGE_LZ4_SOF;
	hdr->version = MF150_FL6532_RANGE_LZ4_VERSION;
	hdr->frame_id = tx->frm_id;
	hdr->seq = tx->seq++;
	hdr->frame_flag = frame_flag;
	hdr->sub_frame_idx = tx->sub_frame_idx;
	hdr->payload_len = (uint16_t)len;
	hdr->data_len = (uint16_t)RANGE_TX_LZ4_PKT_SIZE; // 固定包长(尾部补 0)

	if (!tx->first_sent) {
		memcpy(buf + sizeof(mf150_fl6532_range_lz4_hdr_t), &tx->meta, sizeof(tx->meta));
		tx->first_sent = true;
	}
	memcpy(buf + hdr_len, payload, len);

	atomic_store(&(p_node->write_ok), READABLE);
	tx->pkt_total++;
	return RET_OK;
}

/**
 * @description  : 压缩输出回调: 把压缩字节流按包容量分片发送,
 *                 尾部不足一包的数据暂存 pending(等 END 标记)
 */
static int32_t range_lz4_sink(const uint8_t *data, uint32_t len, void *user)
{
	range_lz4_tx_t *tx = (range_lz4_tx_t *)user;

	if (len == 0) {
		return RET_OK;
	}

	/* 先补 pending 中的半包 */
	if (tx->pending_len > 0) {
		uint32_t cap = range_lz4_pkt_cap(tx);
		uint32_t space = cap - tx->pending_len;
		uint32_t take = (len < space) ? len : space;

		memcpy(tx->pending + tx->pending_len, data, take);
		tx->pending_len += take;
		data += take;
		len -= take;

		if (tx->pending_len >= cap) {
			if (range_lz4_send_pkt(tx, tx->pending, tx->pending_len, tx->first_sent ? 1 : 0) != RET_OK) {
				return RET_FAIL;
			}
			tx->pending_len = 0;
		}
	}

	/* 整包直发, 尾部不足一包的数据暂存 */
	while (len > 0) {
		uint32_t cap = range_lz4_pkt_cap(tx);

		if (len > cap) {
			if (range_lz4_send_pkt(tx, data, cap, tx->first_sent ? 1 : 0) != RET_OK) {
				return RET_FAIL;
			}
			data += cap;
			len -= cap;
		} else {
			memcpy(tx->pending, data, len);
			tx->pending_len = len;
			len = 0;
		}
	}
	return RET_OK;
}

/* 帧尾: 把暂存的最后一片作为 END 包发出 */
static int32_t range_lz4_flush(range_lz4_tx_t *tx, uint8_t frame_flag)
{
	int32_t ret = RET_OK;

	if (tx->pending_len > 0) {
		ret = range_lz4_send_pkt(tx, tx->pending, tx->pending_len, frame_flag);
		tx->pending_len = 0;
	}
	return ret;
}

/* ---- 发送帧率统计(1s 窗口): 统计成功交给发送队列的帧数与帧号推进 ---- */
static struct timespec g_tx_fps_ts = { 0, 0 };
static uint32_t g_tx_fps_frames = 0;
static uint32_t g_tx_fps_pkts = 0;
static uint16_t g_tx_fps_frm_first = 0;
static uint16_t g_tx_fps_frm_last = 0;

static void range_lz4_tx_fps_tick(uint16_t frm_id, uint32_t pkts)
{
	struct timespec now;
	int64_t dt_ms;

	g_tx_fps_frames++;
	g_tx_fps_pkts += pkts;
	if (g_tx_fps_frames == 1) {
		g_tx_fps_frm_first = frm_id;
	}
	g_tx_fps_frm_last = frm_id;

	clock_gettime(CLOCK_MONOTONIC, &now);
	if (g_tx_fps_ts.tv_sec == 0) {
		g_tx_fps_ts = now;
		return;
	}
	dt_ms = (int64_t)(now.tv_sec - g_tx_fps_ts.tv_sec) * 1000 + (now.tv_nsec - g_tx_fps_ts.tv_nsec) / 1000000;
	if (dt_ms >= 1000) {
		uint16_t span = (uint16_t)(g_tx_fps_frm_last - g_tx_fps_frm_first + 1);

		log_info("[FL6532-RANGE] tx fps: frames=%u fps=%.1f | frm=%u..%u span=%u | pkts=%u", g_tx_fps_frames,
			 (double)g_tx_fps_frames * 1000.0 / (double)dt_ms, g_tx_fps_frm_first, g_tx_fps_frm_last, span,
			 g_tx_fps_pkts);
		g_tx_fps_ts = now;
		g_tx_fps_frames = 0;
		g_tx_fps_pkts = 0;
	}
}

/**
 * @description  : Range 压缩发送主循环: 流式转置+LZ4 压缩, 分片入内存池
 * @param         {udp_t} *udp_instance
 * @return        {}
 */
static void producer_get_range_lz4_from_driver(udp_t *udp_instance)
{
	fs_lidar_data_info_t *data_queue = NULL;
	range_lz4_tx_t tx;
	uint16_t last_frm_id = 0xFFFF;
	bool frame_open = false;
	ys_compress_t *comp = NULL;
	uint32_t frame_pkt_total = 0;
	uint32_t frame_cnt = 0;

	memset(&tx, 0, sizeof(tx));
	tx.udp = udp_instance;

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		algo_pixel_queue_pop_data(&data_queue, 100);
		if (!data_queue) {
			continue;
		}

		uint16_t pixel_len = data_queue->group_info.pixel_len;
		uint32_t pixel_count = data_queue->group_info.pixel_count;
		uint8_t roi_index = data_queue->group_info.roi_index & 0x03;
		uint8_t sub_frame_idx = data_queue->ROI_info[roi_index].sub_frame_idx;

		/* 帧边界: frm_id 变化(板端节流会整帧丢弃, 帧号跳变属正常) */
		if (data_queue->frm_id != last_frm_id) {
			if (frame_open) {
				/* 上一帧收尾: 压缩流结束 + END 包 */
				ys_compress_end(comp);
				range_lz4_flush(&tx, 2);
				range_lz4_tx_fps_tick(last_frm_id, tx.pkt_total);

				if ((frame_cnt % RANGE_FRAME_STAT_INTERVAL) == 0) {
					uint64_t comp_out = ys_compress_stream_out(comp);

					log_info("[FL6532-RANGE] lz4 frame: frm=%u pkts=%u uncomp=%llu "
						 "comp=%llu ratio=%.2f:1",
						 last_frm_id, frame_pkt_total, (unsigned long long)tx.uncomp_total,
						 (unsigned long long)comp_out,
						 comp_out ? (double)tx.uncomp_total / (double)comp_out : 0.0);
				}
				frame_pkt_total = 0;
				frame_cnt++;
			}

			/* 新帧: 初始化发送上下文/元信息, 开启压缩流 */
			last_frm_id = data_queue->frm_id;
			tx.frm_id = data_queue->frm_id;
			tx.sub_frame_idx = sub_frame_idx;
			tx.seq = 0;
			tx.first_sent = false;
			tx.pending_len = 0;
			tx.pkt_total = 0;
			tx.uncomp_total = 0;
			range_lz4_fill_meta(&tx, data_queue);

			if (!comp) {
				ys_compress_cfg_t comp_cfg = {
					.algo = YS_COMPRESS_ALGO_LZ4,
					.level = RANGE_TX_LZ4_LEVEL,
					.block_size = RANGE_TX_LZ4_BLOCK_KB * 1024u,
					.checksum = true,
					.transpose_stride = pixel_len,
				};

				comp = ys_compress_create(&comp_cfg);
				if (!comp) {
					log_error("[FL6532-RANGE] compress create failed");
					free(data_queue->data);
					free(data_queue);
					return;
				}
			}
			ys_compress_begin(comp, range_lz4_sink, &tx);
			frame_open = true;
		}

		/* 喂入本 push 像素数据(转置+压缩在组件内完成) */
		if (ys_compress_update(comp, data_queue->data, pixel_count * pixel_len) != RET_OK) {
			log_error("[FL6532-RANGE] compress update failed, frm=%u vir=%u", data_queue->frm_id,
				  data_queue->group_info.vir_group_idx);
		}
		tx.uncomp_total += (uint64_t)pixel_count * pixel_len;
		frame_pkt_total = tx.pkt_total;

		free(data_queue->data);
		free(data_queue);
	}

	if (frame_open && comp) {
		ys_compress_end(comp);
		range_lz4_flush(&tx, 2);
	}
	ys_compress_destroy(comp);
}
#endif // RANGE_TX_LZ4
#endif // RANGE_MEASURE_ONLY == 0

#if RANGE_MEASURE_ONLY == 1
#define RANGE_MEASURE_GROUP_MAX 64 // vir_group_idx 上限(当前 RANGE_01 为 44)
#define RANGE_MEASURE_PUSH_LOG_FRAMES 4 // 前 N 帧打印逐 push 明细(0 关闭)
#define RANGE_MEASURE_FULL_LOG_INTERVAL 100 // 每 N 帧打印一次完整摘要(roi 分布/hits)
#define RANGE_MEASURE_WIN_SEC 1.0 // 统计窗口(秒), 输出该窗口内的真实数据量
#define RANGE_MEASURE_SUB_PROBE_VIR 32 // 子帧互补性探针 group(ROI1 第一个 group)
#define RANGE_MEASURE_PROBE_ROW_MAX 128 // 探针位图行范围
#define RANGE_MEASURE_PROBE_COL_MAX 256 // 探针位图列范围
#define RANGE_MEASURE_COMP_LEVEL 0 // 压缩率实测级别(LZ4: 0~12, >9 走 HC)
#define RANGE_MEASURE_COMP_BLOCK_KB 1024 // 压缩率实测块大小(KB)

/* 子帧互补性验证位图: 累积 vir=32 跨 4 个子帧的 (row,col) 集合 */
static uint8_t g_sub_probe_bits[RANGE_MEASURE_PROBE_ROW_MAX * RANGE_MEASURE_PROBE_COL_MAX / 8];

/* 压缩率实测: sink 只统计压缩输出字节数, 不发送 */
static uint64_t g_compress_probe_out = 0;

static int32_t measure_compress_sink(const uint8_t *data, uint32_t len, void *user)
{
	(void)data;
	(void)user;
	g_compress_probe_out += len;
	return RET_OK;
}

static uint64_t g_trans_probe_out = 0;

static int32_t measure_trans_sink(const uint8_t *data, uint32_t len, void *user)
{
	(void)data;
	(void)user;
	g_trans_probe_out += len;
	return RET_OK;
}

/* 单次 push 的关键字段快照 */
typedef struct {
	uint16_t frm_id;
	uint16_t vir;
	uint16_t phy;
	uint8_t roi;
	uint8_t sub;
	uint8_t sub_cnt;
	uint32_t pcnt;
	uint16_t plen;
} range_push_snap_t;

static void measure_print_snap(const char *tag, const range_push_snap_t *s)
{
	log_info("[RANGE-FRM] %s: frm=%u vir=%u phy=%u roi=%u sub=%u/%u pcnt=%u plen=%u", tag, s->frm_id, s->vir,
		 s->phy, s->roi, s->sub, s->sub_cnt, s->pcnt, s->plen);
}

/**
 * @description  : 纯测量: 消费 fs_sdk Range 队列, 统计帧率/帧大小/数据速率
 *                 不组包不发送; 按 frm_id 归帧, 每帧打印 group 覆盖与 push 分布,
 *                 前几帧额外打印逐 push 明细, 用于核对真实组帧格式
 */
static void producer_measure_range_from_driver(void)
{
	fs_lidar_data_info_t *data_queue = NULL;
	uint16_t last_frm_id = 0xFFFF;
	struct timespec win_ts = { 0, 0 };
	/* 压缩率实测实例(带字节转置预处理) */
	ys_compress_cfg_t comp_cfg = {
		.algo = YS_COMPRESS_ALGO_LZ4,
		.level = RANGE_MEASURE_COMP_LEVEL,
		.block_size = RANGE_MEASURE_COMP_BLOCK_KB * 1024u,
		.checksum = true,
		.transpose_stride = 278, // 像素记录宽度
	};
	ys_compress_t *comp = NULL; // 对照: 不转置压缩(测量时置 NULL 只测转置路径)
	ys_compress_t *comp_trans = ys_compress_create(&comp_cfg); // 转置压缩实验实例

	/* 1s 窗口内累计量 */
	uint64_t win_frames = 0; // 收到的完整帧数(frm_id 变化)
	uint64_t win_pushes = 0; // push(组xslot/子帧)数
	uint64_t win_dots = 0; // 像素总数
	uint64_t win_bytes = 0; // 像素净荷字节数
	uint64_t win_net_bytes = 0; // 模拟 UDP 分片后的发送字节数(含每包 40B 包头)
	uint64_t win_comp_in = 0; // 压缩输入字节数(像素净荷)
	uint64_t win_comp_out = 0; // 压缩输出字节数
	uint64_t win_trans_in = 0; // 转置压缩输入字节数
	uint64_t win_trans_out = 0; // 转置压缩输出字节数
	uint16_t win_frm_first = 0; // 窗口内首个帧号
	uint16_t win_frm_last = 0; // 窗口内最后帧号

	/* 当前帧聚合量(按 frm_id 归帧) */
	uint64_t cur_pushes = 0;
	uint64_t cur_dots = 0;
	uint64_t cur_bytes = 0;
	uint16_t cur_group_seen = 0; // 本帧出现过的不同 vir_group_idx 数
	uint16_t cur_hits[RANGE_MEASURE_GROUP_MAX] = { 0 }; // 各 group 的 push 次数
	uint64_t cur_roi_pushes[4] = { 0 }; // 各 ROI 的 push 次数
	uint64_t cur_roi_dots[4] = { 0 }; // 各 ROI 的像素数
	uint8_t cur_roi_sub[4] = { 0 }; // 各 ROI 本帧最后一次 push 的 sub_frame_idx
	uint8_t cur_roi_sub_cnt[4] = { 0 }; // 各 ROI 本帧最后一次 push 的 sub_frame_cnt
	uint32_t frame_idx = 0; // 已结算帧计数(控制明细打印)
	bool meta_logged = false;
	range_push_snap_t cur_first = { 0 };
	range_push_snap_t cur_last = { 0 };

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		algo_pixel_queue_pop_data(&data_queue, 100);
		if (!data_queue) {
			continue;
		}

		uint16_t pixel_len = data_queue->group_info.pixel_len;
		uint16_t vir_group_idx = data_queue->group_info.vir_group_idx;
		uint16_t phy_group_idx = data_queue->group_info.phy_group_idx;
		uint8_t roi_index = data_queue->group_info.roi_index & 0x03;
		uint32_t pixel_count = data_queue->group_info.pixel_count;
		uint8_t sub_frame_idx = data_queue->ROI_info[roi_index].sub_frame_idx;
		uint8_t sub_frame_cnt = data_queue->ROI_info[roi_index].sub_frame_cnt;

		/* 帧边界: frm_id 变化即为新帧(板端帧级节流会导致帧号跳变) */
		if (data_queue->frm_id != last_frm_id) {
			if (last_frm_id != 0xFFFF) {
				uint16_t expect_groups = data_queue->group_cnt;
				bool full = (frame_idx < RANGE_MEASURE_PUSH_LOG_FRAMES) ||
					    ((frame_idx % RANGE_MEASURE_FULL_LOG_INTERVAL) == 0) ||
					    (cur_group_seen != expect_groups);

				/* 上一帧结算 */
				log_info("[RANGE-FRM] frm=%u pushes=%llu groups=%u/%u dots=%llu bytes=%llu",
					 last_frm_id, (unsigned long long)cur_pushes, cur_group_seen, expect_groups,
					 (unsigned long long)cur_dots, (unsigned long long)cur_bytes);
				log_info("[RANGE-FRM] frm=%u sub: r0=%u/%u r1=%u/%u r2=%u/%u r3=%u/%u", last_frm_id,
					 cur_roi_sub[0], cur_roi_sub_cnt[0], cur_roi_sub[1], cur_roi_sub_cnt[1],
					 cur_roi_sub[2], cur_roi_sub_cnt[2], cur_roi_sub[3], cur_roi_sub_cnt[3]);

				if (full) {
					char line[RANGE_MEASURE_GROUP_MAX * 6 + 16];
					int pos = 0;
					uint16_t i;

					log_info("[RANGE-FRM] roi: r0(push=%llu dot=%llu) r1(push=%llu dot=%llu) "
						 "r2(push=%llu dot=%llu) r3(push=%llu dot=%llu)",
						 (unsigned long long)cur_roi_pushes[0],
						 (unsigned long long)cur_roi_dots[0],
						 (unsigned long long)cur_roi_pushes[1],
						 (unsigned long long)cur_roi_dots[1],
						 (unsigned long long)cur_roi_pushes[2],
						 (unsigned long long)cur_roi_dots[2],
						 (unsigned long long)cur_roi_pushes[3],
						 (unsigned long long)cur_roi_dots[3]);
					measure_print_snap("first", &cur_first);
					measure_print_snap("last ", &cur_last);

					line[0] = '\0';
					for (i = 0; i < expect_groups && i < RANGE_MEASURE_GROUP_MAX; i++) {
						pos += snprintf(line + pos, sizeof(line) - (size_t)pos,
								(i == 0) ? "%u" : " %u", cur_hits[i]);
						if ((size_t)pos >= sizeof(line)) {
							break;
						}
					}
					log_info("[RANGE-FRM] hits[0..%u]: %s",
						 (expect_groups > 0) ? (uint16_t)(expect_groups - 1) : 0, line);
				}

				/* 上一帧压缩流收尾并累计压缩率 */
				if (comp) {
					ys_compress_end(comp);
					win_comp_in += ys_compress_stream_in(comp);
					win_comp_out += ys_compress_stream_out(comp);
				}
				if (comp_trans) {
					ys_compress_end(comp_trans);
					win_trans_in += ys_compress_stream_in(comp_trans);
					win_trans_out += ys_compress_stream_out(comp_trans);
				}
				frame_idx++;
			}

			/* 新帧: 重置聚合量 */
			last_frm_id = data_queue->frm_id;
			if (comp) {
				ys_compress_begin(comp, measure_compress_sink, NULL);
			}
			if (comp_trans) {
				ys_compress_begin(comp_trans, measure_trans_sink, NULL);
			}
			cur_pushes = 0;
			cur_dots = 0;
			cur_bytes = 0;
			cur_group_seen = 0;
			memset(cur_hits, 0, sizeof(cur_hits));
			memset(cur_roi_pushes, 0, sizeof(cur_roi_pushes));
			memset(cur_roi_dots, 0, sizeof(cur_roi_dots));
			memset(cur_roi_sub, 0, sizeof(cur_roi_sub));
			memset(cur_roi_sub_cnt, 0, sizeof(cur_roi_sub_cnt));
			cur_first.frm_id = data_queue->frm_id;
			cur_first.vir = vir_group_idx;
			cur_first.phy = phy_group_idx;
			cur_first.roi = roi_index;
			cur_first.sub = sub_frame_idx;
			cur_first.sub_cnt = sub_frame_cnt;
			cur_first.pcnt = pixel_count;
			cur_first.plen = pixel_len;
			if (win_frames == 0) {
				win_frm_first = data_queue->frm_id;
			}
			win_frm_last = data_queue->frm_id;
			win_frames++;

			if (!meta_logged) {
				log_info(
					"[RANGE-META] frm=%u group_cnt=%u roi_cnt=%u max_sub=%u row_unit=%u peak=%u bin=%u",
					data_queue->frm_id, data_queue->group_cnt, data_queue->roi_cnt,
					data_queue->max_sub_frame_cnt, data_queue->group_row_unit, data_queue->peak_num,
					data_queue->max_bin_cnt);
				for (uint8_t r = 0; r < data_queue->roi_cnt && r < 4; r++) {
					log_info(
						"[RANGE-META] roi%u: w=%u h=%u gstart=%u sub=%u bin=%u rmax=%.0f pcl_unit=%u",
						r, data_queue->ROI_info[r].width, data_queue->ROI_info[r].height,
						data_queue->ROI_info[r].group_start,
						data_queue->ROI_info[r].sub_frame_cnt, data_queue->ROI_info[r].bin_cnt,
						data_queue->ROI_info[r].range_max, data_queue->ROI_info[r].pcl_unit);
				}
				meta_logged = true;
			}
		}

		/* 本帧聚合 */
		if (vir_group_idx < RANGE_MEASURE_GROUP_MAX) {
			if (cur_hits[vir_group_idx] == 0) {
				cur_group_seen++;
			}
			cur_hits[vir_group_idx]++;
		}
		cur_pushes++;
		cur_dots += pixel_count;
		cur_bytes += (uint64_t)pixel_count * pixel_len;
		cur_roi_pushes[roi_index]++;
		cur_roi_dots[roi_index] += pixel_count;
		cur_roi_sub[roi_index] = sub_frame_idx;
		cur_roi_sub_cnt[roi_index] = sub_frame_cnt;

		/* 压缩率实测: 逐 push 喂入像素数据 */
		if (comp) {
			ys_compress_update(comp, data_queue->data, pixel_count * pixel_len);
		}

		/* 转置压缩实验: 组件内部自动字节转置 */
		if (comp_trans) {
			ys_compress_update(comp_trans, data_queue->data, pixel_count * pixel_len);
		}

		cur_last.frm_id = data_queue->frm_id;
		cur_last.vir = vir_group_idx;
		cur_last.phy = phy_group_idx;
		cur_last.roi = roi_index;
		cur_last.sub = sub_frame_idx;
		cur_last.sub_cnt = sub_frame_cnt;
		cur_last.pcnt = pixel_count;
		cur_last.plen = pixel_len;

		/* 逐 push 明细(仅前 N 帧): 统计像素 (row,col) 范围与唯一行/列数,
		 * 用于判断不同 sub_frame 覆盖的像素是否互补(跨帧拼图)还是同一批像素(重复测量) */
		if (frame_idx < RANGE_MEASURE_PUSH_LOG_FRAMES) {
			uint16_t row_min = 0xFFFF, row_max = 0, col_min = 0xFFFF, col_max = 0;
			uint16_t row_uniq = 0, col_uniq = 0;
			uint8_t row_seen[128] = { 0 };
			uint8_t col_seen[256] = { 0 };

			if ((pixel_count > 0) && (pixel_len >= 60)) {
				for (uint32_t k = 0; k < pixel_count; k++) {
					uint8_t *px = data_queue->data + (size_t)k * pixel_len;
					uint16_t r, c;

					/* cicl_pix_info_t: row 在偏移 56, col 在偏移 58 */
					memcpy(&r, px + 56, sizeof(r));
					memcpy(&c, px + 58, sizeof(c));
					if (r < row_min)
						row_min = r;
					if (r > row_max)
						row_max = r;
					if (c < col_min)
						col_min = c;
					if (c > col_max)
						col_max = c;
					if (r < 128)
						row_seen[r] = 1;
					if (c < 256)
						col_seen[c] = 1;
				}
				for (int32_t k = 0; k < 128; k++) {
					row_uniq += row_seen[k];
				}
				for (int32_t k = 0; k < 256; k++) {
					col_uniq += col_seen[k];
				}
			}

			log_info("[RANGE-PUSH] frm=%u #%llu vir=%u phy=%u roi=%u sub=%u/%u pcnt=%u "
				 "row[%u..%u]x%u col[%u..%u]x%u",
				 data_queue->frm_id, (unsigned long long)cur_pushes, vir_group_idx, phy_group_idx,
				 roi_index, sub_frame_idx, sub_frame_cnt, pixel_count, row_min, row_max, row_uniq,
				 col_min, col_max, col_uniq);
		}

		/* 子帧互补性验证: 累积 vir=32 跨 4 个子帧的 (row,col) 集合,
		 * 子帧收齐时统计并集: 768 = 4 帧互补拼图, 192 = 同一批像素重复测量 */
		if ((vir_group_idx == RANGE_MEASURE_SUB_PROBE_VIR) && (pixel_len >= 60)) {
			for (uint32_t k = 0; k < pixel_count; k++) {
				uint8_t *px = data_queue->data + (size_t)k * pixel_len;
				uint16_t r, c;

				memcpy(&r, px + 56, sizeof(r));
				memcpy(&c, px + 58, sizeof(c));
				if ((r < RANGE_MEASURE_PROBE_ROW_MAX) && (c < RANGE_MEASURE_PROBE_COL_MAX)) {
					uint32_t bit = (uint32_t)r * RANGE_MEASURE_PROBE_COL_MAX + c;

					g_sub_probe_bits[bit >> 3] |= (uint8_t)(1u << (bit & 7u));
				}
			}

			if ((sub_frame_cnt > 0) && (sub_frame_idx >= (uint8_t)(sub_frame_cnt - 1))) {
				uint32_t union_pixels = 0;

				for (uint32_t k = 0; k < sizeof(g_sub_probe_bits); k++) {
					union_pixels += (uint32_t)__builtin_popcount(g_sub_probe_bits[k]);
				}
				log_info("[RANGE-SUB] 4-sub union vir=%u: pixels=%u/768 (%s)", vir_group_idx,
					 union_pixels, (union_pixels == 768) ? "complementary" : "overlap-or-loss");
				memset(g_sub_probe_bits, 0, sizeof(g_sub_probe_bits));
			}
		}

		win_pushes++;
		win_dots += pixel_count;
		win_bytes += (uint64_t)pixel_count * pixel_len;

		/* 模拟发送分片(与 producer_get_range_from_driver 一致), 估算真实网络数据量 */
		{
			int32_t max_dot_num = (int32_t)((FS_PROTOCOL_UDP_MTU_SIZE - sizeof(fs_protocol_head_t) -
							 sizeof(fs_lidar_pcl_echo_data_t)) /
							pixel_len);

			if (max_dot_num > 0) {
				uint32_t pkt_num = (pixel_count + (uint32_t)max_dot_num - 1) / (uint32_t)max_dot_num;

				win_net_bytes += (uint64_t)pkt_num * (sizeof(fs_protocol_head_t) +
								      sizeof(fs_lidar_pcl_echo_data_t)) +
						 (uint64_t)pixel_count * pixel_len;
			}
		}

		free(data_queue->data);
		free(data_queue);

		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		if (win_ts.tv_sec == 0) {
			win_ts = now;
			continue;
		}
		double dt = (double)(now.tv_sec - win_ts.tv_sec) + (double)(now.tv_nsec - win_ts.tv_nsec) / 1e9;
		if (dt >= RANGE_MEASURE_WIN_SEC) {
			double payload_mb_s = (double)win_bytes / (1024.0 * 1024.0) / dt;
			double net_mb_s = (double)win_net_bytes / (1024.0 * 1024.0) / dt;
			double comp_in_mb_s = (double)win_comp_in / (1024.0 * 1024.0) / dt;
			double comp_out_mb_s = (double)win_comp_out / (1024.0 * 1024.0) / dt;
			double comp_ratio = win_comp_out ? (double)win_comp_in / (double)win_comp_out : 0.0;
			int32_t frm_span = (int32_t)win_frm_last - (int32_t)win_frm_first + 1;

			log_info("[RANGE-MEAS] dt=%.2fs frames=%llu fps=%.1f frm=%u..%u span=%d | "
				 "pushes=%llu push/frame=%.1f dots=%llu | payload=%.1f MB/s net=%.1f MB/s | "
				 "avg_frame=%.2f MB net_frame=%.2f MB",
				 dt, (unsigned long long)win_frames, (double)win_frames / dt, win_frm_first,
				 win_frm_last, frm_span, (unsigned long long)win_pushes,
				 win_frames ? (double)win_pushes / win_frames : 0.0, (unsigned long long)win_dots,
				 payload_mb_s, net_mb_s,
				 win_frames ? (double)win_bytes / (1024.0 * 1024.0) / win_frames : 0.0,
				 win_frames ? (double)win_net_bytes / (1024.0 * 1024.0) / win_frames : 0.0);
			log_info("[RANGE-MEAS] lz4: in=%.1f MB/s out=%.1f MB/s ratio=%.2f:1 "
				 "in_total=%llu out_total=%llu",
				 comp_in_mb_s, comp_out_mb_s, comp_ratio,
				 (unsigned long long)ys_compress_total_in(comp),
				 (unsigned long long)ys_compress_total_out(comp));
			log_info("[RANGE-MEAS] lz4+transpose: in=%.1f MB/s out=%.1f MB/s ratio=%.2f:1",
				 (double)win_trans_in / (1024.0 * 1024.0) / dt,
				 (double)win_trans_out / (1024.0 * 1024.0) / dt,
				 win_trans_out ? (double)win_trans_in / (double)win_trans_out : 0.0);
			win_ts = now;
			win_frames = 0;
			win_pushes = 0;
			win_dots = 0;
			win_bytes = 0;
			win_net_bytes = 0;
			win_comp_in = 0;
			win_comp_out = 0;
			win_trans_in = 0;
			win_trans_out = 0;
			win_frm_first = 0;
			win_frm_last = 0;
		}
	}

	ys_compress_destroy(comp);
	ys_compress_destroy(comp_trans);
}
#endif // RANGE_MEASURE_ONLY == 1
#endif // LOCAL_TEST

void producer_get_range(udp_t *udp_instance, int driver_slot_num)
{
	(void)driver_slot_num;
	atomic_init(&g_run_flag, RUNNING);

	if (udp_instance == NULL) {
		return;
	}

#if LOCAL_TEST == 0
#if RANGE_MEASURE_ONLY == 1
	producer_measure_range_from_driver();
#elif RANGE_TX_LZ4
	producer_get_range_lz4_from_driver(udp_instance);
#else
	producer_get_range_from_driver(udp_instance);
#endif
#endif

	return;
}
