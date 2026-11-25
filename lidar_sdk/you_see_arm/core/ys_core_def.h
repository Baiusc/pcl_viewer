/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-06-04 21:28:32
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-10-15 04:30:15
 * @FilePath     : /you_see_arm/core/ys_core_def.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_CORE_DEF_H
#define YS_CORE_DEF_H

#include <uv.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>

#include "../common/common.h"

#define VELOCITYLIGHT 299792458 // 光速，取值 299792458 m/s

static const double velocitylight = (VELOCITYLIGHT * 1 * 1e-12); // 将光速单位 m/s 转为 m/ps

#pragma pack(push, 1)

typedef union float_union {
	uint32_t u32;
	float f;
} float_union_t;

typedef struct {
	uint8_t *buf;
	int len;
} data_t;

typedef struct node {
	atomic_int write_ok; //0:读完成，可写 1:写完成，可读
	data_t *data; // 数据块
} node_t;

typedef struct block {
	node_t *node; // 数据体
	struct block *next; // 指向下一个块的指针
} block_t;

typedef struct {
	_Atomic(block_t *) head; // 环形链表的头
	_Atomic(block_t *) tail; // 环形链表的尾
	size_t size; // 链表中的块的数量
} memory_pool_t;

// 创建线程时需要传递的参数
typedef struct {
	void *thread_arg; // 线程运行参数
	int signal_enable; // 是否监听信号
	int signum; // 监听信号的编号
} thread_t;

// 保存所有线程的信息结构体
typedef struct {
	pthread_t tid[5]; // 某个线程的 ID 号，根据索引对应
	int signal_num[5]; // 某个线程监听的信号编号，根据索引对应
	int thpool_count; // 当前正在运行的线程数

	pthread_mutex_t lock;
} thpool_t;

typedef struct {
	int sockfd;
	int port; // 中心端口/上位机端口
	int broadcast_port; // 广播端口
	int local_port; // 本地端口
	char ip[32];
	pthread_mutex_t lock;
} server_info_t;

typedef struct {
	void *data;
	int port; // 中心端口/上位机端口
	int broadcast_port; // 广播端口
	int local_port; // 本地端口
	char ip[32];
	struct sockaddr_in addr;
	int broadcast_flag;

	uv_udp_t uv_handle;
	uint32_t local_address; // 当前本机 IPv4，网络字节序
	uint32_t received_local_address; // 最近控制报文的真实目标 IPv4，网络字节序

	char *boradcast_buf;
	long long timestamp;
	long long handshake_timeout_msec; // 单位毫秒

	// 需要先连接网络，在网络握手，发送数据的时候会检查网络是否连接
	int is_connected; // 网络连接标志，0 为网络未连接，1 为网络已连接
	int is_handshaked; // 网络握手标志，0 为未握手，1 为握手成功
	uint32_t pre_heartbeat;
	uint32_t heartbeat_cnt; // 心跳标志，每次接收到上位机的心跳指令就自增，如果长时间没有变化认为是上位机离线，转为广播模式
	pthread_mutex_t lock;
} udp_center_t;

typedef struct {
	void *data;

	uint8_t *boradcast_buf;
	int boradcast_buf_len;
	uv_loop_t uv_loop;

	udp_center_t center;

	uv_timer_t hb_timer;
	uv_poll_t receive_poll;
	int receive_poll_initialized;
} cent_udp_client_t;

typedef struct udp_h {
	int simplex; // 单/双工标识符
	uint8_t *boradcast_buf; // 广播发现包，用于网络发现和网络握手
	int boradcast_buf_len; // 广播发现包数据长度

	memory_pool_t *pool; // 每个 udp 实例应该有自己的内存池

	server_info_t ser_info; // 配置服务端 ip 和端口
	pthread_mutex_t lock;

	int (*deal_data_to_input_link)(void *arg, uint8_t *data, int datalen); // network -> local
	void (*network_maintenance)(void *arg); // 控制线程内执行有时间上限的网络维护
	int (*deal_data_to_output_link)(void *arg, uint8_t *data, int datalen); // local -> network

	cent_udp_client_t udpc;

	// socket
	struct hostent *socket_host;
	struct sockaddr_in client_addr;
	int socket_fd;
	int (*send_data_via_socket)(struct udp_h *udp, uint8_t *data, int len);
} udp_t;

typedef struct {
	uint16_t slot_number; // slot 序号 最大值 55
	uint8_t reserved[2]; // 预留
	uint16_t pixel_start_number; // pixel 起始序号
	uint16_t pixel_end_number; // pixel 结束序号
#if CS96_GAMMA == 1
	uint8_t sl100_reserved[24]; // SL100 包头共 32 字节
#endif
	uint8_t value[1];
} raw_packet_t; // 原始直方图数据格式 (从驱动读取出来的)

#if CS96_ALPHA == 1
typedef struct {
	uint16_t slot_number; // slot 序号 最大值 799
	uint16_t code_disc_angle; // 码盘的编码值
	uint8_t delay[10]; // 时间戳，精确到 us
	uint8_t reserved[18]; // 预留
	uint8_t value[1];
} org_point_cloud_packet_t; // 原始点云数据格式 (从驱动读取出来的)
#elif CS96_BETA == 1
typedef struct {
	uint8_t header
		[16]; // 包头，slot_num(10 bit) + mirror_angle(20 bit) + delay(timestamp 10 byte) + frame_id(1 byte) + reserved(1 byte)
	uint8_t value[1];
} org_point_cloud_packet_t; // 原始点云数据格式 (从驱动读取出来的)
/* ---------- 访问器：从 header 中提取各字段 ---------- */
// slot_number: byte[0] + byte[1]低四位
static inline uint16_t cs96_beta_get_slot_number(org_point_cloud_packet_t *p)
{
	return ((uint16_t)p->header[1] << 8 | p->header[0]) & 0x0FFF;
}

// mirror_angle: byte[1] 高四位 + byte[2] + byte[3] 共20位
static inline uint32_t cs96_beta_get_mirror_angle(org_point_cloud_packet_t *p)
{
	uint32_t v = (uint32_t)p->header[3] << 24 | (uint32_t)p->header[2] << 16 | (uint32_t)p->header[1] << 8 |
		     (uint32_t)p->header[0];
	return (v >> 12) & 0xFFFFF;
}

// delay(timestamp): byte[4] ~ byte[13], 共10字节时间戳
// 直接返回指向header中的时间戳的地址，获取到之后可按照长度直接memcpy
static inline uint8_t *cs96_beta_get_delay(org_point_cloud_packet_t *p)
{
	return p->header + 4;
}

// fpga_frame_id: byte[14], 1字节, 在0~255 循环
static inline uint8_t cs96_beta_get_fpga_frame_id(org_point_cloud_packet_t *p)
{
	return p->header[14];
}

/* ---------- 灰度图数据（Beta，work_mode=3 时使用） ---------- */
// 每个像素 4 字节: 灰度值[21:0] + 保留[31:22]
typedef struct {
	uint32_t data;
} gray_pixel_data_t;

// 一个 SLOT: 8 字节头(槽号 [15:0] 有效, 0~167) + GRAY_SLOT_PIXELS 个像素
typedef struct {
	uint8_t header[8];
	uint8_t value[1];
} org_gray_packet_t; // 原始灰度图数据格式 (从驱动读取出来的)

// slot_number: byte[0] + byte[1], 小端 16 位
static inline uint16_t cs96_beta_gray_get_slot_number(org_gray_packet_t *p)
{
	return (uint16_t)((uint16_t)p->header[1] << 8 | p->header[0]);
}

// 灰度值: data[21:0]
static inline uint32_t cs96_beta_gray_pixel_get_value(const gray_pixel_data_t *p)
{
	return (uint32_t)(p->data & 0x3FFFFF);
}

#elif CS96_GAMMA == 1
typedef struct {
	uint8_t header
		[16]; // 包头，slot_num(12 bit) + mirror_angle(20 bit) + relative_time(24 bit, us) + reserved(56 bit) + frame_id(8 bit) + reserved(8 bit)
	uint8_t value[1];
} org_point_cloud_packet_t; // 原始点云数据格式 (从驱动读取出来的)
/* ---------- 访问器：从 header 中提取各字段 ---------- */
// slot_number: byte[0] + byte[1]低四位
static inline uint16_t cs96_gamma_get_slot_number(org_point_cloud_packet_t *p)
{
	return ((uint16_t)p->header[1] << 8 | p->header[0]) & 0x0FFF;
}

// mirror_angle: byte[1] 高四位 + byte[2] + byte[3] 共20位
static inline uint32_t cs96_gamma_get_mirror_angle(org_point_cloud_packet_t *p)
{
	uint32_t v = (uint32_t)p->header[3] << 24 | (uint32_t)p->header[2] << 16 | (uint32_t)p->header[1] << 8 |
		     (uint32_t)p->header[0];
	return (v >> 12) & 0xFFFFF;
}

// relative_time: byte[4] ~ byte[6], 24位, 单位 us (相对时间)
static inline uint32_t cs96_gamma_get_relative_time(org_point_cloud_packet_t *p)
{
	return (uint32_t)p->header[6] << 16 | (uint32_t)p->header[5] << 8 | (uint32_t)p->header[4];
}

// fpga_frame_id: byte[14], 1字节, 在0~255 循环
static inline uint8_t cs96_gamma_get_fpga_frame_id(org_point_cloud_packet_t *p)
{
	return p->header[14];
}

/* ---------- 灰度图数据（Gamma，work_mode=3 时使用） ---------- */
// 每个像素 8 字节: ECHO1 灰度[19:0] + ECHO2 灰度[39:20] + 保留[63:40]
typedef struct {
	uint64_t data;
} gray_pixel_data_t;

// 一个 SLOT: 8 字节头(槽号 [4:0] 有效) + lines_v 个像素
typedef struct {
	uint8_t header[8];
	uint8_t value[1];
} org_gray_packet_t; // 原始灰度图数据格式 (从驱动读取出来的)

static inline uint16_t cs96_gamma_gray_get_slot_number(org_gray_packet_t *p)
{
	return (uint16_t)(p->header[0] & 0x1F);
}

static inline uint32_t gray_pixel_get_echo1(const gray_pixel_data_t *p)
{
	return (uint32_t)(p->data & 0xFFFFF);
}

static inline uint32_t gray_pixel_get_echo2(const gray_pixel_data_t *p)
{
	return (uint32_t)((p->data >> 20) & 0xFFFFF);
}

#elif MF150_FL6031 == 1
typedef struct {
	uint16_t data_type;
	uint16_t frame_id;
	uint16_t packet_len;
	uint16_t row_pos;
	uint16_t col_pos;
	uint8_t value[1];
} mf150_fl6031_rw_packet_t;

typedef struct {
	uint16_t env_sum;
	uint16_t noise;
	uint8_t laser_all_cnt[3];
	uint8_t hist_bins[501];
} mf150_fl6031_pixel_data_t;

typedef struct {
	uint16_t slot_number;
	uint8_t delay[10];
	uint8_t reserved[18];
	uint8_t value[1];
} org_point_cloud_packet_t;

typedef struct {
	uint16_t row_pos;
	uint16_t col_pos;
	uint8_t data[439];
} mf150_fl6031_pixel_info_t;

typedef struct {
	uint8_t sof;
	uint16_t frame_id;
	uint16_t data_len;
	uint8_t value[1];
} mf150_fl6031_histogram_packet_t;

#define MF150_FL6031_PIXELS_PER_SEND 3
#define MF150_FL6031_PIXEL_INFO_SIZE 443
#define MF150_FL6031_HISTOGRAM_PACKET_SIZE \
	(sizeof(mf150_fl6031_histogram_packet_t) - 1 + MF150_FL6031_PIXELS_PER_SEND * MF150_FL6031_PIXEL_INFO_SIZE)
// 内存池容量：每帧约18240包，需要足够容量应对启动时缓存堆积
// 板子有700MB内存，可以分配更大的内存池
#define MF150_FL6031_HISTOGRAM_MEMORY_POOL_MAX_CNT (18240 * 8) // 约 145920 个块，约 65MB

/* === MF150 点云新数据结构 === */
typedef struct {
	float x;
	float y;
	float z;
} point_3d;

typedef struct {
	uint16_t row_position;
	uint16_t column_position;
	float distance; // 单位 cm
	uint32_t reflectivity; // 反射率
	point_3d position_3d; // 真实坐标值
	uint32_t peak0_position; // 第一重回波前沿
	uint32_t peak0_value; // 第一重回波峰值
	uint32_t peak1_position; // 第二重回波前沿
	uint32_t peak1_value; // 第二重回波峰值
	uint32_t rising_edge_distance; // 上升沿距离（第一重回波前沿）
	uint32_t falling_edge_distance; // 下降沿距离（第一重回波后沿）
	uint32_t pulse_width; // 脉宽
	uint64_t timestamp; // Unit: us
} one_mf150_fl6031_point_cloud_point;

#define MF150_FL6031_RAW_GROUP_BYTES 1024
#define MF150_FL6031_RAW_GROUP_PIXELS 32

typedef struct {
	uint8_t sof; // 0xAA
	uint16_t frame_id; // 帧 ID
	uint16_t group_id; // group 序号 0~1709
	uint16_t data_len; // 1024
	uint64_t timestamp; // 采集时间戳 us
	uint8_t raw_pixels[MF150_FL6031_RAW_GROUP_BYTES]; // 32 像素 × 32 字节
} mf150_fl6031_raw_group_packet_t;

typedef struct {
	uint8_t sof; // 0xAA
	uint16_t frame_id;
	uint16_t group_id;
	uint16_t point_count; // 本包有效点数
	uint64_t timestamp; // us
} mf150_fl6031_point_cloud_head_t;

typedef struct {
	mf150_fl6031_point_cloud_head_t head;
	uint8_t value[1];
} mf150_fl6031_point_cloud_packet_t;
#elif MF150_FL6532 == 1
typedef struct {
	uint16_t slot_number;
	uint8_t delay[10];
	uint8_t reserved[18];
	uint8_t value[1];
} org_point_cloud_packet_t;

typedef struct {
	uint8_t sof;
	uint16_t frame_id;
	uint16_t data_len;
	uint8_t value[1];
} mf150_fl6532_histogram_packet_t;

#define MF150_FL6532_RAW_GROUP_BYTES 1472
#define MF150_FL6532_PIXELS_PER_SEND 3
#define MF150_FL6532_PIXEL_INFO_SIZE 443
#define MF150_FL6532_HISTOGRAM_PACKET_SIZE \
	(sizeof(mf150_fl6532_histogram_packet_t) - 1 + MF150_FL6532_PIXELS_PER_SEND * MF150_FL6532_PIXEL_INFO_SIZE)
#define MF150_FL6532_HISTOGRAM_MEMORY_POOL_MAX_CNT (18240 * 8) // 约 145920 个块，约 65MB

/* 点云内存池: 每帧 36 个group * 每group约32个udp包 约为 1152包，8帧缓冲 */
#define MF150_FL6532_POINT_CLOUD_MEMORY_POOL_MAX_CNT (1152 * 8)

#define MF150_FL6532_RAW_GROUP_PIXELS 32

typedef struct {
	uint8_t sof; // 0xAA
	uint16_t frame_id; // 帧 ID（同 echo->frame_cnt）
	uint16_t group_id; // 虚拟 group 索引 （vir_gruop_idx）
	uint16_t data_len; // 本包总长 = 6 + 34 + dot_num * pixel_len
	uint64_t timestamp; // 帧基准时间戳 us
	uint8_t data_type; // 协议头 data_type (原始值)
	/* === echo 头字段平铺 (对应 fs_lidar_pcl_echo_data_t, pack(1)) 共 34 字节 === */
	uint8_t echo_protocol_ver_minor;
	uint16_t echo_data_sequence;
	uint16_t echo_frame_cnt;
	uint16_t echo_dot_num;
	uint8_t echo_type; // 映射后的点云类型(FS_PCL_DIS_XYZ/FS_CICL_DATA/FS_GL_DATA)
	uint8_t echo_peak_type_2_1;
	uint8_t echo_frame_flag; // START/NORMAL/END
	uint16_t echo_width;
	uint16_t echo_height;
	uint16_t echo_max_bin_cnt;
	uint8_t echo_ab_frame_idx;
	uint8_t echo_max_sub_frame_cnt;
	uint8_t echo_roi_index;
	uint16_t echo_pixel_len;
	uint16_t echo_data_offset;
	uint16_t echo_data_len;
	uint8_t echo_peak_type_4_3;
	uint64_t echo_timestamp_base;
	uint8_t raw_pixels[MF150_FL6532_RAW_GROUP_BYTES]; // dot_num*pixel_len ≤ 1426 字节
} mf150_fl6532_raw_group_packet_t;

typedef struct {
	uint8_t sof; // 0xAA
	uint16_t frame_id;
	uint16_t group_id;
	uint16_t point_count; // 本包有效点数
	uint64_t timestamp; // us
} mf150_fl6532_point_cloud_head_t;

/* ==== Range LZ4 压缩数据包 (producer -> consumer -> 上位机) ====
 *
 * 板端对 Range 像素流做"按 push 字节转置 + LZ4 Frame 压缩",
 * 压缩流按节点容量分片发送, 上位机按 seq 重组后解压并逆转置。
 *
 * 一帧 = 一个 LZ4F 流; 帧首包携带元信息(像素布局/ROI 描述),
 * 上位机据此完成: 解压 -> 按 push 逆转置 -> 按 (roi,row,col) 拼图。
 */
#define MF150_FL6532_RANGE_LZ4_VERSION 1
#define MF150_FL6532_RANGE_LZ4_SOF 0xAA
#define DATA_TYPE_CICL_LZ4 0x30 // 自定义 data_type: Range LZ4 压缩数据

/* Range 发送路径选择: 1 = LZ4 压缩发送(转置+压缩), 0 = 原始像素发送 */
#ifndef RANGE_TX_LZ4
#define RANGE_TX_LZ4 1
#endif

/* 帧元信息: 仅帧首包携带 */
typedef struct {
	uint16_t pixel_len; // 1 个像素的字节数(278)
	uint32_t total_pixels; // 本帧像素总数(26880)
	uint8_t push_cnt; // 本帧 push 数(44)
	uint8_t group_cnt; // 虚拟 group 总数(44)
	uint8_t roi_cnt; // ROI 数(2)
	uint8_t max_sub_frame_cnt; // 最大子帧数(4)
	uint8_t peak_num; // 峰数(2)
	uint8_t group_row_unit; // group 行单位(spad 数, 12)
	uint16_t max_bin_cnt; // 最大 bin 长(2048)
	uint8_t reserved[2];
	struct {
		uint16_t width; // ROI 宽(列数)
		uint16_t height; // ROI 高(行数)
		uint16_t push_pixels; // 本 ROI 每个 group 单次 push 的像素数
		uint8_t group_cnt; // 本 ROI 的 group 数
		uint8_t group_start; // 本 ROI 首个物理 group 索引
		uint8_t sub_frame_cnt; // 本 ROI 子帧数
		uint8_t pcl_unit; // 点云单位编码
		float range_max; // 量程(cm)
	} roi[4];
} mf150_fl6532_range_lz4_meta_t;

/* 压缩包基础头: 每包携带 */
typedef struct {
	uint8_t sof; // 0xAA
	uint8_t version; // 协议版本(1)
	uint16_t frame_id; // 雷达帧号(frm_id)
	uint16_t seq; // 帧内包序号(从 0 递增, 用于重组)
	uint8_t frame_flag; // 0=START, 1=NORMAL, 2=END
	uint8_t sub_frame_idx; // 子帧号(ROI1 为 0~3)
	uint16_t data_len; // 本包总长(头+有效负载)
	uint16_t payload_len; // 本包压缩负载有效长度
	uint16_t reserved;
	uint8_t payload[]; // 压缩流片段
} mf150_fl6532_range_lz4_hdr_t;

/* 帧首包: 基础头 + 元信息 + 负载 */
typedef struct {
	mf150_fl6532_range_lz4_hdr_t hdr;
	mf150_fl6532_range_lz4_meta_t meta;
	uint8_t payload[];
} mf150_fl6532_range_lz4_first_t;

typedef struct {
	mf150_fl6532_point_cloud_head_t head;
	uint8_t value[1];
} mf150_fl6532_point_cloud_packet_t;

typedef struct {
	float x;
	float y;
	float z;
} point_3d;

typedef struct {
	uint16_t row_position;
	uint16_t column_position;
	float distance; // 单位 cm
	uint32_t reflectivity; // 反射率
	point_3d position_3d; // 真实坐标值
	uint32_t peak0_position; // 第一重回波前沿
	uint32_t peak0_value; // 第一重回波峰值
	uint32_t peak1_position; // 第二重回波前沿
	uint32_t peak1_value; // 第二重回波峰值
	uint32_t rising_edge_distance; // 上升沿距离（第一重回波前沿）
	uint32_t falling_edge_distance; // 下降沿距离（第一重回波后沿）
	uint32_t pulse_width; // 脉宽
	uint64_t timestamp; // Unit: us
} mf150_fl6532_point_cloud_point;

#endif

typedef struct {
	uint8_t sof; // 起始字节，固定为 0xAA
	uint16_t frame_id; // 帧 ID，在 0~65535 循环递增，当 slot == 0 时自增
	uint16_t slot_number; // slot 序号 最大值 799
	uint16_t seq_num; // 数据段序列号
	uint16_t data_len; // 数据区长度
	uint8_t value[1]; // 数据区，格式为 raw_packet_t 或者 org_point_cloud_packet_t
} raw_protocal_data_packet_t; // 通信协议中的原始数据格式

// typedef struct echo_data_h {
// 	uint8_t ft[3];
// 	uint16_t wd;
// 	uint16_t pk;
// 	uint8_t reserved[3];
// } echo_data_t;

typedef struct echo_data_h {
	uint8_t ft[3]; // 前沿时间
	uint8_t et[3]; // 后沿时间
	uint16_t pk; // 峰值
} echo_data_t;

// typedef struct point_cloud_piexl_data_h {
// 	echo_data_t echo_data[3];
// 	uint8_t reserved[2];
// } point_cloud_pixel_data_t;

typedef struct point_cloud_piexl_data_h {
	echo_data_t echo_data[2];
} point_cloud_pixel_data_t;

typedef struct cartesian_point_data_h {
	float_union_t x;
	float_union_t y;
	float_union_t z;
	uint8_t albedo;
} cartesian_point_data_t; // 笛卡尔坐标系/直角坐标系中的一个点

typedef struct spherical_point_data_h {
	float_union_t r; // 距离，单位：m
	float_union_t theta; // 俯仰角
	float_union_t omega; // 水平转角
	uint8_t albedo; // 反射率
} spherical_point_data_t; // 极坐标系/球坐标系中的一个点

typedef struct {
	uint8_t delay[10]; // 时间戳，精确到 us
	uint16_t frame_index; // 帧标识符
	uint16_t total_slots; // 转镜扫描一面中的 slot 数量，当前固定为 800
	uint16_t slot_number; // slot 序号 最大值 799
	uint8_t echo_mode; // 0:单回波模式 1:双回波模式
	uint8_t coordinate_mode; // 0:直角坐标系 1:球坐标系
} point_cloud_head_t; // 真实点云的协议报文头

typedef struct {
	point_cloud_head_t p_head;
	uint8_t value[1];
} point_cloud_packet_t; // 点云数据格式

typedef struct {
	char net_mode[16]; // 网络模式，"static" 或 "dhcp"
	char ip[16]; // IP 地址
	char mask[16]; // 子网掩码
	char gateway[16]; // 网关地址
} network_info_t; // 网络信息结构体

typedef struct {
	/* parameter */
	uint16_t lines_v; // 垂直线数
	uint16_t lines_h; // 水平线数
	float_union_t omega0; // 浮点数，水平转角零度偏移量
	float_union_t theta_min; // 浮点数，俯仰角最小值
	float_union_t theta_max; // 浮点数，俯仰角最大值
	float_union_t theta_tilt; //  浮点数，俯仰倾斜角
	uint16_t u; // SPAD 中心在 y 竖直方向上的偏移量（像素单位）
	uint16_t sat_peak_max; // 回波的饱和峰值最大值
	uint16_t sat_peak_min; // 回波的饱和峰值最小值
	uint16_t dist_max; // 可测最远距离
	float_union_t fov_max; // 浮点数，视场角
	float_union_t k; // 浮点数，反射率标定系数

	/* fpga */
	uint8_t threshold_time; // 阈值标定时间点 取值范围 [0,255]（正整数）
	uint8_t threshold_back; // 背景阈值 取值范围 [0,255]（正整数）
	uint32_t vop_bias_freq; // SPAD 偏压频率 取值范围（正整数）
	uint32_t vop_bias_duty; // SPAD 偏压占空比 取值范围 [0，频率的 66%]（正整数）

	/* side_angular_offset */
	uint16_t side_angular_offset_cnt;
	uint16_t *side_angular_offset; // 第 x 面的水平角相对于第 0 面的误差

	/* standard_encoder */
	uint16_t standard_encoder_cnt;
	uint16_t *angle_face; // 第 x 面转镜的起始发光编码器值

	/* side_light */
	uint8_t side_light; // 每面发光使能设置

	/* vcsel_index */
	uint16_t pixel_in_vcsel_id_cnt;
	uint16_t *pixel_in_vcsel_id; // 属于 VCSEL 的像素结束序号

	/* light_intensity */
	uint16_t int_ld_cnt;
	float_union_t *int_ld; // LD 的发光强度 取值范围 0~1（默认 1）

	/* horizontal_angular_offset */
	uint16_t omega_delta_cnt;
	float_union_t *omega_delta; // pixel 的水平转角

	/* zero_distance_offset */
	uint16_t d_delta_cnt;
	float_union_t *d_delta; // VCSEL 的零点距离偏移量

	/* distance_factor */
	uint16_t a_cnt;
	float_union_t *a; // 距离系数
} calibration_parm_t; // 标定参数结构体

typedef struct {
	/* parameter */
	uint16_t lines_v; // 垂直线数
	uint16_t lines_h; // 水平线数
	float_union_t omega0; // 浮点数，水平转角零度偏移量
	float_union_t theta_min; // 浮点数，俯仰角最小值
	float_union_t theta_max; // 浮点数，俯仰角最大值
	float_union_t theta_tilt; //  浮点数，俯仰倾斜角
	uint16_t u; // SPAD 中心在 y 竖直方向上的偏移量（像素单位）
	uint16_t sat_peak_max; // 回波的饱和峰值最大值
	uint16_t sat_peak_min; // 回波的饱和峰值最小值
	uint16_t dist_max; // 可测最远距离
	float_union_t fov_max; // 浮点数，视场角
	float_union_t k; // 浮点数，反射率标定系数

	uint16_t side_angular_offset_cnt;
	uint16_t standard_encoder_cnt;
	uint16_t pixel_in_vcsel_id_cnt;
	uint16_t int_ld_cnt;
	uint16_t omega_delta_cnt;
	uint16_t d_delta_cnt;
	uint16_t a_cnt;
	uint8_t value[1];
} calibration_parm_head_t; // 标定参数头部的结构体（不带指针）

typedef struct {
	/* system */
	int log_level; // 日志等级
	char log_file_path[64]; // 日志文件路径
	FILE *fp; // 日志文件描述符
	uint8_t work_mode; // 工作模式，0:原始直方图 1:原始点云 2:解算点云 3:灰度图
	bool enable_reflectivity; // 运行时启用反射率标定库
	bool enable_spot_slope; // 运行时启用光斑斜率修正
	uint8_t echo_mode; // 回波模式，0:单回波模式 1:双回波模式
	uint8_t echo_type; // 回波类型，0:最强回波 1:最后回波 2:第一个回波
	uint8_t coordinate_mode; //坐标系模式，0:直角坐标系 1:球坐标系

	/* hardware */
	uint8_t vcsel_start_location; // 硬件上 vcsel 起始位置
	uint8_t fps; // 帧率 10Hz 或者 20Hz

	/* network */
	int center_broadcast_port; // 广播端口
	char center_ip[32]; // 上位机 ip 地址
	int local_cmd_port; // 本地指令端口
	int center_cmd_port; // 指令端口
	int local_data_port; // 本地数据端口
	int center_data_port; // 数据端口
	int default_state; // 默认状态
	uint8_t handshake_timeout_sec; // 心跳超时时间 单位秒
	int frame_delay; // 每帧间隔时间，单位 ms
	uint8_t protocol; // 发送数据的协议类型 可选：udp/tcp

	/* config */
	int set_spad; // 是否配置 SPAD 寄存器 0：不配置 1：配置
	int set_pl_threshold; // 是否配置 FPGA 阈值寄存器 0：不配置 1：配置
	int set_pl_zero_angle; // 是否配置零位角寄存器 0：不配置 1：配置
	int set_pl_standard_encoder; // 是否配置起始发光编码器值寄存器 0：不配置 1：配置
	int set_pl_side_en; // 是否配置面使能寄存器 0：不配置 1：配置
	int deal_pl_system_state; // 是否响应 FPGA 系统状态寄存器 0：不响应 1：响应
	int set_vop_bias; // 是否设置 SPAD 偏压 0：不配置 1：配置

	/* thread */
	bool en_dbus_thread; // 是否启用 D-Bus 线程 0：不启用线程 1：启用线程
	bool en_camera_trigger_thread; // 是否获取相机 trigger 信号 0：不启用线程 1：启用线程
	bool en_pl_system_state_thread; // 是否获取 FPGA 系统状态寄存器 0：不启用线程 1：启用线程
	bool en_report_encoder_thread; // 是否上报电机实时编码值 0：不启用线程 1：启用线程

} configuration_t; // 配置参数结构体
#pragma pack(pop)

#endif // YS_CORE_DEF_H
