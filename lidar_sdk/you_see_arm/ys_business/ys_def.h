/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-06-04 05:18:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-16 23:11:58
 * @FilePath     : /you_see_arm/ys_business/ys_def.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_DEF_H
#define YS_DEF_H

#include "../common/common.h"

#include "../core/ys_core_def.h"
#include "../core/ys_memory_pool.h"
#include "../network/network.h"

#include <pthread.h>
#include <stdbool.h>

#if MF150_FL6532 == 0
#include "yousee_lidar_api_public.h"
#endif

// (0xA411C0 / 4000) * 2 = 5378 创建一个可以存储两帧原始直方图数据的内存池
#define ORG_HISTOGRAM_MEMORY_POOL_MAX_CNT 5378

#if CS96_BETA == 1
// beta 灰度: 一帧 168 个 SLOT，每 SLOT 拆 2 片(共 336 块/帧)，
// 预留 10 帧缓冲(168 × 2 × 10 × 1165B ≈ 3.9MB)
#define ORG_GRAY_MEMORY_POOL_MAX_CNT (168 * 2 * 10)
#else
// 灰度图一帧 20 个 SLOT，预留 200 帧缓冲(20 × 200 × 977B ≈ 3.9MB)
#define ORG_GRAY_MEMORY_POOL_MAX_CNT (20 * 200)
#endif

#define MAX_THPOOL_CNT 5

#define MY_GLOBAL_SIGNAL (SIGRTMAX - 10) // 自定义的信号值，用于分配给不同的线程作为退出信号，使用于全局的线程
#define MY_SIGNAL (SIGRTMIN + 1) // 自定义的信号值，用于分配给不同的线程作为退出信号

typedef struct {
#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	struct lidar_fpga_selftest *fpga_state;
#endif
	pthread_mutex_t lock;
} sys_state_t;

typedef struct ys_state_machine {
	int state;
	pthread_mutex_t lock;
} ys_state_machine_t;

typedef void (*event_handler_t)(void *arg);

#pragma pack(push, 1)
typedef struct {
	uint8_t code_set; // 指令集
	uint8_t code_id; // 指令 ID
	uint8_t avail_state; // 在何种状态下可用
	event_handler_t handler; // 处理函数
	char silent; // 是否响应
	uint8_t version; // 协议版本号
	uint8_t *buf; // 响应内容
} management_id_t;

typedef struct {
	uint8_t sof; // 起始字节，固定为 0xAA
	uint8_t version; // 协议版本，当前为 1
	uint16_t length; // 数据段长度，最大值：1400
	uint8_t cmd_type; // 命令类型
	uint16_t seq_num; // 数据段序列号
	uint16_t crc_16; // 包头校验码
} cmd_head_packet_t;

typedef struct {
	uint32_t crc_32; // 整个数据段校验码
} cmd_tail_packet_t;

#define CMD_MAX_LEN 1400
#define ACK_DATA_MAX_LEN (CMD_MAX_LEN - sizeof(cmd_head_packet_t) - sizeof(cmd_tail_packet_t) - 2)

typedef struct {
	uint8_t code_set; // 指令集
	uint8_t code_id; // 指令 ID
	uint8_t ack_data[1385]; // 响应内容
} cmd_body_packet_t;

typedef struct {
	uint8_t sn[NET_SN_LENGTH]; // 完整 SN，或以 NUL 填满的 MAC 兼容标识
	uint8_t work_mode;
	uint8_t echo_mode;
	uint8_t echo_type;
	uint8_t coordinate_mode;
	uint16_t line_counts;
	uint16_t slot_counts;
} boradcast_info_t;

typedef struct {
	uint8_t ret_code;
	uint8_t model;
} model_info_t;

typedef struct {
	uint8_t ret_code;
	uint8_t ip_addr[4];
	uint8_t net_mask[4];
	uint8_t gw_addr[4];
} ip_info_t;

typedef struct {
	uint8_t ret_code;
	uint8_t mac_addr[6];
} mac_info_t;

typedef struct {
	uint8_t ret_code;
	uint8_t echo_mode;
	uint8_t echo_type;
} point_cloud_echo_info_t;

typedef struct {
	uint8_t ret_code;
	uint32_t frequency;
} frame_rate_t;

typedef struct {
	uint8_t ret_code;
	uint8_t mode;
} coordinate_info_t;

typedef struct {
	uint8_t ret_code;
	uint8_t state;
} ps_state_info_t;

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
typedef struct {
	uint8_t ret_code;
	struct lidar_fpga_selftest pl_state;
	int16_t ld_temp;
	int16_t spad_temp;
	int16_t fpga_temp;
	uint32_t motor_cycles;
	uint32_t vop_voltage;
	uint8_t ps_state;
} device_state_info_t;
#endif

/* 组件 ID, 顺序即上报顺序 */
typedef enum {
	YS_COMPONENT_YOU_SEE = 0, // you_see 应用(总成)
	YS_COMPONENT_ARM_DRIVER = 1, // ARM 驱动
	YS_COMPONENT_ALGORITHM = 2, // 算法
	YS_COMPONENT_FPGA = 3, // FPGA
	YS_COMPONENT_WEB_SERVICE = 4, // Web 服务程序
	YS_COMPONENT_MAX = 5
} ys_component_id_t;

/* 单个组件的版本标识(32 B) */
typedef struct {
	uint8_t id; // 组件 ID, 见 ys_component_id_t
	uint8_t valid; // 1 = 该组件版本有效; 0 = 尚未接入(占位)
	uint8_t major; // 该组件 主版本号
	uint8_t minor; // 该组件 次版本号
	uint8_t patch; // 该组件 修订版本号
	uint8_t dirty; // 1 = 该组件源码有未提交改动, 该组件二进制不可复现
	uint8_t sha_len; // git_sha 有效长度, 0 表示无 git 信息(dirty 此时不可信)
	char git_sha[12]; // git 短 sha(ASCII), 不足补 0
	uint32_t build_time; // 构建时刻 UTC Unix 秒 低 32 位
	uint32_t build_time_hi; // 构建时刻 UTC Unix 秒 高 32 位(2038-01-19 前恒 0)
	uint8_t reserved[5]; // 预留, 填 0
} version_component_t;

/* 0x0302 查看系统固件版本 应答头部: 固件总包信息(40 B) */
typedef struct {
	uint8_t ret_code; // 0x00 成功; 0x03 获取失败
	uint8_t component_count; // 后续组件记录条数
	uint8_t fw_major; // 固件总包 主版本号(来自 v1.2.3 形式 tag)
	uint8_t fw_minor; // 固件总包 次版本号
	uint8_t fw_patch; // 固件总包 修订版本号
	uint8_t reserved0; // 预留, 填 0
	char product[16]; // 产品名(ASCII, NUL 结尾)
	char release_id[16]; // 发布标识(ASCII, NUL 结尾)
	uint8_t reserved1[2]; // 预留, 填 0
} version_head_t;

/* 0x0302 应答完整结构: 固件总包头部 + 各组件版本记录列表 */
typedef struct {
	version_head_t head;
	version_component_t component[YS_COMPONENT_MAX];
} version_list_t;

/* 线上结构体的尺寸必须与协议文档严格一致: 悄悄变长正是本功能要防的那类事故 */
_Static_assert(sizeof(version_component_t) == 32, "version_component_t 必须为 32 字节");
_Static_assert(sizeof(version_head_t) == 40, "version_head_t 必须为 40 字节");
_Static_assert(sizeof(version_list_t) == 200, "version_list_t 必须为 200 字节");

typedef struct {
	uint8_t ret_code;
	uint8_t sn[10];
} sn_info_t;

typedef struct {
	uint8_t ret_code;
	calibration_parm_t cal_param;
} calibration_info_t;

typedef struct {
	uint8_t ret_code;
	uint8_t shistogram_data[2000];
} shistogram_data_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
} set_pl_respond_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} vcsel_chan_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} vcsel_refreq_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	int16_t value;
} ld_temp_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint8_t value;
} calib_lightnd_delay_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	int16_t value;
} spad_temp_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} vop_sample_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} spad_pwm_freq_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} spad_pwm_duty_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} spad_spi_read_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	float_union_t value;
} motor_agle_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} motor_zero_time_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	int16_t value;
} calib_zero_offset_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} motor_ctrl_t;

typedef union {
	struct {
		uint32_t enable : 1;
		uint32_t : 7;
		uint32_t p : 8;
		uint32_t i : 8;
		uint32_t d : 8;
	} pid;
	uint32_t u32;
} pid_info_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value[4];
} motor_pid_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} motor_pwm_freq_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} motor_pwm_duty_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} motor_stop_angle_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint32_t value;
} motor_period_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint16_t value[4];
} calib_flat_offset_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint8_t value;
} calib_threshold_time_t;

typedef struct {
	uint8_t ret_code;
	uint8_t cmd_type;
	uint8_t value;
} calib_threshold_back_t;

typedef struct {
	uint8_t ret_code;
	uint8_t value;
} hist_u8_param_t;

typedef struct {
	uint8_t ret_code;
	uint16_t value;
} hist_u16_param_t;

typedef struct {
	uint8_t ret_code;
	uint32_t value;
} hist_u32_param_t;

typedef struct {
	uint16_t cnt;
	float_union_t *value;
} horizontal_corner_data_t;

typedef struct {
	uint8_t cmd_set;
	uint8_t cmd_id;
	uint8_t error_code;
} error_info_t;

typedef struct {
	uint8_t ret_code;
	error_info_t value;
} error_code_t;

typedef struct {
	uint8_t cmd_set;
	uint8_t cmd_id;
	uint8_t ip[4]; // 上位机 IPV4 地址
	uint16_t cmd_port; // 上位机控制指令 UDP 目的端口
	uint16_t cloud_data_port; // 上位机点云数据 UDP 目的端口
	uint16_t org_data_port; // 上位机原始直方图数据 UDP 目的端口
} network_handshake_t;

typedef union trigger_time_h {
	uint8_t delay[8];
	uint32_t time[2];
} trigger_time_t;

typedef struct {
	uint8_t ret_code;
	uint8_t now_state;
} set_run_state_t;

#if CS96_BETA == 1 || CS96_GAMMA == 1
typedef struct {
	uint8_t cnt;
	float_union_t value;
} encoder_info_t;
#endif

#pragma pack(pop)

typedef void *(*pthread_handler_t)(void *arg);

typedef struct {
	int core_id; // 绑定 CPU 核心
	int priority; // 线程优先级
	pthread_handler_t handler;
} p_handler_t;

enum {
	INIT = -1, // 初始化状态
	IDLE = 0, // 空闲状态
	SCAN, // 扫描状态
	FAULT, // 故障状态
	DEBUG, // 调试状态
};

// PS->PL 设置的状态
enum {
	PL_IDLE = 0x00, // 空闲状态
	PL_SCAN = 0x02, // 扫描状态
	PL_FAULT = 0x04, // 故障状态
	PL_DEBUG = 0x08, // 调试状态
	PL_ERROR = 0xFF, // 未知错误
};

// FPGA 的工作模式
enum {
	FPGA_IDLE = 0x00, // 空闲状态
	FPGA_SCAN = 0x01, // 扫描状态
	FPGA_FAULT = 0x02, // 故障状态
	FPGA_DEBUG = 0x04, // 调试状态
};

#endif // YS_DEF_H
