/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 23:23:30
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 19:38:59
 * @FilePath     : /you_see_arm/ys_business/ys_msg_distribute.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_MSG_DISTRIBUTE_H
#define YS_MSG_DISTRIBUTE_H

#include <stdint.h>
#include <uv.h>

#include "ys_def.h"

#define TAB_MAX_ROW 6 // 最大行数
#define TAB_MAX_COL 32 // 最大列数

#define CMD_SET_COMMON 0x00 // 通用指令集
#define CMD_SET_PARAMETER 0x01 // 参数指令集
#define CMD_SET_STATE 0X02 // 状态指令集
#define CMD_SET_SYSTEM 0X03 // 系统指令集
#define CMD_SET_DEBUG 0x04 // 调试指令集

#define AWAYS_AVAILABLE 255 // 任何状态都可用

#define CHECK_CMD_AVAILABLE(avai_state, now_state) ((avai_state) & (1 << now_state))

#define SET_AVAIL_STATE(avai_state) (0 | (1 << (avai_state)))

/***********************************通用指令集ID号******************************** */
enum {
	MSG_ID_BROADCAST = 0x00, // 广播
	CMD_ID_GET_MODEL = 0x01, // 获取雷达型号
	CMD_ID_HANDSHAKE = 0x02, // 网络握手指令
	CMD_ID_HEARTBEAT = 0x03, // 心跳指令
	CMD_ID_SET_IP = 0x04, // 设置设备 IP 信息
	CMD_ID_GET_IP = 0x05, // 获取设备 IP 信息
	CMD_ID_GET_MAC = 0x06, // 获取设备 MAC 地址
	CMD_ID_BREAK_HANDSHAKE = 0x07, // 解除网络握手
};

/***********************************参数指令集ID号******************************** */
enum {
	MSG_ID_HORIZONTAL_CORNER_DATA = 0x00, // 上传水平角度
	CMD_ID_SET_RS_FILTRATION = 0x02, // 设置雨雪过滤
	CMD_ID_SET_ANIT_SUNLIGHT = 0x03, // 设置抗阳光算法
	CMD_ID_SET_ECHO_PARM = 0x04, // 设置点云回波模式
	CMD_ID_GET_ECHO_PARM = 0x05, // 获取点云回波模式
	CMD_ID_SET_REFRESH_RATE = 0x06, // 设置刷新频率
	CMD_ID_GET_REFRESH_RATE = 0x07, // 获取刷新频率
	CMD_ID_SET_COORDINATE = 0x08, // 设置坐标系选项
	CMD_ID_GET_COORDINATE = 0x09, // 获取坐标系选项
};

/***********************************状态指令集ID号******************************** */
enum {
	CMD_ID_GET_RUN_STATE = 0x00, // 获取当前运行状态
	CMD_ID_GET_SELF_STATE = 0x01, // 获取自检状态表
	CMD_ID_SET_STATE = 0x02, // 切换状态
	CMD_ID_GET_LAST_ERROR_CODE = 0x03, // 获取最近一次的错误码
};

/***********************************系统指令集ID号******************************** */
enum {
	/* 0x00 原「查看 FPGA 固件版本」已废弃: FPGA 版本作为一行并入 0x0302 的组件版本列表。
	 * 其余项必须保留显式数值, 否则删掉首项后枚举会整体前移、把所有指令 ID 挪错 */
	CMD_ID_UPGRADE_FPGA = 0x01, // 升级 FPGA
	CMD_ID_GET_SYS_VERSION = 0x02, // 查看系统固件版本: 固件总包 + 各组件版本列表
	CMD_ID_UPGRADE_SYS = 0x03, // 升级系统
	CMD_ID_REBOOT = 0x04, // 重启设备
	CMD_ID_GET_SYS_SN = 0x05 // 获取设备 SN 号
};

/***********************************调试指令集ID号******************************** */
enum {
	CMD_ID_DOWNLOAD_SYS_LOG = 0x00, // 下载系统日志
	CMD_ID_CLEAN_SYS_LOG = 0x01, // 清除系统日志
	CMD_ID_DOWNLOAD_KERNEL_LOG = 0x02, // 下载内核日志
	CMD_ID_CLEAN_KERNEL_LOG = 0x03, // 清除内核日志
	CMD_ID_DOWNLOAD_APP_LOG = 0x04, // 下载应用程序日志
	CMD_ID_CLEAN_APP_LOG = 0x05, // 清除应用程序日志
	CMD_ID_GET_CALIBRATION_PARM = 0x06, // 获取标定参数
	CMD_ID_SET_CALIBRATION_PARM = 0x07, // 设置标定参数
	CMD_ID_GET_SHISTOGRAM = 0x08, // 获取单点直方图
	CMD_ID_SET_VCSEL_PARM = 0x09, // 设置 VCSEL 参数
	CMD_ID_GET_VCSEL_PARM = 0x0a, // 获取 VCSEL 参数
	CMD_ID_SET_SPAD_PARM = 0x0b, // 设置 SPAD 参数
	CMD_ID_GET_SPAD_PARM = 0x0c, // 获取 SPAD 参数
	CMD_ID_SET_MOTOR_PARM = 0x0d, // 设置电机参数
	CMD_ID_GET_MOTOR_PARM = 0x0e, // 获取电机参数
	CMD_ID_SET_PL_CALIB_PARM = 0x0f, // 设置 PL 标定参数
	CMD_ID_GET_PL_CALIB_PARM = 0x10, // 获取 PL 标定参数
	CMD_ID_REPORT_ENCODER = 0x11, // 上报电机编码值
	CMD_ID_SET_SIDE_H_OFFSET = 0x12, // 设置水平角度起始偏移量
	CMD_ID_SET_SIDE_LIGHT_EN = 0x13, // 设置面发光使能
	CMD_ID_SET_VCSEL_H_OFFSET = 0x14, // 设置 VCSEL 之间的水平角度偏移量
	CMD_ID_SET_DISTANCE_OFFSET = 0x15, // 设置零点距离偏移量
	/* 0x16~0x22 原为直方图参数指令(set/get hist one), 已废弃; 编号保留, 避免复用造成新旧固件语义冲突 */
};

/**
 * @description  : trigger event
 * @param         {uint8_t} version
 * @param         {uint8_t} *data
 * @param         {int} data_len
 * @return        {*}
 */
void trigger_event(uint8_t version, uint8_t *data, int data_len);

/**
 * @description  : event loop function
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int event_loop();

#endif // YS_MSG_DISTRIBUTE_H
