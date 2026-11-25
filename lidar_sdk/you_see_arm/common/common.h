/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 05:25:01
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-16 23:33:05
 * @FilePath     : /you_see_arm/common/common.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef COMMON_H
#define COMMON_H

#ifndef MODEL
#define MODEL "unknown"
#endif

#ifndef ERR_FPGA_REG32
#define ERR_FPGA_REG32 (0xffffffff)
#endif

#ifndef ERR_FPGA_REG16
#define ERR_FPGA_REG16 (0xffff)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#include "../third_party/log.c/log.h"

#define USE_SOCKET 1 // 0：使用 libuv 发送 udp 数据（仅针对数据线程），1：使用 socket 发送 udp 数据（仅针对数据线程）

#define USE_KFIFO 0 // 0：使用内存池存储原始点云数据，1：使用 kfifo 存储原始点云数据

#define RET_OK 0 // 成功
#define RET_FAIL -1 // 失败

#define MSG_BUF_MAX_LEN 128 // 消息报文最大长度
#define ACK_BUF_MAX_LEN 2048 // 响应报文最大长度
#define RECV_BUF_MAX_LEN 2048 // 接收报文最大长度

#define MAX_PORT 65535 // 端口最大值
#define MIN_PORT 1 // 端口最小值

#define IS_REG 1 // 已网络握手
#define NO_REG 0 // 未网络握手

#define CONNECTED 1 // 连接
#define DISCONNECTED 0 // 断开

#define ON 1 // 开启
#define OFF 0 // 关闭

#define SIMPLEX 1 // 单工通信
#define DUPLEX 0 // 双工通信

#define NO_SILENT 0 // 不沉默，需要响应
#define SILENT 1 // 沉默，不需要响应

#if CS96_ALPHA == 1
#define UDP_SEND_INTERVAL 3 // UDP 发送间隔，单位微秒

#elif CS96_BETA == 1 // 硬件性能较差机型，延长间隔
#define UDP_SEND_INTERVAL 10 // UDP 发送间隔，单位微秒

#elif CS96_GAMMA == 1 // 与 beta 同平台(am6204)，单 SLOT 数据量更大(1936B)，沿用延长间隔
#define UDP_SEND_INTERVAL 10 // UDP 发送间隔，单位微秒

#else
#define UDP_SEND_INTERVAL 3 // UDP 发送间隔，单位微秒

#endif

#define UV_RUN_INTERVAL 10 * 1000 // 事件循环延迟，以免过于频繁地检查事件，单位微秒

#define LINK_ADD_INTERVAL 10 // 链表添加间隔，单位微秒

#define FINISH 1 // 完成
#define UN_FINISH 0 // 未完成

#define NORMAL 1 // 正常
#define ABNORMAL 0 // 异常

#define RAW_MODE 0 // 原始直方图模式
#define TEST_POINT_CLOUD_MODE 1 // 测试点云模式
#define FORMAL_POINT_CLOUD_MODE 2 // 正式点云模式
#define GRAY_MODE 3 // 灰度图模式（gamma 对应 DEF_RCV_MODE=4，beta 对应 DEF_RCV_MODE=5，需与 FPGA 格式配对）

#define SINGLE_ECHO_MODE 0 // 单回波模式
#define DOUBLE_ECHO_MODE 1 // 双回波模式

#define STRONGEST_ECHO_TYPE 0 // 最强回波
#define LAST_ECHO_TYPE 1 // 最后回波
#define FIRST_ECHO_TYPE 2 // 第一个回波

#define LEFT_VCSEL 0 // 左边的 vcsel
#define RIGHT_VCSEL 1 // 右边的 vcsel

#define CARTESIAN_COORDINATE_SYSTEM 0 // 笛卡尔坐标系/直角坐标系
#define SPHERICAL_COORDINATE_SYSTEM 1 // 极坐标系/球坐标系

#define PI 3.1415926

#define WRITABLE 0 // 可写
#define READABLE 1 // 可读
#define SENDABLE 2 // 可发送

#define NO_SET_PRIORITY 0
#define SET_HIGHEST_PRIORITY 99

#define CMD_NOT_AVAILABLE 0

#define UDP 0
#define TCP 1

#define RUNNING 1 // 正在运行
#define STOPPED 0 // 停止运行

#define USE_ZERO_COPY_SENDTO 1 // 使用 零拷贝 + sendto 发送数据
#define USE_SENDMMSG_FUNC 0 // 使用 sendmmsg 发送数据

#endif // COMMON_H
