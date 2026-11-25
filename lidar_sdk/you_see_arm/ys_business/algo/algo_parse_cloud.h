/*
 * @Author       : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @Date         : 2026-04-20 14:16:56
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2026-04-20 14:26:23
 * @FilePath     : /algo_solid_lidar/include/algo_parse_cloud.h
 * @Description  : 固态雷达(Solid-State LiDAR) FPGA原始数据解析及点云转换算法 (面向ARM端)（与网络层封包逻辑解耦）
 * 
 * Copyright (c) 2026 by vitalchem, All Rights Reserved. 
 */

#ifndef ALGO_PARSE_CLOUD_H
#define ALGO_PARSE_CLOUD_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- 固态雷达硬件参数宏定义 --- */
#define SSL_MAX_ROWS 152        // 最大行数
#define SSL_MAX_COLS 360        // 最大列数
#define SSL_PIXELS_PER_FRAME (SSL_MAX_ROWS * SSL_MAX_COLS) // 单帧像素数 (54720)
#define SSL_MAX_ECHOES 3        // 每像素最大回波数量
#define SSL_MAX_POINTS_PER_FRAME (SSL_PIXELS_PER_FRAME * SSL_MAX_ECHOES) // 单帧最大输出点数 360*152*3=164160
#define SSL_MAX_POINTS_PER_PACKET (32 * SSL_MAX_ECHOES) // 一个长包内含 32 个像素点，最大可能产生 32 * 3 = 96 个点
#define SSL_RAW_PIXEL_BYTES 32  // 每个像素占用的原始字节数

/* --- 标定参数配置 (由ARM端读取配置后传入) --- */
typedef struct {
    float tof_lsb_to_meter;     // ToF前沿时间转换为距离的系数 (目前TDC的LSB是1ps，对应单程距离约 0.15 毫米)
    float start_theta_rad;      // 第0行的起始垂直俯仰角 (弧度)
    float step_theta_rad;       // 每行的垂直角度步进 (弧度)
    float start_omega_rad;      // 第0列的起始水平方位角 (弧度)
    float step_omega_rad;       // 每列的水平角度步进 (弧度)
} SslCalibParam;

/* --- 输出极坐标点云数据结构 --- */
typedef struct {
    float r;             // 距离 (米)
    float theta_rad;     // 垂直俯仰角 (弧度)
    float omega_rad;     // 水平方位角 (弧度)
    float intensity;     // 反射强度 (可根据脉宽/峰值计算)
    uint16_t peak;       // 原始峰值数据
    uint16_t row_idx;    // 像素行号 (0-151)
    uint16_t col_idx;    // 像素列号 (0-359)
    uint8_t echo_idx;    // 回波序号 (0:第一重, 1:第二重, 2:第三重)
} SslPolarPoint;

/* --- 输出笛卡尔(XYZ)点云数据结构 --- */
typedef struct {
    float x;             // X坐标 (米)
    float y;             // Y坐标 (米)
    float z;             // Z坐标 (米)
    float intensity;     // 反射强度
    uint16_t peak;       // 原始峰值数据
    uint16_t row_idx;    // 像素行号 (0-151)
    uint16_t col_idx;    // 像素列号 (0-359)
    uint8_t echo_idx;    // 回波序号 (0:第一重, 1:第二重, 2:第三重)
} SslCartesianPoint;

/* --- 算法接口 API --- */

/**
 * @brief 解析FPGA传来的原始MIPI负载数据，提取有效回波生成极坐标点云。
 * * @param raw_buffer    FPGA原始数据缓冲区指针 (小端字节序)
 * @param raw_len       原始数据长度 (字节)
 * @param calib         雷达标定参数指针
 * @param out_polar_pts 输出的极坐标点云数组 (需由调用者预先分配足够空间, 单帧最大为 SSL_MAX_POINTS_PER_FRAME)
 * @param out_count     解析出的有效点数量
 * @return int          0: 成功; -1: 参数错误; -2: 数据长度不匹配
 */
int SslParseRawDataToPolar(
    const uint8_t* raw_buffer, 
    size_t raw_len, 
    const SslCalibParam* calib,
    SslPolarPoint* out_polar_pts, 
    size_t* out_count
);

/**
 * @brief 将极坐标点云转换为XYZ笛卡尔点云。
 * * @param in_polar_pts  输入的极坐标点云数组
 * @param point_count   点云数量
 * @param out_xyz_pts   输出的笛卡尔点云数组 (需由调用者预先分配足够空间)
 * @return int          0: 成功; -1: 参数错误
 */
int SslPolarToCartesian(
    const SslPolarPoint* in_polar_pts, 
    size_t point_count, 
    SslCartesianPoint* out_xyz_pts
);

#ifdef __cplusplus
}
#endif

#endif // ALGO_PARSE_CLOUD_H