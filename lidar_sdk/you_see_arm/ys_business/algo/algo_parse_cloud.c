/*
 * @Author       : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @Date         : 2026-04-20 11:27:28
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2026-04-20 14:26:48
 * @FilePath     : /algo_solid_lidar/src/algo_parse_cloud.c
 * @Description  : 固态雷达 FPGA原始数据解析及坐标转换实现
 * 
 * Copyright (c) 2026 by vitalchem, All Rights Reserved. 
 */
#include "algo_parse_cloud.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 内部辅助函数：解析3字节数据 (小端) */
static inline uint32_t Parse3Bytes(const uint8_t* buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
}

/* 内部辅助函数：解析2字节数据 (小端) */
static inline uint16_t Parse2Bytes(const uint8_t* buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

/* 内部辅助函数：根据前沿、后沿、峰值计算反射强度 TODO  */
static inline float CalculateIntensity(uint32_t front_edge, uint32_t rear_edge, uint16_t peak) {
    // 示例：采用脉宽(rear - front)结合峰值评估强度 TODO
    if (rear_edge > front_edge) {
        return (float)(rear_edge - front_edge) * 0.1f + (float)peak * 0.5f;
    }
    return (float)peak;
}

int SslParseRawDataToPolar(
    const uint8_t* raw_buffer, 
    size_t raw_len, 
    const SslCalibParam* calib,
    SslPolarPoint* out_polar_pts, 
    size_t* out_count) 
{
    if (!raw_buffer || !calib || !out_polar_pts || !out_count) {
        return -1; // 参数错误
    }

    // 检查数据长度是否为像素大小的整数倍
    if (raw_len % SSL_RAW_PIXEL_BYTES != 0) {
        return -2; // 长度异常
    }

    size_t valid_pts = 0;
    size_t num_pixels = raw_len / SSL_RAW_PIXEL_BYTES;

    for (size_t i = 0; i < num_pixels; ++i) {
        const uint8_t* pixel_buf = raw_buffer + i * SSL_RAW_PIXEL_BYTES;

        // 1. 解析坐标信息 (字节0-2)
        uint32_t coord_info = Parse3Bytes(&pixel_buf[0]);
        uint16_t row = coord_info & 0x1FF;          // bit 0-8
        uint16_t col = (coord_info >> 9) & 0x1FF;   // bit 9-17

        // 行列合法性校验
        if (row >= SSL_MAX_ROWS || col >= SSL_MAX_COLS) {
            continue; 
        }

        // 计算该像素的光学角度 (查表或线性映射) TODO
        float theta = calib->start_theta_rad + row * calib->step_theta_rad;
        float omega = calib->start_omega_rad + col * calib->step_omega_rad;

        // 2. 解析三重回波
        // 偏移分别为 3, 11, 19
        size_t echo_offsets[3] = {3, 11, 19};

        for (uint8_t e = 0; e < SSL_MAX_ECHOES; ++e) {
            const uint8_t* echo_buf = &pixel_buf[echo_offsets[e]];
            
            uint32_t front_edge = Parse3Bytes(&echo_buf[0]);
            uint32_t rear_edge  = Parse3Bytes(&echo_buf[3]);
            uint16_t peak       = Parse2Bytes(&echo_buf[6]);

            // 判断回波是否有效 (约定：如果前沿或峰值为0，则判定为无效点无反射)
            if (front_edge > 0 || peak > 0) {
                SslPolarPoint* pt = &out_polar_pts[valid_pts];
                
                pt->r         = (float)front_edge * calib->tof_lsb_to_meter;
                pt->theta_rad = theta;
                pt->omega_rad = omega;
                pt->intensity = CalculateIntensity(front_edge, rear_edge, peak);
                pt->peak      = peak;
                pt->row_idx   = row;
                pt->col_idx   = col;
                pt->echo_idx  = e;

                valid_pts++;
            }
        }
    }

    *out_count = valid_pts;
    return 0;
}

int SslPolarToCartesian(
    const SslPolarPoint* in_polar_pts, 
    size_t point_count, 
    SslCartesianPoint* out_xyz_pts) 
{
    if (!in_polar_pts || !out_xyz_pts) {
        return -1;
    }

    for (size_t i = 0; i < point_count; ++i) {
        const SslPolarPoint* p_in = &in_polar_pts[i];
        SslCartesianPoint* p_out = &out_xyz_pts[i];

        // 复制非坐标数据
        p_out->intensity = p_in->intensity;
        p_out->peak      = p_in->peak;
        p_out->row_idx   = p_in->row_idx;
        p_out->col_idx   = p_in->col_idx;
        p_out->echo_idx  = p_in->echo_idx;

        // 极坐标转空间直角坐标 
        // x = r * cos(theta) * cos(omega)
        // y = r * cos(theta) * sin(omega)
        // z = r * sin(theta)
        float cos_theta = cosf(p_in->theta_rad);
        
        p_out->x = p_in->r * cos_theta * cosf(p_in->omega_rad);
        p_out->y = p_in->r * cos_theta * sinf(p_in->omega_rad);
        p_out->z = p_in->r * sinf(p_in->theta_rad);
    }

    return 0;
}