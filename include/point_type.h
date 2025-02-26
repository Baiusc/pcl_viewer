/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-11-28 14:41:19
 * @LastEditors  : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-01-02 12:02:43
 * @FilePath     : /calib_r_offset/include/point_type.h
 * @Description  : 点云格式定义，格式转换，按行列截取操作
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */
#pragma once
#include <vector>
#include <cmath>
#include <cstdint>
#include <string> 
namespace alog_lidar
{
    // 前端的极坐标点类型
    struct PolarPoint
    {
        union
        {
            uint32_t u;
            float f;
        } r, theta_rad, omega_rad; // 极坐标的距离, 俯仰角（弧度），水平角（弧度）
        uint8_t intensity;      // 强度
        uint16_t peak;      // 峰值
    };
    // 前端的笛卡尔点类型
    struct CartesianPoint
    {
        union
        {
            uint32_t u;
            float f;
        } x, y, z;
        uint8_t intensity; // 强度
        uint16_t peak;        // 峰值
    };

    // 算法点类型
    struct PointData
    {
        uint16_t pixel_idx;           // 行号
        uint16_t slot_idx;           // 列号
        uint16_t peak;           // 峰值
        float r;           // 距离
        float theta_rad;   // 俯仰角（弧度）
        float omega_rad;   // 水平角（弧度）
        float theta;       // 俯仰角（角度）
        float slot_ang;    // 水平角（角度）
        float reflect; // 反射率 0~1.0
        uint8_t intensity; // 强度 0~255
        float x;
        float y;
        float z;
        bool is_valid; // 点的回波数据是否有效
    };

    using PointMatrix = std::vector<std::vector<PointData>>;

    // 将 PointMatrix 转换为 8 位单通道灰度图并保存为 BMP 格式
    void saveMatAsBMP(const PointMatrix &mat, const std::string &filename);

    // 将 PointMatrix 逐行提取到 std::vector<PointData>
    std::vector<PointData> extractPoints(const PointMatrix &mat);

    // 按指定行列截取矩阵数据
    PointMatrix extractData(const PointMatrix &mat, const std::vector<int> &row_indices, const std::vector<int> &col_indices);

    // 对 mat 应用 r_offset
    void resetROffset(PointMatrix &mat, const std::vector<float> &r_offset);

    // 应用偏移量，返回一个新矩阵
    PointMatrix applyROffset(const PointMatrix &mat, const std::vector<float> &r_offset);

    // 重载 applyROffset 方法，支持点向量输入
    std::vector<PointData> applyROffset(const std::vector<PointData> &pts, const std::vector<float> &r_offset);

    // 重载方法 1：sa_offset 是单一的浮动值
    void resetSaOffset(PointMatrix& mat, float sa_offset);

    // 重载方法 2：sa_offset 是单一的浮动值，mat 改为 std::vector<PointData> pts
    void resetSaOffset(std::vector<PointData>& pts, float sa_offset);

    // 对目标矩阵按行重写 sa_offset，并重新计算点数据
    void resetSaOffset(PointMatrix &mat, const std::vector<float> &sa_offset);

    // 重载方法 ：sa_offset 是单一的浮动值
    PointMatrix applySaOffset(PointMatrix &mat, float sa_offset);

    // 应用偏移量，返回一个新矩阵
    PointMatrix applySaOffset(const PointMatrix &mat, const std::vector<float> &sa_offset);

    // 重载方法，输入向量点
    std::vector<PointData> applySaOffset(const std::vector<PointData> &pts, const std::vector<float> &sa_offset);

    // 对目标矩阵按行重写 r_offset 和 sa_offset，并重新计算点数据
    void resetRsaOffset(PointMatrix &mat, const std::vector<float> &r_offset, const std::vector<float> &sa_offset);

    // 应用偏移量，返回一个新矩阵
    PointMatrix applyRsaOffset(const PointMatrix &mat, const std::vector<float> &r_offset, const std::vector<float> &sa_offset);

    // 计算 PointMatrix 的质心
    PointData computeCentroid(const PointMatrix &mat);

    class Converter
    {
    public:
        // 将 vector<PolarPoint> 转换为 PointMatrix
        static PointMatrix polar2Mat(const std::vector<PolarPoint> &src, int rows, int cols);
        // 将 PointMatrix 转换为 vector<CartesianPoint>
        static std::vector<CartesianPoint> mat2Cartesian(const PointMatrix &matrix);
        // 极坐标转笛卡尔坐标
        static CartesianPoint polar2Cartesian(const PolarPoint &polar);
        // 将 vector<PolarPoint> 转换为 vector<CartesianPoint>
        static std::vector<CartesianPoint> polar2Cartesian(const std::vector<PolarPoint> &pts_polar);
        // 生成src数据 (debug用)
        static std::vector<PolarPoint> createPolarPt(int rows, int cols);
        // 将 PointMatrix 转换为 PolarPoint
        static std::vector<PolarPoint> mat2Polar(const PointMatrix& matrix);
        // 辅助函数：将 PointData 转换为 PolarPoint
        static PolarPoint buildPolarPt(const PointData& point);

    };
}
