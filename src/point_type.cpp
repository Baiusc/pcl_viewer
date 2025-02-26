/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-11-28 14:48:55
 * @LastEditors  : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-02-17 10:30:40
 * @FilePath     : /calib_r_offset/src/point_type.cpp
 * @Description  : 点云格式定义，格式转换，按行列截取操作
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */
#include "point_type.h"
#include <stdexcept>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <algorithm>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace alog_lidar
{
    // 将 PointMatrix 转换为 8 位单通道灰度图并保存为 BMP 格式
    void saveMatAsBMP(const PointMatrix &mat, const std::string &filename)
    {
        const int width = 800;
        const int height = 96;
        std::vector<uint8_t> image(height * width, 0);
        // 找出最小和最大 peak 值
        float min_peak = std::numeric_limits<float>::max();
        float max_peak = std::numeric_limits<float>::lowest();

        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                const auto &pt = mat[i][j];
                if (pt.is_valid)
                {
                    min_peak = std::min(min_peak, static_cast<float>(pt.peak));
                    max_peak = std::max(max_peak, static_cast<float>(pt.peak));
                }
            }
        }
        min_peak = 700.f; // 20250214 临时修改

        // 遍历每一行
        for (int i = 0; i < height; ++i)
        {
            // 遍历当前行的每个点
            for (int j = 0; j < width; ++j)
            {
                const auto &pt = mat[i][j];
                if (pt.is_valid)
                {
                    // 归一化到 0~255
                    float normalized_peak = (pt.peak - min_peak) / (max_peak - min_peak) * 255.0f;
                    image[i * width + width - 1 - j ] = static_cast<uint8_t>(normalized_peak);
                }
            }
        }

        // BMP 文件头
        uint8_t bmpfileheader[14] = {
            'B', 'M',   // 文件类型
            0, 0, 0, 0, // 文件大小
            0, 0,       // 保留
            0, 0,       // 保留
            54, 4, 0, 0 // 图像数据偏移量（1078 = 54 + 256 * 4）
        };
        // BMP 信息头
        uint8_t bmpinfoheader[40] = {
            40, 0, 0, 0,                         // 信息头大小
            0, 0, 0, 0,                          // 图像宽度
            0, 0, 0, 0,                          // 图像高度
            1, 0,                                // 平面数
            8, 0,                                // 每像素位数
            0, 0, 0, 0,                          // 压缩方式
            0, 0, 0, 0,                          // 图像大小
            0, 0, 0, 0,                          // 水平分辨率
            0, 0, 0, 0,                          // 垂直分辨率
            256 & 0xFF, (256 >> 8) & 0xFF, 0, 0, // 颜色数
            0, 0, 0, 0                           // 重要颜色数
        };
        // 设置文件大小
        int filesize = 54 + 256 * 4 + width * height;
        bmpfileheader[2] = (uint8_t)(filesize);
        bmpfileheader[3] = (uint8_t)(filesize >> 8);
        bmpfileheader[4] = (uint8_t)(filesize >> 16);
        bmpfileheader[5] = (uint8_t)(filesize >> 24);

        // 设置图像宽度和高度
        bmpinfoheader[4] = (uint8_t)(width);
        bmpinfoheader[5] = (uint8_t)(width >> 8);
        bmpinfoheader[6] = (uint8_t)(width >> 16);
        bmpinfoheader[7] = (uint8_t)(width >> 24);
        bmpinfoheader[8] = (uint8_t)(height);
        bmpinfoheader[9] = (uint8_t)(height >> 8);
        bmpinfoheader[10] = (uint8_t)(height >> 16);
        bmpinfoheader[11] = (uint8_t)(height >> 24);

        // 打开文件
        std::ofstream file(filename, std::ios::out | std::ios::binary);
        if (!file)
        {
            throw std::runtime_error("无法打开文件: " + filename);
        }

        // 写入文件头
        file.write(reinterpret_cast<const char *>(bmpfileheader), sizeof(bmpfileheader));
        file.write(reinterpret_cast<const char *>(bmpinfoheader), sizeof(bmpinfoheader));

        // 写入调色板（灰度图）
        for (int i = 0; i < 256; ++i)
        {
            uint8_t color[4] = { (uint8_t)i, (uint8_t)i, (uint8_t)i, 0 };
            file.write(reinterpret_cast<const char *>(color), sizeof(color));
        }

        // 写入图像数据
        file.write(reinterpret_cast<const char *>(image.data()), image.size());

        // 关闭文件
        file.close();
    }
    
    // ...existing code...
    // 弧度转角度
    constexpr float radToDeg(float radians)
    {
        return radians * 180.0f / static_cast<float>(M_PI);
    }
    // 将 PointMatrix 逐行提取到 std::vector<PointData>
    std::vector<PointData> extractPoints(const PointMatrix &mat)
    {
        std::vector<PointData> pts;

        // 遍历每一行
        for (const auto &row : mat)
        {
            // 遍历当前行的每个点
            for (const auto &pt : row)
            {
                if (pt.is_valid)
                { // 只提取有效点
                    pts.push_back(pt);
                }
            }
        }

        return pts;
    }

    // 按指定行列截取矩阵数据
    PointMatrix extractData(const PointMatrix &mat, const std::vector<int> &row_indices, const std::vector<int> &col_indices)
    {
        std::vector<std::vector<PointData>> extracted_data;

        // 遍历指定的行
        for (int row : row_indices)
        {
            if (row < 0 || row >= mat.size())
                continue; // 检查行索引有效性

            std::vector<PointData> extracted_row;

            // 遍历指定行中的指定列
            for (int col : col_indices)
            {
                if (col < 0 || col >= mat[row].size())
                    continue; // 检查列索引有效性

                extracted_row.push_back(mat[row][col]);
            }

            // 将当前行加入结果矩阵
            if (!extracted_row.empty())
            {
                extracted_data.push_back(extracted_row);
            }
        }

        return extracted_data;
    }

    // 对目标矩阵按行重写 r_offset 并重新计算点数据 (检查矩阵的行数是否等于 r_offset 的 size ) （若offset 的大小为 1，则将偏移量应用到矩阵中的所有点）
    void resetROffset(PointMatrix &mat, const std::vector<float> &r_offset)
    {
        // 如果 r_offset 的大小为 1，则使用该唯一的值应用到每一行
        if (r_offset.size() == 1)
        {
            float offset = r_offset[0];
            for (auto &row : mat)
            {
                for (auto &point : row)
                {
                    if (!point.is_valid)
                        continue;      // 跳过无效点
                    point.r += offset; // 应用全局偏移 20241209 改为 r += r_offset
                    // 重新计算 x, y, z
                    float theta_rad = point.theta * M_PI / 180.0;
                    float slot_ang_rad = point.slot_ang * M_PI / 180.0;
                    point.x = point.r * std::cos(theta_rad) * std::cos(slot_ang_rad);
                    point.y = point.r * std::cos(theta_rad) * std::sin(slot_ang_rad);
                    point.z = point.r * std::sin(theta_rad);
                }
            }
        }
        else
        {
            // 检查 r_offset 的大小是否与矩阵的行数匹配
            if (mat.size() != r_offset.size())
            {
                std::cerr << "Error: The size of r_offset (" << r_offset.size()
                          << ") does not match the number of rows in the matrix ("
                          << mat.size() << ")." << std::endl;
                return;
            }
            // 按行偏移
            for (size_t i = 0; i < mat.size(); ++i)
            {
                float offset = r_offset[i];
                for (auto &point : mat[i])
                {
                    if (!point.is_valid)
                        continue;      // 跳过无效点
                    point.r += offset; // 应用该行的偏移 20241209 改为 r += r_offset
                    // 重新计算 x, y, z
                    float theta_rad = point.theta * M_PI / 180.0;
                    float slot_ang_rad = point.slot_ang * M_PI / 180.0;
                    point.x = point.r * std::cos(theta_rad) * std::cos(slot_ang_rad);
                    point.y = point.r * std::cos(theta_rad) * std::sin(slot_ang_rad);
                    point.z = point.r * std::sin(theta_rad);
                }
            }
        }
    }

    // 接收原始矩阵和 r_offset，在不改变原始矩阵的情况下应用偏移量，返回一个新的矩阵 （若offset 的大小为 1，则将偏移量应用到矩阵中的所有点）
    PointMatrix applyROffset(const PointMatrix &mat, const std::vector<float> &r_offset)
    {
        PointMatrix new_mat = mat; // 创建一个新矩阵

        // 应用 r_offset
        resetROffset(new_mat, r_offset);

        return new_mat;
    }
    // 重载 applyROffset 方法，支持点向量输入
    std::vector<PointData> applyROffset(const std::vector<PointData> &pts, const std::vector<float> &r_offset)
    {
        // 创建一个新点云数据，用于存储应用偏移量后的点
        std::vector<PointData> new_pts = pts;
        if (r_offset.size() == pts.size()) // 偏移量大小与点数量相同，逐点应用偏移
        {
            for (size_t i = 0; i < new_pts.size(); ++i)
            {
                auto &point = new_pts[i];
                if (!point.is_valid)
                    continue; // 跳过无效点

                point.r += r_offset[i]; // 应用逐点偏移
                // 重新计算 x, y, z
                float theta_rad = point.theta * M_PI / 180.0;
                float slot_ang_rad = point.slot_ang * M_PI / 180.0;
                point.x = point.r * std::cos(theta_rad) * std::cos(slot_ang_rad);
                point.y = point.r * std::cos(theta_rad) * std::sin(slot_ang_rad);
                point.z = point.r * std::sin(theta_rad);
            }
        }
        else // 偏移量大小不合法，抛出异常
        {
            throw std::invalid_argument("Invalid size of r_offset: must be 1 or equal to the number of points.");
        }

        return new_pts; // 返回应用偏移后的新点向量
    }
    // 重载方法 1：sa_offset 是单一的浮动值
    void resetSaOffset(PointMatrix &mat, float sa_offset)
    {
        // 使用单一的 sa_offset 偏移所有点
        for (auto &row : mat)
        {
            for (auto &point : row)
            {
                if (!point.is_valid)
                    continue;                // 跳过无效点
                point.slot_ang += sa_offset; // 应用全局偏移
                // 重新计算 x, y, z
                float theta_rad = point.theta * M_PI / 180.0;
                float slot_ang_rad = point.slot_ang * M_PI / 180.0;
                point.x = point.r * std::cos(theta_rad) * std::cos(slot_ang_rad);
                point.y = point.r * std::cos(theta_rad) * std::sin(slot_ang_rad);
                point.z = point.r * std::sin(theta_rad);
            }
        }
    }
    // 重载方法 ：sa_offset 是单一的浮动值
    PointMatrix applySaOffset(PointMatrix &mat, float sa_offset)
    {
        PointMatrix new_mat = mat; // 创建一个新矩阵

        resetSaOffset(new_mat, sa_offset); // 应用 sa_offset

        return new_mat;
    }
    // 重载方法 2：sa_offset 是单一的浮动值，mat 改为 std::vector<PointData> pts
    void resetSaOffset(std::vector<PointData> &pts, float sa_offset)
    {
        // 使用单一的 sa_offset 偏移所有点
        for (auto &point : pts)
        {
            if (!point.is_valid)
                continue;                // 跳过无效点
            point.slot_ang += sa_offset; // 应用全局偏移
            // 重新计算 x, y, z
            float theta_rad = point.theta * M_PI / 180.0;
            float slot_ang_rad = point.slot_ang * M_PI / 180.0;
            point.x = point.r * std::cos(theta_rad) * std::cos(slot_ang_rad);
            point.y = point.r * std::cos(theta_rad) * std::sin(slot_ang_rad);
            point.z = point.r * std::sin(theta_rad);
        }
    }
    // 对目标矩阵按行重写 sa_offset，并重新计算点数据 （若offset 的大小为 1，则将偏移量应用到矩阵中的所有点）
    void resetSaOffset(PointMatrix &mat, const std::vector<float> &sa_offset)
    {
        // 如果 sa_offset 的大小为 1，则使用该唯一的值应用到每一行
        if (sa_offset.size() == 1)
        {
            float offset = sa_offset[0];
            for (auto &row : mat)
            {
                for (auto &point : row)
                {
                    if (!point.is_valid)
                        continue;             // 跳过无效点
                    point.slot_ang += offset; // 应用全局偏移
                    // 重新计算 x, y, z
                    float theta_rad = point.theta * M_PI / 180.0;
                    float slot_ang_rad = point.slot_ang * M_PI / 180.0;
                    point.x = point.r * std::cos(theta_rad) * std::cos(slot_ang_rad);
                    point.y = point.r * std::cos(theta_rad) * std::sin(slot_ang_rad);
                    point.z = point.r * std::sin(theta_rad);
                }
            }
        }
        else
        {
            // 检查 sa_offset 的大小是否与矩阵的行数匹配
            if (mat.size() != sa_offset.size())
            {
                std::cerr << "Error: The size of sa_offset (" << sa_offset.size()
                          << ") does not match the number of rows in the matrix ("
                          << mat.size() << ")." << std::endl;
                return;
            }
            // 按行偏移
            for (size_t i = 0; i < mat.size(); ++i)
            {
                float offset = sa_offset[i];
                for (auto &point : mat[i])
                {
                    if (!point.is_valid)
                        continue;             // 跳过无效点
                    point.slot_ang += offset; // 应用该行的偏移
                    // 重新计算 x, y, z
                    float theta_rad = point.theta * M_PI / 180.0;
                    float slot_ang_rad = point.slot_ang * M_PI / 180.0;
                    point.x = point.r * std::cos(theta_rad) * std::cos(slot_ang_rad);
                    point.y = point.r * std::cos(theta_rad) * std::sin(slot_ang_rad);
                    point.z = point.r * std::sin(theta_rad);
                }
            }
        }
    }
    // 接收原始矩阵和 sa_offset，在不改变原始矩阵的情况下应用偏移量，返回一个新的矩阵 （若offset 的大小为 1，则将偏移量应用到矩阵中的所有点）
    PointMatrix applySaOffset(const PointMatrix &mat, const std::vector<float> &sa_offset)
    {
        PointMatrix new_mat = mat; // 创建一个新矩阵

        // 应用 sa_offset
        resetSaOffset(new_mat, sa_offset);

        return new_mat;
    }
    // 重载方法，输入向量点
    std::vector<PointData> applySaOffset(const std::vector<PointData> &pts, const std::vector<float> &sa_offset)
    {
        // 创建一个新点云数据，用于存储应用偏移量后的点
        std::vector<PointData> new_pts = pts;
        if (sa_offset.size() == pts.size())
        {
            for (size_t i = 0; i < new_pts.size(); ++i)
            {
                auto &point = new_pts[i];
                if (!point.is_valid)
                    continue;                   // 跳过无效点
                point.slot_ang += sa_offset[i]; // 应用全局偏移
                // 重新计算 x, y, z
                float theta_rad = point.theta * M_PI / 180.0;
                float slot_ang_rad = point.slot_ang * M_PI / 180.0;
                point.omega_rad = slot_ang_rad;
                point.x = point.r * std::cos(theta_rad) * std::cos(slot_ang_rad);
                point.y = point.r * std::cos(theta_rad) * std::sin(slot_ang_rad);
                point.z = point.r * std::sin(theta_rad);
            }
        }
        // 偏移量大小不合法时，抛出异常或处理错误
        else
        {
            throw std::invalid_argument("Invalid size of sa_offset: must be 1 or equal to the number of points.");
        }

        return new_pts;
    }

    // 对目标矩阵同时应用 r_offset 和 sa_offset，并重新计算点数据
    void resetRsaOffset(PointMatrix &mat, const std::vector<float> &r_offset, const std::vector<float> &sa_offset)
    {
        // 检查 r_offset 和 sa_offset 是否匹配矩阵行数
        if (mat.size() != r_offset.size() || mat.size() != sa_offset.size())
        {
            std::cerr << "Error: The size of offsets does not match the number of rows in the matrix." << std::endl;
            return;
        }

        // 遍历矩阵的每一行并应用相应的 r_offset 和 sa_offset
        for (size_t i = 0; i < mat.size(); ++i)
        {
            float r_offset_value = r_offset[i];
            float sa_offset_value = sa_offset[i];

            for (auto &point : mat[i])
            {
                if (!point.is_valid)
                    continue; // 跳过无效点

                // 应用 r_offset 计算新的 r
                point.r += r_offset_value;

                // 应用 sa_offset 计算新的 slot_ang
                point.slot_ang += sa_offset_value;

                // 重新计算 x, y, z
                // 使用更新后的 r 和 slot_ang 重新计算 x, y, z 坐标
                float theta_rad = point.theta * M_PI / 180.0;
                float slot_ang_rad = point.slot_ang * M_PI / 180.0;
                point.x = point.r * std::cos(theta_rad) * std::cos(slot_ang_rad);
                point.y = point.r * std::cos(theta_rad) * std::sin(slot_ang_rad);
                point.z = point.r * std::sin(theta_rad);
            }
        }
    }

    // 接收原始矩阵和偏移量，在不改变原始矩阵的情况下应用偏移量，返回一个新的矩阵
    PointMatrix applyRsaOffset(const PointMatrix &mat, const std::vector<float> &r_offset, const std::vector<float> &sa_offset)
    {
        PointMatrix new_mat = mat; // 创建一个新矩阵

        // 应用 r_offset 和 sa_offset
        resetRsaOffset(new_mat, r_offset, sa_offset);

        return new_mat;
    }

    // 从 PolarPoint 构建 PointData
    PointData buildPointData(const PolarPoint &src)
    {
        PointData point;

        // 从 PolarPoint 填充 PointData 的基本属性
        point.r = src.r.f;
        point.theta_rad = src.theta_rad.f;
        point.omega_rad = src.omega_rad.f;
        point.theta = radToDeg(src.theta_rad.f);    // 弧度转角度
        point.slot_ang = radToDeg(src.omega_rad.f); // 弧度转角度
        point.intensity = src.intensity;
        point.peak = src.peak;

        // 计算笛卡尔坐标
        point.x = src.r.f * std::cos(src.theta_rad.f) * std::cos(src.omega_rad.f);
        point.y = src.r.f * std::cos(src.theta_rad.f) * std::sin(src.omega_rad.f);
        point.z = src.r.f * std::sin(src.theta_rad.f);

        // 验证点的有效性
        point.is_valid = (src.r.f > 0.0f); // 简单判断距离大于 0 为有效点

        return point;
    }
    // 从 PointData 构建 CartesianPoint （X Y Z 坐标）
    CartesianPoint buildCartesianPt(const PointData &point)
    {
        CartesianPoint src;

        // 填充 PolarPoint
        src.x.f = point.x;
        src.y.f = point.y;
        src.z.f = point.z;

        // 填充强度
        src.intensity = point.intensity;
        src.peak = point.peak;
        return src;
    }
    // 从 PointData 构建 PolarPoint （极坐标）
    PolarPoint buildPolarPt(const PointData &point)
    {
        PolarPoint src;

        // 填充 PolarPoint 的极坐标属性
        src.r.f = point.r;
        src.theta_rad.f = point.theta_rad;
        src.omega_rad.f = point.omega_rad;

        // 填充强度
        src.intensity = point.intensity;
        src.peak = point.peak;

        return src;
    }

    // 将 SrcPoint 转换为矩阵 PointMatrix
    PointMatrix Converter::polar2Mat(const std::vector<PolarPoint> &src, int rows, int cols)
    {
        if (src.size() != static_cast<size_t>(rows * cols))
        {
            throw std::invalid_argument("Source data size does not match rows * cols dimensions.");
        }

        // 初始化矩阵
        PointMatrix matrix(rows, std::vector<PointData>(cols));

        // 填充矩阵
        size_t index = 0;
        for (int col = 0; col < cols; ++col)
        {
            for (int row = 0; row < rows; ++row)
            {
                if (index < src.size())
                {
                    matrix[row][col] = buildPointData(src[index]);
                    matrix[row][col].pixel_idx = row;
                    matrix[row][col].slot_idx = col;
                    ++index;
                }
            }
        }

        return matrix;
    }

    // 将 PointMatrix 转换为 CartesianPoint
    std::vector<CartesianPoint> Converter::mat2Cartesian(const PointMatrix &matrix)
    {
        // 获取矩阵的行数和列数
        int rows = matrix.size();
        if (rows == 0)
        {
            return {}; // 空矩阵直接返回空向量
        }
        int cols = matrix[0].size();

        // 初始化一维向量
        std::vector<CartesianPoint> src;
        src.reserve(rows * cols);

        // 一列一列提取
        for (int col = 0; col < cols; ++col)
        {
            for (int row = 0; row < rows; ++row)
            {
                // 将 PointData 转换为 CartesianPoint
                src.push_back(buildCartesianPt(matrix[row][col]));
            }
        }

        return src;
    }

    CartesianPoint Converter::polar2Cartesian(const PolarPoint &polar)
    {
        CartesianPoint cartesian;

        // 极坐标转笛卡尔坐标
        float r = polar.r.f;             // 距离
        float theta = polar.theta_rad.f; // 俯仰角（弧度）
        float omega = polar.omega_rad.f; // 水平角（弧度）

        // 计算笛卡尔坐标
        cartesian.x.f = r * std::cos(theta) * std::cos(omega);
        cartesian.y.f = r * std::cos(theta) * std::sin(omega);
        cartesian.z.f = r * std::sin(theta);

        // 传递强度和峰值
        cartesian.intensity = polar.intensity;
        cartesian.peak = polar.peak;

        return cartesian;
    }

    std::vector<CartesianPoint> Converter::polar2Cartesian(const std::vector<PolarPoint> &pts_polar)
    {
        // 如果输入数组为空，打印警告并返回空数组
        if (pts_polar.empty())
        {
            std::cerr << "polar2Cartesian Warning: Input polar points array is empty." << std::endl;
            return {}; // 返回空数组
        }

        std::vector<CartesianPoint> cartesian_points;
        cartesian_points.reserve(pts_polar.size()); // 预分配空间，提高性能

        for (const auto &polar : pts_polar)
        {
            // 对每个 PolarPoint 调用 polar2Cartesian 方法进行转换
            cartesian_points.push_back(polar2Cartesian(polar));
        }

        return cartesian_points;
    }
    // 将 PointMatrix 转换为 PolarPoint
    std::vector<PolarPoint> Converter::mat2Polar(const PointMatrix& matrix)
    {
        // 获取矩阵的行数和列数
        int rows = matrix.size();
        if (rows == 0)
        {
            return {}; // 空矩阵直接返回空向量
        }
        int cols = matrix[0].size();

        // 初始化一维向量
        std::vector<PolarPoint> src;
        src.reserve(rows * cols);

        // 一列一列提取
        for (int col = 0; col < cols; ++col)
        {
            for (int row = 0; row < rows; ++row)
            {
                // 将 PointData 转换为 PolarPoint
                src.push_back(buildPolarPt(matrix[row][col]));
            }
        }

        return src;
    }

    // 辅助函数：将 PointData 转换为 PolarPoint
    PolarPoint Converter::buildPolarPt(const PointData& point)
    {
        PolarPoint polar_pt;

        // 设置距离
        polar_pt.r.f = point.r;

        // 设置俯仰角（弧度）
        polar_pt.theta_rad.f = point.theta_rad;

        // 设置水平角（弧度）
        polar_pt.omega_rad.f = point.omega_rad;

        // 设置强度
        polar_pt.intensity = point.intensity;

        // 设置峰值
        polar_pt.peak = point.peak;

        return polar_pt;
    }
    // 生成调试用的 std::vector<PolarPoint>  数据
    std::vector<PolarPoint> Converter::createPolarPt(int rows, int cols)
    {
        std::vector<PolarPoint> src;
        src.reserve(rows * cols);

        for (int col = 0; col < cols; ++col)
        {
            for (int row = 0; row < rows; ++row)
            {
                PolarPoint point;

                // 用行号和列号生成数据
                point.r.f = (row + 1) * 10.f + col + 1;                  // 距离示例数据
                point.theta_rad.f = point.r.f / 10.f;                    // 俯仰角（弧度）
                point.omega_rad.f = ((row + 1) + (col + 1) * 10) / 10.f; // 水平角（弧度）
                point.intensity = row + 1;                               // 强度示例数据

                src.push_back(point);
            }
        }

        return src;
    }

    // 计算 PointMatrix 的质心
    PointData computeCentroid(const PointMatrix &mat)
    {
        // 初始化质心的 x, y, z 坐标总和
        float total_x = 0.0f;
        float total_y = 0.0f;
        float total_z = 0.0f;

        // 计数有效点数量
        size_t valid_point_count = 0;

        // 遍历 PointMatrix
        for (const auto &row : mat)
        {
            for (const auto &point : row)
            {
                if (!point.is_valid)
                    continue; // 跳过无效点

                // 累加有效点的坐标
                total_x += point.x;
                total_y += point.y;
                total_z += point.z;

                ++valid_point_count;
            }
        }

        // 如果没有有效点，返回原点作为质心
        if (valid_point_count == 0)
        {
            std::cerr << "Error: No valid points in the matrix to compute the centroid." << std::endl;
        }

        // 计算质心坐标
        PointData centroid;
        centroid.x = total_x / valid_point_count;
        centroid.y = total_y / valid_point_count;
        centroid.z = total_z / valid_point_count;
        centroid.is_valid = true;

        return centroid;
    }

}
