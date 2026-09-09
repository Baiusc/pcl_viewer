/*
 * @Author       : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @Date         : 2025-02-17 10:32:59
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2026-07-14 14:50:07
 * @FilePath     : /pcl_viewer/src/point_type.cpp
 * @Description  :
 *
 * Copyright (c) 2025 by vitalchem, All Rights Reserved.
 */

#include "point_type.h"
#include <stdexcept>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <algorithm>
#include <sstream>
#include <json.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace algo_lidar
{
    namespace
    {
        constexpr float kRange6532RawPerMetre = 25600.0f;
        constexpr float kLightSpeedMetresPerSecond = 3.0e8f;

        std::string trim(const std::string &text)
        {
            const auto begin = text.find_first_not_of(" \t\r\n");
            if (begin == std::string::npos)
                return {};
            const auto end = text.find_last_not_of(" \t\r\n");
            return text.substr(begin, end - begin + 1);
        }

        void updateRangePoint(PointData &point)
        {
            point.x = point.r * std::cos(point.theta_rad) * std::cos(point.omega_rad);
            point.y = point.r * std::cos(point.theta_rad) * std::sin(point.omega_rad);
            point.z = point.r * std::sin(point.theta_rad);

            if (point.select_idx < point.list_pulse.size())
                point.list_pulse[point.select_idx].tof =
                    point.r * 2.0f / kLightSpeedMetresPerSecond * 1.0e12f;
        }
    } // namespace

    bool loadRange6532CalibrationCsv(const std::string &filename,
                                      Range6532CalibrationTable &table)
    {
        table = {};
        std::ifstream input(filename);
        if (!input.is_open())
        {
            std::cerr << "[6532标定] 无法打开标定表: " << filename << std::endl;
            return false;
        }

        std::string line;
        std::getline(input, line); // 跳过表头：family,index0,index1,...
        bool has_supported_item = false;
        while (std::getline(input, line))
        {
            std::vector<std::string> fields;
            std::stringstream stream(line);
            std::string field;
            while (std::getline(stream, field, ','))
                fields.push_back(trim(field));
            if (fields.size() < 7 || fields[3] != "1")
                continue;

            try
            {
                const int index0 = std::stoi(fields[1]);
                const int index1 = std::stoi(fields[2]);
                const float gain = fields[6].empty() ? 0.0f : std::stof(fields[6]);

                if (fields[0] == "global" && !fields[4].empty())
                {
                    table.global_offset_m = std::stof(fields[4]) / kRange6532RawPerMetre;
                    table.global_gain = gain;
                    table.global_supported = true;
                }
                else if (fields[0] == "global_gain")
                {
                    table.global_gain = gain;
                    table.global_gain_supported = true;
                }
                else if (fields[0] == "intra_row_offset" && !fields[4].empty() && index1 >= 0 && index1 < 4)
                {
                    table.intra_row_offset_m[index1] = std::stof(fields[4]) / kRange6532RawPerMetre;
                    table.intra_row_supported[index1] = true;
                }
                else if (fields[0] == "intra_row_gain" && index1 >= 0 && index1 < 4)
                {
                    table.intra_row_gain[index1] = gain;
                    table.intra_row_gain_supported[index1] = true;
                }
                else if (fields[0] == "intra_col_offset" && !fields[4].empty() && index1 >= 0 && index1 < 8)
                {
                    table.intra_col_offset_m[index1] = std::stof(fields[4]) / kRange6532RawPerMetre;
                    table.intra_col_supported[index1] = true;
                }
                else if (fields[0] == "intra_col_gain" && index1 >= 0 && index1 < 8)
                {
                    table.intra_col_gain[index1] = gain;
                    table.intra_col_gain_supported[index1] = true;
                }
                else if (fields[0] == "inter_block_offset" && !fields[4].empty() &&
                         index0 >= 0 && index0 < 6 && index1 >= 0 && index1 < 4)
                {
                    table.inter_block_offset_m[index0][index1] = std::stof(fields[4]) / kRange6532RawPerMetre;
                    table.inter_block_supported[index0][index1] = true;
                }
                else if (fields[0] == "inter_block_gain" &&
                         index0 >= 0 && index0 < 6 && index1 >= 0 && index1 < 4)
                {
                    table.inter_block_gain[index0][index1] = gain;
                    table.inter_block_gain_supported[index0][index1] = true;
                }
                else
                {
                    continue;
                }
                has_supported_item = true;
            }
            catch (const std::exception &)
            {
                std::cerr << "[6532标定] 忽略格式错误行: " << line << std::endl;
            }
        }

        return has_supported_item;
    }

    void applyRange6532Calibration(PointMatrix &mat,
                                   const Range6532CalibrationTable &table,
                                   const Range6532CalibrationApplyOptions &options)
    {
        for (auto &row : mat)
        {
            for (auto &point : row)
            {
                if (!point.is_valid)
                    continue;

                const int u = point.pixel_idx % 4;
                const int v = point.slot_idx % 8;
                float offset_m = 0.0f;
                float gain = 0.0f;
                if (table.intra_row_supported[u])
                {
                    offset_m += table.intra_row_offset_m[u];
                }
                if (table.intra_row_gain_supported[u])
                {
                    gain += table.intra_row_gain[u];
                }
                if (table.intra_col_supported[v])
                {
                    offset_m += table.intra_col_offset_m[v];
                }
                if (table.intra_col_gain_supported[v])
                {
                    gain += table.intra_col_gain[v];
                }

                if (options.apply_inter_block_offset)
                {
                    const int macro_block = point.slot_idx / 32;
                    const int firing_phase = (point.slot_idx / 8) % 4;
                    if (macro_block < 6)
                    {
                        if (table.inter_block_supported[macro_block][firing_phase])
                            offset_m += table.inter_block_offset_m[macro_block][firing_phase];
                        if (table.inter_block_gain_supported[macro_block][firing_phase])
                            gain += table.inter_block_gain[macro_block][firing_phase];
                    }
                }
                if (options.apply_global_offset && table.global_supported)
                {
                    offset_m += table.global_offset_m;
                }
                if (options.apply_global_offset && table.global_gain_supported)
                {
                    gain += table.global_gain;
                }

                // 标定模型为 q_measured = (1 + gain) * q_true + offset。
                point.r -= offset_m;
                if (options.apply_gain && std::abs(1.0f + gain) > 1e-6f)
                    point.r /= (1.0f + gain);
                updateRangePoint(point);
            }
        }
    }

    void to_json(nlohmann::json &j, const PointData &p)
    {
        j = nlohmann::json{
            {"pixel_idx", p.pixel_idx},
            {"slot_idx", p.slot_idx},
            {"peak", p.peak},
            {"r", p.r},
            {"intensity", p.intensity},
            {"x", p.x},
            {"y", p.y},
            {"z", p.z},
            {"is_valid", p.is_valid}};
    }

    void from_json(const nlohmann::json &j, PointData &p)
    {
        // 使用 .at() 确保字段存在，如果不存在会抛出异常
        j.at("pixel_idx").get_to(p.pixel_idx);
        j.at("slot_idx").get_to(p.slot_idx);
        // 对于非必须或可能缺失的字段，可以使用 .value() 提供默认值
        p.peak = j.value("peak", 0); // 如果 JSON 中没有 "peak" 字段，则默认为 0
        j.at("r").get_to(p.r);
        j.at("intensity").get_to(p.intensity);
        j.at("x").get_to(p.x);
        j.at("y").get_to(p.y);
        j.at("z").get_to(p.z);
        j.at("is_valid").get_to(p.is_valid);

        // 对于未从JSON反序列化的 PointData 成员，需要手动初始化其默认值
        // 以确保 PointData 对象的完整性
        p.theta_rad = 0.0f;
        p.omega_rad = 0.0f;
        p.theta = 0.0f;
        p.slot_ang = 0.0f;
        p.reflect = 0.0f;
        p.list_pulse.clear(); // 清空任何默认构造的 Pulse 列表
        p.select_idx = 0;
    }

    // --- 新增的 JSON 读写函数 ---

    /// @brief 将 PointMatrix 保存为 JSON 文件。
    /// @param mat 要保存的 PointMatrix。
    /// @param filename 输出 JSON 文件的路径。
    void save_mat_to_json(const PointMatrix &mat, const std::string &filename)
    {
        nlohmann::json j_output = nlohmann::json::array(); // JSON 根是一个数组

        // 遍历 PointMatrix 的每一行
        for (const auto &row_vec : mat)
        {
            nlohmann::json row_json = nlohmann::json::array(); // 每一行也是一个数组
            // 遍历行中的每个 PointData 对象
            for (const auto &point : row_vec)
            {
                row_json.push_back(point); // 这里会自动调用 to_json(json&, PointData&)
            }
            j_output.push_back(row_json);
        }

        std::ofstream file(filename);
        if (file.is_open())
        {
            file << j_output.dump(4); // 以4个空格缩进格式化输出
            file.close();
            // 替换为你的日志宏
            // FS_LOGI("成功将 PointMatrix 保存到 %s\n", filename.c_str());
            std::cout << "成功将 PointMatrix 保存到 " << filename << std::endl;
        }
        else
        {
            // 替换为你的日志宏
            // FS_LOGE("错误：无法打开文件 %s 进行写入。\n", filename.c_str());
            std::cerr << "错误：无法打开文件 " << filename << " 进行写入。" << std::endl;
        }
    }

    /// @brief 从 JSON 文件中加载 PointMatrix。
    /// @param filename 输入 JSON 文件的路径。
    /// @return 加载的 PointMatrix。如果加载失败，则返回一个空的 PointMatrix。
    PointMatrix load_mat_by_json(const std::string &filename)
    {
        PointMatrix loaded_mat;
        std::ifstream file(filename);
        if (!file.is_open())
        {
            // 替换为你的日志宏
            // FS_LOGE("错误：无法打开文件 %s 进行读取。\n", filename.c_str());
            std::cerr << "错误：无法打开文件 " << filename << " 进行读取。" << std::endl;
            return loaded_mat; // 返回空矩阵
        }

        try
        {
            nlohmann::json j_input;
            file >> j_input; // 从文件读取 JSON 数据

            if (!j_input.is_array())
            {
                // 替换为你的日志宏
                // FS_LOGE("错误：JSON 根不是一个数组。\n");
                std::cerr << "错误：JSON 根不是一个数组。" << std::endl;
                return loaded_mat;
            }

            loaded_mat.reserve(j_input.size()); // 预分配行数以提高效率
            for (const auto &row_json : j_input)
            {
                if (!row_json.is_array())
                {
                    // 替换为你的日志宏
                    // FS_LOGE("错误：JSON 中的行不是一个数组。\n");
                    std::cerr << "错误：JSON 中的行不是一个数组。" << std::endl;
                    loaded_mat.clear(); // 清除任何部分加载的数据
                    return loaded_mat;
                }
                std::vector<PointData> row_vec;
                row_vec.reserve(row_json.size()); // 预分配列数
                for (const auto &point_json : row_json)
                {
                    // 这里会自动调用 from_json(const json&, PointData&)
                    row_vec.push_back(point_json.get<PointData>());
                }
                loaded_mat.push_back(row_vec);
            }
            // 替换为你的日志宏
            // FS_LOGI("成功从 %s 加载 PointMatrix。\n", filename.c_str());
            std::cout << "成功从 " << filename << " 加载 PointMatrix。" << std::endl;
        }
        catch (const nlohmann::json::exception &e)
        {
            // 捕获 JSON 解析过程中可能发生的异常
            // 替换为你的日志宏
            // FS_LOGE("解析 JSON 文件 %s 时发生错误：%s\n", filename.c_str(), e.what());
            std::cerr << "解析 JSON 文件 " << filename << " 时发生错误：" << e.what() << std::endl;
            loaded_mat.clear(); // 清除任何部分加载的数据
        }

        return loaded_mat;
    }

    // 矩阵转置
    PointMatrix transMat(const PointMatrix &mat_src)
    {
        if (mat_src.empty())
        {
            return {}; // 如果矩阵为空，直接返回空矩阵
        }

        size_t rows = mat_src.size();    // 原矩阵的行数
        size_t cols = mat_src[0].size(); // 原矩阵的列数

        // 创建转置后的矩阵，大小为 cols x rows
        PointMatrix mat_transposed(cols, std::vector<PointData>(rows));

        // 进行转置操作
        for (size_t i = 0; i < rows; ++i)
        {
            for (size_t j = 0; j < cols; ++j)
            {
                mat_transposed[j][i] = mat_src[i][j];
            }
        }

        return mat_transposed;
    }
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
                    image[i * width + width - 1 - j] = static_cast<uint8_t>(normalized_peak);
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
            uint8_t color[4] = {(uint8_t)i, (uint8_t)i, (uint8_t)i, 0};
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
    
    /**
     * @brief 矩形ROI截取PointMatrix
     * @param src 原始矩阵
     * @param r0 起始行
     * @param r1 结束行(包含)
     * @param c0 起始列
     * @param c1 结束列(包含)
     * @return roi子矩阵
     */
    PointMatrix extractData(const PointMatrix &src, int r0, int r1, int c0, int c1)
    {
        std::vector<int> rows, cols;
        for (int r = r0; r <= r1; r++)
            rows.push_back(r);
        for (int c = c0; c <= c1; c++)
            cols.push_back(c);
        return extractData(src, rows, cols);
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
    // 对目标矩阵按行重写 r_offset 并重新计算点数据
    void resetROffset(PointMatrix &mat, float r_offset)
    {
        float offset = r_offset;
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
                point.slot_ang += sa_offset; // 应用全局偏移
                if (!point.is_valid)
                    continue; // 跳过无效点
                // point.slot_ang += sa_offset; // 应用全局偏移
                // 重新计算 x, y, z
                float theta_rad = point.theta * M_PI / 180.0;
                float slot_ang_rad = point.slot_ang * M_PI / 180.0;
                point.x = point.r * std::cos(theta_rad) * std::cos(slot_ang_rad);
                point.y = point.r * std::cos(theta_rad) * std::sin(slot_ang_rad);
                point.z = point.r * std::sin(theta_rad);
            }
        }
    }
    // 根据 k_delta_omega 重新计算 slot_ang
    void resetOmegaByK(PointMatrix &mat, float k_delta_omega)
    {
        // 确保矩阵不为空
        if (mat.empty() || mat[0].empty())
            return;

        for (size_t pixel_idx = 0; pixel_idx < mat.size(); ++pixel_idx)
        {
            auto &cur_row = mat[pixel_idx]; // 当前行

            // 从第 1 个点开始（跳过第 0 个点）
            for (size_t slot_idx = 1; slot_idx < cur_row.size(); ++slot_idx)
            {
                // 计算当前点的 ang_delta
                float ang_delta = cur_row[slot_idx].slot_ang - cur_row[slot_idx - 1].slot_ang;

                // 更新当前点的 slot_ang
                cur_row[slot_idx].slot_ang = cur_row[slot_idx - 1].slot_ang + ang_delta * k_delta_omega;
                float slot_ang_rad = cur_row[slot_idx].slot_ang * M_PI / 180.0f;
                cur_row[slot_idx].omega_rad = slot_ang_rad;
                if (!cur_row[slot_idx].is_valid)
                    continue; // 无效点不计算xyz

                // 重新计算 x, y, z
                cur_row[slot_idx].x = cur_row[slot_idx].r * std::cos(cur_row[slot_idx].theta_rad) * std::cos(slot_ang_rad);
                cur_row[slot_idx].y = cur_row[slot_idx].r * std::cos(cur_row[slot_idx].theta_rad) * std::sin(slot_ang_rad);
                cur_row[slot_idx].z = cur_row[slot_idx].r * std::sin(cur_row[slot_idx].theta_rad);
            }
        }
    }
    // 根据 list_k 重新计算 slot_ang
    void resetOmegaByListK(PointMatrix &mat, const std::vector<float> &list_k, int start_slot)
    {
        // 确保矩阵不为空
        if (mat.empty() || mat[0].empty())
            return;

        // 检查 list_k 的大小是否与 slot_idx 的数量匹配
        size_t num_slots = mat[0].size(); // 假设所有行的 slot 数量相同
        if (list_k.size() != num_slots)   // list_k 的数量应与 slot_idx 的数量相同
        {
            std::cerr << "Error: list_k size (" << list_k.size() << ") does not match number of slots (" << num_slots << ")" << std::endl;
            return;
        }

        // 检查 start_slot 是否在有效范围内
        if (start_slot < 0 || start_slot >= num_slots)
        {
            std::cerr << "Error: start_slot (" << start_slot << ") is out of range [0, " << num_slots - 1 << "]" << std::endl;
            return;
        }

        for (size_t pixel_idx = 0; pixel_idx < mat.size(); ++pixel_idx)
        {
            auto &cur_row = mat[pixel_idx]; // 当前行

            // 从 start_slot 向右遍历（从 start_slot + 1 到 num_slots - 1）
            for (int slot_idx = start_slot + 1; slot_idx < num_slots; ++slot_idx)
            {
                // 计算当前点的 ang_delta
                float ang_delta = cur_row[slot_idx].slot_ang - cur_row[slot_idx - 1].slot_ang;

                // 使用 list_k[slot_idx] 更新当前点的 slot_ang
                cur_row[slot_idx].slot_ang = cur_row[slot_idx - 1].slot_ang + ang_delta * list_k[slot_idx];

                // 将 slot_ang 转换为弧度
                float slot_ang_rad = cur_row[slot_idx].slot_ang * M_PI / 180.0f;
                cur_row[slot_idx].omega_rad = slot_ang_rad;

                if (!cur_row[slot_idx].is_valid)
                    continue; // 无效点不计算 xyz

                // 重新计算 x, y, z
                cur_row[slot_idx].x = cur_row[slot_idx].r * std::cos(cur_row[slot_idx].theta_rad) * std::cos(slot_ang_rad);
                cur_row[slot_idx].y = cur_row[slot_idx].r * std::cos(cur_row[slot_idx].theta_rad) * std::sin(slot_ang_rad);
                cur_row[slot_idx].z = cur_row[slot_idx].r * std::sin(cur_row[slot_idx].theta_rad);
            }

            // 从 start_slot 向左遍历（从 start_slot - 1 到 0）
            for (int slot_idx = start_slot - 1; slot_idx >= 0; --slot_idx)
            {
                // 计算当前点的 ang_delta
                float ang_delta = cur_row[slot_idx + 1].slot_ang - cur_row[slot_idx].slot_ang;

                // 使用 list_k[slot_idx] 更新当前点的 slot_ang
                cur_row[slot_idx].slot_ang = cur_row[slot_idx + 1].slot_ang - ang_delta * list_k[slot_idx];

                // 将 slot_ang 转换为弧度
                float slot_ang_rad = cur_row[slot_idx].slot_ang * M_PI / 180.0f;
                cur_row[slot_idx].omega_rad = slot_ang_rad;

                if (!cur_row[slot_idx].is_valid)
                    continue; // 无效点不计算 xyz

                // 重新计算 x, y, z
                cur_row[slot_idx].x = cur_row[slot_idx].r * std::cos(cur_row[slot_idx].theta_rad) * std::cos(slot_ang_rad);
                cur_row[slot_idx].y = cur_row[slot_idx].r * std::cos(cur_row[slot_idx].theta_rad) * std::sin(slot_ang_rad);
                cur_row[slot_idx].z = cur_row[slot_idx].r * std::sin(cur_row[slot_idx].theta_rad);
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
    // 应用 sa_offset 到 PointMatrix，先转置矩阵再应用偏移
    PointMatrix applySaOffsetPerSlot(const PointMatrix &mat, const std::vector<float> &sa_offset)
    {
        // 转置矩阵
        PointMatrix mat_trans = transMat(mat);

        // 检查 sa_offset 的大小是否与转置矩阵的行数匹配
        if (mat_trans.size() != sa_offset.size())
        {
            throw std::invalid_argument("Error: The size of sa_offset (" + std::to_string(sa_offset.size()) +
                                        ") does not match the number of rows in the transposed matrix (" + std::to_string(mat_trans.size()) + ").");
        }

        // 按行偏移
        for (size_t i = 0; i < mat_trans.size(); ++i)
        {
            float offset = sa_offset[i];
            for (auto &point : mat_trans[i])
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

        // 再次转置矩阵以恢复原始布局
        auto mat_result = transMat(mat_trans);

        return mat_result;
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
    std::vector<PolarPoint> Converter::mat2Polar(const PointMatrix &matrix)
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
    PolarPoint Converter::buildPolarPt(const PointData &point)
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
    PointMatrix buildMatByPulseIdx(const PointMatrix &mat, int pulse_idx)
    {
        const double LIGHT_SPEED = 299792458e-9; // 单位：m/ns

        // 深拷贝源矩阵，保留所有不变字段（角度、索引等）
        PointMatrix result = mat;

        for (auto &row : result)
        {
            for (auto &pt : row)
            {
                // ── 1. 原始点本身无效，跳过（保持 is_valid = false） ──────
                if (!pt.is_valid)
                    continue;

                // ── 2. 检查 list_pulse 是否包含目标索引 ──────────────────
                if (pulse_idx < 0 ||
                    pulse_idx >= static_cast<int>(pt.list_pulse.size()))
                {
                    pt.is_valid = false;
                    continue;
                }

                const Pulse &pulse = pt.list_pulse[pulse_idx];

                // ── 3. 过滤无效回波（与 readPulseData 保持一致） ─────────
                if (pulse.tof == 0.016f ||
                    pulse.width == 0.016f ||
                    pulse.peak <= 1.0f)
                {
                    pt.is_valid = false;
                    continue;
                }

                // ── 4. 重新计算物理量 ─────────────────────────────────────
                pt.select_idx = static_cast<uint8_t>(pulse_idx);
                pt.peak = static_cast<uint16_t>(std::round(pulse.peak));

                // 距离：tof(ns) * 光速(m/ns) / 2
                pt.r = static_cast<float>(pulse.tof * LIGHT_SPEED / 2.0);

                // 强度与反射率（与 readPulseData 公式一致）
                float intensity = static_cast<float>(
                    std::min(255.0, std::round(
                                        static_cast<double>(pulse.peak) *
                                        static_cast<double>(pulse.width) / 702.0 / 5.0 * 256.0)));
                pt.intensity = static_cast<uint8_t>(intensity);
                pt.reflect = intensity / 255.0f;

                // 笛卡尔坐标（theta_rad / omega_rad 不变，仅 r 更新）
                pt.x = pt.r * std::cos(pt.theta_rad) * std::cos(pt.omega_rad);
                pt.y = pt.r * std::cos(pt.theta_rad) * std::sin(pt.omega_rad);
                pt.z = pt.r * std::sin(pt.theta_rad);
            }
        }

        return result;
    }


    void resetROffset(PointMatrix &mat, const std::vector<std::vector<float>> &r_offset)
    {
        const int num_rows = static_cast<int>(mat.size());
        // ========== 硬编码标定系数（可自行修改）==========
        // Step2 全局线性修正 K、B
        const float linear_k = 0.80494535f;
        const float linear_b = 0.30557389f;

        // Step3 Time-Walk 线性修正 降幂 c1*peak + c0
        const float tw_c1 = -4.97306069e-05f;
        const float tw_c0 = 7.59726215e-02f;

        // 峰值饱和阈值，peak >= 该值跳过TimeWalk修正
        const float peak_sat_thresh = 4500.0f;

        // 行数校验
        if (static_cast<int>(r_offset.size()) != num_rows)
        {
            std::cerr << "[resetROffset] 警告：r_offset 行数（"
                      << r_offset.size()
                      << "）与 mat 行数（" << num_rows
                      << "）不匹配，操作取消。" << std::endl;
            return;
        }

        float offset_last_valid = 0.0f; // 最近一个有效偏移值，用于填补 0.0f 空洞

        for (int row = 0; row < num_rows; ++row)
        {
            const int num_cols = static_cast<int>(mat[row].size());

            // 列数校验（逐行）
            if (static_cast<int>(r_offset[row].size()) != num_cols)
            {
                std::cerr << "[resetROffset] 警告：第 " << row
                          << " 行 r_offset 列数（"
                          << r_offset[row].size()
                          << "）与 mat 列数（" << num_cols
                          << "）不匹配，跳过该行。" << std::endl;
                continue;
            }

            for (int col = 0; col < num_cols; ++col)
            {
                PointData &pt = mat[row][col];
                float offset = r_offset[row][col];

                if (!pt.is_valid)
                    continue; // 跳过无效点

                // 偏移为 0.0f 时（未标定像素），使用最近有效偏移值代替
                if (offset == 0.0f)
                    offset = offset_last_valid;
                else
                    offset_last_valid = offset; // 更新最近有效偏移值

                pt.r += offset;
                // ---------------- Step2 全局线性度修正 k/b ----------------
                // if (std::abs(linear_k) > 1e-6f) // 防止除零
                // {
                //     pt.r = (pt.r - linear_b) / linear_k;
                // }

                // // ---------------- Step3 Peak Time-Walk 线性修正 ----------------
                // if (pt.peak < peak_sat_thresh)
                // {
                //     // 线性修正：Δtw = c1*peak + c0
                //     float peak = pt.peak;
                //     float delta_tw = tw_c1 * peak + tw_c0;
                //     pt.r -= delta_tw;
                // }

                // 重新计算笛卡尔坐标（使用 theta_rad / omega_rad，与 readPulseData 一致）
                pt.x = pt.r * std::cos(pt.theta_rad) * std::cos(pt.omega_rad);
                pt.y = pt.r * std::cos(pt.theta_rad) * std::sin(pt.omega_rad);
                pt.z = pt.r * std::sin(pt.theta_rad);
            }
        }
    }
    
    PointMatrix applyROffset(const PointMatrix &mat,
                             const std::vector<std::vector<float>> &list_r_offset)
    {
        PointMatrix result = mat;            // 深拷贝，不修改原矩阵
        resetROffset(result, list_r_offset); // 复用 reset 逻辑，避免代码重复
        return result;
    }
}
