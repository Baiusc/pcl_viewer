/*
 * @Author       : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @Date         : 2025-02-17 10:07:58
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2026-09-09 13:25:33
 * @FilePath     : /pcl_viewer/include/point_type.h
 * @Description  : 
 * 
 * Copyright (c) 2025 by vitalchem, All Rights Reserved. 
 */

#pragma once
#include <array>
#include <vector>
#include <cmath>
#include <cstdint>
#include <string> 
namespace algo_lidar
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
        uint16_t peak;     // 峰值
    };
    // 回波数据类型
    struct Pulse
    {
        float tof;
        float width;
        float peak;
        // 默认构造函数
        Pulse() : tof(0.0f), width(0.0f), peak(0.0f) {}
        // 带参数的构造函数
        Pulse(float t, float w, float p) : tof(t), width(w), peak(p) {}
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
        uint32_t timestamp; // 时间戳
        uint8_t intensity; // 强度 0~255
        float x;
        float y;
        float z;
        bool is_valid;                 // 点的回波数据是否有效
        std::vector<Pulse> list_pulse; // 3组回波数据 按峰值从大到小排序后的
        uint8_t select_idx;            // 0 1 2 表示当前点的回波依据是 最强回波 次强回波 最弱回波
        bool is_tail_pt = false;       // 是否是托尾点
    };

    using PointMatrix = std::vector<std::vector<PointData>>;

    // 6532 标定程序输出的 range6532_distance_calibration.csv 对应的参数表，单位统一为 m。
    struct Range6532CalibrationTable
    {
        std::array<float, 4> intra_row_offset_m{};
        std::array<float, 8> intra_col_offset_m{};
        std::array<std::array<float, 4>, 6> inter_block_offset_m{};
        std::array<float, 4> intra_row_gain{};
        std::array<float, 8> intra_col_gain{};
        std::array<std::array<float, 4>, 6> inter_block_gain{};
        std::array<bool, 4> intra_row_supported{};
        std::array<bool, 8> intra_col_supported{};
        std::array<std::array<bool, 4>, 6> inter_block_supported{};
        std::array<bool, 4> intra_row_gain_supported{};
        std::array<bool, 8> intra_col_gain_supported{};
        std::array<std::array<bool, 4>, 6> inter_block_gain_supported{};
        float global_offset_m = 0.0f;
        float global_gain = 0.0f;
        bool global_supported = false;
        bool global_gain_supported = false;
    };

    struct Range6532CalibrationApplyOptions
    {
        bool apply_global_offset = false;      // 全局项在单距离拟合中可能不可辨识，默认不使用。
        bool apply_inter_block_offset = true; // 是否应用块间偏置。
        bool apply_gain = false; // 是否应用距离比例误差（斜率）。
    };

    /// @brief 读取 6532 标定程序输出的 range6532_distance_calibration.csv。
    bool loadRange6532CalibrationCsv(const std::string &filename,
                                      Range6532CalibrationTable &table);

    /// @brief 原地应用 6532 的块内/块间距离标定，并同步更新 r、xyz 和选中回波 tof。
    void applyRange6532Calibration(PointMatrix &mat,
                                   const Range6532CalibrationTable &table,
                                   const Range6532CalibrationApplyOptions &options = {});

    // 矩阵转置
    PointMatrix transMat(const PointMatrix& mat_src);

    // 将 PointMatrix 转换为 8 位单通道灰度图并保存为 BMP 格式
    void saveMatAsBMP(const PointMatrix &mat, const std::string &filename);

    // 将 PointMatrix 逐行提取到 std::vector<PointData>
    std::vector<PointData> extractPoints(const PointMatrix &mat);

    // 按指定行列截取矩阵数据
    PointMatrix extractData(const PointMatrix &mat, const std::vector<int> &row_indices, const std::vector<int> &col_indices);

    /**
     * @brief 矩形ROI截取PointMatrix
     * @param src 原始矩阵
     * @param r0 起始行
     * @param r1 结束行(包含)
     * @param c0 起始列
     * @param c1 结束列(包含)
     * @return roi子矩阵
     */
    PointMatrix extractData(const PointMatrix &src, int r0, int r1, int c0, int c1);

    // 对 mat 应用 r_offset
    void resetROffset(PointMatrix &mat, const std::vector<float> &r_offset);
    // 对 mat 应用 r_offset
    void resetROffset(PointMatrix &mat, float r_offset);

    // 应用偏移量，返回一个新矩阵
    PointMatrix applyROffset(const PointMatrix &mat, const std::vector<float> &r_offset);

    // 重载 applyROffset 方法，支持点向量输入
    std::vector<PointData> applyROffset(const std::vector<PointData> &pts, const std::vector<float> &r_offset);

    /// @brief 按二维偏差表逐点应用 r_offset，返回新矩阵，不修改原矩阵。适用于固态雷达的逐点校准。
    /// @param mat            源矩阵。
    /// @param list_r_offset  与 mat 同尺寸的二维偏差表，list_r_offset[row][col] 为该点的距离偏移量（m）。
    ///                       尺寸不匹配时跳过对应点并输出警告。无效点（is_valid=false）保持不变，偏移为 0.0f 的点使用最近一个有效偏移值代替。
    /// @return 应用偏移后的新矩阵。
    PointMatrix applyROffset(const PointMatrix &mat,
                             const std::vector<std::vector<float>> &list_r_offset);

    /// @brief 按二维偏差表逐点原地应用 r_offset，直接修改 mat。适用于固态雷达的距离校准。
    /// @param mat      目标矩阵（原地修改）。
    /// @param r_offset 与 mat 同尺寸的二维偏差表，r_offset[row][col] 为该点的距离偏移量（m）。
    ///                 尺寸不匹配时跳过对应点并输出警告。无效点（is_valid=false）保持不变，偏移为 0.0f 的点使用最近一个有效偏移值代替。
    void resetROffset(PointMatrix &mat, const std::vector<std::vector<float>> &r_offset);

    // 重载方法 1：sa_offset 是单一的浮动值
    void resetSaOffset(PointMatrix &mat, float sa_offset);
    // 根据 k_delta_omega 重新计算 slot_ang
    void resetOmegaByK(PointMatrix &mat, float k_delta_omega);
    // 根据 list_k 重新计算 slot_ang
    void resetOmegaByListK(PointMatrix &mat, const std::vector<float> &list_k, int start_slot);

    // 重载方法 2：sa_offset 是单一的浮动值，mat 改为 std::vector<PointData> pts
    void resetSaOffset(std::vector<PointData>& pts, float sa_offset);

    // 对目标矩阵按行重写 sa_offset，并重新计算点数据
    void resetSaOffset(PointMatrix &mat, const std::vector<float> &sa_offset);

    // 重载方法 ：sa_offset 是单一的浮动值
    PointMatrix applySaOffset(PointMatrix &mat, float sa_offset);

    // 应用偏移量，返回一个新矩阵
    PointMatrix applySaOffset(const PointMatrix &mat, const std::vector<float> &sa_offset);
    // 应用 sa_offset 到 PointMatrix，先转置矩阵再应用偏移 800列
    PointMatrix applySaOffsetPerSlot(const PointMatrix &mat, const std::vector<float> &sa_offset);

    // 重载方法，输入向量点
    std::vector<PointData> applySaOffset(const std::vector<PointData> &pts, const std::vector<float> &sa_offset);

    // 对目标矩阵按行重写 r_offset 和 sa_offset，并重新计算点数据
    void resetRsaOffset(PointMatrix &mat, const std::vector<float> &r_offset, const std::vector<float> &sa_offset);

    // 应用偏移量，返回一个新矩阵
    PointMatrix applyRsaOffset(const PointMatrix &mat, const std::vector<float> &r_offset, const std::vector<float> &sa_offset);

    /// @brief 按指定回波索引重建 PointMatrix。
    /// @param mat       源矩阵（list_pulse 已按 peak 降序排列）。
    /// @param pulse_idx 目标回波索引：0=最强，1=次强，2=最弱。
    /// @return 新矩阵，每个有效点的物理量均由 list_pulse[pulse_idx] 重新计算得出。
    ///         若某点 list_pulse 长度不足或对应回波为无效值（tof==0.016/width==0.016），
    ///         则该点 is_valid 置为 false。
    PointMatrix buildMatByPulseIdx(const PointMatrix &mat, int pulse_idx);

    // 计算 PointMatrix 的质心
    PointData computeCentroid(const PointMatrix &mat);

    /// @brief 将 PointMatrix 保存为 JSON 文件。
    /// @param mat 要保存的 PointMatrix。
    /// @param filename 输出 JSON 文件的路径。
    void save_mat_to_json(const PointMatrix &mat, const std::string &filename);

    /// @brief 从 JSON 文件中加载 PointMatrix。
    /// @param filename 输入 JSON 文件的路径。
    /// @return 加载的 PointMatrix。如果加载失败，则返回一个空的 PointMatrix。
    PointMatrix load_mat_by_json(const std::string &filename);

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
