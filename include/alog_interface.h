/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-02 14:04:29
 * @LastEditors  : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-01-03 11:03:11
 * @FilePath     : /calib_r_offset/include/alog_interface.h
 * @Description  : 距离-水平角耦合标定算法接口
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */
#pragma once
#include <vector>
#include <string>
#include <point_type.h>
namespace alog_lidar
{
    // 距离标定及水平角标定算法 接口类
    class CalibRsa_I
    {
    public:
        // 距离标定 - 接口
        void doCalibR();
        // 水平角标定 - 接口
        void doCalibSa();
        // 距离-水平角 耦合标定 - 接口
        void doCalibRSa();

        // 对输入的极坐标点云应用新的offset参数，输出新的xyz点云，用于实时显示应用offset参数后的点云
        static std::vector<CartesianPoint> applyParam(const std::vector<PolarPoint> &pts_src, const std::vector<float> &r_offset, const std::vector<float> &sa_offset);

        // 输入 参数 输出
        struct Data
        {
            // 输入
            std::vector<PolarPoint> pts_src; // 单帧原始点云（极坐标） 适配前端格式
            std::vector<float> list_r_old;   // 96个距离offset 旧值 从lidar配置文件读到的
            std::vector<float> list_sa_old;  // 96个水平角offset 旧值 从lidar配置文件读到的
            // 参数
            float work_dist = 3.343f;      // VCSEL到靶面的工作距离 单位：米
            uint16_t slot_idx_start = 200; // ROI区域的列序号起点
            uint16_t slot_idx_end = 600;   // ROI区域的列序号终点
            // 高级参数，一般都用默认值
            uint16_t pixel_idx_start = 0; // ROI区域的行序号起点
            uint16_t pixel_idx_end = 95;  // ROI区域的行序号终点
            uint16_t max_iterations = 10; // 最大迭代次数
            uint16_t pixel_num = 96;      // src点云的行数
            uint16_t slot_num = 800;      // src点云的列数
            // 输出
            bool status = true;               // 计算是否成功
            std::string msg;                  // 报错信息
            PointMatrix mat_result;           // 结果点云矩阵
            std::vector<float> list_r_delta;  // 96个距离offset的变化（新值相对旧值的变化量） debug显示用
            std::vector<float> list_sa_delta; // 96个水平角offset的变化（新值相对旧值的变化量）
            std::vector<float> list_r_new;    // 96个距离offset 新值  写入用
            std::vector<float> list_sa_new;   // 96个水平角offset 新值
            // 算法私有字段
            PointMatrix mat_src; // 单帧原始点云转换为矩阵
            PointMatrix mat_roi; // 单帧原始点云转化为矩阵 ROI部分
        };
        Data _data;

        // 填充idx列表 步长为1
        static void fillRange(std::vector<int> &vec, int start, int end);
        // 填充range列表 步长为step
        static void fillRangeWithStep(std::vector<float> &vec, float start, float end, float step);

    private:
        // 算法测试
        void doTest();
        void computeMatROI();
    };

    // 转镜零位角 接口类
    class CalibMirror_I
    {
    public:
		// 转镜零位角标定 - 接口
		void doCalibMirror();
		// 传入当前时刻的4个面的点云, 以及4个面的水平角偏移, 以及用户选择的面的idx (前端会一直调用此函数)
		void sendCloud4(std::vector<std::vector<PolarPoint>> list_pts,
			            std::vector<float> list_sa,
			            std::vector<uint8_t> select_index);
		struct Data
		{
			// 输入
			std::vector<std::vector<PolarPoint>> list_pts_src; // 4个转镜面的原始点云列表  极坐标
            // 参数
            float tolerance = 0.05; // 距离阈值
            // 高级参数，一般都用默认值
            uint16_t pixel_idx_start = 0; // ROI区域的行序号起点
            uint16_t pixel_idx_end = 95;  // ROI区域的行序号终点
            uint16_t pixel_num = 96;      // src点云的行数
            uint16_t slot_num = 800;      // src点云的列数
            // 输出
            bool status = true;                                // 计算是否成功
            std::string msg;                                   // 报错信息
            std::vector<float> list_mirror_offset;             // 4个转镜点云所需要修正的水平角偏移量
            std::vector<std::vector<CartesianPoint>> list_pts_result; // 4个转镜面的修正后的点云列表 笛卡尔坐标
            // 算法私有字段
            std::vector<PointMatrix> list_mat_src; //
            std::vector<PointMatrix> list_mat_roi; // 
        };
        Data _data;
    };
    // 空间畸变角 接口类
    class CalibSpace_I
    {
    public:
        // 空间畸变角 - 接口
        void doCalibSpace();
        struct Data
        {
            // 输入
            std::vector<PolarPoint> pts_src; // 原始点云  极坐标
            std::vector<float> list_real_h;  // 实测 多个十字交叉点的高程 从低到高排序
            // 参数
            float work_dist = 3.343f;      // VCSEL到墙面的工作距离 单位：米
            uint16_t slot_idx_start = 470; // ROI区域的列序号起点
            uint16_t slot_idx_end = 500;   // ROI区域的列序号终点
            // 高级参数，一般都用默认值
            float u = 47;                  // 中心像素序号
            float theta_tilt = -2.5f;      // 机械安装误差 俯仰角（度） 固定参数
            uint16_t pixel_idx_start = 0;  // ROI区域的行序号起点
            uint16_t pixel_idx_end = 95;  // ROI区域的行序号终点
            uint16_t pixel_num = 96;      // src点云的行数
            uint16_t slot_num = 800;      // src点云的列数
            // 输出
            bool status = true;           // 计算是否成功
            std::string msg;              // 报错信息
            float theta_range;            // 标定后的 俯仰角范围 固定参数
            // 算法私有字段
            PointMatrix mat_src; //
            PointMatrix mat_roi; // 
        };
        Data _data;
    };
}
