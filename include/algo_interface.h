/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2025-02-26 16:35:50
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-04-24 15:05:40
 * @FilePath     : /pcl_viewer/include/alog_interface.h
 * @Description  : 
 * 
 * Copyright (c) 2025 by vitalchem, All Rights Reserved. 
 */
#pragma once
#include <vector>
#include <string>
#include <point_type.h>
namespace algo_lidar
{
    // 修复地面算法 接口类
    class RepairGround_I
    {
    public:
        // 修复地面 - 接口
        void doRepairGround();
        // 算法测试
        void doTest();
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

    private:

    };
}
