/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-09 14:32:48
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-08-15 09:00:40
 * @FilePath     : /calib_reflectivity/include/calc_intensity.h
 * @Description  : 计算反射率
 * 
 * Copyright (c) 2024 by vitalchem, All Rights Reserved. 
 */
#ifndef CALC_INTENSITY_H
#define CALC_INTENSITY_H

#include <map>
#include <array>
#include <string>
#include <vector>

#include "point_type.h"

using Pt = algo_lidar::PointData;
using PointMatrix = std::vector<std::vector<Pt>>;

// 定义 CalibReflect 类
class CalibReflect
{
public:
    // 构造函数与析构函数
    CalibReflect() = default;
    ~CalibReflect() = default;
    // 接口函数
    bool calcIntensity(Pt& pt); // 计算单点强度值
    bool calcCloudIntensity(PointMatrix& mat); // 对点云的每个点计算强度值

    // 测了多少个反射率
    static const size_t REFLECTANCE_SIZE = 5;
    // 测了多少个距离
    static const size_t DISTANCE_SIZE = 397;
    // 测了多少行 
    static const size_t PIXEL_SIZE_TABLE = 10;

    // 反射率标定表的存储结构
    std::map<uint16_t, std::map<float, std::array<float, REFLECTANCE_SIZE>>> _map;


    // 拼缝通道
    std::array<uint16_t, 6> _pixel_overlap = { 24,25,48,49,72,73 };

    // 测了哪几个反射率
    std::array<float, REFLECTANCE_SIZE> _reflectances = {0.1,0.3,0.5,0.7,0.9};

private:
    // 查表的结果 峰值数组
    std::array<float, REFLECTANCE_SIZE> _peaks;

    // 定长数组 78 KB
    // std::array<std::array<std::array<float, REFLECTANCE_SIZE>, DISTANCE_SIZE>, PIXEL_SIZE_TABLE> _table;

    bool queryMap(Pt &pt); // 查表
};
#endif // CALC_INTENSITY_H
