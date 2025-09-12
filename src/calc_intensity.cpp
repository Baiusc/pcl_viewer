/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-09 14:43:01
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-09-12 15:15:28
 * @FilePath     : /pcl_viewer/src/calc_intensity.cpp
 * @Description  :
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */
#include "calc_intensity.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <stdexcept>

#include <Spline.h>
#include <regex>
// #include <timer.h>

// Timer timer;

// 查找匹配的反射率区间，并更新 pt 中的相关字段 20241213 更新搭接段查表策略
bool CalibReflect::queryMap(Pt& pt) {
    // 查与pixel_idx对应的表
    auto idx_it = _map.lower_bound(pt.pixel_idx);  // 查找第一个大于或等于 pt.pixel_idx 的元素
    if (idx_it == _map.end()) {
        // 如果找不到，说明 pt.pixel_idx 大于所有现有的 pixel_idx，返回最后一项
        idx_it = std::prev(_map.end());
    }
    if (pt.pixel_idx == 77) {
        
        int k = 1;
    }
    // 当从表中找不到直接的通道（即 pt.pixel_idx 为不在表中的正常通道）时，且 idx_it 为拼缝通道时 ，idx_it 前移
    while (idx_it->first > 0 && pt.pixel_idx != idx_it->first && std::find(std::begin(_pixel_overlap), std::end(_pixel_overlap), idx_it->first) != std::end(_pixel_overlap)) {
        --idx_it;
    }

    const auto& distance_map = idx_it->second;

    // 查找与 pt.r 对应的表
    auto dist_it = distance_map.lower_bound(pt.r);  // 查找大于或等于 pt.r 的元素
    if (dist_it == distance_map.end()) {
        // 如果找不到，说明 pt.r 大于所有的距离值，返回最后一项
        dist_it = std::prev(distance_map.end());
    }

    // 判断与 pt.r 最接近的元素
    auto dist_nn = dist_it;
    if (dist_it != distance_map.begin()) {
        auto prev_dist_it = std::prev(dist_it);
        if (std::abs(pt.r - prev_dist_it->first) < std::abs(pt.r - dist_it->first)) {
            dist_nn = prev_dist_it;
        }
    }

    // 更新 _peaks
    _peaks = dist_nn->second;

    return true;
}


// 三次样条插值，计算强度值，更新 pt 中的强度字段
bool CalibReflect::calcIntensity(Pt &pt){
    //Timer timer;
    // 查表
    queryMap(pt);
    //timer.reset();
    //timer.out("\n查表:");
    // 用 _peaks 和 _reflectances 构成的 5 组点，进行二次多项式拟合，得到方程。
    // 线性插值
    float reflect= linearInterp(_peaks.data(), _reflectances.data(), REFLECTANCE_SIZE, pt.peak);

    //timer.out("\n线性插值_vevtor:");
    // 二次样条插值
    // SplineQ splineQ =splineQuadratic(_peaks.data(), _reflectances.data(), REFLECTANCE_SIZE);
    // float reflect = splineEvalQ(splineQ, pt.peak);
    //timer.out("\n二次样条:");
    // 三次样条插值
    //SplineC splineC = splineCubic(_peaks.data(), _reflectances.data(), REFLECTANCE_SIZE);
    //float reflect = splineEvalC(splineC, pt.peak);
    //timer.out("\n三次样条:");
    
    // 写入
    pt.reflect = reflect;
    pt.intensity = uint8_t(255 * reflect);
    return true;
}

bool CalibReflect::calcCloudIntensity(PointMatrix &mat){
    //Timer timer_sum;
   for (size_t pixel_idx = 0; pixel_idx < mat.size(); ++pixel_idx)
   {
       auto &row = mat[pixel_idx];
       for (size_t slot_idx = 0; slot_idx < row.size(); ++slot_idx)
       {
           auto &point = row[slot_idx];
           if (!point.is_valid)
               continue;      // 跳过无效点
           //timer_sum.reset();
           calcIntensity(point);
           //timer_sum.out("\n单此循环耗时:");

       }
   }
   return true;
}
