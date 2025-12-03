/*
 * @Author       : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @Date         : 2025-12-03 11:12:55
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-12-03 11:12:56
 * @FilePath     : /pcl_viewer/include/tail_point_filter.h
 * @Description  : 
 * 
 * Copyright (c) 2025 by vitalchem, All Rights Reserved. 
 */

#ifndef TAIL_POINT_FILTER_H
#define TAIL_POINT_FILTER_H

#include <string>
#include <vector>
#include "point_type.h"
namespace algo_lidar {

class TailPointFilter {
public:
    TailPointFilter(/* args */);
    ~TailPointFilter() = default;

public:
    bool Init(std::string yaml_path);

    void Process(std::vector<PointData>& row_data);

private:
    /* data */
};

}  // namespace lidar_algo

#endif