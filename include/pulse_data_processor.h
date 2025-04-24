/*
 * @Author       : baizs zhongshan.bai@vitalchem.com
 * @Date         : 2024-11-21 15:08:45
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-03-25 14:06:17
 * @FilePath     : /calib_r_offset/include/pulse_data_processor.h
 * @Description  : 读本地的脉冲回波数据
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */

#pragma once
#include <vector>
#include <string>
#include <point_type.h>
namespace algo_lidar {

	// 定义矩阵类型：96行 * 800列
	using PointMatrix = std::vector<std::vector<PointData>>;

	// 读取文件并填充矩阵
	void readPulseData(const std::string& filename, PointMatrix& mat);

	bool saveListToFile(const std::vector<float> &list, const std::string &filename);

	bool loadListFromFile(std::vector<float> &list, const std::string &filename);
	
}
