/*
 * @Author       : baizs zhongshan.bai@vitalchem.com
 * @Date         : 2024-11-21 15:08:45
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2026-08-11 09:47:44
 * @FilePath     : /pcl_viewer/include/pulse_data_processor.h
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

	// 读取 转镜雷达的原始点云 
	void readPulseData(const std::string &filename, PointMatrix &mat);

	// 读取 固态雷达的原始点云 
	bool readSolidLidarData(const std::string &filename, PointMatrix &mat);

	// 读取 任意雷达的原始点云
	bool readLidarData(const std::string &filename, PointMatrix &mat);

	bool saveListToFile(const std::vector<float> &list, const std::string &filename);

	bool loadListFromFile(std::vector<float> &list, const std::string &filename);

	// 公开接口：loadListFromFile 二维重载
	bool loadListFromFile(std::vector<std::vector<float>> &list2,
						  const std::string &filename,
						  const std::string &type = ".csv");
	// 公开接口：saveListToFile 二维重载
	bool saveListToFile(const std::vector<std::vector<float>> &list2,
						const std::string &filename,
						const std::string &type = ".csv");
}
