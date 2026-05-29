/*
 * @Author       : baizs zhongshan.bai@vitalchem.com
 * @Date         : 2024-11-21 15:08:45
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2026-05-29 16:15:05
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

	/// @brief 从 r_offset CSV 文件加载二维偏差表。使用于固态雷达的逐点距离校准。
	/// @param list_r_offset  输出的二维偏差表，list_r_offset[row][col] 对应像素偏移量（m）。
	/// @param filename       CSV 文件路径，格式为 saveROffsetToCsv 写出的带行列表头格式：
	///                       首行 "row\col,0,1,2,..."，首列为行号，其余为各列偏移值。
	/// @return true 成功；false 文件打开失败或格式错误。
	bool loadListFromFile(std::vector<std::vector<float>> &list_r_offset,
						  const std::string &filename);
}
