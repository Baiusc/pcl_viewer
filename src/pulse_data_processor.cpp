/*
 * @Author       : baizs zhongshan.bai@vitalchem.com
 * @Date         : 2024-11-21 15:10:40
 * @LastEditors  : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-03-03 10:09:58
 * @FilePath     : /pcl_viewer/src/pulse_data_processor.cpp
 * @Description  : 读本地的脉冲回波数据
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */

#include "pulse_data_processor.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <iterator>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace alog_lidar {
	// 工具函数：分割字符串
	std::vector<std::string> splitString(const std::string &str, char delimiter)
	{
		std::vector<std::string> tokens;
		std::stringstream ss(str);
		std::string token;
		while (std::getline(ss, token, delimiter))
		{
			if (!token.empty())
			{
				tokens.push_back(token);
			}
		}
		return tokens;
	}

	// 读取文件并填充矩阵 全0为无效点
	void readPulseData(const std::string &filename, PointMatrix &mat)
	{
		const double LIGHT_SPEED = 299792458e-9; // 单位：m/ns
		const int PIXEL_NUM = 96;
		const int SLOT_NUM = 800;

		mat.resize(PIXEL_NUM, std::vector<PointData>(SLOT_NUM));

		std::ifstream infile(filename);
		if (!infile.is_open())
		{
			std::cerr << "Failed to open file: " << filename << std::endl;
			return;
		}

		std::string line;
		int slot_index = 0, last_slot_dics = 0, pixel_cnt = 0;
		float slot_ang = 0.0;
		bool bDataBegin = false;

		while (std::getline(infile, line))
		{
			if (line.find("start of slot") != std::string::npos)
			{
				auto f1 = line.find('[');
				auto f2 = line.find(']');
				slot_index = std::stoi(line.substr(f1 + 1, f2 - f1 - 1));
				bDataBegin = false;
				pixel_cnt = 0;
				continue;
			}

			if (line.find("angle") != std::string::npos)
			{
				auto tokens = splitString(line, ' ');
				int slot_dics = std::stod(tokens.back());
				if (slot_index == 0)
				{
					slot_ang = -60.0f;
				}
				else if (slot_index == 1)
				{
					slot_ang = -59.85f;
					last_slot_dics = slot_dics;
				}
				else
				{
					int delta_dics = (slot_dics >= last_slot_dics) ? slot_dics - last_slot_dics : slot_dics + 65536 - last_slot_dics;
					slot_ang += delta_dics * 360.0f / 65536.0f * 2.0f;
					last_slot_dics = slot_dics;
				}
				bDataBegin = true;
				continue;
			}

			if (bDataBegin)
			{
				pixel_cnt++;
				if (pixel_cnt > PIXEL_NUM || slot_index >= SLOT_NUM)
				{
					continue;
				}

				auto tokens = splitString(line, ' ');
				if (tokens.size() < 9)
					continue;

				std::vector<double> peaks = {std::stod(tokens[2 + 2]), std::stod(tokens[5 + 2]), std::stod(tokens[8 + 2])};
				std::vector<double> tofs = {std::stod(tokens[0 + 2]) / 1000, std::stod(tokens[3 + 2]) / 1000, std::stod(tokens[6 + 2]) / 1000};
				std::vector<double> widths = {std::stod(tokens[1 + 2]) / 1000, std::stod(tokens[4 + 2]) / 1000, std::stod(tokens[7 + 2]) / 1000};

				int select_index = -1;
				double max_peak = 1.0;

				for (int i = 0; i < 3; ++i)
				{
					if (tofs[i] == 0.016)
						continue; // 过滤无效数据
					if (peaks[i] > max_peak)
					{
						max_peak = peaks[i];
						select_index = i;
					}
				}

				if (select_index == -1)
					continue;

				int pixel_roi_start = 0 ; // 若原始点云是裁剪后的roi点云 则加上起始pixel号 默认为0
				int pixel_idx = pixel_cnt - 1 + pixel_roi_start ;
				int u = 47; // u: SPAD中心在y竖直方向上的偏移量（像素单位），默认 47

				float r = tofs[select_index] * LIGHT_SPEED / 2.0;
				float intensity = std::min(255.0, std::round(peaks[select_index] * widths[select_index] / 702.0 / 5 * 256));
				float reflect = intensity / 255.0;
				uint8_t intensity_uint8 = static_cast<uint8_t>(intensity);
				// float theta = -pixel_idx / 96.0 * 20 + 10 - 2.5; // 20241213此日之前的旧版本
				float theta = (pixel_idx - u) / 96.0 * 20  - 2.5; // 20241213 添加u 20250102 删除+10
				float theta_rad = theta * M_PI / 180.0f;
				float slot_ang_rad = slot_ang * M_PI / 180.0f;

				PointData point = {
					static_cast<uint16_t>(pixel_idx),                // 显式转换为 uint16_t
					static_cast<uint16_t>(slot_index),              // 显式转换为 uint16_t
					static_cast<uint16_t>(std::round(peaks[select_index])), // 显式转换为 uint16_t，并四舍五入
					r,
					theta_rad,
					slot_ang_rad,
					theta,
					slot_ang,
					reflect,
					intensity_uint8,
					r * std::cos(theta_rad) * std::cos(slot_ang_rad),
					r * std::cos(theta_rad) * std::sin(slot_ang_rad),
					r * std::sin(theta_rad),
					true};

				mat[pixel_idx][slot_index] = point;
			}
		}

		infile.close();
	}

	bool saveListToFile(const std::vector<float> &list, const std::string &filename)
	{
		// 打开文件进行写入（若文件不存在则创建，若文件存在则清空）
		std::ofstream outFile(filename, std::ios::trunc); // 使用trunc模式清空文件内容
		if (!outFile)
		{
			std::cerr << "无法打开文件: " << filename << std::endl;
			return false; // 打开文件失败，返回false
		}
		try
		{
			// 使用 std::copy 和 ostream_iterator 一次性将整个列表写入文件
			std::copy(list.begin(), list.end(), std::ostream_iterator<float>(outFile, "\n"));
		}
		catch (const std::exception &e)
		{
			std::cerr << "写入文件时发生异常: " << e.what() << std::endl;
			return false; // 写入文件失败，返回 false
		}
		// 成功写入文件后返回 true
		return true;
	}
	bool loadListFromFile(std::vector<float> &list, const std::string &filename)
	{
		std::ifstream inFile(filename);
		if (!inFile.is_open())
		{
			std::cerr << "loadListFromFile出错, 无法打开文件: " << filename << std::endl;
			return false;
		}

		float value;
		std::string line;
		while (std::getline(inFile, line))
		{
			std::stringstream ss(line);
			if (ss >> value)
			{
				list.push_back(value); // 将每行的值转换为float并添加到list
			}
		}

		inFile.close();
		return true;
	}

	// 降维
	std::vector<PointData> squeeze(const std::vector<std::vector<PointData>> &mat)
	{
		std::vector<PointData> squeezed;

		if (mat.size() == 1) {
			// 如果只有一行，将这行所有元素放入结果中
			squeezed = mat[0];
		}
		else if (!mat.empty() && mat[0].size() == 1) {
			// 如果只有一列，将每行的第一个元素放入结果中
			for (const auto& row : mat) {
				squeezed.push_back(row[0]);
			}
		}

		// 如果既不是单行也不是单列，则返回空的 squeezed，表示无法压缩
		return squeezed;
	}



	void readPulseData3(const std::string &filename, PointMatrix3 &mat)
	{
		const double LIGHT_SPEED = 299792458e-9; // 单位：m/ns
		const int PIXEL_NUM = 96;
		const int SLOT_NUM = 800;
	
		mat.resize(PIXEL_NUM, std::vector<std::vector<PointData>>(SLOT_NUM));
	
		std::ifstream infile(filename);
		if (!infile.is_open())
		{
			std::cerr << "Failed to open file: " << filename << std::endl;
			return;
		}
	
		std::string line;
		int slot_index = 0, last_slot_dics = 0, pixel_cnt = 0;
		float slot_ang = 0.0f;
		bool bDataBegin = false;
	
		while (std::getline(infile, line))
		{
			if (line.find("start of slot") != std::string::npos)
			{
				size_t f1 = line.find('[');
				size_t f2 = line.find(']');
				slot_index = std::stoi(line.substr(f1 + 1, f2 - f1 - 1));
				bDataBegin = false;
				pixel_cnt = 0;
				continue;
			}
	
			if (line.find("angle") != std::string::npos)
			{
				auto tokens = splitString(line, ' ');
				int slot_dics = std::stoi(tokens.back());
				if (slot_index == 0)
				{
					slot_ang = -60.0f;
				}
				else if (slot_index == 1)
				{
					slot_ang = -59.85f;
					last_slot_dics = slot_dics;
				}
				else
				{
					int delta_dics = (slot_dics >= last_slot_dics) ? (slot_dics - last_slot_dics) : (slot_dics + 65536 - last_slot_dics);
					slot_ang += delta_dics * 360.0f / 65536.0f * 2.0f;
					last_slot_dics = slot_dics;
				}
				bDataBegin = true;
				continue;
			}
	
			if (bDataBegin)
			{
				pixel_cnt++;
				if (pixel_cnt > PIXEL_NUM || slot_index >= SLOT_NUM)
				{
					continue;
				}
	
				auto tokens = splitString(line, ' ');
				if (tokens.size() < 9)
					continue;
	
				std::vector<double> peaks = {
					std::stod(tokens[2 + 2]), // 第3、6、9个元素是峰值
					std::stod(tokens[5 + 2]),
					std::stod(tokens[8 + 2])
				};
				std::vector<double> tofs = {
					std::stod(tokens[0 + 2]) / 1000, // TOF1
					std::stod(tokens[3 + 2]) / 1000, // TOF2
					std::stod(tokens[6 + 2]) / 1000  // TOF3
				};
				std::vector<double> widths = {
					std::stod(tokens[1 + 2]) / 1000, // 宽度1
					std::stod(tokens[4 + 2]) / 1000, // 宽度2
					std::stod(tokens[7 + 2]) / 1000  // 宽度3
				};
	
				int pixel_roi_start = 0;
				int pixel_idx = pixel_cnt - 1 + pixel_roi_start;
				int u = 47;
	
				float theta = (pixel_idx - u) / 96.0f * 20.0f - 2.5f;
				float theta_rad = theta * M_PI / 180.0f;
				float slot_ang_rad = slot_ang * M_PI / 180.0f;
	
				// 处理三个回波
				for (int i = 0; i < 3; ++i)
				{
					if (tofs[i] == 0.016)
						continue; // 跳过无效数据
	
					float r = tofs[i] * LIGHT_SPEED / 2.0f;
					float intensity_val = std::min(255.0, std::round(peaks[i] * widths[i] / 702.0 / 5.0 * 256.0));
					uint8_t intensity = static_cast<uint8_t>(intensity_val);
					float reflect = intensity_val / 255.0f;
	
					PointData point{
						static_cast<uint16_t>(pixel_idx),
						static_cast<uint16_t>(slot_index),
						static_cast<uint16_t>(std::round(peaks[i])),
						r,
						theta_rad,
						slot_ang_rad,
						theta,
						slot_ang,
						reflect,
						intensity,
						r * std::cos(theta_rad) * std::cos(slot_ang_rad),
						r * std::cos(theta_rad) * std::sin(slot_ang_rad),
						r * std::sin(theta_rad),
						true};
	
					// 检查边界后添加点到三维矩阵
					if (pixel_idx < PIXEL_NUM && slot_index < SLOT_NUM)
					{
						mat[pixel_idx][slot_index].push_back(point);
					}
				}
			}
		}
	
		infile.close();
	}

}