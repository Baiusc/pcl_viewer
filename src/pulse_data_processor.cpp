/*
 * @Author       : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @Date         : 2025-03-06 17:24:45
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2026-07-21 15:41:45
 * @FilePath     : /pcl_viewer/src/pulse_data_processor.cpp
 * @Description  :
 *
 * Copyright (c) 2025 by vitalchem, All Rights Reserved.
 */

#include "pulse_data_processor.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <iomanip>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace algo_lidar
{
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

	// 读取 转镜雷达原始点云
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
			if (line.find("start of") != std::string::npos)
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
					if (delta_dics > 100)
					{
						delta_dics = 13;
					}
					//  delta_dics = 13;
					auto delta_ang = delta_dics * 360.0f / 65536.0f * 2.0f;
					slot_ang += delta_dics * 360.0f / 65536.0f * 2.0f;
					//  slot_ang += 0.15f;
					last_slot_dics = slot_dics;
					if (slot_ang > 65.0f || slot_ang < -65.0f || delta_ang < 0.15f * 0.8 || delta_ang > 0.3f)
					{
						std::cout << "err slot_ang: " << slot_ang << " err delta_ang: " << delta_ang << " slot_index: " << slot_index << std::endl;
					}
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
					{
						mat[pixel_cnt - 1][slot_index].slot_ang = slot_ang; // 20250320 添加无效点的slot_ang
						continue;											// 过滤无效数据
					}
					if (widths[i] == 0.016)
					{
						mat[pixel_cnt - 1][slot_index].slot_ang = slot_ang; // 20250320 添加无效点的slot_ang
						continue;											// 过滤无效数据
					}
					if (peaks[i] > max_peak)
					{
						max_peak = peaks[i];
						select_index = i;
						// select_index = 0;
					}
				}

				if (select_index == -1)
				{
					mat[pixel_cnt - 1][slot_index].slot_ang = slot_ang; // 20250320 添加无效点的slot_ang
					continue;
				}

				int pixel_roi_start = 0; // 若原始点云是裁剪后的roi点云 则加上起始pixel号 默认为0
				int pixel_idx = pixel_cnt - 1 + pixel_roi_start;
				int u = 47; // u: SPAD中心在y竖直方向上的偏移量（像素单位），默认 47

				float r = tofs[select_index] * LIGHT_SPEED / 2.0;
				float intensity = std::min(255.0, std::round(peaks[select_index] * widths[select_index] / 1638.0 / 10.0 * 256));
				float reflect = intensity / 255.0;
				uint8_t intensity_uint8 = static_cast<uint8_t>(intensity);
				// float theta = -pixel_idx / 96.0 * 20 + 10 - 2.5; // 20241213此日之前的旧版本
				float theta = (pixel_idx - u) / 96.0 * 20 - 2.5; // 20241213 添加u 20250102 删除+10
				float theta_rad = theta * M_PI / 180.0f;
				float slot_ang_rad = slot_ang * M_PI / 180.0f;

				std::vector<Pulse> list_pulse; // 3组回波数据
				for (size_t i = 0; i < peaks.size(); ++i)
				{
					list_pulse.emplace_back(
						static_cast<float>(tofs[i]),
						static_cast<float>(widths[i]),
						static_cast<float>(peaks[i]));
				}
				// 按 peak 从大到小排序
				std::sort(list_pulse.begin(), list_pulse.end(), [](const Pulse &a, const Pulse &b)
						  { return a.peak > b.peak; });

				PointData point = {
					static_cast<uint16_t>(pixel_idx),						// 显式转换为 uint16_t
					static_cast<uint16_t>(slot_index),						// 显式转换为 uint16_t
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
					true,
					list_pulse,
					0};

				mat[pixel_idx][slot_index] = point;
			}
		}

		infile.close();
	}
	// 读取 固态雷达原始点云
	bool readSolidLidarData(const std::string &filename, PointMatrix &mat)
	{
		const double LIGHT_SPEED = 299792458e-9; // 单位：m/ns

		std::ifstream infile(filename);
		if (!infile.is_open())
		{
			std::cerr << "Failed to open file: " << filename << std::endl;
			return false;
		}

		// ── 1. 解析文件头，获取帧级参数 ──────────────────────────────────────
		int num_rows = 152, num_cols = 360;
		int angle_per_col = 100; // 默认值，单位：1/65536 圈 * 某倍率（见下方换算）

		std::string line;
		while (std::getline(infile, line))
		{
			if (line.find("cols:") != std::string::npos)
			{
				// 格式: "cols: 360  rows: 152"
				auto tokens = splitString(line, ' ');
				for (size_t i = 0; i + 1 < tokens.size(); ++i)
				{
					if (tokens[i] == "cols:")
						num_cols = std::stoi(tokens[i + 1]);
					if (tokens[i] == "rows:")
						num_rows = std::stoi(tokens[i + 1]);
				}
			}
			if (line.find("angle_per_col:") != std::string::npos)
			{
				auto tokens = splitString(line, ' ');
				if (tokens.size() >= 2)
					angle_per_col = std::stoi(tokens.back());
			}
			// 头部结束标志：遇到第一个 "start of col" 则退出头部解析
			if (line.find("start of col") != std::string::npos)
			{
				// 把这行放回去（通过回退文件位置）再统一处理
				// 用 seekg 不方便，直接在主循环里处理该行
				goto parse_body;
			}
		}

	parse_body:
		// ── 2. 初始化矩阵 ───────────────────────────────────────────────────
		mat.assign(num_rows, std::vector<PointData>(num_cols));
		for (int i = 0; i < num_rows; ++i)
			for (int j = 0; j < num_cols; ++j)
				mat[i][j].is_valid = false;

		// ── 3. 解析列数据 ───────────────────────────────────────────────────
		int col_index = 0;
		float col_ang = 0.0f; // 当前列的水平角（角度）
		bool bDataBegin = false;

		// 注意：上面 goto 之前已经 getline 出了第一个 "start of col" 行，
		// 但我们用 goto 跳过了那次处理，所以先把 line 当作当前行处理一次。
		auto processLine = [&](const std::string &ln)
		{
			// ── 3a. 列起始行 ──────────────────────────────────────────────
			if (ln.find("start of col") != std::string::npos)
			{
				// 格式: "start of col [N], angle: A"
				auto f1 = ln.find('[');
				auto f2 = ln.find(']');
				col_index = std::stoi(ln.substr(f1 + 1, f2 - f1 - 1));

				// 解析 angle（原始编码值）
				auto fa = ln.find("angle:");
				int raw_angle = 0;
				if (fa != std::string::npos)
					raw_angle = std::stoi(ln.substr(fa + 6)); // 跳过 "angle:"

				// 换算：raw_angle 单位与 angle_per_col 相同（均为 1/65536 圈 * 2 倍）
				// 实际水平角（度） = raw_angle / 65536 * 360 * 2
				// col_ang = raw_angle / 65536.0f * 360.0f * 2.0f;
				col_ang = col_index / 360.f * 120.f - 60.f;

				bDataBegin = false;
				return;
			}

			// ── 3b. 列结束行（忽略） ──────────────────────────────────────
			if (ln.find("end of col") != std::string::npos)
			{
				bDataBegin = false;
				return;
			}

			// ── 3c. 行数据行 ──────────────────────────────────────────────
			if (ln.find("row:") != std::string::npos)
			{
				bDataBegin = true;

				if (col_index < 0 || col_index >= num_cols)
					return;

				// 格式: "row:   N   tof1 width1 peak1  tof2 width2 peak2  tof3 width3 peak3"
				// 先截掉 "row:" 前缀，再按空格分割
				auto colon_pos = ln.find(':');
				std::string data_str = ln.substr(colon_pos + 1);
				auto tokens = splitString(data_str, ' ');

				// tokens[0] = row_idx, tokens[1..9] = 3组(tof, width, peak)
				if (tokens.size() < 10)
					return;

				int row_idx = std::stoi(tokens[0]);
				if (row_idx < 0 || row_idx >= num_rows)
					return;

				// 3组回波数据（原始值单位均为 ps，需 /1000 转 ns）
				std::vector<double> tofs = {std::stod(tokens[1]) / 1000.0,
											std::stod(tokens[4]) / 1000.0,
											std::stod(tokens[7]) / 1000.0};
				std::vector<double> widths = {std::stod(tokens[2]) / 1000.0,
											  std::stod(tokens[5]) / 1000.0,
											  std::stod(tokens[8]) / 1000.0};
				std::vector<double> peaks = {std::stod(tokens[3]),
											 std::stod(tokens[6]),
											 std::stod(tokens[9])};

				// 选最强有效回波（与 readPulseData 逻辑一致）
				int select_index = -1;
				double max_peak = 1.0;
				for (int i = 0; i < 3; ++i)
				{
					if (tofs[i] == 0.016)
						continue; // 无效标记
					if (widths[i] == 0.016)
						continue;
					if (peaks[i] > max_peak)
					{
						max_peak = peaks[i];
						select_index = i;
					}
				}

				// 记录无效点的角度信息，方便后续可视化
				mat[row_idx][col_index].slot_ang = col_ang;
				mat[row_idx][col_index].pixel_idx = static_cast<uint16_t>(row_idx);
				mat[row_idx][col_index].slot_idx = static_cast<uint16_t>(col_index);

				if (select_index == -1)
					return; // 无有效回波

				// ── 计算物理量 ───────────────────────────────────────────
				const int U_ROW = 75; // 固态雷达垂直中心行（152行，中心约第76行，0-based取75）

				float r = static_cast<float>(tofs[select_index] * LIGHT_SPEED / 2.0);
				float intensity = static_cast<float>(
					std::min(255.0, std::round(peaks[select_index] * widths[select_index] / 4095.0 / 3.0 * 255.0)));
				float reflect = intensity / 255.0f;
				uint8_t intensity_u8 = static_cast<uint8_t>(intensity);

				// 垂直视场角：152行对应约50°，中心偏移 U_ROW
				float theta = (row_idx - U_ROW) / 152.0f * 50.0f;
				float theta_rad = theta * static_cast<float>(M_PI) / 180.0f;
				float col_ang_rad = col_ang * static_cast<float>(M_PI) / 180.0f;

				// 构建有序回波列表（按 peak 降序）
				std::vector<Pulse> list_pulse;
				for (int i = 0; i < 3; ++i)
					list_pulse.emplace_back(static_cast<float>(tofs[i]),
											static_cast<float>(widths[i]),
											static_cast<float>(peaks[i]));
				std::sort(list_pulse.begin(), list_pulse.end(),
						  [](const Pulse &a, const Pulse &b)
						  { return a.peak > b.peak; });

				PointData point;
				point.pixel_idx = static_cast<uint16_t>(row_idx);
				point.slot_idx = static_cast<uint16_t>(col_index);
				point.peak = static_cast<uint16_t>(std::round(peaks[select_index]));
				point.r = r;
				point.theta_rad = theta_rad;
				point.omega_rad = col_ang_rad;
				point.theta = theta;
				point.slot_ang = col_ang;
				point.reflect = reflect;
				point.intensity = intensity_u8;
				point.x = r * std::cos(theta_rad) * std::cos(col_ang_rad);
				point.y = r * std::cos(theta_rad) * std::sin(col_ang_rad);
				point.z = r * std::sin(theta_rad);
				point.is_valid = true;
				point.list_pulse = list_pulse;
				point.select_idx = static_cast<uint8_t>(select_index);

				mat[row_idx][col_index] = point;
			}
		};

		// 处理 goto 之前已读出的 "start of col [0]" 行
		processLine(line);

		// 继续读取剩余行
		while (std::getline(infile, line))
			processLine(line);

		infile.close();
		return true;
	}
	// 读取 任意雷达的原始点云
	bool readLidarData(const std::string &filename, PointMatrix &mat)
	{
		std::ifstream infile(filename);
		if (!infile.is_open())
		{
			std::cerr << "Failed to open file: " << filename << std::endl;
			return false;
		}

		std::string first_line;
		std::getline(infile, first_line);
		infile.close();

		if (first_line.find("cols: 360  rows: 152") != std::string::npos)
		{
			return readSolidLidarData(filename, mat);
		}
		else
		{
			readPulseData(filename, mat);
			return true;
		}
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

		if (mat.size() == 1)
		{
			// 如果只有一行，将这行所有元素放入结果中
			squeezed = mat[0];
		}
		else if (!mat.empty() && mat[0].size() == 1)
		{
			// 如果只有一列，将每行的第一个元素放入结果中
			for (const auto &row : mat)
			{
				squeezed.push_back(row[0]);
			}
		}

		// 如果既不是单行也不是单列，则返回空的 squeezed，表示无法压缩
		return squeezed;
	}
	// ================================================================
	// 内部辅助：二维数组 CSV 存取
	// 格式与 saveROffsetToCsv 一致：
	//   首行: "row\col,0,1,2,..."
	//   数据行: "row_idx,val0,val1,..."
	// ================================================================
	static bool saveList2ToCsv(const std::vector<std::vector<float>> &list2,
							   const std::string &filename)
	{
		std::ofstream ofs(filename, std::ios::trunc);
		if (!ofs)
		{
			std::cerr << "[saveListToFile] 无法打开文件: " << filename << std::endl;
			return false;
		}

		const int num_rows = static_cast<int>(list2.size());
		const int num_cols = num_rows > 0 ? static_cast<int>(list2[0].size()) : 0;

		// 表头行
		ofs << "row\\col";
		for (int c = 0; c < num_cols; ++c)
			ofs << ',' << c;
		ofs << '\n';

		// 数据行
		ofs << std::fixed << std::setprecision(6);
		for (int r = 0; r < num_rows; ++r)
		{
			ofs << r;
			const int cur_cols = static_cast<int>(list2[r].size());
			for (int c = 0; c < num_cols; ++c)
			{
				// 当前行列数不足时补 0（容错）
				ofs << ',' << (c < cur_cols ? list2[r][c] : 0.0f);
			}
			ofs << '\n';
		}

		ofs.close();
		return true;
	}

	static bool loadList2FromCsv(std::vector<std::vector<float>> &list2,
								 const std::string &filename)
	{
		std::ifstream ifs(filename);
		if (!ifs.is_open())
		{
			std::cerr << "[loadListFromFile] 无法打开文件: " << filename << std::endl;
			return false;
		}

		list2.clear();
		std::string line;
		bool is_header = true;
		int expected_cols = -1;

		while (std::getline(ifs, line))
		{
			if (line.empty())
				continue;

			std::stringstream ss(line);
			std::string token;
			std::vector<std::string> tokens;
			while (std::getline(ss, token, ','))
				tokens.push_back(token);

			// 解析表头，确定列数
			if (is_header)
			{
				expected_cols = static_cast<int>(tokens.size()) - 1; // 去掉首列标签
				if (expected_cols <= 0)
				{
					std::cerr << "[loadListFromFile] CSV 表头格式非法: " << line << std::endl;
					return false;
				}
				is_header = false;
				continue;
			}

			// 数据行：tokens[0] 为行号，tokens[1..] 为数据
			if (static_cast<int>(tokens.size()) != expected_cols + 1)
			{
				std::cerr << "[loadListFromFile] 第 " << list2.size()
						  << " 行列数不匹配，跳过。" << std::endl;
				continue;
			}

			try
			{
				std::vector<float> row;
				row.reserve(expected_cols);
				for (int c = 1; c <= expected_cols; ++c)
					row.push_back(std::stof(tokens[c]));
				list2.push_back(std::move(row));
			}
			catch (const std::exception &e)
			{
				std::cerr << "[loadListFromFile] 解析失败（" << e.what()
						  << "），行: " << line << std::endl;
			}
		}

		ifs.close();
		return !list2.empty();
	}

	// ================================================================
	// 内部辅助：二维数组 BIN 存取
	// 二进制格式（参考 binary_storage.cpp）：
	//   [uint32_t num_rows]
	//   [uint32_t num_cols]        ← 以第一行列数为准
	//   [float * num_rows * num_cols]  ← 行优先平铺，不足列补 0
	// ================================================================
	static bool saveList2ToBin(const std::vector<std::vector<float>> &list2,
							   const std::string &filename)
	{
		std::ofstream ofs(filename, std::ios::binary | std::ios::trunc);
		if (!ofs)
		{
			std::cerr << "[saveListToFile] 无法打开文件: " << filename << std::endl;
			return false;
		}

		const uint32_t num_rows = static_cast<uint32_t>(list2.size());
		const uint32_t num_cols = num_rows > 0
									  ? static_cast<uint32_t>(list2[0].size())
									  : 0u;

		// 写入行列数
		ofs.write(reinterpret_cast<const char *>(&num_rows), sizeof(num_rows));
		ofs.write(reinterpret_cast<const char *>(&num_cols), sizeof(num_cols));

		// 逐行写入，不足列补 0
		const float zero = 0.0f;
		for (uint32_t r = 0; r < num_rows; ++r)
		{
			const uint32_t cur_cols = static_cast<uint32_t>(list2[r].size());
			const uint32_t write_cols = std::min(cur_cols, num_cols);

			// 写有效数据
			if (write_cols > 0)
				ofs.write(reinterpret_cast<const char *>(list2[r].data()),
						  sizeof(float) * write_cols);

			// 补齐不足列
			for (uint32_t c = write_cols; c < num_cols; ++c)
				ofs.write(reinterpret_cast<const char *>(&zero), sizeof(float));
		}

		ofs.close();
		return true;
	}

	static bool loadList2FromBin(std::vector<std::vector<float>> &list2,
								 const std::string &filename)
	{
		std::ifstream ifs(filename, std::ios::binary);
		if (!ifs)
		{
			std::cerr << "[loadListFromFile] 无法打开文件: " << filename << std::endl;
			return false;
		}

		uint32_t num_rows = 0, num_cols = 0;
		ifs.read(reinterpret_cast<char *>(&num_rows), sizeof(num_rows));
		ifs.read(reinterpret_cast<char *>(&num_cols), sizeof(num_cols));

		if (!ifs || num_rows == 0 || num_cols == 0)
		{
			std::cerr << "[loadListFromFile] BIN 文件头读取失败或尺寸为零。" << std::endl;
			return false;
		}

		list2.assign(num_rows, std::vector<float>(num_cols, 0.0f));

		for (uint32_t r = 0; r < num_rows; ++r)
		{
			ifs.read(reinterpret_cast<char *>(list2[r].data()),
					 sizeof(float) * num_cols);
			if (!ifs)
			{
				std::cerr << "[loadListFromFile] BIN 文件在第 " << r
						  << " 行读取中断，数据可能不完整。" << std::endl;
				return false;
			}
		}

		ifs.close();
		return true;
	}

	// ================================================================
	// 公开接口：saveListToFile 二维重载
	// ================================================================
	bool saveListToFile(const std::vector<std::vector<float>> &list2,
						const std::string &filename,
						const std::string &type)
	{
		if (type == ".csv")
			return saveList2ToCsv(list2, filename);

		if (type == ".bin")
			return saveList2ToBin(list2, filename);

		std::cerr << "[saveListToFile] 不支持的文件类型: " << type
				  << "，当前支持 \".csv\" 和 \".bin\"。" << std::endl;
		return false;
	}

	// ================================================================
	// 公开接口：loadListFromFile 二维重载
	// ================================================================
	bool loadListFromFile(std::vector<std::vector<float>> &list2,
						  const std::string &filename,
						  const std::string &type)
	{
		if (type == ".csv")
			return loadList2FromCsv(list2, filename);

		if (type == ".bin")
			return loadList2FromBin(list2, filename);

		std::cerr << "[loadListFromFile] 不支持的文件类型: " << type
				  << "，当前支持 \".csv\" 和 \".bin\"。" << std::endl;
		return false;
	}
}