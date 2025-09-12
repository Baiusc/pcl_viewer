/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-02 14:30:29
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-09-12 15:50:04
 * @FilePath     : /pcl_viewer/src/algo_interface.cpp
 * @Description  : 算法接口的实现
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */
#include "algo_interface.h"

#include <stdexcept>
#include <algorithm>
#include <numeric>

namespace algo_lidar
{

	// 统计 select_idx 分布情况并打印
	void printSelectIdx(const PointMatrix& mat, const std::string& label) {
		int count0 = 0, count1 = 0, count2 = 0;

		// 遍历矩阵，统计 select_idx 的个数
		for (const auto& row : mat) {
			for (const auto& point : row) {
				if (point.select_idx == 0) {
					count0++;
				} else if (point.select_idx == 1) {
					count1++;
				} else if (point.select_idx == 2) {
					count2++;
				}
			}
		}

		// 打印统计结果
		std::cout << label << "：select_idx 0 有 " << count0 << " 个，select_idx 1 有 " << count1 << " 个，select_idx 2 有 " << count2 << " 个。" << std::endl;
	}
	// 根据回波数据更新点数据
	void updatePtByPulse(PointData &pt, const Pulse &pulse, uint8_t select_idx)
	{
		const double LIGHT_SPEED = 299792458e-9; // 单位：m/ns
		// 更新 select_idx
		pt.select_idx = select_idx;
		// 重新计算参数
		pt.r = pulse.tof * LIGHT_SPEED / 2.0f;
		pt.x = pt.r * cos(pt.theta_rad) * cos(pt.omega_rad);
		pt.y = pt.r * cos(pt.theta_rad) * sin(pt.omega_rad);
		pt.z = pt.r * sin(pt.theta_rad);
		pt.peak = static_cast<uint16_t>(pulse.peak);
	}
	
	// 单列修复 无反转
	void repairSingleCol(std::vector<PointData> &col_pts, float tof_valid_min)
	{
		for (int i = 1; i < col_pts.size(); ++i)
		{
			// 跳过无效点
			if (!col_pts[i - 1].is_valid || !col_pts[i].is_valid)
				continue;
			// 若当前点的 TOF 值无效
			if (col_pts[i].list_pulse[col_pts[i].select_idx].tof < tof_valid_min)
			{
				// 执行修复
				auto new_idx = col_pts[i].select_idx + 1;
				if (new_idx >= col_pts[i].list_pulse.size())
				{
					continue;
				}
				updatePtByPulse(col_pts[i], col_pts[i].list_pulse[new_idx], new_idx);
				continue;
			}
			// 若上一个点的 TOF 值无效
			if (col_pts[i - 1].list_pulse[col_pts[i - 1].select_idx].tof < tof_valid_min)
				continue;
			// 动态TOF阈值计算
			float tof_threshold = 1.f; // 每列的初始阈值 0.15m
			if (i >= 2 && col_pts[i - 2].is_valid)
			{
				float tof_diff = std::abs(col_pts[i - 1].list_pulse[col_pts[i - 1].select_idx].tof -
										  col_pts[i - 2].list_pulse[col_pts[i - 2].select_idx].tof);
				if (tof_diff > 1.0f && tof_diff < 5.f) // 0.15 ~ 0.75m
				{
					tof_threshold = tof_diff * 2.0f;
				}
			}

			// 关键指标
			auto &curr_pt = col_pts[i];
			const auto &prev_pt = col_pts[i - 1];
			const float delta_z = curr_pt.z - prev_pt.z;
			const float delta_r = curr_pt.r - prev_pt.r;

			// 多回波遍历修复
			for (int pulse_idx = 0; pulse_idx < curr_pt.list_pulse.size(); ++pulse_idx)
			{
				if (pulse_idx == curr_pt.select_idx) // 跳过当前选择的脉冲
					continue;

				// 计算TOF差异
				float tof_diff = std::abs(
					curr_pt.list_pulse[pulse_idx].tof -
					prev_pt.list_pulse[prev_pt.select_idx].tof);

				// 2025.02.28 考虑添加判断：不能被离群的孤立点影响
				if (i >= 2 && col_pts[i - 2].is_valid)
				{
					float tof_diff_pp = std::abs(curr_pt.list_pulse[pulse_idx].tof -
												 col_pts[i - 2].list_pulse[col_pts[i - 2].select_idx].tof);
					tof_diff = (tof_diff + tof_diff_pp) / 2.0f;
				}

				// 修复条件判断
				if (delta_r > 0.f &&
					delta_z < 0.f &&
					std::abs(delta_r) > std::abs(delta_z) &&
					tof_diff < tof_threshold)
				{
					// 执行修复
					updatePtByPulse(curr_pt, curr_pt.list_pulse[pulse_idx], pulse_idx);
				}
			}
		}
	}

	// 模拟实时的单个slot修复（单列修复+单列反转 trans +单列修复） 
	void repairSingleSlot(std::vector<PointData> &pts_slot, float tof_valid_min)
	{
		repairSingleCol(pts_slot, tof_valid_min);
		std::reverse(pts_slot.begin(), pts_slot.end());
		repairSingleCol(pts_slot, tof_valid_min);
		std::reverse(pts_slot.begin(), pts_slot.end());
	}

	// 输入单帧点云，按列修复
	void repairByCol(PointMatrix &mat, float tof_valid_min)
	{
		// 矩阵行列转置
		transMat(mat);

		// 遍历每列
		for (int col_idx = 0; col_idx < mat.size(); ++col_idx)
		{
			auto &col_pts = mat[col_idx];
			repairSingleSlot(col_pts, tof_valid_min);
		}
		// 转置回原始矩阵结构
		transMat(mat);
		return;
	}

	// 输入单帧点云，按行修复
	void repairByRow(PointMatrix &mat,float tof_valid_min)
	{
		// 遍历每行
		for (int row_idx = 0; row_idx < mat.size(); ++row_idx)
		{
			auto &row_pts = mat[row_idx];
			for (int i = 1; i < row_pts.size(); ++i)
			{
				// 跳过无效点
				if (!row_pts[i - 1].is_valid || !row_pts[i].is_valid)
					continue;
				// 若当前点的 TOF 值无效
				if (row_pts[i].list_pulse[row_pts[i].select_idx].tof < tof_valid_min)
				{
					// 执行修复
					auto new_idx = row_pts[i].select_idx + 1;
					if (new_idx >= row_pts[i].list_pulse.size())
					{
						continue;
					}
					updatePtByPulse(row_pts[i], row_pts[i].list_pulse[new_idx], new_idx);
					continue;
				}

				// 若上一个点的 TOF 值无效
				if (row_pts[i - 1].list_pulse[row_pts[i - 1].select_idx].tof < tof_valid_min)
					continue;

				// 固定TOF阈值 (行修复专用)
				constexpr float tof_threshold = 1.0f; // 对应0.15m

				// 几何特征计算
				auto &curr_pt = row_pts[i];
				const auto &prev_pt = row_pts[i - 1];
				const float delta_z = curr_pt.z - prev_pt.z;
				const float delta_r = curr_pt.r - prev_pt.r;

				// 多回波遍历修复
				for (int pulse_idx = 0; pulse_idx < curr_pt.list_pulse.size(); ++pulse_idx)
				{
					if (pulse_idx == curr_pt.select_idx) // 跳过已选脉冲
						continue;

					// 计算TOF差异
					float tof_diff = std::abs(
						curr_pt.list_pulse[pulse_idx].tof -
						prev_pt.list_pulse[prev_pt.select_idx].tof);
					// 2025.02.28 考虑添加判断：不能被离群的孤立点影响
					if (i >= 2 && row_pts[i - 2].is_valid)
					{
						float tof_diff_pp = std::abs(curr_pt.list_pulse[pulse_idx].tof -
													 row_pts[i - 2].list_pulse[row_pts[i - 2].select_idx].tof);
						tof_diff = (tof_diff + tof_diff_pp) / 2.0f;
					}

					// 修复条件判断（行修复阈值更严格）
					if (delta_r > 0.f &&
						// delta_z < 0.f &&
						std::abs(delta_r) > std::abs(delta_z) &&
						tof_diff < tof_threshold)
					{
						// 执行修复
						updatePtByPulse(curr_pt, curr_pt.list_pulse[pulse_idx], pulse_idx);
					}
				}
			}
		}
	}

	// 辅助方法：反转矩阵的列序和行序
	void reverseMatrix(PointMatrix &mat)
	{
		// 反转每一列（内层向量）
		for (auto &col : mat)
		{
			std::reverse(col.begin(), col.end());
		}

		// 反转外层向量（行序）
		std::reverse(mat.begin(), mat.end());
	}
	
	// 输入单帧点云，进行一条龙全流程的修复
	void repairMat(PointMatrix &mat, float tof_valid_min)
	{
		// 按列修复
		repairByCol(mat, tof_valid_min);

		// 按行修复
		repairByRow(mat, tof_valid_min);

		reverseMatrix(mat);

		// 按列修复
		repairByCol(mat, tof_valid_min);

		// 按行修复
		repairByRow(mat, tof_valid_min);

		reverseMatrix(mat);
	}

	// 地面反射点修复算法接口  输入单个slot 实时 （包含3个回波数据）
	void Ground_I::doRepair(std::vector<PointData> &pts_slot, float r_offset)
	{
		float tof_valid_min = r_offset * 2 / 299792458e-9; // 低于此值的回波认为无效or需要修复
		// tof_valid_min = 36.6f; // 0.15m

		// 模拟实时的单个slot的修复
		Timer timer;
		repairSingleSlot(pts_slot, tof_valid_min);
		timer.out("repairSingleSlot:");

		return;
	}

	// 地面反射点修复算法接口  输入单帧点云 （包含3个回波数据）
	void Ground_I::doRepair(PointMatrix &mat, float r_offset)
	{
		float tof_valid_min = r_offset * 2 / 299792458e-9; // 低于此值的回波认为无效or需要修复
		// tof_valid_min = 36.6f; // 0.15m

		// 统计修复前的 select_idx 分布情况并打印
		printSelectIdx(mat, "修复前");
		Timer timer;
		// 模拟单帧点云的全流程的修复
		repairMat(mat, tof_valid_min);
		timer.out("repairMat:");
		// 统计修复后的 select_idx 分布情况并打印
		printSelectIdx(mat, "修复后");
		return;
	}
}
