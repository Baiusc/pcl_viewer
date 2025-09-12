/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-02 14:04:29
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-09-12 15:50:11
 * @FilePath     : /pcl_viewer/include/algo_interface.h
 * @Description  : 地面反射点修复算法接口
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */
#pragma once
#include <vector>
#include <string>
#include <chrono>
#include <iostream>
#include "point_type.h"
namespace algo_lidar
{
    // 地面修复算法 接口类
    class Ground_I
    {
    public:
        // 地面反射点修复算法接口  输入单个slot 实时 （包含3个回波数据）
        static void doRepair(std::vector<PointData> &pts_slot, float r_offset = 5.5f + 1.f);
        // 地面反射点修复算法接口  输入单帧点云 （包含3个回波数据）
        static void doRepair(PointMatrix &mat, float r_offset = 5.5f + 1.f);

    private:

    };
    // 计时器类，用于测量程序执行时间
	class Timer
	{
	public:
		Timer() : beg_(clock_::now()) {}
		void reset() { beg_ = clock_::now(); }
		double elapsed() const {
			return std::chrono::duration_cast<second_>
				(clock_::now() - beg_).count();
		}
		void out(std::string message = "") {
			double t = elapsed();
			std::cout << message << "\nelapsed time:" << t << "s" << std::endl;
			reset();
		}
	private:
		typedef std::chrono::high_resolution_clock clock_;
		typedef std::chrono::duration<double, std::ratio<1> > second_;
		std::chrono::time_point<clock_> beg_;
	};
}
