/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2025-02-26 16:36:30
 * @LastEditors  : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-02-26 16:36:30
 * @FilePath     : /pcl_viewer/src/alog_interface.cpp
 * @Description  : 
 * 
 * Copyright (c) 2025 by vitalchem, All Rights Reserved. 
 */

#include "alog_interface.h"

#include <stdexcept>
#include <algorithm>
#include <numeric>

#include <debug_visual.h>

namespace alog_lidar
{
	// 计算一个浮点数组的中位数 若数组为空 则定义 median 为 0
	float getMedian(std::vector<float> &list)
	{
		if (list.empty())
			return 0.0f;
		std::sort(list.begin(), list.end());
		size_t n = list.size();
		if (n % 2 == 0)
		{
			return (list[n / 2 - 1] + list[n / 2]) / 2.0f; // 偶数个元素时取中间两个数的平均值
		}
		else
		{
			return list[n / 2]; // 奇数个元素时取中间的元素
		}
	}

	// 接口的实现
	void RepairGround_I::doRepairGround()
	{
	}
	// 算法DEBUG本地测试
	void RepairGround_I::doTest()
	{
	}

}
