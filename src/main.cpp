/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-02 16:48:05
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2026-07-21 15:28:57
 * @FilePath     : /pcl_viewer/src/main.cpp
 * @Description  : 算法本地测试的主程序
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */

#include <filesystem>
#include <fstream>

#include <point_type.h>

#include <pulse_data_processor.h>
#include <PclHelperB.h>
#include <debug_visual.h>
#include <algo_interface.h>
#include <binary_storage.h>
#include <calc_intensity.h>
#include "tail_point_filter.h"

using namespace algo_lidar;
namespace fs = std::filesystem;

// 测试点云格式转化器
void testConvert()
{
	int rows = 96, cols = 800;

	// 生成调试数据
	std::vector<PolarPoint> debugData = Converter::createPolarPt(rows, cols);

	// 转换为矩阵
	PointMatrix matrix = Converter::polar2Mat(debugData, rows, cols);

	// 打印部分矩阵数据验证
	for (int row = 0; row < rows; ++row) {
	    for (int col = 0; col < cols; ++col) {
	        const auto& point = matrix[row][col];
	        std::cout << "Point(" << row << ", " << col << "): "
	                  << "r = " << point.r << ", "
	                  << "theta = " << point.theta << ", "
	                  << "slot_ang = " << point.slot_ang << "\n";
	    }
	}
}

// 获取文件夹中所有符合后缀的文件路径
std::vector<std::string> getFullFilePath(const std::string &folder, const std::string &postfix)
{
	std::vector<std::string> filePaths;
	for (const auto &entry : fs::directory_iterator(folder))
	{
		// 增加 postfix.empty() 判断
		if (postfix.empty() || entry.path().extension() == postfix)
		{
			filePaths.push_back(entry.path().string());
		}
	}
	return filePaths;
}

/// @brief 获取指定目录下匹配后缀的文件完整路径
/// @param folder 根目录路径
/// @param postfix 文件后缀（如 ".txt"，空则匹配所有文件）
/// @param recursive 是否递归遍历子文件夹
/// @return 所有匹配文件的完整路径数组
std::vector<std::string> getFullFilePath(const std::string &folder, const std::string &postfix, bool recursive)
{
    std::vector<std::string> filePaths;

    // 目录合法性校验
    if (!fs::exists(folder) || !fs::is_directory(folder))
    {
        return filePaths;
    }

    if (recursive)
    {
        // 递归遍历所有子目录
        for (const auto &entry : fs::recursive_directory_iterator(folder))
        {
            if (!entry.is_regular_file())
                continue;

            if (postfix.empty() || entry.path().extension() == postfix)
            {
                filePaths.push_back(entry.path().string());
            }
        }
    }
    else
    {
        // 仅遍历当前目录，不进入子文件夹
        for (const auto &entry : fs::directory_iterator(folder))
        {
            if (!entry.is_regular_file())
                continue;

            if (postfix.empty() || entry.path().extension() == postfix)
            {
                filePaths.push_back(entry.path().string());
            }
        }
    }

    return filePaths;
}

// 托尾点滤波测试 
void filtTailPoints(PointMatrix& mat) {
    // 1. 创建滤波器实例
    algo_lidar::TailPointFilter filter;
    // // 2. 初始化（传入配置文件路径）TODO
    // if (!filter.Init("config.yaml")) {
    //     std::cerr << "Failed to initialize TailPointFilter!" << std::endl;
    //     return -1;
    // }

    // 3. 遍历 mat_src 的每一行，进行滤波处理
	for ( auto& row : mat) {
		// 执行滤波处理
		filter.Process(row);
	}

    return ;
}

// 解析回波数据，可视化原始点云
int testShowCloud()
{
	PclHelperB pclHelper;
	Ground_I algo_gnd;
	CalibReflect algo_reflect;
	// 设置文件夹路径和文件后缀
	std::string folder = "../../data/beta薄款雷达点云"; // 数据文件夹路径
	// std::string folder = "../../data/固态雷达标定数据/20260708多点标定2%和90%靶板/2%+90%"; // 数据文件夹路径
	// std::string folder = "../data/山东点云/20260618山东德州数据/反射率标定车间"; // 数据文件夹路径
	std::string postfix = ".txt";	// 文件后缀，按需求替换
	// std::string postfix = "";	// 文件后缀，按需求替换

	std::vector<float> list_r;
	std::vector<float> list_sa;
	std::vector<std::vector<float>> list_r2; // 固态雷达 距离偏差表
	loadListFromFile(list_r, "../data/param/list_r.txt"); // 读取距离标定参数
	loadListFromFile(list_sa, "../data/param/list_sa.txt"); // 读取水平角标定参数
	loadListFromFile(list_r2, "../data/param/map_single.csv"); // 读取固态雷达距离偏差表
	std::map<uint16_t, std::map<float, std::array<float, 5>>> map_bin;
	binary_storage::loadMapFromBinary("../data/map_device_0812_sum10_extend54.bin", map_bin);
	algo_reflect._map = map_bin; 
	// 读取点云
	std::vector<std::string> filePaths = getFullFilePath(folder, postfix, true);
	std::vector<PointMatrix> list_mat; //
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_cloud; // 存储点云

	for (size_t i = 0; i < filePaths.size(); ++i)
	{
		std::cout << "Processing file: " << filePaths[i] << std::endl;

		std::string filename = filePaths[i];
		// 读取脉冲数据文件
		PointMatrix mat_origin;
		PointMatrix mat_calib;
		// readPulseData(filename, mat_origin); // 全0为无效点
		readLidarData(filename, mat_origin); // 全0为无效点
		// auto mat_2 = buildMatByPulseIdx(mat_origin, 1); // 以第2回波重建点云
		// auto mat_3 = buildMatByPulseIdx(mat_origin, 2); // 以第3回波重建点云

		// // 地面反射点修复
		// algo_gnd.doRepair(mat_origin);
		// 距离和水平角标定
		// mat_src = mat_origin;
		// mat_calib = applyRsaOffset(mat_origin, list_r, list_sa); 
		// mat_calib = applyROffset(mat_origin, list_r2);   
		// 反射率标定
		// algo_reflect.calcCloudIntensity(mat_src);
		// m_intensity_src_name = "z"; // 设置点云强度着色来源为z
		m_intensity_src_name = "peak"; // 设置点云强度着色来源为峰值
		// filtTailPoints(mat_src); // 托尾点滤波
		// m_intensity_src_name = "is_tail_pt"; // 设置点云强度着色来源为 托尾点标记
		// showCloud(mat2xyzi(mat_src, true), "mat_src"); // 可视化
		// list_mat.push_back(mat_src);
		// list_cloud.clear();
		// list_cloud.push_back(mat2pcl(mat_origin));
		// list_cloud.push_back(mat2pcl(mat_2));
		// list_cloud.push_back(mat2pcl(mat_3));
		auto cloud_name = "cloud_" + std::to_string(i);
		showCloud(mat2xyzi(mat_origin, true), cloud_name); // 可视化
		// showCloud(mat2xyzi(mat_calib, true), cloud_name); // 可视化
		// showListCloudRGB(mat2pcl(mat_origin), list_cloud, cloud_name); // 可视化
	}

	// auto mat_json = load_mat_by_json("../data/scene_init.json"); // 读取json文件中的点云
	// showCloud(mat2xyzi(mat_json, true), "mat_json"); // 可视化
	// list_cloud.push_back(mat2pcl(mat_json)); // 将json转换为点云并添加到列表中

    // 初始化用于循环显示4帧点云的实时窗口（RealTimeCloud）
    static pcl::visualization::PCLVisualizer::Ptr viewer_rt;
    if (!viewer_rt)
    {
        viewer_rt.reset(new pcl::visualization::PCLVisualizer("RealTimeCloud"));
        viewer_rt->setBackgroundColor(37/ 255.f, 37/ 255.f, 37/ 255.f);
    }

	while (true)
	{
		// 清除并重新显示点云
		for (size_t j = 0; j < list_cloud.size(); ++j)
		{
			// 跳过索引为1+4*n 这里的1+4*n是为了跳过第1帧和每隔4帧的点云 例如：1, 5, 9, 13, ... 等索引
			// if (j % 4 != 2) // 跳过索引为1+4*n的点云
			// 	continue;
			viewer_rt->removeAllPointClouds();
			pclHelper.addCloud(viewer_rt, list_cloud[j], 0.4f, 0, 2, "cloud_" + std::to_string(j));
			viewer_rt->spinOnce(100);
		}
	}

	// for (size_t i = 0; i < list_mat.size(); ++i)
	// {
	// 	auto cloud_name = "cloud_" + std::to_string(i);
	// 	showCloud(mat2xyzi(list_mat[i], true), cloud_name); // 可视化
	// }
	//showCloud2in1port(mat2pcl(list_mat[0]), mat2pcl(list_mat[1]), "view_result");
	// showCloud4in1port(mat2pcl(list_mat[0]), mat2pcl(list_mat[1]), mat2pcl(list_mat[2]), mat2pcl(list_mat[3]));
	return 0;
}

int main()
{
	// MSVC 设置控制台输出为 UTF-8
	//  SetConsoleOutputCP(CP_UTF8);
	testShowCloud();
	// testConvert(); // 测试debug模拟点云格式转化
}