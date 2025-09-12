/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-02 16:48:05
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-09-12 15:51:44
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
		if (entry.path().extension() == postfix)
		{
			filePaths.push_back(entry.path().string());
		}
	}
	return filePaths;
}

// 解析回波数据，可视化原始点云
int testShowCloud()
{
	PclHelperB pclHelper;
	Ground_I algo_gnd;
	CalibReflect algo_reflect;
	// 设置文件夹路径和文件后缀
	std::string folder = "../data/repair_gnd_high_delay_lab"; // 数据文件夹路径
	std::string postfix = ".txt";	// 文件后缀，按需求替换

	std::vector<float> list_r;
	std::vector<float> list_sa;
	loadListFromFile(list_r, "../data/param/list_r.txt"); // 读取距离标定参数
	loadListFromFile(list_sa, "../data/param/list_sa.txt"); // 读取水平角标定参数
	std::map<uint16_t, std::map<float, std::array<float, 5>>> map_bin;
	binary_storage::loadMapFromBinary("../data/map_device_0812_sum10_extend54.bin", map_bin);
	algo_reflect._map = map_bin; 
	// 读取点云
	std::vector<std::string> filePaths = getFullFilePath(folder, postfix);
	std::vector<PointMatrix> list_mat; //
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_cloud; // 存储点云

	for (size_t i = 0; i < filePaths.size(); ++i)
	{
		std::cout << "Processing file: " << filePaths[i] << std::endl;

		std::string filename = filePaths[i];
		// 读取脉冲数据文件
		PointMatrix mat_origin;
		PointMatrix mat_src;
		readPulseData(filename, mat_origin); // 全0为无效点
		mat_src = applyRsaOffset(mat_origin, list_r, list_sa); // 应用距离和水平角标定参数
		// mat_src = mat_origin;
		// 地面反射点修复
		algo_gnd.doRepair(mat_src);
		// 反射率标定
		algo_reflect.calcCloudIntensity(mat_src);
		// showCloud(mat2xyzi(mat_src, true), "mat_src"); // 可视化
		list_mat.push_back(mat_src);
		list_cloud.push_back(mat2pcl(mat_src));
		auto cloud_name = "cloud_" + std::to_string(i);
		showCloud(mat2xyzi(list_mat[i], true), cloud_name); // 可视化
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