/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-02 16:48:05
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-04-17 10:28:58
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
#include <alog_interface.h>

using namespace alog_lidar;
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
	// 设置文件夹路径和文件后缀
	std::string folder = "../data"; // 数据文件夹路径
	std::string postfix = ".txt";	// 文件后缀，按需求替换

	std::vector<float> list_r;
	std::vector<float> list_sa;
	loadListFromFile(list_r, "../data/param/list_r.txt"); // 读取距离标定参数
	loadListFromFile(list_sa, "../data/param/list_sa.txt"); // 读取水平角标定参数

	// 读取点云
	std::vector<std::string> filePaths = getFullFilePath(folder, postfix);
	std::vector<PointMatrix> list_mat; //
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> list_cloud; // 存储点云
	std::vector<int> list_slot_idx; // 全部列索引
	for (size_t i = 0; i < 800; ++i)
	{
		list_slot_idx.push_back(i);
	}
	for (size_t i = 0; i < filePaths.size(); ++i)
	{
		std::cout << "Processing file: " << filePaths[i] << std::endl;

		std::string filename = filePaths[i];
		// 读取脉冲数据文件
		PointMatrix mat_origin;
		PointMatrix mat_src;
		readPulseData(filename, mat_origin); // 全0为无效点
		// mat_src = applyRsaOffset(mat_origin, list_r, list_sa); // 应用距离和水平角标定参数
		mat_src = mat_origin;
		// showCloud(mat2xyzi(mat_src, true), "mat_src"); // 可视化
		list_mat.push_back(mat_src);
		list_cloud.push_back(mat2xyzi(mat_src));
	}
	for (size_t i = 0; i < list_mat.size(); ++i)
	{
		auto cloud_name = "cloud_" + std::to_string(i);
		showCloud(mat2xyzi(list_mat[i], true), cloud_name); // 可视化
	}
	//showCloud2in1port(mat2pcl(list_mat[0]), mat2pcl(list_mat[1]), "view_result");
	// showCloud4in1port(mat2pcl(list_mat[0]), mat2pcl(list_mat[1]), mat2pcl(list_mat[2]), mat2pcl(list_mat[3]));
	return 0;
}
// 解析回波数据3，可视化原始点云
void testShowCloud3()
{
	PclHelperB pclHelper;
	// 设置文件夹路径和文件后缀
	std::string folder = "../data"; // 数据文件夹路径
	std::string postfix = ".txt";	// 文件后缀，按需求替换

	std::vector<float> list_r;
	std::vector<float> list_sa;
	loadListFromFile(list_r, "../data/param/list_r.txt"); // 读取距离标定参数
	loadListFromFile(list_sa, "../data/param/list_sa.txt"); // 读取水平角标定参数

	// 读取点云
	std::vector<std::string> filePaths = getFullFilePath(folder, postfix);

	for (size_t i = 0; i < filePaths.size(); ++i)
	{
		std::cout << "Processing file: " << filePaths[i] << std::endl;

		std::string filename = filePaths[i];
		// 读取脉冲数据文件
		PointMatrix mat_src;
		
		PointMatrix3 mat_3;
		readPulseData(filename, mat_src); // 全0为无效点
		readPulseData3(filename, mat_3); // 全0为无效点
		// mat_src = applyRsaOffset(mat_src, list_r, list_sa); // 应用距离和水平角标定参数
		// mat_src = applyROffset(mat_origin, list_r);
		showCloud2(mat2xyzi(mat_src, true), mat3toxyzi(mat_3, true), "mat_src"); // 可视化
		showCloud(mat3toxyzi(mat_3, true), "mat_src_3"); // 可视化

	}
	return ;
}

int main()
{
	// MSVC 设置控制台输出为 UTF-8
	//  SetConsoleOutputCP(CP_UTF8);
	testShowCloud();
	// testShowCloud3();
	// testConvert(); // 测试debug模拟点云格式转化
}