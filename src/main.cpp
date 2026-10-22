/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-02 16:48:05
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2026-10-22 10:49:54
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

// 获取文件夹中所有符合后缀的文件路径（非递归，兼容旧调用）
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

// 判断是否为 FL6031 单算法 Frame CSV（由 6031_直方图算单帧Csv_多版算法选一个.py 生成）。
// 特征：表头含 center_bin / timing_bin / algorithm / echo_bins，且不含旧版前缀列。
bool isFrameCsv6031(const std::string &filename)
{
	std::ifstream input(filename);
	std::string header;
	if (!std::getline(input, header))
		return false;

	// 新版 CSV 的判别依据（同时存在才算 6031 Frame CSV）
	return header.find("center_bin") != std::string::npos &&
	       header.find("timing_bin") != std::string::npos &&
	       header.find("algorithm") != std::string::npos &&
	       header.find("echo_bins") != std::string::npos;
}

// 6532 Frame CSV 由 range 模式解析工程输出，包含 roi/行列/回波原始距离字段。
bool isFrameCsv6532(const std::string &filename)
{
	std::ifstream input(filename);
	std::string header;
	if (!std::getline(input, header))
		return false;
	return header.find("roi") != std::string::npos &&
	       header.find("行row") != std::string::npos &&
	       header.find("列col") != std::string::npos &&
	       header.find("p0_dist_rp_raw") != std::string::npos;
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
	std::string folder = "../../data/固态雷达标定数据/6532标定算法效果对比"; // 数据文件夹路径
	// std::string folder = "../../algo_solid_lidar/data/固态雷达range模式data/6031_range_csv_output_multi_algorithm"; // 数据文件夹路径
	// std::string folder = "../../data/固态雷达标定数据/6031直方图算法效果对比"; // 数据文件夹路径
	// std::string folder = "/mnt/windows_share/0_标定_bai/项目_固态雷达/6031直方图和点云/20260918走廊26米靶板只有几个点r75c166";
	// std::string folder = "/mnt/windows_share/0_标定_bai/项目_薄款雷达/薄款雷达_原始点云/20260917薄款雷达茶水间大楼点云异常/frame_8";
	// std::string folder = "../../data/beta薄款雷达点云问题分析/2026-08-19何工适配800线版本在工位"; // 数据文件夹路径
	// std::string folder = "../data/地面修复场景"; // 数据文件夹路径 
	// std::string postfix = ".txt";	// 文件后缀，按需求替换 
	// std::string postfix = ".pcd";
	// std::string postfix = ".csv";
	std::string postfix = "";
	// FL6031 Frame CSV 的算法由文件自身的 algorithm 列标识，加载时无需再指定。

	// 6532 老版 ROI0 逐像素标定表；只在确认输入为 6532 Frame CSV 时应用。
	const std::string calibration_csv =
		"../../algo_solid_lidar/build/calib_data_legacy_parse/roi0_pixel_tables.csv";
	Range6532Roi0PixelCalibration calibration_table;
	const Range6532TimeWalkModel time_walk_model{}; // enabled=false 可只观察 table_b 偏置效果。
	bool has_6532_calibration = false;
	bool tried_6532_calibration = false;

	std::vector<float> list_r;
	std::vector<float> list_sa;
	std::vector<std::vector<float>> list_r2; // 固态雷达 距离偏差表
	loadListFromFile(list_r, "../data/param/list_r.txt"); // 读取距离标定参数

	loadListFromFile(list_sa, "../data/param/list_sa.txt"); // 读取水平角标定参数
	loadListFromFile(list_r2, "../data/param/map_single.csv"); // 读取固态雷达距离偏差表
	std::map<uint16_t, std::map<float, std::array<float, 5>>> map_bin;
	binary_storage::loadMapFromBinary("../data/map_device_0812_sum10_extend54.bin", map_bin);
	algo_reflect._map = map_bin;
	// 读取点云（同时支持 .pcd 与 Frame*.csv 原始点云）
	std::vector<std::string> filePaths = getFullFilePath(folder, postfix, false);
	// std::vector<std::string> filePaths;
	// {
	// 	auto v_pcd = getFullFilePath(folder, ".pcd", false);
	// 	auto v_csv = getFullFilePath(folder, ".csv", false);
	// 	filePaths = v_pcd;
	// 	filePaths.insert(filePaths.end(), v_csv.begin(), v_csv.end());
	// }
	std::vector<PointMatrix> list_mat; //
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_cloud; // 存储点云

	for (size_t i = 0; i < filePaths.size(); ++i)
	{
		// 跳过 .~ 开头的临时文件
		if (fs::path(filePaths[i]).filename().string().find(".~") == 0)
		{
			continue;
		}
		std::cout << "Processing file: " << filePaths[i] << std::endl;

		std::string filename = filePaths[i];
		const bool is_6031_frame_csv =
			fs::path(filename).extension() == ".csv" && isFrameCsv6031(filename);
		const bool is_6532_frame_csv =
			fs::path(filename).extension() == ".csv" && isFrameCsv6532(filename);
		if (is_6532_frame_csv && !tried_6532_calibration)
		{
			has_6532_calibration =
				loadRange6532Roi0PixelCalibrationCsv(calibration_csv, calibration_table);
			tried_6532_calibration = true;
		}
		// 读取脉冲数据文件
		PointMatrix mat_origin, mat_roi, mat_calib, mat_p0, mat_p1, mat_pcd, mat_v0, mat_v1, mat_v2, mat_v3;
		if (fs::path(filename).extension() == ".csv")
		{
			if (is_6031_frame_csv)
			{
				// 6031 单算法 CSV：一次加载即可；如需对比不同算法，
				// 分别加载对应算法的 CSV 文件，而不是在同一文件上切换 method。
				readFrameCsv_6031(filename, mat_origin);

			}
			else
				readFrameCsv(filename, mat_origin);
			// readPcdData(filename + ".pcd", mat_pcd);
			// readFrameCsv(filename, mat_p0, "p0_only");
			// readFrameCsv(filename, mat_p1, "p1_only");
		}
		else if (fs::path(filename).extension() == ".pcd")
		{
			readPcdData(filename, mat_origin); // 读pcd点云
		}
		else
		{
			// readPulseData(filename, mat_origin); // 全0为无效点
			readLidarData(filename, mat_origin);
		}

		// mat_p0 = buildMatByPulseIdx(mat_origin, 0); // 以第1回波重建点云
		// mat_p1 = buildMatByPulseIdx(mat_origin, 1); // 以第2回波重建点云

		// // 地面反射点修复
		// mat_src = mat_origin;
		// algo_gnd.doRepair(mat_src);
		// 距离和水平角联合标定
		// mat_calib = applyRsaOffset(mat_src, list_r, list_sa);
		// 6532 模组标定
		if (is_6031_frame_csv)
		{
			mat_calib = applyROffset(mat_origin, list_r2);
		}
		else if (is_6532_frame_csv && has_6532_calibration)
		{
			mat_calib = mat_origin;
			applyRange6532Roi0PixelCalibration(mat_calib, calibration_table, time_walk_model);
		}
		else
		{
			mat_calib = applyROffset(mat_origin, list_r2);
		}
		// 反射率标定
		// algo_reflect.calcCloudIntensity(mat_calib);
		m_intensity_src_name = "r"; // 设置点云强度着色来源为r
		// m_intensity_src_name = "z"; // 设置点云强度着色来源为z
		// m_intensity_src_name = "peak"; // 设置点云强度着色来源为峰值
		// m_intensity_src_name = "timestamp"; // 设置点云强度着色来源为时间戳
		// m_intensity_src_name = "intensity"; // 设置点云强度着色来源为标定后的反射率
		// filtTailPoints(mat_src); // 托尾点滤波
		// m_intensity_src_name = "is_tail_pt"; // 设置点云强度着色来源为 托尾点标记
		// showCloud(mat2xyzi(mat_src, true), "mat_src"); // 可视化
		// list_mat.push_back(mat_src);
		// list_cloud.clear();
		// list_cloud.push_back(mat2pcl(mat_origin));
		// list_cloud.push_back(mat2pcl(mat_p0));
		// list_cloud.push_back(mat2pcl(mat_p1));
		auto cloud_name = "cloud_" + std::to_string(i);

		// if(mat_v0.size() > 0)
		// {
		// 	showCloud4(mat2xyzi(mat_v0), mat2xyzi(mat_v1), mat2xyzi(mat_v2), mat2xyzi(mat_v3)); // 可视化
		// 	continue; // 跳过后续处理，直接进入下一文件
		// }

		// 按行列截取矩阵数据
		mat_roi =  extractData(mat_origin, 0, 150, 100, 250);
		// 转换为 pcl 点云
		auto cloud_pcl = mat2xyzi(mat_roi, true);
		// 快速欧式聚类分割
		float clusterTolerance = 0.5f;
		std::vector<pcl::PointIndices> cluster_fec;
		pclHelper.fastEuclideanClustering(cloud_pcl, cluster_fec, clusterTolerance);
		// 调用 getsubcloud
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_main(new pcl::PointCloud<pcl::PointXYZI>);
		pclHelper.getSubCloud(cloud_pcl, cluster_fec[0], cloud_main);
		// showCloud(cloud_main, cloud_name + "_main"); // 可视化

		// 行列同时取反
		// algo_lidar::flipMatInPlace(mat_origin, true, true);
		showCloud(mat2xyzi(mat_origin, true), cloud_name + "_origin"); // 可视化
		if (is_6532_frame_csv && has_6532_calibration)
			showCloud(mat2xyzi(mat_calib, true), cloud_name + "_calib"); // 6532 标定后点云
		// showCloud2(mat2xyzi(mat_origin, false), mat2xyzi(mat_calib,true),cloud_name + "_origin_calib"); // 可视化
		// showCloud3(mat2xyzi(mat_origin), mat2xyzi(mat_calib,true), mat2xyzi(mat_pcd), cloud_name + "_origin;calib;pcd"); // 可视化
		// showCloud2in1port(mat2pcl(mat_p0), mat2pcl(mat_p1, true), cloud_name + "_red:echo_0 ; blue:echo_1"); // 可视化
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

int main(int argc, char **argv)
{
	// MSVC 设置控制台输出为 UTF-8
	// SetConsoleOutputCP(CP_UTF8);

	// 直接打开 FL6031 单算法 Frame CSV：
	//     PCL_VIEWER --fl6031-csv Frame6031.csv [--validate-only]
	// CSV 内的 algorithm 列标明所使用的算法；加 --validate-only 可只验证解析/XYZ，
	// 不打开 GUI。
	if (argc >= 3 && std::string(argv[1]) == "--fl6031-csv")
	{
		const std::string filename = argv[2];
		bool validate_only = false;
		for (int i = 3; i < argc; ++i)
		{
			const std::string arg = argv[i];
			if (arg == "--validate-only")
				validate_only = true;
		}

		PointMatrix mat;
		if (!readFrameCsv_6031(filename, mat))
			return 2;
		const auto cloud = mat2xyzi(mat, true);
		std::cout << "FL6031 XYZ cloud: " << cloud->size()
		          << " points" << std::endl;
		if (validate_only)
			return 0;
		showCloud(cloud, "FL6031");
		return 0;
	}

	testShowCloud();
	// testConvert(); // 测试debug模拟点云格式转化
}
