/*
 * @Author       : baizs zhongshan.bai@vitalchem.com
 * @Date         : 2024-11-21 09:44:32
 * @LastEditors  : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-03-03 13:41:50
 * @FilePath     : /pcl_viewer/include/PclHelperB.h
 * @Description  : PCL库算法助手
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */

#pragma once
#include <iostream>
#include <thread>
// Boost Library
#include <boost/make_shared.hpp>

// VTK Library
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>

// PCL - Common
#include <pcl/common/colors.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
// PCL - Features
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

// PCL - Filters
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
// PCL - IO
#include <pcl/io/pcd_io.h>

// PCL - Model
#include <pcl/ModelCoefficients.h>
// PCL - Point Cloud
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// PCL - Search
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
// PCL - Segmentation
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
// PCL - Visualization
#include <pcl/visualization/common/shapes.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

// 定义 POINTSWITHINDEX 宏以启用索引支持
#define POINTSWITHINDEX
class PclHelperB
{
private:
	// 定义点云类型模板
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointXYZLNormal NormalT;
	pcl::console::TicToc tt; // 测耗时

public:
#pragma region 读写操作和格式转换
	// 簇排序
	void sortClusters(std::vector<pcl::PointIndices> &cluster, bool largeFirst = true)
	{
		// 定义比较函数
		auto compareSize = [largeFirst](const pcl::PointIndices &a, const pcl::PointIndices &b)
		{
			return largeFirst ? a.indices.size() > b.indices.size() : a.indices.size() < b.indices.size();
		};
		// 使用比较函数对cluster进行排序
		std::sort(cluster.begin(), cluster.end(), compareSize);
	}

	// 保存点云
	void savePcd(pcl::PointCloud<PointT>::Ptr cloud, std::string filename = "cloud_latest.pcd")
	{
		if (cloud->points.size() == 0)
		{
			std::cerr << filename << "的size为0，无法保存点云\n";
			return;
		}
		// 检查文件是否已存在
		std::ifstream ifile(filename); // 尝试打开名为filename的文件
		if (ifile)
		{
			// 如果文件存在(能被打开)，则删除它
			ifile.close();				   // 删除前必须首先关闭文件
			std::remove(filename.c_str()); // 删除后，再保存。达到替换的目的
		}
		// 如果点云的宽度或高度为0，那么设置宽度和高度
		if (cloud->width == 0 || cloud->height == 0)
		{
			cloud->width = cloud->points.size();
			cloud->height = 1;
		}
		// 保存点云为PCD文件
		// pcl::io::savePCDFile(filename, *cloud); // 读写慢
		pcl::PCDWriter writer;
		writer.writeBinary(filename, *cloud); // 读写快
	}
	// 保存点云
	void savePcd(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string filename = "cloud_latest.pcd")
	{
		if (cloud->points.size() == 0)
		{
			std::cerr << filename << "的size为0，无法保存点云\n";
			return;
		}
		// 检查文件是否已存在
		std::ifstream ifile(filename); // 尝试打开名为filename的文件
		if (ifile)
		{
			// 如果文件存在(能被打开)，则删除它
			ifile.close();				   // 删除前必须首先关闭文件
			std::remove(filename.c_str()); // 删除后，再保存。达到替换的目的
		}
		// 如果点云的宽度或高度为0，那么设置宽度和高度
		if (cloud->width == 0 || cloud->height == 0)
		{
			cloud->width = cloud->points.size();
			cloud->height = 1;
		}
		// 保存点云为PCD文件
		// pcl::io::savePCDFile(filename, *cloud); // 读写慢
		pcl::PCDWriter writer;
		writer.writeBinary(filename, *cloud); // 读写快
	}
	// 0f~1.0f的hsv转成0f~1.0f的rgb
	void hsv2rgb(float h, float s, float v, float *r, float *g, float *b)
	{
		float f, x, y, z;
		int i;
		if (s == 0.0)
		{
			*r = *g = *b = v;
		}
		else
		{
			h = fmod(h * 360.0, 360.0) / 60.0;
			i = (int)h;
			f = h - i;
			x = v * (1.0 - s);
			y = v * (1.0 - (s * f));
			z = v * (1.0 - (s * (1.0 - f)));
			switch (i)
			{
			case 0:
				*r = v;
				*g = z;
				*b = x;
				break;
			case 1:
				*r = y;
				*g = v;
				*b = x;
				break;
			case 2:
				*r = x;
				*g = v;
				*b = z;
				break;
			case 3:
				*r = x;
				*g = y;
				*b = v;
				break;
			case 4:
				*r = z;
				*g = x;
				*b = v;
				break;
			case 5:
				*r = v;
				*g = x;
				*b = y;
				break;
			}
		}
	}
#pragma endregion

#pragma region 通用算法
	// 模型异常值剔除  (从点云中提取与模型距离小于阈值的点)
	void removeModelOutlier(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::SacModel mode, const pcl::ModelCoefficients::Ptr coeff, float distanceThreshold)
	{
		std::cerr << "\n模型异常值剔除...\n", tt.tic();
		float radius = 0.5f;														 // 请根据你的实际需求设置法向计算的半径
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // 法向
		computeNormals(cloud, normals, radius);										 // 计算法向

		pcl::ModelOutlierRemoval<PointT> filter;
		filter.setModelCoefficients(*coeff);
		filter.setThreshold(distanceThreshold);
		filter.setModelType(mode);
		filter.setInputCloud(cloud);
		filter.setInputNormals(normals);
		filter.filter(*cloud_filtered);
		std::cerr << ">> Done: " << tt.toc() << " ms, 模型异常值剔除后剩余：" << cloud_filtered->points.size() << " points\n";
	}

	// 已知两个向量分别为n1和n2，求旋转矩阵
	Eigen::Matrix4f getRotationMatrix(const Eigen::Vector3f &n1, const Eigen::Vector3f &n2)
	{
		// 计算旋转轴
		Eigen::Vector3f axis = n1.cross(n2);

		// 计算旋转角度
		float cos_angle = n1.dot(n2) / (n1.norm() * n2.norm());
		float angle = acos(cos_angle);

		// 构造旋转矩阵
		Eigen::AngleAxisf rotation(angle, axis.normalized());
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
		transform.block<3, 3>(0, 0) = rotation.matrix();
		return transform;
	}
	// 检查平面法向量差异是否小于阈值
	bool checkNormalSimilar(const pcl::ModelCoefficients::Ptr &coeff1,
							const pcl::ModelCoefficients::Ptr &coeff2,
							float diffThreshold)
	{
		// 确保 coeff1 和 coeff2 都有有效的法线信息
		if (coeff1->values.size() < 3 || coeff2->values.size() < 3)
		{
			std::cerr << "平面模型系数不完整。\n";
			return false;
		}

		// 提取两个平面的法向量
		Eigen::Vector3f normal1(coeff1->values[0], coeff1->values[1], coeff1->values[2]);
		Eigen::Vector3f normal2(coeff2->values[0], coeff2->values[1], coeff2->values[2]);

		// 计算法向量之间的余弦值
		float cos_angle = normal1.normalized().dot(normal2.normalized());

		// 将余弦值转换为角度（0~180度）
		float angle_diff = std::acos(std::min(std::max(cos_angle, -1.0f), 1.0f)) * (180.0 / M_PI);

		// 检查角度差异是否小于阈值
		return angle_diff < diffThreshold;
	}

	// 区域生长分割
	void regionGrowingSegmentation(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointIndices> &clusters)
	{
		std::cerr << "\n区域生长分割...\n", tt.tic();
		// 建立搜索KD树
		pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
		// 计算点云法向
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimator;
		// pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(tree); // 搜索方法为kd树走索
		normal_estimator.setInputCloud(cloud);	// 填入点云
		normal_estimator.setKSearch(30);		// 设置搜索范围
		int num = std::thread::hardware_concurrency();
		normal_estimator.setNumberOfThreads(num); // 线程数
		// normal_estimator.setRadiusSearch(0.1);        // 设置搜索范围
		normal_estimator.compute(*normals); // 将法相保存在normals
		pcl::IndicesPtr indices(new std::vector<int>);
		pcl::removeNaNFromPointCloud(*cloud, *indices); // 对点云建立索引
		pcl::RegionGrowing<PointT, pcl::Normal> reg;	// 区域增长类
		size_t min_size = cloud->points.size() / 1000 > 100 ? 100 : cloud->points.size() / 1000;
		reg.setMinClusterSize(min_size);			 // 设置过小聚类的标准
		reg.setMaxClusterSize(cloud->points.size()); // 设置过大聚类的标准
		reg.setSearchMethod(tree);					 // 设置kd树搜索方法

		reg.setNumberOfNeighbours(30);					// 设置每次邻域搜索数(影响计算速度)
		reg.setInputCloud(cloud);						// 设置输入点云
		reg.setIndices(indices);						// 设置输入的索引
		reg.setInputNormals(normals);					// 设置输入法向
		reg.setSmoothnessThreshold(6.0 / 180.0 * M_PI); // 设置平滑度阈值（弧度） 两个相邻点之间的法向量夹角的最大值
		reg.setCurvatureThreshold(0.1);					// 设置曲率阈值 每个点的曲率的最大值 （达到阈值则为簇群边界点）
		reg.extract(clusters);
		// 族排序 size大的在前面
		sortClusters(clusters);
		std::cerr << ">> Done: " << tt.toc() << " ms, 提取到 " << clusters.size() << " 个簇\n";
	}
	// 过点沿z轴方向的直线与点云的最近点 nkn
	PointT getNearestPlanePointNKN(PointT point, pcl::PointCloud<PointT>::Ptr cloud)
	{
		// 创建二维点云
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXY>);
		for (const auto &p : cloud->points)
		{
			pcl::PointXY p2d;
			p2d.x = p.x;
			p2d.y = p.y;
			cloud_2d->points.push_back(p2d);
		}

		// 创建二维查询点
		pcl::PointXY point_2d;
		point_2d.x = point.x;
		point_2d.y = point.y;

		// 创建k-d树
		pcl::KdTreeFLANN<pcl::PointXY> kdtree;
		kdtree.setInputCloud(cloud_2d);

		// 执行最近邻搜索
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		if (kdtree.nearestKSearch(point_2d, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			return cloud->points[pointIdxNKNSearch[0]];
		}
		else
		{
			throw std::runtime_error("No nearest neighbor found , err at getNearestPlanePoint ");
		}
	}

	// 过点沿z轴方向的直线与平面的交点
	PointT getNearestPlanePoint(PointT point, pcl::ModelCoefficients::Ptr coeff)
	{
		// 平面方程为 ax + by + cz + d = 0
		// 系数 a, b, c, d 分别为 coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3]
		// 直线方程为 x = point.x, y = point.y, z = t
		// 将直线方程代入平面方程，解出 t
		float t = -(coeff->values[0] * point.x + coeff->values[1] * point.y + coeff->values[3]) / coeff->values[2];

		// 创建一个新的点，它的x和y坐标与输入点相同，但z坐标为t
		PointT intersection_point;
		intersection_point.x = point.x;
		intersection_point.y = point.y;
		intersection_point.z = t;

		return intersection_point;
	}

	// 计算点到平面的距离 有正负
	float computePointToPlaneDistance(const PointT point, const pcl::ModelCoefficients::Ptr &plane_coefficients)
	{
		// 距离值含正负  代表在平面的正侧还是反侧
		float distance = (point.x * plane_coefficients->values[0] +
						  point.y * plane_coefficients->values[1] +
						  point.z * plane_coefficients->values[2] +
						  plane_coefficients->values[3]) /
						 std::sqrt(
							 std::pow(plane_coefficients->values[0], 2) +
							 std::pow(plane_coefficients->values[1], 2) +
							 std::pow(plane_coefficients->values[2], 2));
		;
		return distance;
	}
	// 计算两点之间的三维距离
	float computePointDistance3d(const PointT point1, const PointT point2)
	{
		float distance = std::sqrt(
			std::pow(point2.x - point1.x, 2) +
			std::pow(point2.y - point1.y, 2) +
			std::pow(point2.z - point1.z, 2));
		return distance;
	}
	// 已知一个平面上的三维点和该平面的法向量，求该平面方程系数
	void computePlaneCoeff(const PointT &point, const Eigen::Vector3f &normal, pcl::ModelCoefficients::Ptr &plane_coefficients)
	{
		Eigen::Vector3f unit_normal = normal.normalized();
		plane_coefficients->values.resize(4);
		plane_coefficients->values[0] = unit_normal.x();
		plane_coefficients->values[1] = unit_normal.y();
		plane_coefficients->values[2] = unit_normal.z();
		plane_coefficients->values[3] = -(unit_normal.x() * point.x + unit_normal.y() * point.y + unit_normal.z() * point.z);

		std::cout << "Plane coefficients: ";
		for (const float value : plane_coefficients->values)
		{
			std::cout << value << " ";
		}
		std::cout << std::endl;
	}
	// 计算簇内中位数点
	void computeClusterMedian(pcl::PointCloud<PointT>::Ptr points_3d, pcl::IndicesClustersPtr cluster_euc, pcl::PointCloud<PointT>::Ptr cloud_median)
	{
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_euc->begin(); it != cluster_euc->end(); ++it)
		{
			std::vector<float> xs, ys, zs;
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			{
				PointT point = points_3d->points[*pit];
				xs.push_back(point.x);
				ys.push_back(point.y);
				zs.push_back(point.z);
			}

			std::sort(xs.begin(), xs.end());
			std::sort(ys.begin(), ys.end());
			std::sort(zs.begin(), zs.end());

			float median_x = xs[xs.size() / 2];
			float median_y = ys[ys.size() / 2];
			float median_z = zs[zs.size() / 2];

			PointT median_point;
			median_point.x = median_x;
			median_point.y = median_y;
			median_point.z = median_z;

			cloud_median->points.push_back(median_point);
		}
	}
	// 点云投影
	void projectCloudToPlane(pcl::PointCloud<PointT>::Ptr &cloud, pcl::ModelCoefficients::Ptr &plane_coefficients, pcl::PointCloud<PointT>::Ptr &cloud_projected)
	{
		std::cerr << "\n投影...\n", tt.tic();
		// 创建滤波器对象
		pcl::ProjectInliers<PointT> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud);
		proj.setModelCoefficients(plane_coefficients);
		proj.filter(*cloud_projected);
		cloud_projected->width = cloud_projected->size();
		std::cerr << ">> Done: " << tt.toc() << " ms\n";
	}

	// 倒置点云
	void reverseCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
	{
		Eigen::Vector3f normal_z(0.0f, 0.0f, 1.0f);				 // 标准Z向量
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(); // 创建一个旋转矩阵，用于将点云旋转180度
		transform(1, 1) = -1;									 // 在y轴上旋转180度
		transform(2, 2) = -1;									 // 在z轴上旋转180度
		pcl::transformPointCloud(*cloud, *cloud, transform);	 // 旋转
	}
	// 倒置点云
	void reverseCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
	{
		Eigen::Vector3f normal_z(0.0f, 0.0f, 1.0f);					// 标准Z向量
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();	// 创建一个旋转矩阵，用于将点云旋转180度
		transform(1, 1) = -1;										// 在y轴上旋转180度
		transform(2, 2) = -1;										// 在z轴上旋转180度
		pcl::transformPointCloud(*cloud_in, *cloud_out, transform); // 旋转
	}
	// 计算点云尺度
	float computeScale(const pcl::PointCloud<PointT>::Ptr &cloud)
	{
		Eigen::Vector4f min_pt, max_pt;
		pcl::getMinMax3D(*cloud, min_pt, max_pt);
		Eigen::Vector4f diff = max_pt - min_pt;
		float scale = std::max(std::max(diff[0], diff[1]), diff[2]);
		return scale;
	}
	// 根据索引，提取子点云
	void getSubCloud(const pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointIndicesPtr &ids, pcl::PointCloud<PointT>::Ptr &sub_cloud)
	{
		// 创建ExtractIndices过滤器对象
		pcl::ExtractIndices<PointT> extract;
		// 设置要提取的点云索引
		extract.setInputCloud(cloud);

		pcl::IndicesPtr indices(new std::vector<int>(ids->indices));

		extract.setIndices(indices);
		// 调用filter函数提取子集
		extract.filter(*sub_cloud);
	}
	// 重载版本，直接支持pcl::PointIndices类型（自动转换为pcl::PointIndicesPtr）
	void getSubCloud(const pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointIndices &ids, pcl::PointCloud<PointT>::Ptr &sub_cloud)
	{
		// 将pcl::PointIndices转换为pcl::PointIndicesPtr
		pcl::PointIndicesPtr ids_ptr(new pcl::PointIndices(ids));

		// 调用原有的getSubCloud函数
		getSubCloud(cloud, ids_ptr, sub_cloud);
	}
	// 根据索引，提取子点云
	void getSubCloud(pcl::PointCloud<PointT>::Ptr &cloud, pcl::IndicesPtr ids, pcl::PointCloud<PointT>::Ptr &sub_cloud)
	{
		// 创建ExtractIndices过滤器对象
		pcl::ExtractIndices<PointT> extract;
		// 设置要提取的点云索引
		extract.setInputCloud(cloud);
		extract.setIndices(ids);
		// 调用filter函数提取子集
		extract.filter(*sub_cloud);
	}
	void getSubCloud(
		pcl::PointCloud<PointT>::Ptr &cloud,
		const std::vector<size_t> &target_indices,
		pcl::PointCloud<PointT>::Ptr &sub_cloud)
	{
		// 创建一个 pcl::PointIndices 对象来存储索引
		pcl::PointIndices::Ptr indices(new pcl::PointIndices);

		// 将 std::vector<size_t> 转换为 pcl::PointIndices
		for (size_t index : target_indices)
		{
			indices->indices.push_back(static_cast<int>(index)); // 转换为 int 类型
		}

		// 调用原始的 getSubCloud 方法，传入转换后的 pcl::PointIndices
		getSubCloud(cloud, indices, sub_cloud);
	}

	// 按照法向方向过滤点云
	void filterByNormalDirection(pcl::PointCloud<NormalT>::Ptr &normals, Eigen::Vector3f target_direction, pcl::IndicesPtr &positive_idx, pcl::IndicesPtr &negative_idx, pcl::IndicesPtr &all_idx, float cos_angle_threshold = 0.5)
	{
		for (size_t i = 0; i < normals->size(); ++i)
		{
			Eigen::Vector3f normal(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);

			// 计算法向量和目标方向之间的角度的余弦值 -1~1
			float cos_angle = normal.normalized().dot(target_direction.normalized());

			if (cos_angle > cos_angle_threshold)
			{
				positive_idx->push_back(i);
				all_idx->push_back(i);
			}

			if (cos_angle < -cos_angle_threshold)
			{
				negative_idx->push_back(i);
				all_idx->push_back(i);
			}
		}
	}
	// 阈值滤波器
	void passThroughFilter(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, float z_min, float z_max)
	{
		// 创建过滤器
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(z_min, z_max);
		pass.setKeepOrganized(false);

		// 应用过滤器
		pass.filter(*cloud_filtered);
	}
	// 计算点云法向
	void computeNormals(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float radius)
	{
		// std::cerr << "计算法向:" << cloud->points.size() << " points ...\n", tt.tic();
		// 建立搜索KD树
		pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
		// 计算点云法向
		pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(tree); // 搜索方法为kd树走索
		normal_estimator.setInputCloud(cloud);	//
		normal_estimator.setViewPoint(0, 0, std::numeric_limits<float>::max());
		normal_estimator.setKSearch(30); // 设置搜索范围
		// normal_estimator.setRadiusSearch(radius);//设置搜索半径大小
		int num = std::thread::hardware_concurrency();
		normal_estimator.setNumberOfThreads(num); // 线程数
		normal_estimator.compute(*normals);		  // 将法相保存在normals
												  // std::cerr << ">> Done: " << tt.toc() << " ms\n"; //计算法线估计所使用的时间
	}

	// 求子点云在原点云中的索引（kdtree）
	void findIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_region, std::vector<int> &indices)
	{
		std::cerr << "\n计算在源点云中的索引...\n", tt.tic();
		// 创建一个KdTreeFLANN对象
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(cloud_src);

		// 遍历子点云，找到每个点在源点云中的索引
		for (const auto &point : cloud_region->points)
		{
			std::vector<int> pointIdxNKNSearch(1);
			std::vector<float> pointNKNSquaredDistance(1);

			if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				indices.push_back(pointIdxNKNSearch[0]);
			}
		}
		std::cerr << ">> Done: " << tt.toc() << " ms\n";
	}
	// 求子点云在原点云中的索引 （哈希表）
	void findIndices_Hashtable(const pcl::PointCloud<PointT>::Ptr cloud_src, const pcl::PointCloud<PointT>::Ptr cloud_region, std::vector<int> &indices)
	{
		// 创建一个哈希表来存储源点云中每个点的索引
		std::unordered_map<std::string, int> src_map;
		for (size_t i = 0; i < cloud_src->points.size(); ++i)
		{
			std::string point_str = std::to_string(cloud_src->points[i].x) + "-" +
									std::to_string(cloud_src->points[i].y) + "-" +
									std::to_string(cloud_src->points[i].z);
			src_map[point_str] = i;
		}

		// 遍历子点云，找到每个点在源点云中的索引
		for (const auto &point : cloud_region->points)
		{
			std::string point_str = std::to_string(point.x) + "-" +
									std::to_string(point.y) + "-" +
									std::to_string(point.z);
			if (src_map.count(point_str))
			{
				indices.push_back(src_map[point_str]);
			}
		}
	}

	// 欧式聚类分割
	void euclideanClustering(const pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointIndices> &cluster_indices, float tolerance = 0.1)
	{
		std::cerr << "\n欧氏聚类：" << cloud->points.size() << "个点...\n", tt.tic();
		// 创建 KdTree 对象
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
		tree->setInputCloud(cloud);
		// 创建欧式聚类分割对象
		pcl::EuclideanClusterExtraction<PointT> ec;
		ec.setClusterTolerance(tolerance);
		size_t min_size = cloud->points.size() / 1000 > 100 ? 100 : cloud->points.size() / 1000;
		ec.setMinClusterSize(min_size);
		ec.setMaxClusterSize(cloud->points.size());
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);

		// 执行欧式聚类分割
		ec.extract(cluster_indices);
		std::cerr << ">> Done: " << tt.toc() << " ms, 提取到 " << cluster_indices.size() << " 个簇\n";
	}

	// 从簇群中找出已知点所在的簇
	void findClusterContainingPoint(const PointT &point, const pcl::PointCloud<PointT>::Ptr &region, const std::vector<pcl::PointIndices> &clusters, pcl::PointCloud<PointT>::Ptr &plane)
	{
		std::cerr << "\n从簇群中找出已知点所在的簇...\n", tt.tic();
		int containing_cluster_index = -1;

		for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index)
		{
			const pcl::PointIndices &cluster_indices = clusters[cluster_index];
			for (const int index : cluster_indices.indices)
			{
				const PointT &cluster_point = region->points[index];
				if (cluster_point.x == point.x && cluster_point.y == point.y && cluster_point.z == point.z)
				{
					containing_cluster_index = static_cast<int>(cluster_index);
					break;
				}
			}

			if (containing_cluster_index != -1)
			{
				// Extract the indices of the points in the specified cluster
				const pcl::PointIndices &selected_indices = clusters[containing_cluster_index];

				// Create a new point cloud containing the points in the cluster
				plane.reset(new pcl::PointCloud<PointT>);
				pcl::copyPointCloud(*region, selected_indices.indices, *plane);

				std::cerr << ">> Done: " << tt.toc() << " ms, 已知点在簇 " << containing_cluster_index << "（" << plane->points.size() << " points）中 \n";
				return;
			}
		}

		std::cout << "Point is not found in any cluster" << std::endl;
	}

	// 从NNK最近邻搜索中找出与已知点最近的点所在的簇
	void findClusterByNNK(const PointT &point, const pcl::PointCloud<PointT>::Ptr &region,
						  const std::vector<pcl::PointIndices> &clusters,
						  pcl::PointCloud<PointT>::Ptr &plane, std::vector<size_t> &plane_indices)
	{
		std::cerr << "\n从簇群中找出与已知点最近的簇...\n", tt.tic();

		// 使用 KdTree 查找最近邻点
		pcl::KdTreeFLANN<PointT> kdtree;
		kdtree.setInputCloud(region);

		std::vector<int> pointIdx(1);				// 用于存储最近邻点的索引
		std::vector<float> pointSquaredDistance(1); // 用于存储与已知点的距离

		// 搜索与已知点最近的点
		if (kdtree.nearestKSearch(point, 1, pointIdx, pointSquaredDistance) > 0)
		{
			int nearest_index = pointIdx[0]; // 最近邻点的索引
			const PointT &nearest_point = region->points[nearest_index];

			// 查找最近邻点所属的簇
			int containing_cluster_index = -1;
			for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index)
			{
				const pcl::PointIndices &cluster_indices = clusters[cluster_index];
				for (const int index : cluster_indices.indices)
				{
					if (region->points[index].x == nearest_point.x &&
						region->points[index].y == nearest_point.y &&
						region->points[index].z == nearest_point.z)
					{
						containing_cluster_index = static_cast<int>(cluster_index);
						break;
					}
				}
				if (containing_cluster_index != -1)
					break;
			}

			if (containing_cluster_index != -1)
			{
				// 提取该簇中所有点的索引
				const pcl::PointIndices &selected_indices = clusters[containing_cluster_index];
				plane_indices.clear(); // 清空之前的索引
				for (const int index : selected_indices.indices)
				{
					plane_indices.push_back(static_cast<size_t>(index)); // 将索引添加到 plane_indices 中
				}

				// 创建一个新的点云对象包含该簇中的所有点
				plane.reset(new pcl::PointCloud<PointT>);
				pcl::copyPointCloud(*region, selected_indices.indices, *plane);

				std::cerr << ">> Done: " << tt.toc() << " ms, 已知点的最近邻在簇 "
						  << containing_cluster_index << "（" << plane->points.size()
						  << " points）中 \n";
				return;
			}
		}

		std::cout << "Point not found in any cluster or no nearest neighbor found." << std::endl;
	}

	// 计算轴端点 axis必须是单位向量
	std::pair<PointT, PointT> getAxisEndPoints(const pcl::PointCloud<PointT>::Ptr &cloud_cone, const Eigen::Vector3f &center, const Eigen::Vector3f &axis)
	{
		pcl::PointXYZ min_pt, max_pt;
		pcl::getMinMax3D(*cloud_cone, min_pt, max_pt);
		// 计算点云中所有点在轴线方向上的投影
		std::vector<float> projections(cloud_cone->points.size());
		for (size_t i = 0; i < cloud_cone->points.size(); ++i)
		{
			auto point = cloud_cone->points[i];
			Eigen::Vector3f point_vec(point.x, point.y, point.z);
			projections[i] = axis.dot(point_vec - center);
		}
		Eigen::Vector3f ve_t(max_pt.x, max_pt.y, max_pt.z);
		// 找到投影值的最大值和最小值
		auto minmax = std::minmax_element(projections.begin(), projections.end());

		// 计算极小值点和极大值点在轴线上的投影点
		Eigen::Vector3f min_proj_point = center + axis * (*minmax.first);
		Eigen::Vector3f max_proj_point = center + axis * (*minmax.second);

		// 极小值点和极大值点在轴线上的投影点
		PointT min_proj_pt;
		min_proj_pt.x = min_proj_point[0];
		min_proj_pt.y = min_proj_point[1];
		min_proj_pt.z = min_proj_point[2];

		PointT max_proj_pt;
		max_proj_pt.x = max_proj_point[0];
		max_proj_pt.y = max_proj_point[1];
		max_proj_pt.z = max_proj_point[2];

		return std::make_pair(min_proj_pt, max_proj_pt);
	}

	// SAC形状拟合 输出coeff
	void sacComputeNormal(const pcl::PointCloud<PointT>::Ptr &cloud, pcl::ModelCoefficients::Ptr &coeff, int mode, float distanceThreshold, float radius = 0.1f)
	{
		// std::cerr << "\nSAC形状拟合...\n", tt.tic();
		// 建立搜索KD树
		pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
		// 计算点云法向
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		computeNormals(cloud, normals, radius);
		// 创建一个SACSegmentation对象，方法类型为RANSAC
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
		seg.setModelType(mode);
		seg.setMethodType(pcl::SAC_RANSAC);
		// 设置距离阈值
		seg.setDistanceThreshold(distanceThreshold);
		// 设置最大迭代次数
		// seg.setMaxIterations(100);
		// 设置概率
		// seg.setProbability(0.6);
		seg.setInputCloud(cloud);	  // 设置输入点云
		seg.setInputNormals(normals); // 设置输入法向
		// 创建一个模型系数对象和一个内点索引对象
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// 调用segment方法进行分割
		seg.segment(*inliers, *coefficients);
		*coeff = *coefficients;
		// std::cerr << ">> Done: " << tt.toc() << " ms, " << inliers->indices.size() << " points\n";
		// 检查是否分割成功
		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		}
	}

	// SAC形状拟合 输出indices
	void sacComputeNormal(const pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, int mode, float distanceThreshold, float radius = 0.1f)
	{
		std::cerr << "\nSAC形状拟合...\n", tt.tic();
		// 建立搜索KD树
		pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
		// 计算点云法向
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		computeNormals(cloud, normals, radius);
		// 创建一个SACSegmentation对象，方法类型为RANSAC
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
		seg.setModelType(mode);
		seg.setMethodType(pcl::SAC_RANSAC);
		// 设置距离阈值
		seg.setDistanceThreshold(distanceThreshold);
		// 设置最大迭代次数
		// seg.setMaxIterations(100);
		// 设置概率
		// seg.setProbability(0.6);
		seg.setInputCloud(cloud);	  // 设置输入点云
		seg.setInputNormals(normals); // 设置输入法向
		// 创建一个模型系数对象和一个内点索引对象
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		// 调用segment方法进行分割
		seg.segment(*inliers, *coefficients);
		std::cerr << ">> Done: " << tt.toc() << " ms, " << inliers->indices.size() << " points\n";
		// 检查是否分割成功
		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		}
	}

#pragma endregion

#pragma region PCL可视化

	// 初始化viewer
	void initViewer(pcl::visualization::PCLVisualizer::Ptr &viewer, pcl::PointCloud<PointT>::Ptr &cloud)
	{
		viewer->initCameraParameters();
		viewer->setBackgroundColor(37, 37, 37);
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud, centroid);
		viewer->setCameraPosition(centroid[0], centroid[1], centroid[2] + 10.0f, centroid[0], centroid[1], centroid[2], 0, 1, 0);
	}
	// 初始化viewer
	void initViewer(pcl::visualization::PCLVisualizer::Ptr &viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
	{
		viewer->initCameraParameters();
		viewer->setBackgroundColor(37, 37, 37);
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud, centroid);
		viewer->setCameraPosition(centroid[0], centroid[1], centroid[2] + 10.0f, centroid[0], centroid[1], centroid[2], 0, 1, 0);
	}
	// 添加viewport (从1开始)
	void addViewport(pcl::visualization::PCLVisualizer::Ptr &viewer, int viewport, int count = 1, float r = 50.f/255.f, float g = 48.f/255.f, float b = 48.f/255.f)
	{
		float x_min = (viewport - 1) * (1.0 / count);
		float x_max = viewport * (1.0 / count);
		viewer->createViewPort(x_min, 0.0, x_max, 1.0, viewport);
		// viewer->createViewPort((viewport - 1) * 0.25, 0.0, viewport * 0.25, 1.0, viewport); // 这种写法只能保证4个视口不重叠排列
		viewer->setBackgroundColor(r, g, b, viewport);
		viewer->addCoordinateSystem(1.0, "coordinate", viewport);
		std::string x_label = "x_label_v" + std::to_string(viewport);
		std::string y_label = "y_label_v" + std::to_string(viewport);
		std::string z_label = "z_label_v" + std::to_string(viewport);
		// viewer->addText3D("x", pcl::PointXYZ(11, 0, 0), 1.0, 1.0, 0.0, 0.0, x_label, viewport);
		// viewer->addText3D("y", pcl::PointXYZ(0, 11, 0), 1.0, 0.0, 1.0, 0.0, y_label, viewport);
		// viewer->addText3D("z", pcl::PointXYZ(0, 0, 11), 1.0, 0.0, 0.0, 1.0, z_label, viewport);
		// 标题
		viewer->addText("viewport " + std::to_string(viewport), 10, 10, std::to_string(viewport), viewport);
	}
	// 添加viewport (从1开始)
	void addViewport(std::string name, pcl::visualization::PCLVisualizer::Ptr& viewer, int viewport, int count = 1, float r = 50.f/255.f, float g = 48.f/255.f, float b = 48.f/255.f)
	{
		float x_min = (viewport - 1) * (1.0 / count);
		float x_max = viewport * (1.0 / count);
		viewer->createViewPort(x_min, 0.0, x_max, 1.0, viewport);
		// viewer->createViewPort((viewport - 1) * 0.25, 0.0, viewport * 0.25, 1.0, viewport); // 这种写法只能保证4个视口不重叠排列
		viewer->setBackgroundColor(r, g, b, viewport);
		viewer->addCoordinateSystem(1.0, "coordinate", viewport);
		std::string x_label = "x_label_v" + std::to_string(viewport);
		std::string y_label = "y_label_v" + std::to_string(viewport);
		std::string z_label = "z_label_v" + std::to_string(viewport);
		// viewer->addText3D("x", pcl::PointXYZ(11, 0, 0), 1.0, 1.0, 0.0, 0.0, x_label, viewport);
		// viewer->addText3D("y", pcl::PointXYZ(0, 11, 0), 1.0, 0.0, 1.0, 0.0, y_label, viewport);
		// viewer->addText3D("z", pcl::PointXYZ(0, 0, 11), 1.0, 0.0, 0.0, 1.0, z_label, viewport);
		// 标题
		viewer->addText(name, 10, 10, std::to_string(viewport), viewport);
	}

	// 在可视化工具中添加线段
	void addLine(pcl::visualization::PCLVisualizer::Ptr &viewer, PointT point1, PointT point2, std::string name = "line", int viewport = 1)
	{
		if (viewer->contains(name))
		{
			viewer->removeShape(name);
		}
		// 在可视化工具中添加线段
		viewer->addLine(point1, point2, name, viewport);

		// 设置线段的颜色和宽度
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0 / 255.0, 0 / 255.0, 0 / 255.0, name);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, name);
	}

	// 在可视化工具中添加线段
	void addLine(pcl::visualization::PCLVisualizer::Ptr &viewer, PointT point1, PointT point2, int viewport)
	{
		std::string name = "line" + std::to_string(viewport);
		if (viewer->contains(name))
		{
			viewer->removeShape(name);
		}
		// 在可视化工具中添加线段
		viewer->addLine(point1, point2, name, viewport);

		// 设置线段的颜色和宽度
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, name);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, name);
	}
	// 添加点云 自定义hsv
	void addCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<PointT>::Ptr &cloud, float hue, int viewport, int pt_size = 2)
	{
		// 将HSV值转换为RGB
		float r, g, b;
		hsv2rgb(hue, 1.0, 1.0, &r, &g, &b);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud, r * 255.0, g * 255.0, b * 255.0);
		std::string cloud_id = "cloud" + std::to_string(viewport) + std::to_string(hue);
		viewer->addPointCloud(cloud, color, cloud_id, viewport);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_size, cloud_id);
	}
	// 添加点云 自定义hsv  传入id
	void addCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<PointT>::Ptr &cloud, float hue, int viewport, int pt_size = 2, std::string id = "cloud_src")
	{
		// 将HSV值转换为RGB
		float r, g, b;
		hsv2rgb(hue, 1.0, 1.0, &r, &g, &b);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud, r * 255.0, g * 255.0, b * 255.0);
		viewer->addPointCloud(cloud, color, id, viewport);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_size, id);
	}
	// 添加点云 黑色
	void addCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<PointT>::Ptr &cloud, std::string id = "cloud_src", int viewport = 1)
	{
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud, 0, 0, 0);
		viewer->addPointCloud(cloud, color, id, viewport);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
	}

	// 添加点云 Z值着色
	void addCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<PointT>::Ptr &cloud, int viewport, std::string axis, int pt_size = 2, std::string cloud_id = "cloud_src")
	{
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloud, axis); // 按Z轴值着色
		viewer->addPointCloud(cloud, color_handler, cloud_id, viewport);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_size, cloud_id);
	}
	void addCloudWithCustomColor(pcl::visualization::PCLVisualizer::Ptr &viewer,
								 const typename pcl::PointCloud<PointT>::Ptr &cloud,
								 int viewport,
								 float x_min,
								 float x_max,
								 int pt_size = 2,
								 std::string cloud_id = "cloud_src")
	{
		// 创建存储颜色的点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (const auto &point : *cloud)
		{
			pcl::PointXYZRGB colored_point;
			colored_point.x = point.x;
			colored_point.y = point.y;
			colored_point.z = point.z;

			// 判断x是否在区间内
			if (point.x >= x_min && point.x <= x_max)
			{
				// 按x值着色，映射到0~255之间
				float hue = (point.x - x_min) / (x_max - x_min);
				float r, g, b;
				hsv2rgb(hue, 1.0, 1.0, &r, &g, &b);
				colored_point.r = int(r * 255);
				colored_point.g = int(g * 255);
				colored_point.b = int(b * 255);
			}
			else
			{
				// 不在区间内设为黑色
				colored_point.r = 0;
				colored_point.g = 0;
				colored_point.b = 0;
			}

			colored_cloud->points.push_back(colored_point);
		}
		colored_cloud->width = cloud->width;
		colored_cloud->height = cloud->height;
		colored_cloud->is_dense = cloud->is_dense;

		// 显示点云
		viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, cloud_id, viewport);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_size, cloud_id);
	}
	// 添加RGB点云
	void addCloudRGB(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb, std::string id, int pt_size = 10, int viewport = 1)
	{
		std::string cloud_id = "cloud_rgb_" + id;
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, cloud_id, viewport);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_size, cloud_id);
	}

	// 添加点云，强度值着色，限制最小强度，红强蓝弱。
	void addCloudI(pcl::visualization::PCLVisualizer::Ptr &viewer,
				   const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
				   int viewport,
				   std::string cloud_id = "cloud_custom_color")
	{
		// 创建新的点云用于存储颜色信息
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>());
		cloud_colored->points.resize(cloud->points.size());
		cloud_colored->width = cloud->width;
		cloud_colored->height = cloud->height;
		cloud_colored->is_dense = cloud->is_dense;

		// 获取强度值的最小值和最大值
		float min_intensity = std::numeric_limits<float>::max();
		float max_intensity = std::numeric_limits<float>::lowest();
		for (const auto &point : *cloud)
		{
			min_intensity = std::min(min_intensity, point.intensity);
			max_intensity = std::max(max_intensity, point.intensity);
		}

		// min_intensity = 0; // DEBUG可视化 自定义强度区间
		max_intensity = 1600; 

		// 映射强度值到颜色
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			const auto &point = cloud->points[i];
			float normalized_intensity = (point.intensity - min_intensity) / (max_intensity - min_intensity);

			// HSV 映射：H 从 0 (红色) -> 240 (蓝色)，S 和 V 固定为 1
			float h = (1.f - normalized_intensity) * 240.0f / 360.0f; // 反向映射：强度越大，H 越接近 0 ，为红色
			float s = 0.8f;
			float v = 0.8f;

			// 转换 HSV 到 RGB
			float r, g, b;
			hsv2rgb(h, s, v, &r, &g, &b);
			if (point.intensity < min_intensity)
				r = g = b = 0.0;

			// 填充点云颜色信息
			pcl::PointXYZRGB colored_point;
			colored_point.x = point.x;
			colored_point.y = point.y;
			colored_point.z = point.z;
			colored_point.r = static_cast<uint8_t>(r * 255);
			colored_point.g = static_cast<uint8_t>(g * 255);
			colored_point.b = static_cast<uint8_t>(b * 255);
			cloud_colored->points[i] = colored_point;
		}

		// 添加点云到可视化器
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_colored, cloud_id, viewport);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_id);
	}

	// 可视化簇群
	void addClusters(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<pcl::PointIndices> &cluster_indices, std::string name = "cluster_", int viewport = 1)
	{
		// 构造颜色句柄
		std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointT>> color_handlers;
		for (size_t i = 0; i < cluster_indices.size(); ++i)
		{
			float r, g, b;
			float hue = float(i) / cluster_indices.size();
			hsv2rgb(hue, 1.0, 1.0, &r, &g, &b);
			color_handlers.emplace_back(cloud, r * 255.0f, g * 255.0f, b * 255.0f);
		}
		// 添加点云到可视化对象中
		for (size_t i = 0; i < cluster_indices.size(); ++i)
		{
			pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
			for (auto index : cluster_indices[i].indices)
				cluster->push_back(cloud->points[index]);
			name += std::to_string(i);
			if (viewer->contains(name))
				viewer->removePointCloud(name, viewport);
			viewer->addPointCloud(cluster, color_handlers[i], name, viewport);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
		}
	}

	// 添加点到点的线段及打印其距离
	void addLine_point2point(pcl::visualization::PCLVisualizer::Ptr &viewer, const PointT &point1, const PointT &point2, float hue, int viewport, std::string name)
	{
		// 计算两点之间的距离
		float distance = std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
		std::ostringstream stream;
		stream << std::fixed << std::setprecision(3) << distance;
		std::string distance_str = stream.str(); // 这个字符串就是保留了三位小数的距离

		PointT midpoint;
		midpoint.x = (point1.x + point2.x) / 2.0f;
		midpoint.y = (point1.y + point2.y) / 2.0f;
		midpoint.z = (point1.z + point2.z) / 2.0f;
		PointT text_point;
		float offset = 0.2;
		text_point.x = midpoint.x;
		text_point.y = midpoint.y;
		text_point.z = midpoint.z + offset;

		// 将HSV值转换为RGB
		float r, g, b;
		hsv2rgb(hue, 1.0, 1.0, &r, &g, &b);

		std::string line_id = "line_pt2pt_" + name;
		std::string txtH_id = "txtH_pt2pt_" + name;
		std::string txtName_id = "txtName_pt2pt_" + name;

		// 在可视化工具中添加线段
		viewer->addLine(point1, point2, r, g, b, line_id, viewport);
		// 在可视化工具中添加3D文本
		viewer->addText3D(distance_str, midpoint, 0.1, r, g, b, txtH_id, viewport);
		viewer->addText3D(name, text_point, 0.1, r, g, b, txtName_id, viewport);
		// 设置线段的宽度
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, line_id, viewport);
	}

	// 添加点到面的线段及 打印其距离 自定义hsv
	void addLine_point2plane(pcl::visualization::PCLVisualizer::Ptr &viewer, const PointT &point, const pcl::ModelCoefficientsPtr &plane, float hue, int viewport, std::string name)
	{
		// 计算点到平面的距离
		float distance = computePointToPlaneDistance(point, plane);
		std::ostringstream stream;
		stream << std::fixed << std::setprecision(3) << distance;
		std::string distance_str = stream.str(); // 这个字符串就是保留了两位小数的距离
		// 计算平面上的点
		PointT point_on_plane;
		point_on_plane.x = point.x - distance * plane->values[0];
		point_on_plane.y = point.y - distance * plane->values[1];
		point_on_plane.z = point.z - distance * plane->values[2];

		PointT midpoint;
		midpoint.x = (point.x + point_on_plane.x) / 2.0f;
		midpoint.y = (point.y + point_on_plane.y) / 2.0f;
		midpoint.z = (point.z + point_on_plane.z) / 2.0f;
		PointT text_point;
		float offset = 0.2;
		text_point.x = midpoint.x;
		text_point.y = midpoint.y;
		text_point.z = midpoint.z + offset;

		// 将HSV值转换为RGB
		float r, g, b;
		hsv2rgb(hue, 1.0, 1.0, &r, &g, &b);

		std::string line_id = "line_pt2plane_" + name;
		std::string txtH_id = "txtH_pt2plane_" + name;
		std::string txtName_id = "txtName_pt2plane_" + name;
		// std::string text_id = "text_h" ;
		//  在可视化工具中添加线段
		viewer->addLine(point, point_on_plane, r, g, b, line_id, viewport);
		// 在可视化工具中添加3D文本
		viewer->addText3D(distance_str, midpoint, 0.1, r, g, b, txtH_id, viewport);
		viewer->addText3D(name, text_point, 0.1, r, g, b, txtName_id, viewport);
		// 设置线段的宽度
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, line_id, viewport);
	}
#pragma endregion
};