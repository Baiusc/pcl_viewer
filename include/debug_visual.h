/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-11-28 10:32:20
 * @LastEditors  : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-02-14 15:36:52
 * @FilePath     : /calib_r_offset/include/debug_visual.h
 * @Description  : PCL可视化方法 （算法测试用）
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */

#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <fstream>
#include <PclHelperB.h>
#include "pulse_data_processor.h"

namespace alog_lidar
{
    // 重载方法：输入参数 PointMatrix 改为 std::vector<PointData> pts
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat2pcl(const std::vector<PointData> &pts);
    // 将 PointMatrix 转换为 pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat2pcl(const PointMatrix &matrix, bool pick_src = false);
    // 重载方法，强度，对矩阵点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr mat2xyzi(const PointMatrix &matrix, bool pick_src = false);
    // 重载方法，强度，对向量点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr mat2xyzi(const std::vector<PointData>& pts);
    // 重载方法：将 PointMatrix 转换为 pcl::PointCloud<pcl::PointXYZ>::Ptr 并输出索引到 PointData 的映射
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat2pcl(const PointMatrix &matrix, std::map<size_t, PointData> &map);
    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat2pcl_crop24(const PointMatrix &matrix);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat2pcl_crop96(const PointMatrix &matrix);

    // 显示 背景黑色点云和多个强度点云列表
    void showListXyzi(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> list_cloud);
    // 显示 背景黑色点云和多个彩色点云（每个点云一个颜色）列表
    void showListCloudRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_cloud);

    // 显示单个点云  强度着色
    void showCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string viewer_name);

    // 可视化簇群
    void showCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     std::vector<pcl::PointIndices> &cluster,
                     size_t target_idx,
                     std::string name = "showCluster");

    // 显示两个点云在同一视口中
    void showCloud2in1port(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string name = "showCloud2in1port");
    // 显示两个点云在同一视口中，第二个点云是xyzi
    void showCloud2in1port(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, std::string viewer_name);
    // 显示4个点云在同一视口中
    void showCloud4in1port(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4);
    // 显示两个点云
    void showCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string name = "showCloud2");
    // 显示两个点云
    void showCloud2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, std::string name);
    // 显示四个点云
    void showCloud4(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4);

} // namespace debug_visual
