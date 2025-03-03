/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-11-28 10:30:48
 * @LastEditors  : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-03-03 11:22:55
 * @FilePath     : /pcl_viewer/src/debug_visual.cpp
 * @Description  : PCL可视化方法 （算法测试用）
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */

#include "debug_visual.h"
#include "point_selection.h"
#include <ctime>
#include <mutex>
#include <deque>
namespace alog_lidar
{
    namespace
    {
        std::mutex picked_points_mutex;                                                            // 用于线程安全访问 picked_points
        pcl::PointCloud<pcl::PointXYZI>::Ptr picked_cloud(new pcl::PointCloud<pcl::PointXYZI>());  // 保存选中的点
        pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_visual(new pcl::PointCloud<pcl::PointXYZI>()); // 显示的点云
        std::map<int, PointData> index_to_pointdata;                                               // 全局变量：索引到原始点数据的映射

        // 定义颜色数组（R, G, B）
        std::vector<std::tuple<float, float, float>> list_color = {
            {225 / 255.f, 20 / 255.f, 10 / 255.f},   // 红色
            {43 / 255.f, 220 / 255.f, 112 / 255.f},  // 绿色
            {103 / 255.f, 166 / 255.f, 238 / 255.f}, // 蓝色
            {254 / 255.f, 209 / 255.f, 1 / 255.f},   // 黄色
            {113 / 255.f, 81 / 255.f, 168 / 255.f}   // 紫色
        };

        std::deque<std::pair<std::string, std::string>> recent_points_info;
        const int max_recent_points = 10; // 显示最近的点数量
        int pick_count = 0;               // 选点计数器
    }

    template <typename PointT>
    // 重写定制化选点事件
    void pointPickingCallback(const pcl::visualization::PointPickingEvent &event, void *args)
    {
        auto *data = static_cast<CallbackArgs<PointT> *>(args);

        // 获取世界坐标
        data->getWorldPoint();

        // 绘制点击的射线
        data->viewerPtr->removeShape("clicked_line");
        data->viewerPtr->addLine(data->world_points_near, data->world_points_far, "clicked_line");

        // 找到射线最接近的点
        data->getNearestPointOfRay(); // 返回结果到 clicked_points_3d

        // 真正的选点
        pcl::PointXYZ point(data->clicked_points_3d.x, data->clicked_points_3d.y, data->clicked_points_3d.z);

        // 使用颜色列表来选择球体的颜色
        size_t color_index = pick_count % list_color.size(); // 通过取余确保颜色索引在有效范围内
        auto [r, g, b] = list_color[color_index];

        // 为每个选中的点生成唯一的 ID，并添加球体
        std::string sphere_id = "sphere_" + std::to_string(data->clicked_points_3d.x) + "_" +
                                std::to_string(data->clicked_points_3d.y) + "_" +
                                std::to_string(data->clicked_points_3d.z);
        data->viewerPtr->addSphere(point, 0.01, r, g, b, sphere_id);

        // 构建文本信息
        // 获取当前时间（小时、分钟和秒）
        std::time_t now = std::time(nullptr);
        std::tm *local_time = std::localtime(&now);
        int hour = local_time->tm_hour;
        int minute = local_time->tm_min;
        int second = local_time->tm_sec;

        // 格式化时间和点信息
        std::string time_info = std::string(hour < 10 ? "0" : "") + std::to_string(hour) + ":" +
                                std::string(minute < 10 ? "0" : "") + std::to_string(minute) + ":" +
                                std::string(second < 10 ? "0" : "") + std::to_string(second);

        std::ostringstream oss;
        oss << " x: " << data->clicked_points_3d.x
            << ", y: " << data->clicked_points_3d.y
            << ", z: " << data->clicked_points_3d.z;

        std::string point_info = time_info + oss.str();

        // 获取额外字段数据
        if (index_to_pointdata.find(data->clicked_points_idx) != index_to_pointdata.end())
        {
            const PointData &pt = index_to_pointdata[data->clicked_points_idx];
            std::ostringstream extra_info;
            extra_info << " pixel_idx: " << static_cast<int>(pt.pixel_idx)
                       << ", slot_idx: " << static_cast<int>(pt.slot_idx)
                       << ", R: " << std::fixed << std::setprecision(3) << pt.r
                       << ", SlotAngle: " << std::fixed << std::setprecision(2) << pt.slot_ang
                       << ", Theta: " << std::fixed << std::setprecision(2) << pt.theta
                       << ", Peak: " << static_cast<int>(pt.peak)
                       << ", I: " << static_cast<int>(pt.intensity)
                       << ", Reflect: " << std::fixed << std::setprecision(1) << pt.reflect;
            point_info += extra_info.str();
        }

        // 添加新选点信息到队列的最前面
        recent_points_info.insert(recent_points_info.begin(), {sphere_id, point_info});

        // 如果超出最大点数，删除最早的球体和对应的文本
        if (recent_points_info.size() > max_recent_points)
        {
            // 删除最早的球体
            const std::string &remove_sphere_id = recent_points_info.back().first;
            data->viewerPtr->removeShape(remove_sphere_id);
            recent_points_info.pop_back(); // 移除最后的一条
        }

        // 打印文本信息到屏幕，确保最新的文本显示在底部
        int line_y = 10;
        int line_height = 20;

        // 从底部开始显示最新的文本
        for (size_t i = 0; i < recent_points_info.size(); ++i)
        {
            const auto &point_info = recent_points_info[i];
            const std::string &sphere_id = point_info.first;
            const std::string &text = point_info.second;

            // 确保文本显示在从底部开始的位置
            // int y_position = line_y + (recent_points_info.size() - 1 - i) * line_height;
            int y_position = line_y + i * line_height;
            // 确定文本颜色
            size_t color_idx = (pick_count - i) % list_color.size(); // 通过取余确保颜色索引在有效范围内
            auto [r, g, b] = list_color[color_idx];

            // 更新或添加文本
            std::string text_id = "recent_point_" + std::to_string(i);
            if (!data->viewerPtr->updateText(text, 10, y_position, 16, r, g, b, text_id))
            {
                data->viewerPtr->addText(text, 10, y_position, 16, r, g, b, text_id);
            }
        }
        pick_count++;

        // 输出选点信息到控制台
        std::cout << point_info << std::endl;
    }

    // 重载方法：输入参数 PointMatrix 改为 std::vector<PointData> pts
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat2pcl(const std::vector<PointData> &pts)
    {
        int point_index = 0; // 用于记录索引
        // 创建一个新的 pcl 点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 遍历 std::vector<PointData> 中的每个点，将坐标值存储到 pcl 点云中
        for (const auto &point : pts)
        {
            if (!point.is_valid)
                continue; // 跳过无效点

            pcl::PointXYZ pcl_point;
            // 将 PointData 中的 x, y, z 坐标赋值给 pcl::PointXYZ
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z;

            // 将 pcl_point 添加到点云对象中
            cloud->points.push_back(pcl_point);

            // 仅用于 DEBUG 可视化：建立索引到 PointData 的映射
            // if (false)
            //     index_to_pointdata[point_index++] = point;
        }

        // 返回点云对象
        return cloud;
    }

    // 将 PointMatrix 转换为 pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat2pcl(const PointMatrix &matrix, bool pick_src)
    {
        int point_index = 0; // 用于记录索引
        // 创建一个新的 pcl 点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 遍历 PointMatrix 中的每个点，将坐标值存储到 pcl 点云中
        for (const auto &row : matrix)
        {
            for (const auto &point : row)
            {
                if (!point.is_valid)
                    continue; // 跳过无效点
                pcl::PointXYZ pcl_point;
                // 将 PointData 中的 x, y, z 坐标赋值给 pcl::PointXYZ
                pcl_point.x = point.x;
                pcl_point.y = point.y;
                pcl_point.z = point.z;

                // 将 pcl_point 添加到点云对象中
                cloud->points.push_back(pcl_point);
                if (pick_src)
                    index_to_pointdata[point_index++] = point; // 建立索引到 PointData 的映射 仅用于可视化
            }
        }

        // 返回点云对象
        return cloud;
    }
    // 重载方法，强度，对矩阵点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr mat2xyzi(const PointMatrix &matrix, bool pick_src)
    {
        int point_index = 0; // 用于记录索引
        // 创建一个新的 pcl 点云对象
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // 遍历 PointMatrix 中的每个点，将坐标值存储到 pcl 点云中
        for (const auto &row : matrix)
        {
            for (const auto &point : row)
            {
                if (!point.is_valid)
                    continue; // 跳过无效点
                pcl::PointXYZI pcl_point;
                // 将 PointData 中的 x, y, z 坐标赋值给 pcl::PointXYZ
                pcl_point.x = point.x;
                pcl_point.y = point.y;
                pcl_point.z = point.z;
                pcl_point.intensity = point.peak; // DEBUG可视化 自定义强度映射字段
                // 将 pcl_point 添加到点云对象中
                cloud->points.push_back(pcl_point);
                if (pick_src)
                    index_to_pointdata[point_index++] = point; // 建立索引到 PointData 的映射 仅用于可视化
            }
        }

        // 返回点云对象
        return cloud;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr mat3toxyzi(const PointMatrix3 &matrix, bool pick_src)
    {
        int point_index = 0; // 用于记录索引
        // 创建一个新的 pcl 点云对象
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // 遍历 PointMatrix3 中的每个点
        for (size_t pixel_idx = 0; pixel_idx < matrix.size(); ++pixel_idx)
        {
            for (size_t slot_idx = 0; slot_idx < matrix[pixel_idx].size(); ++slot_idx)
            {
                // 遍历每个位置的回波点
                for (size_t echo_idx = 0; echo_idx < matrix[pixel_idx][slot_idx].size(); ++echo_idx)
                {
                    const PointData &point = matrix[pixel_idx][slot_idx][echo_idx];
                    if (!point.is_valid)
                        continue; // 跳过无效点

                    // 创建 pcl 点并赋值
                    pcl::PointXYZI pcl_point;
                    pcl_point.x = point.x;
                    pcl_point.y = point.y;
                    pcl_point.z = point.z;
                    if(echo_idx != 0)
                    {
                        std::cout << "pixel_idx: " << pixel_idx << ", slot_idx: " << slot_idx << ", echo_idx: " << echo_idx << std::endl;
                        continue;
                    }
                    
                    pcl_point.intensity = point.peak; // 设置反射率值
                    // pcl_point.intensity = static_cast<float>(echo_idx); // 将 intensity 赋值为回波索引 (0, 1, 2)

                    // 添加到点云中
                    cloud->points.push_back(pcl_point);
                    if (pick_src)
                        index_to_pointdata[point_index++] = point; // 建立索引到 PointData 的映射 仅用于可视化
                }
            }
        }

        // 返回点云对象
        return cloud;
    }
    // 重载方法，强度，对向量点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr mat2xyzi(const std::vector<PointData> &pts)
    {
        // 创建一个新的 pcl 点云对象
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // 遍历 std::vector<PointData> 中的每个点，将坐标和反射率存储到 pcl 点云中
        for (const auto &point : pts)
        {
            if (!point.is_valid)
                continue; // 跳过无效点

            pcl::PointXYZI pcl_point;
            // 将 PointData 中的 x, y, z 坐标赋值给 pcl::PointXYZI
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z;
            pcl_point.intensity = point.peak; // 设置反射率值

            // 将 pcl_point 添加到点云对象中
            cloud->points.push_back(pcl_point);
        }

        // 返回点云对象
        return cloud;
    }

    // 重载方法：将 PointMatrix 转换为 pcl::PointCloud<pcl::PointXYZ>::Ptr 并输出索引到 PointData 的映射
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat2pcl(const PointMatrix &matrix, std::map<size_t, PointData> &map)
    {
        size_t point_index = 0; // 用于记录索引
        // 创建一个新的 pcl 点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 遍历 PointMatrix 中的每个点，将坐标值存储到 pcl 点云中
        for (const auto &row : matrix)
        {
            for (const auto &point : row)
            {
                if (!point.is_valid)
                    continue; // 跳过无效点

                pcl::PointXYZ pcl_point;
                // 将 PointData 中的 x, y, z 坐标赋值给 pcl::PointXYZ
                pcl_point.x = point.x;
                pcl_point.y = point.y;
                pcl_point.z = point.z;

                // 将 pcl_point 添加到点云对象中
                cloud->points.push_back(pcl_point);

                // 建立索引到 PointData 的映射
                map[point_index] = point;
                index_to_pointdata[point_index++] = point; // 建立索引到 PointData 的映射 仅用于可视化
            }
        }

        // 返回点云对象
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr mat2pcl_crop24(const PointMatrix &matrix)
    {
        // 创建一个新的 pcl 点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 遍历 PointMatrix 中的每个点，将坐标值存储到 pcl 点云中
        for (int i = 0; i < matrix.size(); ++i)
        {
            if (i < 3 || i > matrix.size() - 3 - 1)
                continue; // 跳过搭接段
            auto row = matrix[i];
            for (const auto &point : row)
            {
                if (!point.is_valid)
                    continue; // 跳过无效点
                pcl::PointXYZ pcl_point;
                // 将 PointData 中的 x, y, z 坐标赋值给 pcl::PointXYZ
                pcl_point.x = point.x;
                pcl_point.y = point.y;
                pcl_point.z = point.z;

                // 将 pcl_point 添加到点云对象中
                cloud->points.push_back(pcl_point);
            }
        }

        // 返回点云对象
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr mat2pcl_crop96(const PointMatrix &matrix)
    {
        // 创建一个新的 pcl 点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 遍历 PointMatrix 中的每个点，将坐标值存储到 pcl 点云中
        for (int i = 0; i < matrix.size(); ++i)
        {
            if (i < 3)
                continue;
            if (i < 23 + 2 + 1 && i > 23 - 2 - 1)
                continue;
            if (i < 47 + 2 + 1 && i > 47 - 2 - 1)
                continue;
            if (i < 71 + 2 + 1 && i > 71 - 2 - 1)
                continue;
            if (i > 92)
                continue;
            auto row = matrix[i];
            for (const auto &point : row)
            {
                if (!point.is_valid)
                    continue; // 跳过无效点
                pcl::PointXYZ pcl_point;
                // 将 PointData 中的 x, y, z 坐标赋值给 pcl::PointXYZ
                pcl_point.x = point.x;
                pcl_point.y = point.y;
                pcl_point.z = point.z;

                // 将 pcl_point 添加到点云对象中
                cloud->points.push_back(pcl_point);
            }
        }

        // 返回点云对象
        return cloud;
    }

    // 显示单个点云  强度着色
    void showCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string viewer_name)
    {
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewer_name));
        pclHelper.initViewer(viewer, cloud);
        pclHelper.addViewport(viewer, 1, 1);
        pclHelper.addCloudI(viewer, cloud, 1, "cloud_i");
        // 注册选点回调函数
        CallbackArgs<pcl::PointXYZI> args;
        args.cloud = cloud;
        args.viewerPtr = viewer;
        viewer->registerMouseCallback(mouseMoveCallback<pcl::PointXYZI>, &args);
        viewer->registerPointPickingCallback(pointPickingCallback<pcl::PointXYZI>, &args);
        viewer->spin();
    }

    void showCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     std::vector<pcl::PointIndices> &cluster,
                     size_t target_idx,
                     std::string name)
    {
        // 创建PclHelperB对象，处理一些PCL可视化的操作
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
        // 初始化viewer，设置基本视图和数据
        pclHelper.initViewer(viewer, cloud);
        pclHelper.addViewport(viewer, 1, 1); // 设置显示窗口

        // 添加所有聚类到viewer中
        pclHelper.addClusters(viewer, cloud, cluster);

        // 提取目标聚类点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
        pclHelper.getSubCloud(cloud, cluster[target_idx], cloud_target);

        // 将目标聚类点云添加到viewer
        pclHelper.addCloud(viewer, cloud_target);   

        // 注册选点回调函数
        CallbackArgs<pcl::PointXYZ> args;
        args.cloud = cloud;
        args.viewerPtr = viewer;
        viewer->registerMouseCallback(mouseMoveCallback<pcl::PointXYZ>, &args);
        viewer->registerPointPickingCallback(pointPickingCallback<pcl::PointXYZ>, &args);

        // 进入事件循环，保持显示
        viewer->spin();
    }

    // 显示两个点云在同一视口中
    void showCloud2in1port(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string name)
    {
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
        pclHelper.initViewer(viewer, cloud1);
        pclHelper.addViewport(viewer, 1, 1);
        pclHelper.addCloud(viewer, cloud1, 0.1, 1, 1, "cloud1");
        pclHelper.addCloud(viewer, cloud2, 0.6, 1, 1, "cloud2");

        // 注册选点回调函数
        CallbackArgs<pcl::PointXYZ> args;
        args.cloud = cloud1;
        args.viewerPtr = viewer;
        viewer->registerMouseCallback(mouseMoveCallback<pcl::PointXYZ>, &args);
        viewer->registerPointPickingCallback(pointPickingCallback<pcl::PointXYZ>, &args);
        viewer->spin();
    }

    // 显示两个点云在同一视口中
    void showCloud2in1port(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, std::string viewer_name)
    {
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewer_name));
        pclHelper.initViewer(viewer, cloud1);
        pclHelper.addViewport(viewer, 1, 1);
        pclHelper.addCloudI(viewer, cloud1, 1, "cloud_1");
        pclHelper.addCloudI(viewer, cloud2, 1, "cloud_2");

        // 注册选点回调函数
        CallbackArgs<pcl::PointXYZI> args;
        args.cloud = cloud1;
        args.viewerPtr = viewer;
        viewer->registerMouseCallback(mouseMoveCallback<pcl::PointXYZI>, &args);
        viewer->registerPointPickingCallback(pointPickingCallback<pcl::PointXYZI>, &args);
        viewer->spin();
    }
    // 显示 背景黑色点云和多个强度点云列表
    void showListXyzi(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> list_cloud)
    {
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("showListXyzi"));
        pclHelper.initViewer(viewer, cloud_base);
        pclHelper.addViewport(viewer, 1, 1);
        pclHelper.addCloud(viewer, cloud_base, "cloud_base", 1);
        for(size_t i=0;i<list_cloud.size();++i)
        {
            pclHelper.addCloudI(viewer, list_cloud[i], 1, "cloud_"+std::to_string(i));
        }
        // 注册选点回调函数
        CallbackArgs<pcl::PointXYZ> args;
        args.cloud = cloud_base;
        args.viewerPtr = viewer;
        viewer->registerMouseCallback(mouseMoveCallback<pcl::PointXYZ>, &args);
        viewer->registerPointPickingCallback(pointPickingCallback<pcl::PointXYZ>, &args);
        viewer->spin();
    }
    // 显示 背景黑色点云和多个彩色点云（每个点云一个颜色）列表
    void showListCloudRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_cloud)
    {
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("showListCloudRGB"));
        pclHelper.initViewer(viewer, cloud_base);
        pclHelper.addViewport(viewer, 1, 1);
        pclHelper.addCloud(viewer, cloud_base, "cloud_base", 1);
        for (size_t i = 0; i < list_cloud.size(); ++i)
        {
            pclHelper.addCloud(viewer, list_cloud[i], 1.0 / list_cloud.size() * i, 1, 3, "cloud_" + std::to_string(i));
        }
        // 注册选点回调函数
        CallbackArgs<pcl::PointXYZ> args;
        args.cloud = cloud_base;
        args.viewerPtr = viewer;
        viewer->registerMouseCallback(mouseMoveCallback<pcl::PointXYZ>, &args);
        viewer->registerPointPickingCallback(pointPickingCallback<pcl::PointXYZ>, &args);
        viewer->spin();
    }

    // 显示4个点云在同一视口中
    void showCloud4in1port(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4)
    {
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("showCloud4in1port"));
        pclHelper.initViewer(viewer, cloud1);
        pclHelper.addViewport(viewer, 1, 1);
        pclHelper.addCloud(viewer, cloud1, 0.1, 1, 3, "cloud1");
        pclHelper.addCloud(viewer, cloud2, 0.4, 1, 3, "cloud2");
        pclHelper.addCloud(viewer, cloud3, 0.6, 1, 3, "cloud3");
        pclHelper.addCloud(viewer, cloud4, 0.8, 1, 3, "cloud4");
        // 注册选点回调函数
        CallbackArgs<pcl::PointXYZ> args;
        args.cloud = cloud1;
        args.viewerPtr = viewer;
        viewer->registerMouseCallback(mouseMoveCallback<pcl::PointXYZ>, &args);
        viewer->registerPointPickingCallback(pointPickingCallback<pcl::PointXYZ>, &args);
        viewer->spin();
    }

    // 显示两个点云
    void showCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string name)
    {
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
        pclHelper.initViewer(viewer, cloud1);
        pclHelper.addViewport(viewer, 1, 2);
        pclHelper.addViewport(viewer, 2, 2);

        float r_offset = 5.5;
        pclHelper.addCloudWithCustomColor(viewer, cloud1, 1, 3.1 + r_offset, 3.6 + r_offset, 2, "cloud1");
        pclHelper.addCloudWithCustomColor(viewer, cloud2, 2, 3.1 + r_offset, 3.6 + r_offset, 2, "cloud2");
        viewer->spin();
    }
    // 显示两个点云
    void showCloud2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, std::string name)
    {
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
        pclHelper.initViewer(viewer, cloud1);
        pclHelper.addViewport(viewer, 1, 2);
        pclHelper.addViewport(viewer, 2, 2);
        pclHelper.addCloudI(viewer, cloud1, 1, "showCloud2_cloud_1");
        pclHelper.addCloudI(viewer, cloud2, 2, "showCloud2_cloud_2");
        
        // 注册选点回调函数
        CallbackArgs<pcl::PointXYZI> args;
        args.cloud = cloud1;
        args.viewerPtr = viewer;
        viewer->registerMouseCallback(mouseMoveCallback<pcl::PointXYZI>, &args);
        viewer->registerPointPickingCallback(pointPickingCallback<pcl::PointXYZI>, &args);
        viewer->spin();
    }

    // 显示四个点云
    void showCloud4(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4)
    {
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("showCloud4"));
        pclHelper.initViewer(viewer, cloud1);
        pclHelper.addViewport(viewer, 1, 2);
        pclHelper.addViewport(viewer, 2, 2);
        pclHelper.addCloud(viewer, cloud1, 1, "cloud1");
        pclHelper.addCloud(viewer, cloud2, 2, "cloud2");

        viewer->spin();
    }

}
