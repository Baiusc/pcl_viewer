/*
 * @Author       : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @Date         : 2025-03-06 17:17:38
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2026-08-31 14:27:28
 * @FilePath     : /pcl_viewer/src/debug_visual.cpp
 * @Description  :
 *
 * Copyright (c) 2025 by vitalchem, All Rights Reserved.
 */

#include "debug_visual.h"
#include <ctime>
#include <mutex>
#include <deque>
#include <ctime> // 添加头文件
#include <sstream>
namespace algo_lidar
{
    namespace
    {
        std::mutex picked_points_mutex;                                                            // 用于线程安全访问 picked_points
        pcl::PointCloud<pcl::PointXYZI>::Ptr picked_cloud(new pcl::PointCloud<pcl::PointXYZI>());  // 保存选中的点
        pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_visual(new pcl::PointCloud<pcl::PointXYZI>()); // 显示的点云
        std::map<int, PointData> index_to_pointdata;                                               // 全局变量：索引到原始点数据的映射
        std::vector<std::map<int, PointData>> list_idx_map_mat;                                    // 全局变量：索引到原始点数据的映射  （多点云）

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
    std::string m_intensity_src_name = "peak"; // 初始化默认值
    template <typename PointT>
    // 重写定制化选点事件 打印更详细的选点信息
    void pointPickingCallback(const pcl::visualization::PointPickingEvent &event, void *args)
    {
        auto *data = static_cast<CallbackArgs<PointT> *>(args);

        // 获取世界坐标并绘制射线
        data->getWorldPoint();
        data->viewerPtr->removeShape("clicked_line");
        data->viewerPtr->addLine(data->world_points_near, data->world_points_far, "clicked_line");

        // 获取最近点并创建球体
        data->getNearestPointOfRay();
        const auto &pt = data->clicked_points_3d;
        pcl::PointXYZ point(pt.x, pt.y, pt.z);

        // 生成唯一标识符和颜色
        const auto &[r, g, b] = list_color[pick_count % list_color.size()];
        const std::string sphere_id = "sphere_" + std::to_string(pt.x) + "_" +
                                      std::to_string(pt.y) + "_" +
                                      std::to_string(pt.z);

        // 添加可视化元素
        data->viewerPtr->addSphere(point, 0.01, r, g, b, sphere_id);

        // 构建基础信息
        std::ostringstream oss;
        char time_buf[9];

        // 获取当前时间
        std::time_t now = std::time(nullptr);                                  // 定义并初始化 now
        std::strftime(time_buf, sizeof(time_buf), "%T", std::localtime(&now)); // 格式化时间

        // 添加时间和其他信息到 oss
        oss << time_buf << " x: " << pt.x << ", y: " << pt.y << ", z: " << pt.z;

        // 初始化 PointData
        PointData pd;
        if (!data->list_cloud.empty() && data->clicked_cloud_idx >= 0 &&
            static_cast<size_t>(data->clicked_cloud_idx) < list_idx_map_mat.size())
        {
            auto &cloud_map = list_idx_map_mat[data->clicked_cloud_idx];
            if (auto it = cloud_map.find(data->clicked_points_idx); it != cloud_map.end())
            {
                pd = it->second; // 多点云模式赋值
            }
        }
        else if (auto it = index_to_pointdata.find(data->clicked_points_idx);
                 it != index_to_pointdata.end())
        {
            pd = it->second; // 单点云模式赋值
        }

        // 获取扩展点信息
        oss << " pixel_idx: " << static_cast<int>(pd.pixel_idx)
            << ", slot_idx: " << static_cast<int>(pd.slot_idx)
            << ", R: " << std::fixed << std::setprecision(3) << pd.r
            << ", Peak: " << static_cast<int>(pd.peak)
            << ", Timestamp: " << pd.timestamp
            << ", reflect: " << std::fixed << std::setprecision(2) << pd.reflect
            << ", slot_ang: " << std::fixed << std::setprecision(2) << pd.slot_ang
            << ", theta_ang: " << std::fixed << std::setprecision(2) << pd.theta
            << ", select_idx: " << static_cast<int>(pd.select_idx);

        if (!pd.list_pulse.empty())
        {
            oss << ", Pulse: [";
            for (size_t i = 0; i < pd.list_pulse.size(); ++i)
            {
                const auto &pulse = pd.list_pulse[i];
                oss << "{tof:" << std::fixed << std::setprecision(2) << pulse.tof
                    << ", w:" << pulse.width << ", p:" << pulse.peak << "}";
                if (i < pd.list_pulse.size() - 1)
                    oss << ", ";
            }
            oss << "]";
        }

        // 管理最近点显示
        recent_points_info.emplace(recent_points_info.begin(), sphere_id, oss.str());
        if (recent_points_info.size() > max_recent_points)
        {
            data->viewerPtr->removeShape(recent_points_info.back().first);
            recent_points_info.pop_back();
        }

        // 更新文本显示
        constexpr int line_height = 20;
        for (size_t i = 0; i < recent_points_info.size(); ++i)
        {
            const auto &[cr, cg, cb] = list_color[(pick_count - i) % list_color.size()];
            const std::string text_id = "recent_point_" + std::to_string(i);
            //  data->viewerPtr->addText(recent_points_info[i].second, 10, 10 + i * line_height, 16, cr, cg, cb, text_id);
            if (!data->viewerPtr->updateText(recent_points_info[i].second, 10, 10 + i * line_height, 16, cr, cg, cb, text_id))
            {
                data->viewerPtr->addText(recent_points_info[i].second, 10, 10 + i * line_height, 16, cr, cg, cb, text_id);
            }
        }

        // 输出详细信息
        std::cout << oss.str() << std::endl;
        ++pick_count;
    }

    // 修改后的注册函数 args由外部传入
    void registerMultiPtPicking(pcl::visualization::PCLVisualizer::Ptr &viewer,
                                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_cloud,
                                CallbackArgs<pcl::PointXYZ> &args) // 改为外部参数，避免后续回调时访问无效内存（悬垂指针）。
    {
        args.list_cloud = list_cloud;
        args.viewerPtr = viewer;
        viewer->registerMouseCallback(mouseMoveCallback<pcl::PointXYZ>, &args);
        viewer->registerPointPickingCallback(pointPickingCallback<pcl::PointXYZ>, &args);
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
    // 将 std::vector<PointMatrix> 转换为 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> mat2pcl(const std::vector<PointMatrix> &list_mat, bool pick_src)
    {
        // 清空全局变量 list_idx_map_mat
        list_idx_map_mat.clear();

        // 创建输出的点云列表
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_cloud;

        // 遍历每个 PointMatrix
        for (const auto &matrix : list_mat)
        {
            // 创建一个新的 pcl 点云对象
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // 用于记录当前点云的索引到 PointData 的映射
            std::map<int, PointData> idx_map;

            int point_index = 0; // 用于记录索引

            // 遍历当前 PointMatrix 中的每个点
            for (const auto &row : matrix)
            {
                for (const auto &point : row)
                {
                    if (!point.is_valid)
                        continue; // 跳过无效点

                    // 创建 pcl::PointXYZ 点并赋值
                    pcl::PointXYZ pcl_point;
                    pcl_point.x = point.x;
                    pcl_point.y = point.y;
                    pcl_point.z = point.z;

                    // 将 pcl_point 添加到点云对象中
                    cloud->points.push_back(pcl_point);

                    // 如果需要建立索引到 PointData 的映射
                    if (pick_src)
                    {
                        idx_map[point_index++] = point;
                    }
                }
            }

            // 将当前点云添加到输出列表
            list_cloud.push_back(cloud);

            // 将当前点云的索引映射添加到全局变量 list_idx_map_mat
            if (pick_src)
            {
                list_idx_map_mat.push_back(idx_map);
            }
        }

        // 返回点云列表
        return list_cloud;
    }

    // 辅助函数，根据全局变量获取强度值
    float getIntensityValue(const PointData &point)
    {
        if (m_intensity_src_name == "intensity")
        {
            return point.intensity;
        }
        if (m_intensity_src_name == "reflect")
        {
            return point.reflect;
        }
        if (m_intensity_src_name == "x")
        {
            return point.x;
        }
        if (m_intensity_src_name == "y")
        {
            return point.y;
        }
        if (m_intensity_src_name == "z")
        {
            return point.z;
        }
        if (m_intensity_src_name == "r")
        {
            return point.r;
        }
                if (m_intensity_src_name == "r")
        {
            return point.r;
        }
        if (m_intensity_src_name == "timestamp")
        {
            return static_cast<float>(point.timestamp);
        }
        if (m_intensity_src_name == "is_tail_pt")
        {
            return point.is_tail_pt ? 1.0f : 0.0f;
        }
        // 默认或在其他情况下，使用 peak
        return point.peak;
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
                pcl_point.intensity = getIntensityValue(point); // 20250811 使用辅助函数获取强度值
                // 将 pcl_point 添加到点云对象中
                cloud->points.push_back(pcl_point);
                if (pick_src)
                    index_to_pointdata[point_index++] = point; // 建立索引到 PointData 的映射 仅用于可视化
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
            pcl_point.intensity = getIntensityValue(point); // 20250811 使用辅助函数获取强度值
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

    // 从PCD点云构建PointMatrix（标准XYZ）
    bool buildMatFromPcd(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, PointMatrix &mat, float scale_factor)
    {
        if (!cloud || cloud->points.empty())
        {
            std::cerr << "PCD点云为空！" << std::endl;
            return false;
        }

        // 打印点云信息
        std::cout << "PCD点云信息:" << std::endl;
        std::cout << "  总点数: " << cloud->points.size() << std::endl;
        std::cout << "  宽度: " << cloud->width << ", 高度: " << cloud->height << std::endl;
        std::cout << "  是否有序: " << (cloud->isOrganized() ? "是" : "否") << std::endl;
        std::cout << "  单位换算系数: " << scale_factor << " (默认厘米->米)" << std::endl;

        // 假设点云为单行或单列格式
        const int num_rows = cloud->height > 1 ? cloud->height : 1;
        const int num_cols = cloud->width > 1 ? cloud->width : cloud->points.size();

        std::cout << "  转换后矩阵: " << num_rows << " 行 x " << num_cols << " 列" << std::endl;

        // 如果高度>1，则按行列组织；否则按单行组织
        mat.resize(num_rows, std::vector<PointData>(num_cols));

        int valid_count = 0;
        float min_r = std::numeric_limits<float>::max();
        float max_r = std::numeric_limits<float>::lowest();

        for (int row = 0; row < num_rows; ++row)
        {
            for (int col = 0; col < num_cols; ++col)
            {
                int idx = row * num_cols + col;
                if (idx >= static_cast<int>(cloud->points.size()))
                    break;

                const auto &p = cloud->points[idx];

                // 检查是否为无效点（NaN或无穷大）
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                {
                    mat[row][col].is_valid = false;
                    continue;
                }

                PointData point;
                // 应用单位换算
                point.x = p.x * scale_factor;
                point.y = p.y * scale_factor;
                point.z = p.z * scale_factor;
                point.r = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                point.is_valid = true;
                point.pixel_idx = static_cast<uint16_t>(row);
                point.slot_idx = static_cast<uint16_t>(col);
                point.intensity = 128; // 默认灰色
                point.reflect = 0.5f;
                point.peak = 128;

                // 计算角度信息（近似）
                point.theta = std::asin(point.z / (point.r + 1e-6)) * 180.0f / M_PI;
                point.theta_rad = point.theta * M_PI / 180.0f;
                point.slot_ang = std::atan2(point.y, point.x) * 180.0f / M_PI;
                point.omega_rad = point.slot_ang * M_PI / 180.0f;

                mat[row][col] = point;
                valid_count++;

                if (point.r < min_r)
                    min_r = point.r;
                if (point.r > max_r)
                    max_r = point.r;
            }
        }

        std::cout << "  有效点数: " << valid_count << std::endl;
        std::cout << "  距离范围: [" << min_r << ", " << max_r << "] 米" << std::endl;
        return true;
    }

    // 从带强度的PCD点云构建PointMatrix
    bool buildMatFromPcdI(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, PointMatrix &mat, float scale_factor)
    {
        if (!cloud || cloud->points.empty())
        {
            std::cerr << "PCD点云为空！" << std::endl;
            return false;
        }

        // 打印点云信息
        std::cout << "PCD点云信息 (带强度):" << std::endl;
        std::cout << "  总点数: " << cloud->points.size() << std::endl;
        std::cout << "  宽度: " << cloud->width << ", 高度: " << cloud->height << std::endl;
        std::cout << "  是否有序: " << (cloud->isOrganized() ? "是" : "否") << std::endl;
        std::cout << "  单位换算系数: " << scale_factor << " (默认厘米->米)" << std::endl;

        // 统计强度信息
        float min_intensity = std::numeric_limits<float>::max();
        float max_intensity = std::numeric_limits<float>::lowest();
        float sum_intensity = 0.0f;
        for (const auto &p : cloud->points)
        {
            if (std::isfinite(p.intensity))
            {
                min_intensity = std::min(min_intensity, p.intensity);
                max_intensity = std::max(max_intensity, p.intensity);
                sum_intensity += p.intensity;
            }
        }
        std::cout << "  强度范围: [" << min_intensity << ", " << max_intensity << "]" << std::endl;
        std::cout << "  平均强度: " << (cloud->points.size() > 0 ? sum_intensity / cloud->points.size() : 0) << std::endl;

        const int num_rows = cloud->height > 1 ? cloud->height : 1;
        const int num_cols = cloud->width > 1 ? cloud->width : cloud->points.size();

        std::cout << "  转换后矩阵: " << num_rows << " 行 x " << num_cols << " 列" << std::endl;

        mat.resize(num_rows, std::vector<PointData>(num_cols));

        int valid_count = 0;
        float min_r = std::numeric_limits<float>::max();
        float max_r = std::numeric_limits<float>::lowest();

        for (int row = 0; row < num_rows; ++row)
        {
            for (int col = 0; col < num_cols; ++col)
            {
                int idx = row * num_cols + col;
                if (idx >= static_cast<int>(cloud->points.size()))
                    break;

                const auto &p = cloud->points[idx];

                // 检查是否为无效点
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.intensity))
                {
                    mat[row][col].is_valid = false;
                    continue;
                }

                PointData point;
                // 应用单位换算
                point.x = p.x * scale_factor;
                point.y = p.y * scale_factor;
                point.z = p.z * scale_factor;
                point.r = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                point.is_valid = true;
                point.pixel_idx = static_cast<uint16_t>(row);
                point.slot_idx = static_cast<uint16_t>(col);

                // 使用强度信息，归一化到0-255
                float intensity_val = std::min(255.0f, std::max(0.0f, p.intensity));
                point.intensity = static_cast<uint8_t>(intensity_val);
                point.reflect = intensity_val / 255.0f;
                point.peak = static_cast<uint16_t>(intensity_val);

                // 计算角度信息
                point.theta = std::asin(point.z / (point.r + 1e-6)) * 180.0f / M_PI;
                point.theta_rad = point.theta * M_PI / 180.0f;
                point.slot_ang = std::atan2(point.y, point.x) * 180.0f / M_PI;
                point.omega_rad = point.slot_ang * M_PI / 180.0f;

                mat[row][col] = point;
                valid_count++;

                if (point.r < min_r)
                    min_r = point.r;
                if (point.r > max_r)
                    max_r = point.r;
            }
        }

        std::cout << "  有效点数: " << valid_count << std::endl;
        std::cout << "  距离范围: [" << min_r << ", " << max_r << "] 米" << std::endl;
        return true;
    }
    // 读取PCD文件并转换为PointMatrix
    bool readPcdData(const std::string &filename, PointMatrix &mat)
    {
        // 先尝试读取带强度的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud_i) == 0)
        {
            return buildMatFromPcdI(cloud_i, mat);
        }

        // 如果失败，尝试读取标准XYZ点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == 0)
        {
            return buildMatFromPcd(cloud, mat);
        }

        std::cerr << "读取PCD文件失败: " << filename << std::endl;
        return false;
    }

    // 读取 Frame*_ROI*.csv(离线重建的原始点云, 一像素一行含 row/col/角度/距离/双回波) → PointMatrix
    bool readFrameCsv_Old1(const std::string &filename, PointMatrix &mat)
    {
        std::ifstream infile(filename);
        if (!infile.is_open())
        {
            std::cerr << "Failed to open Frame CSV: " << filename << std::endl;
            return false;
        }
        std::string line;
        std::getline(infile, line); // header

        struct FramePt
        {
            int row, col, peak, valid;
            float dist, theta, sa, refl;
            int p0_cent, p0_fwhm, p0_conv;
            int p1_cent, p1_fwhm, p1_conv;
        };
        std::vector<FramePt> pts;
        int max_row = 0, max_col = 0;
        while (std::getline(infile, line))
        {
            if (line.empty())
                continue;
            std::vector<std::string> t;
            std::stringstream ss(line);
            std::string tok;
            while (std::getline(ss, tok, ','))
                t.push_back(tok);
            if (t.size() < 24)
                continue;
            FramePt p;
            p.row = std::stoi(t[2]);
            p.col = std::stoi(t[3]);
            p.sa = std::stof(t[4]);
            p.theta = std::stof(t[5]);
            p.dist = std::stof(t[6]);
            p.peak = std::stoi(t[7]);
            p.refl = std::stof(t[8]);
            p.valid = std::stoi(t[9]);
            p.p0_cent = std::stoi(t[16]);
            p.p0_fwhm = std::stoi(t[14]);
            p.p0_conv = std::stoi(t[15]);
            p.p1_cent = std::stoi(t[23]);
            p.p1_fwhm = std::stoi(t[21]);
            p.p1_conv = std::stoi(t[22]);
            pts.push_back(p);
            max_row = std::max(max_row, p.row);
            max_col = std::max(max_col, p.col);
        }
        if (pts.empty())
        {
            std::cerr << "No valid data in " << filename << std::endl;
            return false;
        }
        mat.assign(max_row + 1, std::vector<PointData>(max_col + 1));
        int valid_count = 0;
        for (auto &p : pts)
        {
            PointData pd;
            pd.pixel_idx = static_cast<uint16_t>(p.row);
            pd.slot_idx = static_cast<uint16_t>(p.col);
            pd.r = p.dist;
            pd.slot_ang = p.sa;
            pd.omega_rad = p.sa * M_PI / 180.0f;
            pd.theta = p.theta;
            pd.theta_rad = p.theta * M_PI / 180.0f;
            pd.reflect = p.refl;
            pd.peak = static_cast<uint16_t>(std::min(p.peak, 65535));
            pd.intensity = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, p.refl * 255.0f)));
            pd.is_valid = (p.valid != 0) && (p.dist > 0.0f) && (p.dist < 1000.0f);
            if (pd.is_valid)
            {
                pd.x = p.dist * std::cos(pd.theta_rad) * std::cos(pd.omega_rad);
                pd.y = p.dist * std::cos(pd.theta_rad) * std::sin(pd.omega_rad);
                pd.z = p.dist * std::sin(pd.theta_rad);
                valid_count++;
            }
            // list_pulse: 双回波(dist_cent cm×256→tof s, fwhm bin, conv峰高)
            const int INVALID = 0xFFFFFF;
            if (p.p0_cent > 0 && p.p0_cent < INVALID)
                pd.list_pulse.emplace_back((p.p0_cent / 25600.0f) * 2.0f / 3.0e8f,
                                           p.p0_fwhm / 256.0f, static_cast<float>(p.p0_conv));
            if (p.p1_cent > 0 && p.p1_cent < INVALID)
                pd.list_pulse.emplace_back((p.p1_cent / 25600.0f) * 2.0f / 3.0e8f,
                                           p.p1_fwhm / 256.0f, static_cast<float>(p.p1_conv));
            mat[p.row][p.col] = pd;
        }
        std::cout << "Frame CSV: " << pts.size() << " 行, 矩阵 " << max_row + 1 << "x" << max_col + 1
                  << ", 有效点 " << valid_count << std::endl;
        return true;
    }

    bool readFrameCsv_Old2(const std::string &filename, PointMatrix &mat,
                           const std::string &dist_method) // dist_method 不再使用，保留以兼容接口
    {
        std::ifstream infile(filename);
        if (!infile.is_open())
        {
            std::cerr << "Failed to open Frame CSV: " << filename << std::endl;
            return false;
        }

        std::string line;
        std::getline(infile, line); // 跳过表头

        struct FramePt
        {
            int row, col, peak, valid, select_idx;
            float theta, sa, refl, dist;
            int p0_dist_rp_raw, p0_fwhm, p0_conv, p0_cent;
            int p1_dist_rp_raw, p1_fwhm, p1_conv, p1_cent;
        };

        std::vector<FramePt> pts;
        int max_row = 0, max_col = 0;

        while (std::getline(infile, line))
        {
            if (line.empty())
                continue;

            std::vector<std::string> t;
            std::stringstream ss(line);
            std::string tok;
            while (std::getline(ss, tok, ','))
                t.push_back(tok);

            // 至少需要 32 个字段（包含 echo_bins）
            if (t.size() < 32)
                continue;

            FramePt p;
            // 基础字段
            p.row = std::stoi(t[2]);         // 行row
            p.col = std::stoi(t[3]);         // 列col
            p.sa = std::stof(t[4]);          // 水平角deg
            p.theta = std::stof(t[5]);       // 俯仰角deg
            p.dist = std::stof(t[6]);        // 距离m（直接使用CSV值）
            p.peak = std::stoi(t[7]);        // 峰高
            p.refl = std::stof(t[8]);        // 反射率
            p.valid = std::stoi(t[9]);       // 有效
            p.select_idx = std::stoi(t[10]); // 最优回波（0/1/-1）

            // p0 原始字段
            p.p0_dist_rp_raw = std::stoi(t[14]); // p0_dist_rp_raw
            p.p0_fwhm = std::stoi(t[16]);        // p0_fwhm_raw
            p.p0_conv = std::stoi(t[18]);        // p0_conv_intensity
            p.p0_cent = std::stoi(t[19]);        // p0_dist_cent_raw

            // p1 原始字段
            p.p1_dist_rp_raw = std::stoi(t[24]); // p1_dist_rp_raw
            p.p1_fwhm = std::stoi(t[26]);        // p1_fwhm_raw
            p.p1_conv = std::stoi(t[28]);        // p1_conv_intensity
            p.p1_cent = std::stoi(t[29]);        // p1_dist_cent_raw

            pts.push_back(p);
            max_row = std::max(max_row, p.row);
            max_col = std::max(max_col, p.col);
        }

        if (pts.empty())
        {
            std::cerr << "No valid data in " << filename << std::endl;
            return false;
        }

        mat.assign(max_row + 1, std::vector<PointData>(max_col + 1));
        int valid_count = 0;
        const int INVALID = 0xFFFFFF;
        const float DIST_SCALE = 25600.0f; // cm×256 → m

        for (const auto &p : pts)
        {
            PointData pd;
            pd.pixel_idx = static_cast<uint16_t>(p.row);
            pd.slot_idx = static_cast<uint16_t>(p.col);
            pd.slot_ang = p.sa;
            pd.omega_rad = p.sa * M_PI / 180.0f;
            pd.theta = p.theta;
            pd.theta_rad = p.theta * M_PI / 180.0f;
            pd.reflect = p.refl;
            // 根据 p.select_idx 选择 conv_intensity 作为 peak
            if (p.select_idx == 1)
                pd.peak = p.p1_conv;
            else if (p.select_idx == 0)
                pd.peak = p.p0_conv;
            else
                pd.peak = 0;
            pd.intensity = p.peak; // 0-255
            // ----- 直接使用 CSV 中的距离和有效标志 -----
            float dist = p.dist;
            bool is_valid = (p.valid != 0) && (dist > 0.0f) && (dist < 1000.0f);
            pd.r = dist;
            pd.is_valid = is_valid;

            if (pd.is_valid)
            {
                pd.x = dist * std::cos(pd.theta_rad) * std::cos(pd.omega_rad);
                pd.y = dist * std::cos(pd.theta_rad) * std::sin(pd.omega_rad);
                pd.z = dist * std::sin(pd.theta_rad);
                valid_count++;
            }

            // ----- 赋值 select_idx -----
            // 将 -1 映射为 0（表示未选择/无效），0/1 保持不变
            pd.select_idx = p.select_idx;

            // ========== 修改点：用 dist_rp_raw 计算 tof，单位为皮秒 ==========
            const int INVALID = 0xFFFFFF;
            const float DIST_SCALE = 25600.0f;
            if (p.p0_dist_rp_raw > 0 && p.p0_dist_rp_raw < INVALID)
            {
                float dist_m = p.p0_dist_rp_raw / DIST_SCALE;
                float tof_ps = dist_m * 2.0f / 3.0e8f * 1e12f;
                pd.list_pulse.emplace_back(tof_ps, p.p0_fwhm / 256.f, static_cast<float>(p.p0_conv));
                // 若 p.p0_fwhm / 256.f 小于2.0，则标记为 is_tail_pt
                if (p.p0_fwhm / 256.f < 2.0f)
                {
                    pd.is_tail_pt = true;
                }
            }
            if (p.p1_dist_rp_raw > 0 && p.p1_dist_rp_raw < INVALID)
            {
                float dist_m = p.p1_dist_rp_raw / DIST_SCALE;
                float tof_ps = dist_m * 2.0f / 3.0e8f * 1e12f;
                pd.list_pulse.emplace_back(tof_ps, p.p1_fwhm / 256.f, static_cast<float>(p.p1_conv));
            }

            mat[p.row][p.col] = pd;
        }

        std::cout << "Frame CSV: " << pts.size() << " 行, 矩阵 "
                  << max_row + 1 << "x" << max_col + 1
                  << ", 有效点 " << valid_count << std::endl;
        return true;
    }

    bool readFrameCsv(const std::string &filename, PointMatrix &mat,
                      const std::string &dist_method) // dist_method 保留以兼容接口
    {
        (void)dist_method;

        std::ifstream infile(filename);
        if (!infile.is_open())
        {
            std::cerr << "Failed to open Frame CSV: " << filename << std::endl;
            return false;
        }
        std::string line;
        std::getline(infile, line); // 跳过表头

        struct FramePt
        {
            int row = 0;
            int col = 0;
            int peak = 0;
            int valid = 0;
            int select_idx = -1;

            uint32_t timestamp = 0; // CSV 新增的“时间戳”列

            float theta = 0.0f;
            float sa = 0.0f;
            float refl = 0.0f;
            float dist = 0.0f;

            int p0_dist_rp_raw = 0;
            int p0_fwhm = 0;
            int p0_conv = 0;
            int p0_cent = 0;

            int p1_dist_rp_raw = 0;
            int p1_fwhm = 0;
            int p1_conv = 0;
            int p1_cent = 0;
        };

        std::vector<FramePt> pts;
        int max_row = 0;
        int max_col = 0;

        while (std::getline(infile, line))
        {
            if (line.empty())
                continue;

            std::vector<std::string> t;
            std::stringstream ss(line);
            std::string tok;

            while (std::getline(ss, tok, ','))
                t.push_back(tok);

            // 新 CSV 共 33 列：
            // frame, roi, 时间戳, row, col, ..., echo_bins
            if (t.size() < 33)
                continue;

            FramePt p;

            // 基础字段
            p.timestamp = t[2].empty()
                              ? 0U
                              : static_cast<uint32_t>(std::stoul(t[2]));

            p.row = std::stoi(t[3]);
            p.col = std::stoi(t[4]);
            p.sa = std::stof(t[5]);
            p.theta = std::stof(t[6]);
            p.dist = std::stof(t[7]);
            p.peak = std::stoi(t[8]);
            p.refl = std::stof(t[9]);
            p.valid = std::stoi(t[10]);
            p.select_idx = std::stoi(t[11]);

            // p0 字段
            p.p0_dist_rp_raw = std::stoi(t[15]);
            p.p0_fwhm = std::stoi(t[17]);
            p.p0_conv = std::stoi(t[19]);
            p.p0_cent = std::stoi(t[20]);

            // p1 字段
            p.p1_dist_rp_raw = std::stoi(t[25]);
            p.p1_fwhm = std::stoi(t[27]);
            p.p1_conv = std::stoi(t[29]);
            p.p1_cent = std::stoi(t[30]);

            pts.push_back(p);
            max_row = std::max(max_row, p.row);
            max_col = std::max(max_col, p.col);
        }

        if (pts.empty())
        {
            std::cerr << "No valid data in " << filename << std::endl;
            return false;
        }

        mat.assign(max_row + 1, std::vector<PointData>(max_col + 1));

        constexpr int INVALID = 0xFFFFFF;
        constexpr float DIST_SCALE = 25600.0f;
        constexpr float LIGHT_SPEED = 3.0e8f;
        constexpr float PI = 3.14159265358979323846f;

        int valid_count = 0;

        for (const auto &p : pts)
        {
            PointData pd{};

            pd.pixel_idx = static_cast<uint16_t>(p.row);
            pd.slot_idx = static_cast<uint16_t>(p.col);

            pd.slot_ang = p.sa;
            pd.omega_rad = p.sa * PI / 180.0f;

            pd.theta = p.theta;
            pd.theta_rad = p.theta * PI / 180.0f;

            pd.reflect = p.refl;
            pd.intensity = static_cast<uint8_t>(p.peak);

            // PointData::timestamp 已改为 uint32_t，无精度损失。
            pd.timestamp = p.timestamp;

            if (p.select_idx == 1)
                pd.peak = static_cast<uint16_t>(p.p1_conv);
            else if (p.select_idx == 0)
                pd.peak = static_cast<uint16_t>(p.p0_conv);
            else
                pd.peak = 0;

            const float dist = p.dist;
            pd.r = dist;
            pd.is_valid = (p.valid != 0) && (dist > 0.0f) && (dist < 1000.0f);

            if (pd.is_valid)
            {
                pd.x = dist * std::cos(pd.theta_rad) * std::cos(pd.omega_rad);
                pd.y = dist * std::cos(pd.theta_rad) * std::sin(pd.omega_rad);
                pd.z = dist * std::sin(pd.theta_rad);
                ++valid_count;
            }

            // -1 表示无有效回波，映射为 0，避免转 uint8_t 后成为 255。
            pd.select_idx = (p.select_idx >= 0)
                                ? static_cast<uint8_t>(p.select_idx)
                                : 0;

            if (p.p0_dist_rp_raw > 0 && p.p0_dist_rp_raw < INVALID)
            {
                const float dist_m = p.p0_dist_rp_raw / DIST_SCALE;
                const float tof_ps = dist_m * 2.0f / LIGHT_SPEED * 1e12f;

                pd.list_pulse.emplace_back(
                    tof_ps,
                    p.p0_fwhm / 256.0f,
                    static_cast<float>(p.p0_conv));

                if (p.p0_fwhm / 256.0f < 2.0f)
                    pd.is_tail_pt = true;
            }

            if (p.p1_dist_rp_raw > 0 && p.p1_dist_rp_raw < INVALID)
            {
                const float dist_m = p.p1_dist_rp_raw / DIST_SCALE;
                const float tof_ps = dist_m * 2.0f / LIGHT_SPEED * 1e12f;

                pd.list_pulse.emplace_back(
                    tof_ps,
                    p.p1_fwhm / 256.0f,
                    static_cast<float>(p.p1_conv));
            }

            mat[p.row][p.col] = std::move(pd);
        }

        std::cout << "Frame CSV: " << pts.size()
                  << " 行, 矩阵 " << max_row + 1 << "x" << max_col + 1
                  << ", 有效点 " << valid_count << std::endl;

        return true;
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
        pclHelper.addCloud(viewer, cloud1, 0.0, 1, 2, "cloud1");
        pclHelper.addCloud(viewer, cloud2, 0.4, 1, 2, "cloud2");

        // 注册选点回调函数
        CallbackArgs<pcl::PointXYZ> args;
        args.cloud = cloud1;
        args.viewerPtr = viewer;
        viewer->registerMouseCallback(mouseMoveCallback<pcl::PointXYZ>, &args);
        viewer->registerPointPickingCallback(pointPickingCallback<pcl::PointXYZ>, &args);
        viewer->spin();
    }
    // 显示两个点云在同一视口中 支持多点云选点
    void showCloud2in1port(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_cloud, std::string viewer_name)
    {
        if (list_cloud.empty())
        {
            std::cerr << "Error: list_cloud is empty." << std::endl;
            return;
        }

        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewer_name));
        pclHelper.initViewer(viewer, list_cloud[0]); // 初始化使用第一个点云
        pclHelper.addViewport(viewer, 1, 1);
        // 将点云按索引顺序添加到可视化窗口
        for (size_t i = 0; i < list_cloud.size(); ++i)
        {
            pclHelper.addCloud(viewer, list_cloud[i], static_cast<float>(i) / list_cloud.size(), 1, 2, "cloud_" + std::to_string(i));
        }

        // 注册选点回调函数
        CallbackArgs<pcl::PointXYZ> args;
        registerMultiPtPicking(viewer, list_cloud, args);

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
        for (size_t i = 0; i < list_cloud.size(); ++i)
        {
            pclHelper.addCloudI(viewer, list_cloud[i], 1, "cloud_" + std::to_string(i));
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
    void showListCloudRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_cloud, std::string viewer_name)
    {
        PclHelperB pclHelper;
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewer_name));
        pclHelper.initViewer(viewer, cloud_base);
        pclHelper.addViewport(viewer, 1, 1);
        pclHelper.addCloud(viewer, cloud_base, "cloud_base", 1);
        for (size_t i = 0; i < list_cloud.size(); ++i)
        {
            pclHelper.addCloud(viewer, list_cloud[i], 1.0 / list_cloud.size() * i, 1, 2, "cloud_" + std::to_string(i));
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
        pclHelper.addCloud(viewer, cloud1, 0.1, 1, 2, "cloud1");
        pclHelper.addCloud(viewer, cloud2, 0.4, 1, 2, "cloud2");
        pclHelper.addCloud(viewer, cloud3, 0.6, 1, 2, "cloud3");
        pclHelper.addCloud(viewer, cloud4, 0.8, 1, 2, "cloud4");
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
