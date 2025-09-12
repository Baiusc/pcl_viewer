/*
 * @Author       : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @Date         : 2025-03-06 17:17:38
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-09-12 15:54:57
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
             << ", reflect: " << std::fixed << std::setprecision(2) << pd.reflect
             << ", theta: " << std::fixed << std::setprecision(2) << pd.theta
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
         if (list_cloud.empty()) {
             std::cerr << "Error: list_cloud is empty." << std::endl;
             return;
         }
     
         PclHelperB pclHelper;
         pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewer_name));
         pclHelper.initViewer(viewer, list_cloud[0]); // 初始化使用第一个点云
         pclHelper.addViewport(viewer, 1, 1);
         // 将点云按索引顺序添加到可视化窗口
         for (size_t i = 0; i < list_cloud.size(); ++i) {
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
 