/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-15 10:01:59
 * @LastEditors  : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @LastEditTime : 2024-12-15 10:02:00
 * @FilePath     : /calib_reflectivity/include/point_selection.h
 * @Description  : 
 * 
 * Copyright (c) 2024 by vitalchem, All Rights Reserved. 
 */
#ifndef POINT_SELECTION_H
#define POINT_SELECTION_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>

// 模板化回调参数
template <typename PointT>
struct CallbackArgs {
    typename pcl::PointCloud<PointT>::Ptr cloud;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
    int clicked_points_idx;
    PointT clicked_points_3d;
    PointT world_points_near;
    PointT world_points_far;
    int screen_x, screen_y;

    void getWorldPoint();
    void getNearestPointOfRay();
};

// 得到射线的两个点
template <typename PointT>
void CallbackArgs<PointT>::getWorldPoint() {
    vtkRenderer* renderer = viewerPtr->getRenderWindow()->GetRenderers()->GetFirstRenderer();

    renderer->SetDisplayPoint(screen_x, screen_y, 0);
    renderer->DisplayToWorld();
    world_points_near.x = static_cast<float>(renderer->GetWorldPoint()[0]);
    world_points_near.y = static_cast<float>(renderer->GetWorldPoint()[1]);
    world_points_near.z = static_cast<float>(renderer->GetWorldPoint()[2]);

    renderer->SetDisplayPoint(screen_x, screen_y, 1);
    renderer->DisplayToWorld();
    world_points_far.x = static_cast<float>(renderer->GetWorldPoint()[0]);
    world_points_far.y = static_cast<float>(renderer->GetWorldPoint()[1]);
    world_points_far.z = static_cast<float>(renderer->GetWorldPoint()[2]);
}

// 点云中距射线最近的点
template <typename PointT>
void CallbackArgs<PointT>::getNearestPointOfRay() {
    Eigen::Vector3f point1(world_points_near.x, world_points_near.y, world_points_near.z);
    Eigen::Vector3f point2(world_points_far.x, world_points_far.y, world_points_far.z);

    float minDistance = std::numeric_limits<float>::max();
    size_t closestPointIndex = 0;

    for (size_t i = 0; i < cloud->size(); ++i) {
        Eigen::Vector3f point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

        float distance = ((point - point1).cross(point2 - point1)).squaredNorm();

        if (distance < minDistance) {
            minDistance = distance;
            closestPointIndex = i;
        }
    }
    clicked_points_idx = closestPointIndex;
    clicked_points_3d = cloud->points[closestPointIndex];
}

// 鼠标移动回调函数
template <typename PointT>
void mouseMoveCallback(const pcl::visualization::MouseEvent& e, void* args) {
    auto* data = static_cast<CallbackArgs<PointT>*>(args);
    data->screen_x = e.getX();
    data->screen_y = e.getY();
}

// 点选回调函数
template <typename PointT>
void choosePointCallback(const pcl::visualization::PointPickingEvent& event, void* args) {
    auto* data = static_cast<CallbackArgs<PointT>*>(args);

    data->getWorldPoint();

    data->viewerPtr->removeShape("clicked_line");
    data->viewerPtr->addLine(data->world_points_near, data->world_points_far, "clicked_line");

    data->getNearestPointOfRay();

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    cloud->push_back(data->clicked_points_3d);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(cloud, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
}

#endif // POINT_SELECTION_H
