#ifndef POINT_SELECTION_H
#define POINT_SELECTION_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vector>
#include <limits>

// 模板化回调参数（兼容单点云和多点云）
// 如果 list_cloud 为空，则使用 cloud 成员（保持原来的兼容性），
// 否则使用 list_cloud 进行多点云选点。
template <typename PointT>
struct CallbackArgs {
    // 旧接口，单点云指针，保持兼容性
    typename pcl::PointCloud<PointT>::Ptr cloud;
    // 新接口，多点云容器。对于单云场景，可以将 cloud 放入此容器中
    std::vector<typename pcl::PointCloud<PointT>::Ptr> list_cloud;
    
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
    int clicked_points_idx;         // 在所属点云中的索引
    int clicked_cloud_idx;          // 若使用 list_cloud，则记录选中点所在云的索引
    PointT clicked_points_3d;
    PointT world_points_near;
    PointT world_points_far;
    int screen_x, screen_y;

    void getWorldPoint();
    // 如果 list_cloud 非空，则遍历 list_cloud，否则使用 cloud
    void getNearestPointOfRay();
};

template <typename PointT>
void CallbackArgs<PointT>::getWorldPoint() {
    if (!viewerPtr) {
        std::cerr << "Error: viewerPtr is null." << std::endl;
        return;
    }
    if (!viewerPtr->getRenderWindow()) {
        std::cerr << "Error: getRenderWindow() returned nullptr." << std::endl;
        return;
    }

    auto* rendererCollection = viewerPtr->getRenderWindow()->GetRenderers();
    if (!rendererCollection || rendererCollection->GetNumberOfItems() == 0) {
        std::cerr << "Error: Renderer collection is empty." << std::endl;
        return;
    }

    vtkRenderer* renderer = rendererCollection->GetFirstRenderer();
    if (!renderer) {
        std::cerr << "Error: Failed to get renderer." << std::endl;
        return;
    }

    if (screen_x < 0 || screen_y < 0) {
        std::cerr << "Error: Screen coordinates are invalid." << std::endl;
        return;
    }

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


template <typename PointT>
void CallbackArgs<PointT>::getNearestPointOfRay() {
    Eigen::Vector3f ray_start(world_points_near.x, world_points_near.y, world_points_near.z);
    Eigen::Vector3f ray_end(world_points_far.x, world_points_far.y, world_points_far.z);

    float minDistance = std::numeric_limits<float>::max();
    clicked_cloud_idx = -1;
    clicked_points_idx = -1;

    // 如果 list_cloud 非空，则遍历所有云；否则，使用旧的 cloud
    if (!list_cloud.empty()) {
        for (size_t cloudIdx = 0; cloudIdx < list_cloud.size(); ++cloudIdx) {
            auto &curCloud = list_cloud[cloudIdx];
            for (size_t i = 0; i < curCloud->size(); ++i) {
                Eigen::Vector3f pt(curCloud->points[i].x, curCloud->points[i].y, curCloud->points[i].z);
                float distance = ((pt - ray_start).cross(ray_end - ray_start)).squaredNorm();
                if (distance < minDistance) {
                    minDistance = distance;
                    clicked_cloud_idx = static_cast<int>(cloudIdx);
                    clicked_points_idx = static_cast<int>(i);
                    clicked_points_3d = curCloud->points[i];
                }
            }
        }
    }
    else if (cloud) {
        for (size_t i = 0; i < cloud->size(); ++i) {
            Eigen::Vector3f pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            float distance = ((pt - ray_start).cross(ray_end - ray_start)).squaredNorm();
            if (distance < minDistance) {
                minDistance = distance;
                clicked_points_idx = static_cast<int>(i);
                clicked_points_3d = cloud->points[i];
            }
        }
        // 如果只有单点云，则默认将 clicked_cloud_idx 设为 0
        clicked_cloud_idx = 0;
    }
}

// 鼠标移动回调函数（保持不变）
template <typename PointT>
void mouseMoveCallback(const pcl::visualization::MouseEvent& e, void* args) {
    auto* data = static_cast<CallbackArgs<PointT>*>(args);
    data->screen_x = e.getX();
    data->screen_y = e.getY();
}

// 点选回调函数（可根据所属点云设置不同颜色）
// 若使用多点云，则可根据 clicked_cloud_idx 来决定颜色
template <typename PointT>
void choosePointCallback(const pcl::visualization::PointPickingEvent& event, void* args) {
    auto* data = static_cast<CallbackArgs<PointT>*>(args);

    data->getWorldPoint();

    data->viewerPtr->removeShape("clicked_line");
    data->viewerPtr->addLine(data->world_points_near, data->world_points_far, "clicked_line");

    data->getNearestPointOfRay();

    // 创建一个新的点云，仅包含选中的点
    typename pcl::PointCloud<PointT>::Ptr selectedCloud(new pcl::PointCloud<PointT>);
    selectedCloud->push_back(data->clicked_points_3d);

    // 根据所属点云不同设置颜色
    int colorR = 255, colorG = 0, colorB = 0; // 默认红色
    if (!data->list_cloud.empty()) {
        if (data->clicked_cloud_idx == 0) {
            // 第一组点云显示红色
            colorR = 255; colorG = 0; colorB = 0;
        } else if (data->clicked_cloud_idx == 1) {
            // 第二组点云显示绿色
            colorR = 0; colorG = 255; colorB = 0;
        } else {
            // 其它云显示蓝色
            colorR = 0; colorG = 0; colorB = 255;
        }
    }
    pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(selectedCloud, colorR, colorG, colorB);

    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(selectedCloud, colorHandler, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
}

#endif // POINT_SELECTION_H
