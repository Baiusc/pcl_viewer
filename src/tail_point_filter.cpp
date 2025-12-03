#include "tail_point_filter.h"

#include <math.h>
#include <iostream>

namespace algo_lidar {
TailPointFilter::TailPointFilter(/* args */) {}

bool TailPointFilter::Init(std::string yaml_path) {
    return false;
}

void TailPointFilter::Process(std::vector<PointData>& row_data) {
    if (row_data.empty()) {
        return;
    }

    const size_t pt_size = row_data.size();
    if (pt_size < 3) {
        std::cout << "too few point" << std::endl;
        return;
    }

    for (size_t i = 1; i < pt_size - 1; i++) {
        const auto left_pt = row_data[i - 1];
        auto& cur_pt = row_data[i];
        const auto right_pt = row_data[i + 1];

        // 必须保证连续相邻的三个点都有效
        if (!left_pt.is_valid || !cur_pt.is_valid || !right_pt.is_valid) {
            continue;
        }

        // 当前段距离左边相邻点和右边相邻点的距离
        double dis_left = (cur_pt.x - left_pt.x) * (cur_pt.x - left_pt.x) + (cur_pt.y - left_pt.y) * (cur_pt.y - left_pt.y) + (cur_pt.z - left_pt.z) * (cur_pt.z - left_pt.z);
        double dis_right = (cur_pt.x - right_pt.x) * (cur_pt.x - right_pt.x) + (cur_pt.y - right_pt.y) * (cur_pt.y - right_pt.y) + (cur_pt.z - right_pt.z) * (cur_pt.z - right_pt.z);

        // 激光雷达远点、坐标相邻点，当前点形成三角形，雷达原点所在三角形内角大小
        double mag_AB1 = std::sqrt(left_pt.x * left_pt.x + left_pt.y * left_pt.y + left_pt.z * left_pt.z);
        double mag_AB2 = std::sqrt(right_pt.x * right_pt.x + right_pt.y * right_pt.y + right_pt.z * right_pt.z);
        double mag_AC = std::sqrt(cur_pt.x * cur_pt.x + cur_pt.y * cur_pt.y + cur_pt.z * cur_pt.z);

        double AB1_dot_AC = left_pt.x * cur_pt.x + left_pt.y * cur_pt.y + left_pt.z * cur_pt.z;
        double AB2_dot_AC = right_pt.x * cur_pt.x + right_pt.y * cur_pt.y + right_pt.z * cur_pt.z;
        double angle_left = std::acos(AB1_dot_AC / (mag_AB1 * mag_AC));
        double angle_right = std::acos(AB2_dot_AC / (mag_AB2 * mag_AC));

        // std::cout << "dis left:" << dis_left << ", right:" << dis_right << ", ang left:" << angle_left << ", right:" << angle_right << std::endl;

        if( cur_pt.pixel_idx == 65 && cur_pt.slot_idx == 380 ) {
            std::cout << "debug point at pixel_idx: " << cur_pt.pixel_idx
                      << ", slot_idx: " << cur_pt.slot_idx
                      << ", r: " << cur_pt.r
                      << ", peak: " << cur_pt.peak
                      << ", dis_left: " << dis_left
                      << ", dis_right: " << dis_right
                      << ", angle_left: " << angle_left
                      << ", angle_right: " << angle_right
                      << std::endl;
        }
        if (dis_left > 0.1 * 0.1 && dis_right > 0.1 * 0.1 &&
            angle_left < 0.15 && angle_right < 0.15) {
            cur_pt.is_tail_pt = true;
            // debug 打印
            std::cout << "tail point found at pixel_idx: " << cur_pt.pixel_idx
                      << ", slot_idx: " << cur_pt.slot_idx
                      << ", r: " << cur_pt.r
                      << ", peak: " << cur_pt.peak << std::endl;
        }
    }

    return;
}
}  // namespace algo_lidar