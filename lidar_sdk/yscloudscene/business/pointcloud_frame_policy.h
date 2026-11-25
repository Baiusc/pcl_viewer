#ifndef YS_SCENE_POINTCLOUD_FRAME_POLICY_H_
#define YS_SCENE_POINTCLOUD_FRAME_POLICY_H_

#include <cstddef>

namespace ys_scene {
// 点云成帧策略(纯函数，供 business/pointcloud_parser.cpp 使用，同时作为单元测试切入点)：
// 每 SLOT 点数由 SDK 按 UDP 包长解析得到(1266→96、2514→192、1578→120)，
// 上层不再假定 96 像素，alpha/beta 与 CS96-gamma 共用同一套成帧判定。
namespace frame_policy {

// 超时帧接纳/乱序帧弹出的最小可用数据量比例(沿用既有 alpha/beta 阈值)
constexpr double kMinFrameDataRatio = 0.92;

// 整帧期望点数 = 期望 SLOT 数 × 每 SLOT 点数
inline size_t ExpectedFramePoints(int points_per_slot, int expected_slot_count)
{
    return static_cast<size_t>(points_per_slot) * static_cast<size_t>(expected_slot_count);
}

// 最小可用点数(整帧期望的 92%)：用于超时帧的不完整接纳判定
inline int MinAvailableDataSize(int points_per_slot, int expected_slot_count,
    double ratio = kMinFrameDataRatio)
{
    return static_cast<int>(ExpectedFramePoints(points_per_slot, expected_slot_count) * ratio);
}

// 完整帧必须同时收满点数和去重 SLOT 数；丢包帧由超时路径按 92% 阈值接纳。
inline bool IsFrameComplete(size_t accumulated_points, size_t unique_slots, size_t expected_points,
    int expected_slot_count)
{
    return expected_slot_count > 0 && expected_points > 0 && accumulated_points >= expected_points &&
        unique_slots >= static_cast<size_t>(expected_slot_count);
}

} // namespace frame_policy
} // namespace ys_scene

#endif // YS_SCENE_POINTCLOUD_FRAME_POLICY_H_
