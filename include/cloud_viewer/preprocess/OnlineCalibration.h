#pragma once
#include "cloud_viewer/core/CloudFrame.h"
namespace cloud_viewer {
enum class OnlineCalibrationMode { Off, TableB, Tail3LnFixed };
// 仅接受 raw_range，Cloud 原样传递；永远不修改发布后的原始帧。
FrameHandle calibrateOnlineRange(const FrameHandle& raw,
    const algo_lidar::Range6532LegacyCalibrationBin& table, OnlineCalibrationMode mode);
}
