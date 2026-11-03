#include "cloud_viewer/preprocess/OnlineCalibration.h"
namespace cloud_viewer {
FrameHandle calibrateOnlineRange(const FrameHandle& raw,
    const algo_lidar::Range6532LegacyCalibrationBin& table, OnlineCalibrationMode mode) {
    if (!raw || mode==OnlineCalibrationMode::Off || raw->metadata.calibrationState!="raw_range" ||
        raw->metadata.roiId<0 || raw->metadata.roiId>1) return raw;
    const auto& roi=table.roi[raw->metadata.roiId];
    const auto count=std::size_t(roi.rows)*roi.cols;
    if(!count || roi.intensity_reference.size()!=count || roi.distance_correction_cm.size()!=count) return raw;
    auto matrix=raw->points;
    algo_lidar::Range6532LegacyApplyOptions options;
    options.model=mode==OnlineCalibrationMode::TableB ? algo_lidar::Range6532LegacyModel::TableBOnly
                                                    : algo_lidar::Range6532LegacyModel::Tail3LnFixed;
    algo_lidar::applyRange6532LegacyCalibration(matrix,table,static_cast<std::uint8_t>(raw->metadata.roiId),options);
    auto metadata=raw->metadata;
    metadata.calibrationState=std::string("app:")+algo_lidar::range6532LegacyModelName(options.model);
    return makeFrame(std::move(matrix),metadata);
}
}
