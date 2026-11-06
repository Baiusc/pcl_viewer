#include "cloud_viewer/core/CloudFrame.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cloud_viewer
{
namespace
{

bool isFiniteCoordinate(float value)
{
    return std::isfinite(value);
}

} // namespace

const char *radarModelName(RadarModel model)
{
    switch (model)
    {
    case RadarModel::AlphaRotating:
        return "alpha_rotating";
    case RadarModel::BetaRotating:
        return "beta_rotating";
    case RadarModel::GammaRotating:
        return "gamma_rotating";
    case RadarModel::Fl6031:
        return "fl6031";
    case RadarModel::Fl6532:
        return "fl6532";
    case RadarModel::Generic:
        return "generic";
    case RadarModel::Unknown:
    default:
        return "unknown";
    }
}

const char *colorSourceName(ColorSource source)
{
    switch (source)
    {
    case ColorSource::Range:
        return "r";
    case ColorSource::X:
        return "x";
    case ColorSource::Y:
        return "y";
    case ColorSource::Z:
        return "z";
    case ColorSource::Intensity:
        return "intensity";
    case ColorSource::Reflectivity:
        return "reflect";
    case ColorSource::Peak:
        return "peak";
    case ColorSource::Timestamp:
        return "timestamp";
    case ColorSource::Theta:
        return "theta";
    case ColorSource::SlotAngle:
        return "slot_ang";
    case ColorSource::EchoIndex:
        return "echo_index";
    case ColorSource::ClusterId:
        return "cluster_id";
    case ColorSource::IsTailPoint:
        return "is_tail_pt";
    case ColorSource::IsValid:
        return "is_valid";
    case ColorSource::SolidColor:
    default:
        return "solid";
    }
}

const char *colorSourceDisplayName(ColorSource source)
{
    switch (source)
    {
    case ColorSource::Range:
        return "距离 r";
    case ColorSource::X:
        return "X";
    case ColorSource::Y:
        return "Y";
    case ColorSource::Z:
        return "Z";
    case ColorSource::Intensity:
        return "强度 intensity";
    case ColorSource::Reflectivity:
        return "反射率 reflect";
    case ColorSource::Peak:
        return "峰值 peak";
    case ColorSource::Timestamp:
        return "时间戳";
    case ColorSource::Theta:
        return "俯仰角 theta";
    case ColorSource::SlotAngle:
        return "水平角 slot_ang";
    case ColorSource::EchoIndex:
        return "回波序号";
    case ColorSource::ClusterId:
        return "簇 id";
    case ColorSource::IsTailPoint:
        return "托尾点标记";
    case ColorSource::IsValid:
        return "有效标记";
    case ColorSource::SolidColor:
    default:
        return "固定色";
    }
}

void rebuildDerivedFields(CloudFrame &frame, std::optional<ColorSource> only)
{
    // 给了 only 时只生成该颜色源：在线点云每帧 30 万点时，
    // 13 个标量场全部填充要 400 万次写入 + 15MB 分配，是掉帧的主要原因之一。
    const bool all = !only.has_value();
    const auto want = [all, only](ColorSource source)
    {
        return all || *only == source;
    };

    frame.availability.sources.clear();

    const std::size_t rows = frame.points.size();
    const std::size_t cols = rows > 0 ? frame.points.front().size() : 0;
    const std::size_t total = rows * cols;

    frame.metadata.rows = static_cast<int>(rows);
    frame.metadata.cols = static_cast<int>(cols);
    frame.metadata.totalSamples = total;

    // 结构体中始终存在的字段先标记为可用。
    frame.availability.sources.insert(ColorSource::Range);
    frame.availability.sources.insert(ColorSource::X);
    frame.availability.sources.insert(ColorSource::Y);
    frame.availability.sources.insert(ColorSource::Z);
    frame.availability.sources.insert(ColorSource::Intensity);
    frame.availability.sources.insert(ColorSource::Reflectivity);
    frame.availability.sources.insert(ColorSource::Peak);
    frame.availability.sources.insert(ColorSource::Theta);
    frame.availability.sources.insert(ColorSource::SlotAngle);
    frame.availability.sources.insert(ColorSource::EchoIndex);
    frame.availability.sources.insert(ColorSource::IsTailPoint);
    frame.availability.sources.insert(ColorSource::IsValid);
    frame.availability.sources.insert(ColorSource::SolidColor);

    ScalarField rangeField;
    ScalarField xField;
    ScalarField yField;
    ScalarField zField;
    ScalarField intensityField;
    ScalarField reflectField;
    ScalarField peakField;
    ScalarField thetaField;
    ScalarField slotAngleField;
    ScalarField echoIndexField;
    ScalarField tailField;
    ScalarField validField;
    ScalarField timestampField;

    if (want(ColorSource::Range)) rangeField.resize(total);
    if (want(ColorSource::X)) xField.resize(total);
    if (want(ColorSource::Y)) yField.resize(total);
    if (want(ColorSource::Z)) zField.resize(total);
    if (want(ColorSource::Intensity)) intensityField.resize(total);
    if (want(ColorSource::Reflectivity)) reflectField.resize(total);
    if (want(ColorSource::Peak)) peakField.resize(total);
    if (want(ColorSource::Theta)) thetaField.resize(total);
    if (want(ColorSource::SlotAngle)) slotAngleField.resize(total);
    if (want(ColorSource::EchoIndex)) echoIndexField.resize(total);
    if (want(ColorSource::IsTailPoint)) tailField.resize(total);
    if (want(ColorSource::IsValid)) validField.resize(total);
    if (want(ColorSource::Timestamp)) timestampField.resize(total);

    std::size_t validCount = 0;
    bool hasTimestamp = false;

    for (std::size_t r = 0; r < rows; ++r)
    {
        for (std::size_t c = 0; c < cols; ++c)
        {
            const algo_lidar::PointData &point = frame.points[r][c];
            const std::size_t idx = r * cols + c;
            const bool valid = point.is_valid;

            if (valid)
            {
                ++validCount;
            }

            if (want(ColorSource::Range))
            {
                rangeField.values[idx] = point.r;
                rangeField.valid[idx] = valid ? 1u : 0u;
            }

            if (want(ColorSource::X))
            {
                xField.values[idx] = point.x;
                xField.valid[idx] = (valid && isFiniteCoordinate(point.x)) ? 1u : 0u;
            }
            if (want(ColorSource::Y))
            {
                yField.values[idx] = point.y;
                yField.valid[idx] = (valid && isFiniteCoordinate(point.y)) ? 1u : 0u;
            }
            if (want(ColorSource::Z))
            {
                zField.values[idx] = point.z;
                zField.valid[idx] = (valid && isFiniteCoordinate(point.z)) ? 1u : 0u;
            }

            if (want(ColorSource::Intensity))
            {
                intensityField.values[idx] = static_cast<float>(point.intensity);
                intensityField.valid[idx] = valid ? 1u : 0u;
            }

            if (want(ColorSource::Reflectivity))
            {
                reflectField.values[idx] = point.reflect;
                reflectField.valid[idx] = valid ? 1u : 0u;
            }

            if (want(ColorSource::Peak))
            {
                peakField.values[idx] = static_cast<float>(point.peak);
                peakField.valid[idx] = valid ? 1u : 0u;
            }

            if (want(ColorSource::Theta))
            {
                thetaField.values[idx] = point.theta;
                thetaField.valid[idx] = valid ? 1u : 0u;
            }

            if (want(ColorSource::SlotAngle))
            {
                slotAngleField.values[idx] = point.slot_ang;
                slotAngleField.valid[idx] = valid ? 1u : 0u;
            }

            if (want(ColorSource::EchoIndex))
            {
                echoIndexField.values[idx] = static_cast<float>(point.select_idx);
                echoIndexField.valid[idx] = valid ? 1u : 0u;
            }

            if (want(ColorSource::IsTailPoint))
            {
                tailField.values[idx] = point.is_tail_pt ? 1.0f : 0.0f;
                tailField.valid[idx] = valid ? 1u : 0u;
            }

            if (want(ColorSource::IsValid))
            {
                validField.values[idx] = valid ? 1.0f : 0.0f;
                validField.valid[idx] = 1u;
            }

            if (want(ColorSource::Timestamp))
            {
                timestampField.values[idx] = static_cast<float>(point.timestamp);
                timestampField.valid[idx] = valid ? 1u : 0u;
                if (point.timestamp != 0)
                {
                    hasTimestamp = true;
                }
            }
        }
    }

    frame.metadata.validSamples = validCount;
    frame.metadata.invalidSamples = total - validCount;

    if (want(ColorSource::Range)) frame.scalarFields[ColorSource::Range] = std::move(rangeField);
    if (want(ColorSource::X)) frame.scalarFields[ColorSource::X] = std::move(xField);
    if (want(ColorSource::Y)) frame.scalarFields[ColorSource::Y] = std::move(yField);
    if (want(ColorSource::Z)) frame.scalarFields[ColorSource::Z] = std::move(zField);
    if (want(ColorSource::Intensity))
        frame.scalarFields[ColorSource::Intensity] = std::move(intensityField);
    if (want(ColorSource::Reflectivity))
        frame.scalarFields[ColorSource::Reflectivity] = std::move(reflectField);
    if (want(ColorSource::Peak)) frame.scalarFields[ColorSource::Peak] = std::move(peakField);
    if (want(ColorSource::Theta)) frame.scalarFields[ColorSource::Theta] = std::move(thetaField);
    if (want(ColorSource::SlotAngle))
        frame.scalarFields[ColorSource::SlotAngle] = std::move(slotAngleField);
    if (want(ColorSource::EchoIndex))
        frame.scalarFields[ColorSource::EchoIndex] = std::move(echoIndexField);
    if (want(ColorSource::IsTailPoint))
        frame.scalarFields[ColorSource::IsTailPoint] = std::move(tailField);
    if (want(ColorSource::IsValid))
        frame.scalarFields[ColorSource::IsValid] = std::move(validField);

    // 时间戳为 0 视为缺失：只有存在非零样本时才把该字段标记为可用。
    if (hasTimestamp)
    {
        frame.scalarFields[ColorSource::Timestamp] = std::move(timestampField);
        frame.availability.sources.insert(ColorSource::Timestamp);
    }
    else
    {
        frame.scalarFields.erase(ColorSource::Timestamp);
        frame.metadata.timestampUnit = "unavailable";
    }

    // 簇 id 需要外部 sidecar 提供，缺失时不可用，不启动聚类补值。
    frame.scalarFields.erase(ColorSource::ClusterId);
}

std::shared_ptr<CloudFrame> makeFrame(algo_lidar::PointMatrix points,
                                      FrameMetadata metadata,
                                      FrameId id,
                                      std::optional<ColorSource> only)
{
    auto frame = std::make_shared<CloudFrame>();
    frame->id = id;
    frame->revision = 1;
    frame->points = std::move(points);
    frame->metadata = std::move(metadata);
    rebuildDerivedFields(*frame, only);
    return frame;
}

} // namespace cloud_viewer
