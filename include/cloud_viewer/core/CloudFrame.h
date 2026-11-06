/**
 * @brief cloud_viewer 统一帧模型
 *
 * 设计要点（见 doc/codex_design.md 第 3 章）：
 *  - 复用 algo_lidar::PointData / PointMatrix / Pulse / Converter，不另建点类型。
 *  - CloudFrame 只增加元数据、字段可用性与标量 sidecar，属于容器而非另一套点数据。
 *  - 发布后通过 shared_ptr<const CloudFrame> 传递，避免显示过程中被改写。
 */
#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "point_type.h"

namespace cloud_viewer
{

using FrameId = std::uint64_t;
using SequenceId = std::uint64_t;
using SceneObjectId = std::uint64_t;

/// 支持的雷达型号。TXT 无法区分 alpha/beta 时由用户配置指定。
enum class RadarModel
{
    Unknown = 0,
    AlphaRotating,
    BetaRotating,
    /// CS120 Gamma：SL100 的 dToF SPAD，120×800。沿用 Beta 的解析链路与 UI，
    /// 差异只在规格（120 线/包长）与面号策略（设备不做 PA 纠偏）。
    GammaRotating,
    Fl6031,
    Fl6532,
    Generic
};

const char *radarModelName(RadarModel model);

/// 转镜雷达（Alpha / Beta / Gamma）共用同一套解析、面选择与标定链路。
inline bool isRotatingModel(RadarModel model)
{
    return model == RadarModel::AlphaRotating || model == RadarModel::BetaRotating ||
           model == RadarModel::GammaRotating;
}

/// 着色源。字段名与界面显示名一一对应。
enum class ColorSource
{
    Range = 0,
    X,
    Y,
    Z,
    Intensity,
    Reflectivity,
    Peak,
    Timestamp,
    Theta,
    SlotAngle,
    EchoIndex,
    ClusterId,
    IsTailPoint,
    IsValid,
    SolidColor
};

const char *colorSourceName(ColorSource source);
const char *colorSourceDisplayName(ColorSource source);

/// 单点标量场。valid 与 values 等长，0 表示缺失，不能用 0 值代替缺失。
struct ScalarField
{
    std::vector<float> values;
    std::vector<std::uint8_t> valid;

    bool empty() const { return values.empty(); }
    std::size_t size() const { return values.size(); }

    void resize(std::size_t count)
    {
        values.assign(count, 0.0f);
        valid.assign(count, 0u);
    }
};

struct FieldAvailability
{
    std::set<ColorSource> sources;

    bool has(ColorSource source) const { return sources.count(source) != 0; }
};

struct FrameMetadata
{
    std::filesystem::path sourcePath;
    RadarModel model = RadarModel::Unknown;
    std::uint64_t frameNumber = 0;
    int roiId = -1;
    std::string algorithmId;
    double timestampOrigin = 0.0;
    std::string timestampUnit = "raw";
    std::string clockDomain = "unknown";
    int rows = 0;
    int cols = 0;
    std::size_t totalSamples = 0;
    std::size_t validSamples = 0;
    std::size_t invalidSamples = 0;
    bool partial = false;   ///< 文件结束处的不完整帧
    bool sparse = false;    ///< 受限导入/非组织化
    std::string calibrationState = "none";
};

struct CloudFrame
{
    FrameId id = 0;
    std::uint64_t revision = 0;
    algo_lidar::PointMatrix points;
    FrameMetadata metadata;
    FieldAvailability availability;
    std::map<ColorSource, ScalarField> scalarFields;
};

using FrameHandle = std::shared_ptr<const CloudFrame>;

/// 建立一帧，并填充统计与派生标量场。
/// only != Unknown 时只生成该颜色源的标量场：在线点云每帧 30 万点，
/// 13 个场全填要 400 万次写入 + 15MB 分配，是掉帧主因之一。
std::shared_ptr<CloudFrame> makeFrame(algo_lidar::PointMatrix points,
                                      FrameMetadata metadata,
                                      FrameId id = 0,
                                      std::optional<ColorSource> only = std::nullopt);

/// 重新计算可用字段与标量 sidecar（原地）。only 同上。
void rebuildDerivedFields(CloudFrame &frame, std::optional<ColorSource> only = std::nullopt);

/// 多帧序列描述。不永久持有整个 PointMatrix，只保存定位信息。
struct FrameDescriptor
{
    FrameId id = 0;
    std::filesystem::path sourcePath;
    std::uint64_t frameNumber = 0;
    int roiId = -1;
    std::string algorithmId;
    std::uint64_t byteOffset = 0;
    bool partial = false;
};

struct CloudFrameSequence
{
    SequenceId id = 0;
    std::vector<FrameDescriptor> entries;
    std::size_t current = 0;
    double playbackRate = 1.0;
    bool loop = false;

    std::size_t frameCount() const { return entries.size(); }
    bool empty() const { return entries.empty(); }
};

} // namespace cloud_viewer
