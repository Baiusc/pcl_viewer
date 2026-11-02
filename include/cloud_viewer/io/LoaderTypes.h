/**
 * @brief 离线点云加载器接口
 *
 * 设计要点（见 doc/codex_design.md 第 5 章）：
 *  - 先 probe 再 buildIndex，最后按 FrameDescriptor 按需 decode。
 *  - CSV 按列名识别，不依赖固定列顺序。
 *  - 解析失败保留诊断信息，不静默丢整帧。
 */
#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "cloud_viewer/core/CloudFrame.h"

namespace cloud_viewer
{

enum class LoaderId
{
    Unknown = 0,
    RotatingTxt,
    SolidTxt,
    FrameCsv6532,
    FrameCsv6031,
    Pcd
};

const char *loaderIdName(LoaderId id);

struct ProbeResult
{
    LoaderId loader = LoaderId::Unknown;
    RadarModel model = RadarModel::Unknown;
    std::string formatName;
    std::string reason;

    bool recognized() const { return loader != LoaderId::Unknown; }
};

struct LoadOptions
{
    RadarModel modelHint = RadarModel::Unknown;
    bool strict = false;      ///< 严格模式：坏行直接失败
    float pcdScale = 1.0f;    ///< PCD 坐标单位换算系数
};

struct FileFrameIndex
{
    std::filesystem::path path;
    LoaderId loader = LoaderId::Unknown;
    RadarModel model = RadarModel::Unknown;
    std::vector<FrameDescriptor> frames;

    int rows = 0;
    int cols = 0;
    std::size_t dataLines = 0;
    std::size_t badLines = 0;
    std::size_t validSamples = 0;
    std::size_t invalidSamples = 0;
    std::string diagnostic;
};

class IPointCloudLoader
{
public:
    virtual ~IPointCloudLoader() = default;

    virtual LoaderId id() const = 0;

    /// 嗅探文件格式，只读少量头部内容。
    virtual ProbeResult probe(const std::filesystem::path &path) const = 0;

    /// 建立帧索引（不保留整帧点数据）。
    virtual bool buildIndex(const std::filesystem::path &path,
                            const LoadOptions &options,
                            FileFrameIndex &index,
                            std::string &error) const = 0;

    /// 按描述符解码一帧。
    virtual FrameHandle decode(const FrameDescriptor &descriptor,
                               const LoadOptions &options,
                               std::string &error) const = 0;
};

} // namespace cloud_viewer
