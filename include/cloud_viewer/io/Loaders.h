/**
 * @brief 五类离线格式加载器与注册表
 */
#pragma once

#include <memory>
#include <vector>

#include "cloud_viewer/io/LoaderTypes.h"

namespace cloud_viewer
{

/// 转镜雷达 TXT：复用 algo_lidar::readPulseData。
class RotatingTxtLoader : public IPointCloudLoader
{
public:
    LoaderId id() const override { return LoaderId::RotatingTxt; }
    ProbeResult probe(const std::filesystem::path &path) const override;
    bool buildIndex(const std::filesystem::path &path, const LoadOptions &options,
                    FileFrameIndex &index, std::string &error) const override;
    FrameHandle decode(const FrameDescriptor &descriptor, const LoadOptions &options,
                       std::string &error) const override;
};

/// 固态雷达 TXT：复用 algo_lidar::readSolidLidarData。
class SolidTxtLoader : public IPointCloudLoader
{
public:
    LoaderId id() const override { return LoaderId::SolidTxt; }
    ProbeResult probe(const std::filesystem::path &path) const override;
    bool buildIndex(const std::filesystem::path &path, const LoadOptions &options,
                    FileFrameIndex &index, std::string &error) const override;
    FrameHandle decode(const FrameDescriptor &descriptor, const LoadOptions &options,
                       std::string &error) const override;
};

/// FL6532 Frame CSV：按列名解析，支持 frame 列多帧。
class FrameCsv6532Loader : public IPointCloudLoader
{
public:
    LoaderId id() const override { return LoaderId::FrameCsv6532; }
    ProbeResult probe(const std::filesystem::path &path) const override;
    bool buildIndex(const std::filesystem::path &path, const LoadOptions &options,
                    FileFrameIndex &index, std::string &error) const override;
    FrameHandle decode(const FrameDescriptor &descriptor, const LoadOptions &options,
                       std::string &error) const override;
};

/// FL6031 Frame CSV：复用既有按列名解析逻辑。
class FrameCsv6031Loader : public IPointCloudLoader
{
public:
    LoaderId id() const override { return LoaderId::FrameCsv6031; }
    ProbeResult probe(const std::filesystem::path &path) const override;
    bool buildIndex(const std::filesystem::path &path, const LoadOptions &options,
                    FileFrameIndex &index, std::string &error) const override;
    FrameHandle decode(const FrameDescriptor &descriptor, const LoadOptions &options,
                       std::string &error) const override;
};

/// PCD：自含 ASCII / binary 解析，不依赖 PCL IO。
class PcdLoader : public IPointCloudLoader
{
public:
    LoaderId id() const override { return LoaderId::Pcd; }
    ProbeResult probe(const std::filesystem::path &path) const override;
    bool buildIndex(const std::filesystem::path &path, const LoadOptions &options,
                    FileFrameIndex &index, std::string &error) const override;
    FrameHandle decode(const FrameDescriptor &descriptor, const LoadOptions &options,
                       std::string &error) const override;
};

class LoaderRegistry
{
public:
    static LoaderRegistry &instance();

    void registerLoader(const std::shared_ptr<IPointCloudLoader> &loader);

    /// 按注册顺序嗅探，返回首个识别的加载器；未识别时返回 nullptr。
    const IPointCloudLoader *selectLoader(const std::filesystem::path &path,
                                          const LoadOptions &options,
                                          ProbeResult &probe) const;

    bool buildIndex(const std::filesystem::path &path,
                    const LoadOptions &options,
                    FileFrameIndex &index,
                    std::string &error) const;

    FrameHandle decode(const FrameDescriptor &descriptor,
                       const LoadOptions &options,
                       const IPointCloudLoader *loader,
                       std::string &error) const;

private:
    LoaderRegistry();
    std::vector<std::shared_ptr<IPointCloudLoader>> loaders_;
};

} // namespace cloud_viewer
