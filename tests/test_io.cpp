#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "cloud_viewer/io/Loaders.h"

namespace
{

int g_failures = 0;

void check(bool condition, const char *message)
{
    if (!condition)
    {
        std::cerr << "[FAIL] " << message << "\n";
        ++g_failures;
    }
}

} // namespace

int main()
{
    namespace fs = std::filesystem;
    const fs::path dataDir = CV_TEST_DATA_DIR;

    cloud_viewer::LoaderRegistry &registry = cloud_viewer::LoaderRegistry::instance();
    cloud_viewer::LoadOptions options;

    // FL6532 Frame CSV：嗅探、索引、解码全链路。
    const fs::path csvPath = dataDir / "Frame1_ROI0_distRP.csv";
    cloud_viewer::ProbeResult probe;
    const cloud_viewer::IPointCloudLoader *loader = registry.selectLoader(csvPath, options, probe);
    check(loader != nullptr, "6532 CSV 应被识别");
    check(probe.loader == cloud_viewer::LoaderId::FrameCsv6532, "6532 CSV 嗅探类型正确");

    if (loader)
    {
        cloud_viewer::FileFrameIndex index;
        std::string error;
        const bool indexed = loader->buildIndex(csvPath, options, index, error);
        check(indexed, "6532 CSV 应建立帧索引");
        if (indexed)
        {
            check(index.rows > 0 && index.cols > 0, "6532 索引应给出矩阵尺寸");
            check(!index.frames.empty(), "6532 索引应包含至少一帧");

            const auto frame = loader->decode(index.frames.front(), options, error);
            check(frame != nullptr, "6532 首帧应解码成功");
            if (frame)
            {
                check(frame->metadata.validSamples > 0, "6532 首帧应包含有效点");
                check(frame->metadata.cols > 0, "6532 首帧应有列数");
                check(frame->availability.has(cloud_viewer::ColorSource::Range),
                      "6532 帧应提供距离字段");
            }
        }
    }

    // 转镜 TXT：只做嗅探，避免集成测试耗时过长。
    const fs::path txtPath = dataDir / "小测试间点云" / "011小实验室_对准2块反射板.txt";
    if (fs::exists(txtPath))
    {
        cloud_viewer::ProbeResult txtProbe;
        const auto *txtLoader = registry.selectLoader(txtPath, options, txtProbe);
        check(txtLoader != nullptr, "转镜 TXT 应被识别");
        check(txtProbe.loader == cloud_viewer::LoaderId::RotatingTxt, "转镜 TXT 嗅探类型正确");
    }

    // PCD：嗅探 + 索引。
    const fs::path pcdPath = dataDir / "0811固态雷达6532.pcd";
    if (fs::exists(pcdPath))
    {
        cloud_viewer::ProbeResult pcdProbe;
        const auto *pcdLoader = registry.selectLoader(pcdPath, options, pcdProbe);
        check(pcdLoader != nullptr, "PCD 应被识别");
        check(pcdProbe.loader == cloud_viewer::LoaderId::Pcd, "PCD 嗅探类型正确");
        if (pcdLoader)
        {
            cloud_viewer::FileFrameIndex index;
            std::string error;
            check(pcdLoader->buildIndex(pcdPath, options, index, error), "PCD 应建立索引");
        }
    }

    // 未识别文件不应被误判。
    const fs::path unknownPath = dataDir / "param" / "list_r.txt";
    if (fs::exists(unknownPath))
    {
        cloud_viewer::ProbeResult unknownProbe;
        const auto *unknownLoader = registry.selectLoader(unknownPath, options, unknownProbe);
        check(unknownLoader == nullptr, "参数文本不应被误判为点云");
    }

    if (g_failures == 0)
    {
        std::cout << "[PASS] io loader tests\n";
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
