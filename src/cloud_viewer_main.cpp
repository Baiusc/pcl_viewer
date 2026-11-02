/**
 * @brief cloud_viewer Lite 主程序入口
 *
 * M0 阶段职责：建立可启动的 VTK 窗口与命令行骨架。
 * 渲染层直接使用 VTK 9，PCL 仅提供点类型与 KDTree（见 cmake/ViewerDependencies.cmake）。
 */
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <vtkAxesActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

#include "cloud_viewer/app/ViewerApp.h"
#include "cloud_viewer/core/CloudFrame.h"
#include "cloud_viewer/io/Loaders.h"
#include "cloud_viewer/stream/RadarStream.h"

namespace
{

constexpr const char *kAppName = "cloud_viewer Lite";
constexpr const char *kVersion = "0.1.0";

void printUsage()
{
    std::cout
        << kAppName << " " << kVersion << "\n"
        << "用法:\n"
        << "  cloud_viewer                     启动主界面（顶部按钮导入文件）\n"
        << "  cloud_viewer --open <file...>    直接导入指定文件（自动化/脚本）\n"
        << "  cloud_viewer --inspect <file>    仅解析并输出报告，不打开窗口\n"
        << "  cloud_viewer --screenshot <png>  启动并保存界面截图\n"
        << "  cloud_viewer --help              显示本帮助\n"
        << "  cloud_viewer --version           显示版本\n";
}

std::string jsonEscape(const std::string &text)
{
    std::string out;
    out.reserve(text.size() + 8);
    for (char ch : text)
    {
        switch (ch)
        {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            out.push_back(ch);
            break;
        }
    }
    return out;
}

int runInspect(const std::string &file, const std::string &reportPath)
{
    namespace fs = std::filesystem;
    const fs::path path(file);
    if (!fs::exists(path))
    {
        std::cerr << "[cloud_viewer] 文件不存在: " << file << "\n";
        return 6;
    }

    cloud_viewer::LoadOptions options;
    cloud_viewer::LoaderRegistry &registry = cloud_viewer::LoaderRegistry::instance();

    cloud_viewer::ProbeResult probe;
    const cloud_viewer::IPointCloudLoader *loader = registry.selectLoader(path, options, probe);
    if (!loader)
    {
        std::cerr << "[cloud_viewer] 无法识别文件格式: " << file << "\n";
        return 7;
    }

    cloud_viewer::FileFrameIndex index;
    std::string error;
    if (!registry.buildIndex(path, options, index, error))
    {
        std::cerr << "[cloud_viewer] 建立帧索引失败: " << error << "\n";
        return 8;
    }

    // 解码首帧，验证索引与解码链路可用。
    std::size_t decodedValid = 0;
    std::size_t decodedDisplayed = 0;
    std::string decodeError;
    if (const auto frame = registry.decode(index.frames.front(), options, loader, decodeError))
    {
        decodedValid = frame->metadata.validSamples;
        decodedDisplayed = frame->metadata.totalSamples;
    }

    std::ostringstream json;
    json << "{\n";
    json << "  \"file\": \"" << jsonEscape(path.string()) << "\",\n";
    json << "  \"loader\": \"" << cloud_viewer::loaderIdName(index.loader) << "\",\n";
    json << "  \"model\": \"" << cloud_viewer::radarModelName(index.model) << "\",\n";
    json << "  \"format\": \"" << jsonEscape(probe.formatName) << "\",\n";
    json << "  \"reason\": \"" << jsonEscape(probe.reason) << "\",\n";
    json << "  \"rows\": " << index.rows << ",\n";
    json << "  \"cols\": " << index.cols << ",\n";
    json << "  \"frameCount\": " << index.frames.size() << ",\n";
    json << "  \"dataLines\": " << index.dataLines << ",\n";
    json << "  \"badLines\": " << index.badLines << ",\n";
    json << "  \"validSamples\": " << index.validSamples << ",\n";
    json << "  \"invalidSamples\": " << index.invalidSamples << ",\n";
    json << "  \"decodedFirstFrameValidSamples\": " << decodedValid << ",\n";
    json << "  \"decodedFirstFrameTotalSamples\": " << decodedDisplayed << ",\n";
    json << "  \"decodeError\": \"" << jsonEscape(decodeError) << "\",\n";
    json << "  \"frames\": [";
    for (std::size_t i = 0; i < index.frames.size(); ++i)
    {
        const auto &descriptor = index.frames[i];
        if (i != 0)
        {
            json << ",";
        }
        json << "{\"id\": " << descriptor.id
             << ", \"frame\": " << descriptor.frameNumber
             << ", \"roi\": " << descriptor.roiId << "}";
    }
    json << "]\n}\n";

    if (!reportPath.empty())
    {
        std::ofstream output(reportPath);
        if (!output.is_open())
        {
            std::cerr << "[cloud_viewer] 无法写入报告: " << reportPath << "\n";
            return 9;
        }
        output << json.str();
        std::cout << "[cloud_viewer] 报告已写入: " << reportPath << "\n";
    }
    else
    {
        std::cout << json.str();
    }
    return 0;
}

} // namespace

int main(int argc, char **argv)
{
    std::vector<std::string> args(argv + 1, argv + argc);

    if (args.empty())
    {
        // 默认进入主界面：空场景 + 顶部按钮，由用户点击导入文件。
        cloud_viewer::ViewerApp app;
        return app.run({});
    }

    const std::string &first = args.front();
    if (first == "--help" || first == "-h")
    {
        printUsage();
        return 0;
    }
    if (first == "--version" || first == "-v")
    {
        std::cout << kVersion << "\n";
        return 0;
    }
    if (first == "--open" || first == "--source" || first == "--screenshot")
    {
        std::vector<std::string> files;
        std::string source = "offline";
        std::string screenshotPath;
        double fps = 0.0;

        for (std::size_t i = 0; i < args.size(); ++i)
        {
            if (args[i] == "--open")
            {
                ++i;
                while (i < args.size() && args[i].rfind("--", 0) != 0)
                {
                    files.push_back(args[i]);
                    ++i;
                }
                --i;
            }
            else if (args[i] == "--source" && i + 1 < args.size())
            {
                source = args[++i];
            }
            else if (args[i] == "--fps" && i + 1 < args.size())
            {
                try
                {
                    fps = std::stod(args[++i]);
                }
                catch (...)
                {
                    fps = 0.0;
                }
            }
            else if (args[i] == "--screenshot" && i + 1 < args.size())
            {
                screenshotPath = args[++i];
            }
        }

        const bool mock = (source == "mock_replay" || source == "mock_synthetic");
        if (source != "offline" &&
            !cloud_viewer::RadarBackendFactory::instance().isAvailable(source))
        {
            const auto *backend = cloud_viewer::RadarBackendFactory::instance().find(source);
            std::cerr << "[cloud_viewer] 后端未启用: " << source;
            if (backend)
            {
                std::cerr << " (" << backend->displayName << ")";
            }
            std::cerr << "，编译期未安装对应 SDK，返回 BackendUnavailable\n";
            return 13;
        }
        if (files.empty() && first != "--screenshot")
        {
            std::cerr << "[cloud_viewer] " << first << " 需要至少一个文件。\n";
            printUsage();
            return 2;
        }

        cloud_viewer::ViewerApp app;
        return app.run(files, mock ? (fps > 0.0 ? fps : 20.0) : 0.0, screenshotPath);
    }
    if (first == "--inspect")
    {
        if (args.size() < 2)
        {
            printUsage();
            return 2;
        }
        std::string reportPath;
        for (std::size_t i = 2; i + 1 < args.size(); ++i)
        {
            if (args[i] == "--report")
            {
                reportPath = args[i + 1];
            }
        }
        return runInspect(args[1], reportPath);
    }

    std::cerr << "[cloud_viewer] 未知参数: " << first << "\n";
    printUsage();
    return 2;
}
