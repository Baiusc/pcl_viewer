#include "mainwindow.h"
#include "business/minidump.h"
#include "business/ysc_file.h"
#include "component/ys_scene_def.h"
#include "component/crash_reporter/crash_reporter.h"
#include "business/origin_point_processor.h"
#include <cstdlib>
#include <iostream>
#include <QApplication>
#include <QIcon>
#include <QDebug>
#include <QFile>
#include <QDirIterator>
#include <QResource>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Q_INIT_RESOURCE(ys);
    a.setWindowIcon(QIcon(":/image/icons/sun.svg"));
    qRegisterMetaType<ReplayFileHeader>("ReplayFileHeader");
    qRegisterMetaType<AlgorithmItemEnable>("AlgorithmItemEnable");
    qRegisterMetaType<PointCloudROIA>("PointCloudROIA");
    //MiniDumper dump;
    // 创建CrashReporter实例
    CrashReporter reporter;

    // 设置注释信息
    std::map<std::string, std::string> annotations;
    annotations["product"] = "YSCloudScene";
    annotations["version"] = "1.0.0";
    annotations["process_type"] = "main";

    // 初始化，使用本地存储而非上传
    bool success = reporter.Initialize(
        "", // 空URL表示仅本地存储，不上传
        annotations
    );

    if (!success) {
        std::cerr << "Failed to initialize crash reporter" << std::endl;
        // 继续运行，而不是退出，因为这不是关键错误
    }
    else {
        std::cout << "Crash reporter initialized successfully" << std::endl;
    }

    // 可选：在程序运行期间添加更多注释
    reporter.AddAnnotation("app_state", "initialized");

    MainWindow w;
    w.show();
    w.PoincloudRenderInit();

    // 调试入口：设置环境变量 YS_DEBUG_ORIGIN_PORT 时直接监听原始点云口，
    // 绕过设备发现/握手，用于配合真实板端或模拟数据源离线验证接收与点云显示。
    // 例: set YS_DEBUG_ORIGIN_PORT=50001 && YSCloudScene.exe
    if (const char* debug_port = std::getenv("YS_DEBUG_ORIGIN_PORT")) {
        const int port = std::atoi(debug_port);
        if (port > 0 && port <= 65535) {
            you_see::OriginPointProcessorGetInstance().StartListening(static_cast<uint16_t>(port));
            std::cout << "[DEBUG] origin point listening on port " << port << std::endl;
        }
    }

    return a.exec();
}
