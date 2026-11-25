#include "resource_initializer.h"
#include <QFile>
#include <QDir>
#include <QDirIterator>
#include <QDebug>

// 这个函数声明在qrc自动生成的代码中，需要手动调用以初始化静态库中的资源
// extern void qInitResources_ys();

void ResourceInitializer::initialize() {
    // 检查资源状态
    checkResourceStatus();
}

bool ResourceInitializer::resourceExists(const QString& path) {
    return QFile::exists(path);
}

QStringList ResourceInitializer::listResources(const QString& directory) {
    QStringList result;
    QDirIterator it(directory, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        result << it.next();
    }
    return result;
}

void ResourceInitializer::checkResourceStatus() {
    // 检查关键资源是否可访问
    const QStringList criticalResources = {
        ":/image/icons/sun.svg",
        ":/opengl/shader/shader_mesh.vs",
        ":/Internationalization/YSCloudScene_zh_CN.qm"
    };

    qDebug() << "Checking critical resources:";
    for (const QString& res : criticalResources) {
        if (resourceExists(res)) {
            qDebug() << "  " << res << " - OK";
        }
        else {
            qWarning() << "  " << res << " - NOT FOUND";
        }
    }

    // 列出所有资源（可选，可能会输出很多内容）
    // QStringList allResources = listResources();
    // qDebug() << "All available resources:";
    // for (const QString& res : allResources) {
    //     qDebug() << "  " << res;
    // }
}