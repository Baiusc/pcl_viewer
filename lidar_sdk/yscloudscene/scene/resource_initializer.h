#ifndef RESOURCE_INITIALIZER_H
#define RESOURCE_INITIALIZER_H

#include <QString>
#include <QStringList>

/**
 * @brief 资源初始化器类
 *
 * 用于在静态库中初始化和访问Qt资源
 */
class ResourceInitializer {
public:
    /**
     * @brief 初始化资源
     */
    static void initialize();

    /**
     * @brief 检查资源是否存在
     * @param path 资源路径
     * @return 是否存在
     */
    static bool resourceExists(const QString& path);

    /**
     * @brief 列出资源目录内容
     * @param directory 目录路径
     * @return 目录内容列表
     */
    static QStringList listResources(const QString& directory = ":/");

    /**
     * @brief 检测并打印资源状态
     * 用于调试，检查关键资源是否可访问
     */
    static void checkResourceStatus();
};

#endif // RESOURCE_INITIALIZER_H