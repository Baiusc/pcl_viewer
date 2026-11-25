#ifndef YS_TEST_NODE_MANAGER_TEST_H_
#define YS_TEST_NODE_MANAGER_TEST_H_

#include <QtTest>
#include <QMainWindow>
#include <QApplication>
#include <QTimer>
#include <QListWidget>
#include <QListWidgetItem>

class MainWindow;
namespace ys_scene {
	class NodeManager;
}

using namespace ys_scene;

namespace ys_test {
class NodeManagerTest : public QObject {
	Q_OBJECT

private:
    // 主窗口指针
    MainWindow* mainwindow_;
    // NodeManager指针
    ys_scene::NodeManager* node_manager_;
    // 测试状态
    bool node_found_ = false;
    bool connection_tested_ = false;
    bool disconnection_tested_ = false;
    // 当前节点的IP地址
    QString current_node_ip_;

private slots:
    void initTestCase();
    void cleanupTestCase();

public slots:
    void testNodeConnection();
    void testNodeDisconnection();
};

}// namespace ys_test

#endif