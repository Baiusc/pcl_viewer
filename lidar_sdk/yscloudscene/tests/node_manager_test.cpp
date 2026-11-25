#include "node_manager_test.h"

#include "mainwindow.h"
#include "component/node_manager/node_manager.h"
#include "component/switch_button/switch_button.h"
#include "component/ys_scene_def.h"
#include "business/ysc_file.h"
#include <QResource>
#include <QEventLoop>

namespace ys_test {
void NodeManagerTest::initTestCase()
{
    Q_INIT_RESOURCE(ys);
    // 注册元类型
    qRegisterMetaType<ReplayFileHeader>("ReplayFileHeader");
    qRegisterMetaType<AlgorithmItemEnable>("AlgorithmItemEnable");
    qRegisterMetaType<PointCloudROIA>("PointCloudROIA");

    // 创建主窗口
    mainwindow_ = new MainWindow();
    QVERIFY(mainwindow_ != nullptr);

    // 显示主窗口
    mainwindow_->show();
    QTest::qWaitForWindowExposed(mainwindow_);

    // 添加调试信息
    qDebug() << "窗口已显示，窗口状态:" << (mainwindow_->isVisible() ? "可见" : "不可见");
    qDebug() << "窗口位置:" << mainwindow_->geometry();

    // 找到NodeManager实例
    node_manager_ = mainwindow_->findChild<ys_scene::NodeManager*>();
    QVERIFY(node_manager_ != nullptr);

    // 初始化点云渲染 - 这可能很耗时，设置长超时
    QTimer::singleShot(500, mainwindow_, &MainWindow::PoincloudRenderInit);

    // 等待程序完全初始化
    QTest::qWait(5000);

    // 再次确认窗口可见性
    qDebug() << "初始化后窗口状态:" << (mainwindow_->isVisible() ? "可见" : "不可见");

    // 强制保持窗口显示
    mainwindow_->raise();
    mainwindow_->activateWindow();

    // 设置信号监视器来捕获连接状态变化
    QSignalSpy spy(node_manager_, SIGNAL(LidarConnectionChanged(const uint32_t&, bool)));

    // 设置定时器检查是否有节点出现
    QTimer* checkTimer = new QTimer(this);
    connect(checkTimer, &QTimer::timeout, this, [this, checkTimer]() {
        // 确保窗口仍然可见
        if (!mainwindow_->isVisible()) {
            qDebug() << "窗口变为不可见，尝试重新显示";
            mainwindow_->show();
            mainwindow_->raise();
        }

        // 找到ListWidget
        QListWidget* listWidget = node_manager_->findChild<QListWidget*>();
        if (!listWidget || listWidget->count() == 0) {
            qDebug() << "等待节点出现...";
            return;
        }

        // 节点已找到
        node_found_ = true;
        checkTimer->stop();

        // 记录节点IP
        QListWidgetItem* item = listWidget->item(0);
        current_node_ip_ = item->data(Qt::UserRole).toString();
        qDebug() << "找到节点：" << current_node_ip_;

        // 继续测试
        QTimer::singleShot(1000, this, SLOT(testNodeConnection()));
        QTimer::singleShot(2000, this, SLOT(testNodeConnection()));
        QTimer::singleShot(3000, this, SLOT(testNodeConnection()));
        QTimer::singleShot(4000, this, SLOT(testNodeConnection()));
        QTimer::singleShot(5000, this, SLOT(testNodeConnection()));
        });

    // 开始检查是否有节点
    checkTimer->start(1000);

    // 输出等待消息
    qDebug() << "测试启动，等待节点发现...";
    // 在initTestCase中添加
    QEventLoop* eventLoop = nullptr;
    eventLoop = new QEventLoop(this);
    // 使用定时器保持事件循环运行，但不阻塞测试
    //QTimer::singleShot(0, this, [this]() {
    //    if (eventLoop && eventLoop->isRunning())
    //        eventLoop->quit();
    //    });
    eventLoop->exec();
}

void NodeManagerTest::cleanupTestCase()
{
    // 如果已连接，确保断开连接
    QListWidget* listWidget = node_manager_->findChild<QListWidget*>();
    if (listWidget && listWidget->count() > 0) {
        for (int i = 0; i < listWidget->count(); ++i) {
            QWidget* container = listWidget->itemWidget(listWidget->item(i));
            if (!container) continue;

            SwitchButton* switchBtn = container->findChild<SwitchButton*>();
            if (!switchBtn) continue;

            if (switchBtn->getState() == SwitchButton::State::On) {
                QTest::mouseClick(switchBtn, Qt::LeftButton);
                QTest::qWait(2000);
            }
        }
    }

    // 关闭主窗口
    mainwindow_->close();
    delete mainwindow_;

    // 输出测试结果
    if (node_found_) {
        qDebug() << "节点找到: " << (node_found_ ? "是" : "否");
        qDebug() << "连接测试: " << (connection_tested_ ? "通过" : "失败");
        qDebug() << "断开测试: " << (disconnection_tested_ ? "通过" : "失败");
    }
    else {
        qDebug() << "测试失败: 未找到节点";
    }
}

void NodeManagerTest::testNodeConnection()
{
    if (!node_found_) {
        QFAIL("未找到节点，无法测试连接功能");
        return;
    }

    // 找到ListWidget
    QListWidget* listWidget = node_manager_->findChild<QListWidget*>();
    QVERIFY(listWidget != nullptr);

    // 找到节点项
    QListWidgetItem* item = nullptr;
    for (int i = 0; i < listWidget->count(); ++i) {
        if (listWidget->item(i)->data(Qt::UserRole).toString() == current_node_ip_) {
            item = listWidget->item(i);
            break;
        }
    }
    QVERIFY(item != nullptr);

    // 获取节点项的容器widget
    QWidget* container = listWidget->itemWidget(item);
    QVERIFY(container != nullptr);

    // 找到SwitchButton
    SwitchButton* switchBtn = container->findChild<SwitchButton*>();
    QVERIFY(switchBtn != nullptr);

    // 设置信号监视器
    QSignalSpy spy(node_manager_, SIGNAL(LidarConnectionChanged(const uint32_t&, bool)));

    // 检查当前状态
    if (switchBtn->getState() == SwitchButton::State::On) {
        qDebug() << "节点已经处于连接状态，先断开";
        QTest::mouseClick(switchBtn, Qt::LeftButton);
        QTest::qWait(3000);
    }

    // 验证初始状态是Off
    QCOMPARE(switchBtn->getState(), SwitchButton::State::Off);

    // 点击SwitchButton打开连接
    qDebug() << "点击按钮连接节点";
    QTest::mouseClick(switchBtn, Qt::LeftButton);

    // 等待连接建立
    qDebug() << "等待连接建立...";
    for (int i = 0; i < 30; ++i) {
        QTest::qWait(500);
        if (switchBtn->getState() == SwitchButton::State::On) {
            break;
        }
    }

    // 验证按钮状态变为On
    if (switchBtn->getState() == SwitchButton::State::On) {
        qDebug() << "连接成功！按钮状态已变为ON";
        connection_tested_ = true;
    }
    else {
        qDebug() << "连接失败！按钮状态仍为OFF";
        QFAIL("连接节点失败，按钮状态未改变");
        return;
    }

    // 验证信号已发射
    if (spy.count() > 0) {
        QList<QVariant> arguments = spy.takeFirst();
        uint32_t ip = arguments.at(0).toUInt();
        qDebug() << "捕获到LidarConnectionChanged信号: IP=" << node_manager_->Ip2StringT(ip) << ", enabled=" << arguments.at(1).toBool();
    }

    // 继续测试断开连接
    QTimer::singleShot(3000, this, SLOT(testNodeDisconnection()));
}

void NodeManagerTest::testNodeDisconnection()
{
    if (!connection_tested_) {
        QFAIL("连接测试未通过，无法测试断开功能");
        return;
    }

    // 找到ListWidget
    QListWidget* listWidget = node_manager_->findChild<QListWidget*>();
    QVERIFY(listWidget != nullptr);

    // 找到节点项
    QListWidgetItem* item = nullptr;
    for (int i = 0; i < listWidget->count(); ++i) {
        if (listWidget->item(i)->data(Qt::UserRole).toString() == current_node_ip_) {
            item = listWidget->item(i);
            break;
        }
    }
    QVERIFY(item != nullptr);

    // 获取节点项的容器widget
    QWidget* container = listWidget->itemWidget(item);
    QVERIFY(container != nullptr);

    // 找到SwitchButton
    SwitchButton* switchBtn = container->findChild<SwitchButton*>();
    QVERIFY(switchBtn != nullptr);

    // 验证当前状态是On
    QCOMPARE(switchBtn->getState(), SwitchButton::State::On);

    // 设置信号监视器
    QSignalSpy spy(node_manager_, SIGNAL(LidarConnectionChanged(const uint32_t&, bool)));

    // 点击SwitchButton断开连接
    qDebug() << "点击按钮断开节点";
    QTest::mouseClick(switchBtn, Qt::LeftButton);

    // 等待断开连接
    qDebug() << "等待断开连接...";
    for (int i = 0; i < 30; ++i) {
        QTest::qWait(500);
        if (switchBtn->getState() == SwitchButton::State::Off) {
            break;
        }
    }

    // 验证按钮状态变为Off
    if (switchBtn->getState() == SwitchButton::State::Off) {
        qDebug() << "断开成功！按钮状态已变为OFF";
        disconnection_tested_ = true;
    }
    else {
        qDebug() << "断开失败！按钮状态仍为ON";
        QFAIL("断开节点失败，按钮状态未改变");
        return;
    }

    // 验证信号已发射
    if (spy.count() > 0) {
        QList<QVariant> arguments = spy.takeFirst();
        uint32_t ip = arguments.at(0).toUInt();
        qDebug() << "捕获到LidarConnectionChanged信号: IP=" << node_manager_->Ip2StringT(ip) << ", enabled=" << arguments.at(1).toBool();
    }

    // 测试完成
    qDebug() << "===== 测试全部完成 =====";
    qDebug() << "节点找到: 是";
    qDebug() << "连接测试: " << (connection_tested_ ? "通过" : "失败");
    qDebug() << "断开测试: " << (disconnection_tested_ ? "通过" : "失败");
}

}// namespace ys_test

QTEST_MAIN(ys_test::NodeManagerTest)
#include "node_manager_test.moc"