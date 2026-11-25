#include "component/node_manager/node_manager.h"
#include "component/switch_button/switch_button.h"

#include <QResource>
#include <QSignalSpy>
#include <QtEndian>
#include <QtTest>

namespace {
using ys_scene::NodeManager;
using ys_scene::SwitchButton;

you_see::LidarInfo Node()
{
    you_see::LidarInfo node{};
    node.ip = qToBigEndian(quint32{0x0A2A00DA});
    node.unique_id = node.ip;
    node.model = you_see::LidarModel::KCS;
    node.node_work_mode = you_see::NodeWorkMode::KGray;
    node.instruction_processor_stoped = true;
    node.data_processor_stoped = true;
    return node;
}
} // namespace

class NodeManagerStateTest : public QObject
{
    Q_OBJECT
  private slots:
    void initTestCase();
    void IntermediateStateKeepsPending();
    void StaleDisconnectDoesNotStopNewConnection();
    void ConnectedSnapshotCanCreateNode();
};

void NodeManagerStateTest::initTestCase()
{
    Q_INIT_RESOURCE(ys);
}

void NodeManagerStateTest::IntermediateStateKeepsPending()
{
    NodeManager manager;
    auto node = Node();
    manager.AddNodeList(node, nullptr);
    QTRY_VERIFY(manager.findChild<SwitchButton *>());
    auto *button = manager.findChild<SwitchButton *>();
    QCOMPARE(button->getState(), SwitchButton::State::Off);
    QTest::mouseClick(button, Qt::LeftButton);
    QCOMPARE(button->getState(), SwitchButton::State::Pending);
    QSignalSpy requests(&manager, &NodeManager::LidarConnectionChanged);

    // 部署日志中的状态：未连接、指令运行、数据接收器已停止。
    node.instruction_processor_stoped = false;
    manager.AddNodeList(node, nullptr);
    QCoreApplication::sendPostedEvents(&manager, QEvent::MetaCall);
    QCOMPARE(button->getState(), SwitchButton::State::Pending);
    node.data_processor_stoped = false;
    manager.AddNodeList(node, nullptr);
    QCoreApplication::sendPostedEvents(&manager, QEvent::MetaCall);
    QCOMPARE(button->getState(), SwitchButton::State::Pending);

    QSignalSpy connected(&manager, &NodeManager::LidarConnected);
    node.is_connected = true;
    manager.AddNodeList(node, nullptr);
    QTRY_COMPARE(button->getState(), SwitchButton::State::On);
    QCOMPARE(connected.count(), 1);
    manager.AddNodeList(node, nullptr);
    QCoreApplication::sendPostedEvents(&manager, QEvent::MetaCall);
    QCOMPARE(connected.count(), 1);

    node.is_connected = false;
    node.instruction_processor_stoped = true;
    node.data_processor_stoped = true;
    manager.AddNodeList(node, nullptr);
    QTRY_COMPARE(button->getState(), SwitchButton::State::Off);
    QCOMPARE(requests.count(), 1);
    QCOMPARE(requests.at(0).at(1).toBool(), false);
}

void NodeManagerStateTest::StaleDisconnectDoesNotStopNewConnection()
{
    NodeManager manager;
    auto node = Node();
    node.is_connected = true;
    node.instruction_processor_stoped = false;
    node.data_processor_stoped = false;
    manager.AddNodeList(node, nullptr);
    QTRY_VERIFY(manager.findChild<SwitchButton *>());
    auto *button = manager.findChild<SwitchButton *>();
    QTRY_COMPARE(button->getState(), SwitchButton::State::On);
    QSignalSpy requests(&manager, &NodeManager::LidarConnectionChanged);

    // GUI 线程尚未处理旧会话的清理消息时，新会话已重新就绪。
    auto stopped = node;
    stopped.is_connected = false;
    stopped.instruction_processor_stoped = true;
    stopped.data_processor_stoped = true;
    manager.AddNodeList(stopped, nullptr);
    auto preparing = node;
    preparing.is_connected = false;
    manager.AddNodeList(preparing, nullptr);
    QCoreApplication::sendPostedEvents(&manager, QEvent::MetaCall);
    QCOMPARE(button->getState(), SwitchButton::State::Pending);
    QCOMPARE(requests.count(), 0);
    manager.AddNodeList(node, nullptr);
    QTRY_COMPARE(button->getState(), SwitchButton::State::On);
    QCOMPARE(requests.count(), 0);
}

void NodeManagerStateTest::ConnectedSnapshotCanCreateNode()
{
    NodeManager manager;
    auto node = Node();
    // 广播快照被更新的握手成功快照替代时，也应创建节点并显示绿色。
    manager.AddNodeList(node, nullptr);
    node.is_connected = true;
    node.instruction_processor_stoped = false;
    node.data_processor_stoped = false;
    manager.AddNodeList(node, nullptr);
    QTRY_VERIFY(manager.findChild<SwitchButton *>());
    QCOMPARE(manager.findChild<SwitchButton *>()->getState(), SwitchButton::State::On);
}

QTEST_MAIN(NodeManagerStateTest)
#include "node_manager_state_test.moc"
