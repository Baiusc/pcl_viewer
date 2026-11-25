#include "business/device_management/network_protocol.h"

#include <QtTest>

using namespace ys_scene::lidar_network;

class LidarNetworkProtocolTest : public QObject
{
    Q_OBJECT

private slots:
    void Normalize();
    void Transaction();
};

void LidarNetworkProtocolTest::Normalize()
{
    QVERIFY(NormalizeConfig({{"mode", "auto"}}).error.isEmpty());
    QVERIFY(!NormalizeConfig({{"mode", "auto"}, {"address", "10.42.1.0"}}).error.isEmpty());
    QJsonObject fixed{{"mode", "static"}, {"address", "010.042.1.0"}, {"netmask", "255.255.254.0"}};
    const auto result = NormalizeConfig(fixed);
    QVERIFY(result.error.isEmpty());
    QCOMPARE(result.value.value("address").toString(), QString("10.42.1.0"));
    fixed.insert("gateway", "10.42.1.0");
    QVERIFY(!NormalizeConfig(fixed).error.isEmpty());
    QJsonObject pool{{"mode", "pool"}, {"start", "10.42.0.255"}, {"end", "10.42.1.0"},
                     {"netmask", "255.255.254.0"}};
    QVERIFY(NormalizeConfig(pool).error.isEmpty());
    pool.insert("end", "10.42.1.255");
    QVERIFY(!NormalizeConfig(pool).error.isEmpty());
}

void LidarNetworkProtocolTest::Transaction()
{
    const auto config = NormalizeConfig({{"mode", "static"}, {"address", "10.42.0.30"},
                                         {"netmask", "255.255.255.0"}}).value;
    const Endpoint old_endpoint{"nic", "10.42.0.20", 80};
    const Endpoint new_endpoint{"nic", "10.42.0.30", 80};
    Task task{old_endpoint, "28:b5:e8:e2:60:19", "test-device", QString(32, 'a'), {}, config, true, {}, false};
    QJsonObject snapshot{{"schema_version", 1}, {"mac", task.mac}, {"epoch", task.epoch},
                         {"identity", task.identity}, {"simulation", true}, {"ready", true},
                         {"recovery", false}, {"configuration_valid", true}, {"pending", true},
                         {"phase", "READY"}, {"transaction_phase", "WAIT_CONFIRM"}, {"source", "static"},
                         {"address", new_endpoint.address}, {"netmask", "255.255.255.0"}, {"gateway", ""},
                         {"operation_code", 202}, {"operation_id", QString(32, 'b')},
                         {"confirm_remaining_ms", 6000}, {"confirmed_config", QJsonObject{{"mode", "auto"}}},
                         {"desired_config", config}};
    QVERIFY(task.Evaluate(new_endpoint, snapshot) == Decision::KObserve);
    QVERIFY(!task.Accept(0, snapshot));
    QVERIFY(task.Accept(202, snapshot));
    QVERIFY(task.Evaluate(old_endpoint, snapshot) == Decision::KStop);
    QVERIFY(task.Evaluate(new_endpoint, snapshot) == Decision::KConfirm);
    task.confirmation_sent = true;
    QVERIFY(task.Evaluate(new_endpoint, snapshot) == Decision::KObserve);
    snapshot.insert("pending", false);
    snapshot.insert("transaction_phase", "COMMITTED");
    snapshot.insert("operation_code", 200);
    snapshot.insert("confirmed_config", config);
    QVERIFY(task.Evaluate(new_endpoint, snapshot) == Decision::KCommitted);
    snapshot.insert("epoch", QString(32, 'c'));
    QVERIFY(task.Evaluate(new_endpoint, snapshot) == Decision::KStop);
}

QTEST_APPLESS_MAIN(LidarNetworkProtocolTest)
#include "lidar_network_protocol_test.moc"
