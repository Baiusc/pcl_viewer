#include "business/device_management/management_protocol.h"
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QtTest>
using namespace ys_scene;

class ManagementTest : public QObject
{
    Q_OBJECT
  private slots:

    void GrayUsesMode3()
    {
        QFile file(QStringLiteral(MANAGEMENT_EXAMPLES));
        QVERIFY(file.open(QIODevice::ReadOnly));
        auto body = QJsonDocument::fromJson(file.readAll()).object()["scenarios"].toArray().first().toObject()["body"]
                        .toObject();
        body["validated_work_modes"] = QJsonArray{0, 1, 2, 3, 4};
        body["supported_modes"] = QJsonArray{"histogram", "pointcloud", "gray"};
        ManagementSnapshot snapshot;
        QVERIFY(ParseManagementSnapshot(body, snapshot));
        QCOMPARE(snapshot.modes, QList<int>({0, 1, 2, 3}));
        QCOMPARE(ManagementModeName(3), QStringLiteral("灰度图"));
        QVERIFY(ManagementModeName(4).contains(QStringLiteral("未知模式")));

        body["supported_modes"] = QJsonArray{"histogram"};
        QVERIFY(ParseManagementSnapshot(body, snapshot));
        QCOMPARE(snapshot.modes, QList<int>({0}));
        body["supported_modes"] = QJsonArray{"gray"};
        QVERIFY(ParseManagementSnapshot(body, snapshot));
        QCOMPARE(snapshot.modes, QList<int>({3}));
    }

    void Contract()
    {
        QFile file(QStringLiteral(MANAGEMENT_EXAMPLES));
        QVERIFY(file.open(QIODevice::ReadOnly));
        const auto cases = QJsonDocument::fromJson(file.readAll()).object()["scenarios"].toArray();
        QVERIFY(cases.size() >= 18);
        for (const auto &entry : cases) {
            const auto c = entry.toObject();
            const auto body = c["body"].toObject();
            ManagementSnapshot s;
            if (body["source"] == "bridge") {
                QVERIFY(!ParseManagementSnapshot(body, s));
                continue;
            }
            QVERIFY2(ParseManagementSnapshot(body, s), qPrintable(c["name"].toString()));
            QVERIFY(!s.modes.contains(0));
            QVERIFY(!s.modes.contains(3));
            QVERIFY(!s.modes.contains(4));
            if (c["name"] == "rolled_back")
                QVERIFY(ManagementOutcome(s, 1).contains(QStringLiteral("未成功")));
            if (c["name"] == "switch_succeeded")
                QCOMPARE(ManagementOutcome(s, 1), QStringLiteral("切换成功"));
            auto invalid = body;
            invalid["actual_mode"] = true;
            QVERIFY(!ParseManagementSnapshot(invalid, s));
            invalid = body;
            invalid.remove("phase");
            QVERIFY(!ParseManagementSnapshot(invalid, s));
        }
    }
};
QTEST_GUILESS_MAIN(ManagementTest)
#include "management_test.moc"
