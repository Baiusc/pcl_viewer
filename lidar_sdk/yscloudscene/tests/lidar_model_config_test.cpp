#include "lidar_model_config_test.h"

#include "business/lidar_properties_config/lidar_model_config.h"

namespace ys_test {

static you_see::LidarInfo MakeNode(you_see::LidarModel model, char hw_first = '\0')
{
    you_see::LidarInfo node{};
    node.model = model;
    node.hardware_version[0] = static_cast<uint8_t>(hw_first);
    return node;
}

void LidarModelConfigTest::testCs96BetaKey()
{
    auto node = MakeNode(you_see::LidarModel::KCS, 'B');
    QCOMPARE(ys_scene::LidarModelConfigKey(node), QStringLiteral("cs96_beta"));

    node = MakeNode(you_see::LidarModel::KCS, 'b');
    QCOMPARE(ys_scene::LidarModelConfigKey(node), QStringLiteral("cs96_beta"));
}

void LidarModelConfigTest::testNonBetaReturnsEmpty()
{
    // CS 大类但硬件版本非 B：无属性配置
    QCOMPARE(ys_scene::LidarModelConfigKey(MakeNode(you_see::LidarModel::KCS, 'A')), QString());
    QCOMPARE(ys_scene::LidarModelConfigKey(MakeNode(you_see::LidarModel::KCS, '\0')), QString());

    // 其它大类即使硬件版本首字符为 B 也不误判
    QCOMPARE(ys_scene::LidarModelConfigKey(MakeNode(you_see::LidarModel::KMF, 'B')), QString());
    QCOMPARE(ys_scene::LidarModelConfigKey(MakeNode(you_see::LidarModel::KUnknown, 'B')), QString());
}

void LidarModelConfigTest::testConfigDirPath()
{
    const QString dir = ys_scene::LidarModelConfigDir(QStringLiteral("cs96_beta"));
    QVERIFY(dir.endsWith(QStringLiteral("/lidar_properties_config/cs96_beta")));
}

} // namespace ys_test

QTEST_GUILESS_MAIN(ys_test::LidarModelConfigTest)
#include "lidar_model_config_test.moc"
