#ifndef YS_TEST_LIDAR_MODEL_CONFIG_TEST_H_
#define YS_TEST_LIDAR_MODEL_CONFIG_TEST_H_

#include <QtTest>

namespace ys_test {
class LidarModelConfigTest : public QObject {
    Q_OBJECT

private slots:
    void testCs96BetaKey();
    void testNonBetaReturnsEmpty();
    void testConfigDirPath();
};
}

#endif
