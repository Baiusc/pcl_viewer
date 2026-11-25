#ifndef YS_TEST_POINTCLOUD_FRAME_POLICY_TEST_H_
#define YS_TEST_POINTCLOUD_FRAME_POLICY_TEST_H_

#include <QtTest>

namespace ys_test {

/**
 * @brief CS96-gamma(1578B / 120 点每 SLOT)点云包长识别与成帧判定测试(纯函数，无需硬件)
 */
class PointcloudFramePolicyTest : public QObject {
    Q_OBJECT

private slots:
    void testPacketSizeDispatch();
    void testGammaPacketIs120Points();
    void testGammaFrameCompletesAt800Slots();
    void testAlphaBetaUnchanged();
    void testPartialSlotLoss();
    void testSlotCountAdaptive();
    void testRoiDefaultCoversGammaPixels();
};

} // namespace ys_test

#endif // YS_TEST_POINTCLOUD_FRAME_POLICY_TEST_H_
