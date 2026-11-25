#include "pointcloud_frame_policy_test.h"

#include <cstddef>

#include "business/pointcloud_frame_policy.h"
#include "component/ys_scene_def.h"
#include "ys_lidar_const.h"
#include "ys_lidar_type.h"

namespace ys_test {

using ys_scene::frame_policy::ExpectedFramePoints;
using ys_scene::frame_policy::IsFrameComplete;
using ys_scene::frame_policy::MinAvailableDataSize;

// CS96-gamma 单回波：18(协议头) + 120(像素/SLOT) × 1(回波) × 13(每点) = 1578
static constexpr size_t kGammaPacketSize = 1578;
static constexpr size_t kGammaPointsPerSlot = 120;
static constexpr size_t kGammaSlotsPerFrame = 800;
static constexpr size_t kUnrecognized = 0;

void PointcloudFramePolicyTest::testPacketSizeDispatch()
{
    // 既有 alpha/beta 判定保持不变
    QCOMPARE(you_see::ResolveCartesianPacketSize(1266), you_see::cartesian_96_lines_packet_size);
    QCOMPARE(you_see::ResolveCartesianPacketSize(1266 * 2), you_see::cartesian_96_lines_packet_size);
    QCOMPARE(you_see::ResolveCartesianPacketSize(2514), you_see::cartesian_192_lines_packet_size);

    // CS96-gamma 新识别：单包与多包合并的数据报都归到 1578
    QCOMPARE(you_see::ResolveCartesianPacketSize(kGammaPacketSize), you_see::cartesian_120_lines_packet_size);
    QCOMPARE(you_see::ResolveCartesianPacketSize(kGammaPacketSize * 2), you_see::cartesian_120_lines_packet_size);
    QCOMPARE(you_see::ResolveCartesianPacketSize(kGammaPacketSize * 4), you_see::cartesian_120_lines_packet_size);

    // gamma 的整数倍不会被其它机型的包长常量截获
    for (size_t k = 1; k <= 8; ++k) {
        QCOMPARE(you_see::ResolveCartesianPacketSize(kGammaPacketSize * k), you_see::cartesian_120_lines_packet_size);
    }

    // 与其它机型路径不冲突：MF150(1935)、FL6532(1462 / 166) 均不命中
    QCOMPARE(you_see::ResolveCartesianPacketSize(1935), kUnrecognized);
    QCOMPARE(you_see::ResolveCartesianPacketSize(1462), kUnrecognized);
    QCOMPARE(you_see::ResolveCartesianPacketSize(166), kUnrecognized);

    // 空数据报不误判
    QCOMPARE(you_see::ResolveCartesianPacketSize(0), kUnrecognized);

    // gamma 双回波(18 + 120×2×13 = 3138)本次不支持，显式记录该边界
    QCOMPARE(you_see::ResolveCartesianPacketSize(3138), kUnrecognized);
}

void PointcloudFramePolicyTest::testGammaPacketIs120Points()
{
    QCOMPARE(you_see::CartesianPointCountOfPacket(kGammaPacketSize), kGammaPointsPerSlot);

    // 既有包长的点数不变
    QCOMPARE(you_see::CartesianPointCountOfPacket(you_see::cartesian_96_lines_packet_size),
        static_cast<size_t>(96));
    QCOMPARE(you_see::CartesianPointCountOfPacket(you_see::cartesian_192_lines_packet_size),
        static_cast<size_t>(192));

    // 防漂移：常量必须与 PC 端结构体布局一致(18B 协议头 / 13B 单点)
    QCOMPARE(you_see::cartesian_packet_header_size, sizeof(you_see::PointCloudNetPacket) - 1);
    QCOMPARE(you_see::cartesian_point_size, sizeof(you_see::CartesianPoint));

    // 包长公式自洽：18 + 像素数 × 回波数 × 13
    QCOMPARE(kGammaPacketSize,
        you_see::cartesian_packet_header_size + kGammaPointsPerSlot * you_see::cartesian_point_size);
}

void PointcloudFramePolicyTest::testGammaFrameCompletesAt800Slots()
{
    const int points_per_slot = static_cast<int>(kGammaPointsPerSlot);
    const int expected_slots = static_cast<int>(kGammaSlotsPerFrame);
    const size_t expected_points = ExpectedFramePoints(points_per_slot, expected_slots);
    QCOMPARE(expected_points, static_cast<size_t>(96000));

    // 逐 SLOT 模拟累积：必须到第 800 个 SLOT 才成帧
    size_t points = 0;
    size_t first_complete_slot = 0;
    for (size_t slot = 1; slot <= kGammaSlotsPerFrame; ++slot) {
        points += kGammaPointsPerSlot;
        if (first_complete_slot == 0 && IsFrameComplete(points, slot, expected_points, expected_slots)) {
            first_complete_slot = slot;
        }
    }
    QCOMPARE(first_complete_slot, kGammaSlotsPerFrame);

    // 回归保护：第 640 个 SLOT 时点数(640×120=76800)恰好等于旧的 800×96 阈值，
    // 修复前会在此时被误判成帧并截断，修复后必须不再成帧
    QVERIFY(!IsFrameComplete(640 * kGammaPointsPerSlot, 640, expected_points, expected_slots));
    QVERIFY(!IsFrameComplete(799 * kGammaPointsPerSlot, 799, expected_points, expected_slots));
    // 即使重复包将点数堆到整帧阈值，缺少槽位也不能判为完整帧。
    QVERIFY(!IsFrameComplete(expected_points, 799, expected_points, expected_slots));
}

void PointcloudFramePolicyTest::testAlphaBetaUnchanged()
{
    // 注意：不可用 slots 作变量名，它是 Qt 的保留宏(qobjectdefs.h: #define slots Q_SLOTS)
    const int expected_slots = 800;
    const int points_per_slot = 96;

    // 与旧常量 cartesianpoint_frame_size(800×96) 逐值一致
    QCOMPARE(ExpectedFramePoints(points_per_slot, expected_slots), you_see::cartesianpoint_frame_size);
    QCOMPARE(MinAvailableDataSize(points_per_slot, expected_slots), static_cast<int>(800 * 96 * 0.92));

    // 首帧仍为第 800 个 SLOT，行为与改动前一致
    size_t points = 0;
    size_t first_complete_slot = 0;
    for (size_t slot = 1; slot <= 800; ++slot) {
        points += static_cast<size_t>(points_per_slot);
        if (first_complete_slot == 0 &&
            IsFrameComplete(points, slot, ExpectedFramePoints(points_per_slot, expected_slots), expected_slots)) {
            first_complete_slot = slot;
        }
    }
    QCOMPARE(first_complete_slot, static_cast<size_t>(800));
}

void PointcloudFramePolicyTest::testPartialSlotLoss()
{
    const size_t expected_points = ExpectedFramePoints(static_cast<int>(kGammaPointsPerSlot),
        static_cast<int>(kGammaSlotsPerFrame));

    // 丢 1 个 SLOT(收到 799 个)：必须等待末包或超时，不能把迟到的末包另起为残帧。
    QVERIFY(!IsFrameComplete(799 * kGammaPointsPerSlot, 799, expected_points, 800));

    // 丢 2 个 SLOT：点数判据不成立，但点数 95760 ≥ 阈值 88320，仍可由超时路径接纳
    QVERIFY(!IsFrameComplete(798 * kGammaPointsPerSlot, 798, expected_points, 800));
    QCOMPARE(MinAvailableDataSize(static_cast<int>(kGammaPointsPerSlot), static_cast<int>(kGammaSlotsPerFrame)),
        static_cast<int>(800 * 120 * 0.92));
    QVERIFY(798 * kGammaPointsPerSlot >=
        static_cast<size_t>(MinAvailableDataSize(static_cast<int>(kGammaPointsPerSlot),
            static_cast<int>(kGammaSlotsPerFrame))));
}

void PointcloudFramePolicyTest::testSlotCountAdaptive()
{
    // 20Hz：每面 400 个 SLOT
    QCOMPARE(ExpectedFramePoints(120, 400), static_cast<size_t>(48000));
    QCOMPARE(MinAvailableDataSize(120, 400), static_cast<int>(400 * 120 * 0.92));

    // alpha/beta 各档位与既有 N×96×0.92 逐值一致
    QCOMPARE(MinAvailableDataSize(96, 200), static_cast<int>(200 * 96 * 0.92));
    QCOMPARE(MinAvailableDataSize(96, 400), static_cast<int>(400 * 96 * 0.92));
    QCOMPARE(MinAvailableDataSize(96, 600), static_cast<int>(600 * 96 * 0.92));
    QCOMPARE(MinAvailableDataSize(96, 800), static_cast<int>(800 * 96 * 0.92));
    QCOMPARE(MinAvailableDataSize(96, 1000), static_cast<int>(1000 * 96 * 0.92));
}

void PointcloudFramePolicyTest::testRoiDefaultCoversGammaPixels()
{
    const int max_pixel = static_cast<int>(ys_scene::kPointRoiDefaultMaxPixel);

    // 复现 PointcloudGen 的过滤条件(j > max_pixel 即丢弃)：gamma 的 120 个垂直像素必须全部保留
    for (int j = 0; j < static_cast<int>(ys_scene::kMaxCartesianPixelsPerSlot); ++j) {
        QVERIFY2(j <= max_pixel, qPrintable(QString("像素 %1 会被默认 ROI 丢弃").arg(j)));
    }

    // 回归保护：默认值必须已从旧的 95/96 放宽
    QVERIFY(max_pixel >= 119);
}

} // namespace ys_test

QTEST_MAIN(ys_test::PointcloudFramePolicyTest)
#include "pointcloud_frame_policy_test.moc"
