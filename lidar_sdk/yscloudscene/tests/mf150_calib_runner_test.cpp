#include "mf150_calib_runner_test.h"

#include <QTest>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>

#include "business/mf150_calibration_runner.h"

namespace ys_test {

namespace {
constexpr double kLightSpeed = 299792458e-9;
constexpr size_t kRows = 152;
constexpr size_t kCols = 360;
constexpr size_t kTestRow = 75;
constexpr size_t kTestCol = 180;

// 距离(米) -> tof 计数(与换算公式 r = tof/1000*c/2 互逆)
uint32_t MetresToTof(float metres)
{
    return static_cast<uint32_t>(std::round(metres * 2.0 / kLightSpeed * 1000.0));
}

// 构造全空网格，只在一个像素放一个有效值
ys_scene::Mf150CalibGrid MakeGridWithPixel(float metres)
{
    ys_scene::Mf150CalibGrid grid(kRows, std::vector<ys_scene::Mf150CalibPixel>(kCols));
    auto& cell = grid[kTestRow][kTestCol];
    cell.valid = true;
    cell.tof = MetresToTof(metres);
    cell.peak = 500;
    return grid;
}
} // namespace

void Mf150CalibRunnerTest::testSinglePointCalibration()
{
    // 真实 3.0m，测量 2.9m(一个像素) → base offset ≈ 3.0 - 2.9 = 0.1
    const float dist_real = 3.0f;
    const float measured = 2.9f;
    auto grid = MakeGridWithPixel(measured);

    std::vector<std::vector<float>> out_map;
    const bool ok = ys_scene::RunMf150Calibration(grid, 0, dist_real, "", nullptr, out_map);
    QVERIFY(ok);
    QCOMPARE(out_map.size(), kRows);
    QVERIFY(!out_map.empty() && out_map[0].size() == kCols);
    QVERIFY(std::fabs(out_map[kTestRow][kTestCol] - (dist_real - measured)) < 1e-3f);
    // 其余像素 offset 应为 0
    QCOMPARE(out_map[0][0], 0.0f);
}

void Mf150CalibRunnerTest::testSingleTableFileRoundTrip()
{
    auto grid = MakeGridWithPixel(2.0f);
    std::vector<std::vector<float>> out_map;
    QVERIFY(ys_scene::RunMf150Calibration(grid, 0, 2.5f, "", nullptr, out_map));

    const std::string path = "mf150_calib_runner_test_map.bin";
    QVERIFY(ys_scene::SaveCalibMapToFile(out_map, path));

    std::vector<std::vector<float>> loaded;
    QVERIFY(ys_scene::LoadCalibMapFromFile(path, loaded));
    QCOMPARE(loaded.size(), out_map.size());
    QCOMPARE(loaded[0].size(), out_map[0].size());
    QVERIFY(loaded[kTestRow][kTestCol] == out_map[kTestRow][kTestCol]);

    std::remove(path.c_str());
}

void Mf150CalibRunnerTest::testMultiPointLut()
{
    // 两个距离组：1.0m 与 3.0m，各只有测试像素有效
    const float d1 = 1.0f;
    const float d2 = 3.0f;
    std::vector<std::pair<float, ys_scene::Mf150CalibGrid>> samples;
    samples.push_back({ d1, MakeGridWithPixel(0.95f) });
    samples.push_back({ d2, MakeGridWithPixel(2.95f) });

    auto table = std::make_unique<SslDistCalibMulti>();
    const bool ok = ys_scene::RunMf150MultiCalibration(samples, 0, "", nullptr, *table);
    QVERIFY(ok);

    const SslPixelDistCurve& curve = table->curves[kTestRow][kTestCol];
    QCOMPARE(curve.num_pts, static_cast<uint8_t>(2));
    QVERIFY(std::fabs(curve.raw_dists[0] - 0.95f) < 1e-3f);
    QVERIFY(std::fabs(curve.offsets[0] - (d1 - 0.95f)) < 1e-3f);
    QVERIFY(std::fabs(curve.offsets[1] - (d2 - 2.95f)) < 1e-3f);
    // 未标定像素应无曲线
    QCOMPARE(table->curves[0][0].num_pts, static_cast<uint8_t>(0));
}

void Mf150CalibRunnerTest::testMultiLutFileRoundTrip()
{
    std::vector<std::pair<float, ys_scene::Mf150CalibGrid>> samples;
    samples.push_back({ 1.0f, MakeGridWithPixel(0.95f) });
    samples.push_back({ 3.0f, MakeGridWithPixel(2.95f) });

    auto table = std::make_unique<SslDistCalibMulti>();
    QVERIFY(ys_scene::RunMf150MultiCalibration(samples, 0, "", nullptr, *table));

    const std::string path = "mf150_calib_runner_test_lut.bin";
    QVERIFY(ys_scene::SaveMultiCalibTableToFile(*table, path));

    auto loaded = std::make_unique<SslDistCalibMulti>();
    QVERIFY(ys_scene::LoadMultiCalibTableFromFile(path, *loaded));
    QCOMPARE(loaded->curves[kTestRow][kTestCol].num_pts, table->curves[kTestRow][kTestCol].num_pts);
    QVERIFY(std::fabs(loaded->curves[kTestRow][kTestCol].offsets[1] -
        table->curves[kTestRow][kTestCol].offsets[1]) < 1e-6f);

    std::remove(path.c_str());
}

} // namespace ys_test

QTEST_GUILESS_MAIN(ys_test::Mf150CalibRunnerTest)
