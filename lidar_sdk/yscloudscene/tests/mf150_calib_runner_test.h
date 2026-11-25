#ifndef YS_TEST_MF150_CALIB_RUNNER_TEST_H_
#define YS_TEST_MF150_CALIB_RUNNER_TEST_H_

#include <QObject>

namespace ys_test {

// MF150 标定 runner 迁移到算法统一库(新版 API)后的冒烟测试：
// 单点标定(base offset 数值)、多点 LUT 表构建与文件往返。
class Mf150CalibRunnerTest : public QObject {
    Q_OBJECT
private slots:
    // 单点标定：给定已知距离与单像素 tof，base offset = 真实距离 - 换算距离
    void testSinglePointCalibration();
    // 单点表文件(旧 BIN 格式)保存/加载往返
    void testSingleTableFileRoundTrip();
    // 多点标定：两个距离组生成每像素 LUT(num_pts=2)，插值可用
    void testMultiPointLut();
    // 多点 LUT 文件(结构直写)往返
    void testMultiLutFileRoundTrip();
};

} // namespace ys_test

#endif // YS_TEST_MF150_CALIB_RUNNER_TEST_H_
