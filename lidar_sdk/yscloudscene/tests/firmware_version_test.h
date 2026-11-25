#ifndef YS_TEST_FIRMWARE_VERSION_TEST_H_
#define YS_TEST_FIRMWARE_VERSION_TEST_H_

#include <QtTest>

namespace ys_test {

/**
 * @brief 0x0302 系统固件版本解析测试(合成报文，无需硬件)
 */
class FirmwareVersionTest : public QObject {
    Q_OBJECT

private slots:
    void testPayloadLayout();
    void testParseSuccess();
    void testNulTruncation();
    void testTraceStates();
    void testBuildTime64Bit();
    void testAbnormalInput();
};

} // namespace ys_test

#endif // YS_TEST_FIRMWARE_VERSION_TEST_H_
