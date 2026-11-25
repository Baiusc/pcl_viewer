#include "firmware_version_test.h"

#include "business/firmware_version/firmware_version_codec.h"

#include <cstddef>
#include <cstring>

namespace ys_test {

namespace {

constexpr int kComponentCount = 5;

// 按协议文档 §4.2 构造一条组件记录
you_see::FirmwareComponentInfoT MakeComponent(uint8_t id, uint8_t valid, uint8_t dirty,
                                              const char* sha, uint32_t build_time,
                                              uint32_t build_time_hi = 0)
{
    you_see::FirmwareComponentInfoT component{};
    component.id = id;
    component.valid = valid;
    component.major = 1;
    component.minor = 2;
    component.patch = 3;
    component.dirty = dirty;
    component.build_time = build_time;
    component.build_time_hi = build_time_hi;

    if (sha != nullptr) {
        const size_t sha_len = std::strlen(sha);
        component.sha_len = static_cast<uint8_t>(sha_len);
        std::memcpy(component.git_sha, sha, qMin<size_t>(sha_len, sizeof(component.git_sha)));
    }
    return component;
}

// 典型的成功应答：组件 0(you_see 应用)已接入且干净，其余为未接入占位
you_see::FirmwareVersionInfoT MakeSuccessAck()
{
    you_see::FirmwareVersionInfoT ack{};
    ack.ret_code = 0x00;
    ack.component_count = kComponentCount;
    ack.fw_major = 1;
    ack.fw_minor = 2;
    ack.fw_patch = 3;

    const char kProduct[] = "MF150-fl6532";
    const char kReleaseId[] = "20260921-000";
    // 只写入有效字符，其余字节保持 0（模拟板端的 NUL 结尾 + 补齐）
    std::memcpy(ack.product, kProduct, sizeof(kProduct) - 1);
    std::memcpy(ack.release_id, kReleaseId, sizeof(kReleaseId) - 1);

    ack.components[0] = MakeComponent(0, 1, 0, "38da7cf", 1758400000u);
    ack.components[1] = MakeComponent(1, 0, 0, nullptr, 0);
    ack.components[2] = MakeComponent(2, 0, 0, nullptr, 0);
    ack.components[3] = MakeComponent(3, 0, 0, nullptr, 0);
    ack.components[4] = MakeComponent(4, 0, 0, nullptr, 0);
    return ack;
}

} // namespace

void FirmwareVersionTest::testPayloadLayout()
{
    // 与协议文档 §4 的字节布局绑定：若 SDK 结构体被改动，这里立即失败
    QCOMPARE(sizeof(you_see::FirmwareComponentInfoT), size_t(32));
    QCOMPARE(sizeof(you_see::FirmwareVersionInfoT), size_t(200));
    QCOMPARE(offsetof(you_see::FirmwareVersionInfoT, product), size_t(6));
    QCOMPARE(offsetof(you_see::FirmwareVersionInfoT, release_id), size_t(22));
    QCOMPARE(offsetof(you_see::FirmwareVersionInfoT, components), size_t(40));
    QCOMPARE(offsetof(you_see::FirmwareComponentInfoT, git_sha), size_t(7));
    QCOMPARE(offsetof(you_see::FirmwareComponentInfoT, build_time), size_t(19));
    QCOMPARE(offsetof(you_see::FirmwareComponentInfoT, build_time_hi), size_t(23));
}

void FirmwareVersionTest::testParseSuccess()
{
    const lumosy::FirmwareVersionInfo info = ys_scene::ToFirmwareVersionInfo(MakeSuccessAck());

    QVERIFY(info.ok);
    QCOMPARE(info.ret_code, quint8(0x00));
    QCOMPARE(info.component_count, quint8(5));
    QCOMPARE(info.fw_major, quint8(1));
    QCOMPARE(info.fw_minor, quint8(2));
    QCOMPARE(info.fw_patch, quint8(3));
    QCOMPARE(info.product, QStringLiteral("MF150-fl6532"));
    QCOMPARE(info.release_id, QStringLiteral("20260921-000"));
    QCOMPARE(info.components.size(), 5);

    // 组件 0：已接入且干净
    const lumosy::FirmwareComponentInfo& app = info.components.at(0);
    QCOMPARE(app.id, quint8(0));
    QVERIFY(app.valid);
    QCOMPARE(app.git_sha, QStringLiteral("38da7cf"));
    QCOMPARE(app.sha_len, quint8(7));
    QCOMPARE(app.build_time, quint64(1758400000));
    QCOMPARE(int(lumosy::TraceStateOf(app)), int(lumosy::FirmwareTraceState::kClean));

    // 组件 1：未接入的占位记录
    const lumosy::FirmwareComponentInfo& arm = info.components.at(1);
    QVERIFY(!arm.valid);
    QVERIFY(arm.git_sha.isEmpty());
    QCOMPARE(int(lumosy::TraceStateOf(arm)), int(lumosy::FirmwareTraceState::kUnknown));
}

void FirmwareVersionTest::testNulTruncation()
{
    // 字段被占满（无 NUL）时取满长度，且不得越界读到相邻字段
    you_see::FirmwareVersionInfoT ack{};
    ack.ret_code = 0x00;
    ack.component_count = 0;
    std::memset(ack.product, 'A', sizeof(ack.product));
    std::memset(ack.release_id, 'B', sizeof(ack.release_id));

    const lumosy::FirmwareVersionInfo info = ys_scene::ToFirmwareVersionInfo(ack);
    QCOMPARE(info.product, QString(16, QLatin1Char('A')));
    QCOMPARE(info.release_id, QString(16, QLatin1Char('B')));

    // 短串时不得把尾部补齐的 0 带进结果
    you_see::FirmwareVersionInfoT short_ack{};
    const char kName[] = "CS96-beta";
    std::memcpy(short_ack.product, kName, sizeof(kName) - 1);
    const lumosy::FirmwareVersionInfo short_info = ys_scene::ToFirmwareVersionInfo(short_ack);
    QCOMPARE(short_info.product, QStringLiteral("CS96-beta"));
}

void FirmwareVersionTest::testTraceStates()
{
    auto make = [](bool valid, bool dirty, quint8 sha_len) {
        lumosy::FirmwareComponentInfo component;
        component.valid = valid;
        component.dirty = dirty;
        component.sha_len = sha_len;
        return component;
    };

    // valid == 0：未接入，优先于其它判定
    QCOMPARE(int(lumosy::TraceStateOf(make(false, false, 0))), int(lumosy::FirmwareTraceState::kUnknown));
    QCOMPARE(int(lumosy::TraceStateOf(make(false, true, 7))), int(lumosy::FirmwareTraceState::kUnknown));

    // sha_len == 0：来源不可追溯，dirty 不可信
    // 关键：此处必须是 kNoGitInfo，绝不能因 dirty==0 而报成 kClean
    QCOMPARE(int(lumosy::TraceStateOf(make(true, false, 0))), int(lumosy::FirmwareTraceState::kNoGitInfo));
    QCOMPARE(int(lumosy::TraceStateOf(make(true, true, 0))), int(lumosy::FirmwareTraceState::kNoGitInfo));

    // 有 git 信息时按 dirty 区分
    QCOMPARE(int(lumosy::TraceStateOf(make(true, true, 7))), int(lumosy::FirmwareTraceState::kDirty));
    QCOMPARE(int(lumosy::TraceStateOf(make(true, false, 7))), int(lumosy::FirmwareTraceState::kClean));
}

void FirmwareVersionTest::testBuildTime64Bit()
{
    you_see::FirmwareVersionInfoT ack = MakeSuccessAck();
    // 高 32 位为 1、低 32 位为 0x80000000：有符号 32 位会在此回绕
    ack.components[0].build_time = 0x80000000u;
    ack.components[0].build_time_hi = 0x00000001u;

    const lumosy::FirmwareVersionInfo info = ys_scene::ToFirmwareVersionInfo(ack);
    const quint64 expected = (quint64(1) << 32) | quint64(0x80000000u);
    QCOMPARE(info.components.at(0).build_time, expected);
    QVERIFY(!ys_scene::FormatFirmwareBuildTimeUtc(expected).isEmpty());
    QVERIFY(ys_scene::FormatFirmwareBuildTimeUtc(expected).endsWith(QStringLiteral(" UTC")));

    // build_time 为 0 表示未知
    ack.components[0].build_time = 0;
    ack.components[0].build_time_hi = 0;
    const lumosy::FirmwareVersionInfo zero_info = ys_scene::ToFirmwareVersionInfo(ack);
    QCOMPARE(zero_info.components.at(0).build_time, quint64(0));
    QVERIFY(ys_scene::FormatFirmwareBuildTimeUtc(0).isEmpty());
}

void FirmwareVersionTest::testAbnormalInput()
{
    // component_count 来自板端，异常大时按固定 5 条截断，不得越界
    you_see::FirmwareVersionInfoT ack = MakeSuccessAck();
    ack.component_count = 0xFF;
    QCOMPARE(ys_scene::ToFirmwareVersionInfo(ack).components.size(), 5);

    // sha_len 超过 git_sha 容量时按容量截断
    ack = MakeSuccessAck();
    ack.components[0].sha_len = 200;
    std::memset(ack.components[0].git_sha, 'a', sizeof(ack.components[0].git_sha));
    const lumosy::FirmwareVersionInfo clipped = ys_scene::ToFirmwareVersionInfo(ack);
    QCOMPARE(clipped.components.at(0).sha_len, quint8(12));
    QCOMPARE(clipped.components.at(0).git_sha.size(), 12);
    QCOMPARE(int(lumosy::TraceStateOf(clipped.components.at(0))),
             int(lumosy::FirmwareTraceState::kClean));

    // ret_code != 0：ok 为假，但载荷字段仍按协议解析（供 UI 显示上次数据/失败原因）
    ack = MakeSuccessAck();
    ack.ret_code = 0x03;
    const lumosy::FirmwareVersionInfo failed = ys_scene::ToFirmwareVersionInfo(ack);
    QVERIFY(!failed.ok);
    QCOMPARE(failed.ret_code, quint8(0x03));
    QCOMPARE(failed.product, QStringLiteral("MF150-fl6532"));
}

} // namespace ys_test

QTEST_GUILESS_MAIN(ys_test::FirmwareVersionTest)
#include "firmware_version_test.moc"
