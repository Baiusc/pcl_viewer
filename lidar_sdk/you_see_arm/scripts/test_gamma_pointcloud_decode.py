#!/usr/bin/env python3
"""验证 Gamma 真实解码路径的定点 TOF、保留位、双回波和单回波选择。"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include "ys_business/decode/ys_decode_common.c"
#include <assert.h>

calibration_parm_t cal_param;
configuration_t config;
static uint16_t expected_peak;
void log_log(int level, const char *file, int line, const char *fmt, ...) {}
int decode_build_protocal_head(raw_protocal_data_packet_t *raw, point_cloud_head_t *head)
{
    memset(head, 0, sizeof(*head));
    return sizeof(*head);
}
float decode_get_horizontal_corners(point_cloud_head_t *head, org_point_cloud_packet_t *raw)
{
    return 0;
}
uint8_t get_reflect(uint16_t pixel, uint16_t slot, float r, uint16_t pk, float coefficient)
{
    assert(pk == expected_peak);
    return 42;
}
static void put24(uint8_t *dest, uint32_t value)
{
    dest[0] = value;
    dest[1] = value >> 8;
    dest[2] = value >> 16;
}
static float expected_distance(uint32_t code)
{
    return (299792458.0 * 1e-9 * (code / 64.0) / 2.0) * 1.25 + 0.3;
}
static void check_point(point_cloud_packet_t *out, int index, uint32_t code)
{
    spherical_point_data_t point;
    memcpy(&point, out->value + index * sizeof(point), sizeof(point));
    float expected = code <= k_error_ft ? 0.0f : expected_distance(code);
    assert(fabsf(point.r.f - expected) < 0.0003f);
    assert(point.theta.f == 0 && point.omega.f == 0);
}
int main(void)
{
    _Static_assert(sizeof(point_cloud_pixel_data_t) == 16, "每像素 16 字节");
    uint8_t input[9 + 16 + 2 * 16] = {0};
    uint8_t output[sizeof(point_cloud_head_t) + 4 * sizeof(spherical_point_data_t)] = {0};
    raw_protocal_data_packet_t *raw = (void *)input;
    org_point_cloud_packet_t *slot = (void *)raw->value;
    point_cloud_packet_t *out = (void *)output;
    float zero[2] = {0}, factor[2] = {1.25f, 1.25f}, offset[2] = {0.3f, 0.3f};
    g_theta = g_omega_delta = g_ld_intensity_coefficient = zero;
    g_distance_factor = factor;
    g_delta_distance_index = offset;
    cal_param.lines_v = 2;
    config.coordinate_mode = SPHERICAL_COORDINATE_SYSTEM;
    config.echo_mode = DOUBLE_ECHO_MODE;
    // 独立指定 16 字节布局：不同前后沿保留位，峰值保留完整 16 位。
    const uint8_t pixel[16] = {0x01, 0x19, 0xA0, 0x03, 0x1E, 0xB0, 0xDC, 0xFE,
                               0x3F, 0x32, 0xC0, 0x20, 0x35, 0xD0, 0xDC, 0xFE};
    memcpy(slot->value, pixel, sizeof(pixel));
    // 下一像素为空，不能沿用上一个有效回波的距离。
    expected_peak = 0xFEDC;
    int len = deal_orgin_point_cloud_data(raw, out);
    assert(len == sizeof(output));
    check_point(out, 0, 0x1901);
    check_point(out, 1, 0x323F);
    check_point(out, 2, 0);
    check_point(out, 3, 0);
    assert(decode_tof(slot->value + 3) == 0x1E03);
    assert(decode_tof(slot->value + 11) == 0x3520);
    assert(decode_tof((uint8_t[]){0xFF, 0xFF, 0xFF}) == 0xFFFFF);
    config.echo_mode = SINGLE_ECHO_MODE;
    config.echo_type = FIRST_ECHO_TYPE;
    deal_orgin_point_cloud_data(raw, out);
    check_point(out, 0, 0x1901);
    config.echo_type = LAST_ECHO_TYPE;
    deal_orgin_point_cloud_data(raw, out);
    check_point(out, 0, 0x323F);
    // 峰值最大的回波必须连同自己的 TOF 一起选出。
    echo_data_t *echo = (void *)slot->value;
    echo[0].pk = 0xFFFF;
    expected_peak = 0xFFFF;
    config.echo_type = STRONGEST_ECHO_TYPE;
    deal_orgin_point_cloud_data(raw, out);
    check_point(out, 0, 0x1901);
    echo[0].pk = 1;
    expected_peak = 0xFEDC;
    deal_orgin_point_cloud_data(raw, out);
    check_point(out, 0, 0x323F);
    // 覆盖所有保留位取值和最大有效定点值。
    config.echo_type = FIRST_ECHO_TYPE;
    expected_peak = 1;
    for (int reserved = 0; reserved < 16; ++reserved) {
        put24(echo[0].ft, ((uint32_t)reserved << 20) | 0xFFFFF);
        deal_orgin_point_cloud_data(raw, out);
        check_point(out, 0, 0xFFFFF);
        put24(echo[0].ft, ((uint32_t)reserved << 20) | k_error_ft);
        deal_orgin_point_cloud_data(raw, out);
        check_point(out, 0, k_error_ft);
    }
}
'''

with tempfile.TemporaryDirectory(prefix="ys-gamma-decode-") as folder:
    directory = Path(folder)
    harness = directory / "test.c"
    harness.write_text(SOURCE)
    executable = directory / "test"
    subprocess.run([
        "gcc", "-std=gnu11", "-DCS96_GAMMA=1", "-ffunction-sections", "-fdata-sections",
        "-I" + str(ROOT), "-I" + str(ROOT / "driver/inc"),
        "-I" + str(ROOT / "third_party/libuv/include"), str(harness),
        "-Wl,--gc-sections", "-lm", "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True, timeout=10)
print("Gamma 点云解码验证通过：定点 TOF、保留位、峰值、回波选择及空回波。")
