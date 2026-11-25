#!/usr/bin/env python3
"""验证 Beta 水平角解算和整帧面号行为。"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include "core/ys_conf.h"
#include "ys_business/algo/ys_algo_horizontal_corners.h"
#include "ys_business/algo/ys_algo_def.h"
#include "ys_business/decode/ys_decode_internal.h"
#include "ys_business/producer/point_cloud/ys_producer_point_cloud_cs96_beta.c"
#include <assert.h>
#include <math.h>
calibration_parm_t cal_param;
configuration_t config;
uint16_t total_slots = 800;
int org_point_cloud_slot_size = 16;
int g_encoder_angle_cnt = 256;
static float encoder[256];
float *g_encoder_angle = encoder;
void log_log(int level, const char *file, int line, const char *fmt, ...) {}
#if BETA_SPOT_SLOPE
static int calls;
float get_beta_spot_slope(float angle, uint16_t face)
{
    ++calls;
    assert(face == 2);
    assert(fabsf(angle - 20.0f) < 0.001f);
    return angle + 3.0f;
}
#endif

static void check_scan_face(void)
{
    uint8_t batch[2][16] = { { 0 }, { [0] = 1, [14] = 2 } };
    beta_scan_face_t scan = { .face = -1 };
    uint16_t sequence;
    config.work_mode = FORMAL_POINT_CLOUD_MODE;
    assert(beta_scan_sequence(&scan, batch[0], 0, 2, 0, &sequence));

    uint8_t bytes[sizeof(raw_protocal_data_packet_t) + 16] = { 0 };
    raw_protocal_data_packet_t *raw = (void *)bytes;
    raw->seq_num = sequence;
    point_cloud_head_t head;
    assert(decode_build_protocal_head(raw, &head) > 0);
    assert(head.slot_number == 0 && head.frame_index == 2);

    /* 即使尾部的逐槽判面不同，也使用已确定的整帧面号。 */
    batch[0][0] = 799 & 255;
    batch[0][1] = 799 >> 8;
    batch[0][14] = 3;
    assert(beta_scan_sequence(&scan, batch[0], 0, 1, 799, &sequence));
    raw->seq_num = sequence;
    memcpy(raw->value, batch[0], 16);
    assert(decode_build_protocal_head(raw, &head) > 0);
    assert(head.slot_number == 799 && head.frame_index == 2);

    /* 下一帧丢了同步槽，仍能按槽号回绕重新判面。 */
    batch[1][14] = 1;
    assert(beta_scan_sequence(&scan, batch[1], 0, 1, 1, &sequence));
    assert(scan.face == 1);

    config.work_mode = TEST_POINT_CLOUD_MODE;
    assert(beta_scan_sequence(&scan, batch[0], 1, 2, 1, &sequence));
    assert(sequence == 1);
}

int main(void)
{
    cal_param.omega0.f = -60.0f;
    g_staring_angle[2] = 350.0f;
    encoder[7] = 30.0f;
    float angle = get_horizontal_corners(1, 2, 7U << 12, 0);
#if BETA_SPOT_SLOPE
    assert(fabsf(angle - 23.0f) < 0.001f);
    assert(calls == 1);
#else
    assert(fabsf(angle - 20.0f) < 0.001f);
#endif
    (void)get_horizontal_corners(0, 2, 0, 0);
#if BETA_SPOT_SLOPE
    assert(calls == 1);
#endif
    check_scan_face();
}
'''
with tempfile.TemporaryDirectory(prefix="ys-horizontal-") as root:
    directory = Path(root)
    harness = directory / "test.c"
    harness.write_text(SOURCE)
    for enabled in (0, 1):
        executable = directory / f"test-{enabled}"
        subprocess.run([
            "gcc", "-std=gnu11", "-ffunction-sections", "-Wl,--gc-sections",
            "-DCS96_BETA=1", f"-DBETA_SPOT_SLOPE={enabled}",
            "-I" + str(ROOT), "-I" + str(ROOT / "driver/inc"),
            str(harness), str(ROOT / "ys_business/algo/ys_algo_horizontal_corners.c"),
            str(ROOT / "ys_business/algo/ys_algo_build_protocal_head.c"),
            str(ROOT / "ys_business/decode/ys_decode_cs96_beta.c"),
            "-lm", "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)
print("Beta 水平角、同步 SLOT、整帧面号、漏同步槽和原始序号兼容检查通过。")
