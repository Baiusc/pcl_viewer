#!/usr/bin/env python3
"""验证真实 Gamma 消费流水线逐字节透传，以及上位机原始字段解析。"""
from pathlib import Path
import struct
import subprocess
import tempfile

from gamma_raw_pointcloud import parse_packet, parse_pixel

ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include "core/ys_conf.h"
#include "ys_business/consumer/ys_consumer_pipeline.h"
#include <assert.h>

calibration_parm_t cal_param;
configuration_t config;
int raw_point_cloud_full_size = 1945;
static uint8_t expected[1945];
static int sent;
void log_log(int level, const char *file, int line, const char *fmt, ...) {}

int send_data(udp_t *udp, uint8_t *data, int len)
{
    assert(len == sizeof(expected));
    assert(memcmp(data, expected, sizeof(expected)) == 0);
    ++sent;
    consumer_pipeline_sighandler(0);
    return len;
}

int main(void)
{
    _Static_assert(sizeof(raw_protocal_data_packet_t) - 1 == 9, "外层头为 9 字节");
    _Static_assert(sizeof(org_point_cloud_packet_t) - 1 == 16, "原始头为 16 字节");
    _Static_assert(sizeof(point_cloud_pixel_data_t) == 16, "每像素 16 字节");
    uint8_t input[1945];
    data_t data = { .buf = input, .len = sizeof(input) };
    node_t node = { .data = &data };
    block_t block = { .node = &node };
    block.next = &block;
    memory_pool_t pool = {0};
    atomic_store(&pool.head, &block);
    udp_t udp = { .pool = &pool };
    cal_param.lines_v = 120;
    /* 所有面均禁用时也必须发出全部原始 SLOT，包括帧号回绕。 */
    cal_param.side_light = 0;
    for (int frame = 0; frame < 256; ++frame) {
        for (int i = 0; i < sizeof(input); ++i) {
            input[i] = (uint8_t)(i * 37 + i / 16 + frame);
        }
        raw_protocal_data_packet_t *packet = (void *)input;
        packet->sof = 0xAA;
        packet->frame_id = (uint16_t)(65530 + frame);
        packet->slot_number = frame % 2 ? 799 : 0;
        packet->seq_num = frame % 10;
        packet->data_len = 1936;
        input[9 + 14] = (uint8_t)frame;
        memcpy(expected, input, sizeof(input));
        atomic_store(&node.write_ok, READABLE);
        consumer_pipeline_reset_run_flag();
        consumer_pipeline_raw_point_cloud(&udp);
        assert(atomic_load(&node.write_ok) == WRITABLE);
        assert(memcmp(input, expected, sizeof(input)) == 0);
    }
    assert(sent == 256);
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="ys-gamma-raw-") as folder:
        folder = Path(folder)
        harness = folder / "test.c"
        harness.write_text(SOURCE)
        executable = folder / "test"
        subprocess.run([
            "gcc", "-std=gnu11", "-DCS96_GAMMA=1", "-ffunction-sections", "-fdata-sections",
            "-I" + str(ROOT), "-I" + str(ROOT / "driver/inc"),
            "-I" + str(ROOT / "third_party/libuv/include"),
            str(harness), str(ROOT / "core/ys_management.c"), str(ROOT / "ys_business/consumer/ys_consumer_pipeline.c"),
            str(ROOT / "ys_business/consumer/models/ys_consumer_model_cs96_gamma.c"),
            "-Wl,--gc-sections", "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)

    raw = bytearray((i * 37 + i // 16) % 256 for i in range(1936))
    struct.pack_into("<I", raw, 0, (0xABCDE << 12) | 799)
    raw[4:7] = (0xFEDCBA).to_bytes(3, "little")
    raw[14] = 255
    raw[16:24] = bytes.fromhex("56 34 12 bc 9a 78 f0 de")
    packet = struct.pack("<BHHHH", 0xAA, 65535, 799, 9, 1936) + raw
    metadata, saved = parse_packet(packet)
    assert saved == raw
    assert metadata == dict(frame_id=65535, slot_number=799, seq_num=9,
                            raw_slot_number=799, mirror_angle=0xABCDE,
                            relative_time_us=0xFEDCBA, fpga_frame_id=255)
    assert parse_pixel(saved, 0)[0] == dict(ft=0x123456, et=0x789ABC, pk=0xDEF0)
    assert parse_pixel(saved, 119)[1]["pk"] == int.from_bytes(raw[-2:], "little")
    for invalid in (packet[:-1], packet + b"\0", bytes(1945),
                    packet[:7] + b"\0\0" + packet[9:]):
        try:
            parse_packet(invalid)
        except ValueError:
            continue
        raise AssertionError("必须拒绝非法包长或协议头")
    print("Gamma 原始点云验证通过：256 个 FPGA 帧号、禁用面、完整字节及像素顺序、协议解析。")


if __name__ == "__main__":
    main()
