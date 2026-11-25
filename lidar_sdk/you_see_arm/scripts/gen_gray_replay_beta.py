#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 CS96-beta 灰度图回放数据（LOCAL_TEST 用）

数据布局（与驱动 gray 通道一致，小端）：
  每个 SLOT = 8B 头 + 576 × 4B 像素
    [0:2]   SLOT 头：slot_num (0~167)
    [2:8]   保留
    像素 i 的 4B：bit[21:0] 灰度值，bit[31:22] 保留
  一帧 = 168 个 SLOT

用法：
  python3 scripts/gen_gray_replay_beta.py [输出路径] [帧数]
  默认输出 ./file/CS96-beta/gray_data，10 帧（约 3.9 MB）
"""

import os
import struct
import sys

PIXELS_PER_SLOT = 576
SLOT_HEADER_SIZE = 8
PIXEL_SIZE = 4
SLOT_SIZE = SLOT_HEADER_SIZE + PIXELS_PER_SLOT * PIXEL_SIZE  # 2312
SLOTS_PER_FRAME = 168

MAX_GRAY = (1 << 22) - 1  # 22 bit 灰度满量程 4194303
GRAY_STEP = MAX_GRAY // (PIXELS_PER_SLOT - 1)


def build_slot(slot_number, frame_index):
    """合成一个 SLOT：斜条纹灰度图案，便于上位机肉眼核对解析是否正确"""
    slot = bytearray(SLOT_SIZE)
    struct.pack_into("<H", slot, 0, slot_number & 0xFFFF)  # SLOT 序号 [15:0]

    for pixel in range(PIXELS_PER_SLOT):
        # 沿像素方向渐变，并按 slot 平移形成斜条纹
        level = (pixel + slot_number * 3) % PIXELS_PER_SLOT
        value = (level * GRAY_STEP) & 0x3FFFFF  # 0 ~ 4194303，22 bit 有效范围内
        struct.pack_into("<I", slot, SLOT_HEADER_SIZE + pixel * PIXEL_SIZE, value)

    return bytes(slot)


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "./file/CS96-beta/gray_data"
    frames = int(sys.argv[2]) if len(sys.argv) > 2 else 10

    data = bytearray()
    for frame in range(frames):
        for slot in range(SLOTS_PER_FRAME):
            data += build_slot(slot, frame)

    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(out_path, "wb") as fp:
        fp.write(data)

    print(
        "生成完毕: %s, %d 帧 × %d SLOT × %d B = %d B"
        % (out_path, frames, SLOTS_PER_FRAME, SLOT_SIZE, len(data))
    )


if __name__ == "__main__":
    main()
