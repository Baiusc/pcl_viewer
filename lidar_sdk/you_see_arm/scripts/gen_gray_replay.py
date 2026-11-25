#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 CS96-gamma 灰度图回放数据（LOCAL_TEST 用）

数据布局（与驱动 gray_sl100 通道一致，小端）：
  每个 SLOT = 8B 头 + 120 × 8B 像素
    [0:8]   SLOT 头：byte0 低 5 位 = SLOT 序号 (0~19)，其余保留
    像素 i 的 8B：bit[19:0]  ECHO1 灰度
                  bit[39:20] ECHO2 灰度
                  bit[63:40] 保留
  一帧 = 20 个 SLOT

用法：
  python3 scripts/gen_gray_replay.py [输出路径] [帧数]
  默认输出 ./file/CS96-gamma/gray_data，200 帧（约 3.87 MB）
"""

import os
import struct
import sys

PIXELS_PER_SLOT = 120
SLOT_HEADER_SIZE = 8
PIXEL_SIZE = 8
SLOT_SIZE = SLOT_HEADER_SIZE + PIXELS_PER_SLOT * PIXEL_SIZE  # 968
SLOTS_PER_FRAME = 20


def build_slot(slot_number, frame_index):
    """合成一个 SLOT：斜条纹灰度图案，便于上位机肉眼核对解析是否正确"""
    slot = bytearray(SLOT_SIZE)
    slot[0] = slot_number & 0x1F  # SLOT 序号 [4:0]

    for pixel in range(PIXELS_PER_SLOT):
        # 沿像素方向渐变，并按 slot 平移形成斜条纹
        level = (pixel + slot_number * 6) % PIXELS_PER_SLOT
        echo1 = level * 34  # 0 ~ 4029，20 bit 有效范围内
        echo2 = (echo1 // 2) & 0xFFFFF
        value = (echo1 & 0xFFFFF) | ((echo2 & 0xFFFFF) << 20)
        struct.pack_into("<Q", slot, SLOT_HEADER_SIZE + pixel * PIXEL_SIZE, value)

    return bytes(slot)


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "./file/CS96-gamma/gray_data"
    frames = int(sys.argv[2]) if len(sys.argv) > 2 else 200

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
