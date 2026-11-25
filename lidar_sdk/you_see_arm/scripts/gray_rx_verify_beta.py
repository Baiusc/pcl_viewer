#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CS96-beta 灰度图 UDP 接收验证脚本

按 doc/网络通讯/CS96-beta-灰度图-数据组帧协议.md 解析：
  每个 UDP 包 = 9B 协议头 + 1156B（半个 SLOT），总长 1165B
  一个 SLOT(2312B) = 8B 头 + 576 × 4B 像素(灰度值 [21:0])，由 seq_num 0/1 两片拼成
  一帧 = 168 个 SLOT(slot_number 0~167)

用法：
  python3 -I scripts/gray_rx_verify_beta.py [端口] [监听秒数]
  默认监听 50027 端口 10 秒（与 conf/ys_conf.ini 的 center_data_port 一致）
"""

import collections
import socket
import struct
import sys
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 50027
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

PKT_LEN = 1165
DATA_LEN = 1156
SLOT_LEN = 2312
SLOTS_PER_FRAME = 168
PIXELS = 576
GRAY_MASK = 0x3FFFFF


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    s.bind(("0.0.0.0", PORT))
    # 单包接收超时: 实机帧率低(约 2~3 秒/帧)且板端启动到首包有数秒延迟,
    # 超时过短会在数据开始前就退出(曾经设为 2 秒踩过此坑)
    s.settimeout(6.0)

    len_hist = collections.Counter()
    # frame_id -> slot_number -> {seq_num: 半个 SLOT}
    frames = collections.defaultdict(lambda: collections.defaultdict(dict))
    bad = collections.Counter()
    n = 0
    t0 = time.time()
    while time.time() - t0 < DURATION:
        try:
            data, _ = s.recvfrom(65535)
        except socket.timeout:
            break
        n += 1
        len_hist[len(data)] += 1
        if len(data) != PKT_LEN:
            bad["len"] += 1
            continue
        if data[0] != 0xAA:
            bad["sof"] += 1
            continue
        frame_id, slot_number, seq_num, data_len = struct.unpack_from("<HHHH", data, 1)
        if data_len != DATA_LEN or seq_num > 1:
            bad["hdr"] += 1
            continue
        if slot_number >= SLOTS_PER_FRAME:
            bad["slot"] += 1
            continue
        frames[frame_id][slot_number][seq_num] = data[9:9 + data_len]

    print("总包数: %d" % n)
    print("包长分布: %s" % dict(len_hist))
    print("异常: %s" % (dict(bad) if bad else "无"))
    print("收到帧数: %d" % len(frames))

    checked_sample = False
    for fid in sorted(frames.keys()):
        slots = frames[fid]
        complete = [sn for sn, v in slots.items() if set(v.keys()) == {0, 1}]
        print("frame_id=%d: slot 数=%d, 完整(两片)=%d, slot 范围=[%d,%d]" %
              (fid, len(slots), len(complete), min(slots), max(slots)))

        for sn in sorted(complete)[:3]:
            slot = frames[fid][sn][0] + frames[fid][sn][1]
            assert len(slot) == SLOT_LEN, "SLOT 长度 %d != %d" % (len(slot), SLOT_LEN)
            head_sn = struct.unpack_from("<H", slot, 0)[0]
            px0 = struct.unpack_from("<I", slot, 8)[0]
            px287 = struct.unpack_from("<I", slot, 8 + 287 * 4)[0]  # 片0 最后一个像素
            px575 = struct.unpack_from("<I", slot, 8 + 575 * 4)[0]  # 片1 最后一个像素
            assert head_sn == sn, "头内槽号 %d != 协议头槽号 %d" % (head_sn, sn)
            assert (px0 >> 22) == 0 and (px575 >> 22) == 0, "保留位非 0"
            print("  slot=%d: 头内槽号=%d, pixel0=%d, pixel287=%d, pixel575=%d" %
                  (sn, head_sn, px0 & GRAY_MASK, px287 & GRAY_MASK, px575 & GRAY_MASK))
        if complete:
            checked_sample = True
            break

    if not checked_sample:
        print("警告: 未收到任一完整 SLOT(两片齐全), 请检查 seq_num 是否 0/1 都到达")


if __name__ == "__main__":
    main()
