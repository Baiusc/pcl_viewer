#!/usr/bin/env python3
"""本地回环模拟面阵雷达 UDP 点云，用于验证 cloud_viewer 在线接收与解析。

协议来源：matrix_lidar_sdk/global.h
  head    : <HBBH  0xA55A, version_major=1, data_type=1, reserved
  protocol: <BHHBHHHBBB11sBIQ
  pixel   : <HhhhBB  (distance cm, x/y/z raw, reflect, label)

用法:
  python3 tools/mock_radar_sender.py [--rows 20] [--cols 16] [--fps 20] [--seconds 10]
"""

import argparse
import math
import socket
import struct
import time

HEAD = struct.Struct("<HBBH")
PROTO = struct.Struct("<BHHBHHHBBB11sBIQ")
PIXEL = struct.Struct("<HhhhBB")

HEAD_MAGIC = 0xA55A
DATA_TYPE_CLOUD = 0x01
POINT_TYPE_DIST_XYZ = 2


def build_packet(col, row, points, frame_flag, sequence, frame_cnt, timestamp_us):
    payload = b"".join(PIXEL.pack(*point) for point in points)
    head = HEAD.pack(HEAD_MAGIC, 1, DATA_TYPE_CLOUD, 0)
    proto = PROTO.pack(
        0,                 # version_minor
        sequence & 0xFFFF, # data_sequence
        frame_cnt & 0xFFFF,
        0,                 # time_type
        col,
        row,
        len(points),
        POINT_TYPE_DIST_XYZ,
        1,                 # echo_num
        frame_flag,
        b"\0" * 11,
        0,                 # crc8
        0,                 # crc32
        timestamp_us,
    )
    return head + proto + payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--radar-ip", default="127.0.0.1")
    parser.add_argument("--radar-port", type=int, default=6001)
    parser.add_argument("--local-port", type=int, default=6002)
    parser.add_argument("--rows", type=int, default=20)
    parser.add_argument("--cols", type=int, default=16)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.radar_ip, args.radar_port))

    sequence = 0
    frame_cnt = 0
    deadline = time.time() + args.seconds

    while time.time() < deadline:
        timestamp_us = int(time.time() * 1e6)
        groups = (args.cols + 3) // 4

        for row in range(args.rows):
            for group in range(groups):
                points = []
                for k in range(4):
                    col = group * 4 + k
                    if col >= args.cols:
                        break
                    # 构造一个朝向相机的曲面，便于肉眼确认。
                    raw_x = int((col - args.cols / 2) * 40)
                    raw_y = int((row - args.rows / 2) * 40)
                    raw_z = 800 + int(120 * math.sin(col * 0.4))
                    distance_cm = int(math.sqrt(raw_x ** 2 + raw_y ** 2 + raw_z ** 2) * 0.1)
                    points.append((distance_cm, raw_x, raw_y, raw_z, 200, 0))

                is_first = (row == 0 and group == 0)
                is_last = (row == args.rows - 1 and group == groups - 1)
                flag = 0 if is_first else (2 if is_last else 1)

                packet = build_packet(group * 4, row, points, flag, sequence, frame_cnt,
                                      timestamp_us)
                sock.sendto(packet, (args.radar_ip, args.local_port))
                sequence += 1

        frame_cnt += 1
        time.sleep(1.0 / args.fps)

    sock.close()
    print(f"sent {frame_cnt} frames, {sequence} packets")


if __name__ == "__main__":
    main()
