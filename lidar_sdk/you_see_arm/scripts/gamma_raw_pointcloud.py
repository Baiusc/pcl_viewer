#!/usr/bin/env python3
"""接收 Gamma 原始点云，保存完整 UDP 载荷及可直接与 FPGA 对比的 SLOT 字节。"""
import argparse
import csv
from pathlib import Path
import socket
import struct

HEAD = struct.Struct("<BHHHH")
SLOT_SIZE = 1936
PACKET_SIZE = HEAD.size + SLOT_SIZE


def parse_packet(packet):
    """仅校验外层包；原始槽号异常仍保留，供定位 FPGA 或驱动问题。"""
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"包长应为 {PACKET_SIZE}，实际为 {len(packet)}")
    sof, frame, slot, seq, size = HEAD.unpack_from(packet)
    if sof != 0xAA or size != SLOT_SIZE:
        raise ValueError("原始点云协议头无效")
    raw = packet[HEAD.size:]
    word = int.from_bytes(raw[:4], "little")
    return {
        "frame_id": frame, "slot_number": slot, "seq_num": seq,
        "raw_slot_number": word & 0xFFF, "mirror_angle": word >> 12,
        "relative_time_us": int.from_bytes(raw[4:7], "little"),
        "fpga_frame_id": raw[14],
    }, raw


def parse_pixel(raw, index):
    """按 FPGA 顺序解析一个像素的两个回波，不做单位换算。"""
    if len(raw) != SLOT_SIZE or not 0 <= index < 120:
        raise ValueError("SLOT 长度或像素序号无效")
    echoes = []
    for echo in range(2):
        offset = 16 + index * 16 + echo * 8
        echoes.append({
            "ft": int.from_bytes(raw[offset:offset + 3], "little"),
            "et": int.from_bytes(raw[offset + 3:offset + 6], "little"),
            "pk": int.from_bytes(raw[offset + 6:offset + 8], "little"),
        })
    return echoes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="输出目录，必须尚不存在")
    parser.add_argument("--bind", default="0.0.0.0", help="本地监听地址")
    parser.add_argument("--port", type=int, default=50027, help="握手指定的数据端口")
    parser.add_argument("--source", help="只接收指定雷达 IP 的数据")
    parser.add_argument("--count", type=int, default=0, help="保存包数，0 表示持续到 Ctrl+C")
    args = parser.parse_args()
    if args.count < 0:
        parser.error("保存包数不能为负数")
    received = rejected = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        sock.bind((args.bind, args.port))
        args.output.mkdir(parents=True, exist_ok=False)
        with (args.output / "packets.bin").open("wb") as packets, \
                (args.output / "slots.bin").open("wb") as slots, \
                (args.output / "index.csv").open("w", newline="") as index:
            fields = ["packet_index", "frame_id", "slot_number", "seq_num",
                      "raw_slot_number", "mirror_angle", "relative_time_us", "fpga_frame_id"]
            writer = csv.DictWriter(index, fieldnames=fields)
            writer.writeheader()
            print(f"监听 {args.bind}:{args.port}；需由上位机完成握手并维持心跳。", flush=True)
            try:
                while args.count == 0 or received < args.count:
                    packet, address = sock.recvfrom(65535)
                    if args.source and address[0] != args.source:
                        continue
                    try:
                        metadata, raw = parse_packet(packet)
                    except ValueError:
                        rejected += 1
                        continue
                    packets.write(packet)
                    slots.write(raw)
                    writer.writerow(dict(packet_index=received, **metadata))
                    received += 1
            except KeyboardInterrupt:
                pass
    print(f"已保存 {received} 包，忽略 {rejected} 个非本协议包；目录：{args.output}")


if __name__ == "__main__":
    main()
