#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 spot_slope_map.csv 转成同目录同名的 .bin（已存在则覆盖）。

二进制布局严格按

    ReposCV/LightSpotSlope/src/algo_calib_rotate_angle.cpp
    CalibRotateAngle::saveTable()

即：

    uint8   face_count
    每面重复：
        uint8   face_index        # F1 -> 0, F2 -> 1, ...
        uint16  start_code        # 本面起始编码值（小端）
        uint16  kv_count          # 该面的点数（小端）
        每个点 8 字节：
            float angle           # Angle_FPGA
            float diff            # Diff

start_code 在 CSV 里没有对应列，脚本沿用同目录现有 .bin 里的值；
若 .bin 不存在或读不出，则退化为默认值 1。

注意：本脚本只负责“CSV -> saveTable 布局”。仓库里现存的部分
spot_slope_map.bin 的点数据是压缩布局（u16 + i16 + float），
与 saveTable 不同；若目标端用的是压缩布局的读取接口，需要另行适配。

直接运行，无需参数：
    python3 spot_slope_csv_to_bin.py
"""

import csv
import struct
from pathlib import Path

src_csv = "/media/baiusc/DATA11/Repositories/ReposLidar/pcl_viewer/data/param/cs96beta_calib_data/spot_slope_map.csv"

DEFAULT_START_CODE = 1


def read_start_codes(bin_path):
    """按 saveTable 的头部布局读每面的 start_code。

    头部(count/idx/start_code/kv_count)与点布局无关，且两种已知布局
    每点都是 8 字节，所以可以安全跳过点数据。
    """
    starts = {}
    if not bin_path.is_file():
        return starts
    data = bin_path.read_bytes()
    if not data:
        return starts

    face_count = data[0]
    offset = 1
    for _ in range(face_count):
        if offset + 5 > len(data):
            break
        face_index = data[offset]
        start_code = struct.unpack_from("<H", data, offset + 1)[0]
        kv_count = struct.unpack_from("<H", data, offset + 3)[0]
        starts[face_index] = start_code
        offset += 5 + kv_count * 8
    return starts


def load_csv(csv_path):
    """按 Face 分组，保持文件中的原有顺序。"""
    groups = {}
    with csv_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {"Face", "Angle_FPGA", "Diff"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise SystemExit("CSV 缺少列: " + ", ".join(sorted(missing)))
        for row in reader:
            face_text = (row["Face"] or "").strip()
            if not face_text:
                continue
            # 支持 F1 / f1 / 1
            digits = "".join(ch for ch in face_text if ch.isdigit())
            if not digits:
                raise SystemExit("无法解析 Face 列: " + repr(face_text))
            face_index = int(digits) - 1
            groups.setdefault(face_index, []).append(
                (float(row["Angle_FPGA"]), float(row["Diff"]))
            )
    if not groups:
        raise SystemExit("CSV 里没有有效数据行")
    return groups


def write_bin(bin_path, groups, start_codes):
    faces = sorted(groups)
    payload = bytearray()
    payload.append(len(faces) & 0xFF)
    for face_index in faces:
        points = groups[face_index]
        start_code = start_codes.get(face_index, DEFAULT_START_CODE)
        payload += struct.pack("<BHH", face_index & 0xFF, start_code & 0xFFFF,
                               len(points) & 0xFFFF)
        for angle, diff in points:
            payload += struct.pack("<ff", angle, diff)
    bin_path.write_bytes(bytes(payload))
    return faces


def verify(bin_path, groups):
    """回读校验：面数、面号、点数、每点角度与修正量都要对得上。"""
    data = bin_path.read_bytes()
    face_count = data[0]
    offset = 1
    problems = []
    for _ in range(face_count):
        face_index, start_code, kv_count = struct.unpack_from("<BHH", data, offset)
        offset += 5
        expected = groups.get(face_index, [])
        if kv_count != len(expected):
            problems.append("面 %d 点数 %d != CSV %d" % (face_index, kv_count, len(expected)))
        for i in range(kv_count):
            angle, diff = struct.unpack_from("<ff", data, offset)
            offset += 8
            if i < len(expected):
                exp_angle, exp_diff = expected[i]
                if abs(angle - exp_angle) > 1e-4 or abs(diff - exp_diff) > 1e-4:
                    problems.append("面 %d 第 %d 点不一致" % (face_index, i))
        del start_code
    if offset != len(data):
        problems.append("尾部有 %d 字节多余数据" % (len(data) - offset))
    return problems


def main():
    csv_path = Path(src_csv)
    if not csv_path.is_file():
        raise SystemExit("找不到 CSV: %s" % csv_path)

    bin_path = csv_path.with_suffix(".bin")
    groups = load_csv(csv_path)
    start_codes = read_start_codes(bin_path)

    faces = write_bin(bin_path, groups, start_codes)
    problems = verify(bin_path, groups)

    print("CSV   : %s" % csv_path)
    print("输出  : %s (%d 字节)" % (bin_path, bin_path.stat().st_size))
    for face_index in faces:
        print("  面 %d: %3d 点, start_code=%d"
              % (face_index, len(groups[face_index]),
                 start_codes.get(face_index, DEFAULT_START_CODE)))
    if problems:
        print("校验失败:")
        for item in problems:
            print("  - " + item)
        raise SystemExit(1)
    print("校验通过：回读结果与 CSV 完全一致")


if __name__ == "__main__":
    main()
