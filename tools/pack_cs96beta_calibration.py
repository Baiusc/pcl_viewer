#!/usr/bin/env python3
"""把 CS96 Beta 的多份标定文件合并成单个 CSV。

输入（默认 data/param/cs96beta_calib_data）：
    ys_calibration.ini      参数、4 个面角、96 线水平角/距离修正、编码器粗值
    ys_encoder_angle.csv    scale 0~240 -> 机械绝对角
    reflectivity_map.csv    (pixel, distance) -> 5 档参考反射率
    spot_slope_map.csv      按面的角度旋转曲线

输出（默认 data/param/cs96beta_calibration.csv）：
    section,key,index,value0,value1,value2,value3,value4,value5

不同 section 对字段的解释：
    parameter                  key=参数名, value0=数值
    side_angular_offset        index=面号(0~3), value0=误差
    standard_encoder           key=angle_face_N, index=N-1, value0=编码器计数
    vcsel_index                key=pixel_in_vcsel_idN, index=N, value0=像素结束序号
    light_intensity            key=int_ldN, index=N, value0=强度
    horizontal_angular_offset  index=线号(0~95), value0=水平角修正(度)
    zero_distance_offset       index=线号(0~95), value0=零点偏移(cm)
    distance_factor            index=线号(0~95), value0=距离系数
    encoder_angle              index=scale(0~240), value0=机械角(度)
    reflectivity               index=pixel, value0=distance, value1..5=reflectance_0..4
    spot_slope                 index=面号(1~4), value0=角度, value1=修正量

用法：
    python3 tools/pack_cs96beta_calibration.py
    python3 tools/pack_cs96beta_calibration.py <源目录> <输出csv>
"""

from __future__ import annotations

import csv
import re
import sys
from pathlib import Path

HEADER = ["section", "key", "index", "value0", "value1", "value2", "value3", "value4", "value5"]

# 带索引的数组段：段名 -> (CSV section, 键名正则)
INDEXED_SECTIONS = {
    "horizontal_angular_offset": ("horizontal_angular_offset", re.compile(r"omega_delta(\d+)")),
    "zero_distance_offset": ("zero_distance_offset", re.compile(r"d(\d+)_delta")),
    "distance_factor": ("distance_factor", re.compile(r"a(\d+)")),
    "side_angular_offset": ("side_angular_offset", re.compile(r"side(\d+)_angular_offset")),
    "standard_encoder": ("standard_encoder", re.compile(r"angle_face_(\d+)")),
    "vcsel_index": ("vcsel_index", re.compile(r"pixel_in_vcsel_id(\d+)")),
    "light_intensity": ("light_intensity", re.compile(r"int_ld(\d+)")),
}


def strip_comment(text: str) -> str:
    return text.split(";", 1)[0].strip().strip('"')


def read_ini(path: Path):
    """返回 (params, arrays)；params 为 [(key, value)]，arrays 为 {section: {index: value}}。"""
    params: list[tuple[str, str]] = []
    arrays: dict[str, dict[int, str]] = {}
    section = ""
    for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.strip()
        if not line or line[0] in ";#":
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1].strip()
            continue
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = strip_comment(value)
        if not value:
            continue
        if section in INDEXED_SECTIONS:
            pattern = INDEXED_SECTIONS[section][1]
            match = pattern.fullmatch(key)
            if match:
                arrays.setdefault(section, {})[int(match.group(1))] = value
                continue
        # 其余（parameter / fpga / side_light 等）统一进 parameter，键名保留原样。
        params.append((key, value))
    return params, arrays


def rows_from_ini(path: Path):
    params, arrays = read_ini(path)
    for key, value in params:
        yield ["parameter", key, "0", value, "", "", "", "", ""]
    for section, (csv_section, _) in INDEXED_SECTIONS.items():
        for index, value in sorted(arrays.get(section, {}).items()):
            if csv_section == "standard_encoder":
                key = f"angle_face_{index}"
                idx = index - 1
            elif csv_section == "vcsel_index":
                key = f"pixel_in_vcsel_id{index}"
                idx = index
            elif csv_section == "light_intensity":
                key = f"int_ld{index}"
                idx = index
            else:
                key = ""
                idx = index
            yield [csv_section, key, str(idx), value, "", "", "", "", ""]


def rows_from_encoder(path: Path):
    with path.open(newline="", encoding="utf-8", errors="ignore") as handle:
        for row in csv.DictReader(handle):
            if not row or row.get("scale") in (None, ""):
                continue
            yield ["encoder_angle", "", str(int(float(row["scale"]))), row["angle"], "", "", "", "", ""]


def rows_from_reflectivity(path: Path):
    with path.open(newline="", encoding="utf-8", errors="ignore") as handle:
        for row in csv.DictReader(handle):
            if not row or row.get("pixel_target") in (None, ""):
                continue
            values = [row.get(f"reflectance_{i}", "") for i in range(5)]
            yield ["reflectivity", "", str(int(float(row["pixel_target"]))), row["distance"], *values]


def rows_from_spot_slope(path: Path):
    with path.open(newline="", encoding="utf-8", errors="ignore") as handle:
        for row in csv.DictReader(handle):
            if not row or row.get("Face") in (None, ""):
                continue
            face = str(row["Face"]).strip().lstrip("Ff")
            if not face.isdigit():
                continue
            yield ["spot_slope", "", face, row["Angle_FPGA"], row["Diff"], "", "", "", ""]


def main() -> int:
    source = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("data/param/cs96beta_calib_data")
    target = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("data/param/cs96beta_calibration.csv")

    if not source.is_dir():
        print(f"源目录不存在: {source}", file=sys.stderr)
        return 1

    rows: list[list[str]] = []
    rows.extend(rows_from_ini(source / "ys_calibration.ini"))

    encoder = source / "ys_encoder_angle.csv"
    if encoder.is_file():
        rows.extend(rows_from_encoder(encoder))
    else:
        print(f"警告: 缺少 {encoder}，编码器表为空", file=sys.stderr)

    reflectivity = source / "reflectivity_map.csv"
    if reflectivity.is_file():
        rows.extend(rows_from_reflectivity(reflectivity))

    spot_slope = source / "spot_slope_map.csv"
    if spot_slope.is_file():
        rows.extend(rows_from_spot_slope(spot_slope))

    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(HEADER)
        writer.writerows(rows)

    counts: dict[str, int] = {}
    for row in rows:
        counts[row[0]] = counts.get(row[0], 0) + 1
    print(f"已写入 {target}（{len(rows)} 行）")
    for section, count in counts.items():
        print(f"  {section}: {count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
