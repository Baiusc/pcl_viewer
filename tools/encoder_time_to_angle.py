#!/usr/bin/env python3
"""把 FPGA 采到的「整数刻度时间」换算成 ys_encoder_angle.csv 的角度表。

## 数据从哪来

PPT《薄款雷达点云左右晃动原因分析及解决措施》slide 6/8 的采集方案：

    FPGA：输出 (n, cnt)，n = 刻度序号 0~239，cnt = 相对 0 刻度的时间计数（单位自定义）
    ARM ：透传数据到上位机
    上位机：把 (n, cnt) 转成 (Θ, t)，并计算相邻刻度的瞬时转速

多圈（PPT 是 15 圈，现场也可 49 圈）对同一 n 的间隔取平均，得到每个刻度到
下一个刻度的时间间隔 Δt[n]（单位 us，实测均值约 1700us、理论 1000/240/2.5 = 1666.7us）。

## 为什么时间能换角度

转镜匀速旋转，转过的时间 ∝ 转过的角度。码盘刻度本身不均匀（PPT slide 4：
相邻刻度角度偏差标准差 0.046~0.066°、最大 0.223°），所以**不能**直接用
`n * 360/240`，必须按实测时间比例分配：

    angle[0]   = 0
    angle[i]   = 360 * ΣΔt[0..i-1] / ΣΔt[0..239]
    angle[240] = 360

这就是 `ys_encoder_angle.csv` 里 `angle` 列的来源（对应 xlsx sheet2 的
「角度（刻度时间测算）」列，与现有 csv 逐位一致）。

## 用法

    # 直接改下面「参数配置」区的常量后运行：
    python3 tools/encoder_time_to_angle.py

    参数含义：
    INPUT_PATH        输入文件：每行一个刻度时间间隔（240 行）
    OUTPUT_PATH       输出 csv 路径；设为 None 则只打印统计
    UNIT_SCALE        乘到每个数上的系数（默认 1，即输入已是 us）
                      # 单位是 125MHz 计数（1 计数 = 1/125 us）时设为 0.008
    FIRST_IS_OFFSET   True 表示首行是 FPGA 的 t0 偏移（约 266us，不是完整间隔），自动丢弃
                      # 此时输入是 241 行原始数据
    COMPARE_CSV       与参考角度表对比（如现有的 ys_encoder_angle.csv）；None 则不对比

只依赖标准库。
"""

import math
import sys

SCALES_PER_REV = 240          # 码盘一圈 240 个粗刻度
ANGLES_PER_SCALE = 360.0 / SCALES_PER_REV
NOMINAL_INTERVAL_US = 1.0e6 / SCALES_PER_REV / 2.5   # 1666.7us（2.5 圈/秒）


# ============================ 参数配置 ============================
INPUT_PATH = "./tools/intervals.txt"          # 每行一个刻度时间间隔（240 行）
OUTPUT_PATH = "./tools/enc.csv"          # 输出 csv 路径；None 则只打印统计
UNIT_SCALE = 1.0                      # 乘到每个数上的系数（1 = 输入已是 us）
FIRST_IS_OFFSET = True               # 首行是 FPGA 的 t0 偏移而非完整间隔时设 True
COMPARE_CSV = None                    # 参考角度表路径（如 ys_encoder_angle.csv）；None 不对比
# =================================================================


def load_values(path, unit_scale=1.0, first_is_offset=False):
    values = []
    with open(path, "r", encoding="utf-8") as handle:
        for lineno, line in enumerate(handle, 1):
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            # 兼容 "1,1703.25" 这种带序号的两列写法
            for sep in (",", "\t", ";"):
                if sep in text:
                    text = text.replace(sep, " ")
                    break
            parts = text.split()
            if not parts:
                continue
            try:
                value = float(parts[-1])
            except ValueError:
                raise SystemExit(f"{path}:{lineno} 无法解析为数字: {line.rstrip()}")
            if not math.isfinite(value):
                raise SystemExit(f"{path}:{lineno} 不是有限数: {value}")
            values.append(value * unit_scale)

    if first_is_offset:
        # FPGA 的 t0 是首个刻度相对计数起点的时间，不是一整格的间隔。
        values = values[1:]
    return values


def intervals_to_angles(intervals):
    """Δt → 累计角度，返回 241 个点（scale 0..240）。"""
    if len(intervals) != SCALES_PER_REV:
        raise SystemExit(
            f"需要 {SCALES_PER_REV} 个间隔，实际 {len(intervals)} 个。"
            f"如果是 241 行原始数据，把 FIRST_IS_OFFSET 设为 True。")
    if any(v <= 0.0 for v in intervals):
        bad = [i for i, v in enumerate(intervals) if v <= 0.0]
        raise SystemExit(f"间隔必须为正，异常下标: {bad[:8]}")

    total = sum(intervals)
    angles = [0.0]
    acc = 0.0
    for dt in intervals:
        acc += dt
        angles.append(360.0 * acc / total)
    return angles, total


def load_reference(path):
    table = {}
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            parts = line.strip().replace(",", " ").split()
            if len(parts) < 2 or parts[0] == "scale":
                continue
            try:
                table[int(parts[0])] = float(parts[1])
            except ValueError:
                continue
    return table


def write_csv(path, angles):
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("scale,angle\n")
        for index, angle in enumerate(angles):
            handle.write(f"{index},{angle:.9f}".rstrip("0").rstrip(".") + "\n")


def print_report(intervals, angles, total, reference=None):
    n = len(intervals)
    mean_interval = total / n
    rpm = 60.0 * 1e6 / total          # total 是 us

    print(f"间隔个数          : {n}")
    print(f"单圈总时间        : {total:.1f} us  ({total / 1000.0:.3f} ms)")
    print(f"平均每格间隔      : {mean_interval:.2f} us  (理论 {NOMINAL_INTERVAL_US:.1f} us)")
    print(f"折合转速          : {rpm:.2f} rpm  ({rpm / 60.0:.3f} 圈/秒)")

    deviation = [i - mean_interval for i in intervals]
    rms = math.sqrt(sum(d * d for d in deviation) / n)
    print(f"间隔偏离均值      : RMS {rms:.2f} us, max {max(abs(d) for d in deviation):.2f} us "
          f"({100.0 * max(abs(d) for d in deviation) / mean_interval:.3f}%)")

    deltas = [angles[i + 1] - angles[i] for i in range(len(angles) - 1)]
    dmean = sum(deltas) / len(deltas)
    dstd = math.sqrt(sum((d - dmean) ** 2 for d in deltas) / len(deltas))
    print(f"每格角度          : 均值 {dmean:.4f}°  标准差 {dstd:.4f}°  "
          f"范围 [{min(deltas):.4f}, {max(deltas):.4f}]  (理想 {ANGLES_PER_SCALE:.4f}°)")

    if reference:
        shared = [i for i in range(len(angles)) if i in reference]
        diffs = [angles[i] - reference[i] for i in shared]
        max_abs = max(abs(d) for d in diffs)
        diff_rms = math.sqrt(sum(d * d for d in diffs) / len(diffs))
        print(f"对比参考表        : {len(shared)} 点, 最大偏差 {max_abs:.4f}°, "
              f"RMS {diff_rms:.4f}°")
        worst = max(shared, key=lambda i: abs(angles[i] - reference[i]))
        print(f"最大偏差位置      : scale={worst} "
              f"本次 {angles[worst]:.4f}° 参考 {reference[worst]:.4f}°")


def main():
    intervals = load_values(INPUT_PATH, UNIT_SCALE, FIRST_IS_OFFSET)
    angles, total = intervals_to_angles(intervals)
    reference = load_reference(COMPARE_CSV) if COMPARE_CSV else None
    print_report(intervals, angles, total, reference)

    if OUTPUT_PATH:
        write_csv(OUTPUT_PATH, angles)
        print(f"已写出            : {OUTPUT_PATH} ({len(angles)} 行)")
    return 0


if __name__ == "__main__":
    sys.exit(main())