#!/usr/bin/env python3
"""6532 table_a time-peak-walk 拟合与留出验证；不带参数即可运行。"""

from pathlib import Path

import numpy as np
import pandas as pd


REPO = Path(__file__).resolve().parents[1]
DATA_DIR = REPO.parent / "data/固态雷达标定数据/6532标定算法效果对比"
TABLE_CSV = REPO.parent / "algo_solid_lidar/build/calib_data_legacy_parse/roi0_pixel_tables.csv"
OUTPUT_DIR = REPO / "build/range6532_timewalk_analysis"
TRAIN_CSV = DATA_DIR / "aFrame1_ROI0.csv"


def load_samples(filename: Path, known_distance_m: float, table_a, table_b,
                 full_wall: bool):
    frame = pd.read_csv(filename)
    row = frame["行row"].to_numpy(dtype=int)
    col = frame["列col"].to_numpy(dtype=int)
    selected = frame["最优回波"].to_numpy(dtype=int)
    intensity = np.where(selected == 1,
                         frame["p1_conv_intensity"],
                         frame["p0_conv_intensity"]).astype(float)
    ratio = intensity / table_a[row, col]
    direction_x = (np.cos(np.deg2rad(frame["俯仰角deg"])) *
                   np.cos(np.deg2rad(frame["水平角deg"]))).to_numpy()
    after_b = frame["距离m"].to_numpy() + table_b[row, col]
    wanted_correction = known_distance_m / direction_x - after_b
    valid = ((frame["有效"].to_numpy() == 1) & (selected >= 0) &
             (ratio > 0.002) & (ratio < 1.05) &
             (np.abs(wanted_correction) < 1.0))
    if full_wall:
        valid &= ((row <= 99) &
                  (known_distance_m / direction_x < 4.0))
    else:
        valid &= ((row >= 64) & (row <= 71) &
                  (col >= 94) & (col <= 100))
    return ratio, wanted_correction, after_b, direction_x, valid


def fit_equal_log_bins(ratio, correction, valid):
    # 每个强度区间等权，防止接近饱和的样本数量压倒弱回波样本。
    edges = np.geomspace(0.05, 1.05, 25)
    centers, medians, counts = [], [], []
    for low, high in zip(edges[:-1], edges[1:]):
        chosen = valid & (ratio >= low) & (ratio < high)
        if np.count_nonzero(chosen) < 20:
            continue
        centers.append(float(np.median(np.log(ratio[chosen]))))
        medians.append(float(np.median(correction[chosen])))
        counts.append(int(np.count_nonzero(chosen)))
    design = np.column_stack([np.ones(len(centers)), centers])
    offset_m, slope_m = np.linalg.lstsq(design, medians, rcond=None)[0]
    return float(offset_m), float(slope_m), centers, medians, counts


def evaluate(filename, distance_m, full_wall, table_a, table_b, offset, slope):
    ratio, _, after_b, direction_x, valid = load_samples(
        filename, distance_m, table_a, table_b, full_wall)
    walk = offset + slope * np.log(np.clip(ratio, 0.002, 1.05))
    before_error = after_b * direction_x - distance_m
    after_error = (after_b + walk) * direction_x - distance_m
    return {
        "file": filename.name,
        "samples": int(np.count_nonzero(valid)),
        "b_only_rmse_m": float(np.sqrt(np.mean(before_error[valid] ** 2))),
        "a_b_rmse_m": float(np.sqrt(np.mean(after_error[valid] ** 2))),
        "b_only_median_abs_m": float(np.median(np.abs(before_error[valid]))),
        "a_b_median_abs_m": float(np.median(np.abs(after_error[valid]))),
    }


def main():
    table = pd.read_csv(TABLE_CSV, encoding="utf-8-sig")
    table_a = table.pivot(index="row", columns="col", values="table_a_raw").to_numpy()
    table_b = table.pivot(index="row", columns="col", values="table_b_if_cm_m").to_numpy()
    ratio, correction, _, _, valid = load_samples(TRAIN_CSV, 0.99, table_a, table_b, True)
    offset, slope, centers, medians, counts = fit_equal_log_bins(ratio, correction, valid)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    pd.DataFrame({"log_intensity_ratio": centers,
                  "median_correction_m": medians,
                  "samples": counts}).to_csv(OUTPUT_DIR / "fit_bins.csv", index=False)
    pd.DataFrame([{"offset_m": offset, "log_slope_m": slope,
                   "min_intensity_ratio": 0.002,
                   "max_intensity_ratio": 1.05}]).to_csv(
                       OUTPUT_DIR / "range6532_timewalk_model.csv", index=False)

    cases = [
        (DATA_DIR / "aFrame1_ROI0.csv", 0.99, True),
        (DATA_DIR / "aFrame2_ROI0.csv", 0.99, True),
        (DATA_DIR / "e走廊1.5m靶板90%.csv", 1.5, False),
        (DATA_DIR / "b13mFrame1_ROI0.csv", 13.0, False),
        (DATA_DIR / "c走廊13m靶板2%.csv", 13.0, False),
    ]
    metrics = [evaluate(*case, table_a, table_b, offset, slope) for case in cases]
    pd.DataFrame(metrics).to_csv(OUTPUT_DIR / "validation.csv", index=False)
    with (OUTPUT_DIR / "REPORT.txt").open("w", encoding="utf-8") as report:
        report.write("模型: correction_m = offset_m + log_slope_m * ln(conv_intensity/table_a_raw)\n")
        report.write(f"offset_m={offset:.9f}\nlog_slope_m={slope:.9f}\n")
        for item in metrics:
            report.write(str(item) + "\n")
    print(f"PASS offset_m={offset:.9f} log_slope_m={slope:.9f} "
          f"holdout_rmse={metrics[1]['a_b_rmse_m']:.6f} "
          f"output={OUTPUT_DIR}")


if __name__ == "__main__":
    main()
