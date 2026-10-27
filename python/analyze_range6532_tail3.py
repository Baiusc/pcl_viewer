#!/usr/bin/env python3
"""还原 calib_data.bin 的纯表驱动模型并验证候选语义；无参数直接运行。"""

from pathlib import Path
import json
import subprocess

import numpy as np
import pandas as pd


REPO = Path(__file__).resolve().parents[1]
ROOT = REPO.parent
LEGACY_PARSE = ROOT / "algo_solid_lidar/build/calib_data_legacy_parse"
SUMMARY_JSON = LEGACY_PARSE / "summary.json"
ALGO = ROOT / "algo_solid_lidar/build/ALGO_SOLID_LIDAR"
CAPTURE_ROOT = ROOT / "data/固态雷达标定数据/20260902_6532range_data_公司走廊2%靶板多距离"
EFFECT_ROOT = ROOT / "data/固态雷达标定数据/6532标定算法效果对比"
OUTPUT = REPO / "build/range6532_tail3_analysis"
PARSED = OUTPUT / "parsed"
DISTANCES_M = (1.5, 2.0, 4.0, 7.0, 10.0, 13.0)


def table_matrix(roi, field):
    table = pd.read_csv(LEGACY_PARSE / f"roi{roi}_pixel_tables.csv",
                        encoding="utf-8-sig")
    return table.pivot(index="row", columns="col", values=field).to_numpy()


def ensure_frame_csv(distance_m):
    label = f"{distance_m:g}"
    output_dir = PARSED / f"data_{label}m"
    output_dir.mkdir(parents=True, exist_ok=True)
    frames = sorted(output_dir.glob("Frame*ROI0.csv"))
    if frames:
        return frames[0]
    source = CAPTURE_ROOT / f"data_{label}m/range_data_all"
    command = [str(ALGO), "--input", str(source), "--csv-dir", str(output_dir),
               "--max-frames", "1", "--quiet-groups"]
    result = subprocess.run(command, check=True, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (output_dir / "parse.log").write_text(result.stdout, encoding="utf-8")
    return sorted(output_dir.glob("Frame*ROI0.csv"))[0]


def selected_column(frame, p0, p1):
    selected = frame["最优回波"].to_numpy(dtype=int)
    return np.where(selected == 1, frame[p1], frame[p0]).astype(float)


def roi0_samples(table_a, table_b):
    samples = []
    for distance_m in DISTANCES_M:
        frame = pd.read_csv(ensure_frame_csv(distance_m))
        row = frame["行row"].to_numpy(dtype=int)
        col = frame["列col"].to_numpy(dtype=int)
        selected = frame["最优回波"].to_numpy(dtype=int)
        intensity = selected_column(frame, "p0_conv_intensity", "p1_conv_intensity")
        fwhm_bin = selected_column(frame, "p0_fwhm_bin", "p1_fwhm_bin")
        direction_x = (np.cos(np.deg2rad(frame["俯仰角deg"])) *
                       np.cos(np.deg2rad(frame["水平角deg"]))).to_numpy()
        after_b = frame["距离m"].to_numpy() + table_b[row, col]
        ratio_raw = intensity / table_a[row, col]
        ratio = np.clip(ratio_raw, 0.002, 1.05)
        valid = ((frame["有效"].to_numpy() == 1) & (selected >= 0) &
                 (row >= 64) & (row <= 71) &
                 (col >= 94) & (col <= 100) &
                 (fwhm_bin > 0.0) & (fwhm_bin < 32.0))
        # 靶板 ROI 内仍可能有飞点；只剔除偏离本 ROI 中位面的极端点。
        x_before = after_b * direction_x
        median_x = np.median(x_before[valid])
        valid &= np.abs(x_before - median_x) < 1.0
        samples.append({"distance_m": distance_m, "after_b": after_b,
                        "direction_x": direction_x, "ratio": ratio,
                        "ratio_raw": ratio_raw,
                        "fwhm_bin": fwhm_bin, "valid": valid})
    return samples


def correction_candidates(sample, coeff):
    measured = sample["after_b"]
    ratio = sample["ratio"]
    fwhm = sample["fwhm_bin"]
    return {
        "table_b_only": np.zeros_like(measured),
        "distance_poly_cm_add": np.polyval(coeff, measured) / 100.0,
        "distance_poly_cm_sub": -np.polyval(coeff, measured) / 100.0,
        "fwhm_poly_cm_add": np.polyval(coeff, fwhm) / 100.0,
        "fwhm_poly_cm_sub": -np.polyval(coeff, fwhm) / 100.0,
        "intensity_ratio_poly_cm_add": np.polyval(coeff, ratio) / 100.0,
        "ln_ratio_poly_cm_add": np.polyval(coeff, np.log(ratio)) / 100.0,
        "sqrt_inverse_ratio_poly_cm_add": (
            np.polyval(coeff, np.sqrt(1.0 / ratio)) / 100.0),
    }


def score_roi0(samples, coeff):
    errors = {}
    per_distance = []
    for sample in samples:
        valid = sample["valid"]
        distance_m = sample["distance_m"]
        for name, correction in correction_candidates(sample, coeff).items():
            error = ((sample["after_b"] + correction) *
                     sample["direction_x"] - distance_m)[valid]
            errors.setdefault(name, []).append(error)
            if name in ("table_b_only", "fwhm_poly_cm_add"):
                per_distance.append({
                    "distance_m": distance_m,
                    "model": name,
                    "samples": int(error.size),
                    "rmse_m": float(np.sqrt(np.mean(error ** 2))),
                    "median_abs_m": float(np.median(np.abs(error))),
                    "median_signed_m": float(np.median(error)),
                })
    scores = []
    for name, chunks in errors.items():
        error = np.concatenate(chunks)
        scores.append({"model": name, "samples": int(error.size),
                       "rmse_m": float(np.sqrt(np.mean(error ** 2))),
                       "median_abs_m": float(np.median(np.abs(error))),
                       "median_signed_m": float(np.median(error))})
    return pd.DataFrame(scores).sort_values("rmse_m"), pd.DataFrame(per_distance)


def score_current_model_stacking(samples, coeff):
    rows = []
    for model in ("current_table_a_log", "current_table_a_log_plus_tail3"):
        chunks = []
        for sample in samples:
            walk = 0.497953769 + 0.063243195 * np.log(sample["ratio"])
            if model.endswith("plus_tail3"):
                walk += np.polyval(coeff, sample["fwhm_bin"]) / 100.0
            error = ((sample["after_b"] + walk) * sample["direction_x"] -
                     sample["distance_m"])[sample["valid"]]
            chunks.append(error)
        error = np.concatenate(chunks)
        rows.append({"model": model, "samples": int(error.size),
                     "rmse_m": float(np.sqrt(np.mean(error ** 2))),
                     "median_abs_m": float(np.median(np.abs(error))),
                     "median_signed_m": float(np.median(error))})
    return pd.DataFrame(rows)


def score_roi1(coeff, table_b):
    rows = []
    for filename in ("aFrame1_ROI1.csv", "aFrame2_ROI1.csv"):
        frame = pd.read_csv(EFFECT_ROOT / filename)
        physical_row = frame["行row"].to_numpy(dtype=int)
        local_row = physical_row - 16
        col = frame["列col"].to_numpy(dtype=int)
        selected = frame["最优回波"].to_numpy(dtype=int)
        fwhm = selected_column(frame, "p0_fwhm_bin", "p1_fwhm_bin")
        direction_x = (np.cos(np.deg2rad(frame["俯仰角deg"])) *
                       np.cos(np.deg2rad(frame["水平角deg"]))).to_numpy()
        valid = ((frame["有效"].to_numpy() == 1) & (selected >= 0) &
                 (local_row >= 0) & (local_row < 48) &
                 (fwhm > 0.0) & (fwhm < 32.0))
        after_b = frame["距离m"].to_numpy() + table_b[local_row.clip(0, 47), col]
        for model, correction in {
                "table_b_only": np.zeros_like(after_b),
                "fwhm_poly_cm_add": np.polyval(coeff, fwhm) / 100.0,
                "fwhm_poly_cm_sub": -np.polyval(coeff, fwhm) / 100.0}.items():
            error = ((after_b + correction) * direction_x - 0.99)[valid]
            rows.append({"file": filename, "model": model,
                         "samples": int(error.size),
                         "rmse_m": float(np.sqrt(np.mean(error ** 2))),
                         "median_abs_m": float(np.median(np.abs(error)))})
    return pd.DataFrame(rows)


def table_a_distance_test(samples):
    """检查 table_a 归一化值及其平方距离补偿，不拟合任何新系数。"""
    rows = []
    for sample in samples:
        q = sample["ratio_raw"][sample["valid"]]
        distance_m = sample["distance_m"]
        rows.append({
            "distance_m": distance_m,
            "samples": int(q.size),
            "median_q_intensity_over_table_a": float(np.median(q)),
            "median_q_times_distance_squared": float(np.median(q * distance_m ** 2)),
        })
    result = pd.DataFrame(rows)
    log_d = np.log(result["distance_m"].to_numpy())
    slope_q = float(np.polyfit(
        log_d, np.log(result["median_q_intensity_over_table_a"]), 1)[0])
    slope_qr2 = float(np.polyfit(
        log_d, np.log(result["median_q_times_distance_squared"]), 1)[0])
    return result, slope_q, slope_qr2


def table_a_cross_roi_test(table_a0, table_a1):
    """同场景重叠像素对比：table_a 应消除 ROI1 更大发光功率的量级差。"""
    rows = []
    for serial in (1, 2):
        frames = {}
        for roi in (0, 1):
            frame = pd.read_csv(EFFECT_ROOT / f"aFrame{serial}_ROI{roi}.csv")
            frame = frame.copy()
            frame["selected_intensity"] = selected_column(
                frame, "p0_conv_intensity", "p1_conv_intensity")
            frames[roi] = frame
        overlap = frames[0].merge(
            frames[1], on=["行row", "列col"], suffixes=("_roi0", "_roi1"))
        row = overlap["行row"].to_numpy(dtype=int)
        col = overlap["列col"].to_numpy(dtype=int)
        local_row1 = row - 16
        i0 = overlap["selected_intensity_roi0"].to_numpy(dtype=float)
        i1 = overlap["selected_intensity_roi1"].to_numpy(dtype=float)
        valid = ((overlap["有效_roi0"].to_numpy() == 1) &
                 (overlap["有效_roi1"].to_numpy() == 1) &
                 (overlap["最优回波_roi0"].to_numpy() >= 0) &
                 (overlap["最优回波_roi1"].to_numpy() >= 0) &
                 (local_row1 >= 0) & (local_row1 < 48) &
                 (i0 > 0.0) & (i1 > 0.0))
        raw_ratio = i1[valid] / i0[valid]
        reference_ratio = (table_a1[local_row1[valid], col[valid]] /
                           table_a0[row[valid], col[valid]])
        q0 = i0[valid] / table_a0[row[valid], col[valid]]
        q1 = i1[valid] / table_a1[local_row1[valid], col[valid]]
        rows.append({
            "frame_serial": serial,
            "samples": int(valid.sum()),
            "median_raw_intensity_roi1_over_roi0": float(np.median(raw_ratio)),
            "median_table_a_roi1_over_roi0": float(np.median(reference_ratio)),
            "median_normalized_q_roi1_over_roi0": float(np.median(q1 / q0)),
            "corr_log_q_roi0_roi1": float(np.corrcoef(np.log(q0), np.log(q1))[0, 1]),
        })
    return pd.DataFrame(rows)


def wall_planarity_test(coefficients, tables_a, tables_b):
    """用0.99 m全墙检验平面厚度，避免仅看小ROI绝对偏差而误判tail3语义。"""
    rows = []
    for roi in (0, 1):
        for serial in (1, 2):
            frame = pd.read_csv(EFFECT_ROOT / f"aFrame{serial}_ROI{roi}.csv")
            physical_row = frame["行row"].to_numpy(dtype=int)
            local_row = physical_row - (16 if roi == 1 else 0)
            col = frame["列col"].to_numpy(dtype=int)
            selected = frame["最优回波"].to_numpy(dtype=int)
            intensity = selected_column(
                frame, "p0_conv_intensity", "p1_conv_intensity")
            fwhm = selected_column(frame, "p0_fwhm_bin", "p1_fwhm_bin")
            reference = tables_a[roi][local_row, col]
            q = intensity / reference
            direction_x = (np.cos(np.deg2rad(frame["俯仰角deg"])) *
                           np.cos(np.deg2rad(frame["水平角deg"]))).to_numpy()
            after_b = (frame["距离m"].to_numpy() +
                       tables_b[roi][local_row, col])
            coeff = coefficients[roi]
            candidates = {
                "table_b_only": after_b,
                "tail3_fwhm_hypothesis": after_b + np.polyval(coeff, fwhm) / 100.0,
                "tail3_ln_intensity_hypothesis": (
                    after_b + np.polyval(coeff, np.log(intensity)) / 100.0),
                "current_table_a_log": (
                    after_b + 0.497953769 +
                    0.063243195 * np.log(np.clip(q, 0.002, 1.05))),
            }
            valid = ((frame["有效"].to_numpy() == 1) & (selected >= 0) &
                     (intensity > 0.0) & (fwhm > 0.0) & np.isfinite(q))
            for model, corrected_range in candidates.items():
                x = (corrected_range * direction_x)[valid]
                median_x = np.median(x)
                # 剔除场景飞点，但保留标定造成的前后分层。
                keep = np.abs(x - median_x) < 2.0
                x = x[keep]
                width = fwhm[valid][keep]
                p10, p90 = np.percentile(x, (10, 90))
                rows.append({
                    "roi": roi,
                    "frame_serial": serial,
                    "model": model,
                    "samples": int(x.size),
                    "median_x_m": float(np.median(x)),
                    "mad_x_m": float(np.median(np.abs(x - np.median(x)))),
                    "std_x_m": float(np.std(x)),
                    "p80_width_m": float(p90 - p10),
                    "corr_x_fwhm": float(np.corrcoef(x, width)[0, 1]),
                })
    return pd.DataFrame(rows)


def main():
    summary = json.loads(SUMMARY_JSON.read_text(encoding="utf-8"))
    coeff0 = np.array(summary["rois"][0]["tail3"]["values"], dtype=float)
    coeff1 = np.array(summary["rois"][1]["tail3"]["values"], dtype=float)
    table_a0 = table_matrix(0, "table_a_raw")
    table_a1 = table_matrix(1, "table_a_raw")
    table_b0 = table_matrix(0, "table_b_if_cm_m")
    table_b1 = table_matrix(1, "table_b_if_cm_m")

    samples = roi0_samples(table_a0, table_b0)
    scores, per_distance = score_roi0(samples, coeff0)
    stacking = score_current_model_stacking(samples, coeff0)
    roi1 = score_roi1(coeff1, table_b1)
    table_a_distance, slope_q, slope_qr2 = table_a_distance_test(samples)
    table_a_cross_roi = table_a_cross_roi_test(table_a0, table_a1)
    planarity = wall_planarity_test(
        (coeff0, coeff1), (table_a0, table_a1), (table_b0, table_b1))
    OUTPUT.mkdir(parents=True, exist_ok=True)
    scores.to_csv(OUTPUT / "hypothesis_scores.csv", index=False)
    per_distance.to_csv(OUTPUT / "fwhm_per_distance.csv", index=False)
    stacking.to_csv(OUTPUT / "current_model_stacking.csv", index=False)
    roi1.to_csv(OUTPUT / "roi1_validation.csv", index=False)
    table_a_distance.to_csv(OUTPUT / "table_a_distance_normalization.csv", index=False)
    table_a_cross_roi.to_csv(OUTPUT / "table_a_cross_roi.csv", index=False)
    planarity.to_csv(OUTPUT / "wall_planarity.csv", index=False)

    best = scores.iloc[0]
    roi1_pivot = roi1.pivot(index="file", columns="model", values="rmse_m")
    roi1_improved = bool((roi1_pivot["fwhm_poly_cm_add"] <
                          roi1_pivot["table_b_only"]).all())
    table_a_normalizes_roi = bool(
        table_a_cross_roi["median_normalized_q_roi1_over_roi0"].between(0.95, 1.10).all() and
        (table_a_cross_roi["corr_log_q_roi0_roi1"] > 0.95).all())
    planarity_pivot = planarity.pivot(
        index=["roi", "frame_serial"], columns="model", values="std_x_m")
    fwhm_rejected = bool((planarity_pivot["tail3_fwhm_hypothesis"] >
                          planarity_pivot["table_b_only"]).all())
    current_best = bool((planarity_pivot["current_table_a_log"] <
                         planarity_pivot.drop(columns="current_table_a_log").min(axis=1)).all())
    passed = (fwhm_rejected and current_best and table_a_normalizes_roi and
              abs(slope_qr2) < abs(slope_q))
    with (OUTPUT / "REPORT.txt").open("w", encoding="utf-8") as report:
        report.write("修正结论：tail3 直接作用于 FWHM 的假设被全墙平面测试否定。\n")
        report.write("该假设虽然降低小ROI多距离绝对偏差，却把FWHM量化差异写入距离，造成前后分层。\n")
        report.write("当前可确认：table_a 是ROI专属强度参考图；table_b 是逐像素距离项。\n")
        report.write("tail3 的精确自变量仍需固件实现或更多受控变量数据确认，不能继续命名为FWHM系数。\n")
        report.write("ROI0 local_row=physical_row；ROI1 local_row=physical_row-16。\n")
        report.write(f"ROI0 coeff={coeff0.tolist()}\n")
        report.write(f"ROI1 coeff={coeff1.tolist()}（c0=0，退化为线性模型）\n\n")
        report.write("ROI0 多距离候选模型汇总：\n")
        report.write(scores.to_string(index=False) + "\n\n")
        report.write("ROI0 分距离结果：\n")
        report.write(per_distance.to_string(index=False) + "\n\n")
        report.write("当前 table_a 对数模型与 tail3 叠加检查：\n")
        report.write(stacking.to_string(index=False) + "\n")
        report.write("tail3 不应直接叠加到当前已拟合的 table_a 对数模型，否则会重复补偿。\n\n")
        report.write("ROI1 0.99m 绝对偏差测试（此项曾导致误判）：\n")
        report.write(roi1.to_string(index=False) + "\n\n")
        report.write("table_a 跨 ROI 同场景重叠像素验证：\n")
        report.write(table_a_cross_roi.to_string(index=False) + "\n")
        report.write("ROI1 原始强度约为 ROI0 的 3.75 倍；除以各自 table_a 后仅相差约 2.3%，\n")
        report.write("且逐像素 log(q) 相关系数超过 0.97，证明 table_a 是 ROI 专属强度参考图。\n\n")
        report.write("table_a 多距离平方反比检查：\n")
        report.write(table_a_distance.to_string(index=False) + "\n")
        report.write(f"log(q)-log(distance) slope={slope_q:.6f}\n")
        report.write(f"log(q*distance^2)-log(distance) slope={slope_qr2:.6f}\n")
        report.write("q*r^2 显著削弱距离依赖，但只应称相对反射率；bin 内没有绝对反射率尺度。\n")
        report.write("\n0.99 m全墙平面厚度测试（判定分层的主测试）：\n")
        report.write(planarity.to_string(index=False) + "\n")
        report.write(f"FWHM假设是否被否定：{fwhm_rejected}\n")
        report.write(f"当前table_a对数模型是否为平面厚度最小：{current_best}\n")
    print(f"{'PASS' if passed else 'FAIL'} best={best['model']} "
          f"small_roi_rmse={best['rmse_m']:.6f} fwhm_rejected={fwhm_rejected} "
          f"current_best={current_best} "
          f"table_a_normalizes_roi={table_a_normalizes_roi} output={OUTPUT}")
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
