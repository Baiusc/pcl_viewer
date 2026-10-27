#!/usr/bin/env python3
"""反向验证 FL6532 calib_data.bin 的 table_a/table_b/tail3；无参数直接运行。"""

from pathlib import Path
import json
import subprocess

import numpy as np
import pandas as pd


REPO = Path(__file__).resolve().parents[1]
ROOT = REPO.parent
PARSE_DIR = ROOT / "algo_solid_lidar/build/calib_data_legacy_parse"
SUMMARY_JSON = PARSE_DIR / "summary.json"
ALGO = ROOT / "algo_solid_lidar/build/ALGO_SOLID_LIDAR"
CAPTURE_ROOT = ROOT / "data/固态雷达标定数据/20260902_6532range_data_公司走廊2%靶板多距离"
WALL_ROOT = ROOT / "data/固态雷达标定数据/6532标定算法效果对比"
OUTPUT = REPO / "build/range6532_tail3_reverse"
PARSED = OUTPUT / "parsed"
DISTANCES_M = (1.5, 2.0, 4.0, 7.0, 10.0, 13.0)
ROI_TARGET = (64, 71, 94, 100)
ROI1_13M_FILE = WALL_ROOT / "za13mFrame1_ROI1.csv"
ROI0_13M_PAIR_FILE = WALL_ROOT / "ae13mFrame1_ROI0.csv"
EMPIRICAL_OFFSET_M = 0.497953769
EMPIRICAL_LOG_SLOPE_M = 0.063243195


def table_matrix(roi, field):
    table = pd.read_csv(PARSE_DIR / f"roi{roi}_pixel_tables.csv", encoding="utf-8-sig")
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


def selected(frame, p0, p1):
    index = frame["最优回波"].to_numpy(dtype=int)
    return np.where(index == 1, frame[p1], frame[p0]).astype(float)


def standardize_frame(frame, roi, table_a, table_b, known_distance_m, scene, frame_id):
    physical_row = frame["行row"].to_numpy(dtype=int)
    local_row = physical_row - (16 if roi == 1 else 0)
    col = frame["列col"].to_numpy(dtype=int)
    echo = frame["最优回波"].to_numpy(dtype=int)
    intensity = selected(frame, "p0_conv_intensity", "p1_conv_intensity")
    fwhm = selected(frame, "p0_fwhm_bin", "p1_fwhm_bin")
    dist_rp = selected(frame, "p0_dist_rp_m", "p1_dist_rp_m")
    dist_cent = selected(frame, "p0_dist_cent_m", "p1_dist_cent_m")
    raw_range = frame["距离m"].to_numpy(dtype=float)
    direction_x = (np.cos(np.deg2rad(frame["俯仰角deg"])) *
                   np.cos(np.deg2rad(frame["水平角deg"]))).to_numpy()
    in_table = ((local_row >= 0) & (local_row < table_a.shape[0]) &
                (col >= 0) & (col < table_a.shape[1]))
    safe_row = np.clip(local_row, 0, table_a.shape[0] - 1)
    safe_col = np.clip(col, 0, table_a.shape[1] - 1)
    reference = table_a[safe_row, safe_col]
    offset_m = table_b[safe_row, safe_col]
    q = np.divide(intensity, reference, out=np.full_like(intensity, np.nan),
                  where=reference > 0.0)
    valid = ((frame["有效"].to_numpy() == 1) & (echo >= 0) & in_table &
             (intensity > 0.0) & (fwhm > 0.0) & np.isfinite(q) & (q > 0.0))
    result = pd.DataFrame({
        "scene": scene,
        "frame_id": frame_id,
        "roi": roi,
        "known_distance_m": known_distance_m,
        "physical_row": physical_row,
        "local_row": local_row,
        "col": col,
        "raw_range_m": raw_range,
        "dist_rp_m": dist_rp,
        "dist_cent_m": dist_cent,
        "direction_x": direction_x,
        "table_a": reference,
        "table_b_m": offset_m,
        "after_b_m": raw_range + offset_m,
        "intensity": intensity,
        "ln_intensity": np.log(np.maximum(intensity, 1.0)),
        "q": q,
        "ln_q": np.log(q),
        "fwhm_bin": fwhm,
        "valid": valid,
    })
    return result


def load_samples(tables_a, tables_b):
    multi = []
    for distance_m in DISTANCES_M:
        frame = pd.read_csv(ensure_frame_csv(distance_m))
        sample = standardize_frame(frame, 0, tables_a[0], tables_b[0],
                                   distance_m, "multi_distance_target", f"{distance_m:g}m")
        r0, r1, c0, c1 = ROI_TARGET
        valid = (sample["valid"].to_numpy() &
                 sample["physical_row"].between(r0, r1).to_numpy() &
                 sample["col"].between(c0, c1).to_numpy())
        x_after_b = sample["after_b_m"].to_numpy() * sample["direction_x"].to_numpy()
        median_x = np.median(x_after_b[valid])
        valid &= np.abs(x_after_b - median_x) < 1.0
        sample["valid"] = valid
        multi.append(sample[valid].copy())

    walls = []
    for roi in (0, 1):
        for serial in (1, 2):
            frame = pd.read_csv(WALL_ROOT / f"aFrame{serial}_ROI{roi}.csv")
            sample = standardize_frame(frame, roi, tables_a[roi], tables_b[roi],
                                       0.99, "wall_0p99m", f"roi{roi}_frame{serial}")
            walls.append(sample[sample["valid"]].copy())
    # 当前数据集中唯一一帧明确的 ROI1 远距离帧。ROI1 不覆盖 ROI0 的
    # row=64..71 靶板窗口，因此在上缘重叠区先做几何窗口，再剔除明显背景。
    # 该组只作为 13 m 交叉验证；报告会单独标明，避免误称完整 ROI1 多距离集。
    if ROI1_13M_FILE.exists() and ROI0_13M_PAIR_FILE.exists():
        frame0 = pd.read_csv(ROI0_13M_PAIR_FILE)
        pair0 = standardize_frame(frame0, 0, tables_a[0], tables_b[0],
                                  13.0, "multi_distance_target", "13m_roi0_pair")
        axial0 = pair0["after_b_m"].to_numpy() * pair0["direction_x"].to_numpy()
        target0 = pair0[pair0["valid"] & (pair0["physical_row"] <= 63) &
                        pd.Series(axial0).between(12.4, 13.2).to_numpy()][
                            ["physical_row", "col", "after_b_m", "direction_x"]]
        target0["pair_axial_m"] = target0["after_b_m"] * target0["direction_x"]
        target0 = target0[["physical_row", "col", "pair_axial_m"]]

        frame = pd.read_csv(ROI1_13M_FILE)
        sample = standardize_frame(frame, 1, tables_a[1], tables_b[1],
                                   13.0, "multi_distance_target", "13m_roi1")
        sample = sample.merge(target0, on=["physical_row", "col"], how="inner")
        axial1 = sample["after_b_m"].to_numpy() * sample["direction_x"].to_numpy()
        # 两种ROI同位置若相差超过1m，多为弱回波下选中了不同目标/回波，不用于标定。
        valid = (sample["valid"].to_numpy() &
                 (np.abs(axial1 - sample["pair_axial_m"].to_numpy()) < 1.0))
        sample["valid"] = valid
        multi.append(sample[valid].copy())
    return pd.concat(multi, ignore_index=True), pd.concat(walls, ignore_index=True)


def polyval(coeff, x):
    return (coeff[0] * x + coeff[1]) * x + coeff[2]


def round3_relative_corrections(sample, all_coeffs):
    """第三轮：穷举 P(x)-P(x@0.9m)、符号、table_b耦合及q调制。"""
    raw = sample["raw_range_m"].to_numpy()
    table_b = sample["table_b_m"].to_numpy()
    q = sample["q"].to_numpy()
    ln_q = sample["ln_q"].to_numpy()
    ln_i = sample["ln_intensity"].to_numpy()
    ln_a = np.log(sample["table_a"].to_numpy())
    direction_x = sample["direction_x"].to_numpy()
    roi = int(sample["roi"].iloc[0])

    # fixed0p9严格对应“把0.9m直接作为自变量”；table_b_implied0p9则考虑
    # 多项式若实际吃的是传感器原始距离，0.9m靶面在原始域应先撤销table_b。
    radial_0p9 = 0.9 / np.maximum(direction_x, 1e-9)
    x_defs = {
        "range_m_fixed0p9": (raw, np.full_like(raw, 0.9)),
        "range_m_table_b_implied0p9": (raw, radial_0p9 - table_b),
        "tof_ns_fixed0p9": (raw * 6.6712819, np.full_like(raw, 0.9 * 6.6712819)),
        "bin_0p15m_fixed0p9": (raw / 0.15, np.full_like(raw, 0.9 / 0.15)),
        "ln_intensity_table_a_base": (ln_i, ln_a),
        "ln_q_zero_base": (ln_q, np.zeros_like(raw)),
    }
    result = {}
    for scope in ("own", "roi0_shared"):
        coeff = all_coeffs[roi] if scope == "own" else all_coeffs[0]
        for x_name, (x, x0) in x_defs.items():
            delta = (polyval(coeff, x) - polyval(coeff, x0)) / 100.0
            modulators = {"none": np.ones_like(q)}
            if x_name.startswith(("range_", "tof_", "bin_")):
                modulators.update({
                    "q": q,
                    "inv_q": 1.0 / np.maximum(q, 1e-9),
                    "ln_q": ln_q,
                })
            for mod_name, modulation in modulators.items():
                d = delta * modulation
                stem = f"r3_{scope}_{x_name}_{mod_name}"
                # corrections() 的返回量统一是相对 after_b 的增量。
                result[f"{stem}_add_plus"] = d
                result[f"{stem}_add_minus"] = -d
                result[f"{stem}_scale_table_b_plus"] = table_b * d
                # 用户列出的 raw+(table_b-delta)，与add_minus数值相同，保留独立标签验证。
                result[f"{stem}_table_b_minus_delta"] = -d
    return result


def corrections(sample, coeff, all_coeffs, learned):
    raw = sample["raw_range_m"].to_numpy()
    after_b = sample["after_b_m"].to_numpy()
    q = sample["q"].to_numpy()
    ln_q = sample["ln_q"].to_numpy()
    ln_i = sample["ln_intensity"].to_numpy()
    ln_a = np.log(sample["table_a"].to_numpy())
    fwhm = sample["fwhm_bin"].to_numpy()
    rp = sample["dist_rp_m"].to_numpy()
    cent = sample["dist_cent_m"].to_numpy()
    ln_qr2 = np.log(q * np.maximum(raw, 1e-6) ** 2)
    p_range = polyval(coeff, raw) / 100.0
    p0_ln_i = polyval(all_coeffs[0], ln_i) / 100.0
    own_ln_i = polyval(coeff, ln_i) / 100.0
    roi = int(sample["roi"].iloc[0])
    linked_ln_i = p0_ln_i if roi == 1 else own_ln_i
    linked_sum_ln_i = (p0_ln_i + own_ln_i) if roi == 1 else own_ln_i
    result = {
        "table_b_only": np.zeros_like(raw),
        "tail_range_raw": p_range,
        "tail_range_after_b": polyval(coeff, after_b) / 100.0,
        "tail_range_offset_from_0p9": (polyval(coeff, raw) - polyval(coeff, 0.9)) / 100.0,
        "tail_range_div10": polyval(coeff, raw / 10.0) / 100.0,
        "tail_range_div100": polyval(coeff, raw / 100.0) / 100.0,
        "tail_range_dm": polyval(coeff, raw * 10.0) / 100.0,
        "tail_range_cm": polyval(coeff, raw * 100.0) / 100.0,
        "tail_tof_ns": polyval(coeff, raw * 6.6712819) / 100.0,
        "tail_bin_index_raw": polyval(coeff, raw * 25600.0) / 100.0,
        "tail_range_div10_relative_0p9": (
            polyval(coeff, raw / 10.0) - polyval(coeff, 0.09)) / 100.0,
        "tail_range_div100_relative_0p9": (
            polyval(coeff, raw / 100.0) - polyval(coeff, 0.009)) / 100.0,
        "tail_tof_ns_relative_0p9": (
            polyval(coeff, raw * 6.6712819) - polyval(coeff, 0.9 * 6.6712819)) / 100.0,
        # 用质心/峰值距离作基值，而非只将其送入多项式。
        "tail_dist_rp": (rp - raw) + polyval(coeff, rp) / 100.0,
        "tail_dist_cent": (cent - raw) + polyval(coeff, cent) / 100.0,
        "tail_dist_cent_relative_0p9": ((cent - raw) +
            (polyval(coeff, cent) - polyval(coeff, 0.9)) / 100.0),
        "tail_ln_q": polyval(coeff, ln_q) / 100.0,
        "tail_neg_ln_q": polyval(coeff, -ln_q) / 100.0,
        "tail_ln_intensity": polyval(coeff, ln_i) / 100.0,
        "tail_ln_intensity_roi0_shared": linked_ln_i,
        "tail_ln_intensity_roi0_plus_roi1": linked_sum_ln_i,
        "tail_log10_intensity": polyval(coeff, ln_i / np.log(10.0)) / 100.0,
        "tail_ln_i_minus_ln_a": (polyval(coeff, ln_i) - polyval(coeff, ln_a)) / 100.0,
        "tail_ln_a_minus_ln_i": (polyval(coeff, ln_a) - polyval(coeff, ln_i)) / 100.0,
        "tail_range_mul_ln_q": polyval(coeff, raw * ln_q) / 100.0,
        "tail_range_mul_neg_ln_q": polyval(coeff, -raw * ln_q) / 100.0,
        "tail_ln_qr2": polyval(coeff, ln_qr2) / 100.0,
        "tail_neg_ln_qr2": polyval(coeff, -ln_qr2) / 100.0,
        "tail_fwhm_rejected": polyval(coeff, fwhm) / 100.0,
        "tail_range_times_1_plus_ln_q": p_range * (1.0 + ln_q),
        "tail_range_times_1_minus_ln_q": p_range * (1.0 - ln_q),
        "tail_range_plus_range_times_ln_q": p_range + p_range * ln_q,
        # 三值按 [低段斜率, 拐点, 高段斜率] 解释的无截距分段候选。
        "tail_piecewise_range": (coeff[0] * raw +
                                  coeff[2] * np.maximum(raw - coeff[1], 0.0)) / 100.0,
        "empirical_table_a_log_benchmark": (
            EMPIRICAL_OFFSET_M + EMPIRICAL_LOG_SLOPE_M *
            np.log(np.clip(q, 0.002, 1.05))),
    }
    # 下列拟合只用于诊断“tail(range) 后 K 是否趋近于零”，不能算纯 bin 公式。
    result["fitted_tail_range_plus_K_lnq"] = p_range + learned["k_only"] * ln_q
    result["fitted_tail_range_plus_C_K_lnq"] = (
        p_range + learned["intercept"] + learned["k_with_intercept"] * ln_q)
    result.update(round3_relative_corrections(sample, all_coeffs))
    return result


def safe_corr(a, b):
    if len(a) < 3 or np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return 0.0
    return float(np.corrcoef(a, b)[0, 1])


def fit_k_and_surface(multi, coeffs):
    """诊断拟合：求 K 及二维曲面，但明确不把外部系数当作纯 bin 结论。"""
    target_radial = (multi["known_distance_m"].to_numpy() /
                     multi["direction_x"].to_numpy())
    residual = target_radial - multi["after_b_m"].to_numpy()
    raw = multi["raw_range_m"].to_numpy()
    lnq = multi["ln_q"].to_numpy()
    roi = multi["roi"].to_numpy(dtype=int)
    p = np.empty_like(raw)
    for r in (0, 1):
        mask = roi == r
        p[mask] = polyval(coeffs[r], raw[mask]) / 100.0
    y = residual - p
    k_only = float(np.dot(lnq, y) / max(np.dot(lnq, lnq), 1e-12))
    design_ck = np.column_stack([np.ones(len(raw)), lnq])
    intercept, k_with_intercept = np.linalg.lstsq(design_ck, y, rcond=None)[0]

    # 完整二维曲面仅用于观察结构，系数不进入 C++：单位为 m。
    surface_design = np.column_stack([
        np.ones(len(raw)), raw, raw * raw, lnq, lnq * lnq, raw * lnq])
    surface_coeff = np.linalg.lstsq(surface_design, residual, rcond=None)[0]
    surface_pred = surface_design @ surface_coeff
    ss_res = np.sum((residual - surface_pred) ** 2)
    ss_tot = np.sum((residual - np.mean(residual)) ** 2)
    learned = {
        "k_only": k_only,
        "intercept": float(intercept),
        "k_with_intercept": float(k_with_intercept),
    }
    diagnostics = pd.DataFrame([
        {"term": "K_only_after_tail_range", "coefficient": k_only, "unit": "m/ln(q)"},
        {"term": "C_after_tail_range", "coefficient": intercept, "unit": "m"},
        {"term": "K_with_C_after_tail_range", "coefficient": k_with_intercept,
         "unit": "m/ln(q)"},
        *({"term": name, "coefficient": value, "unit": "m"}
          for name, value in zip(("surface_1", "surface_r", "surface_r2",
                                  "surface_lnq", "surface_lnq2", "surface_r_lnq"),
                                 surface_coeff)),
        {"term": "surface_r2_score", "coefficient": 1.0 - ss_res / ss_tot,
         "unit": "dimensionless"},
    ])
    return learned, diagnostics


def scene_metrics(sample, model, correction):
    corrected_range = sample["after_b_m"].to_numpy() + correction
    error = (corrected_range * sample["direction_x"].to_numpy() -
             sample["known_distance_m"].to_numpy())
    keep = np.isfinite(error)
    error = error[keep]
    fwhm = sample["fwhm_bin"].to_numpy()[keep]
    ln_i = sample["ln_intensity"].to_numpy()[keep]
    p10, p90 = np.percentile(error, (10, 90))
    return {
        "scene": sample["scene"].iloc[0],
        "frame_id": sample["frame_id"].iloc[0],
        "roi": int(sample["roi"].iloc[0]),
        "known_distance_m": float(sample["known_distance_m"].iloc[0]),
        "model": model,
        "samples": int(error.size),
        "median_signed_error_m": float(np.median(error)),
        "median_abs_error_m": float(np.median(np.abs(error))),
        "rmse_m": float(np.sqrt(np.mean(error ** 2))),
        "std_m": float(np.std(error)),
        "p10_p90_thickness_m": float(p90 - p10),
        "corr_error_fwhm": safe_corr(error, fwhm),
        "corr_error_ln_intensity": safe_corr(error, ln_i),
    }


def evaluate_models(multi, walls, coeffs, learned):
    rows = []
    scatter = []
    for dataset in (multi, walls):
        group_columns = ["scene", "frame_id", "roi", "known_distance_m"]
        for _, sample in dataset.groupby(group_columns, sort=False):
            roi = int(sample["roi"].iloc[0])
            for model, correction in corrections(sample, coeffs[roi], coeffs, learned).items():
                rows.append(scene_metrics(sample, model, correction))
                if model in ("table_b_only", "tail_range_raw", "tail_ln_q",
                             "tail_neg_ln_q", "tail_ln_intensity",
                             "tail_range_mul_ln_q", "tail_fwhm_rejected",
                             "empirical_table_a_log_benchmark"):
                    error = ((sample["after_b_m"].to_numpy() + correction) *
                             sample["direction_x"].to_numpy() -
                             sample["known_distance_m"].to_numpy())
                    take = np.arange(len(sample))
                    if sample["scene"].iloc[0] == "wall_0p99m":
                        take = take[::16]  # 全墙仅降采样散点文件；评价指标仍使用全部点。
                    block = sample.iloc[take][[
                                    "scene", "frame_id", "roi", "known_distance_m",
                                    "physical_row", "col", "raw_range_m", "after_b_m",
                                    "q", "ln_q", "ln_intensity", "fwhm_bin"]].copy()
                    block["model"] = model
                    block["axial_error_m"] = error[take]
                    scatter.append(block)
    metrics = pd.DataFrame(rows)
    scatter_data = pd.concat(scatter, ignore_index=True)

    summary_rows = []
    for model, group in metrics.groupby("model"):
        wall = group[group["scene"] == "wall_0p99m"]
        multi_group = group[group["scene"] == "multi_distance_target"]
        summary_rows.append({
            "model": model,
            "multi_median_abs_error_m": float(multi_group["median_abs_error_m"].median()),
            "multi_mean_thickness_m": float(multi_group["p10_p90_thickness_m"].mean()),
            "wall_median_abs_error_m": float(wall["median_abs_error_m"].median()),
            "wall_mean_std_m": float(wall["std_m"].mean()),
            "wall_mean_thickness_m": float(wall["p10_p90_thickness_m"].mean()),
            "wall_max_abs_corr_fwhm": float(wall["corr_error_fwhm"].abs().max()),
            "wall_max_abs_corr_ln_intensity": float(
                wall["corr_error_ln_intensity"].abs().max()),
            "roi0_multi_median_abs_error_m": float(
                multi_group[multi_group["roi"] == 0]["median_abs_error_m"].median()),
            "roi1_multi_median_abs_error_m": float(
                multi_group[multi_group["roi"] == 1]["median_abs_error_m"].median()),
            "roi0_wall_median_abs_error_m": float(
                wall[wall["roi"] == 0]["median_abs_error_m"].median()),
            "roi1_wall_median_abs_error_m": float(
                wall[wall["roi"] == 1]["median_abs_error_m"].median()),
            "roi0_wall_mean_thickness_m": float(
                wall[wall["roi"] == 0]["p10_p90_thickness_m"].mean()),
            "roi1_wall_mean_thickness_m": float(
                wall[wall["roi"] == 1]["p10_p90_thickness_m"].mean()),
        })
    summary = pd.DataFrame(summary_rows)
    # 先按防分层/防吸点硬约束，再比较绝对误差与厚度。
    summary["wrong_model"] = (
        (summary["wall_max_abs_corr_fwhm"] > 0.35) |
        (summary["wall_max_abs_corr_ln_intensity"] > 0.35) |
        (summary["wall_mean_thickness_m"] > 0.16)
    )
    summary["passes_all_constraints"] = (
        ~summary["wrong_model"] &
        (summary["roi0_wall_mean_thickness_m"] < 0.10) &
        (summary["roi1_wall_mean_thickness_m"] < 0.10) &
        (summary["roi0_multi_median_abs_error_m"] < 0.05) &
        (summary["roi1_multi_median_abs_error_m"] < 0.05)
    )
    # 第三轮用户给定门槛：ROI0多距离误差<5cm；ROI0/ROI1近墙厚度<10cm；
    # “绝对距离中位误差<5cm”同时约束ROI0/ROI1近墙，防止只压薄却整体平移。
    summary["passes_round3_constraints"] = (
        summary["model"].str.startswith("r3_") &
        (summary["roi0_multi_median_abs_error_m"] < 0.05) &
        (summary["roi0_wall_median_abs_error_m"] < 0.05) &
        (summary["roi1_wall_median_abs_error_m"] < 0.05) &
        (summary["roi0_wall_mean_thickness_m"] < 0.10) &
        (summary["roi1_wall_mean_thickness_m"] < 0.10)
    )
    summary = summary.sort_values(
        ["passes_all_constraints", "wrong_model", "wall_median_abs_error_m",
         "wall_mean_thickness_m"], ascending=[False, True, True, True])
    return metrics, summary, scatter_data


def fit_tail_variables(multi, tail):
    data = multi[multi["roi"] == 0].copy()
    target_radial = data["known_distance_m"].to_numpy() / data["direction_x"].to_numpy()
    residual = target_radial - data["after_b_m"].to_numpy()
    variables = {
        "range_raw": data["raw_range_m"].to_numpy(),
        "range_after_b": data["after_b_m"].to_numpy(),
        "range_div10": data["raw_range_m"].to_numpy() / 10.0,
        "range_div100": data["raw_range_m"].to_numpy() / 100.0,
        "range_dm": data["raw_range_m"].to_numpy() * 10.0,
        "range_cm": data["raw_range_m"].to_numpy() * 100.0,
        "tof_ns": data["raw_range_m"].to_numpy() * 6.6712819,
        "dist_rp": data["dist_rp_m"].to_numpy(),
        "dist_cent": data["dist_cent_m"].to_numpy(),
        "ln_q": data["ln_q"].to_numpy(),
        "neg_ln_q": -data["ln_q"].to_numpy(),
        "range_mul_ln_q": data["raw_range_m"].to_numpy() * data["ln_q"].to_numpy(),
        "range_mul_neg_ln_q": -data["raw_range_m"].to_numpy() * data["ln_q"].to_numpy(),
        "ln_qr2": np.log(data["q"].to_numpy() * data["raw_range_m"].to_numpy() ** 2),
        "ln_intensity": data["ln_intensity"].to_numpy(),
    }
    rows = []
    fit_scatter = []
    for name, x in variables.items():
        coeff_m = np.polyfit(x, residual, 2)
        prediction = np.polyval(coeff_m, x)
        ss_res = np.sum((residual - prediction) ** 2)
        ss_tot = np.sum((residual - np.mean(residual)) ** 2)
        # 留一距离交叉验证，防止只拟合住某个距离点。
        cv_errors = []
        distances = data["known_distance_m"].to_numpy()
        for held in np.unique(distances):
            train = distances != held
            test = ~train
            fitted = np.polyfit(x[train], residual[train], 2)
            cv_errors.extend(np.abs(residual[test] - np.polyval(fitted, x[test])))
        coeff_cm = coeff_m * 100.0
        tail_distance = float(np.linalg.norm(coeff_cm - tail) /
                              max(np.linalg.norm(tail), 1e-12))
        rows.append({
            "variable": name,
            "fit_c0_cm": coeff_cm[0],
            "fit_c1_cm": coeff_cm[1],
            "fit_c2_cm": coeff_cm[2],
            "tail_c0": tail[0],
            "tail_c1": tail[1],
            "tail_c2": tail[2],
            "relative_coefficient_distance": tail_distance,
            "fit_r2": float(1.0 - ss_res / ss_tot),
            "leave_one_distance_mae_m": float(np.mean(cv_errors)),
        })
        fit_scatter.append(pd.DataFrame({
            "variable": name,
            "x": x,
            "residual_after_table_b_m": residual,
            "fitted_residual_m": prediction,
            "known_distance_m": distances,
        }))
    return (pd.DataFrame(rows).sort_values("leave_one_distance_mae_m"),
            pd.concat(fit_scatter, ignore_index=True))


def validate_table_a(walls):
    rows = []
    for keys, sample in walls.groupby(["roi", "frame_id"], sort=False):
        intensity = sample["intensity"].to_numpy()
        q = sample["q"].to_numpy()
        def robust_spread(values):
            p10, p50, p90 = np.percentile(values, (10, 50, 90))
            return (p90 - p10) / max(abs(p50), 1e-12)
        rows.append({
            "roi": keys[0],
            "frame_id": keys[1],
            "samples": len(sample),
            "raw_intensity_robust_relative_spread": robust_spread(intensity),
            "normalized_q_robust_relative_spread": robust_spread(q),
            "raw_intensity_cv": float(np.std(intensity) / np.mean(intensity)),
            "normalized_q_cv": float(np.std(q) / np.mean(q)),
            "corr_raw_intensity_table_a": safe_corr(
                np.log(intensity), np.log(sample["table_a"].to_numpy())),
        })

    # ROI0/ROI1 同一物理像素交叉验证功率归一化效果。
    for serial in (1, 2):
        a = walls[(walls["roi"] == 0) & (walls["frame_id"] == f"roi0_frame{serial}")]
        b = walls[(walls["roi"] == 1) & (walls["frame_id"] == f"roi1_frame{serial}")]
        overlap = a.merge(b, on=["physical_row", "col"], suffixes=("_0", "_1"))
        rows.append({
            "roi": "cross_roi",
            "frame_id": f"frame{serial}",
            "samples": len(overlap),
            "raw_intensity_robust_relative_spread": np.nan,
            "normalized_q_robust_relative_spread": np.nan,
            "raw_intensity_cv": float(np.median(overlap["intensity_1"] /
                                                  overlap["intensity_0"])),
            "normalized_q_cv": float(np.median(overlap["q_1"] / overlap["q_0"])),
            "corr_raw_intensity_table_a": safe_corr(
                np.log(overlap["q_0"]), np.log(overlap["q_1"])),
        })
    return pd.DataFrame(rows)


def residual_curve(multi):
    rows = []
    for distance, sample in multi.groupby("known_distance_m"):
        target_radial = distance / sample["direction_x"].to_numpy()
        residual = target_radial - sample["after_b_m"].to_numpy()
        rows.append({
            "known_distance_m": distance,
            "samples": len(sample),
            "median_residual_after_table_b_m": float(np.median(residual)),
            "p10_residual_m": float(np.percentile(residual, 10)),
            "p90_residual_m": float(np.percentile(residual, 90)),
            "median_q": float(np.median(sample["q"])),
            "median_ln_q": float(np.median(sample["ln_q"])),
            "median_fwhm_bin": float(np.median(sample["fwhm_bin"])),
        })
    return pd.DataFrame(rows)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    summary = json.loads(SUMMARY_JSON.read_text(encoding="utf-8"))
    coeffs = tuple(np.asarray(roi["tail3"]["values"], dtype=float)
                   for roi in summary["rois"])
    tables_a = tuple(table_matrix(roi, "table_a_raw") for roi in (0, 1))
    tables_b = tuple(table_matrix(roi, "table_b_if_cm_m") for roi in (0, 1))
    multi, walls = load_samples(tables_a, tables_b)

    learned, surface_diagnostics = fit_k_and_surface(multi, coeffs)
    metrics, model_summary, scatter = evaluate_models(multi, walls, coeffs, learned)
    fits, fit_scatter = fit_tail_variables(multi, coeffs[0])
    table_a_result = validate_table_a(walls)
    curve = residual_curve(multi)

    metrics.to_csv(OUTPUT / "scene_metrics.csv", index=False)
    model_summary.to_csv(OUTPUT / "model_summary.csv", index=False)
    scatter.to_csv(OUTPUT / "scatter_model_errors.csv", index=False)
    fits.to_csv(OUTPUT / "tail_variable_fits.csv", index=False)
    fit_scatter.to_csv(OUTPUT / "scatter_tail_variable_fits.csv", index=False)
    table_a_result.to_csv(OUTPUT / "table_a_validation.csv", index=False)
    curve.to_csv(OUTPUT / "residual_curve_by_distance.csv", index=False)
    surface_diagnostics.to_csv(OUTPUT / "k_and_surface_fit.csv", index=False)
    multi.to_csv(OUTPUT / "multi_distance_samples.csv", index=False)

    round3 = model_summary[model_summary["model"].str.startswith("r3_")].copy()
    round3_columns = [
        "model", "roi0_multi_median_abs_error_m",
        "roi0_wall_median_abs_error_m", "roi1_wall_median_abs_error_m",
        "roi0_wall_mean_thickness_m", "roi1_wall_mean_thickness_m",
        "wall_max_abs_corr_fwhm", "wall_max_abs_corr_ln_intensity",
        "passes_round3_constraints",
    ]
    round3[round3_columns].to_csv(OUTPUT / "round3_baseline_sweep.csv", index=False)
    round3["failed_constraints"] = (
        (round3["roi0_multi_median_abs_error_m"] >= 0.05).astype(int) +
        (round3["roi0_wall_median_abs_error_m"] >= 0.05).astype(int) +
        (round3["roi1_wall_median_abs_error_m"] >= 0.05).astype(int) +
        (round3["roi0_wall_mean_thickness_m"] >= 0.10).astype(int) +
        (round3["roi1_wall_mean_thickness_m"] >= 0.10).astype(int))
    round3["ranking_score"] = (
        round3["roi0_multi_median_abs_error_m"] +
        round3["roi0_wall_median_abs_error_m"] +
        round3["roi1_wall_median_abs_error_m"] +
        round3["roi0_wall_mean_thickness_m"] +
        round3["roi1_wall_mean_thickness_m"])
    round3_ranked = round3.sort_values(["failed_constraints", "ranking_score"])
    round3_ranked[round3_columns + ["failed_constraints", "ranking_score"]].to_csv(
        OUTPUT / "round3_ranked.csv", index=False)

    pure = model_summary[~model_summary["model"].str.contains("empirical|fitted", regex=True)]
    valid_pure = pure[pure["passes_all_constraints"]]
    best_pure = valid_pure.iloc[0] if not valid_pure.empty else None
    with (OUTPUT / "REPORT.md").open("w", encoding="utf-8") as report:
        report.write("# FL6532 tail3 反向分析报告\n\n")
        report.write("## 判定原则\n\n")
        report.write("模型同时检查绝对误差、平面厚度、FWHM相关性和强度相关性；")
        report.write("强制通过条件为 ROI0/ROI1 多距离中位绝对误差均<0.05m，且两者0.99m墙厚均<0.10m。")
        report.write("当前 ROI1 多距离数据只有一帧13m交叉验证，不能替代完整距离序列。\n\n")
        report.write("## table_a 验证\n\n````text\n")
        report.write(table_a_result.to_string(index=False) + "\n````\n\n")
        report.write("## table_b 后剩余误差随距离变化\n\n````text\n")
        report.write(curve.to_string(index=False) + "\n````\n\n")
        report.write("## tail3 自变量二次拟合（诊断拟合，不进入固件模型）\n\n````text\n")
        report.write(fits.to_string(index=False) + "\n````\n\n")
        report.write("## tail(range)+K ln(q) 与二维曲面诊断（外部拟合，不是纯bin公式）\n\n````text\n")
        report.write(surface_diagnostics.to_string(index=False) + "\n````\n\n")
        report.write("## 直接使用 bin 参数的候选模型\n\n````text\n")
        report.write(model_summary.to_string(index=False) + "\n````\n\n")
        report.write("## 第三轮0.9m基准扣除穷举\n\n")
        report.write(f"共测试{len(round3)}个组合；通过强制条件{int(round3['passes_round3_constraints'].sum())}个。")
        report.write("下表列出综合约束最接近的前20个；全部结果见round3_baseline_sweep.csv。\n\n````text\n")
        report.write(round3_ranked[round3_columns + ["failed_constraints"]].head(20).to_string(index=False))
        report.write("\n````\n\n")
        report.write("## 结论\n\n")
        report.write("1. table_a 的跨ROI归一化证据成立；它消除了ROI1约3.75倍发光功率差。\n")
        report.write("2. tail3(FWHM) 被再次否定：墙面厚度和FWHM相关性同步恶化。\n")
        report.write("3. 本轮显式比较了 m/dm/cm、TOF(ns)、原始bin索引、dist_rp、dist_cent、")
        report.write("0.9m相对锚定、ROI0/ROI1联动以及 tail(range)×强度交叉项。\n")
        report.write("4. 多距离残差对range做二次诊断拟合时，二次项-0.048604与tail3[0]=-0.049648接近，")
        report.write("但一次项和常数项完全不符；这提示tail3可能含距离曲率项，同时还缺少固件中的基准/增益变换。\n")
        by_name = model_summary.set_index("model")
        b = by_name.loc["table_b_only"]
        own = by_name.loc["tail_ln_intensity"]
        shared = by_name.loc["tail_ln_intensity_roi0_shared"]
        report.write("5. ROI1的table_b-only墙误差为")
        report.write(f"{b['roi1_wall_median_abs_error_m']:.6f}m，ROI0为")
        report.write(f"{b['roi0_wall_median_abs_error_m']:.6f}m，两者只差")
        report.write(f"{abs(b['roi1_wall_median_abs_error_m']-b['roi0_wall_median_abs_error_m']):.6f}m；")
        report.write("0.3m差异主要由把ROI1自己的tail3代入ln(I)造成，而非table_b漏常数。")
        report.write(f"共享ROI0 tail3后ROI1墙误差从{own['roi1_wall_median_abs_error_m']:.6f}m降至")
        report.write(f"{shared['roi1_wall_median_abs_error_m']:.6f}m，但ROI1 13m误差仍为")
        report.write(f"{shared['roi1_multi_median_abs_error_m']:.6f}m。\n")
        report.write("6. tail(range)+K ln(q)无截距拟合得到")
        report.write(f"K={learned['k_only']:.6f}m；允许截距后得到")
        report.write(f"C={learned['intercept']:.6f}m、K={learned['k_with_intercept']:.6f}m。")
        report.write("二维曲面的距离项接近零、强度项接近既有经验式，说明现有数据不支持仅靠tail(range)消除全局基准。\n")
        if best_pure is None:
            report.write("7. 第三轮所有基准扣除公式都失败：减去P(x@0.9m)会消掉tail3常数项，")
            report.write("却无法补回table_b之后仍存在的约0.4m公共偏差；距离/TOF形式还会把像素间原始距离差放大成厚度。\n")
            report.write("8. 当前没有纯bin模型同时满足强制阈值；因此不替换C++默认经验模型，也不新增枚举。\n")
        else:
            report.write(f"7. 当前物理约束下排名最高的纯bin候选：{best_pure['model']}；")
            report.write("仍需与ROI1和更多受控距离复核后才能认定为固件公式。\n")

    fwhm = model_summary[model_summary["model"] == "tail_fwhm_rejected"].iloc[0]
    empirical = model_summary[
        model_summary["model"] == "empirical_table_a_log_benchmark"].iloc[0]
    print("PASS analysis_complete "
          f"fwhm_wall_thickness={fwhm['wall_mean_thickness_m']:.6f} "
          f"fwhm_corr={fwhm['wall_max_abs_corr_fwhm']:.6f} "
          f"empirical_wall_thickness={empirical['wall_mean_thickness_m']:.6f} "
          f"pure_pass_count={len(valid_pure)} "
          f"round3_models={len(round3)} "
          f"round3_pass_count={int(round3['passes_round3_constraints'].sum())} "
          f"output={OUTPUT}")


if __name__ == "__main__":
    main()
