#!/usr/bin/env python3
"""6532 Tail3LnFixed 的逐像素 x 偏置验证；无命令行参数。"""
from pathlib import Path
import json
import sys
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import analyze_tail3_reverse as common

OUTPUT = common.REPO / "build/range6532_tail3_pixel_refine"
C = 12.9


def poly(c, x):
    return (c[0] * x + c[1]) * x + c[2]


def nearest_root(c, target_cm, x0):
    a, b, d = [float(v) for v in c]
    # a*x^2+b*x+d=target_cm，选择最接近观测 x0 的实根。
    aa, bb, cc = a, b, d - target_cm
    if abs(aa) < 1e-10:
        return x0 if abs(bb) < 1e-10 else -cc / bb
    disc = bb * bb - 4.0 * aa * cc
    if disc < 0.0:
        return x0
    roots = [(-bb + np.sqrt(disc)) / (2.0 * aa),
             (-bb - np.sqrt(disc)) / (2.0 * aa)]
    return min(roots, key=lambda r: abs(r - x0))


def make_pixel_bias(walls, coeffs, mode):
    global_targets = {}
    for roi, s in walls.groupby("roi", sort=False):
        c = coeffs[int(roi)]
        global_targets[int(roi)] = 0.0 if mode == "zero" else float(np.median(poly(c, np.log(s.q.to_numpy()) + C)))
    rows = []
    for _, s in walls.groupby(["roi", "physical_row", "col"], sort=False):
        roi = int(s.roi.iloc[0]); c = coeffs[roi]
        q = s.q.to_numpy(); x = np.log(q) + C
        if mode == "observed_plane":
            current = poly(c, x) / 100.0
            direction = s.direction_x.to_numpy()
            error = (s.after_b_m.to_numpy() + current) * direction - s.known_distance_m.to_numpy()
            desired_cm = (current - error / direction) * 100.0
        else:
            desired_cm = np.full_like(x, global_targets[roi])
        roots = np.array([nearest_root(c, target, xx) for target, xx in zip(desired_cm, x)])
        delta = roots - x
        rows.append({"roi": roi, "physical_row": int(s.physical_row.iloc[0]),
                     "col": int(s.col.iloc[0]), "delta_pixel": float(np.median(delta)),
                     "mode": mode,
                     "delta_pixel_p10": float(np.percentile(delta, 10)),
                     "delta_pixel_p90": float(np.percentile(delta, 90)),
                     "samples": len(s)})
    return pd.DataFrame(rows)


def evaluate(dataset, coeffs, pixel_bias, scope):
    rows = []
    for _, s in dataset.groupby(["scene", "frame_id", "roi", "known_distance_m"], sort=False):
        roi = int(s.roi.iloc[0]); c = coeffs[0] if scope == "roi0_shared" else coeffs[roi]
        key = pd.MultiIndex.from_arrays([s.roi, s.physical_row, s.col])
        lookup = pixel_bias[pixel_bias["mode"] == scope].set_index(["roi", "physical_row", "col"]).delta_pixel
        delta = lookup.reindex(key).fillna(0.0).to_numpy()
        x = np.log(s.q.to_numpy()) + C + delta
        correction = poly(c, x) / 100.0
        rows.append(common.scene_metrics(s, f"Tail3LnPixelFixed_{scope}", correction))
    detail = pd.DataFrame(rows)
    summary = []
    for name, g in detail.groupby("model"):
        m = g[g.scene == "multi_distance_target"]; w = g[g.scene == "wall_0p99m"]
        summary.append({"model": name,
                        "roi0_multi_median_abs_error_m": float(m[m.roi == 0].median_abs_error_m.median()),
                        "roi1_multi_median_abs_error_m": float(m[m.roi == 1].median_abs_error_m.median()) if (m.roi == 1).any() else np.nan,
                        "roi0_wall_median_abs_error_m": float(w[w.roi == 0].median_abs_error_m.median()),
                        "roi1_wall_median_abs_error_m": float(w[w.roi == 1].median_abs_error_m.median()),
                        "roi0_wall_thickness_m": float(w[w.roi == 0].p10_p90_thickness_m.mean()),
                        "roi1_wall_thickness_m": float(w[w.roi == 1].p10_p90_thickness_m.mean()),
                        "max_abs_corr_fwhm": float(w.corr_error_fwhm.abs().max()),
                        "max_abs_corr_ln_intensity": float(w.corr_error_ln_intensity.abs().max())})
    return detail, pd.DataFrame(summary)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    parsed = json.loads(common.SUMMARY_JSON.read_text(encoding="utf-8"))
    coeffs = tuple(np.asarray(x["tail3"]["values"], dtype=float) for x in parsed["rois"])
    a = tuple(common.table_matrix(i, "table_a_raw") for i in (0, 1))
    b = tuple(common.table_matrix(i, "table_b_if_cm_m") for i in (0, 1))
    multi, walls = common.load_samples(a, b)
    bias_tables = pd.concat([make_pixel_bias(walls, coeffs, mode)
                             for mode in ("zero", "median", "observed_plane")], ignore_index=True)
    bias_tables.to_csv(OUTPUT / "delta_pixel.csv", index=False)
    # evaluate() uses mode names as lookup scopes; run each table separately.
    details = []; summaries = []
    combined = pd.concat([multi, walls], ignore_index=True)
    for mode in ("zero", "median", "observed_plane"):
        d, s = evaluate(combined, coeffs, bias_tables, mode)
        details.append(d); summaries.append(s)
    detail = pd.concat(details, ignore_index=True)
    summary = pd.concat(summaries, ignore_index=True)
    detail.to_csv(OUTPUT / "scene_metrics_pixel.csv", index=False)
    summary.to_csv(OUTPUT / "model_summary_pixel.csv", index=False)
    print("PIXEL_BIAS_STATS")
    print(bias_tables.groupby(["mode", "roi"]).delta_pixel.agg(["count", "mean", "std", "min", "max"]).to_string())
    print("PIXEL_MODEL_METRICS")
    print(summary.to_string(index=False))
    print(f"PIXEL_REPORT_DATA={OUTPUT}")


if __name__ == "__main__":
    main()
