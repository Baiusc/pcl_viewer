# CS96 Beta 编码器角度表：采集、换算与应用

> `ys_encoder_angle.csv` 描述"码盘每个粗刻度实际对应多少度"。它有**两条独立的获取路径**：
> **光学光条标定**（现网表就用这个）和 **刻度时间测量**（独立验证）。两者大多数刻度
> 吻合到 0.02° 以内，但在 164~169、227~229 等刻度段差 0.3~0.66°，**不能互相替代**。

---

## 1. 两条路子，别混

| | 光学光条标定 | 刻度时间测量 |
|---|---|---|
| 怎么做 | 转镜停在每个粗刻度位置发光，用相机/光条测实际光路角 | FPGA 记 `(n, cnt)`，多圈平均相邻刻度耗时，按时间比例分 360° |
| 得到什么 | 每格**绝对光学角** | 每格**相对时间**，需换算 |
| 现网 `ys_encoder_angle.csv` | ✅ **就是它** | ❌ 不是 |
| 强项 | 直接、可追溯到光路 | 不出光、可长时间无人值守重复测 |
| 弱项 | 依赖标定台与光条判读 | 只反映机械转角，不含光路/装配误差 |

**实证（不猜）**：把 `ys_encoder_angle.csv` 与 xlsx sheet2 的两列做逐点比对：

```
以「光条测算」为基准:
现有 csv vs 光条                 max=+0.0000°  rms=0.0000°   ← 逐位一致
新表(49圈时间) vs 光条           max=-0.6242°  rms=0.1797°
以「刻度时间测算」为基准:
现有 csv vs 时间列               max=+0.6572°  rms=0.1358°
```

**结论：现网表 100% 来自光学光条标定**，时间测算是另一条线。任何"用时间数据重算一张表
覆盖现网表"的做法都必须先做分段比对，否则会把 0.6° 的偏差灌进水平角。

## 2. 时间数据长什么样

采集方案（PPT slide 6）：

```
FPGA  ：每个粗刻度输出 (n, cnt)，n = 0~239，cnt = 相对 0 刻度的时间计数
ARM   ：透传 (n, cnt) 到上位机
上位机：转成 (Θ, t) 算瞬时转速；Θ 由标定算法（光学表）提供
```

上位机对多圈同一刻度的间隔取平均，得到 `Δt[0..239]`：

- PPT 那批：**15 圈**，平均间隔 **1703.7us**，理论 `1000/240/2.5 = 1666.7us`；
- 2026-11 在 `.218` 采的：**49 圈**（`b1-b7 共 49 条曲线叠图`，每列 241 点），
  平均 **1701.3us**，与 PPT 吻合。

多圈平均是必须的：同一刻度跨圈波动只有 `2us/1703.7us = 0.117%`（PPT slide 8），
转速本身很稳，剩下的差异才是**码盘刻度不均匀**——那正是要测的量。

### 241 行 = 1 个 t0 + 240 个间隔

| 位置 | 含义 | 典型值 |
|---|---|---|
| 第 1 个 | FPGA 的 `t0`：首个刻度相对计数起点的时间，**不是一整格** | ~266us |
| 第 2~241 个 | 相邻粗刻度时间间隔 `Δt[0..239]` | 1469~1808us，均值 ~1701us |

转换时必须丢掉第一行，否则整表多一格。

> 纵轴标注的 `(1/125)` 实测与 PPT 的 us 口径一致，按 **us** 处理。若上游给的是时钟计数，
> 用脚本 `--unit-scale` 换算（125MHz → 1 计数 = 0.008us）。

## 3. 时间 → 角度

转镜匀速，**转过时间 ∝ 转过角度**，按累计时间比例分配 360°：

```
angle[0]   = 0
angle[i]   = 360 × Σ Δt[0..i-1] / Σ Δt[0..239]      (i = 1..240)
angle[240] = 360
```

**不能**用 `angle[i] = i × 360/240`：那只在刻度绝对均匀时成立。实测每格角度标准差
0.0454°、最大偏离 0.26°，在 10m 处对应横向 `10 × tan(0.26°) ≈ 4.5cm` 的错位。

### 3.1 细角度也用实测耗时（2026-11-13 起）

粗刻度查表只解决"格与格之间"的不均匀；**一格内部**（`realAngle` 0~4095）原来固定用
`0.0009°/count`（≈1.53°/1700count）线性折算，等于假设每格耗时都是 1700us。转速或个别格子
耗时有偏差时，整格都会被推歪。

现在把实测耗时接到 `apply()` 的 Beta 分支（`Cs96BetaCalibration.cpp`）：

```
angle_cur    = encoderAngle_[scale]
angle_next   = (scale < 240) ? encoderAngle_[scale+1] : 360°
angle_spacing= angle_next - angle_cur

interval     = timeIntervals_[scale]            // time_intervals.txt，us
realPart     = realAngle / interval * angle_spacing
realPart     = clamp(realPart, ±1.05×angle_spacing)
hc           = angle_cur + realPart - startingAngle_[face]
omega        = hc × 2 + omega0
```

要点：

| 项 | 说明 |
|---|---|
| `scale=240` 边界 | 原实现先取 `encoderAngle_[scale+1]` 再判 `>240`，`scale=240` 时是 `[241]` **越界读**；现在先判 `scale < lastScale`，末格取 360° |
| 跨零/脏值 | `angle_next <= angle_cur` 时退化为 `angle_cur + 360/240`，避免负 spacing |
| 超限处理 | 原实现 `realPart > 1.53×2` 时**无条件赋 +1.53**，负向偏差会被翻成正；现在 `std::clamp(±1.05×spacing)` |
| 缺表兜底 | 没有 `time_intervals.txt` 时 `timeIntervalUs()` 返回名义 1701us，行为与旧版一致 |
| 数据来源 | 所选标定目录下的 `time_intervals.txt`，每行一个 us 值，**240 个**（文件末行常无换行，`wc -l` 会少算 1） |
| 兼容 | 也接受 241 行（首行是 FPGA t0，按中位数一半判定后丢弃）、239 行（末格用均值补） |

**落到聚合表**：新增 section `time_interval`，240 行，单位 us。

```
time_interval,,0,1735.7301,,,,,
time_interval,,239,1664.3219,,,,,
```

`Update Calib` 会读所选目录下的 `time_intervals.txt` 并写进聚合表；找不到时日志会明确提示
"细角度退回名义 1701us"。`Select Calib` 加载聚合表时按 `time_interval` 段恢复。

**自检**：`data/param/cs96beta_calib_data/cs96beta_calibration.csv` 里
`grep -c '^time_interval,'` 应为 **240**。

## 4. 脚本

`tools/encoder_time_to_angle.py`（纯标准库）：

```bash
# 240 个间隔 → csv
python3 tools/encoder_time_to_angle.py intervals.txt -o ys_encoder_angle.csv

# 241 行原始数据（首行是 t0）→ csv，并与现网表逐点比对
python3 tools/encoder_time_to_angle.py raw241.txt --first-is-offset \
    --compare data/param/cs96beta_calib_data/ys_encoder_angle.csv \
    -o /tmp/ys_encoder_angle_time.csv

# 上游给的是 125MHz 计数
python3 tools/encoder_time_to_angle.py ticks.txt --unit-scale 0.008 -o enc.csv
```

用 `.218` 的 49 圈数据实跑：

```
间隔个数          : 240
单圈总时间        : 408316.0 us  (408.316 ms)
平均每格间隔      : 1701.32 us  (理论 1666.7 us)
折合转速          : 146.95 rpm  (2.449 圈/秒)
间隔偏离均值      : RMS 51.52 us, max 231.84 us (13.627%)
每格角度          : 均值 1.5000°  标准差 0.0454°  范围 [1.2956, 1.5947]  (理想 1.5000°)
对比参考表        : 241 点, 最大偏差 0.6242°, RMS 0.1797°
最大偏差位置      : scale=169 本次 253.7940° 参考 254.4182°
```

自洽性检查都过：

| 指标 | 本次 49 圈 | 参考 | 判读 |
|---|---|---|---|
| 平均间隔 | 1701.3us | PPT 1703.7us | 差 0.14% |
| 转速 | 146.95 rpm | 名义 150 rpm | 实际慢 2% |
| 每格角度标准差 | **0.0454°** | PPT 0.046~0.066° | 同量级 |
| 每格角度范围 | 1.2956~1.5947° | PPT 最大偏差 0.223° | 一致 |

**说明方法本身没问题，但结果表和现网表不是一回事。**

## 5. 为什么和现网表差 0.62°

把 xlsx sheet2 的「光条测算 − 刻度时间测算」逐点统计：

| 项 | 数值 |
|---|---|
| 差值 > 0.05° 的刻度数 | 107 / 241 |
| 差值范围 | −0.2903 ~ +0.6572 |
| 差 > 0.3° 的刻度段 | **164~169**、**227~229** |
| 吻合 < 0.02° 的刻度段 | 0~22、50~61 等 |

**分段吻合、分段分歧**，说明不是"某个方法整体错了"，而是某些刻度段两条路线看到的东西不同：

- 时间测量只看**机械转角**；
- 光条标定看**光路出射角**，包含转镜面形、装配偏心、镜面倾斜等光学偏差。

在 164~169、227~229 这些段，机械转角和光路角差到 0.5° 量级——如果现场点云在这些方位
出现"黑缝/错位"，就该优先怀疑这几段的光学/机械状态，而不是去改时间数据。

## 6. 落地与应用链条

```
【光学】光条标定 ──► ys_encoder_angle.csv ──┬──► 设备端 /etc/ys_encoder_angle.csv
                                            └──► PC 端 data/param/cs96beta_calib_data/
【时间】FPGA (n,cnt) ──► 多圈平均 ──► 脚本 ──► 对比/兜底（不直接上线）
                                                    │
                    ┌───────────────────────────────┘
                    ▼
        PC：parseEncoderAngle (Cs96BetaCalibration.cpp:328) → encoderAngle_[241]
            paAngle()  : encoderAngle_[scale] + 0.0009×realAngle  → PA 绝对角
            faceByPa() : 按 PA 落在哪个 90° 区间判面（不受四项标定开关影响）
            apply()    : hc = encoderAngle_[scale] + 0.0009×realAngle − startingAngle_[face]
                         omega = hc×2 + omega0
        设备：get_horizontal_corners() 用同一张表（ys_algo_horizontal_corners.c）
```

两端同一张表：`data/param/cs96beta_calib_data/ys_encoder_angle.csv` 与
`lidar_sdk/you_see_arm/conf/ys_encoder_angle.csv` 的 md5 相同（`4b7d7493…`）。

注意 `0.0009°/count` 是**细角度**（`realAngle` 0~4095），与粗刻度表分工：
粗刻度查表消除码盘不均匀，细角度用线性近似。

## 7. 坑

1. **241 ≠ 240**：241 = 1 个 t0 + 240 个间隔，不丢首行会报"需要 240 个间隔"。
2. **别拿时间表直接覆盖现网表**：与光学表差 0.62°，要先按 §5 分段比对。
3. **必须覆盖 scale 0~240 全部**：`parseEncoderAngle` 用 `NaN` 初始化 + `any_of` 检查，
   少一格就"编码器角度表必须包含 scale 0~240"，整张标定表加载失败 → 推流正常但判面全错。
4. **angle 必须 0~360 且 scale 不重复**：代码逐项校验，越界即整表失败。
5. **单调递增**：`faceByPa` 依赖此性质，手改值时别破坏。
6. **换雷达要重测**：表是单台码盘的实测特性，`.218` / `.221` 各测各的。

## 8. 与"左右晃动"的关系

PPT slide 3/5 的结论：细分角度范围内光条左右跳动，原因是

1. 相邻整数刻度线对应的发光角度差不均匀（不是理想值；两条整数刻度线 = 3°，即每粗刻度 1.5°）；
2. 转速不均匀（实测 0.117%，可忽略）。

这张表的作用就是消除第 1 项：把"理想 1.5°/格"换成"实测角度/格"，让码盘刻度误差不进入水平角。
不做这步，误差会直接进点云——表现为同一面内相邻 slot 角度间隔忽大忽小，也就是竖直黑缝和左右晃动。

## 9. 相关文档

| 文档 | 关系 |
|---|---|
| `doc/薄款雷达点云左右晃动原因分析及解决措施.pptx` | 现象、转速测试方案、结论（本表的动机） |
| `doc/薄款雷达1_码盘角度测算0225.xlsx` | sheet1 四面原始角度；sheet2 光条 vs 时间测算逐点对比 |
| `doc/cs96beta_pa_angle_analysis.md` | 拿到 `encoderAngle_` 之后的角度链条（PA → 判面 → omega） |
| `tools/encoder_time_to_angle.py` | 时间数据换算脚本 |
