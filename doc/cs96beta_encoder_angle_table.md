# CS96 Beta 角度标定：从码盘计时到 omega

> 角度不是一次算出来的，是**三步递进 + 一步合成**：
> FPGA 测码盘耗时 → 时间换算得到**初版**角度表 → 光条 CV 修正成**终版**表 →
> 用终版表 + 耗时表 + 面起始参数算 `omega`。
>
> 关键点：**时间表只能给"相对关系"，给不了"绝对光路角"**。所以第 ② 步产出的表是初值，
> 必须再经第 ③ 步光学修正才上线。

---

## 1. 四步流程总览

| 步骤 | 输入 | 动作 | 输出 | 执行方 |
|---|---|---|---|---|
| **① 码盘计时** | 转镜实际转动 | FPGA 逐刻度记 `(n, cnt)`，多圈平均 | `time_intervals.txt`（240 个 us） | FPGA + 上位机 |
| **② 时间→角度** | `time_intervals.txt` | 按累计时间比例分 360°，得**初版**角度表 | `ys_encoder_angle.csv`（初版）→ 写入雷达 **ARM flash** | 上位机 + ARM |
| **③ 光条标定** | 初版表 + 转镜停在刻度位发光 | CV 测真实光路角，逐格修正 | `ys_encoder_angle.csv`（**终版**） | 标定台 |
| **④ 解算 omega** | 终版表 + `time_intervals.txt` + `ys_calibration.ini` | 粗刻度查表 + 细角度按耗时折算 − 面起始角 | `omega`（水平角） | ARM / PC（同构） |

```text
FPGA 逐刻度计时 ──► time_intervals.txt ──┬──► ② 按时间比例分 360° ──► 初版 angle 表 ──► ARM flash
                                        │                                  │
                                        │                          ③ 光条 CV 逐格修正
                                        │                                  │
                                        │                                  ▼
                                        │                          终版 ys_encoder_angle.csv
                                        │                                  │
                                        └──────────────┬───────────────────┘
                                                       ▼
                    ④ omega = f(终版 angle 表, 每格耗时, standard_encoder, side_angular_offset)
```

## 2. 为什么必须是两步：时间表只反映机械转角

| | 码盘计时（①②） | 光条标定（③） |
|---|---|---|
| 测到什么 | **机械**转角的时间比例 | **光路出射角** |
| 含光学误差 | 否（面形、装配偏心、镜面倾斜都测不到） | 是 |
| 能否直接上线 | 能，但精度有限（初版） | 能（终版） |
| 成本 | 不出光、可长时间无人值守重复测 | 依赖标定台与光条判读 |

**实证（不猜）**：把现网 `ys_encoder_angle.csv` 与 xlsx sheet2 的两列逐点比对：

```text
现网 csv    vs 光条列   max=+0.0000°  rms=0.0000°    ← 终版表与光条标定逐位一致
49圈时间表  vs 光条列   max=-0.6242°  rms=0.1797°    ← 初版与终版差多少
现网 csv    vs 时间列   max=+0.6572°  rms=0.1358°    ← 同一差值的另一侧看
```

所以 **0.62° 不是"两条路线打架"，而是第 ③ 步修正掉的量**：

| 项 | 数值 |
|---|---|
| 差 > 0.05° 的刻度 | 107 / 241 |
| 差值范围 | −0.2903 ~ +0.6572 |
| 差 > 0.3° 的刻度段 | **164~169**、**227~229** |
| 吻合 < 0.02° 的刻度段 | 0~22、50~61 等 |

**分段吻合、分段分歧**——吻合的段说明机械与光路一致；分歧 0.3~0.6° 的段，通常是转镜面形 /
装配偏心在该角度上的贡献。若现场点云在这些方位出现黑缝或错位，优先查这几段的机械与光学状态，
不要去改时间数据。

## 3. ① 码盘计时：time_intervals.txt 怎么来的

采集方案（PPT slide 6）：

```text
FPGA  ：每个粗刻度输出 (n, cnt)，n = 0~239，cnt = 相对 0 刻度的时间计数
ARM   ：透传 (n, cnt) 到上位机
上位机：转成 (Θ, t) 算瞬时转速；Θ 由标定算法（当前角度表）提供
```

上位机对多圈同一刻度的间隔取平均，得到 `Δt[0..239]`：

- PPT 那批：**15 圈**，平均间隔 **1703.7us**，理论 `1000/240/2.5 = 1666.7us`；
- 2026-11 在 `.218` 采的：**49 圈**（`b1-b7 共 49 条曲线叠图`，每列 241 点），
  平均 **1701.3us**，与 PPT 吻合。

多圈平均是必须的：同一刻度跨圈波动只有 `2us/1703.7us = 0.117%`（PPT slide 8），
转速本身很稳，剩下的差异才是**码盘刻度不均匀**——那正是要测的量。

### 行数：241 行 = 1 个 t0 + 240 个间隔

| 位置 | 含义 | 典型值 |
|---|---|---|
| 第 1 个 | FPGA 的 `t0`：首个刻度相对计数起点的时间，**不是一整格** | ~266us |
| 第 2~241 个 | 相邻粗刻度时间间隔 `Δt[0..239]` | 1469~1808us，均值 ~1701us |

> 现场落盘的 `time_intervals.txt` 常常只有 240 个值且**末行无换行**，`wc -l` 会少算 1
> （显示 239）。解析器两者都接受。

> 纵轴标注的 `(1/125)` 实测与 PPT 的 us 口径一致，按 **us** 处理。若上游给的是时钟计数，
> 用脚本 `--unit-scale` 换算（125MHz → 1 计数 = 0.008us）。

## 4. ② 时间 → 角度：生成初版表

转镜匀速，**转过时间 ∝ 转过角度**，按累计时间比例分配 360°：

```text
angle[0]   = 0
angle[i]   = 360 × Σ Δt[0..i-1] / Σ Δt[0..239]      (i = 1..240)
angle[240] = 360
```

**不能**用 `angle[i] = i × 360/240`：那只在刻度绝对均匀时成立。实测每格角度标准差
0.0454°、最大偏离 0.26°，在 10m 处对应横向 `10 × tan(0.26°) ≈ 4.5cm` 的错位。

这一步的产物烧进雷达 **ARM flash**，让设备端先有一套可用角度表；随后第 ③ 步再把它修正成终版。

## 5. ③ 光条 CV 修正：得到终版表

- 转镜依次停在每个粗刻度位置发光，用相机/光条测**真实光路出射角**；
- 逐格替换/修正初版值，得到终版 `ys_encoder_angle.csv`；
- 终版表同时下发到两端：

```text
设备端 /etc/ys_encoder_angle.csv          （源：lidar_sdk/you_see_arm/conf/）
PC 端   data/param/cs96beta_calib_data/   （解析：Cs96BetaCalibration.cpp:parseEncoderAngle）
```

两端 md5 必须一致（历史值 `4b7d7493…`），否则设备推流与 PC 解算会按不同表算角度。

`doc/薄款雷达1_码盘角度测算0225.xlsx` 的 sheet2 就是这一步的验收表：
「角度（光条测算）」vs「角度（刻度时间测算）」逐点对差。差异大的段落要人工复核，
不要简单取平均。

## 6. ④ 解算 omega：三段合成

### 6.1 面起始角（来自 `ys_calibration.ini`）

```text
startingAngle[face] = standardEncoder[face] × (360/240) + sideAngularOffset[face] × (360/240/256)
                    = standardEncoder[face] × 1.5°   + sideAngularOffset[face] × 0.005859°
```

| 参数 | 含义 | 单位 | 实测值 |
|---|---|---|---|
| `angle_face_N` / `standard_encoder` | 第 N 面起始发光的粗刻度 | 1 格 = 1.5° | 22 / 82 / 142 / 202 |
| `side_angular_offset` | 面间细调（相对第 0 面） | 1/256 格 ≈ 0.0059° | 见 ini |
| 结果 | 四面间隔约 90° | 度 | 33.13 / 123.00 / 213.18 / 303.19 |

> 上表是**绝对表**（`anchor_per_face=false`）的取法；每面锚定表（20261117 起，`anchor_per_face=true`）
> 不再用这条公式，而是直接取表值，见 6.5。

### 6.2 粗刻度 + 细角度 → `omega`

```text
angle_cur    = encoderAngle[scale]                                  // 终版表
angle_next   = (scale < 240) ? encoderAngle[scale + 1] : 360°
angle_spacing= angle_next - angle_cur

interval     = timeIntervals[scale]                                 // ① 的产物，us
realPart     = realAngle / interval × angle_spacing                 // 细角度
realPart     = clamp(realPart, ±1.05 × angle_spacing)

hc           = angle_cur + realPart - startingAngle[face]           // wrap 到 ±180
omega        = hc × 2 + omega0                                      // ×2：镜面反射放大一倍
```

- `realAngle`：本格内的计时计数（12 位）；
- `× 2`：镜面绕竖直轴转 θ，反射光线偏 2θ；
- `omega0 = -60°`：单面 120° 视场相对光路中心的偏移；
- `timeIntervals` 在这里**第二次派上用场**：① 用它生成初版粗刻度表，④ 用它把格内计时折算成角度。

### 6.3 缺省与兜底

| 情况 | 行为 |
|---|---|
| 没有 `time_intervals.txt` | `timeIntervalUs()` 返回名义 1701us，等价旧实现 |
| `scale = 240`（末格） | 取 360° 收尾；旧实现先取 `[scale+1]` 再判边界，会越界读 |
| `angle_next ≤ angle_cur` | 退化为 `angle_cur + 360/240`，避免负 spacing |
| `realPart` 超限 | `clamp(±1.05×spacing)`；旧实现无条件赋 `+1.53`，会把负偏差翻成正 |

### 6.4 落在聚合表里的样子

`Update Calib` 从所选目录读 `time_intervals.txt`，写进聚合表：

```csv
time_interval,,0,1735.7301,,,,,
time_interval,,239,1664.3219,,,,,
```

`Select Calib` 加载聚合表时按 `time_interval` 段恢复。自检：
`grep -c '^time_interval,' data/param/cs96beta_calib_data/cs96beta_calibration.csv` 应为 **240**。

### 6.5 每面锚定表（`anchor_per_face=true`，20261117 起）

光斑模式的按刻度开火与点云模式的按时间开火，都保证在同一刻度起爆，因此标定端把"**真实起爆角**"
锚定成固定绝对值（机械 -30° / 视场 -60°），四个面各自重算一段，表因此变成四段锯齿：

```text
真实起爆角 = 粗刻度表值 + side_angular_offset × (1.5 / 256)
⇒ 粗刻度表值 = -30° - side_angular_offset × 1.5/256      // 标定端 anchor_start_mech_angle - 细调
⇒ 表值(尺度 s) = 该值 + Σ(相邻刻度实测机械增量)            // 不发光刻度按 0° 或 1.5° 递推补全
```

以 `side3 = 33`、`angle_face_4 = 202` 为例：`202 → -30° - 33×0.005859° = -30.193359°`，
固件加回细调后起爆角仍为 `-30.000000°`。整表 240 行 + 1 行回绕行（第 0 行 + 360°），
所以第 240 行的值会大于 360°。

解算侧（`Cs96BetaCalibration`）的适配：

| 量 | 绝对表 | 每面锚定表 |
|---|---|---|
| 本面起始角 | `startingAngles()`（ini 公式） | 起始刻度在表里的值 `tableStart[face]` |
| `hc` | `表值 + 细角度 - 面起始角` | 同左，但面起始角换成表值 ⇒ 起爆处 `hc = 0` |
| `omega` | `hc×2 + omega0` | 不变（两种表下 `hc` 含义相同） |
| `paAngle()` 判面 | 表值即绝对角 | 表值 + `dialBase[face]`（= `startingAngle[face] - 锚点`）还原绝对角 |

表布局在 `load()` 时按数值范围自动判定：**出现负数或 >360° 的回绕行**即认定每面锚定表
（`anchorPerFace()`），也可用 `setAnchorPerFace()` 手动覆盖。

## 7. 代码位置

```text
PC：parseEncoderAngle (Cs96BetaCalibration.cpp) → encoderAngle_[241]     ← ③ 终版表
    parseTimeIntervals (同文件)                 → timeIntervals_[240]    ← ① 耗时表
    finalizeGeometry                            → startingAngle_[4]      ← 6.1 面起始角
    paAngle()   : encoderAngle[scale] + 0.0009×realAngle  → PA 绝对角（仅判面用）
    faceByPa()  : 按 PA 落在哪个 90° 区间判面（不受四项标定开关影响）
    apply()     : 6.2 的完整链条 → omega
设备：get_horizontal_corners()（ys_algo_horizontal_corners.c）用同一张表，PC 端是复刻
```

> `paAngle()` 里的 `0.0009°/count` 是**判面专用**的粗精度近似，不参与点云几何；
> 点云细角度走 6.2 的 `realAngle / interval × spacing`。两者不要混用。

## 8. 脚本

`tools/encoder_time_to_angle.py`（纯标准库），用于 ②：

```bash
# 240 个间隔 → 初版 csv
python3 tools/encoder_time_to_angle.py intervals.txt -o ys_encoder_angle.csv

# 241 行原始数据（首行是 t0）→ csv，并与终版表逐点比对
python3 tools/encoder_time_to_angle.py raw241.txt --first-is-offset \
    --compare data/param/cs96beta_calib_data/ys_encoder_angle.csv \
    -o /tmp/ys_encoder_angle_time.csv

# 上游给的是 125MHz 计数
python3 tools/encoder_time_to_angle.py ticks.txt --unit-scale 0.008 -o enc.csv
```

用 `.218` 的 49 圈数据实跑（得到的是**初版**，与终版有 0.62° 差属正常）：

```text
间隔个数          : 240
单圈总时间        : 408316.0 us  (408.316 ms)
平均每格间隔      : 1701.32 us  (理论 1666.7 us)
折合转速          : 146.95 rpm  (2.449 圈/秒)
间隔偏离均值      : RMS 51.52 us, max 231.84 us (13.627%)
每格角度          : 均值 1.5000°  标准差 0.0454°  范围 [1.2956, 1.5947]  (理想 1.5000°)
对比终版表        : 241 点, 最大偏差 0.6242°, RMS 0.1797°
最大偏差位置      : scale=169 初版 253.7940° 终版 254.4182°
```

自洽性检查：

| 指标 | 本次 49 圈 | 参考 | 判读 |
|---|---|---|---|
| 平均间隔 | 1701.3us | PPT 1703.7us | 差 0.14% |
| 转速 | 146.95 rpm | 名义 150 rpm | 实际慢 2% |
| 每格角度标准差 | **0.0454°** | PPT 0.046~0.066° | 同量级 |
| 每格角度范围 | 1.2956~1.5947° | PPT 最大偏差 0.223° | 一致 |

## 9. 坑

1. **241 ≠ 240**：241 = 1 个 t0 + 240 个间隔，不丢首行会报"需要 240 个间隔"。
2. **初版 ≠ 终版**：时间换算出的表只能当 ② 的交付物，上线前必须过 ③ 光条修正；
   直接覆盖终版会把 0.62° 的机械-光学差灌进水平角。
3. **必须覆盖 scale 0~240 全部**：`parseEncoderAngle` 用 `NaN` 初始化 + `any_of` 检查，
   少一格就"编码器角度表必须包含 scale 0~240"，整张标定表加载失败 → 推流正常但判面全错。
4. **angle 必须 0~360 且 scale 不重复**：代码逐项校验，越界即整表失败。
5. **单调递增**：`faceByPa` 依赖此性质，手改值时别破坏。
6. **两端同表**：设备 `/etc/ys_encoder_angle.csv` 与 PC `data/param/...` 的 md5 必须一致。
7. **换雷达要重测**：表是单台码盘的实测特性，`.218` / `.221` 各测各的，不能串用。

## 10. 与"左右晃动"的关系

PPT slide 3/5 的结论：细分角度范围内光条左右跳动，原因是

1. 相邻整数刻度线对应的发光角度差不均匀（不是理想值；两条整数刻度线 = 3°，即每粗刻度 1.5°）；
2. 转速不均匀（实测 0.117%，可忽略）。

第 ②③ 步解决第 1 项：把"理想 1.5°/格"换成"实测角度/格"。
不做这步，误差会直接进点云——表现为同一面内相邻 slot 角度间隔忽大忽小，也就是竖直黑缝和左右晃动。

## 11. 相关文档

| 文档 | 关系 |
|---|---|
| `doc/薄款雷达点云左右晃动原因分析及解决措施.pptx` | 现象、转速测试方案、结论（本流程的动机） |
| `doc/薄款雷达1_码盘角度测算0225.xlsx` | sheet1 四面原始角度；sheet2 光条 vs 时间测算逐点对比（③ 的验收表） |
| `doc/cs96beta_pa_angle_analysis.md` | 拿到角度表之后的角度链条（PA → 判面 → omega 细节） |
| `tools/encoder_time_to_angle.py` | ② 的换算脚本 |
| `README.md` | 快速上手与按钮说明 |
