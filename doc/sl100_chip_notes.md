# SL100 / dToF SPAD-SoC 芯片要点（开发速查）

> 依据：`SL100_ApplicationNote_v0.1_20260617_Vitalchem.pdf`（v0.10, 2026-03-13，共 51 页预览版）。
> 只摘录**会影响解析、标定、显示**的部分，寄存器细节查原文档。

## 0. 适用边界（先看这条，避免误用）

本仓库涉及两种规格，**标定表与包长不可混用**：

| 型号 | 线数 × slots | 芯片 | 说明 |
|---|---|---|---|
| CS96 Beta | 96 × 800 | **IMX459** | 老款，包长 1561B / 1266B |
| **CS96 Gamma（应为 cs120 Gamma）** | **120 × 800** | **SL100** | 当前这台，未试产，早期沿用了 CS96 系列名；包长 1945B / 1578B |

芯片与线数的对应关系是硬的：**IMX459 只有 96×800，SL100 只有 120×800**。
所以"120 线"就等于"SL100"，不需要再从别处推断。

本文档分两类信息：

- **通用（两类芯片共有）**：dToF 的 RAW/RANGING 分层、多回波 peak 语义、MEU 编码器、降采样分段、假峰过滤。
- **SL100 专有**：120 PIXEL、1024/2048/4096 bins、SUPER-PIXEL 0/1/2、SHORT 64/128/256 bins。

具体到这台 120 线机器的包格式、坐标解算与现有代码差距，见
[`cs96_gamma_sl100_protocol.md`](./cs96_gamma_sl100_protocol.md)。

线数由广播上报的 `line_counts` 自动识别，不要硬编码；但**广播值与实际像素数不一致时，
包长校验会失败**（表现为"能搜到、连不上/没数据"），先核对广播再怀疑网络。

## 1. 一句话架构

SPAD 阵列 → TDC 计时 → TCSPC 累加直方图 → DSP（RAW 直出 / RANGING 峰值检测）→ Data Buffer → MIPI。

`work_mode` 的 0/1/2 就是**在这条链上切了三刀**，不是软件层的三个功能开关。

## 2. 关键规格（SL100）

| 项 | 值 |
|---|---|
| SPAD 阵列 | 80 (H) × 480 (V) ≈ 38K，pitch 10.08 µm；左右半区 **OR 合并**后进 TCSPC |
| PIXEL | 120 个（每 PIXEL = N(H)×4(V) SPAD binning） |
| SUPER-PIXEL | mode 0/1/2 → 120/60/30 个，bins 1024/2048/4096 |
| bin | 12 bit/bin；宽度 1/2/4 ns 可配 |
| 最大测距 | 153 m @1024 bins/1ns；300 m 靠分段降采样 |
| 多回波 | LONG_MEAS 最多 4 peak，SHORT_MEAS 1 peak |
| 帧 | 内部 FSYNC 每 K 个 measurement 一帧，K = `num_of_meas_per_frame` |
| 数据口 | MIPI CSI-2 4 lane，RAW16 |

换算基准：**1 ns bin ≈ 14.99 cm 单程**（c/2）。代码里 `ft>>1` 得 ns，故 ft 步进 0.5 ns ≈ 7.5 cm，这是原始点云的距离量化下限。

## 3. work_mode 的物理含义（最有价值的一条）

| work_mode | SL100 DSP | 数据形态 | 设备需加载的驱动 |
|---|---|---|---|
| 0 原始直方图 | `dsp_mode=0` **RAW** | 每个 PIXEL 完整 histogram | `histogram/*.ko` |
| 1 原始点云 | `dsp_mode=1` **RANGING** | 设备自己算 ft/peak，PC 端重算角度 | `point_cloud/*.ko` |
| 2 真实点云 | `dsp_mode=1` **RANGING** | 设备算好 xyz 直推 | `point_cloud/*.ko` |
| 4 灰度 | 另一套构建 | 灰度驱动输出 | `point_cloud_gray/*.ko` |

对应设备端 `/mnt/mtdblock2/mode` 文件决定 `insmod` 哪个 `.ko` 并覆盖 `liblidar.so`。
**`work_mode` 必须与该 `mode` 匹配**，否则 `you_see` 启动即退出：

```
ERROR ys_conf.c:407: 工作模式 0 与当前固件数据模式不匹配
FATAL main.c:78: load config fail
```

这就是之前"改 work_mode=0 必然失败"的根因——不是配置写错，是驱动层还是 point_cloud。
软件已按 `mode` 限定可选 work_mode（`histogram→{0}`、`point_cloud→{1,2}`、`point_cloud_gray→{4}`）。

## 4. RANGING 的多回波：`peak` / `echo` 到底是什么

每个 peak 芯片给出：`peak_center`、`peak_index`、`peak_raw_max`、`peak_value`、`intensity_sum`、左右窗和。

三个必须知道的点：

1. **`peak_center` 是高精度值**：定点数（高 14 位整数 + 低 6 位小数），单位 ns，精度 1/64 ns ≈ **2.3 mm**。
   若设备只透传 `ft`（0.5 ns 步进），等于把 mm 级信息丢成了 cm 级。做高精度测距时应争取拿 `peak_center`。
2. **peak 排序方式可配**（`peak_rank_mode`）：00 按幅值高→低、01 按索引近→远、02 按索引远→近。
   代码里"两回波取 pk 更大"只在 00 模式下等价；换模式后 peak0 不再是最强回波。
3. **`peak_index` 是窗口中心 bin**；`peak_raw_max_index` 是最高 bin。两者不等，窗口为偶数宽时取中心右侧。

Echo 数据是每个 peak 附近 **16 bins 的部分直方图**（12 bit/bin），可用来判回波真实性（宽而低＝噪声，窄而高＝硬目标）。

## 5. MEU：`scale` / 240 计数的物理来源

芯片内置 **MEU（Motor encoder unit）**，接收外部电机编码器的单相 PWM，并把 tick 打进包头：
`meu_speed`(20b)、`meu_position_tick_id`(13b)、`meu_position_tick_distance`(20b)。

这解释了 CS96 系列（Beta / Gamma 共用同一套编码器逻辑）的 `mirror_angle → scale`：

- 一圈 240 个 `scale` = 编码器 240 个计数，每计数 1.5°；
- `encoder_angle[scale]` 表不是拟合出来的，而是**编码器装配后的实际角度查表**；
- 因此 slot→角度必须查表，**不能用线性公式代替**（表本身就不是等间距的）。

`slot_ang` 的 1:2 关系（转镜反射角 = 机械角 × 2）也来自这里：镜面转 1°，光束转 2°。

## 6. 降采样：距离换算必须分段

LONG_MEAS 可把直方图切成 2~3 段不同 bin 宽度：

| downsamp_mode | bin 宽度 | ToF 宽度 |
|---|---|---|
| 0 | 全 1 ns | (Max+1)×1ns |
| 1 | 1 ns 前段 / 2 ns 后段（分界 th0） | (2×Max−th0+2)×1ns |
| 2 | 1 ns 前段 / 4 ns 后段（th1） | (4×Max−3×th1+4)×1ns |
| 3 | 1/2/4 ns 三段（th0、th1） | (4×Max−th1−2×th0+4)×1ns |

**后果**：一旦开了降采样，`bin → 距离` 是分段线性，单一 `a[pixel]` 系数只能拟合一段。
若现场出现"近距准、远距系统性偏大/偏小"，先查设备是否开了 downsamp，再决定是否要分段标定。
`downsamp_mode` / `th0` / `th1` 会随每个 PIXEL 的 config info 一起下发，可从包里读出来校验。

## 7. 假峰过滤：近距为什么容易丢弱目标

DSP 有两级阈值，按 `peak_index` 与 `th_k1_k2_switch`（默认 20）切换：

```
threshold1 = th_cal_k1(3) × X + th_cal_offset1(250)   // 近距
threshold2 = th_cal_k2(6) × Y + th_cal_offset2(100)   // 远距
```

低于阈值判为假峰直接丢弃。官方还给了归因：

- **false peak 1 = 镜头反射鬼影**（常在最近距离，固定出现）
- **false peak 2 = 环境光噪声**

实践含义：

- 近距离深色/低反射目标"测不到"多半是阈值问题，不是硬件坏；
- 若点云里总有一个固定距离的近距离层，那是镜头鬼影，应在显示/算法层按距离下限剔除，而不是当真实目标；
- 调阈值能提升检出率，但调高会误杀真峰，必须按场景配。

## 8. 数据布局两个坑

1. **GR 与 SUPER-PIXEL 输出顺序是 `0 → 2 → 1 → 3`**，不是 0→1→2→3。
   1 GR = 4 PIXEL = 16 行 SPAD，共 30 GR。原始点云"像素倒序"很可能与此有关。
2. **`num_gr_per_packet` 必须整除 30**（1/2/3/5/6/10/15/30），`num_pixels_per_gr ∈ {1,2}`。
   分组一变，包内像素顺序就变，解析不能假设固定布局。

另外 `pixel_dword_count` 会随 **SHORT_MEAS 是否开启**变化：

- 关：LONG 段 = 4 peak×11 + 4 echo×6 + fixed 6 DWORD；
- 开：额外多出 SHORT config(4) + SHORT peak(11) + SHORT echo(6) + fixed(6)。

原始点云包长稳定不动（96 线 1561B = 96×16B + 头；120 线 1945B = 120×16B + 头），
说明**当前两种机型都没开 SHORT_MEAS**。若某天包长变了，先查这个开关。

## 9. 还在包头里的好东西（目前未用）

| 字段 | 位置 | 用途 |
|---|---|---|
| `tsensor_data` 11b | EBD | 片上温度 → 距离漂移补偿 |
| `fbk_timer_rise/fall_shot0~15` | EBD | 实测出光延迟（1/16 ns 精度）→ ToF 起点校正 |
| `amb_intensity` 22b + `integrator_count/cycle` | pixel_body | 环境光强度 → 噪声门限、白天性能评估 |
| `num_spads_long` | pixel_body | 该像素使能的 SPAD 数，用于强度归一化 |

`amb_intensity` 真值 = `raw/2^22`，也等于 `integrator_count / integrator_cycle / num_spads_long`。

## 10. 对 viewer 的行动项

已确认可直接用的：

- 按 `mode` 文件限制 work_mode（已实现）。
- 强度归一化：`peak` 的满量程与 `num_spads_long` 有关，跨线/跨型号比较反射率前应先归一化。

待验证（不要在没测之前写死）：

- **直方图 bin 数对不上**：SL100 手册的 LONG_MEAS 全直方图是 1024 bins（mode 0），
  但 Gamma 固件实际每像素只传 **512 bins**（30 包 × 4 像素 × 512 × 2B）。
  512 是**只传前半段**还是**降采样合并**，需实测确认，否则 bin→距离换算会差 2 倍。
- peak 排序模式是否为 00（按幅值）。若不是，"取 pk 更大"的逻辑要改。
- `scale` 随 slot 递增还是递减：属于固件/装调，芯片文档给不出，仍需抓帧实测。
