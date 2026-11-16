# CS96 Beta：PA 值、FPGA 旋转角与水平角的关系分析

> 仅分析与结论，不含代码改动。依据 `lidar_sdk/you_see_arm`（分支 `feat/cs96-beta-spot-slope`）
> 与 `10.42.0.218` 设备实测日志、`data/param/cs96beta_calib_data` 标定文件。

---

## 1. 三个量的定义

| 名称 | 来源 | 位宽/单位 | 说明 |
|---|---|---|---|
| **FPGA 旋转角** `mirror_angle` | 原始点云 slot 头 `word0` | 20 bit | FPGA 直出的转镜编码值，`mirror_angle = (word0 >> 12) & 0xFFFFF` |
| **PA 值** | 由 `mirror_angle` 拆出 | `scale` 8 bit + `real_angle` 12 bit | `scale = (mirror_angle >> 12) & 0xFF`，`real_angle = mirror_angle & 0xFFF` |
| **水平角** `omega` | 由 PA 值 + 标定表算出 | 度 / 弧度 | 最终用于 `x = r·cosθ·cosω` 等 |

关键点：**`mirror_angle` 和 PA 是同一个量的两种视图**，不是两个独立信号。

```
mirror_angle (20 bit)
├── bits 19..12  → scale        （0~239，一圈 240 个计数）
└── bits 11..0   → real_angle   （0~4095，计数内细分）
```

- 一圈 = 240 个 `scale`，每 `scale` = **1.5°**。
- 每 `real_angle` 步进 = `0.0009°`（`0.0009 × 1700 ≈ 1.53°`，20261104与fpga讨论后确认是此关系）。
- PA 角（编码器绝对角）：

```
pa_angle = encoder_angle[scale] + 0.0009 × real_angle
```

`encoder_angle` 来自 `ys_encoder_angle.csv`（`scale,angle`，241 行，0~360°），
是**整圈绝对角度表**，不是 `spot_slope_map.csv`。

> 这张表不是图纸算的，是**实测**的，分两步：先由 FPGA 的码盘耗时表
> （`time_intervals.txt`）按时间比例算出**初版**，再用**光条 CV 标定**逐格修正成**终版**。
> 初版与终版实测最大差 0.62°（164~169、227~229 段最明显），所以**初版不能直接上线**。
> 完整四步流程见 `doc/cs96beta_encoder_angle_table.md`。码盘刻度不均匀约 ±0.22°，
> 用 `n × 1.5°` 代替这张表会在 10m 处产生约 4cm 横向错位。

### 1.1 `scale` / 240 计数从哪来（芯片层依据）

同类 dToF SPAD-SoC（如 SL100，见 [sl100_chip_notes.md](./sl100_chip_notes.md)）内置
**MEU（Motor encoder unit）**，直接接收电机编码器的单相 PWM，并把 tick 打进数据包头
（`meu_speed` / `meu_position_tick_id` / `meu_position_tick_distance`）。

由此可得两条结论：

1. **一圈 240 个 `scale` 就是编码器 240 个计数**，不是软件约定值；
2. `encoder_angle[scale]` 是**装配后实测的角度表**，天然不等间距，
   所以 slot→角度必须查表，用 `scale × 1.5°` 线性公式会引入系统偏差。

> 注意型号差异：当前 CS96 Beta（96 线）是 IMX459，120 线型号才是 SL100。
> 两者 MEU/编码器思路相同，但 bin 数、PIXEL 数、包长不同，标定表不可混用。

---

## 2. 水平角的完整计算链

设备端 `ys_algo_horizontal_corners.c` 的顺序：

```c
// (1) 取该面的起始角
starting_angle = g_staring_angle[face];

// (2) slot 0 是帧同步点，直接给 omega0；其余用 PA 角
if (slot_number != 0) {
    angle = encoder_angle[scale] + 0.0009 * real_angle;
    hc = angle - starting_angle;
    // 归一化到 ±180
} else {
    hc = omega0;                 // -60
}

// (3) 转镜反射的 1:2 关系 + 全局零点
hc = hc * 2 + omega0;            // omega0 = -60

// (4) spot slope 修正（Beta 分支）
hc = get_beta_spot_slope(hc, face);

return hc;                       // 度
```

坐标换算（`ys_decode_common.c`，CS96_BETA 分支）：

```
omega = hc（弧度） + 逐线水平角偏移 omega_delta[pixel]
x = r · cosθ · cosω
y = -r · cosθ · sinω      // Beta 的 Y 轴取反
z = r · sinθ
```

因此三个量的因果关系是：

```
mirror_angle ──拆位──> scale, real_angle ──查表──> pa_angle
                                                     │
                                                     ├─ 减该面起始角 → hc（wrap ±180）
                                                     └─ ×2 + omega0  → 水平角（-60 ~ +57）
```

`face` 只决定**用哪个 `starting_angle[face]`**。

**不要再加 `face × 90°`**：单面视场约 120°，4 个面各自都从 -60° 开始扫，
把面间 90° 偏移加进来会把 4 个面摊成 360°。在线实现里专门写了这条注释
（`Cs96BetaCalibration.cpp:827`），离线解析漏掉"减起始角"后出现的
"点云整体转 180°"，本质就是这一项没扣掉。

## 2.1 常数溯源（别猜，每个都有出处）

| 常数 | 出现位置 | 含义 / 来源 |
|---|---|---|
| `240` | `kPaCountsPerRevolution` | **一圈编码器计数格数**。`kBetaEncoderAngle[]` 共 241 项（index 0~240），每格约 1.5° |
| `61440` | 导出格式、`raw/61440×360` | `240 × 256`，把 0~360° 量化成整数写进文本。1 单位 = 360/61440 = **0.005859°**。它只是**文本编码基数**，不是物理量；取 240×256 是为了让 240 格能整除表示 |
| `0.0009` | `encoder[scale] + 0.0009×real` | **转镜角速度**：`algo/ys_algo_def.h:30` 的 `g_angle_speed = 0.0009 度/us`。`real` 就是 12 位位域 `scale_time`（单位 us，`ys_core_def.h:155`），所以 `0.0009 × scale_time` 是"自该粗刻度起转过的角度"。scale 格距 1.5° 需要 1.5/0.0009 = **1667 us**，12 位满量程 4095 us（3.69°）留了约 2.4 倍余量 —— 之前的"2.4 倍不一致"其实是位宽余量，不是矛盾 |
| `×2` | `hc * 2` | 镜面反射：镜面转 θ，反射光线转 **2θ** |
| `-60` | `omega0` 默认值 | 视场 120°，中心对准正前方 → 范围 [-60, +60] |
| `65536` | 旧 `angle:` 格式的增量单位 | 1/65536 圈。**与 61440 不是同一个基数**，混用会让角度整体偏 6.25% |
| `4` | `kFaces` / `g_staring_angle[4]` | 转镜 4 面 |
| `90°` | 面间隔 | 4 面均分 360° 的**机械角**；×2 后光路相差 **180°** |
| `40` / `5` | `faceByPa()` | 面区间 40 计数 = 60°，容差 5 计数 = 7.5°（都乘 `360/240`） |
| `800` | slots/帧 | 由广播或文件头给，不要硬编码 |

## 2.2 在线原始点云 vs 离线 TXT（两条链的差距）

| 环节 | 在线（`BetaUdpStream` + `Cs96BetaCalibration`） | 离线（`pulse_data_processor.cpp`） |
|---|---|---|
| 角度原始量 | `mirrorAngle` 20 位（scale 8 + real 12） | `rotate mirror angle` 整数（1/61440 圈） |
| 编码器表 | `encoderAngle_[scale]`，从标定加载 | 无，靠 `raw/61440×360` 反算（等价） |
| 面号 | PA 判定 / `frame_id % 4` | **无** |
| 面起始角 | `− startingAngle_[face]` | **无**（等价于当 0 处理） |
| 归一 | wrap **±180** | **`mod 360`** |
| ×2 | ✅ | ✅ |
| `omega0` | `+ omega0_`（−60） | **无** |
| 无标定兜底 | `−60 + 120×slot/slots` 线性展开 | 无 |
| `slot_ang` 语义 | 相对角 **−60 ~ +60** | 绝对方位 **0 ~ 360** |
| Y 轴符号 | `y = −r cosθ sinω` | `y = +r cosθ sinω`（**左右镜像**） |

**离线补齐三样即可与在线一致**（不用面号、不用标定表）：

```
base = colRaw[1] − (colRaw[2] − colRaw[1])   // 本帧 slot0 实测角，替代 startingAngle_[face]
hc   = rawDeg − base
hc   = wrap 到 ±180            // 必须先相减再 wrap，不能先 mod 360（file_8 跨圈）
hc   = hc × 2 − 60
```

> 为什么不用线性 `slot × 0.15 − 60`：实测 799 个间隔共 9961 计数 = **0.1462°/列**
> （116.8°/800），而线性给 0.15°/列 = 120°。差 3.2°，15m 处端点偏约 0.84m。
> 编码器刻度本身不等间距（609 次增量 13、152 次 12），这正是固件要查表的原因。

---

## 3. PA 如何判定面

设备端 `beta_get_pa_frame_index()` 的逻辑：

```
pa_angle = encoder_angle[scale] + 0.0009 × real_angle   // 归一化到 0~360
for face in 0..3:
    delta = pa_angle - starting_angle[face]              // 归一化到 ±180
    if -tolerance·deg <= delta <= (range + tolerance)·deg:
        记录匹配面
唯一匹配 → 返回该面；匹配 0 个或多个 → 返回 -1
```

常量（`ys_decode_cs96_beta.c`）：

| 常量 | 值 | 含义 |
|---|---|---|
| `CS96_BETA_PA_COUNTS_PER_REVOLUTION` | 240 | 一圈 240 个计数 |
| `CS96_BETA_PA_RANGE_COUNTS` | 40 | 每面 40 个计数 = **60°** |
| `CS96_BETA_PA_TOLERANCE_COUNTS` | 5 | 前后各 5 计数容差 = 7.5° |
| `CS96_BETA_FACE_COUNT` | 4 | 4 面 |

每面的判定窗口：

```
[ start − 5计数, start + 45计数 ]  ≈ [ start − 7.5°, start + 67.5° ]
```

面起始间隔是 **60 计数（90°）**，而判定窗口 50 计数（75°），
相邻面之间留约 10 计数（15°）的“空档”，避免边界抖动误判。

实测每帧覆盖的 `scale` 范围（设备日志）：

```
frame_id=1  scale=[58~97]     // 40 个计数
frame_id=2  scale=[118~157]
frame_id=3  scale=[178~217]
frame_id=0  scale=[0~240]     // 统计跨整圈，实际是 238~240 + 0~37
```

相邻面 `scale` 起点相差 60 格 = 90° **机械角**。这是转镜的物理结构差，
**不是要加进水平角里的量**：每个面各自从自己的 `starting_angle` 起算，
减去后都回到 0，所以 4 个面的点云会重叠在同一段视场里（而非拼成 360°）。

---

## 4. 为什么原始点云用 PA，而真实点云用帧 ID

### 4.1 原始点云：FPGA 帧号不可靠

设备日志（每 5 秒一条）：

```
[CS96_Beta] PA 纠正面号: fpga_frame_id=231 raw_face=3 pa_face=0
[CS96_Beta] 最近 32768 个 SLOT 的 PA/frame ID 校验: 纠正=32727, PA无法判定=0
```

- `raw_face = fpga_frame_id % 4` 与真实面 `pa_face` 不一致；
- **纠正率 32727 / 32768 ≈ 99.9%**，即原始点云里的 `frame_id` 几乎一直是错的。

原因：原始点云是 FPGA 直出，`frame_id` 来自 FPGA 内部计数，与转镜实际面号存在系统性错位。

因此原始点云模式必须：

```
face = PA 区间判定（唯一匹配）
若 PA 无法唯一判定 → 才回退 frame_id % 4
```

### 4.2 真实点云：帧号可用

真实点云（`work_mode=2`）由设备端算好 xyz 后输出，其包头 `frame_index`
实测按 `0,1,2,3,0,1,2,3…` 递增且与面一致（用户确认，画面稳定无旋转）。

因此真实点云可以：

```
face = frame_index % 4
```

——但注意：**真实点云模式下 PC 端不需要重新算角度**，设备已经输出 xyz，
`face` 只用于“面选择”这类展示功能。

---

## 5. 当前 PC 端实现的偏差

| 位置 | 现状 | 问题 |
|---|---|---|
| `BetaUdpStream::parseCartesian` | `face = slot / 200` | 真实点云每帧 800 slot 属于同一边，按 slot 切 4 段是错的 |
| `BetaUdpStream::parseOrigin` | `face = faceByScale(mirrorAngle)` | `faceByScale` 用固定 scale 段（8/68/128/188），把一帧切成 4 段，语义错误 |
| `Cs96BetaCalibration::apply` | 用 `startingAngle_[face*60]` 近似 | 与设备实测 `start` 有 1~3° 偏差 |
| `Faces` 按钮 | 按 slot 过滤 | 与“1 面 = 1 帧”语义冲突，用户实际看到的是把每帧切碎 |

另外：`faceByScale()` 的 scale 分段（8/68/128/188）来自**发光刻度**，
与 PA 判定用的 `starting_angle` 起点（约 58/118/178/238）**不是同一套基准**，
两者相差约 50 个计数（75°）。混用会直接导致面号与角度错位。

---

## 6. 实测起始角与仓库公式的矛盾（需以设备为准）

仓库公式：

```c
starting_angle[i] =
    (angle_face[i] * 256 + side_angular_offset[i]) / 256 * 360 / 240;
```

代入 `ys_calibration.ini`：

```
angle_face = {2232, 17592, 32952, 48312}
side_offset = {15, 63, 60, 0}
```

计算得：

```
face0: (2232×256+15)/256 × 1.5 = 3348.09 → mod 360 = 108.09°
face1: (17592×256+63)/256 × 1.5 = 26388.4 → mod 360 = 108.37°
```

**4 个面算出来几乎相同**，与设备日志明显不符。

设备日志实测：

```
face0 start = 357.00
face1 start = 87.43
face2 start = 177.54
face3 start = 267.03
```

两者相差 90°，且 face0 = 357（≈ −3°）。

**可能原因**（按可能性排序）：

1. 设备实际固件的 `PA_COUNTS_PER_REVOLUTION` 不是 240，
   反推约 **2250**（`2232 × 360/2250 ≈ 357.1`），但该值对 face1 又不成立，
   说明 `angle_face` 数组与 ini 的 `angle_face_1..4` 顺序可能不一致。
2. `angle_face` 在设备 ini 中与仓库默认 ini 不同（设备上 `/etc/ys_calibration.ini` 未逐项比对）。
3. 设备固件版本早于当前 `feat/cs96-beta-spot-slope` 分支。

**结论**：PC 端不要用仓库公式硬算，应以**设备日志 `[FACE] … start=` 的实测值**为准，
或从设备读取实际生效的 `starting_angle`。

---

## 7. 对 PC 端实现的分析结论（不含代码）

### 7.1 原始点云（work_mode=1）

```
对每个包：
  mirror_angle = (word0 >> 12) & 0xFFFFF
  scale = (mirror_angle >> 12) & 0xFF
  real_angle = mirror_angle & 0xFFF
  pa_angle = encoder_angle[scale] + 0.0009 × real_angle      // 0~360

  face = 唯一命中的面 i，满足
         (pa_angle - start[i] ∈ [−7.5°, +67.5°])
         无匹配或多重匹配 → 回退 frame_id % 4

  hc = pa_angle - start[face]                                 // ±180 归一化
  omega_deg = hc × 2 − 60 + omega_delta[pixel]        // 不加 face×90
  r = 0.000299792458 × (ft >> 1) × a[pixel] + d_delta[pixel] × 0.01

  x = r·cosθ·cosω
  y = −r·cosθ·sinω
  z = r·sinθ
```

要点：

- **一帧 800 slot 应落在同一个面**（同一段 40 个 scale 内），
  所以面号应在**帧级**确定，而不是逐 slot 抖动。
- 正确做法：对一帧内所有 slot 求 PA 面号的众数/一致性校验，
  整帧统一用一个 `face`。
- 4 帧（face 0→1→2→3）只是转镜转一圈的机械过程。**每帧独立输出 96×800 = 120° FOV，
  不拼接、不拼整圈** —— CS96 是车规级前向主雷达，不是 360° 环视雷达。

### 7.2 真实点云（work_mode=2）

```
face = frame_index % 4
```

- 设备已算好 xyz，`face` 只用于面选择/着色分组；
- 不要用 PA 重算角度，也不要用 slot 分面。

### 7.3 面选择

- 语义：**按帧（面）过滤**，不是按 slot；
- 由于一帧只有一面，要同时显示多面必须**累积 4 面帧**再合成
  （列数 4×800，或按选中面紧凑排列）；
- 只显示单面时，画面应是一段约 60°~118° 的扇区，且**不随帧跳变**。

---

## 8. 一句话总结

- **PA = `mirror_angle` 拆位后查整圈编码器表得到的绝对角度**；
- **PA 面号 = PA 角落在哪个面的 60° 区间**，原始点云专用（因为 FPGA 帧号 99.9% 错位）；
- **水平角 = (PA 角 − 该面起始角) × 2 − 60 + 逐线偏移**（不加 `face×90`）；
- 真实点云已由设备算好 xyz，面号直接用 `frame_index % 4`；
- 当前 PC 端“按 slot 分面”和“用固定 scale 段分面”都是错的，
  这是原始点云旋转异常与面选择语义混乱的共同根因。

---

## 9. alpha 与 beta 的水平角链条对比（源码 `2865c8b`）

来源：`lidar_sdk/you_see_arm` @ `2865c8b`（alpha 转镜版本）。
型号由编译宏 `CS96_ALPHA` / `CS96_BETA` 二选一（`Makefile:38/40`）。

### 9.1 alpha 链条

入口：`ys_business/ys_algo.c:415`

```c
horizontal_corner = get_horizontal_corners(p_head->slot_number, p_head->frame_index,
                                           org_data->code_disc_angle);
```

`ys_business/algo/ys_algo_horizontal_corners.c:80-100`（alpha 生效分支）：

```c
static const double k_mirror_portion = 360.0 / MAX_ENCODER;   // MAX_ENCODER = 65536

int16_t raw_diff = (int16_t)(code_disc_angle - g_omega0[now_frame_index]);
// 跨零点归一，超过半圈就补一圈
if (raw_diff >  MAX_ENCODER / 2) slot_angel = (raw_diff - MAX_ENCODER) * k_mirror_portion * 2;
else if (raw_diff < -MAX_ENCODER / 2) slot_angel = (raw_diff + MAX_ENCODER) * k_mirror_portion * 2;
else                             slot_angel = raw_diff * k_mirror_portion * 2;

horizontal_corner = (slot_number != 0)
    ? get_spot_slope(slot_angel, code_disc_angle, now_frame_index)   // 查 bin 曲线做非线性修正
    : cal_param.omega0.f;                                             // slot0 直接 -60
```

三处基准：

| 量 | 位置 | 计算 |
|---|---|---|
| `MAX_ENCODER` | `algo/ys_algo_def.h:18` | alpha = **65536** |
| `g_standard_encoder[i]` | `algo/ys_algo_def.h:25` | 默认 `{0xBB8, 0x4BB8, 0x8BB8, 0xCBB8}`，间隔 `0x4000 = 90°`；标定可覆盖 |
| `g_omega0[i]` | `ys_algo.c:263` | `get_mid_slot_code(angle_face[i], side_angular_offset[i])`，即该面**中间 slot** 的编码值 |

`g_omega0` 依赖 `spot_slope_map.bin`（`ys_algo_spot_slope.c:58-79`）：

- 有 bin：`calibRotateAngle_GetMidSlotCode(handle, start, offset, code_delay=7)`
- 无 bin：`start + offset + 7 + 136*40`（`136*40 = 5440` 是固定补偿）

判面（`algo/ys_algo_frame_id.c:16-60`）：

```c
if (code_disc_angle >= g_standard_encoder[0] && < [1]) now = 0;
else if (... [1] .. [2]) now = 1;
else if (... [2] .. [3]) now = 2;
else                     now = 3;
if (|code_disc_angle - 上一值| > 6000) now = 上一帧的 index;   // 突变视为错误
if (slot_number == 0) now = (g_frame_index + 1) % 4;          // slot0 编码不准，用自增
```

### 9.2 beta 链条（同版本）

`ys_business/ys_algo.c:417-422` + `algo/ys_algo_horizontal_corners.c:111-127`：

```c
integer_scale_angle = g_encoder_angle[org_data->scale];               // 241 项表
hc = integer_scale_angle + (g_angle_speed * org_data->scale_time)     // 细分
     - g_staring_angle[frame_id];                                     // 减该面起始角
if (slot_number == 0) hc = 0;
hc = hc * 2 + cal_param.omega0.f;                                     // 镜面 ×2 + 零点
```

| 量 | 位置 | 值 |
|---|---|---|
| `MAX_ENCODER` | `algo/ys_algo_def.h:20` | beta = **61409** |
| `g_angle_speed` | `algo/ys_algo_def.h:30` | **0.0009 度/us** ← 这就是 `0.0009` 的出处 |
| `g_staring_angle[i]` | `ys_algo.c:269` | `(angle_face[i] + side_angular_offset[i]) / 256 * 360 / 240` |

### 9.3 差异对照

| 维度 | alpha | beta |
|---|---|---|
| 编码输入 | `code_disc_angle`（16 位绝对编码） | `scale` 查表 + `scale_time` |
| 细分角来源 | 编码盘直接给 | `0.0009 度/us × scale_time` |
| 满量程 | 65536 | 61409（此版本 beta 分支**未使用**它） |
| 判面依据 | `code_disc_angle` 四区间 + 突变保护 | `scale` 四区间 |
| 基准点语义 | 该面**中间 slot** 的编码 | 该面**起始角**（度） |
| 基准换算 | `get_mid_slot_code()`，可查 bin | `(angle_face + offset) / 256 × 360 / 240` |
| `spot_slope_map.bin` | **必经环节**（`GetMidSlotCode` + `DoCalib`） | **此版本未调用**（新版才加 `get_beta_spot_slope`） |
| `×2` 位置 | 在 `raw_diff` 换算时 | 在最后一步 |
| slot0 | `omega0`（−60） | `0 → 0×2 + omega0 = omega0` |
| 输出范围 | [−60, +60] | [−60, +60] |

### 9.4 对 PC 端的影响

- 我们 `Cs96BetaCalibration::apply()` 复刻的是 **beta 链**（`encoderAngle_[scale] + 0.0009×real − startingAngle_[face]`），
  与 alpha 的“编码盘差分 + 中间 slot 基准”不是一套东西；
- alpha 的 `0.0009` 是**角速度**（度/us）；beta 的 `0.0009` 是**12 位细分的每单位角度**，
  数值相同但**含义不同**，不要因为数字一样就当成同一条公式；
- alpha 把 `spot_slope_map.bin` 用在两处（求基准 `mid_slot_code`、逐点 `DoCalib`），
  而我们的 PC 端只用它做逐点修正，**缺少“用 bin 求面基准”这一半**；
- `MAX_ENCODER` 的 65536（alpha）/ 61409（beta）与导出格式的 **61440**
  都不是同一个数，三者混用会让满圈换算差千分之几。

> 待确认：`61409` 与 `61440` 相差 31，疑为笔误或实测标定值，源码无注释说明。

### 9.5 alpha 的面基准 `getMidSlotCode()` 详解

`g_omega0[face]` **不是**该面第一列（slot0）的编码，而是**该帧中点（约 slot 400）**的编码：

```cpp
uint16_t CalibRotateAngle::getMidSlotCode(uint16_t code_start, uint16_t code_offset,
                                          uint16_t code_delay) {
    float    code_spacing_per_slot = 13.6;   // 相邻 slot 的编码间隔
    uint16_t spot_spacing          = 10;     // 相邻光斑相隔几个 slot
    uint16_t spot_mid              = 40;     // 中间光斑是从左往右第几个
    uint16_t code_spacing_per_spot = code_spacing_per_slot * spot_spacing;  // 136
    return code_start + code_offset + code_delay + code_spacing_per_spot * spot_mid;
    //     3000      +  75         +  7         + 136 * 40 = 8522
}
```

常数逐一溯源：

| 常数 | 含义 | 换算 / 依据 |
|---|---|---|
| `13.6` | 每 slot 的编码增量 | `13.6/65536×360 = 0.0747°` 机械；×2 = **0.1494° 光路** |
| `spot_spacing = 10` | 相邻光斑隔 10 个 slot | 800 slot ÷ 80 光斑 = 10 |
| `spot_mid = 40` | 中间光斑序号（80 个光斑的中间） | `40 × 10 = 400 slot` = **半帧** |
| `code_delay = 7` | 光斑短半轴对应的旋转补偿 | `7/65536×360×2 ≈ 0.077°` |

**自洽性（这是最关键的一点）**：

```
spot_mid × spot_spacing = 40 × 10 = 400 slot = 半帧
136 × 40 = 5440 = 400 × 13.6     = 半帧的编码跨度
```

所以 `result = code_start + code_offset + 7 + 5440` 的物理含义就是
**「该面起点 + 半帧编码」= 该帧中点 slot 的编码值**。

**视场自检**：`800 × 13.6 = 10880` 编码，
`10880 / 65536 × 360 × 2 = 119.5°`，与单面 120° 吻合 ——
说明 `13.6` 是**设计常数**，不是实测拟合值。

**为什么拿中点当零点**：

```c
raw_diff = (int16_t)(code_disc_angle - g_omega0[now_frame_index]);
slot_angel = raw_diff * (360/65536) * 2;
```

以帧中点为 0，则 slot0 ≈ −5440、slot799 ≈ +5440，
光路角天然对称落在 **[−60, +60]**，不必再判别正负半圈。
配合 `±MAX_ENCODER/2` 的跨零处理，只在帧边界跨圈时才需要补偿。

**无 bin 时的兜底与此完全等价**：

```c
return code_start + code_offset + code_delay + 136 * 40;   // ys_algo_spot_slope.c:66
```

即"没有 `spot_slope_map.bin` 就用理论常数算中点"，
有 bin 时改由 `calibRotateAngle_GetMidSlotCode()` 查曲线求更准的中点。

> ⚠️ `13.6` 建立在 **alpha 的 `MAX_ENCODER = 65536`** 之上。
> beta 的编码基数是 61409/61440，实测每 slot 约 **12.47**（61440 基数，
> 见 §2.2），换算成机械角后两者仍差约 2%，
> **不能把 13.6 直接套到 beta**。
