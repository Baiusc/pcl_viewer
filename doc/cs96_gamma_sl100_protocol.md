# CS96 Gamma（SL100，120×800）协议与解算差异

> 参考实现：`lidar_sdk/yscloudscene` 分支 `develop/mf150_raw_point_cloud`（下称 SC）、
> 板端固件 `lidar_sdk/you_see_arm`（下称 ARM）。
> 本文只写**与本仓库（96 线 Beta）不同**的部分，相同部分见 `codex_cs96beta_handoff.md`。

## 0. 型号定位（先纠正命名）

| 商品名 | 芯片 | 线数 × slots | 硬件版本位 | 广播 `line_counts` |
|---|---|---|---|---|
| CS96 Beta | **IMX459** | 96 × 800 | SN 首位段含 `B` | 96 |
| **CS96 Gamma / cs120 Gamma** | **SL100** | **120 × 800** | SN 含 `C`（`CS96000C…`，未试产，命名待定） | **120** |

两台机器在 `LidarModel` 里都是 `KCS`（`LidarModel::KCS` 涵盖 alpha/beta/gamma），
**不能用 model 区分**，只能看 `hardware_version`、SN、或包长/点数。

> 关键：本文所有结论只对 `line_counts=120` 的机器成立。
> 若广播报 96 而实际是 120 像素，包长校验会直接失败——先确认广播字段，再怀疑解析。

## 1. 三种模式的包格式

| work_mode | 含义 | UDP 包长 | 内容 | PC 是否算 xyz |
|---|---|---|---|---|
| 1 | 原始点云 | **1945** = 9 + 1936 | 16B slot 头 + 120×16B 原始码值 | **是**（PC 自己算） |
| 2 | 真实点云 | **1578**（单回波）/ **3138**（双回波） | 18B 头 + 120×13B 直角坐标 | 否（板端已算好） |
| 0 / 3 | 直方图 | **4105** = 9 + 4096 | 4 像素 × 512 bin | — |

对比 96 线 Beta：1561 / 1266 / 2514 / 4105。

## 2. 原始点云（1945B）

### 2.1 包头 9B

```
uint8  header      0xAA
uint16 frame_id    16 位，需处理 65535→0 回绕
uint16 slot        0~799
uint16 seq         驱动批次下标，组帧时忽略
uint16 data_length 固定 1936（= 16 + 120×16）
```

### 2.2 slot 头 16B

```
word0 (u32)  slot_number = word0 & 0xFFF
             mirror_angle = (word0 >> 12) & 0xFFFFF
+4  (u24)    relative_time_us          ← 24 位，比 Beta 的 10B 时间戳短
+14 (u8)     fpga_frame_id；face = fpga_frame_id % 4
```

`mirror_angle` 再拆：

```
scale      = (mirror_angle >> 12) & 0xFF    // 编码器刻度，一圈 240
real_angle = mirror_angle & 0xFFF           // 刻度内细分，0.0009°/step
```

### 2.3 像素区 120×16B

```
每个像素 16B，两个回波：
  echo0: ft(u24) et(u24) pk(u16)   // 偏移 0,3,6
  echo1: ft(u24) et(u24) pk(u16)   // 偏移 8,11,14
```

- `ft` 单位 0.5 ns，`ft == 0 || ft <= 0x10` 视为无效；
- 两回波都有效时取 `pk` 更大者（板端为 `peak_rank_mode=00` 幅值排序）；
- **数据区第 i 个 16B 单元 = 物理像素 `line_counts - 1 - i`**，即倒序（`data[0]` 是 pixel[119]）。

## 3. 坐标解算（与 Beta 完全相同，只有参数不同）

板端 `ys_decode_common.c` 与 Gamma/Beta 共用同一函数：

```c
x = r * cos(theta) * cos(omega);
y = r * cos(theta) * sin(omega) * -1.0f;   // ← Gamma 与 Beta 都取负，Alpha 不取
z = r * sin(theta);
```

> 注意 y 取负这件事在 SC 里有两份实现：`Cs96BetaOriginPoint2xyz` 取负（正确），
> 通用 `OriginToPointMatrix` 不取负（给 Alpha 用）。抄代码时容易抄错。

水平角（`ys_algo_horizontal_corners.c`，`#elif CS96_BETA || CS96_GAMMA` 共用）：

```c
if (slot != 0) {
    angle = encoder_angle[scale] + 0.0009f * real_angle;
    hc = wrap180(angle - starting_angle[face]);
} else {
    hc = omega0;                        // -60
}
hc = hc * 2 + omega0;                   // 转镜 1:2 反射关系
omega = (hc + omega_delta[pixel]) * degree;
```

俯仰角（差异所在）：

```
step   = (theta_max - theta_min) / lines_v      // 20 / 120 = 0.16667°
theta[j] = ((lines_v - 1 - j) - u) * step + theta_tilt
        = ((119 - j) - 47) * 0.16667 - 2.5
```

- Beta：`lines_v=96`，`step=20/96`，`u=47`（正好 `(96-1)/2`）。
- Gamma：`lines_v=120`，`step=20/120`，**`u` 仍是 47**（未按 120 重标，属待确认项）。

所以不能简单套 `(line - (lines-1)/2) * 20/lines - 2.5`：对 120 线会得到 `u = 59`，
与板端差 `(59-47) × 0.1667 ≈ 2°`，整幅点云会整体上仰。

距离：

```
r = 2.99792458e-4 * (ft >> 1) * a[pixel] + d_delta[pixel]     // 单位 m
```

`a[]` 与 `d_delta[]` 都是 **120 项**。

## 4. 面号：Gamma 不做 PA 纠偏

| | Beta | Gamma |
|---|---|---|
| 原始点云面号 | PA 编码器区间匹配 + 统计纠偏（`beta_get_pa_frame_index`） | **直接 `fpga_frame_id % 4`** |
| 实测问题 | FPGA 帧号 99.9% 错位，必须纠偏 | 未提供纠偏，若帧号也错位则面序不可信 |

板端代码：

```c
int now_frame_index = cs96_gamma_get_fpga_frame_id(org_data) % 4;
```

所以 PC 端若沿用 Beta 的 PA 判面逻辑，需要 Gamma 自己的 `angle_face` / 侧向偏移参数；
沿用 Beta 的表会整体偏 90° 量级。**这是接 120 线时第二个易错点。**

## 5. 直方图（work_mode=0/3）

```
UDP 4105B = 9B 头 + 4096B 载荷
9B 头: 0xAA, frame_id(u16), slot(u16)=恒 0, seq(u16), data_len(u16)=4096
载荷: 4 像素 × 512 bin × u16（小端）
seq 0~29，第 seq 包携带像素 4*seq .. 4*seq+3
```

- **1 个 SLOT 就是 1 帧**，`slot_number` 恒为 0（Beta 是 56 SLOT 拼一帧）；
- 共 **30 包/帧**（Beta 是 24 包）；
- FPGA 侧总量 `122912 = 32 + 120×512×2`。

## 6. 标定文件

板端用 `conf/ys_calibration_gamma.ini`（刷机后落地为 `/etc/ys_calibration.ini`，同名）：

| key | Gamma 值 | Beta 值 |
|---|---|---|
| `[parameter] lines_v` | **120** | 96 |
| `lines_h` | 800 | 800 |
| `omega0` | -60.0 | 同 |
| `theta_min / theta_max / theta_tilt` | -10 / 10 / -2.5 | 同 |
| `u` | **47**（待按 120 重标） | 47 |
| `[horizontal_angular_offset] omega_delta0..119` | **120 项** | 96 项 |
| `[zero_distance_offset] d_delta0..119` | **120 项** | 96 项 |
| `[distance_factor] a0..119` | **120 项** | 96 项 |
| `[standard_encoder] angle_face_1..4` | 3000/19384/35768/52152 | 同结构 |
| `[vcsel_index] pixel_in_vcsel_id0..3` | 29/59/89/**119** | 23/47/71/95 |

> 逐像素零距偏移的 key 写作 **`d_delta<idx>`**。SC 里有一处拼成了 `d<idx>_delta`，
> 导致该表永远读默认值——接手别人的解析代码时先核对 key。
> 编码器 CSV（`ys_encoder_angle.csv`）两机通用，转镜未换。

## 7. cloud_viewer 的适配实现（已完成）

### 7.1 型号体系

`RadarModel` 新增 `GammaRotating`（`gamma_rotating`），与 alpha/beta 共用解析与 UI：

```cpp
inline bool isRotatingModel(RadarModel model);   // alpha / beta / gamma
```

型号按**设备规格**判定（probe 时广播可能缺失，SN 为空）：

| 线数 | 型号 | 依据 |
|---|---|---|
| ≥120 | `GammaRotating` | SL100 只有 120×800 |
| 96 | `BetaRotating` | IMX459 只有 96×800 |

广播 SN 也参与列表分类：`CS96000C…`（版本位 C）→ 显示 `CS120 Gamma`。

### 7.2 三处真实差异及实现

| 差异 | Beta（96） | Gamma（120） | 实现 |
|---|---|---|---|
| 直方图分片 | 24 包/帧 | **30** 包/帧 | `histogramPacketCount(lines) = lines/4` |
| 俯仰中心线 u | 47（=`(96-1)/2`） | **47**（≠59） | `Config::pitchCenter` ← 标定 `parameter,u` |
| 面号来源 | PA 判面（帧号 99.9% 错位） | `frame_id % 4`（与板端一致） | `Config::faceFromFrameId` |

俯仰角与板端逐行一致：`theta[j] = ((lines-1-j) - u) * (theta_max-theta_min)/lines + theta_tilt`。
**Gamma 的 `theta_tilt = 0`**（Beta 是 -2.5），必须从标定读，不能写死。

### 7.3 标定表（已入库）

```
data/param/cs120gamma_calib_data/      # 从设备拉取的 ys_calibration.ini + ys_encoder_angle.csv
data/param/cs120gamma_calibration.csv  # 563 行，lines_v=120，start=33/123/213/303
```

重新生成：

```bash
python3 tools/pack_cs96beta_calibration.py \
  data/param/cs120gamma_calib_data data/param/cs120gamma_calibration.csv
```

连接时按规格自动选表（120 线 → gamma 表，96 线 → beta 表）。

⚠️ **设备 ini 的逐线数组只有 96 项**（0~95），但 `lines_v=120`。
加载时按 `lines_v` resize，96~119 用默认值（a=1、d=0、omega_delta=0），
即**后 24 线目前没有逐线修正**，需要厂商补 120 项标定。

### 7.4 单位约定（重要，别再搞反）

两类产品的单位体系不同，混用会直接让整帧点云失效：

| 产品 | 标定表距离 | 点云坐标 | 典型量程 |
|---|---|---|---|
| **转镜**（Alpha / Beta / Gamma） | **米** | 米 | 200 m |
| **固态**（6532 / 6031） | **厘米** | 厘米 | < 50 m |

原因：转镜雷达是汽车前向主雷达（有效测距 200 m），固态雷达多用于近距离场景。
换算只在固态侧做，转镜侧**不做任何换算**。

| 位置 | 源单位 | 处理 |
|---|---|---|
| `Cs96BetaCalibration` 的 `d_delta` / `a` | 米 | 直接相加 |
| 反射率表 `distance` 列（实测 4.504 / 5.006） | 米 | 与 `point.r` 直接比较 |
| 6532 SoC 点云 x/y/z | 厘米 | ×0.002（raw → cm → m） |
| 6532 距离 / legacy 标定 | 厘米 | ÷100 |
| 6532 ROI0 逐像素表 | 厘米 | 字段 `distance_correction_cm`，÷100 后再加 |

> 曾经把转镜的 `d_delta = -5.3` 当厘米换算成米，结果 0.5 m 的目标算成 -4.8 m，
> 整帧被判为无效（"显示点 384000，有效 0"）。该换算已回滚——**转镜就是米**。

### 7.5 实测结论（10.42.0.238，光路遮挡下）

- 包长 1945B、slot 0~799、scale 227 等头部字段全部正常；
- 每帧约 1 个有效回波（黑胶带内部反射），距离 0.48~0.52m，与遮挡物位置吻合；
- `d_delta = -5.3 m` 会把 0.5m 的内部反射补偿成负距离，所以**遮挡时有效点为 0 是正常的**；
  对着 3m 以外的真实目标才会有稳定点云；
- 即解析与解算链路正确，**"有效点很少"是该场景的必然结果**，不是 bug。

## 8. 实测：设备没推流时怎么判定（2026-11-04，10.42.0.238）

"上位机在监听但没点云"不一定是解析问题。按下面顺序，几分钟就能区分**设备没产数据**还是**解析错**：

| 判据 | 正常值 | 本次实测 |
|---|---|---|
| 设备网卡 TX 增量 | 20 fps × 800 × 1578B ≈ **25 MB/s** | **2.7 KB/s**（只有广播） |
| 本机 bind 50007 收包 | 每帧 800 包 | **12 秒 0 包** |
| 设备日志 | 持续有输出 | 停在 `set broadcast mode on`，之后无新增 |
| producer `[HDR]` 日志 | 启动时打印前 4 个 slot 头 | **0 条** → `lidar_get_bufaddr()` 一直返回 NULL |
| `/dev/lidar` | 可读 1936B/slot | 阻塞不可读 |

即：**驱动没有从 FPGA 取到任何 SLOT**，与包长、坐标、标定无关。

### 8.1 这台样机的实测状态

| 项 | 值 | 判断 |
|---|---|---|
| 驱动 | `yousee_lidar mode:pcl_sl100`（`lsmod` 已加载） | 正常 |
| `you_see` | 运行中，`Switch state INIT to SCAN success` | 状态机正常 |
| 电机 | `demo_i2c_motor` → `speed:147.11` | 已下发 |
| SPAD PWM | `/sys/class/pwm/pwmchip0/pwm0/enable = 0` | **可疑** |
| GPIO | `gpio-382 gpiod_request: status -16 (EBUSY)` | 可疑 |
| 配置 | `work_mode=2`、`echo_mode=0`、`coordinate_mode=0` | 正常 |
| 库 | `/usr/lib/liblidar.so` 与 `point_cloud/` 的 md5 一致 | 正常 |

`app_startup.sh` 里这一行是**注释状态**：

```sh
#echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable
```

手动 `echo 1 > .../enable` 再重启 `you_see`，`[HDR]` 仍为 0、TX 无变化。
所以 PWM 不是唯一原因，SPAD / FPGA 的使能链路需要固件或硬件同事确认。

### 8.2 两个每次重启都会复现的坑

1. **`center_ip` 被覆盖回出厂值**：实测重启后变成 `10.42.0.166`（不是上一台机器的地址）。
   `install.sh` 每次开机执行 `cp config/* /etc/`；`cloud_viewer` 连接时会自动改回并重启 `you_see`。
   要永久保存就改 `/mnt/mtdblock2/you_see_arm/config/ys_conf.ini`。
2. **SSH 主机密钥每次重启都变**（系统镜像重建密钥），两个文件都要清：
   `ssh-keygen -R 10.42.0.238 -f ~/.ssh/known_hosts` 与
   `ssh-keygen -R 10.42.0.238 -f ~/.config/cloud_viewer/beta_known_hosts`。

### 8.3 结论

120×800 的解析链路（包长 1578/1945、像素倒序 `119-i`、坐标公式、标定 120 项）已就绪；
当前看不到点云是**设备端 FPGA 没有产出 SLOT**，属固件/硬件层，上位机侧无法绕过。

## 9. 已知问题（SC 侧实测记录）

- `SC/docs/cs96_gamma_pointcloud_diagnostics.md`：曾出现真实点云 **96000 点全为 XYZ=0**，
  当时结论是板端输出零值、PC 解析正常。排查时先 dump 一包 1578B 看浮点是否非零。
- Gamma 的 1945B 原始点云在 SC 里**只做保存、不算坐标**，所以 SC 本身没有可抄的 120 线
  xyz 实现，只能按本文第 3 节自己写。
- 广播里的 `line_counts` 与实际像素数不一致时，包长校验会失败而不是误解析——这点是安全的，
  但表现为"能搜到、连不上/没数据"，容易被误判成网络问题。
