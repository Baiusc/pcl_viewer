# Codex 任务交接：CS96 Beta 转镜雷达原始点云面拼接与面选择

## 0. 你的角色与目标

你是本仓库的编码代理。当前分支上已有一套可运行的点云查看器（`cloud_viewer`），
6532 固态雷达与 CS96 Beta 转镜雷达的**真实点云模式**均已打通。

现在需要你修复 CS96 Beta **原始点云模式（work_mode=1）** 的两个问题：

1. **原始点云异常旋转**：多帧画面在“转圈圈”，角度合成错误。
2. **面选择功能语义完全错误**：当前按 slot 分面，而正确语义是
   **1 面 = 1 帧 = 96×800 = 120° FOV**（车规级前向主雷达，不是环视，一帧不拼整圈）。

请先通读本文，再读代码，最后给出最小改动方案并实施。

---

## 1. 仓库与构建

- 仓库根目录：`/media/baiusc/DATA11/Repositories/ReposLidar/pcl_viewer`
- 主程序目标：`cloud_viewer`（Lite 版，非旧 `pcl_viewer` target）
- 构建：

```bash
cmake -S . -B build-lite -DCMAKE_BUILD_TYPE=Debug
cmake --build build-lite -j4
```

- 运行（交互）：

```bash
./build-lite/cloud_viewer
```

- 运行（无人值守截图，便于回归）：

```bash
CV_BETA_IP=10.42.0.218 CV_SCREENSHOT_WAIT_MS=10000 \
  ./build-lite/cloud_viewer --screenshot /tmp/beta.png
```

- 调试开关：
  - `CV_BETA_DEBUG=1`：每 5 秒输出 `[beta] recv/parsed/timeouts/frames` 统计。
  - `CV_BETA_DRAIN=1`：只收包不解析，用于定位是否为 socket 限流。
  - `CV_BETA_CALIB_DIR=<dir>`：覆盖标定目录。

### 技术栈与约束

- C++17，CMake 多 target：`cv_core` / `cv_io` / `cv_stream` / `cv_render` / `cv_app` / `cv_platform` / `cv_preprocess`。
- UI 为自绘 `UiOverlay`（VTK 2D Actor），**没有 Qt / ImGui**。按钮通过归一化视口坐标命中。
- 渲染为 VTK 9.4 直接渲染（PCL 只用了不依赖 Boost 的组件；`pcl_io` / `pcl_visualization` 在本机不可用，不要引入）。
- 点类型复用 `algo_lidar::PointData` / `PointMatrix`（`include/point_type.h`），**不要新建点结构**。

---

## 2. 设备与网络（重要，勿越界）

| 设备 | IP | 归属 | 说明 |
|---|---|---|---|
| CS96 Gamma / SL100（**当前主用**） | `10.42.0.238` | **用户自己的，可操作** | SN `CS96000C0126020001`，**120×800**，SSH `root/111` |
| CS96 Gamma（上一台，已离线） | `10.42.0.222` | 用户旧设备 | 已退场，不要再按 .222 调试 |
| CS96 Beta 转镜（旧机，已离线） | `10.42.0.218` | 用户旧设备 | 已断电退场，**不要再按 .218 调试** |
| CS96 Beta 转镜 | `10.42.0.221` | **用户自用，可操作** | 96×800，SN `CS96000B0126020001`，可切模式/改配置 |
| FL6532 固态 | `192.168.137.200` | 用户设备 | 走 UDP 6001→本机 6002，已正常工作，不要动 |

> 换雷达后 `~/.config/cloud_viewer/beta_known_hosts` 里的主机密钥会冲突。
> 现在 `runBetaSsh()` 会自动 `ssh-keygen -R` 该 IP 并以 `accept-new` 重试一次，
> 输出里会带 `[HOST-KEY] 设备主机密钥已变更`，不需要手工删 known_hosts。
> 你自己在终端 `ssh root@10.42.0.222` 时仍会看到旧的严格校验报错，那是 `~/.ssh/known_hosts`（另一份文件），
> 用 `ssh-keygen -f ~/.ssh/known_hosts -R 10.42.0.222` 清理即可。

CS96 设备侧关键信息：

- 配置文件：`/etc/ys_conf.ini`
- 启动程序：`/root/you_see`（**必须先 `cd /root`**，它靠当前目录找配置和动态库）

#### 手工 SSH 重启 `you_see`

该固件 `/etc/init.d/S99user` 的 `stop` / `restart` 都是空实现，
`/etc/init.d/S99user restart` 什么也不做，必须 `kill` 后手动启动：

```sh
# 1. 停（pidof 为空时不要直接 kill，否则 busybox 报用法错误）
pids=$(pidof you_see || true); [ -z "$pids" ] || kill -9 $pids
sleep 1

# 2. 起
setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &

# 3. 校验（起来了 ≠ 还活着）
sleep 3; pidof you_see || echo RESTART_FAILED

# 4. 看日志
tail -n 20 /tmp/you_stdout.log
```

四个不能省的细节：

| 细节 | 原因 |
|---|---|
| `setsid` | 不加则进程挂在当前 SSH 会话的进程组下，**退出 SSH 时被 SIGHUP 杀掉**，表现成"重启成功，一断线就没了" |
| `</dev/null >/dev/null 2>&1 &` | 不重定向时进程占着 stdin/stdout，`exit` 会卡住、SSH 会话关不掉 |
| `cd /root` | you_see 用相对路径找 `ys_conf.ini`、标定表、`.so`，换目录启动直接失败 |
| `sleep 1` 后再起 | 刚被 `kill -9` 时串口/采集句柄可能没释放，立刻启动常报设备忙并静默退出 |

**第 3 步校验不能省**：`work_mode` 与 `/mnt/mtdblock2/mode` 不匹配时，`you_see`
会启动后立刻退出（还可能回滚配置）。只有 `pidof you_see` 有输出才算真的起来了；
没有输出就看 `/tmp/you_stdout.log` 末尾和 `/var/log/ys.log` 末尾。

**改了 `/etc/ys_conf.ini` 必须重启 `you_see` 才生效**，并且注意：整机重启时
`app_startup.sh` 会用 `/mnt/mtdblock2/you_see_arm/config/ys_conf.ini` 覆盖 `/etc/ys_conf.ini`。
只改 `/etc/ys_conf.ini` 在断电重启后会被冲掉，要长期生效得改那个副本。

- 设备推流目标由 `center_ip` / `center_data_port` 决定，当前已配置为：

```ini
center_ip        = "10.42.0.229"   # 开发机
center_data_port = "50007"
work_mode        = "2"             # 0 直方图 / 1 原始点云 / 2 真实点云
```

- 设备日志：`/var/log/ys.log`（其中 `[FACE]` 行含实测每面起始角）。

---

## 3. 三种 ARM 输出模式（已实测确认）

### 3.00 型号与标定表

| 型号 | 线数 | 芯片 | `RadarModel` | 标定表 |
|---|---|---|---|---|
| CS96 Beta | 96×800 | IMX459 | `beta_rotating` | `data/param/cs96beta_calibration.csv` |
| CS120 Gamma | 120×800 | SL100 | `gamma_rotating` | `data/param/cs120gamma_calibration.csv` |

两者共用解析链路与 UI，型号按**广播/探测到的线数**判定（≥120 → Gamma）。
差异（直方图 30 包、`u=47`、`frame_id%4` 判面）见
[`cs96_gamma_sl100_protocol.md`](./cs96_gamma_sl100_protocol.md)。
**标定表不可混用**，连接时会按线数校验并拒绝不匹配的表。

### 3.0 硬约束：`work_mode` 必须与设备 `mode` 文件匹配

设备端 `/mnt/mtdblock2/mode` 决定开机 `insmod` 哪个驱动、用哪套 `liblidar.so`：

| `mode` 文件 | 驱动 | 允许的 `work_mode` |
|---|---|---|
| `histogram` | `histogram/yousee_lidar_histogram.ko` | **0**（原始直方图） |
| `point_cloud`（当前设备） | `point_cloud/yousee_lidar.ko` | **1 / 2** |
| `point_cloud_gray` | `point_cloud_gray/*.ko`（配套 `you_see_arm_gray`） | **4**（灰度） |

只改 `/etc/ys_conf.ini` 的 `work_mode` 而 `mode` 没换，`you_see` 会启动即退出：

```
ERROR ys_conf.c:407: 工作模式 0 与当前固件数据模式不匹配，请使用同一构建包中的配置
FATAL main.c:78: load config fail
```

`/etc/ys_conf.ini` 每次整机启动都会被 `app_startup.sh` 从 `you_see_arm/config/` 覆盖，
所以出厂默认值（`center_ip=10.42.0.145:50037`）会不断回来；软件每次连接都会自动纠正。

想永久保存上位机地址，改这里（jffs2，掉电保留）：

```sh
sed -i 's/^center_ip.*/center_ip = "10.42.0.229" ;上位机IP/' \
  /mnt/mtdblock2/you_see_arm/config/ys_conf.ini
```

芯片层背景（RAW/RANGING 与 work_mode 的对应、多回波、降采样、假峰）见
[`sl100_chip_notes.md`](./sl100_chip_notes.md)。


### 3.1 真实点云 work_mode=2（1266 B/包）

```
point_cloud_head_t (18B, 小端)
  uint8[10] delay          时间戳 us
  uint16    frame_index
  uint16    total_slots    固定 800
  uint16    slot_number    0~799
  uint8     echo_mode
  uint8     coordinate_mode 0=直角坐标
96 × cartesian_point_data_t (13B)
  float x, float y, float z, uint8 albedo
```

- 设备已算好 xyz，**PC 端不需要标定**。
- 当前真实点云显示正常、帧率稳定、无异常旋转，**不要改这条链路**。

### 3.2 原始点云 work_mode=1（1561 B/包）

```
OriginPointData (9B)
  uint8  header         0xAA
  uint16 frame_id       帧号（对 4 取模即面号）
  uint16 slot_number    0~799
  uint16 fragment
  uint16 data_length    固定 1552
slot (1552B)
  uint32 word0          mirror_angle = (word0 >> 12) & 0xFFFFF
  uint32 word1
  uint8  reserved[8]
  96 × 16B（像素倒序：数据区第 1 个像素是 pixel[95]）
     echo0: ft(3B) et(3B) pk(2B)
     echo1: ft(3B) et(3B) pk(2B)
```

- 无效点：`ft == 0` 或 `ft <= 16`。
- 两回波都有效时取 `pk` 更大的。
- 实测像素数据从包偏移 **25** 开始（9B 头 + 16B slot 头）。

### 3.3 直方图 work_mode=0（4105 B/包，24 包/帧）

```
uint8  sof 0xAA
uint16 frame_id
uint16 slot
uint16 seq          0~23
uint16 data_len     4096
payload: 4 像素 × 512 bins × uint16（每包 4 个像素，像素号 = seq*4 .. seq*4+3）
```

当前实现为“bin × 像素 × 强度”的地形点云，仅为可视化，不参与本轮修复。

---

## 4. 设备端角度解算（关键，以源码为准）

参考源码：`lidar_sdk/you_see_arm/ys_business/algo/ys_algo_horizontal_corners.c`
与 `lidar_sdk/you_see_arm/ys_business/decode/ys_decode_cs96_beta.c`。

设备端逻辑（**必须照抄**）：

```c
// 1) 每面起始角（ys_decode_cs96_beta.c）
g_staring_angle[i] =
    (float)(cal_param.angle_face[i] * 256 + cal_param.side_angular_offset[i]) / 256
    * 360 / CS96_BETA_PA_COUNTS_PER_REVOLUTION;

// 2) 水平角（ys_algo_horizontal_corners.c）
if (slot_number != 0) {
    int scale      = (mirror_angle >> 12) & 0xFF;
    int real_angle = mirror_angle & 0xFFF;
    angle = g_encoder_angle[scale] + 0.0009f * real_angle;
    horizontal_corner = angle - starting_angle;      // starting_angle = g_staring_angle[frame_id]
    // 归一化到 ±180
} else {
    horizontal_corner = omega0;                      // -60
}
horizontal_corner = horizontal_corner * 2 + omega0;  // omega0 = -60

// 3) 坐标（ys_decode_common.c，CS96_BETA 分支）
x = r * cos(theta) * cos(omega);
y = r * cos(theta) * sin(omega) * -1.0f;   // Beta 的 Y 轴取反
z = r * sin(theta);

// 4) 距离
r = velocitylight * (ft >> 1) * g_distance_factor[pixel] + g_delta_distance_index[pixel];
```

补充事实：

- `g_encoder_angle` 来自 `ys_encoder_angle.csv`（`scale,angle`，241 行，0~360），
  不是 `spot_slope_map.csv`。这张表来自**光学光条标定**；刻度时间测量（FPGA `(n,cnt)`）
  是独立验证手段，两者不能互相替代，详见 `doc/cs96beta_encoder_angle_table.md`。
- `frame_id` 就是面号（0~3），**每帧一个面**。
- 设备日志实测每面起始角（`start=`）：

```
旧版标定：face0=357.00  face1=87.43   face2=177.54  face3=267.03
新版标定：face0=33.13   face1=123.00  face2=213.18  face3=303.19
```

  两版相差约 36°（不是 90°），说明是重新标定的结果，不是换个模板。
  换 ini 后必须重新打包 CSV，否则 PC 端仍用旧角。

### 4.1 `ys_calibration.ini` 新旧版差异（2026-11）

`data/param/cs96beta_calib_data/` 下同时保留了 `ys_calibration.ini`（新）与 `ys_calibration旧版.ini`，
打包脚本只读前者。

| 项 | 旧版 | 新版 | 影响 |
|---|---|---|---|
| 值的写法 | `lines_v = 96` | `lines_v = "96"`（全部带双引号） | `stripComment()` 已去引号，两条加载路径都兼容 |
| `theta_tilt` | -2.5 | **0** | 俯仰角整体上移 2.5°，必须从标定读，不能写死 |
| `side*_angular_offset` | 0/74/93/5 | 23/0/31/32 | 面起始角重算 |
| `angle_face_*` | 238/58/118/178 | 22/82/142/202 | 同上 |
| 逐线数组 | 96 项 | 96 项（数值全新） | 水平角/距离偏移全部重标 |

适配要点：

- `dist_max` 现在参与有效点判定（`r < dist_max`），不再硬编码 200；
- `sat_peak_min` 同样从标定读（低于该峰值判无效），与设备端规则一致；
- 两条路径（聚合 CSV / ini 目录）都已验证可加载新版，起始角一致。

- 设备日志实测每面覆盖：`hc` 约 `0 ~ 59`，`×2 - 60` 后约 `-60 ~ 58`，
  即**每帧约 118° 视场**。面间机械间隔 90° 只是转镜的物理结构差，
  **不要加进水平角**：每个面各自由自己的 `starting_angle[face]` 起算，减去后回到同一段视场。
- `ys_calibration.ini` 中 `d0_delta...d95_delta` 单位是 **cm**（约 -5.87），
  换算要 `× 0.01`。

### 标定文件目录

`data/param/cs96beta_calib_data/`：

```
ys_calibration.ini        # omega_delta[96] / d_delta[96](cm) / a[96] / theta 参数 / standard_encoder
ys_encoder_angle.csv      # scale 0~240 -> 0~360°（整圈编码器表，实测生成，见 encoder_angle_table.md）
spot_slope_map.csv        # Face,Angle_FPGA,Diff（4 面 × 80 点，用于 spot slope 修正）
reflectivity_map.csv      # pixel_target,distance,reflectance_0..4
```

---

## 5. 现有实现与文件位置

| 功能 | 文件 |
|---|---|
| CS96 三模式收包 / 组帧 | `src/cloud_viewer/stream/BetaUdpStream.cpp`、`include/cloud_viewer/stream/BetaUdpStream.h` |
| 标定（距离/角度/反射率） | `src/cloud_viewer/preprocess/Cs96BetaCalibration.cpp`、对应头文件 |
| 应用层/按钮/HUD | `src/cloud_viewer/app/ViewerApp.cpp`、`include/cloud_viewer/app/ViewerApp.h` |
| 设备发现（55000 广播） | `src/cloud_viewer/stream/DeviceDiscovery.cpp` |
| 渲染桥 | `src/cloud_viewer/render/CloudRenderBridge.cpp`（`skipInvalid` 会跳过 `is_valid=false` 与非有限坐标） |

### 已能工作的部分（不要回归）

- `Search Device`：监听 UDP 55000 广播，列出 `CS96 Beta <ip> 96x800`。
- 点击设备按钮 → `connectBeta(ip)`，绑定本机 `50007` 收流。
- `Mode` 按钮：对 Beta 通过 SSH 修改 `/etc/ys_conf.ini` 的 `work_mode` 并重启设备，
  脚本回显 `SWITCH_OK` 判定成功；切换后设备重启，流自动恢复。
- 真实点云模式（1266B）显示正常。
- 6532 链路（UDP 6002）正常，未受影响。

### 当前的错误实现（必须改）

1. **面判定错误**：`parseCartesian` 用 `slot / 200` 分面，`parseOrigin` 用
   `faceByScale(mirrorAngle)` 在一帧内分面。正确语义是 **`face = frame_index % 4`**，
   一帧只属于一个面。
2. **面选择错误**：`Faces` 按钮按 slot 过滤，导致一帧被切成 4 段，与用户预期完全不符。
3. **原始点云角度**：`Cs96BetaCalibration::apply` 里 `startingAngle_` 当前取
   `encoderAngle_[face*60]`，与设备实测 `357/87.43/177.54/267.03` 有 1~3° 偏差，
   且面内角度与面间偏移的组合需要按第 4 节公式重算，否则表现为“多帧转圈”。
4. **每帧独立刷新**：原始点云 4 帧轮转显示，用户看到的是 4 个方向跳动，而不是一圈。

---

## 6. 需求（验收标准）

### 6.1 面语义修正

- `face = frame_index % 4`，一帧 800 slot 全部属于同一面。
- 删除按 `slot/200` 与 `faceByScale` 的一帧内分面逻辑。

### 6.2 单面流式输出（原「4 面累积合成」已废弃）

早期设计把 4 个面的缓存拼成一个超宽矩阵（列 = `face*800 + slot`）一次输出，
想做到"一屏看到 4 个面"。**这个设计是错的**：CS96 是车规级**前向主雷达**，
一帧就是 `96×800`、FOV 120°，不拼整圈。实测表现为墙壁变厚 / 同一面墙出现两三次：

- 设备本身就是一帧一个面（10 Hz，`frame_id` 每帧 +1 即换面）；
- `apply()` 输出的是**每面相对自己起始角**的水平角
  （`hc = encoderPart + realPart - startingAngle_[face]`），
  4 面拼在一起后全部落在同一个 ±60° 区间，同一面墙自然被画 4 次；
- 拼接后每帧点数是一帧的 4 倍，`render` 耗时约 36 ms。

现在改为**流式单面**（与真实点云一致）：

- 某个面凑齐 ≥90% slot 就立刻发布，矩阵为 `96 × 800`；
- 不再等 4 面齐，也**不再有限流**（单面重建成本只有 1/4）；
- `metadata.cols = 800`；
- 未选中的面只更新缓存、不发布；切换 Faces 时重发最近一面
  （靠 `faceMask != publishedMask` 触发，见 `setFaceMask`）。

**验收**：`Radar Live ... 显示点 76800`（即 96×800，一帧一面）；
`CV_FACE_TRACE=1` 时 `[face] publish` 严格按 `0→1→2→3` 轮转且帧号连续；
帧率 9.7~10.2 fps，`render` 约 15 ms。

代价：任一时刻只看到一个面的 120° 扇区（真实点云模式本来也是这样）。

### 6.3 Faces 按钮

- 选项循环保持：`All → F1 → F3 → F1+F3 → F2 → F4 → F2+F4 → All`。
- 语义改为“只显示选中的面（帧）”，不再是 slot 过滤。
- HUD 标签同步显示当前选择。
- 未连接 CS96 Beta 时点击应提示无效，不崩溃。

**验收**：选 `F1` 时只看到第一面的 90°~118° 扇区；选 `All` 时 4 个面**轮流刷新**
（每 100ms 换一个面），不是一屏拼成一圈。

### 6.4 角度与距离正确性

- 严格按第 4 节公式实现：
  - `starting_angle[face]` 必须与设备一致（优先从 `standard_encoder` + `side_angular_offset`
    按设备公式计算；若 `CS96_BETA_PA_COUNTS_PER_REVOLUTION` 无法确定，
    可在启动时从设备日志 `[FACE] ... start=` 读取并缓存，或先用 `encoderAngle_[face*60]` 近似但需在代码注释标明）。
  - `hc` 归一化 ±180 后再 `×2 + omega0`。
  - 面偏移 `+ face * 90`。
  - `y` 轴取反。
  - 距离 `d_delta` 按 cm 换算。
- 反射率保留现有 `reflectivity_map.csv` 查表插值实现。

**验收**：原始点云与真实点云（同一场景）整体轮廓、朝向、距离量级一致，偏差在可接受范围。

### 6.5 不许回归

- 真实点云模式（1266B）行为不变、帧率正常。
- 6532 链路不变。
- `Search Device`、`Mode` 切换、标定加载保持可用。
- `10.42.0.221` 是用户自用设备（早前"同事的设备"是误判），可正常操作。

---

## 7. 已知坑与调试建议

1. **设备进程容易没起来**：`Mode` 切换后务必确认 `ps | grep you_see`；
   若为空，用第 2 节的 `setsid` 命令手动拉起。
2. **切换脚本的 SSH 判定**：后台进程会拖住 SSH，成功与否以脚本回显 `SWITCH_OK` 为准，
   不要依赖 `pclose` 返回码。
3. **UDP 接收缓冲**：`net.core.rmem_max` 已调至 16 MB，
   `BetaUdpStream` 设置 `SO_RCVBUF = 8 MB`。若出现帧率骤降，先看
   `CV_BETA_DEBUG=1` 的 `recv/parsed/timeouts`。
4. **`PointData` 含 `std::vector<Pulse>`**：不要逐点 `point = PointData{}` 重建，
   会拖慢接收线程；应逐字段赋值并 `list_pulse.clear()`。
5. **组帧完整性**：每个面各自要求 ≥90% slot 才输出，凑齐即发布 —— 一帧就是一面 96×800。
6. **面内像素倒序**：原始点云数据区第 1 个像素是 `pixel[95]`，映射到 `line` 时要反转。
7. **在线点云拖动卡顿**：曾是三个来源叠加，已全部处理——
   - `SelectionStyle::OnMouseMove` 每次鼠标事件都调 `pumpLiveFrame()`，等于拖动过程中反复触发全量重建（已删除）；
   - 33ms 定时器在拖动期间仍在重建点云（`interacting_` 时暂停，`EndInteractionEvent` 补一帧）；
   - `PclVtkRenderer::upsert()` 每帧新建 vtkPoints/PolyData/Mapper 并逐点 `SetPoint`（改为复用整条管线，直接写底层 `vtkFloatArray` / `vtkUnsignedCharArray`，拓扑数组预生成后按点数截断）。
   验证方式：HUD 首行是渲染 FPS，拖动时应接近刷新率，松手后回落到点云帧率。
8. **原始点云帧率**：曾低到 0.2~5 fps，原因叠加：
   - **Debug 构建**（`build-lite`）下逐点 `apply()` 慢数倍，接收线程跟不上 → UDP 丢包 97%；
   - `rebuildDerivedFields` 每帧为 30 万点填 **13 个标量场**（400 万次写入 + 15MB 分配）
     → 改为按需只生成当前颜色源（`makeFrame(..., colorSource_)`）；
   - `applyIncomingFrame` 每帧 `renderer_.remove` + `scene_.add`，**id 变化导致渲染层缓存失效**、
     VTK 管线整条重建 → 改为复用同一个 `currentCloudId_`；
   - `publishOrigin` 限流 200ms，**正好把帧率锁死在 5 fps** → 改 100ms。

   之后又去掉了多面拼接（见 6.2）：那阶段一帧点数是一帧的 4 倍、`render` 36 ms；
   改成单面 `96×800 = 76800` 点后 `render` 降到 15 ms 左右，限流也随之删除。

   实测（Release，单面流式）：`prep 5~6ms + color 8~9ms + render 15~27ms`，
   稳定 **9.7~10.2 fps**、`gap_max 120~190ms`。Debug 也能到 10fps，但余量小。

   判定方法见 `AGENTS.md` 第 4 节：用 `CV_LIVE_FPS=1` 连续跑，看窗口帧率与 `gap_max`，
   不要只看首帧截图。

## 离线 TXT（原始点云导出）解析

`cs96_raw_save` 导出的单帧文件长这样：

```
cols: 800  rows: 96              ← cols = slot 数，rows = 像素数
start of col [N]                 ← 注意是 col，不是 slot
rotate mirror angle: <1/61440 圈> ← 绝对值，不是增量
pixel_number: <行号>  tof w pk  tof w pk  tof w pk   ← tof/width 单位 ps
```

**同时满足这三条才能解析，缺任何一条都表现为「加载成功但 0 个点」**：

| 要点 | 说明 |
|---|---|
| `RotatingTxtLoader` 必须排在 `SolidTxtLoader` 之前 | `Loaders.cpp` 的 `push_back` 顺序就是 `selectLoader` 的优先级；固态解析器只认 `row:` 前缀，抢到就返回空矩阵 |
| 数据行要兼容 `pixel_number:` 前缀 | 裸 9 数字是另一种格式，两者都从 `tokens[off]` 起跳 |
| `rotate mirror angle` 是绝对值 | `raw / 61440 * 360` 度；走旧的 1/65536 **增量累加**分支会一路跑飞 |

矩阵尺寸从头部 `cols/rows` 读，不要写死 96×800（120 线会整片错位）。

**水平角换算**（完整推导见 `cs96beta_pa_angle_analysis.md` 第 2 / 2.1 / 2.2 节）：

```
base = colRaw[1] − (colRaw[2] − colRaw[1])   // 本帧 slot0 实测角
hc   = rawDeg − base → wrap 到 ±180           // 必须先相减再 wrap
hc   = hc × 2 − 60                            // 镜面 ×2，omega0 = −60
```

三个最容易漏的点：

- **减本帧起始角**：`rotate mirror angle` 是整圈绝对位置，每帧起点都不同
  （实测 213.6° / 123.5° / 33.3°，相差正好 90° 机械）。不减它，相邻帧会差 180°，
  表现就是「点云整体跑到原点后方」；
- **×2**：镜面反射把转角放大一倍。漏掉后 120° 视场缩成 60°；
- **omega0 = −60**：整机零点偏移，缺了整体偏 60°。

`61440` = 240 格/圈 × 256，是导出时的文本量化基数，1 单位 = 0.005859°，
**不是物理量**；旧 `angle:` 格式用的 1/65536 圈是另一个基数，混用会差 6.25%。

第 0 列常出现 `raw = 0` 的脏值（导出时 scale 越界取了角度表首项），
用第 1、2 列的增量补回去，否则会多出一条 0° 方向的假竖线。

**帧 id 必须全进程唯一**：

各 loader 原来都写 `descriptor.id = 1`，而 `ViewerApp` 的帧缓存以 id 为 key，
于是所有文件都命中第一帧 —— 表现为「日志显示切换了、画面纹丝不动」。
现在统一由 `nextFrameId()` 分配。

**换文件后要 `resetCamera()`**：不同文件的点云位置可能完全不重叠，
不重算包围盒会让新点云整体落在视锥外，看起来像「翻页后一片空白」。

**自检**：每个文件应得 `valid = 76800/76800`（96×800）。

**浏览**：`Prev/Next` 越过文件边界会自动切到相邻文件（这些 txt 每个只有 1 帧，
只在文件内取模会表现为"点了完全没反应"，日志一直是 `帧 0 -> 0 / 1`）。
跨文件时日志会写成 `_1.txt -> _2.txt`，`Tab` 仍然直接跳下一个文件。

## 标定表的加载与生效（两个坑）

都会造成"看起来加载了标定，实际没生效"：

1. **连接时加载失败会静默降级**。默认路径曾写死成相对 cwd 的
   `data/param/cs96beta_calibration.csv`；从 IDE / 启动器 / 其它目录启动时 cwd 不是项目根，
   报「无法打开」→ `betaCalibration_.reset()` → 传给流的指针是 `nullptr` →
   `parseOrigin` 里判面和逐线修正全部跳过，**退化成 `applyFallbackNoTable`**
   （线性 `-60 + 120×slot/slots`）。现在按 exe 目录优先多候选查找。

2. **手动 `Select Calib` 不会同步给正在运行的流**。流持有的是**连接那一刻**的指针，
   只改 `betaCalibration_` 等价于只骗过了界面。现在换表后会重新
   `setCalibration()` + `setCalibrationEnabled()`。

判断是否真的生效，看 HUD 的 `Calib:` 行：`ON 96x800` / `OFF` / `未加载（纯几何兜底）`。

### 一键更新聚合表（Update Calib 按钮）

选一个目录标定包 → 解析 → 写回 `<exe>/calib/cs96beta_calibration.csv` →
**回读校验通过才启用**，同时推给正在运行的解码线程。

| 文件 | 必需 | 说明 |
|---|---|---|
| `ys_calibration.ini` | ✅ | 参数、4 面角、逐线水平角/距离修正 |
| `ys_encoder_angle.csv` | ✅ | scale 0~240 → 机械角 |
| `spot_slope_map.csv` 或 `angle_calib_YYYYMMDD.csv` | 可选 | 两种命名都认；`spot_slope_map.bin` 暂不解析 |
| `reflectivity_map.csv` | 可选 | 缺失时**沿用旧表** |

**覆盖策略**：以现有聚合表为基底，目录里没有的 section（反射率、角度曲线）
不会被清空，而是继承旧值——否则一次"更新"就会把 391 行反射率查表整段抹掉。

验证过的一组真实数据：`20261104角标` 目录 → 导出 324 行 `spot_slope`
（来自 `angle_calib_20261106.csv`）+ 继承 391 行 `reflectivity`，
4 个面起始角往返误差 0.000000°。

> `options`（App Cal 开关）与 `calibration` 指针是**两条独立通道**：
> options 全 true 只表示"修正项开着"，指针为空时代码仍走兜底解算。

## `ParsedPacket` 是复用的（调试陷阱）

`ParsedPacket packet;` 在接收线程里只构造一次，`points[]` 是定长数组且**从不清空**。
在 `parseAny` 断点里看到的 96 个点是**上一包**的残留，判断本包是否解析要看
`pointCount`（失败时置 0）和 `mode`（每次解析前置 -1），不要看 `points` 内容。

---

## 8. 交付要求

- 只改必要文件，优先修改：
  - `src/cloud_viewer/stream/BetaUdpStream.{h,cpp}`
  - `src/cloud_viewer/preprocess/Cs96BetaCalibration.{h,cpp}`
  - `src/cloud_viewer/app/ViewerApp.{h,cpp}`（按钮语义与 HUD）
- 不要重构无关模块，不要改 6532 相关代码。
- 提交前保证：

```bash
cmake --build build-lite -j4
./build-lite/tests/cv_test_core
./build-lite/tests/cv_test_beta_stream
```

均通过，并用真实设备截图验证：

```bash
CV_BETA_IP=10.42.0.218 CV_SCREENSHOT_WAIT_MS=12000 \
  ./build-lite/cloud_viewer --screenshot /tmp/beta_after.png
```

- 在回复中说明：
  1. 面判定与单面流式输出（一帧 `96×800`）的实现方式；
  2. `starting_angle` 的取值来源与偏差；
  3. 截图对比（修复前 / 修复后）；
  4. 仍未解决的风险点。
