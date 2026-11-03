# Codex 任务交接：CS96 Beta 转镜雷达原始点云面拼接与面选择

## 0. 你的角色与目标

你是本仓库的编码代理。当前分支上已有一套可运行的点云查看器（`cloud_viewer`），
6532 固态雷达与 CS96 Beta 转镜雷达的**真实点云模式**均已打通。

现在需要你修复 CS96 Beta **原始点云模式（work_mode=1）** 的两个问题：

1. **原始点云异常旋转**：多帧画面在“转圈圈”，角度合成错误。
2. **面选择功能语义完全错误**：当前按 slot 分面，而正确语义是 **1 面 = 1 帧，4 帧 = 转镜 360°**。

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
| CS96 Beta 转镜 | `10.42.0.218` | **用户自己的，可操作** | SSH `root/111` |
| CS96 Beta 转镜 | `10.42.0.221` | **同事的，严禁操作** | 不要 SSH、不要改配置、不要重启 |
| FL6532 固态 | `192.168.137.200` | 用户设备 | 走 UDP 6001→本机 6002，已正常工作，不要动 |

CS96 设备侧关键信息：

- 配置文件：`/etc/ys_conf.ini`
- 启动程序：`/root/you_see`
- 该固件 `S99user` 的 `stop/restart` 是空实现，必须 `kill` 后手动启动：

```sh
kill -9 $(ps | grep you_see | grep -v grep | awk '{print $1}')
setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &
```

- 设备推流目标由 `center_ip` / `center_data_port` 决定，当前已配置为：

```ini
center_ip        = "10.42.0.229"   # 开发机
center_data_port = "50007"
work_mode        = "2"             # 0 直方图 / 1 原始点云 / 2 真实点云
```

- 设备日志：`/var/log/ys.log`（其中 `[FACE]` 行含实测每面起始角）。

---

## 3. 三种 ARM 输出模式（已实测确认）

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
  不是 `spot_slope_map.csv`。
- `frame_id` 就是面号（0~3），**每帧一个面**。
- 设备日志实测每面起始角（`start=`）：

```
frame_id=0  start=357.00
frame_id=1  start=87.43
frame_id=2  start=177.54
frame_id=3  start=267.03
```

- 设备日志实测每面覆盖：`hc` 约 `0 ~ 59`，`×2 - 60` 后约 `-60 ~ 58`，
  即每面约 118° 视场，面间机械间隔 90°，4 面重叠拼成 360°。
- `ys_calibration.ini` 中 `d0_delta...d95_delta` 单位是 **cm**（约 -5.87），
  换算要 `× 0.01`。

### 标定文件目录

`data/param/cs96beta_calib_data/`：

```
ys_calibration.ini        # omega_delta[96] / d_delta[96](cm) / a[96] / theta 参数 / standard_encoder
ys_encoder_angle.csv      # scale 0~240 -> 0~360°（整圈编码器表）
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

### 6.2 4 面累积合成

- 维护 4 个面的帧缓存（每面 96×800）。
- 合成的输出矩阵为 `96 × 3200`（列 = `face*800 + slot`），
  4 面拼成完整 360°。
- 只在选中面全部就绪后输出；未就绪/未选中的面列填 `NaN`（渲染桥会跳过）。
- 输出需同步设置 `metadata.rows = 96`、`metadata.cols = 3200`，
  否则着色重排会错位。
- 合成频率可限制为“每次面缓存更新后”，若 10 fps 下开销过大，可降为每 4 帧合成一次。

**验收**：原始点云画面不再出现 4 方向跳跃/转圈，4 面拼成稳定一圈。

### 6.3 Faces 按钮

- 选项循环保持：`All → F1 → F3 → F1+F3 → F2 → F4 → F2+F4 → All`。
- 语义改为“只显示选中的面（帧）”，不再是 slot 过滤。
- HUD 标签同步显示当前选择。
- 未连接 CS96 Beta 时点击应提示无效，不崩溃。

**验收**：选 `F1` 时只看到第一面的 90°~118° 扇区，画面稳定；选 `All` 时看到完整一圈。

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
- 不要操作 `10.42.0.221`。

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
5. **组帧完整性**：当前要求 ≥90% slot 才输出；4 面合成时应分别判断每面。
6. **面内像素倒序**：原始点云数据区第 1 个像素是 `pixel[95]`，映射到 `line` 时要反转。

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
  1. 面判定与 4 面合成的新实现方式；
  2. `starting_angle` 的取值来源与偏差；
  3. 截图对比（修复前 / 修复后）；
  4. 仍未解决的风险点。
