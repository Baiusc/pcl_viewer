# cloud_viewer Lite

激光雷达点云可视化 PC 软件 Lite 版（`pcl_viewer` 仓库 `cloud_viewer` 分支）。

- 目标平台：Ubuntu 20.04+ / Windows 10+（x64）
- 技术基线：C++17、CMake 3.15+、PCL（点类型/KDTree）、VTK 9（渲染）
- 设计文档：`doc/codex_design.md`、`doc/codex_workbuddy_prompts.md`

## 功能

| 功能 | 状态 |
|---|---|
| 离线加载：转镜 TXT（alpha/beta） | 完成 |
| 离线加载：固态 TXT | 完成 |
| 离线加载：FL6532 Frame CSV（按列名、多帧） | 完成 |
| 离线加载：FL6031 Frame CSV（单算法） | 完成 |
| 离线加载：PCD（ASCII / binary） | 完成 |
| 多文件加载、帧切换、帧缓存 | 完成 |
| 点云渲染、相机、坐标轴 | 完成 |
| 按属性着色（14 种来源） | 完成 |
| 选点信息（完整字段回溯） | 完成 |
| 在线 Mock 推流（`mock_replay`） | 完成 |
| 真实雷达 SDK 接入 | 未接入（需厂商 SDK） |
| 6532 legacy 标定模块 | 接口完成，默认不自动应用 |
| CS96 Beta 转镜实时点云（UDP 50007，96×800） | 完成 |
| CS96 Beta 角度标定（码盘耗时 + 光条表 + 聚合表） | 完成 |
| CS96 Beta 设备搜索 / 连接 / 工作模式切换（SSH） | 完成 |

## 目录结构

```text
include/
  point_type.h                    # 复用：PointData / PointMatrix / Pulse / Converter
  cloud_viewer/
    core/CloudFrame.h             # 帧模型与字段可用性
    io/LoaderTypes.h Loaders.h    # 加载器接口与注册表
    render/CloudRenderBridge.h    # PointMatrix -> 显示云 + 原始索引映射
    render/PclVtkRenderer.h       # VTK Actor 生命周期 / 相机 / 拾取
    color/ColorMapper.h           # 标量场 -> RGB
    scene/SceneGraph.h            # 场景实体树
    stream/RadarStream.h          # 在线后端注册
    preprocess/FramePreprocessor.h# 可选标定模块
    app/ViewerApp.h               # 应用控制器
src/
  cloud_viewer_main.cpp           # Lite 入口
  cloud_viewer/                   # 与 include/cloud_viewer 对应实现
  point_type.cpp                  # 复用：Converter 与标定实现
  pulse_data_processor.cpp        # 复用：转镜 / 固态 TXT 解析
  main.cpp 等                     # 旧调试入口，默认不参与构建
tests/                            # core / io / color 测试
```

**旧调试程序默认不构建**：`CMakeLists.txt` 使用显式源文件列表，不再对 `src/` 做递归 GLOB。
需要时用 `-DCLOUD_VIEWER_BUILD_LEGACY=ON` 单独构建 `PCL_VIEWER_LEGACY`。

## 构建

### Ubuntu 20.04+

```bash
sudo apt install build-essential cmake libeigen3-dev
# PCL / VTK 需按系统版本安装；本仓库在 PCL 1.14 + VTK 9.4 下验证。
cmake -S . -B build-lite -DCMAKE_BUILD_TYPE=Release
cmake --build build-lite -j
ctest --test-dir build-lite --output-on-failure
```

依赖处理策略（`cmake/ViewerDependencies.cmake`）：

1. **标准模式**：Boost `system/iostreams/filesystem/serialization` 齐全时，使用 `find_package(PCL)`
   并启用 PCL IO / Visualization。
2. **手动模式（自动降级）**：Boost 组件不完整时，只链接不依赖 Boost 动态库的
   `pcl_common / pcl_octree / pcl_kdtree / pcl_search / pcl_filters`，渲染由 VTK 直接承担。
   当前验证环境即走此模式。

无论哪种模式，渲染统一由 VTK 完成，PCL 只提供点类型与 KDTree。

### Windows 10+

1. 安装 Visual Studio 2022（x64）。
2. 安装 PCL 1.14 与 VTK 9，或在 vcpkg 中安装 `pcl[visualization]`。
3. 配置：

```bat
cmake -S . -B build-lite -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_PREFIX_PATH="<PCL_ROOT>;<VTK_ROOT>"
cmake --build build-lite --config Release
```

## 运行

### 默认启动（推荐）

```bash
cloud_viewer
```

启动即进入主界面，通过顶部工具栏按钮操作：

| 按钮 | 功能 |
|---|---|
| **Open Files** | 弹出原生文件对话框，多选 CSV / TXT / PCD 导入 |
| **Open Folder** | 选择目录，批量导入该目录下 csv/txt/pcd（非递归、按文件名排序） |
| **Prev / Next** | 上一帧 / 下一帧 |
| **Color: xxx** | 切换着色源 |
| **Reset** | 视角复位 |
| **Hide** | 显示/隐藏当前点云 |

导入后自动建立帧索引、解码首帧、复位相机并刷新 HUD。可反复点击 **Open Files** 重新导入。

原生对话框后端：Linux 使用 `zenity`（或 `kdialog`），Windows 使用 `comdlg32`。
Linux 若缺少 `zenity`，点击按钮会在控制台提示，可用 `sudo apt install zenity` 安装。

### 命令行（自动化 / CI，可选）

```bash
# 格式嗅探 + 帧索引报告（不打开窗口）
cloud_viewer --inspect <file> --report report.json

# 直接导入指定文件（等价于点击 Open Files 后选择）
cloud_viewer --open <file1> [file2 ...]

# Mock 在线推流（20 Hz）
cloud_viewer --source mock_replay --open <file> --fps 20

# 启动并保存界面截图
cloud_viewer --screenshot out.png
cloud_viewer --open <file> --screenshot out.png

# 真实 SDK 未编译时的状态（返回 13，不伪装成功）
cloud_viewer --source alpha_sdk --open <file>
```

### 交互键位

| 操作 | 功能 |
|---|---|
| 左键拖动 / 中键拖动 / 滚轮 | 旋转 / 平移 / 缩放 |
| `Shift` + 左键 | 选点，右侧面板显示原始字段 |
| `←` / `→` | 上一帧 / 下一帧 |
| `Tab` | 切换文件 |
| `C` | 循环切换着色源 |
| `N` | 显示/隐藏当前点云 |
| `R` | 视角复位 |
| `+` / `-` | 点大小 1~10 px |
| `Space` | 冻结/恢复（Mock 模式） |
| `Esc` / `Q` | 退出 |

### 着色源

`r / intensity / reflect / peak / x / y / z / theta / slot_ang / 回波序号 /
is_tail_pt / is_valid / timestamp / 固定色`。

连续值低蓝高红；回波序号、布尔字段使用分类色；缺失值使用中性灰。

## 在线雷达连接（6532）

直接运行 `./build-lite/cloud_viewer`，通过第二排按钮操作，无需命令行参数。

| 按钮 | 行为 |
|---|---|
| Connect Radar | 按当前选择监听 UDP；Disconnect 停止并清空接收缓存 |
| Mode: Cloud / Range | 向设备 TCP 6508 发送切换命令；ACK 成功后重建接收器，失败显示原因 |
| ROI: 0 / 1 | 选择显示 ROI；两种 ROI 独立组帧，切换不混帧，不修改设备 ROI 配置 |
| Load Calib BIN | 选择本机 `calib_data.bin`（文件对话框选“所有文件”）；加载失败保留旧表 |
| App Cal | Range 下循环 OFF → TableB → Tail3LnFixed；Cloud 下显示 N/A |

**Cloud 与 Range 的“关闭标定”含义不同：**

- Cloud (`0x21`) 是设备算法生成的 XYZ；APP 默认原样显示。当前固件代码固定
  `calib_version=2`，把表传给 `Init_Algo_Env`、`Algo_GetRawCloudData`。
  公开控制协议没有距离标定开关；温漂和电压校准开关不是距离标定开关。
  因而 `device_unknown` 表示 APP 未核实设备内部最终校正状态，不表示“设备未标定”。
  Cloud 不携带 `conv_intensity`，也没有可逆的原始测距，因此不套用本地强度模型、不伪造设备标定 OFF。
- Range 是原始记录经本机 `RangePipeline` 重建的点云。OFF 表示不做本地距离补偿；
  TableB 只加逐像素偏置；Tail3LnFixed 复用现有候选模型：
  `q=selected_conv_intensity/table_a; x=ln(q)+log_intensity_bias; r'=r+table_b/100+P(x)/100`。
  `log_intensity_bias` 默认 12.9，是已有模型的额外系统参数，不是从 bin 读出的数值；该模型不是已经证实的原厂公式。
- 每次切换本地标定都从不可变原始帧复制计算，不累加补偿。离线点云不受这些按钮影响。
  比较原始/本地校正效果时选择 Range；Cloud 的设备端距离标定 ON/OFF 仍需固件提供明确接口。

### 数据链与协议边界

```text
Cloud UDP 0x21 → 按 ROI/帧号缓存 → 唯一像素覆盖完整 → PointMatrix → 可视化
Range UDP 0x2E → 严格 40B 头校验 → 278B 记录 → RangePipeline → 原始帧
Range LZ4 v1 → 分包组装 → LZ4F 解压 → 各 push 独立逆转置 → RangePipeline
原始帧 → OFF / TableB / Tail3LnFixed → 可视化
```

- Cloud 保留原有单位缩放；ROI0 为 128×192，ROI1 为 48×192，物理行 16..63。
  乱序帧尾不触发残帧显示，重复像素不重复计数，缓存最多 8 帧。原有通用 `0x01` 点云仍兼容。
- Range API 来自同级 `../algo_solid_lidar`，仅编译 protocol/pipeline 两个源文件，
  不引入其 main 或离线标定器；可用 CMake `CLOUD_VIEWER_RANGE_ROOT` 更换位置。
  ROI1 由连续四个批次的 subframe 0..3 组成完整帧，接收和显示频率可能低于 ROI0。
- LZ4 分包按 `(frame_id, sub_frame_idx)` 缓存，缺包不提交，限制缓存和解压长度。
  当前接入版本为 14B 分包头 + 首包 72B 元数据（16B 帧描述 + 4×14B ROI 描述）。
  该元数据布局已通过合成用例，本轮尚未用设备 LZ4 抓包确认；布局不符会显示错误并拒绝处理。
- TCP 控制采用 SDK `open_common.h` / `open_ctrl_cmd_set_work_mode`：
  `0x7788` 协议头、`0x99AA` 控制头、cmd=2，Cloud=0x21、Range=0x2A。
  后台线程等待 ACK，UI 不阻塞；ACK 仅表示设备接受命令，收到完整帧才证明数据链打通。

### 网络与配置

设备单播目标通常为 `192.168.137.100`；运行现有网络脚本前确认该地址没有被其他主机占用：

```bash
bash tools/connect_radar.sh enp4s0
```

设备启动入口是 `/root/Linux_pcl/fsLAA.sh`；已有 `soc_example` 运行时不重复启动。
APP 不保存设备密码、不自动通过 SSH 重启设备。

| 环境变量 | 默认 | 说明 |
|---|---|---|
| CV_RADAR_IP | 192.168.137.200 | 雷达地址，也是 UDP 源过滤条件 |
| CV_RADAR_PORT | 6001 | UDP 源端口；同时兼容 6301 |
| CV_LOCAL_PORT | 6002 | 本机 UDP 接收端口 |
| CV_CONTROL_PORT | 6508 | 公开 TCP 控制端口 |
| CV_STREAM_MODE | cloud | 设为 range 可只选择接收解析器，不发送设备切换命令 |
| CV_CALIB_BIN | 空 | 可选本机标定文件路径；加载后仍默认关闭本地标定 |

默认不自动连接；`./build-lite/cloud_viewer --source udp` 可启动后直接监听。
若设备输出与所选模式不同，接收器不把另一种模式当作当前模式解析。

### 构建与自测

新增依赖是 LZ4 开发库（Linux 通常为 `liblz4-dev`），同时保留原有 PCL/VTK 依赖。

```bash
cmake -S . -B build-lite
cmake --build build-lite -j2
ctest --test-dir build-lite --output-on-failure
```

`online_range_and_calibration` 自造 ROI0/ROI1 原始记录、LZ4 分包和标定表，
覆盖乱序、缺包、Cloud UDP 回环、TCP 分片 ACK，以及本地标定 OFF/ON/OFF 无累加。
本轮构建和 4 项 CTest 通过；雷达连接返回 `No route to host`，设备切换 ACK、实机 Range/LZ4
显示及墙面标定效果仍待实机联调。本地测试不作为真实设备效果的证明。

## CS96 Beta 转镜雷达（96×800）

与 6532 固态雷达并列的另一条链路，走 `BetaUdpStream`（UDP 50007）。

| 项 | 说明 |
|---|---|
| 设备发现 | UDP 55000 广播 + ARP/MAC 扫描；已知 IP 会先 probe |
| 输出模式 | `work_mode` 0 直方图 / 1 原始点云（1561B 包）/ 2 真实点云（1266B 包，设备已算好 xyz） |
| 一帧 | **96×800 = 76800 点，FOV 120°**，一帧一个面，4 面轮转（**不是** 360° 环视） |
| 坐标 | 真实点云用设备 xyz（右手系）；原始点云水平角取反，呈现左手系（slot0 最左、slot799 最右） |
| 标定 | `Select Calib` 载入聚合 CSV；`Update Calib` 用目录散表重算聚合表（写回所选目录并同步到 `build/calib`） |
| 帧率自检 | `CV_LIVE_FPS=1` 每 2 秒输出窗口帧率与 `gap_max`；`CV_FACE_TRACE=1` 打印面号轮转 |

### 角度标定流程（四步）

```text
① FPGA 测每格耗时          → time_intervals.txt（240 个 us） (20261117 此步为可选项 实际可以不做此步 取1701固定值)
② 按累计时间比例分 360°    → ys_encoder_angle.csv（初版）→ 写入雷达 ARM flash
③ 光条 CV 标定             → ys_encoder_angle.csv（终版，更精准）
④ omega = 终版粗刻度表 + 每格耗时折算细角度 − 面起始角，再 ×2 + omega0
```

第 ④ 步的面起始角来自 `ys_calibration.ini`：`angle_face_N`（粗刻度，1 格 = 1.5°）
与 `side_angular_offset`（1/256 格 ≈ 0.0059°）。

#### 码盘表的两种布局（20261117 起自动识别）

标定端（`LightSpotSlope` / `measure_light_spot`）新增 `anchor_per_face` 选项，产出两种表：

| 布局 | 表值范围 | 本面起始角取法 |
|---|---|---|
| **绝对表**（旧，`anchor_per_face=false`） | `0~360°` 单调递增 | `startingAngles()` = `angle_face_N × 1.5° + side_angular_offset × 1.5°/256` |
| **每面锚定表**（新，`anchor_per_face=true`） | 每个面段各自从 `-30°` 起算，故**含负数**、第 240 行回绕行 `> 360°` | 直接取该面起始刻度在表里的值（细调已折进表值） |

`Cs96BetaCalibration::load()` 按表值范围自动判定：出现负数或 `> 360°` 即认定为每面锚定表
（`anchorPerFace() == true`），也提供 `setAnchorPerFace()` 手动覆盖。
两种布局下 `hc` 的物理含义一致（相对本面起爆点的机械转角），因此 `omega = hc×2 + omega0` 不变；
判面用的 `paAngle()` 在每面锚定表下会自动加回 `dialBase[face]` 还原成码盘绝对角。
界面日志（`Calib` / `Select Calib` / `Update Calib`）会打印 `编码器表=每面锚定(-30°)` 或 `绝对角`。

**注意**：第 ② 步时间换算出的表只是**初版**，上线前必须过第 ③ 步光条修正；
两者实测最大差 0.62°，直接用初版覆盖会把机械-光学差灌进水平角。
完整原理、实测数据与代码位置见 `doc/cs96beta_encoder_angle_table.md`。

### 标定文件

`data/param/cs96beta_calib_data/`：

| 文件 | 用途 |
|---|---|
| `ys_calibration.ini` | `u` / `omega0` / `theta` / `dist_max` / 逐线 `omega_delta`、`d_delta`、`a`；四面 `angle_face_N`、`side_angular_offset` |
| `ys_encoder_angle.csv` | **终版**粗刻度角度表（scale 0~240，241 行） |
| `time_intervals.txt` | 每格实测耗时（us，240 个，末行常无换行） |
| `spot_slope_map.csv` | FPGA 旋转角修正曲线（可选） |
| `reflectivity_map.csv` | 反射率查表（可选） |
| `cs96beta_calibration.csv` | 以上聚合成的单文件表，`Select Calib` 用它 |

### 相关环境变量

| 变量 | 说明 |
|---|---|
| `CV_BETA_IP` | 目标雷达 IP |
| `CV_BETA_PORT` | 本机接收端口（默认 50007） |
| `CV_BETA_WORK_MODE` | 连接前把设备切到 0/1/2 |
| `CV_BETA_CALIB` | 指定标定表路径，覆盖默认查找顺序 |
| `CV_LIVE_FPS` | 每 2 秒输出帧率 / `gap_max` / 分阶段耗时 |
| `CV_FACE_TRACE` | 打印发布的面号，验证四面轮转 |
| `CV_ANGLE_PROBE` | 打印首末 slot 的水平角与坐标，验证左手系方向 |

## 与设计文档的偏差记录

1. **PCL 组件**：当前环境 `libpcl_visualization` / `libpcl_io` 依赖已不存在的 Boost 1.71，
   因此渲染改用 VTK 9.4，PCD 解析自含实现（ASCII/binary），未使用 `pcl::io::loadPCDFile`。
2. **选点**：使用 `vtkPointPicker`（6 px 容差）+ `displayToSource` 回溯；尚未实现设计文档中的
   KDTree 分段射线加速。10 万点级别可交互，超大点云需后续优化。
3. **UI**：按设计不引入 Qt/ImGui，使用 VTK 2D `vtkTextActor` 面板与键盘交互；
   文本使用 ASCII，避免中文字体打包问题。
4. **标定**：默认不自动应用；`Legacy6532Calibration` 需显式配置 `calib_data.bin` 才处理 raw 数据。
   CS96 Beta 是另一套（聚合 CSV + 码盘耗时表），见上文「CS96 Beta 转镜雷达」。
5. **LOD**：`RenderRequest.maxPoints` 已支持确定性抽样，但尚未接入相机运动触发。

## 样本验证

| 样本 | 结果 |
|---|---|
| `data/Frame1_ROI0_distRP.csv` | 128×192，24576 行，24565 有效 |
| `data/小测试间点云/011小实验室_对准2块反射板.txt` | 96×800，73063 有效 |
| `data/0811固态雷达6532.pcd` | 24576 点解码成功 |

## 遗留代码

`python/`（标定分析脚本）、`matlab/`、`data/`（测试数据）与旧调试入口不属于 Lite 产品组成，
默认不参与构建。是否移出仓库请按团队数据管理策略决定，本次未做物理删除。
