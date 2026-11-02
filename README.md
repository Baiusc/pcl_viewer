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

## 与设计文档的偏差记录

1. **PCL 组件**：当前环境 `libpcl_visualization` / `libpcl_io` 依赖已不存在的 Boost 1.71，
   因此渲染改用 VTK 9.4，PCD 解析自含实现（ASCII/binary），未使用 `pcl::io::loadPCDFile`。
2. **选点**：使用 `vtkPointPicker`（6 px 容差）+ `displayToSource` 回溯；尚未实现设计文档中的
   KDTree 分段射线加速。10 万点级别可交互，超大点云需后续优化。
3. **UI**：按设计不引入 Qt/ImGui，使用 VTK 2D `vtkTextActor` 面板与键盘交互；
   文本使用 ASCII，避免中文字体打包问题。
4. **标定**：默认不自动应用；`Legacy6532Calibration` 需显式配置 `calib_data.bin` 才处理 raw 数据。
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
