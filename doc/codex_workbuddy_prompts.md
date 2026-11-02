# pcl_viewer「cloud_viewer 分支」双 Agent 协作提示词

本文件是为 `pcl_viewer` 仓库在 **`cloud_viewer` 分支**开发「激光雷达点云可视化 PC 软件 Lite 版」准备的
**两份可直接交付给编码代理的提示词**：

- **Part A**：给 **Codex** 的提示词 —— 负责出方案、大纲、系统设计（不写代码）。
- **Part B**：给 **WorkBuddy** 的提示词 —— 根据 Codex 的方案输出，做实操写代码。

**协作流程**：

```
用户/整理人  ──►  Codex(方案设计)  ──►  方案文档(system design)
                                          │
                                          ▼
用户审阅(可选调整)  ──►  WorkBuddy(编码实现)  ──►  可编译运行的代码 + 验收自测
```

> 说明：本文件由人工整理，已把仓库现状、复用点、冗余清单、数据格式等事实写进提示词，
> 使两个 agent 无需重新探索仓库即可对齐上下文。下文「仓库现状速览」是两份提示词的公共上下文，
> 建议复制到 Codex / WorkBuddy 的会话开头一并贴入。

---

## 〇、仓库现状速览（公共上下文）

### 0.1 现有目录结构

```
pcl_viewer/
├── CMakeLists.txt          # CMake 3.15+，find_package(PCL/VTK)，GLOB 源文件
├── cmake/                  # CMakeSetCompilerOptions 等 5 个模块
├── include/                # 11 个 .h + 1 个 .hpp（核心在 point_type.h）
├── src/                    # 8 个 .cpp（main.cpp 是调试入口）
├── python/                 # 12 个 analyze_*.py + os_script.py（标定分析脚本）
├── data/                   # 大量测试点云/标定数据（txt/csv/pcd/bin）
├── doc/                    # 空目录
└── matlab/                 # 空目录
```

### 0.2 必须复用的点云结构体（`include/point_type.h`，命名空间 `algo_lidar`）

| 符号 | 说明 |
|---|---|
| `PointData` | **核心点类型**：`pixel_idx`(行)、`slot_idx`(列)、`peak`、`r`(距离)、`theta_rad`/`omega_rad`/`theta`(俯仰角°)/`slot_ang`(水平角°)、`reflect`(反射率0~1)、`timestamp`、`intensity`(0~255)、`x/y/z`、`is_valid`、`list_pulse`(3 组回波，按 peak 降序)、`select_idx`(0/1/2 最强/次强/最弱回波)、`is_tail_pt` |
| `PolarPoint` | 前端极坐标点（r/theta_rad/omega_rad/intensity/peak） |
| `CartesianPoint` | 前端笛卡尔点（x/y/z/intensity/peak） |
| `Pulse` | 单次回波（tof/width/peak） |
| `PointMatrix` | `std::vector<std::vector<PointData>>`（一帧的结构化矩阵） |
| `Converter` | `polar2Mat`/`mat2Cartesian`/`polar2Cartesian`/`mat2Polar`/`buildPolarPt`/`createPolarPt` |

`point_type.h` 还含大量**标定/预处理函数**（`applyROffset`/`applySaOffset`/`applyRsaOffset`/`buildMatByPulseIdx`/`flipMat`/`transMat`/`save_mat_to_json`/`load_mat_by_json`/`extractPoints`/`extractData` 等）以及 **6532 标定结构体/函数**（`Range6532*` 系列，约 200 行）。**这些是标定算法，不是可视化必需，属于裁剪候选。**

### 0.3 现有数据读取函数（`include/pulse_data_processor.h` + `include/debug_visual.h`）

| 函数 | 用途 | 支持的雷达 |
|---|---|---|
| `readPulseData` | 转镜雷达原始 txt（`start of [slot]` + `angle` + 每像素 3 组 tof/width/peak） | alpha/beta 转镜 |
| `readSolidLidarData` | 固态雷达原始 txt（头 `cols: 360 rows: 152` + `start of col [N], angle` + `row: N tof1 width1 peak1 ...`） | 固态 |
| `readLidarData` | 自动识别上两种 txt 格式 | 通用 |
| `readFrameCsv` | 6532 离线重建 CSV（表头含 `roi/行row/列col/p0_dist_rp_raw`） | FL6532 |
| `readFrameCsv_6031` | FL6031 单算法 CSV（表头含 `center_bin/timing_bin/algorithm/echo_bins`） | FL6031 |
| `readPcdData` | 读 PCD → PointMatrix | 通用 |
| `buildMatFromPcd` / `buildMatFromPcdI` | PCD 点云 → PointMatrix | 通用 |

### 0.4 现有可视化与选点

- `include/debug_visual.h`：`mat2pcl`/`mat2xyzi`（PointMatrix→PCL 云，`m_intensity_src_name` 全局变量决定强度来源）、
  `showCloud`/`showCloud2`/`showCloud3`/`showCloud4`/`showCloud2in1port`/`showListCloudRGB`/`showCluster`/`registerMultiPtPicking`。
- `include/PclHelperB.h`（`PclHelperB` 类）：`hsv2rgb`、`initViewer`、`addViewport`、`addLine`、
  `addCloud*`（自定义 hue / 按轴着色 / 强度着色 `addCloudI` / 自定义区间着色）、`addClusters` 等**可视化方法**，
  以及**大量算法方法**（聚类/投影/平面拟合/区域生长/法向等，属算法测试残留）。
- `include/point_selection.h`：模板 `CallbackArgs<T>` + `choosePointCallback`，已实现射线求交选点，
  但 `getNearestPointOfRay` 是 **O(n) 全遍历**，需改用 KDTree 加速；已能区分多点云（`list_cloud`/`clicked_cloud_idx`）。

### 0.5 数据格式样例（用于离线解析设计）

- **转镜 txt**（`readPulseData`）：96 行 × 800 列，文本块结构 `start of [slot]` → `angle ...` → 逐像素 `tof1 width1 peak1 tof2 width2 peak2 tof3 width3 peak3`。
- **固态 txt**（`readSolidLidarData`）：头 `cols: 360  rows: 152`、`angle_per_col:`，`start of col [N], angle: A` → `row: N tof1 width1 peak1 ...`。
- **6532 Frame CSV**（`data/Frame1_ROI0_distRP.csv` 表头）：
  `frame,roi,row,col,slot_ang,theta,dist_m,peak,reflect,is_valid,p0_*...,p1_*...`（128×192 ROI，range 模式）。
- **6031 Frame CSV**：含 `center_bin/timing_bin/algorithm/echo_bins`，算法由 `algorithm` 列标识。
- 另有 PCD（`0811固态雷达6532.pcd`）、JSON（`save_mat_to_json`/`load_mat_by_json`）。

### 0.6 冗余 / 无关内容清单（清理依据）

- **可删除/移出仓库**：
  - `python/` 下 12 个 `analyze_*.py` + `os_script.py`（标定/拖尾点分析脚本，非可视化软件组成）。
  - `matlab/`、`doc/` 空目录。
- **可裁剪代码（算法残留，可视化非必需）**：
  - `PclHelperB.h` 中除可视化外的全部算法模板（`regionGrowingSegmentation`/`euclideanClustering`/`fastEuclideanClustering`/`projectCloudToPlane`/`sacComputeNormal`/`getNearestPlanePoint*`/`findIndices*`/`findCluster*`/`removeModelOutlier`/`computeNormals` 等）。
  - 标定/反射率/地面修复/拖尾滤波：`point_type.h` 的 `Range6532*` 系列、`calc_intensity.h`(`CalibReflect`)、`binary_storage.h`、`algo_interface.h`(`Ground_I`/`Timer`)、`tail_point_filter.h`、`fast_euclidean_clustering.h`、`Spline.h`。
  - `src/main.cpp` 内大量注释掉的调试调用 + 硬编码绝对路径（`/mnt/windows_share/...`）。
- **注意**：`readFrameCsv`（6532）依赖 `applyRange6532LegacyCalibration` 才能得到正确 XYZ；强度着色可能依赖反射率标定。
  → 方案需明确：**标定作为「可选预处理插件/模块」保留接口，不作为可视化核心的一部分**。

---

## Part A —— 给 Codex 的提示词（方案 / 大纲 / 系统设计）

> 以下方框内为可直接复制给 Codex 的提示词正文。

```text
你是一名资深的点云可视化系统架构师。请为「pcl_viewer」仓库在新分支 cloud_viewer 上开发
「激光雷达点云可视化 PC 软件 Lite 版」产出**完整方案、大纲与系统设计文档**。
本轮你只做设计，**不要写任何代码，不要修改任何文件**，输出一份可直接交付给编码代理（WorkBuddy）
据此实施的 markdown 设计文档。

# 目标平台与语言
- Ubuntu 20.04 及以上；Windows 10 及以上。跨平台。
- 开发语言不限但跨平台友好；现有代码为 C++17 + CMake 3.15 + PCL + VTK，优先沿用。

# 功能需求
1. 离线点云可视化：读取、解析、可视化雷达设备离线保存的单帧/多帧点云的 csv、txt 格式的一个或多个文件。
2. 在线点云可视化：根据不同型号雷达的 SDK 进行开发接入。
3. 点云可视化、按属性着色、选点信息显示。

# 支持的雷达型号
- alpha 转镜雷达（已知 96 行 × 800 列）
- beta 薄款转镜雷达（同为转镜，格式与 alpha 一致）
- 面阵全固态雷达：FL6031（直方图/多算法 CSV）、FL6532（128×192 ROI，range 模式 CSV + calib_data.bin）

# 硬性约束（必须遵守）
1. **最大化复用现有代码**，尤其是点云结构体 `algo_lidar::PointData`、`PointMatrix`、`PolarPoint`、
   `CartesianPoint`、`Pulse` 与 `Converter`，不要另起炉灶重造结构体。
2. 可视化通用底层**可参考 cloud_compare 开源项目**的架构思想（实体树/标量场/渲染器分离/交互拾取），
   但**实现仍基于现有 PCL + VTK 技术栈**，不引入 cloud_compare 本身（其 Qt 依赖过重，与 Lite 定位不符）。
3. 明确**代码清理范围**：列出删除/移出/裁剪清单与清理原则（见下方仓库现状）。
4. 方案要**足够具体、可执行**，让编码代理无需二次决策即可按任务落地。

# 仓库现状（请直接采信，不必重新探索）
[此处粘贴「〇、仓库现状速览」的 0.1~0.6 全文]

# 你需要产出的设计文档，必须包含以下章节（每章都要具体到可编码）

## 1. 系统总体架构与模块划分
- 分层架构图（表示层 / 可视化底层 / 数据层 / 设备接入层）。
- 每个模块的职责、对外接口（类名+关键方法签名）、模块间依赖关系。
- 明确哪些模块直接复用现有代码（给出复用点），哪些模块新建。

## 2. 跨平台技术选型与依赖管理策略
- 语言/标准：C++17；构建：CMake 3.15+。
- 点云库 PCL、可视化 VTK 的获取方式：
  - Ubuntu：apt 安装（`libpcl-dev` 等），给出精确包名与最低版本。
  - Windows：给出 vcpkg / conan 或官方安装包方案，并说明 CMake 的 `find_package` 兼容写法。
- 是否需要引入 ImGui（菜单/属性面板）还是用 PCL 自带 VTK 交互；给出取舍与理由。
- 文件选择对话框、UTF-8 路径、跨平台路径处理、`_WIN32` 条件编译策略。
- 依赖的版本兼容矩阵与 CI 建议（可选）。

## 3. 点云结构体与数据流的复用设计
- 保留 `PointData`/`PointMatrix`/`Pulse`/`Converter`，说明如何把它们作为**统一内存表示**。
- 设计一层「帧容器」抽象（如 `CloudFrame`：一帧 = PointMatrix + 元数据(雷达型号/帧号/时间戳/原始文件路径) +
  着色源），并定义「多帧容器」（如 `CloudFrameSequence`）支持多文件/多帧。
- 定义「点云 → PCL 云」的转换边界：现有 `mat2pcl`/`mat2xyzi` 如何演进为带属性的通用转换
  （属性字段与 PCL 字段的映射表）。
- 数据流：`文件/SDK → 解析器 → PointData/PointMatrix → CloudFrame → 可视化渲染`。

## 4. 可视化通用底层设计（参考 cloud_compare）
- 参考 cloud_compare 的以下思想并映射到 PCL/VTK：
  - 场景实体树（`ccHObject` 树）→ 设计 `SceneObject`/`SceneGraph`。
  - 标量场（`ccScalarField`，每个点一个标量，驱动着色）→ 设计「着色源/属性字段」抽象。
  - 渲染器与数据分离 → 设计 `Renderer`/`RenderableCloud` 接口。
  - 相机控制、包围盒/网格/坐标轴、点大小、背景色、视角复位等交互项清单。
- 给出核心类的类图/接口定义（名称 + 关键方法 + 数据成员）。

## 5. 离线点云读取解析与可视化实现规划
- 统一入口设计：根据文件后缀 + 表头嗅探（沿用现有 `isFrameCsv6031`/`isFrameCsv6532`/`readLidarData` 判别逻辑）
  自动路由到对应解析器。
- 明确支持格式：转镜 txt、固态 txt、6532 Frame CSV、6031 Frame CSV、PCD；单帧/多帧；单文件/多文件。
- 多文件加载的交互（文件多选/目录扫描）、加载进度、内存占用与超大点云的 LOD/抽稀策略。
- 解析器接口（`IPointCloudLoader`）与各格式实现类划分。

## 6. 在线点云可视化接入方案（按 SDK）
- 设计设备接入抽象层：`IRadarStream`/`RadarDevice`/`RadarBackendFactory`，统一「帧回调」接口。
- SDK 适配器（`SdkAdapter`）与上述统一帧容器的对接方式；线程模型（采集线程/渲染线程解耦、环形缓冲）。
- 三类雷达（alpha 转镜、beta 转镜、FL6031、FL6532）各对应哪个 SDK/驱动、如何注册进工厂。
- 在 SDK 未就绪前用 **mock/stub 数据源**联调的策略。
- 在线/离线共用同一可视化管线的设计（数据源可切换）。

## 7. 点云按属性着色的设计
- 着色源枚举（至少含：`r`、`x`、`y`、`z`、`intensity`、`reflect`、`peak`、`timestamp`、
  `theta`、`slot_ang`、回波序号、簇 id、`is_tail_pt`、`is_valid`）。
- 着色映射：连续值归一化（现有 `addCloudI` 的 HSV 蓝→红）+ 离散高对比（现有离散色板）+ 自定义 min/max 区间。
- 复用 `PclHelperB::hsv2rgb` 与 `addCloudI` 的着色逻辑，抽象为「ColorMapper」。
- 色标（colorbar）显示设计。

## 8. 选点信息交互设计
- 复用 `point_selection.h` 的 `CallbackArgs` 与射线求交思路，但**将 O(n) 遍历改为 KDTree 加速**。
- 选中点高亮 + 信息面板展示字段（xyz、r、theta/slot_ang、intensity/reflect/peak、3 组回波 tof/width/peak、
  pixel/slot 索引、时间戳、所属帧/文件、簇 id）。
- 多帧/多文件下「点归属」的定位（选中点映射回具体 CloudFrame 与原始文件）。

## 9. 三类雷达适配设计
- 逐一说明 alpha 转镜、beta 转镜、FL6031、FL6532 在：解析格式、坐标计算（角度/距离→xyz 的差异）、
  标定依赖、着色默认值、在线 SDK 接入点 五个维度的差异与统一抽象。
- 明确「标定」作为可选预处理插件的边界（尤其 6532 依赖 calib_data.bin）。

## 10. 代码清理范围（删除清单与原则）
- 给出：删除文件清单、移出仓库清单、裁剪(可编译但不再参与构建/或下沉为可选插件)清单、保留清单。
- 清理原则：可视化核心只保留「结构体 + 读取解析 + 着色 + 选点 + 渲染」；算法类/标定类代码
  要么删除、要么下沉为独立可选模块（默认不参与 Lite 构建）。

## 11. 开发里程碑与分阶段任务拆解（含验收标准）
- 按可独立交付拆成 8~10 个里程碑（M0~Mn），每个里程碑列出：目标、任务清单、涉及文件、**验收标准**。
- 验收标准必须是可检验的（编译通过、某命令运行某功能、某格式加载出正确点云等）。

# 输出要求
- 输出一份结构化 markdown，目录清晰、含类图/接口签名、含复用点标注、含验收标准。
- 语言：中文。不要输出任何代码实现，只输出设计（接口签名/伪代码允许）。
```

---

## Part B —— 给 WorkBuddy 的提示词（实操编码）

> 以下方框内为可直接复制给 WorkBuddy 的提示词正文。需与「Codex 输出的设计文档」配合使用。

```text
你是一名点云可视化 C++ 开发工程师。请根据设计文档，在 pcl_viewer 仓库的 cloud_viewer 分支
**实实操写代码**，实现「激光雷达点云可视化 PC 软件 Lite 版」，并保证在 Ubuntu 20.04+ 与 Windows 10+
都能编译运行。

# 输入
- 设计文档（Codex 输出）：[请将 Codex 产出的 markdown 设计文档全文粘贴在此]
- 仓库现状与复用点、清理清单：见「〇、仓库现状速览」0.1~0.6（与本提示词一起提供）。

# 工作总原则
1. **先清理、后搭建、再功能**：先按清理清单删冗余，建立干净骨架，再逐里程碑实现功能。
2. **最大化复用**现有 `PointData`/`PointMatrix`/`Pulse`/`Converter`、`readPulseData`/`readSolidLidarData`/
   `readLidarData`/`readFrameCsv`/`readFrameCsv_6031`/`readPcdData`、`mat2pcl`/`mat2xyzi`/`showCloud*`、
   `PclHelperB` 的可视化方法、`point_selection.h` 的 `CallbackArgs`。不要重写等价逻辑。
3. **每完成一个里程碑必须自测**并给出验收证据（编译通过、运行结果），未通过不进入下一里程碑。
4. 不引入与可视化无关的依赖；新增依赖必须写入 CMake 并说明理由。
5. 遇到设计文档与仓库事实冲突时，以仓库事实为准，并在交付说明中标注偏差。

# 里程碑任务与验收标准（严格按序执行）

## M0：分支基线清理 + 构建骨架
- 在 cloud_viewer 分支建立干净的目录结构（保留 include/src 核心文件，删除/移出冗余文件）。
- 删除：python/ 下 12 个 analyze_*.py 与 os_script.py；matlab/、doc/ 空目录（如需保留 doc 则清空）。
- 裁剪：将标定/反射率/地面修复/拖尾滤波等算法代码（`Range6532*`、`CalibReflect`、`binary_storage`、
  `Ground_I`/`Timer`、`tail_point_filter`、`fast_euclidean_clustering`、`Spline`）下沉为可选模块或删除，
  保证主程序不依赖它们即可编译。
- 清理 main.cpp 的注释调试代码与硬编码绝对路径。
- 统一 CMake：明确 PCL/VTK 的 find_package 与 Windows(vcpkg) 兼容写法。
- **验收**：`cmake -B build && cmake --build build` 在 Ubuntu 通过；Windows(vcpkg 工具链) 通过；主程序可启动空窗口。

## M1：统一数据层（CloudFrame + 帧序列）
- 实现 `CloudFrame`（PointMatrix + 元数据）与 `CloudFrameSequence`（多帧/多文件容器）。
- 实现 PointData/PointMatrix → PCL 云 的通用属性转换与「属性字段映射表」。
- 保留并复用 `Converter`、`mat2pcl`/`mat2xyzi`。
- **验收**：能用一个最小测试程序把构造的 PointMatrix 转成 PCL 云并打印点数/属性正确。

## M2：离线读取解析（csv/txt 单帧多帧、单文件多文件）
- 实现统一加载入口（后缀 + 表头嗅探路由）与 `IPointCloudLoader` 各格式实现。
- 支持：转镜 txt、固态 txt、6532 Frame CSV、6031 Frame CSV、PCD；单帧/多帧；文件多选/目录扫描。
- 提供加载进度与错误/无效点统计。
- **验收**：用仓库 data/ 下的真实样例（如 `小测试间点云/*.txt`、`Frame1_ROI0_distRP.csv`）加载出正确点云，
  点数与现有 `readPulseData`/`readFrameCsv` 结果一致；多文件/多帧可顺序切换。

## M3：可视化通用底层（参考 cloud_compare 思想，基于 PCL/VTK）
- 实现场景实体树 `SceneObject`/`SceneGraph`、渲染接口 `Renderer`/`RenderableCloud`。
- 实现相机控制、坐标轴/网格/包围盒、点大小、背景、视角复位。
- 复用 `PclHelperB` 的 `initViewer`/`addViewport`/`addLine`/`addCloud*`。
- **验收**：能渲染多帧点云并自由旋转/缩放/平移；视口与坐标轴正常。

## M4：按属性着色
- 实现着色源枚举与 `ColorMapper`（连续 HSV 蓝→红、离散高对比、自定义 min/max），复用 `hsv2rgb`/`addCloudI`。
- 支持运行时切换着色源；提供色标（colorbar）。
- **验收**：同一帧点云可在 r/z/intensity/reflect/peak/timestamp/回波序号 间即时切换，颜色分布正确。

## M5：选点信息交互
- 复用 `CallbackArgs`，将 O(n) 遍历改为 KDTree 加速。
- 选中点高亮 + 信息面板展示完整属性（xyz/r/theta/slot_ang/intensity/reflect/peak/3 组回波/像素与 slot 索引/时间戳/所属文件/簇 id）。
- 多帧下点归属定位。
- **验收**：点击任意点能即时高亮并在面板显示正确属性；选中点能回溯到来源文件与帧号。

## M6：在线 SDK 接入层
- 实现 `IRadarStream`/`RadarDevice`/`RadarBackendFactory` 与 SDK 适配器（`SdkAdapter`）。
- 线程模型（采集/渲染解耦、环形缓冲）；在线/离线共用同一渲染管线。
- 注册 alpha 转镜、beta 转镜、FL6031、FL6532 的适配器占位；SDK 未就绪时用 mock/stub 数据源。
- **验收**：mock 数据源能以固定帧率向渲染管线推流并流畅显示；切换回离线源正常。

## M7：跨平台打包与联调验收
- Ubuntu：apt 依赖一键安装脚本/说明；Windows：vcpkg 依赖清单与构建说明。
- 完成三类雷达（alpha 转镜、beta 转镜、FL6031、FL6532）真实样例的端到端加载+着色+选点联调。
- **验收**：两平台均能 `cmake 构建 + 运行`，三类雷达样例均正确可视化；提供 README 使用说明。

# 编码规范
- C++17，遵循现有命名风格（命名空间 algo_lidar；结构体/类 PascalCase；函数 camelCase）。
- 头文件加 `#pragma once`；中文注释；不写无意义注释。
- 不在代码里保留硬编码绝对路径与调试残留。
- 每里程碑完成后运行 read_lints 修复错误/警告再交付。

# 交付要求
- 每完成一个里程碑，汇报：改动文件清单、关键接口、验收自测结果（命令与输出摘要）、与设计文档的偏差。
- 全部完成后，输出最终 README（构建/运行/依赖说明）与功能清单。
```

---

## 附：使用建议

1. 把「〇、仓库现状速览」与 Part A 的 prompt 一起发给 Codex，得到设计文档。
2. 审阅设计文档，必要时把调整意见写回。
3. 把「设计文档全文」+「〇、仓库现状速览」+ Part B 的 prompt 一起发给 WorkBuddy，逐里程碑执行。
4. 每个里程碑让 WorkBuddy 给出验收证据后再放行，避免一次性堆叠大量未验证代码。
