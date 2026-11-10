# AGENTS.md

面向在本仓库工作的 AI agent。人类开发者请直接读 `doc/`。

## 1. 项目速览

雷达点云查看器（`cloud_viewer`），支持 FL6532 固态雷达与 CS96 Beta 转镜雷达。
C++17 + CMake，UI 是自绘 `UiOverlay`（VTK），**没有 Qt / ImGui**。

```bash
cmake -S . -B build-lite -DCMAKE_BUILD_TYPE=Debug
cmake --build build-lite -j4
ctest --test-dir build-lite --output-on-failure
```

无交互验证（真机跑一遍并截图）：

```bash
DISPLAY=:0 CV_BETA_IP=10.42.0.222 CV_SCREENSHOT_WAIT_MS=10000 \
  ./build-lite/cloud_viewer --screenshot /tmp/x.png
```

## 2. 硬约束

- 点类型统一用 `algo_lidar::PointData` / `PointMatrix`（`include/point_type.h`），**不要新建点结构**。
- 不要引入 `pcl_io` / `pcl_visualization`（本机不可用）。
- `10.42.0.221` 是**用户自用的 CS96 Beta（96×800）**，可正常 SSH、切 work_mode、改 center_ip。
  （早前记为"同事的设备"是误判，已作废。）
- 设备 `work_mode` 必须与 `/mnt/mtdblock2/mode` 匹配，否则 `you_see` 启动即退出。
- 96 线（IMX459 / `beta_rotating`）与 120 线（SL100 / `gamma_rotating`）的**标定表不可混用**，
  分别是 `data/param/cs96beta_calibration.csv` 与 `cs120gamma_calibration.csv`。
- 设备侧坐标是 `y+` 向左，viewer 统一 `x+` 前 / `y+` 右，翻转只在解析层做一次。
- **单位**：转镜雷达（alpha/beta/gamma）的标定表与点云都是**米**，固态雷达（6532/6031）是**厘米**。
  换算只在固态侧做，转镜侧不要做任何换算。

## 3. 文档维护（必做）

**每完成一次排查或改动，都要把「下一个人会踩的坑」写回 `doc/` 的相关 md。**
这是项目要求，不是可选项。

写什么：

- 结论与根因（一句话能说清的那种）
- 现场实测数据（IP、MAC、报错原文、设备回显）
- 改动的判定依据，以及**仍未验证的前提**
- 硬约束（新发现的"必须这样否则会坏"）

不要写什么：

- 过程流水账（试了 A 不行、试了 B 不行）
- 代码能自解释的细节（函数签名、字段列表）
- 一次性的临时命令（清掉 `/tmp` 脚本，需要复现的整理成固定命令再写）

写到哪：

| 内容 | 文件 |
|---|---|
| 设备 IP / SSH / 网络排查 / 连接与模式切换 | `doc/codex_cs96beta_handoff.md` |
| 120×800（CS96 Gamma / SL100）协议与解算差异 | `doc/cs96_gamma_sl100_protocol.md` |
| 芯片层（RAW/RANGING、多回波、降采样、假峰） | `doc/sl100_chip_notes.md` |
| PA / 编码器 / 面与水平角推导 | `doc/cs96beta_pa_angle_analysis.md` |
| 整体架构与模块设计 | `doc/codex_design.md` |
| 设备侧手工排查步骤 | `doc/CS96_Beta_SSH手动排查与连接指南.md` |

风格：表格优先、短句、带数字。**宁可少写，也要让人一遍读懂。**

## 4. 真机性能验证（不要只截图）

**只看首帧截图判断不了帧率**，更看不出"稳定 5fps"和"偶尔 0.2fps"。必须连续跑一段，按时间窗口统计。

```bash
# 每 2 秒吐一行：窗口帧率、最大帧间隔、各阶段耗时
DISPLAY=:0 CV_BETA_IP=10.42.0.221 CV_LIVE_FPS=1 timeout 24 ./build-rel/cloud_viewer 2>&1 | grep '\[fps\]'
```

输出里三个数最关键：

| 字段 | 含义 | 判读 |
|---|---|---|
| `X fps` | 窗口内实际帧率 | 与真实点云（9~10）对比 |
| `gap_max` | 窗口内最大帧间隔 | 稳定在 100~140ms 才算不卡；出现 >1s 就是卡帧 |
| `recv pkts` | 累计收包数 | 每 2 秒应涨约 16000（800 包/帧 × 10 帧） |

`no new frame for Ns` 是心跳，用来区分「设备没推流/解析慢」和「渲染慢」——
**只在连续 2 秒真的没有新帧时输出**（收到帧会刷新基准，不会和 `[fps]` 交替刷屏）。
如果只有心跳没有 `[fps]`，问题在接收侧，不要去优化渲染。

### 面轮转验证（原始点云 / 真实点云通用）

```bash
# 每帧打印发布的面号
DISPLAY=:0 CV_BETA_IP=10.42.0.221 CV_FACE_TRACE=1 timeout 16 ./build-lite/cloud_viewer 2>&1 | grep '\[face\]'
```

正常输出是面号 `0→1→2→3→0` 严格轮转、`frame=` 连续递增：

```
[face] publish face=1 frame=16371
[face] publish face=2 frame=16372
[face] publish face=3 frame=16373
[face] publish face=0 frame=16374
```

面号跳号 = 某些面被 `faceMask` 过滤掉（正常）；**帧号跳号 = 丢包或整帧被丢弃**，
去查 `CV_BETA_DEBUG=1` 的 `recv/parsed/timeouts`。

原始点云与真实点云都是**一帧一个面：96×800 = 76800 点，FOV 120°**。

CS96 是车规级**前向主雷达**，不是 360° 环视雷达。4 个面只是转镜的机械结构：
每帧独立输出一个 120° 扇区，既不相加、也不拼整圈。HUD 的 `Points: N/76800`
分母若不对，说明帧尺寸算错了。

**性能类结论必须用 `build-rel`（Release）复测一遍**：Debug 下单帧成本可高 30% 以上，
且容易被误判成"架构瓶颈"。

### `--screenshot` 的隐藏前提

截图分支里必须先让一个 `vtkOrientationMarkerWidget` 执行 `SetEnabled(1)`
（viewport 给 0.001，肉眼不可见）。它会触发一次完整的多层绘制，把窗口状态坐实；
否则这套 VTK 取窗口像素时拿不到有效 buffer，`vtkWindowToImageFilter`
和 `vtkRenderLargeImage` 都会在 `memcpy` 越界段错误。见 `ViewerApp::run()` 截图分支的注释。

另：坐标轴是 `vtkAxesActor` 直接加到 renderer（原点 + 1m），不是 widget；
它的 X/Y/Z caption 要手动压尺寸（`SetWidth(0.04)` / `SetHeight(0.015)`），
否则按主视口归一化会大到遮屏。

## 5. 收尾检查

- `cmake --build` + `ctest` 通过（8 个测试）。
- 改了坐标/标定/包解析的，必须同步更新相关断言，并在回复里说明"哪个前提没验证"。
- 临时脚本、截图用完删掉。
