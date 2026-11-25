# YSCloudScene

基于 Qt 6 的激光雷达点云可视化桌面应用，采用 Google Repo 管理多仓库项目。

## CS96 Beta 设备 SN 管理

从主窗口工具栏打开「设备 SN 管理」。选择雷达所在网卡，从 UDP 发现结果选择设备或手动输入 IPv4，读取在线 SN。在线服务异常时可查询固件提供的只读诊断入口。

- 写入前填写工单号、操作员、设备标签 MAC 及 MES 已分配的 SN，并在确认窗口核对机型、硬件版本、组装地、生产年周和唯一性。
- 产线电脑需启用 Windows OpenSSH 客户端，并由工装为当前操作系统账户配置授权的 SSH 密钥。在该账户的 `~/.ssh/known_hosts` 登记经核验的设备 SSH 主机公钥，主机字段使用 `lidar-mac-<12 位小写 MAC>`，例如 `lidar-mac-98038a7f32ee`。
- 写入后按在线回读和 DNS-SD 结果处理。工单记录位于应用本地数据目录的 `sn_audit` 子目录，使用 JSON Lines 格式。MES 回传由产线流程完成。

## 为什么放弃 Git Submodule 改用 Google Repo

| Submodule | Google Repo |
|-----------|-------------|
| 每个子仓库需要手动 `git submodule update` | `repo sync` 一键同步所有仓库 |
| 子仓库版本锁定在特定 commit，切换分支繁琐 | 每个仓库独立跟踪自己的分支，`repo sync` 自动对齐 |
| 嵌套子仓库管理困难 | 原生支持多层嵌套仓库 |
| 团队协作时很容易出现子仓库版本不一致 | manifest 统一声明分支映射，杜绝版本混乱 |

本项目包含 1 个主仓库 + 6 个子仓库，分支对应关系复杂（`develop/ui-upgrade-with-design`、`main`、`dev`、`feature/vendor-lidar-user` 等），Repo 的 manifest 机制让分支管理清晰可控。

---

## 环境依赖

### 必须安装的软件

| 软件 | 最低版本 | 用途 |
|------|----------|------|
| [Visual Studio 2026](https://visualstudio.microsoft.com/) | Community | C++ 编译工具链（MSVC） |
| [Git for Windows](https://git-scm.com/download/win) | 2.40+ | 版本控制，提供 Git Bash 终端 |
| [Python 3](https://www.python.org/downloads/) | 3.10+ | Google Repo 运行环境 |
| [CMake](https://cmake.org/download/) | 3.20+ | 跨平台构建系统（vs2026会自行安装，不必单独安装） |
| [Ninja](https://ninja-build.org/) | 1.11+ | 构建生成器（配合 CMake Presets 使用，vs2026会自行安装，不必单独安装） |
| [Qt 6](https://www.qt.io/download) | 6.10.2+ | GUI 框架（msvc2022_64） |

### Google Repo 安装

在 Git Bash 中执行：

```bash
# 下载 repo 启动脚本
mkdir -p ~/bin
curl -o ~/bin/repo https://storage.googleapis.com/git-repo-downloads/repo

# 配置 PATH 和别名
echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
echo "alias repo='python \"\$HOME/bin/repo\"'" >> ~/.bashrc
source ~/.bashrc

# 验证
repo --version
```

> 如果 `repo` 报 "Python was not found"，执行 `alias repo='python ~/bin/repo'` 并重新打开终端。

### vcpkg 安装与三方库配置

本项目所有 C++ 三方库通过 **vcpkg** 管理，安装 x64-windows 版本：

```bash
# 安装 vcpkg（如已安装可跳过）
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat

# 安装依赖（x64-windows）
.\vcpkg install crashpad:x64-windows
.\vcpkg install spdlog:x64-windows
.\vcpkg install nlohmann-json:x64-windows
.\vcpkg install gtest:x64-windows
.\vcpkg install concurrentqueue:x64-windows
.\vcpkg install pybind11:x64-windows
```

> **注意**：安装完成后，`CMakeUserPresets.json` 中的 `CMAKE_TOOLCHAIN_FILE` 需要指向你的 vcpkg 安装路径（见下文配置）。

### 可选安装

| 软件 | 用途 |
|------|------|
| [SourceTree](https://www.sourcetreeapp.com/) | Git 图形化管理工具 |
| [VS Code](https://code.visualstudio.com/) | 代码编辑（搭配 `yscloudscene.code-workspace`） |

---

## 代码克隆

以下所有命令在 **Git Bash** 中执行。

### 1. 开启代理（如有需要）

如果网络访问 Google 不稳定，打开代理软件的 **TUN 模式**，让 Git Bash 的命令也能走代理。

### 2. 克隆主仓库

```bash
git clone -b develop/ui-upgrade-with-design git@10.42.0.10:ys_window/yscloudscene.git
cd yscloudscene
```

> 含义：从 GitLab 服务器克隆主仓库，并切换到 `develop/ui-upgrade-with-design` 开发分支。

### 3. 初始化 Repo 并同步所有子仓库

```bash
repo init -u git@10.42.0.10:ys_window/yscloudscenemanifest.git -b main --no-repo-verify
repo sync
```

> 含义：
> - `repo init` — 从 manifest 仓库下载子仓库配置文件（`default.xml`），声明每个子仓库的远程地址和分支。
> - `--no-repo-verify` — 跳过 repo 工具自身的 GPG 签名验证（Windows 环境常见问题）。
> - `repo sync` — 根据 manifest 拉取全部 6 个子仓库到对应目录。
>
> **如果 `repo init` 失败**：删除 `.repo` 目录后重试。

### 4. 配置 Git 排除规则

```bash
powershell -ExecutionPolicy Bypass -File setup_repo_exclude.ps1
```

> 含义：将子仓库路径写入 `.git/info/exclude`，避免它们被主仓库和 SourceTree 当成未跟踪文件显示。VS 和 VS Code 不读取此文件，不影响代码编辑。

### 5. 配置 CMakeUserPresets.json

编辑项目根目录的 `CMakeUserPresets.json`，将以下路径改为本机实际安装位置：

```json
"CMAKE_TOOLCHAIN_FILE": "你的vcpkg安装路径/scripts/buildsystems/vcpkg.cmake",
```

同时确认 Qt 路径和版本匹配：

```json
"QTDIR": "你的Qt安装路径/6.10.2/msvc2022_64"
```

### 6. 构建

```bash
# Debug 构建
cmake --preset ninja-x64-debug
cmake --build out/build/ninja-x64-debug

# Release 构建
cmake --preset ninja-x64-release
cmake --build out/build/ninja-x64-release
```

---

## 为什么使用 CMake Presets 而不推荐 sln

### CMake Presets 的优势

| 特性 | CMake Presets | Visual Studio .sln |
|------|:--:|:--:|
| 跨平台 | 支持 Windows/Linux/macOS | 仅 Windows |
| 可版本控制 | JSON 文件可提交到 Git | 生成文件，不应提交 |
| 团队一致性 | 所有成员使用相同配置 | 每人本地生成的 sln 可能不同 |
| CI/CD 友好 | 命令行直接构建 | 需要 MSBuild 或 VS 环境 |
| 构建工具灵活性 | 支持 Ninja/MSVC/Makefile | 绑定 MSBuild |
| 编译速度 | **Ninja 构建速度远快于 MSBuild** | MSBuild 较慢 |

### Windows 下不建议使用 .sln 的原因

- **生成文件不应入库**：`.sln` 由 CMake 生成，包含绝对路径和本地环境信息，提交到 Git 会导致冲突。
- **多仓库项目兼容性差**：项目包含 6 个子仓库，各自有独立的 CMakeLists.txt，CMake Presets 通过 `CMAKE_TOOLCHAIN_FILE` 统一管理 vcpkg，而 sln 需要在 VS 中逐个配置。
- **Ninja 性能更优**：Ninja 并行编译效率高于 MSBuild，在全量构建时差异明显。
- **VS 原生支持 CMake Presets**：Visual Studio 2022+ 直接识别 `CMakePresets.json` 和 `CMakeUserPresets.json`，打开文件夹即可自动配置。

---

## 仓库结构

```
yscloudscene/                   # 主仓库
├── business/                   # 业务逻辑
├── component/                  # UI 组件
├── scene/                      # 场景模块
├── tests/                      # 测试
├── third_party/                # 本地三方库（qcustomplot, qt-toast）
└── CMakeLists.txt
you_see_sdk/                    # 点云 SDK（子仓库）
├── vendor/you_lib/             #   嵌套子仓库（不同的分支）
└── you_see/                    #   SDK 核心代码
algorithm/repair_ground/        # 地面修复算法（子仓库）
matrix_lidar_sdk/               # 矩阵激光雷达 SDK（子仓库）
you_lib/                        # 纯c++编写的通用工具库（子仓库）
lumosy/                         # 基于Qt的组件库，包括UI组件（子仓库）
```

## 日常开发命令

```bash
repo sync                       # 同步所有子仓库最新代码
repo status                     # 查看所有仓库分支状态
repo forall -c 'git log -1'     # 查看所有仓库最新提交
```

## 团队加入指南

新成员按以上「代码克隆」步骤执行即可，全程无需管理员权限。

## CS96-gamma 灰度图

Gray 模式（work_mode=3）的连接、双回波选择、显示和保存说明见 [灰度图使用说明](docs/gray_mode.md)。
