# AGENTS 指南（YSCloudScene）

> 适用范围：本仓库所有 C++/Qt/CMake 代码。
> 目标：为自动化代理提供稳定、可执行的开发与测试指引。

## 1. 构建 / 运行 / 测试命令

### CMake 版本与生成器
- 最低版本：CMake 3.20（见根目录 CMakeLists.txt）。
- 推荐生成器：Ninja（见 CMakeSettings.json）。

### 常用构建（Debug）
```bash
cmake -S . -B out/build/x64-Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build out/build/x64-Debug --config Debug
```

### 常用构建（RelWithDebInfo）
```bash
cmake -S . -B out/build/x64-Release -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build out/build/x64-Release --config RelWithDebInfo
```

### 运行测试（CTest）
```bash
ctest --test-dir out/build/x64-Debug
```

### 运行单个测试（示例）
```bash
ctest --test-dir out/build/x64-Debug -R NodeManagerTest
```

> 测试定义：tests/CMakeLists.txt 中 add_test(NAME NodeManagerTest COMMAND node_manager_test)

### 其他 CMake 目标
- 翻译相关目标（在 scene/CMakeLists.txt 定义）：
  - `update_translations`
  - `prepare_translations`

### 可能的依赖
- Qt6 或 Qt5：优先 Qt6，若未找到则使用 Qt5。
- crashpad：必需（CMakeLists.txt 中 find_package(crashpad CONFIG REQUIRED)）。

## 2. 代码风格与约定（基于仓库现有代码）

### 代码格式（.clang-format）
- IndentWidth: 4
- ColumnLimit: 120
- BraceWrapping: Custom
  - AfterClass/AfterFunction/AfterStruct/AfterUnion: true
  - BeforeCatch/BeforeElse: true
  - AfterNamespace: false
- BreakConstructorInitializers: BeforeColon
- PackConstructorInitializers: Never
- AllowShortFunctionsOnASingleLine: None
- SeparateDefinitionBlocks: Always

### 头文件/实现文件结构
- include 顺序（常见模式）：
  1) 本地头文件（"xxx.h"）
  2) 同模块头文件
  3) 标准库头文件（<iostream>, <memory> 等）
  4) Qt 头文件（<Q...>）
- 需要 Qt 元对象的类使用 Q_OBJECT。

### 命名约定（来自代码样式）
- 类名：PascalCase（例如 `NodeManager`, `SwitchButton`）
- 枚举类：PascalCase 枚举名 + `KXxx` 枚举值
- 成员变量：snake_case + 结尾下划线（如 `tree_widget_`, `matrix_adapter_`）
- 方法名：PascalCase（如 `AddNodeList`, `UpdateMatrixLidarStatus`）
- Qt slot/signal：PascalCase 或带语义动词（如 `LidarConnectionChanged`）

### Qt/GUI 相关约定
- UI 组件通常以 `QWidget*` 持有，Qt 信号槽连接使用 `connect`。
- 常见模式：在构造函数中初始化 UI、绑定信号、设置样式（见 NodeManager）。
- 资源使用 `:/` 前缀（Qt 资源系统）。

## 3. 目录与模块定位

- 根构建文件：CMakeLists.txt
- 测试目录：tests/
- UI/业务代码：component/、business/
- SDK/第三方：you_see_sdk/、you_lib/、lumosy/、matrix_lidar_sdk/、third_party/
- 资源与翻译：sources/、translations/

## 4. 规则文件（Cursor/Copilot）

- .cursor/rules/：未发现
- .cursorrules：未发现
- .github/copilot-instructions.md：未发现

## 5. 贡献与修改建议

- 修改 UI 相关类时，优先查看同模块现有实现以保持风格一致。
- 不要引入新的格式化规则；遵循 .clang-format。
- 若新增测试，请在 tests/CMakeLists.txt 中注册 add_executable + add_test。
- 不要在不必要的情况下重构（优先小改动、最小范围修复）。

## 6. 快速检索入口（示例）

- 主程序入口：main.cpp
- 主窗口：mainwindow.h / mainwindow.cpp
- NodeManager：component/node_manager/
- SwitchButton：component/switch_button/
