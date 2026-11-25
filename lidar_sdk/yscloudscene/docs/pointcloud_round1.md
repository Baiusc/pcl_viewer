# 点云显示重构：第一轮检查与测试

日期：2026-10-23。

## 本轮边界

`PointcloudBuild` 生成 CPU 显示帧，`PointcloudParserImp::RenderFrameReady` 通过 queued connection
把只读帧句柄交给 `PointcloudDisplay::UpdateCloud`。显示组件在自己的 OpenGL 上下文中上传数据，
再由 `ApplyUploadedCloud` 接管 VBO。处理器不再创建或共享 OpenGL 上下文。

本轮仍逐帧创建 VBO；配置快照、任务排序、最新帧槽、VBO 复用及 SDK 迁移属于后续工作。
CPU 测试通过不表示现有共享处理器已经具备完整的并发安全性。

## 手动编辑中修正的问题

- `PointcloudRenderFrame::IsValid()` 的颜色数量比较应为 `==`，不是 `!=`。
- `MakeRenderFrame` 检查输入失败后需要立即返回，否则可能继续解引用空点数组。
- `std::span` 的初始化表达式中多了一个分号。
- Y/Z 坐标的非有限数检查遗漏 `!`，会拒绝正常点。
- 补上 `Q_DECLARE_METATYPE`、`qRegisterMetaType` 和新的信号槽连接。
- 恢复误删的解析器初始化列表，包括 QObject 父对象、停止标志、任务计数和保存参数；
  同时初始化播放标志，并在启动线程前创建文件管理器。
- 清理显示组件中不再使用的共享上下文、离屏 surface 和参数传递接口。
- 注册新头文件，并对 `ys_scene` 声明 C++20 编译要求。

## 检查中发现的原有 OpenGL 问题

- VBO/VAO 编号需要初始化为 0，避免第一帧前读取未初始化编号。
- 关闭多面同显后，删除 VBO 的同时清空帧记录和初始化状态，避免旧编号再次删除已复用的资源。
- `BuildPointdata()` 在未绑定 VBO 时调用 `glVertexAttribPointer`，实测触发
  `GL_INVALID_OPERATION`。初始化阶段现在只创建 VAO，属性设置留到对应 VBO 绑定后。

## 自动测试结果

主程序 `out/build/ninja-x64-debug/YSCloudScene.exe` 已重新构建。

| 测试 | 结果 | 内容 |
| --- | --- | --- |
| PointcloudRenderFrameTest | 通过，6 个测试函数 | 帧校验、包围盒重算、原始输入不变、多面颜色、空/非法输入、queued 生命周期、解析器停止 |
| PointcloudDisplayTest | 通过，1 个测试函数 | OpenGL 4.6 上的合成点云上传、像素验证、清空、三轮单面/多面切换 |
| PointcloudFramePolicyTest | 7 passed / 2 failed（含初始化和清理） | 现有成帧规则在 799 个 SLOT 完成，两个测试预期 800 |

本轮没有修改 `business/pointcloud_frame_policy.h` 和该组帧测试，799/800 差异单独留待组帧规则审查。
GPU 测试最终没有报告 OpenGL API 错误；读取 framebuffer 验证像素时，驱动会提示像素读取同步的性能消息。
该测试用于正确性检查，不用于衡量正常显示性能。

构建最初遇到 MSVC C1902（PDB 管理器错误），本次命令临时设置 `_CL_=/Z7` 后完成 Debug 构建，
没有修改仓库的默认 Debug 编译参数。

在具有 CMake 和 Qt 运行环境的开发者终端中，可以重新运行：

```powershell
ctest --test-dir out/build/ninja-x64-debug -R '^(PointcloudRenderFrameTest|PointcloudDisplayTest)$' --output-on-failure
```

若终端中没有 Qt 的 DLL 搜索路径，先把本机 Qt `bin` 和构建目录加入 `PATH`；
图形测试需要可用的桌面 OpenGL 环境，不适合使用 Qt 的 offscreen 平台插件。

本次日志保存在 `out/build/round1-final-build.log`、`out/build/pointcloud_render_frame_test.txt`、
`out/build/pointcloud_display_test.txt`、`out/build/pointcloud_frame_policy_test.txt`。

## 接下来进行真实设备验收

1. 单面实时点云：颜色、坐标方向、旋转缩放和刷新是否正常。
2. 四面同显与反射率模式切换：四个面是否更新，退出后单面是否正常。
3. ROI 排除全部点后画面是否清空，恢复 ROI 后是否恢复显示。
4. 点选冻结、退出冻结，是否仍能恢复刷新。
5. 保存筛选后的点云、录制原始帧、回放是否符合原有行为。
6. 多次连接/断开和退出，是否出现卡住或 OpenGL 错误。

真实设备、全部机型和长时间性能尚未验证。
