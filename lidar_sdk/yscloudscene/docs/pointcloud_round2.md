# 点云显示第二轮：配置快照与稳定着色

实施日期：2026-11-02。

## 数据流程

UI 更新处理器配置 → 在 settings_mutex_ 保护下修改配置原型。
工作任务开始执行 → CaptureBuild() 在同一把锁下复制 PointcloudBuild → 释放锁 → 使用局部构建器过滤、着色 → 发布渲染帧。

快照取得时刻为任务开始执行时，不是任务提交到线程池时。逐点循环不持有配置锁。
PointcloudBuild 使用 std::array 保存完整色表；std::span<const Rgb, max_steps_> 只作为复制入口，不被保存。
每个任务复制约 3 KiB 色表及少量配置，不复制整个输入点云作为配置快照。

解析器的色表编辑仍要求在其 QObject 所属线程进行，已添加 Q_ASSERT 检查。
Update() 完整生成色表后，调用各处理器 SetColorScale() 发布副本。
Q_ASSERT 不是跨线程同步机制；外部调用者仍须遵守线程约定。

## 颜色行为变化

删除 saturation_range_ 和帧末 ComputeMinMax()。
所有映射结果固定按 [0,255] 归一化，再访问现有色表。
保留反射率的 255-reflectivity 方向和现有 LUT 量化方式。
颜色不再使用前一帧的反射率范围；这会改变旧版显示效果，不提供自动拉伸对比度。

结构化点云诊断计数改为函数内原子计数，所有任务合计每 100 帧输出一次，
避免任务局部构建器的计数每次从零开始导致逐帧打印。

## 验证

- YSCloudScene、pointcloud_render_frame_test、pointcloud_display_test 构建成功。
- CTest：PointcloudRenderFrameTest、PointcloudDisplayTest，2/2 通过。
- 新增测试：配置/色表副本独立性；两种点云路径的颜色跨帧稳定性；
  两个工作线程共用处理器时并发更新色表，检查每帧全部点的颜色一致。
- 既有输入保护、组帧、无效点及 OpenGL 显示测试包含在上述测试目标中。
- git diff --check 通过。未进行真实雷达测试或线程消毒器验证。

构建沿用 out/build/ninja-x64-debug，调用 VS 开发环境并仅对本次命令设置 _CL_=/Z7。
构建日志：out/build/round2-build.log；测试日志：out/build/ninja-x64-debug/Testing/Temporary/LastTest.log。

## 范围边界

每次 setter 与快照读取互斥，但多个 setter 不构成一个事务；两个处理器也不是同时切换配置。
正在运行的任务继续使用自己的旧快照，稍后启动的任务使用新配置。
本轮不处理任务完成乱序、UI 待显示帧积压、VBO 复用或 SDK 迁移。
算法开关目前未参与构建计算，未来启用时也应纳入任务快照。
保留现有显示筛选副本与保存行为；输入 SDK 点数组不被显示筛选原地改写。

## 实机检查建议

用同一静态场景连续显示，切换 XYZ/反射率映射和多面同显，确认颜色稳定；
接收期间调整距离阈值与 ROI，确认帧内筛选一致；停止并重连确认无异常。
