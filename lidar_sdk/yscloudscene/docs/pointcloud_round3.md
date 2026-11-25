# 点云显示第三轮：合并待显示帧与复用 GPU 缓冲

实施日期：2026-11-02。

## 最终流程

创建 PointcloudTask / PointcloudTaskFlat 时取得全进程递增 sequence，随后提交线程池。
任务完成后，RenderFrameReady 的 DirectConnection 回调只调用线程安全 mailbox，
不访问 QWidget、相机或 OpenGL。回调持有 mailbox 的 shared_ptr，不捕获显示控件指针。

mailbox 单面只保留最新一帧，多面按 frame_id % 4 各保留最新一帧。
替换的旧帧在锁外释放；保留各槽位已接受的最大 sequence，取走帧后仍拒绝迟到的旧结果。
面号用于分面，sequence 用于判断新旧，两者互不替代。
直接调用处理器/构建器时，在调用入口默认取得 sequence；手工构造渲染帧必须填写非零 sequence。

mailbox 从无待处理通知变为有通知时发出 FramesAvailable，通过 QueuedConnection 请求 GUI update()。
Qt 事件不携带整帧。paintGL() 一次取走待显示帧，上传后释放 CPU 帧引用。
这限制的是显示交付缓存，不是上游线程池任务数量，也不会丢弃录制/保存任务。

## OpenGL 资源

点云上传集中到 paintGL()；Qt 在调用此函数时保证控件的上下文为当前上下文，
参见 [QOpenGLWidget 文档](https://doc.qt.io/qt-6/qopenglwidget.html)。
UpdateCloud() 保留为 GUI 线程入口，只发布到 mailbox，不执行 GL 操作。

单面及四个镜面各有一对 VBO，首次非空上传才创建；容量不足时调用 glBufferData 扩容，
已有容量足够时使用 glBufferSubData。空帧将点数清零，保留存储。
切换模式清除可见数据并保留缓冲容量，不会复活上次模式的旧点云。
每次绘制显式绑定点云 shader 和对应 VBO 属性，避免共享 VAO 留下另一面的绑定。

当前场景包围盒由当前有效 GPU 帧重新求并集，单面更新或某面替换后可以收缩；
空帧和单点也能正确更新状态。首次非空点云适配视野，此后更新不重置用户视角。

GetRenderStatistics() 提供 GUI 线程诊断：上传帧数、创建缓冲对数、扩容次数。
GPU 存储保持历史最大容量，直到控件销毁；不承诺 glBufferSubData 无驱动同步等待。
本轮未引入持久映射、GPU fence 或多缓冲轮转，尚未测量实机帧率收益。

## 切换与生命周期

模式切换及拾取冻结/解冻时清空待显示帧并建立 sequence 边界，切换前提交的任务不能随后回填。
冻结期间不接受新显示帧。控件销毁时 Close() mailbox，仍持有 mailbox 的回调只能丢弃帧。
即使窗口尚未初始化 GL 就销毁，也不会调用未初始化的 GL 函数。

sequence 不随处理器对象重建或硬件面号回绕而归零。
现有 Stop() 仍等待生产线程和线程池结束，保留最后画面；未增加断连自动清屏或流会话 epoch。
SDK 回调的停止/解绑仍遵循原项目生命周期，不属于 mailbox 的职责。
工作任务的配置快照仍在实际执行时取得，与第二轮约定一致。

这里采用固定槽位加短互斥锁，而非 FIFO 无锁队列：显示需要覆盖过时帧，不需要逐帧出队。
这不是对整个点云生产链的背压或有界队列改造。

## 验证结果

- 主程序 YSCloudScene 和两个点云测试目标构建成功。
- CTest：PointcloudRenderFrameTest、PointcloudDisplayTest，2/2 通过。
- CPU 新增覆盖：通知合并、旧帧引用释放、取走后拒绝迟到帧、四面独立新旧判断、
  模式/冻结边界、两个处理器反序执行任务、两个线程并发发布。
- 显示新增覆盖：GL 初始化前收到帧、连续多帧只上传最新帧、空帧清屏、晚到旧帧拒绝、
  同尺寸/缩小复用容量、增大时扩容、四面每面一次上传、模式切换不重建 VBO、
  未初始化控件销毁后的 mailbox 安全丢弃。
- 既有 Gamma 模拟收包到 framebuffer 全链路与 GL 错误检查测试继续通过。
- git diff --check 通过。未连接真实雷达，未运行线程消毒器。

构建沿用 out/build/ninja-x64-debug 和命令局部 _CL_=/Z7。
构建日志：out/build/round3-build.log。
测试日志：out/build/ninja-x64-debug/Testing/Temporary/LastTest.log。

## 实机检查

持续接收时缩放、旋转窗口，确认交互流畅；单面/四面反复切换，确认无旧面残留；
拾取冻结后解冻，确认恢复新数据；停止、重连和回放跳转后观察显示是否恢复。
这些现场行为需要真实设备或录制文件进一步确认。
