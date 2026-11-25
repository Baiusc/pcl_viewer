# CS96 Gamma 点云接收与显示诊断

## 2026-10-28 实机结果

设备 `10.42.0.238`，硬件 C01，序列号 `CS96000C0126020001`。
设备上报直角坐标、单回波、120 行 × 800 列。

修复前，每帧在第 799 个不同 SLOT 到达时提前完成，95880 点交给显示；
最后一个 SLOT 随后另起残帧并在超时后丢弃。
原因是把最大槽号 799 当成了期望槽数量 800。

修复后实测日志：

```text
PointcloudReceive2: frame=0 COMPLETE size=96000 times=800 slots=800 -> enqueued (missing=[none])
PointcloudGen: frame=0 input=96000 output=96000 slots=800 pixels_per_slot=120 roi_rejected=0 invalid=0 zero=96000 distance_rejected=0 roi=[0..799, 0..119] distance=[0, 280]
PointcloudGen bounds: frame=0 min=(-0, -0, 0) max=(-0, -0, 0)
UpdateCloud: frame=0 uploaded_points=96000 bounds_diagonal=0 all_mirrors=false
```

上述诊断首次见于本次连接的 15:19:26，后续采样结果一致。
界面数据和渲染刷新约 10 FPS，但所有点重合在原点，没有可见的场景形状。
少量真实丢包仍会走 92% 完整度的超时接纳路径。

## 已完成的软件修改

- 完整帧要求期望点数和不同 SLOT 数同时收满；799 个 SLOT 等待末包或超时。
- 重复 SLOT 不再追加点数据，防止重复包导致提前成帧。
- 结构化点云逐点过滤 NaN/Inf，避免单个无效 Z 坐标导致整帧被拒绝。
- 每个处理器首帧及每 100 帧输出过滤统计与坐标范围；显示端记录实际上传点数。

## 尚需核对的数据来源

现有 SDK 使用 18 字节包头和 13 字节点结构（float XYZ + uint8 反射率），
1578 字节报文对应 120 个点。接收回调直接复制点区，显示链路只过滤和着色，未重算或清零坐标。
因此本次实机已确认的是：**按当前协议解析的 XYZ 全零**，以及这些点已成功上传到 OpenGL。

尚未独立抓取 UDP 原包，不能据此断定是固件还是协议布局问题。
后续应检查设备端真实点云的距离/XYZ 生成、Gamma 标定参数加载情况；
若固件改过输出格式，需用一份 1578 字节 UDP 原包核对字段偏移、字节序和浮点布局。
上位机无法从全零 XYZ 恢复真实场景。

## 验证

Debug 主程序已重新编译：`out/build/ninja-x64-debug/YSCloudScene.exe`。
以下三个 CTest 目标全部通过：

- `PointcloudFramePolicyTest`：120 点报文识别、800 槽边界和丢包阈值。
- `PointcloudRenderFrameTest`：重复包、末槽接收、96000 点输出、无效坐标逐点过滤。
- `PointcloudDisplayTest`：合成 Gamma 数据经过接收回调、组帧、着色、队列上传后，实际帧缓冲能检测到点云像素；另验证清屏和多面显示切换。
