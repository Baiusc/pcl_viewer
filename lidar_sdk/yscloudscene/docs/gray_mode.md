# CS96 Gamma / Beta Gray 模式

本阶段实现 Gray 接收和显示；旧 alpha/beta/MF150 直方图链路继续使用原有接口。

## Beta 单回波

按节点机型选择协议：CS 机型且硬件版本首字节为 B/b 时使用 Beta，沿用仓库的 Beta 识别规则。
`DEF_RCV_MODE=5` 是板端驱动接收模式；上位机仍通过 Gray 工作模式进入灰度页面。

Beta 严格接收 1165 B UDP 数据报（9 B 小端协议头 + 1156 B 分片），按 frame_id、
slot_number（0～167）、seq_num（0/1）重组。两片拼成 2312 B SLOT，校验首片 SLOT
头低 16 位与协议头一致，每个像素读取小端 32 位并取 `value & 0x3FFFFF`。
每帧需收齐 168 个 SLOT 的 336 片，原始数组按 `slot * 576 + pixel` 排列。
重复片保留首次数据，冲突计入统计，缺片不会显示为完整帧。

Beta 不使用毫秒超时，也不假定固定帧率；frame_id 推进时保留当前帧及上一帧以容许尾包乱序，
更旧的未完成帧丢弃。支持帧号 65535→0 回绕，断连或重连清空重组状态。

界面仅显示单回波，固定灰度范围默认 0～4194303。默认纵向 576 行 × 168 列，
可转置为横向 168 行 × 576 列；悬停显示单回波原始值，支持自动对比度、暂停和 PNG 保存。
Beta CSV 保留 22 位原始强度，元数据为 width=576,height=168,bits=22，
数据列为 `frame_id,slot,pixel,gray,valid`，共 96768 行；连续保存使用相同格式。

以下协议参数、250 ms 超时及双回波操作描述适用于 Gamma；旧直方图链路保持原接口。

## 使用

设备广播的 `work_mode=3` 表示 Gray。连接成功后自动打开灰度页面，监听握手中的
`node.data_port`，默认 50027。`config_gray.json` 中可以设置：

```json
{
  "version": 1,
  "gray": {
    "data_port": 50027,
    "frame_timeout_ms": 250
  }
}
```

组帧超时暂定 250 ms，可在联调时根据实际发送周期调整；有效配置范围为 20～10000 ms。
只显示收齐 20 个不同 SLOT 的完整帧，超时不完整帧丢弃并计入 SDK 统计。

- 回波：默认两个回波取最大值，也可选第一回波或第二回波。
- 自动对比度：按有效像素的全帧范围映射为 0～255，使用 EMA 减少闪烁。
  不同回波会分别拉伸，不宜仅通过显示明暗比较绝对强度。
- 固定灰度范围：黑阶、白阶决定映射范围，默认 0～1048575，超范围值截断。
- 默认纵向显示：120 行 × 20 列，横轴 SLOT，纵轴 Pixel。
  “显示方向”可切换至横向 20 行 × 120 列，横轴 Pixel，纵轴 SLOT。
  使用矩阵转置，两个轴的索引均保持递增；切换时保留原帧、灰度值与 EMA 范围。
- 悬停显示 SLOT、Pixel、两个回波的原始值、选择后的值及 8 位灰阶。
- 图像右键暂停后，可以切换回波或映射方式，仍使用同一原始帧。
- 右键保存 PNG 保存当前 8 位图像；暂停后保存 CSV 保存当前完整双回波原始帧。
- 连续保存选择帧数和目录。记录使用独立队列，不依赖 GUI 刷新频率；显示可合并刷新。
  暂停、断连或点击停止连续保存后，不再接收新的保存任务，已入队的任务继续写入。
  队列满或写入失败时停止本批保存并报告错误。

## 协议与组帧

严格接收 977 B 数据报：9 B 小端协议头 + 968 B SLOT。检查 SOF 0xAA、data_len 968、
SLOT 0～19，以及数据区 SLOT 头低 5 位与协议头一致。seq_num 不参与组帧。

每个像素显式读取小端 64 位字，低 20 位为 ECHO1，接下来的 20 位为 ECHO2，保留位忽略。
原始值使用 uint32_t 存储，不经过旧直方图的 0.06 缩放或 uint16_t 量化。

按设备和连接代次隔离，按 frame_id 分组、SLOT 去重。允许相邻帧交错到达；看到下一帧 SLOT 0
不会立即截断上一帧。重复 SLOT 保留首次数据；冲突、超时、迟到包均可通过 GetStats() 查看。
完整帧单调交付，支持 65535→0 回绕，重连清空所有半帧。

SDK 帧回调取得 shared_ptr<const GrayEchoFrame>，异步处理持有独立所有权。
数组索引为 slot * 120 + pixel。Gamma 收到的 wire pixel 顺序为 `0,4,...,116; 2,6,...,118; 1,5,...,117; 3,7,...,119`；
组帧时会按真实 pixel 编号还原为 0～119，因此显示、悬停和 CSV 都使用物理 pixel 顺序。显示回波选择不改变原始强度值。

## 保存格式

CSV 使用 ys_gray_v1，记录尺寸、位深、矩阵布局、设备、连接代次和显示参数。
数据区含 2400 行：

```text
frame_id,slot,pixel,echo1,echo2,valid
```

CSV 同时保留两个回波。当前接收器只交付完整帧，因此 valid 为 1；格式保留有效性字段以便后续扩展。
CSV 尺寸和 layout 描述原始 SLOT 存储布局（slot * 120 + pixel）；display_orientation 单独记录显示方向。
PNG 导出当前选定的显示方向，纵向图像为 20 × 120 像素，横向为 120 × 20 像素。
自动映射的 EMA 历史不包含在 CSV 中，复现显示应保留 PNG；CSV 用于原始强度分析。

## 验证

```bash
cmake --build out/build/x64-Debug --target YSCloudScene gray_image_test raw_histogram_test
ctest --test-dir out/build/x64-Debug -R "GrayImageTest|RawHistogramTest" --output-on-failure
```

GrayImageTest 覆盖协议校验、20 位边界、双回波、完整组帧、乱序/重复/丢包、回绕/重连、
矩阵方向、灰阶映射、暂停重映射、显示队列合并、双回波 CSV、独立连续保存、
旧直方图显示接口兼容，以及真实本机 UDP 收发与接收器重启。

尚需真实板端验证模式广播、握手端口和实际帧率。
现有 NodeManagerTest 会创建点云窗口并等待真实雷达节点，需要有窗口和设备的环境单独运行。

Beta 自动测试覆盖 22 位及保留位、287 像素分片交界、乱序/重复/冲突/缺片、慢速帧、
机型选择、单回波界面、方向切换、CSV 和本机 336 包 UDP 接收。

## 连接初始化与真机检查

指令接收器启动前必须先设置 peer IP；StartListening 返回时应已安装接收回调、
启动心跳并将节点的 instruction_processor_stoped 置为 false，然后才能发送握手。
否则 Gray 接收器启动期间的线程调度可能使节点查询使用空 IP；即使设备收到握手并关闭广播，
后续 ACK 也因接收状态未就绪而无法向界面发布连接成功。

可按需执行真机测试（默认跳过，需明确指定 Beta Gray 雷达 IP）：

```powershell
$env:YS_LIVE_BETA_IP = "10.42.0.218"
./out/build/x64-Debug/tests/gray_image_test.exe LiveBetaConnection
```

该检查直接使用 SDK 握手、等待连接成功和完整 Beta 灰度帧，结束后断开测试连接。

## Gray 接收缓冲区所有权

Windows IOCP 使用 `SetZeroCopyRecvCb`，将完成的 `IOContext` 直接交给 Gray worker 队列，
不再逐包分配 vector 或复制数据。队列通过 unique_ptr 和自定义 deleter 持有 context，
组帧处理后、拒收、队列溢出以及停止清空队列时均归还 IOContextPool。
调用完整帧回调前即归还该包的接收缓冲区，完整帧仍独立拥有原始灰度数据。
非 IOCP 平台继续按单个 UDP 数据报复制 recvfrom 的临时缓冲区。
这消除接收线程到 worker 的整包复制；Beta SLOT 重组和像素解析仍保存所需数据。

IOContext 记录独立的 buffer_capacity，PostRecv 始终使用分配容量，
避免上一包的实际长度缩小下一次接收容量（例如 Gamma 977 B 切换到 Beta 1165 B）。
