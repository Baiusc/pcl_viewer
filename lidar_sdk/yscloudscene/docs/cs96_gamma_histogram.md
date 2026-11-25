# CS96 Gamma 原始直方图

连续原始直方图（work_mode=0）的组帧规则：**一个 FPGA SLOT 就是一个实际帧**，不等待底层 56 SLOT 的帧中断。

## 接收协议

- FPGA SLOT 为 122912 字节：32 字节头部，加上 120 个像素 × 512 bins × 2 字节。FPGA 头部不经过 UDP 上传，也不能由上位机恢复。
- 每个 UDP 数据报为 4105 字节：`AA`、小端 `frame_id`、小端 `slot_number=0`、小端 `seq_num=0..29`、小端 `data_len=4096`，然后是 4096 字节载荷。
- 第 `seq_num` 包携带像素 `4*seq_num .. 4*seq_num+3`，每像素 512 个小端 `uint16_t`；上位机保留上传顺序，不套用点云或灰度协议的像素重排。
- 使用 `frame_id` 独立组帧，支持乱序、去重及 `65535 -> 0` 回绕。30 个合法分包全部收到才交付；缺包帧不会补零显示或保存。半帧缓存按时间及数量限制回收。
- SOF、包长、SLOT、序号、载荷长度均校验。接收器按连接的设备 IP 过滤。

Gamma 和 Beta 都使用 4105 字节数据报，不能仅依靠包长区分。NodeFounder 建立连接时向接收器传入设备信息：CS 硬件版本以 `B/b` 开头使用 Beta，`A/a` 保留 Alpha，其余 CS 使用 Gamma。这延续已有灰度模式的 Beta 识别方式；测试使用 `G` 标识 Gamma。直接使用 SDK 接收器时，应在启动监听前调用 `SetSourceNode(node)`。

现有 Windows IOCP 接收缓冲为 **65536 字节**，可容纳完整数据报；IP 分片由系统网络栈重组后交付 UDP，无需应用层拼接 IP 分片。网络设备和防火墙须允许分片及重组。本机 UDP 测试验证完整 4105 字节接收，实际链路分片仍需设备联调。

## 显示与保存

- 灰度图为 **1 列 × 120 行**。接收侧始终保留最新完整帧，GUI 每 **50ms** 取一次（最多约 **20fps**）；中间帧被替换，不在 Qt 事件队列或全局线程池里逐帧积压。灰度取所选 bin 区间内的峰值；Gamma 默认区间为 `[0, 512)`。
- 点选像素可查看 bin 0～511 的完整波形。播放时选中的曲线随完整帧刷新，暂停后可查看最近完整帧。
- 右键 `save frame` 保存当前完整帧；`continuous save raws` 按设定数量保存后续完整帧。异步保存使用帧快照，避免播放中数据变化造成混帧。连续保存使用独立线程池，最多同时执行两个任务，不建立无限长度的待保存帧队列；磁盘忙时，剩余数量由后续完整帧继续完成。保存不受 GUI 显示降帧影响。
- 文件位于程序目录的 `raw_data/`。单帧文件名包含 `cs96_gamma`，连续保存文件名包含 `_gamma_`，并使用独立批次目录。
- TXT 头记录设备、UDP 帧号和 `cols: 1  rows: 120  bins: 512`；按 `[slot=0][pixel=0..119]` 输出，每像素 512 个无符号十进制 bin。
- 同名 BIN 为 30 包载荷按序连接，共 **122880 字节**；不含 UDP 头或 FPGA 头。像素 `p` 的 bin `b` 位于 `2*(p*512+b)`，类型为小端 `uint16_t`。

## 验证

`Cs96GammaHistogramTest` 覆盖：乱序及重复包、协议字段校验、跨帧隔离、完整帧去重、帧号回绕与重置、全部 120×512 bin 解码、最后一个像素/最后一个 bin 的显示、TXT/BIN 精确保存，以及连续模式的本机 UDP → 显示 → 连续保存链路。额外覆盖 GUI 暂停消费时连续输入 1000 帧、全局线程池繁忙时刷新最新帧、约 71fps 输入且六条曲线同时显示时的刷新频率、GUI 心跳和缩放范围保持。`RawHistogramTest` 保留 Beta 连续 56 SLOT 组帧及按 SLOT 灰度显示的回归检查。

```powershell
cmake --build out/build/ninja-x64-debug --target cs96_gamma_histogram_test raw_histogram_test
ctest --test-dir out/build/ninja-x64-debug -R 'Cs96GammaHistogramTest|RawHistogramTest' --output-on-failure
```
