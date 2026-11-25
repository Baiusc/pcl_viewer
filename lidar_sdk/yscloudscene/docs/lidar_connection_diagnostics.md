# Node Manager 握手状态排查

SwitchButton 的日志值：`0=Off`（红色）、`1=Pending`（等待）、`2=On`（绿色）。
点击后出现 `1` 只表示连接请求已发起。`Handshake sent` 只表示 UDP 发送成功，
应以 `Handshake ACK accepted` 判断是否收到了雷达确认。

连接入口会先停止旧接收器、重新建立指令和数据监听，所以出现在新握手发送之前的
`Clean shutdown` / `InstructionProcessor Free` 不能单独作为新连接意外断开的证据。

## 已修复的状态与时序问题

- 广播解析中的 `LidarInfo` 原来没有初始化两个接收器停止标志。
- 点云、原始点云和直方图模式原来先启动线程，再设置目标 IP；线程可能查到旧 IP 或空 IP。
- 数据接收器的启动原来异步返回，握手确认可能先于接收器状态更新。
- 握手期间的广播原来会覆盖运行状态及已分配的接收端口。
- 指令接收器重启原来保留上次的心跳等待记录，新连接可能继承旧超时。
- 指令与发现监听器原来在接收线程停止补投递 IO 之前关闭 socket，可能出现无效句柄异常。

现在先设置目标 IP，并等待接收回调和状态初始化完成后才发送握手；广播更新保留运行状态，
重连清空旧心跳记录。成功握手后立即发送心跳，保活不再依赖数据接收器状态通知。

界面只应用同一 IP 的最新排队快照；只要接收器尚未全部停止且未完成握手，就保持
`Pending`。握手确认前不发送心跳，六秒未确认会记录 `Handshake ACK timeout` 并清理本次监听，
避免把握手失败误报为已连接节点的心跳超时。

## 部署电脑上的新日志

1. `Handshake sent`：核对 `peer`、`local`、`command_port`、`data_port` 和 `mode`。
2. `Handshake ACK accepted`：正常情况下 `command_ready=true`、`data_ready=true`。
3. `Node state update`：包含节点 IP、两个停止标志和按钮状态变化，可判断是谁将按钮改回红色。
4. `Handshake not sent`、`Handshake send failed`、`Handshake rejected`、`Heartbeat timeout`
   分别表示本地准备失败、发送错误、雷达拒绝、心跳超时。
5. `Handshake listeners ready` 在发送前记录真实接收器状态，可与稍后应用的 UI 快照区分。
6. `First command UDP received` 表示指令端口收到过 UDP；握手超时中的 `receive_callbacks=0`
   表示指令监听器从未收到 UDP 回调，非零则还需检查包内容、来源和协议解析。
7. `Handshake ACK arrived on discovery port` 表示确认发到了发现端口而非指令端口。

若只有发送日志、始终没有 ACK，继续核对部署电脑实际收到的 UDP 确认包及握手携带的本机 IP、
端口；广播关闭本身不能证明上位机收到过确认。保留从点击前到点击后至少十秒的完整日志。

可在部署电脑上、程序正在等待握手时运行只读诊断脚本（端口以当次握手日志为准）：

```powershell
.\scripts\lidar_connection_diagnostics.ps1 -PeerIp 10.42.0.218 -CommandPort 50014 -DataPort 50027
```

输出 Windows 选择的路由与源 IP、相关 UDP 端口的进程归属、网络类别和防火墙配置，
不修改网卡、路由或防火墙。防火墙配置输出不包含逐条应用规则的判定。
若同一接收端口出现多个进程，需要进一步排除端口共享；当前 SDK 默认使用 `SO_REUSEADDR`，
Windows 下同端口共享的数据投递存在不确定性，参见
[Microsoft Winsock 说明](https://learn.microsoft.com/en-us/windows/win32/winsock/using-so-reuseaddr-and-so-exclusiveaddruse)。

## 无硬件回归

构建 `lidar_connection_test` 后运行：

```powershell
ctest --test-dir out/build/ninja-x64-debug -R '^LidarConnectionTest$' --output-on-failure
```

测试只向本机回环地址收发 UDP，覆盖广播初始化、广播与运行状态合并、五种工作模式的
接收器就绪与握手通知、重连心跳清理、无效握手确认。运行环境需允许本机 UDP 通信。
