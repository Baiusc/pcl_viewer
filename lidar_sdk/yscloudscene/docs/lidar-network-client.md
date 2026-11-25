# 设备 IP 发现与配置

从主窗口工具栏打开「设备 IP 发现与配置」。选择连接雷达的电脑网卡，在发现列表中选择设备，或手动输入 IPv4 和管理端口。查询网络状态后，可以提交自动获取、地址池或固定 IP。设备快照显示实际地址、最后确认配置及当前事务。

UDP 使用 5354，按收到报文的接口过滤。HTTP 使用工程已有的 libcurl，并绑定所选网卡上与目标同网段的本地 IPv4。跨网段配置需要事先为电脑准备覆盖目标地址的本地 IPv4。程序不会自动修改电脑网络配置。

构建使用现有 `ninja-x64-debug` 预设。新增界面使用同一套 Qt 安装中的 Concurrent 模块。协议测试可独立构建和运行：

```powershell
cmake --build --preset ninja-x64-debug --target lidar_network_protocol_test
ctest --test-dir out/build/ninja-x64-debug -R '^LidarNetworkProtocolTest$' --output-on-failure
```

首版验证范围是客户端的配置校验和事务判定。DNS-SD、采集会话自动重连、跨重启任务恢复和端到端网络替身测试仍待接入。实机需要已部署 lidar-network v1 的设备。

## 实机联调记录

| 项目 | 记录 |
| --- | --- |
| 日期、上位机提交、设备镜像和协议版本 | 未测 |
| 专用网卡、本机 IPv4、设备 MAC/SN、原地址及管理端口 | 未测 |
| 原配置、目标配置、任务号、提交前后 epoch | 未测 |
| 发现方式、实际新地址、HTTP 连接出口 | 未测 |
| 确认次数、最终事务阶段和结果码、实际及确认配置 | 未测 |
| 采集握手、首帧来源和脱敏日志 | 未测 |
