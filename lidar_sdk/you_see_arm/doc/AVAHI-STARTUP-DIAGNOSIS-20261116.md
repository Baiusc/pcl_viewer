# #17 板端 Avahi 启动失败核查

日期：2026-11-16。设备：`10.42.0.30`，MAC `98:03:8a:7f:32:ee`。用户确认烧录 Jenkins #17。应用锁定提交为 `5bf8c80951ef5abce75b0491884d74c6c6dd5ca6`。

## 已确认的原因

`S50avahi-daemon` 移到 `/usr/share/lidar/legacy/` 是应用源码 `scripts/integrate_lidar_rootfs.py` 的预期迁移。该脚本将 Avahi 生命周期交给网络服务，避免独立 SysV 入口与网络服务重复启动。

真实启动链路为 `rcS → S97lidar_network → 准备 runit 服务目录`，随后 BusyBox init 的 `::respawn:/usr/libexec/lidar/lidar-service scan` 启动监督树，`runsv lidar_netd` 运行网络服务，再由网络服务启动 Avahi。

用户提供的输出和 SSH 核查均确认：`lidar_netd` 运行、控制接口响应 200、DHCP 地址为 `10.42.0.30`、`ready=true`，但 `mdns_running=false`。故障发生在 Avahi 子进程启动阶段。

### 两处独立的不兼容

1. 应用 `network/network_discovery.c` 的 `net_discovery_tick()` 无条件传入 `--no-chroot`。本次 Buildroot Avahi 0.8 的 `config.h` 中 `ENABLE_CHROOT` 未定义，命令行解析器也不注册该选项，板端实际报错：

   ```text
   /usr/sbin/avahi-daemon: unrecognized option '--no-chroot'
   ```

2. 同一文件的 `net_discovery_open()` 无条件写入 `enable-dbus=no`。本次 Avahi 的 `HAVE_DBUS` 未定义；其配置解析器没有注册 `enable-dbus` 键。因此只去掉上述命令行参数后，板端继续报错：

   ```text
   Invalid configuration key "enable-dbus" in group "server"
   ```

应用的失败重试间隔为 60 秒，`/var/log/messages` 中可见周期性退出及重试。当前系统没有 `logread` 命令；日志应读取 `/var/log/messages`。

## 板端短时验证

保留运行中的网络服务，复制其运行配置为 `/run/lidar-mdns/avahi-daemon.probe.conf`，仅在临时副本中去除 `enable-dbus`，使用以下等效命令启动真实 Avahi：

```sh
/usr/sbin/avahi-daemon --no-drop-root --no-rlimits --debug \
  -f /run/lidar-mdns/avahi-daemon.probe.conf
```

运行 6 秒后发送 SIGTERM，确认正常退出，删除临时配置和探测日志。实际输出包括：

```text
Joining mDNS multicast group on interface eth0.IPv4 with address 10.42.0.30.
Server startup complete. Host name is lidar-mac-98038a7f32ee.local.
Service "lidar-mac-98038a7f32ee" (/etc/avahi/services/lidar.service) successfully established.
Got SIGTERM, quitting.
avahi-daemon 0.8 exiting.
```

该验证确认真实目标二进制在修正参数和配置后能够启动并发布服务；未验证外部上位机发现、重名、多设备、重启后的启动或长时间运行。未刷写固件、重启设备或变更 IP、标定与 SN。当前安装镜像的持久修复尚未实施，原网络服务的 `mdns_running` 仍为 false。

## 应用与镜像修复要求

- 在应用 `network/network_discovery.c` 中移除无条件 `--no-chroot`。当前已使用的 `--no-drop-root` 在启用 chroot 的 Avahi 中也会关闭 chroot，后者无需重复传入；对应依据是 Avahi `avahi-daemon/main.c` 的 `config.use_chroot = config.use_chroot && config.drop_root`。
- 按目标 Avahi 的真实编译能力生成 D-Bus 配置。当前无 D-Bus 的构建应省略 `enable-dbus` 键；若未来启用 D-Bus 支持，则继续显式生成 `enable-dbus=no` 以保持产品既定运行方式。应用需有明确、可验证的构建/集成能力输入，不能只凭版本号或 chroot 支持猜测 D-Bus 能力。
- 保持 `lidar_netd` 为唯一 Avahi 启动者；恢复独立 `S50avahi-daemon` 会绕过产品运行配置及失败管理，不能代替上述修复。
- 更新应用源码后正常重建、签名并归档新的镜像。不要修改已经签名的 #17 归档或把板端临时文件作为构建输入。

## 验证缺口与复验

此前 CI 检查实际镜像中的文件、执行权限、依赖、内核组播选项以及启动所有权，应用网络测试使用 `--simulation`，不会启动真实 Avahi。因此此前 `network_assets_verified=true` 仅表明离线资产核查通过，未证明 Avahi 能启动。

修复应加入真实目标 Avahi 参数/配置的兼容性回归，在目标运行环境或隔离仿真环境验证；同时保留现有主机测试。新镜像需在专用板重启验收：监督服务正常、`mdns_running=true` 持续保持超过 60 秒重试窗口、真实 Avahi 进程只有一个、上位机可解析 `_lidar._tcp.local`，并检查日志没有选项或配置错误。

本次原始证据位于本地忽略目录 `reports/avahi-startup-20261116/`：`startup-chain.json`、`board-initial.txt`、`board-reproduction.txt`、`board-corrected-probe.txt`。未保存 SSH 密码。
