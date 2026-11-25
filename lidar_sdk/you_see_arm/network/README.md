# 设备网络服务与上位机接入

`lidar_netd` 独立管理设备 IPv4 配置，`lidar_mgrd` 仲裁业务请求。当前已完成源码、主机隔离模拟测试和 Beta ARM64 编译；真实网卡、多机交换机、Avahi 实际互通、旧上位机及 Flash 断电行为仍需开发板验收。

上位机开发请从[设备 IP 配置与发现开发指导](../doc/设备IP配置与发现-上位机开发指导.md)开始；另附[可直接交给开发 Agent 的任务书](../doc/交给上位机Agent-IP功能开发任务.md)、[HTTP OpenAPI](../doc/接口/lidar-network-v1.openapi.yaml)和[构造报文与测试场景](../doc/接口/lidar-network-v1.examples.json)。

## 三种模式

| `mode` | 上位机提供 | 设备行为 |
| --- | --- | --- |
| `auto` | 模式 | DHCP 持续获取和续租；没有可用租约时使用 Link-Local，适合直连电脑和无 DHCP 的交换机 |
| `pool` | 起止 IPv4、掩码、可选网关 | 在指定范围内选址并逐项探测，跳过网关；运行中持续冲突时仍在范围内重选 |
| `static` | 固定 IPv4、掩码、可选网关 | 探测指定地址，通过后应用；不擅自改成另一个普通地址 |

范围两端均包含，要求同一子网，最多 65534 个候选；掩码支持 `/1`～`/30`。例如 `/23` 中 `10.42.1.0` 可以是主机地址，不能套用“末段必须 1～254”的旧规则。`169.254/16` 由自动地址流程管理，不作为用户普通地址池。

三种模式互相切换均需重新确认。`auto` 不改变电脑网卡设置；电脑在直连链路上也需具有可用的 Link-Local 地址。若电脑持有另一个固定网段，工具应引导其添加对应本地地址，不能把“收到广播”当作“已可连接”。

没有新版持久配置时，优先导入旧固件保存在第三分区根目录的 `/mnt/mtdblock2/interfaces`，没有该本机配置再使用系统网络备份。显式旧 DHCP 配置也优先保留，不会被镜像里的默认静态 IP 覆盖；新设备无旧配置则默认为 `auto`。已经配置 `static` 或 `pool` 的设备不会因为换成直连就忽略用户模式，需要通过管理接口切换为 `auto`。

新设置在范围耗尽或固定地址冲突时失败并恢复最后确认配置。若开机时最后确认的业务地址也不可用，则启用仅用于管理的 Link-Local 恢复地址：`recovery=true`、`ready=false`、`source=recovery`，保留用户原配置。这不表示已在指定地址范围内成功联网。

## HTTP 接口

业务入口为 `/cgi-bin/lidar_network.cgi`，沿用现有 lighttpd 和本地桥接权限。`GET` 查询；`POST` 接受 JSON，不接受 URL 查询参数。业务请求经现有 `lidar_mgrd` 转发，模式切换与改址互斥。

范围配置示例：

```json
{
  "op": "network_set",
  "config": {
    "mode": "pool",
    "start": "10.42.0.100",
    "end": "10.42.0.150",
    "netmask": "255.255.255.0",
    "gateway": "10.42.0.1"
  }
}
```

自动模式和固定地址：

```json
{"op":"network_set","config":{"mode":"auto"}}
```

```json
{"op":"network_set","config":{"mode":"static","address":"10.42.0.120","netmask":"255.255.255.0","gateway":""}}
```

返回 `202` 只表示已持久记录任务，不表示探测、应用或确认已完成。上位机应：

1. 保存响应 `operation_id`、设备 `identity` 和 `mac`。
2. 继续查询状态；探测默认使用标准时序，单候选通常需要数秒，范围冲突时会更久。
3. 连接断开后，通过 DNS-SD 或 UDP 5354 重新发现同一设备。发现表以身份、MAC 和接收接口联合区分；重复 SN 不合并设备。
4. 当 `transaction_phase=WAIT_CONFIRM` 时，通过新设备地址向同一路径发送以下确认：

```json
{"op":"network_confirm","operation_id":"替换为响应中的32位任务编号"}
```

CGI 从可信 `SERVER_ADDR` 获取本次连接实际到达的本机 IPv4，客户端不能提供或覆盖 `via_address`。默认 60 秒未确认则回滚；带未完成任务重启也恢复最后确认配置。确认成功后返回 `200`、`pending=false`、`transaction_phase=COMMITTED`。

查询的 HTTP 状态为 `200` 时也要检查 `operation_code`、`transaction_phase`、`error`、`ready`、`recovery`，不能把查询成功当作设置成功。任务最终失败代码包括 `409`（冲突或耗尽）、`408`（确认超时）和 `500`（应用或持久化错误）。

`POST {"op":"network_reset"}` 恢复自动模式，仍走确认流程；不是擦除配置文件或绕过损坏状态检查的接口。持久文件损坏或存储不可用时保留原文件，提供临时 Link-Local 诊断地址并报告 `FAULT`、`configuration_valid=false`；拒绝保存新设置，由受控维护流程修复。此时响应中的配置对象仅作结构占位，不能当作已读出的有效用户配置。

系统诊断 CGI `/cgi-bin/lidar_network_status.cgi` 直接查询网络服务，只支持 `GET`，不依赖业务管理进程启动成功。业务 CGI 位于签名应用包，诊断 CGI 由系统镜像提供。

## 本机协议与存储

本地 `SOCK_SEQPACKET` UDS 默认 `/run/lidar-network/control.sock`，权限 `0600`，校验对端 UID。协议版本为 1，响应包含 `schema_version`；本地请求可携带同版本字段，其他版本拒绝。

```sh
# 查看网络服务；业务客户端将套接字换成 /run/lidar/control.sock 经过管理仲裁
/usr/libexec/lidar/lidar_netctl /run/lidar-network/control.sock get
/usr/libexec/lidar/lidar_netctl /run/lidar/control.sock '{"op":"network_set","config":{"mode":"auto"}}'
```

本机确认需提供 `operation_id` 和 `via_address`；这是给可信本地桥接组件使用的字段，普通 HTTP 客户端不使用。

系统策略为 `/etc/lidar/network.json`，包括机型、网卡、可信程序路径、SN 读取路径和超时。用户网络状态为 `/mnt/mtdblock2/lidar/config/network.json`。二者用途不同。

用户文件采用单个原子记录，同时保存 `confirmed_config`、`candidate_config`、`confirmed_address`、`pending` 和 `operation_id`，通过文件 fsync、rename、目录 fsync 提交，避免两个文件更新到一半。成功续租、相同地址恢复等不反复写 Flash；地址或配置真正变化时才更新记录。每次开机恢复用户配置后仍进行地址冲突检测。

SN 按无分隔符的 19 个大写 ASCII 字符存储：型号 7 位、硬件版本 3 位、组装地 1 位、生产年周 4 位、生产批号 4 位。例如 `CS96XXXB01026020001`。展示时可以按 `CS96XXX-B01-0-2602-0001` 分组，但 Flash 和协议字段使用无分隔符形式。批号为十六进制且从 `0001` 开始；生产系统负责分配，不由设备生成。

运行时由 `lidar_netd` 只读 `/dev/mtd3`，使用统一格式校验；有效 SN 作为 Avahi 主机身份，并写入 `_lidar._tcp` 服务记录。当前主机名格式为 `lidar-<SN>.local`。SN 缺失或格式无效时，发现服务使用稳定 MAC 临时身份，SN 字段保持为空。设备忙闲不影响持续发现。

旧采集 UDP v1 广播的身份字段固定为 18 字节，结构保持兼容；新版 19 位 SN 不会被截断塞入该字段，v1 广播改用 `MAC-` 临时身份。完整 SN 可通过 Avahi TXT 记录和 `CMD_ID_GET_SYS_SN` 查询取得；查询应答为 1 字节状态码和 19 字节 SN。需要让旧上位机读取完整 SN 时，应升级上位机解析该查询应答。

出厂写入由独立目标 `lidar_sn_provision` 承担，不进入第三分区应用升级包；完整系统镜像通过 rootfs overlay 将它安装到 `/usr/libexec/lidar/lidar_sn_provision`，设备启动 Linux 后由产线显式调用。Beta 镜像整理脚本会单独构建并纳入该工具。工具要求 `/sys/class/mtd/mtd3/name` 为 `sn`，且 `/dev/mtd3` 恰好是一个擦除块；仅接受空分区、同一 SN 的重复执行或目标 SN 的未完成写入重试，拒绝覆盖其他内容，并在写后回读。写入成功后须重启网络服务，让 `lidar_netd` 重新读取 SN 并更新 Avahi；固件启动、应用升级和 `lidar_netd` 均不自动写 SN。

## 地址、发现与兼容

地址配置使用原生 netlink，ARP 使用限定接口的 `AF_PACKET` 套接字。探测同时检测对方已占用和双方正在探测的冲突，运行中允许一次防御，短时间再次冲突则退让。连续冲突限速重试；过期、冲突的 DHCP 租约不会作为地址应用，也不会持续打断 Link-Local 回退。

DHCP 复用前台运行的 `udhcpc`，固定事件脚本只向网络服务报告 `bound/renew/deconfig/nak`，不执行 `ifconfig`。每次启动后端生成独立回调凭据，拒绝旧代回调；网络服务退出时清理受管进程组。

mDNS/DNS-SD 由系统 Avahi 组件处理，运行时关闭 D-Bus、IPv6 和反射，只服务指定接口，公布 `_lidar._tcp.local`、真实服务端口与 CGI 路径。名称冲突、缓存更新和接口变化交给成熟实现。Avahi 不是地址分配者。`lidar_netd` 启动 Avahi 时不传 `--no-chroot`；`enable-dbus=no` 仅在系统配置的 `avahi_dbus_supported=true` 时写入。镜像集成脚本读取目标 Avahi 的 `config.h` 中 `HAVE_DBUS`，写入该布尔值。相关配置能力见 [Avahi 官方配置说明](https://github.com/avahi/avahi/blob/master/man/avahi-daemon.conf.5.xml.in)。

这一选型替代初稿中“精简系统必须自写 mDNS”的假设：实际不需要引入完整 D-Bus 客户端体系；以最终镜像实测体积决定系统预算，不引用验证稿中的估算值。

UDP 5354 每 5 秒广播一次带 `protocol=lidar-network` 和 `schema_version=1` 的 JSON，含身份、MAC、地址、来源、恢复状态和管理入口。其格式与同事验证脚本的 JSON 有区别，上位机应按此版本对接。mDNS 初始化失败不关闭广播通道。

原有 55000 广播及控制协议数据格式保留。固定 IP 指令转入同一网络事务，控制线程异步等待探测结果，继续处理心跳；探测成功后先通过旧地址 ACK，再切换。托管控制接收保留单播和广播，通过 `recvmsg/IP_PKTINFO` 取得每个包实际到达的本机目标地址，丢弃旧地址排队的单播包，仅在新 IP 的有效单播握手时确认改址；广播握手不能误确认。旧上位机需允许约 10 秒的设置响应等待，并在新 IP 上进行单播握手；此行为尚待实际旧上位机验收。

UDP 控制和广播发现已接入；TCP 收流等尚未完成实机兼容验收，不能据此认定全部旧工具已通过。地址设置不重置 FPGA，不承诺收流不中断。

## 构建与镜像集成

```sh
cmake -S network -B build/network-host -G Ninja
cmake --build build/network-host
python3 scripts/test_lidar_network.py
python3 scripts/test_lidar_mgrd.py
python3 scripts/test_boot_integration.py
```

`--simulation` 只运行内存网卡后端并加速状态机时钟，不创建或更改实际网卡；响应显式标识 `simulation=true`。该参数绝不能放进产品启动入口。

完整 Beta 构建同时生成 ARM64 网络工具和 CGI。签名应用构建阶段整理出系统组件；离线启动包生成工具再将系统组件复制到 overlay，实际系统运行固定副本。日常业务包更新不覆盖系统网络服务。

镜像依赖：BusyBox `udhcpc`、原有 runit 工具、内核 `AF_PACKET` 和 IPv4；另外启用 Buildroot Avahi daemon 及其实际运行依赖。无 D-Bus 的 Avahi 构建不生成 `enable-dbus` 键；有 D-Bus 的构建显式设置 `enable-dbus=no`。不要求部署 D-Bus 客户端或开启系统 D-Bus 服务。保留 Avahi 包的配置和服务目录，许可证及对应源码按系统组件统一交付。

可合入的配置片段位于 [deploy/network/](../deploy/network/)，已根据本机 Buildroot 的 Avahi 0.8 包定义核对选项。将片段合入实际有效的 Buildroot、BusyBox 和内核配置后重新解析依赖与构建，不要用片段覆盖完整配置。生成的网络启动集成包也附带 `network-config-fragments/`；本次没有修改外部 Buildroot 目录。

网络组件齐全的新包会设置 `LIDAR_NETWORK_ENABLED=1`；旧包缺少整套组件时保持原启动方式，不生成半启用配置。启用后的离线镜像合入会：

- 验证网络二进制和镜像内 `udhcpc`、`avahi-daemon` 存在且可执行。
- 备份本机旧 `/etc/network/interfaces` 到 `/etc/lidar/legacy-network.interfaces`，从系统 ifup 配置中移除受管的 `eth0` IPv4 定义，保留其他接口。
- 将原 Avahi S 启动入口移到离线备份目录，交由网络服务启动唯一实例；其他自动配置入口需先明确迁移。
- `S97` 准备网络服务，`S98` 准备管理服务；rcS 返回后由同一 `runsvdir` 并行监督。管理启动只等待网络 API 可查询，不在 rcS 中等待监督树。
- 关机先停管理与采集，再停网络与受管后端，最后卸载存储。网络服务仍在运行时拒绝卸载。

主机已验证配置规则、标准探测时序、同时探测冲突、范围耗尽、固定配置迁移、持续冲突、防御与恢复、DHCP 续租及冲突回退、旧协议准备和确认流程、状态写入失败、重启回滚、HTTP 来源和字段校验、管理仲裁、离线镜像接入及重复合入。真实 ARP/netlink、mDNS 重名互通、多网卡、旧上位机和 JFFS2 断电仍需专用板验证；当前执行环境不允许创建隔离网络命名空间。
