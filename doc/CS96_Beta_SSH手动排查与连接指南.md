# CS96 Beta：SSH 手动排查、连接与回滚指南

本文适用于本仓库的 `cloud_viewer` 和 CS96 Beta 雷达。排查顺序是：**确认目标与网络 → 发现广播 → SSH 与设备进程 → 模式与推流配置 → 接收验证 → 必要时回滚**。以下示例以 `10.42.0.222` 为目标，实际 IP 以现场确认值为准。

> 执行任何写配置、启停进程或启用服务的命令前，先确认 IP 对应的是自己的目标设备。`10.42.0.221` 是他人设备，本示例不连接、不修改它。不要同时对多台设备使用同一份配置或标定表。

## 1. 本机：设置变量并确认网络

以下 `[本机]` 命令在运行 `cloud_viewer` 的 Linux 终端执行；`[雷达]` 命令在 SSH 登录后的雷达 shell 中执行。`DEVICE`、`HOST_IP` 仅是本机当前终端的变量，不会自动传到雷达。

```bash
# [本机] 修改为你确认过的雷达 IP 和本机接收端口。
DEVICE=10.42.0.222
DATA_PORT=50007
REPO=/media/baiusc/DATA11/Repositories/ReposLidar/pcl_viewer
cd "$REPO" || exit 1

# 接口地址、到目标的路由、连通性。
ip -br addr
ip route get "$DEVICE"
ping -c 3 -W 1 "$DEVICE"
```

在 `ip route get` 输出中找到 `dev <网卡>` 和 `src <地址>`。例如 `dev enp4s0 src 10.42.0.229`，则 `enp4s0` 是该路由使用的网卡，`10.42.0.229` 是本机朝该雷达发包的源地址。请按**实际输出**设置，后文不假定网卡或地址固定：

```bash
# [本机] 示例；先替换为上一条实际输出的 dev 和 src。
IFACE=enp4s0
HOST_IP=10.42.0.229
printf '目标雷达=%s  本机=%s  网卡=%s  接收端口=%s\n' "$DEVICE" "$HOST_IP" "$IFACE" "$DATA_PORT"
```

`ping` 通只说明 ICMP 可达，**不代表发现广播、SSH、雷达应用进程或点云推流正常**。若 ping 失败，先检查供电、网线、子网掩码、路由和防火墙。

## 2. 本机：检查发现广播

Search 是被动监听 UDP `55000`，并非按 IP 主动扫描；雷达应用未运行、广播被过滤、广播端口不同等都可能导致搜不到。

```bash
# [本机] 先看监听者；有其他查看器时，结果需结合实际端口共享情况判断。
ss -lunp | grep ':55000 ' || true

# [本机] 抓取该雷达发出的发现报文；tcpdump 需相应权限。
# -i 要换成第 1 节实际网卡；超时约 12 秒，最多显示 10 个包。
sudo timeout 12 tcpdump -i "$IFACE" -nn -c 10 "src host $DEVICE and udp dst port 55000"
```

- 看到从目标 IP 到 UDP 55000 的报文：检查查看器 Search 结果、设备识别和监听端口。
- 看不到报文：继续 SSH 检查 `you_see`、`center_broadcast_port` 和日志；同时排查网络隔离、防火墙、实际广播端口。**抓不到包不是单独证明设备进程未启动**。
- 报权限错误：先解决抓包权限；不在命令行明文放 sudo 密码。

也可用手动入口 **Connect Beta IP** 输入已确认地址。手动入口只绕过广播发现，不会使关闭的服务、错误的推流地址或不兼容的固件自动变正常。

## 3. 本机：确认 SSH、主机密钥和登录

新雷达可能关闭 SSH。如果 TCP 22 连接被拒绝，可在设备 Web 页面 `http://<雷达IP>/` 的“服务配置”中检查 SSH 状态；经设备所有者确认后再启用。页面能打开不意味着点云进程正在运行。

首次连接前，应通过设备控制台或可信管理渠道核对主机密钥指纹。下面的 `ssh-keyscan` **只采集候选密钥，并不能证明身份**。例如把设备现场核对过的指纹和输出比对：

```bash
# [本机] 只查看候选指纹，不自动信任。
ssh-keyscan -T 5 -t ed25519 "$DEVICE" 2>/dev/null > /tmp/cs96_candidate_hostkey
ssh-keygen -lf /tmp/cs96_candidate_hostkey
```

确认指纹后，可以用通常的 `ssh root@<IP>` 按提示验证并保存；已有密钥发生变化时，应先查明原因，再更新对应条目，不要通过关闭主机密钥检查来跳过告警。密码在 SSH 提示后输入，不写进命令或脚本。

```bash
# [本机] 普通交互登录，root 是示例账号，以设备实际账号为准。
ssh "root@$DEVICE"
```

若需要与本查看器使用同一份已确认的专用主机密钥文件：

```bash
# [本机] 确认文件内存在经核验的目标密钥后使用。
KNOWN_HOSTS="$HOME/.config/cloud_viewer/beta_known_hosts"
ssh-keygen -F "$DEVICE" -f "$KNOWN_HOSTS"
ssh -o "UserKnownHostsFile=$KNOWN_HOSTS" -o StrictHostKeyChecking=yes "root@$DEVICE"
```

软件的 SSH 密码来源是环境变量 `CV_BETA_SSH_PASS` 或 `~/.config/cloud_viewer/beta_ssh.conf`；不要把真实密码提交进仓库或粘贴到公开日志。若专用密钥文件中没有这台设备的密钥，须在核验指纹后登记；不同 IP/设备各自核验。

## 4. 雷达：只读检查进程、启动方式、模式与配置

成功登录后，命令提示符已在雷达上，下面命令不再引用本机的 `$DEVICE`、`$HOST_IP`。先查看，不要立即改配置：

```sh
# [雷达]
hostname
ip addr 2>/dev/null || ifconfig
pidof you_see || true
ps | grep '[y]ou_see'
ls -l /root/you_see /etc/ys_conf.ini /etc/ys_calibration.ini
sed -n '1,100p' /etc/init.d/S99user

grep -E '^[[:space:]]*(work_mode|center_ip|center_data_port|center_broadcast_port|local_cmd_port|center_cmd_port|protocol)[[:space:]]*=' /etc/ys_conf.ini

tail -80 /var/log/ys.log 2>/dev/null
tail -80 /tmp/you_stdout.log 2>/dev/null
```

重点：

| 项目 | 含义与排查 |
|---|---|
| `pidof you_see` | 有 PID 表示该名称进程存在；仍需结合日志和实际 UDP 确认工作正常。 |
| `center_broadcast_port` | 发现广播目标端口；查看器默认监听 55000。 |
| `center_ip` | 设备发数据的目标地址，应为第 1 节的本机 `src`。 |
| `center_data_port` | 数据目标端口，应与查看器监听端口一致，默认 50007；`CV_BETA_PORT` 可改变查看器端口。 |
| `work_mode` | 此查看器已实现 0=直方图、1=原始点云、2=真实点云。新 `.222` 现场曾是 4=灰度；按固件实际配置判断。 |
| `/etc/init.d/S99user` | 先看本机设备脚本；有的固件 `stop/restart` 只是空实现，不能据此认为进程已重启。 |

**不同固件的可用模式可能不同。**曾在 `.222` 上将配置试为 2 时，设备日志报“工作模式 2 与当前固件数据模式不匹配”，进程退出并已恢复原配置。不要把 `work_mode=4` 直接改成 2 当作通用修复；应先核实设备端固件、FPGA 数据模式和配置来自匹配构建包。

查看当前设备标定，但不要把内容直接覆盖其他设备的标定：

```sh
# [雷达]
grep -A 6 '^\[standard_encoder\]' /etc/ys_calibration.ini
grep -A 6 '^\[side_angular_offset\]' /etc/ys_calibration.ini
ls -l /etc/ys_encoder_angle.csv /etc/spot_slope_map.csv 2>/dev/null
```

真实点云模式的 xyz 由设备计算；原始点云在 PC 解算时需要**当前雷达对应的标定**。序列号相同也不等于标定相同。

## 5. 修改前备份配置

以下操作会改变目标设备。先确认实际目标，保存备份，并核对输出：

```sh
# [雷达] 按时间命名；备份路径记录在当前 SSH 会话变量中。
BACKUP="/tmp/ys_conf.before.$(date +%Y%m%d_%H%M%S).ini"
cp -p /etc/ys_conf.ini "$BACKUP" && echo "备份文件: $BACKUP"
cmp /etc/ys_conf.ini "$BACKUP" && echo '备份字节一致'
grep -E '^[[:space:]]*(work_mode|center_ip|center_data_port)[[:space:]]*=' "$BACKUP"
```

`/tmp` 可能在设备重启后清空。若要跨设备重启保留备份，可在**本机新终端**执行 `scp`，并按第 3 节使用已确认的 SSH 密钥；远端路径替换为上一步实际打印的路径：

```bash
# [本机另一个终端] 将路径替换成第 5 节打印的完整备份路径。
DEVICE=10.42.0.222
REMOTE_BACKUP=/tmp/ys_conf.before_实际文件名.ini
scp -o "UserKnownHostsFile=$HOME/.config/cloud_viewer/beta_known_hosts" \
  -o StrictHostKeyChecking=yes "root@$DEVICE:$REMOTE_BACKUP" ./ys_conf.device-backup.ini
```

## 6. 雷达：修改推流目标并重启进程

仅在已确认该固件当前模式受查看器支持、且目标 IP/端口确实不一致时操作。此例只修改两项网络目标，**不修改 work_mode、标定和发现端口**。将下面两个值替换成第 1 节实际本机 IP 和软件监听端口：

```sh
# [雷达] 与本机实际路由和监听端口一致；不要写入雷达自身的 IP。
HOST_IP=10.42.0.229
DATA_PORT=50007

# 先验证每个字段恰好一行且为双引号配置，再生成临时文件。
awk '/^[[:space:]]*center_ip[[:space:]]*=/{n++} END{print "center_ip 行数:", n+0}' /etc/ys_conf.ini
awk '/^[[:space:]]*center_data_port[[:space:]]*=/{n++} END{print "center_data_port 行数:", n+0}' /etc/ys_conf.ini

tmp=/tmp/ys_conf.pending.ini
awk -v ip="$HOST_IP" -v port="$DATA_PORT" '
/^[[:space:]]*center_ip[[:space:]]*=/{sub(/"[^"]*"/, "\"" ip "\"")}
/^[[:space:]]*center_data_port[[:space:]]*=/{sub(/"[^"]*"/, "\"" port "\"")}
{print}' /etc/ys_conf.ini > "$tmp"
grep -E '^[[:space:]]*(work_mode|center_ip|center_data_port)[[:space:]]*=' "$tmp"
```

检查上述两项行数均为 1、临时文件内容无误且第 5 节备份已保存后，再执行：

```sh
# [雷达] 先人工复核临时文件；拷贝前再次确认目标设备。
cp "$tmp" /etc/ys_conf.ini
cmp /etc/ys_conf.ini "$tmp" && echo '配置写入一致'
```

配置是否热加载取决于固件；此处按现场已验证的 `/root/you_see` 启动方式重启。先用 `pidof` 精确获取目标进程，避免用 `ps | grep` 把 SSH 自身误当目标。`kill -9` 是强制终止，只在明确需要重启且确认目标后使用。

```sh
# [雷达] 先查看 PID，再结束该程序；无 PID 时跳过结束步骤。
pidof you_see || true
pids=$(pidof you_see || true)
if [ -n "$pids" ]; then kill -9 $pids; fi
sleep 1
setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &
sleep 3
pidof you_see || echo '进程未启动，请立即看日志并按第 8 节回滚'
tail -40 /tmp/you_stdout.log 2>/dev/null
tail -40 /var/log/ys.log 2>/dev/null
```

后台命令返回、打印了启动信息，或 `pidof` 短暂有值都不是完整成功标准；还要检查进程持续存在、日志无致命错误以及数据确实到达本机。其他固件的工作目录/启动方式应以该设备实际脚本和程序要求为准。

## 7. 本机：验证数据与查看器

退出 SSH 或另开本机终端，使用第 1 节的 `DEVICE`、`HOST_IP`、`IFACE` 和 `DATA_PORT`。确保查看器监听端口与设备 `center_data_port` 一致。

```bash
# [本机] 查看 UDP 监听情况。
ss -lunp | grep ":${DATA_PORT} " || true

# [本机] 抓目标雷达到本机的真实数据；抓包观察不会作为 UDP socket 抢走查看器数据。
sudo timeout 12 tcpdump -i "$IFACE" -nn -c 10 \
  "src host $DEVICE and dst host $HOST_IP and udp dst port $DATA_PORT"
```

启动查看器（确保变量已在此本机终端设置）：

```bash
# [本机]
cd /media/baiusc/DATA11/Repositories/ReposLidar/pcl_viewer || exit 1
./build-lite/cloud_viewer
```

在界面点击 **Search Device**，或使用 **Connect Beta IP** 输入雷达 IP。也可从终端直接指定目标并启用统计：

```bash
# [本机] DEVICE、DATA_PORT 需沿用第 1 节实际值。
CV_BETA_IP="$DEVICE" CV_BETA_PORT="$DATA_PORT" CV_BETA_DEBUG=1 ./build-lite/cloud_viewer
```

检查 `Beta Target`、`Radar Live` 以及 `[beta] recv/parsed/timeouts/frames`。仅有截图文件或 `pidof` 不等于收到有效点云；要同时确认设备模式、来源 IP、包格式和有效点数。

已有协议中的常见 UDP 包长：真实点云 1266 B、原始点云 1561 B、直方图 4105 B。灰度模式是另一条协议，**不要按上述三种点云包长猜测其含义**。如存在其他监听程序，留意它们可能与查看器争用 UDP 单播数据。

## 8. 回滚配置并恢复原模式

若更改后进程起不来或不能收流，按第 5 节备份回滚。以下命令在**同一雷达 SSH 会话**使用第 5 节的 `$BACKUP`；如果另开会话，先手动将 `BACKUP` 设置为实际备份完整路径。恢复前再次确认目标设备。

```sh
# [雷达] 替换为真实备份路径；同一会话可沿用已有 BACKUP。
echo "将恢复: $BACKUP"
test -f "$BACKUP" || { echo '备份不存在，停止'; exit 1; }
cp -p "$BACKUP" /etc/ys_conf.ini
cmp "$BACKUP" /etc/ys_conf.ini && echo '配置已逐字节恢复'

pids=$(pidof you_see || true)
if [ -n "$pids" ]; then kill -9 $pids; fi
sleep 1
setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &
sleep 3
pidof you_see || echo '原模式进程未启动；检查日志'
grep -E '^[[:space:]]*(work_mode|center_ip|center_data_port)[[:space:]]*=' /etc/ys_conf.ini
tail -40 /var/log/ys.log 2>/dev/null
```

如果备份只保存在本机，先用 `scp` 将它复制回**已确认的目标设备**的临时路径，再按上述流程恢复。修改软件代码后的回滚与设备配置回滚是两件事，分别执行。

## 9. 常见问题速查

| 现象 | 首先核对 |
|---|---|
| ping 通，Search 无结果 | UDP 55000 抓包、`you_see` 是否运行、广播端口与网络过滤；已知 IP 可走手动入口。 |
| SSH `Connection refused` | SSH 是否启用、端口是否为 22；Web 可用不代表 SSH 开启。 |
| SSH 提示主机密钥改变 | 重新核实设备身份与指纹，再更新对应条目；不要忽略告警。 |
| Search 找到设备但无点 | `center_ip`、数据端口、`work_mode`、固件兼容性、本机抓包和查看器接收统计。 |
| 配置写入后进程退出 | 立刻看日志，核对固件和模式；按备份回滚并确认旧进程恢复。 |
| ON/OFF 距离无预期变化 | 确认处于原始模式、加载的是该设备标定、开关作用于当前帧；真实点云 xyz 由设备提供，PC 原始标定开关不应被当作设备侧标定开关。 |
