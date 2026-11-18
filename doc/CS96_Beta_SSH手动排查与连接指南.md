# CS96 Beta：SSH 手动排查、连接与回滚指南

本文适用于本仓库的 `cloud_viewer` 和 CS96 Beta 雷达。排查顺序是：**确认目标与网络 → 发现广播 → SSH 与设备进程 → 驱动模式（`mode`）与推流配置 → 接收验证 → 必要时回滚**。以下示例以 `10.42.0.222` 为目标，实际 IP 以现场确认值为准。

> 执行任何写配置、启停进程或启用服务的命令前，先确认 IP 对应的是自己的目标设备。同一网段可能有多台雷达同时在广播，**软件已不再内置地址黑名单**，连接哪一台完全由你从搜索结果中选择——请自行确认目标归属。不要同时对多台设备使用同一份配置或标定表。

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

**"设备重启后突然搜不到"绝大多数不是网络问题，而是雷达应用没运行。** 该固件的开机脚本 `app_startup.sh` 里启动 `you_see` 的那一行是注释状态：

```sh
#    chmod +x /root/you_see && /root/you_see &
```

也就是说**设备重启后不会自动推流，也不会发发现广播**。不依赖抓包权限的快速判别：

```bash
# [本机] 22 通 + 65000 不通 + 50007 无数据 ⇒ 主机在线但应用未运行
for p in 22 65000 50007; do
  timeout 2 bash -c "</dev/tcp/$DEVICE/$p" 2>/dev/null && echo "$p OPEN" || echo "$p closed"
done
```

`65000`/`50007` 显示 closed 时，直接跳到第 4 节登录确认，再按第 6 节末尾的启动命令拉起进程（这种情况下无需改任何配置），不必在广播抓包上继续耗时间。

### 2.1 软件内置的两级发现（不必手工抓包）

**Search Device** 现在是两级：

1. **监听广播**（UDP 55000，2.2 秒）——正常情况下走这条。
2. **主动扫描兜底**（广播无结果时自动触发）——不依赖雷达上的任何应用：
   - 对本机所有 `/24` 网段并行 `ping`，目的只是让内核发出 ARP；
   - 读 `/proc/net/arp` 取回 MAC，按厂商标识筛选；
   - 对命中的地址探测 TCP 22，判断能否远程配置。

设备按钮上的标记：

| 标记 | 含义 |
|---|---|
| `CS96 Beta 10.42.0.222 96x800` | 广播发现，元数据完整，可直接连接 |
| `10.42.0.222 [scan+ssh]` | 扫描发现且 SSH 可用；点击会连带纠正 `center_ip` 并拉起 `you_see` |
| `10.42.0.222 [scan/no-ssh]` | 扫描到但 22 不通；点击时会先尝试用 web 接口开启 SSH，失败才提示现场处理 |

配套开关：

```bash
# 不同批次雷达的 MAC 前缀可能不同，可配多个（逗号分隔）；设为空串则列出全部邻居
CV_RADAR_MAC_PREFIX='98:03:8a,e0:be:03' ./build/cloud_viewer

# 跳过广播、直接扫描，用于排障
CV_FORCE_SCAN=1 ./build/cloud_viewer
```

实测参考：一个含 105 台邻居的网段，扫描耗时约 5～7 秒；`98:03:8a` 命中 1 台（即本机接的 CS96 Beta），`e0:be:03` 命中 10 台但全部 `no-ssh`，说明 SSH 探测能有效把"同类 FPGA 设备"和"真雷达"区分开。

### 2.2 独立的"Restart you_see"按钮

工具栏第二行的 **Restart you_see** 对当前目标设备（最近连接/最近操作的那台，没有则取搜索结果第一台）**按顺序做三件事**：

1. **检查 SSH**：TCP 22 可连就直接进入下一步。
2. **SSH 关闭时自动开启**：向设备自带 web 接口发 `POST /cgi-bin/set_ssh.cgi`（body `enable=true`），该 CGI 内部执行 `S50sshd start`；随后轮询 22 端口，最多等 10 秒。
3. **重启 you_see**（先 kill 已有进程再拉起，不改任何配置）：

```sh
old=$(pidof you_see || true)
if [ -n "$old" ]; then
  kill -9 $old
  sleep 2                      # 等串口/采集句柄释放
  left=$(pidof you_see || true)
  [ -z "$left" ] || { kill -9 $left; sleep 1; }
fi
setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &
sleep 4
pid=$(pidof you_see || true)
[ -n "$pid" ] || echo RESTART_FAILED
```

为什么必须"先杀后启"：改过 `/etc/ys_conf.ini`、换过 `work_mode`、或设备端进程卡在旧配置上时，
旧逻辑（`pidof` 有输出就算"已在运行"、直接跳过）不会让新配置生效。重启能保证进程用当前
配置文件重新初始化。

`sleep 2` 不能省：刚被 `kill -9` 时串口/采集句柄可能还没释放，立刻拉起常报设备忙并静默退出，
表面是 `RESTART_FAILED` 但日志里没有明显错误行。

之所以能远程开启 SSH，是因为设备的 `/www` 由 **lighttpd 独立提供**，不依赖 sshd；`set_ssh.cgi` 的核心逻辑就是：

```sh
if [ "$SERVICE_STATUS" = "true" ]; then
    sed -i "s/#start/start/g" /etc/init.d/S50sshd && /etc/init.d/S50sshd start
```

实测（10.42.0.221，22 端口正常时走"已就绪"分支）：

```
[ui] Restart you_see OK | 检查 10.42.0.221 的 SSH 服务
[ui] Restart you_see OK | SSH 已就绪（10.42.0.221）
[ui] Restart you_see OK | 已重启设备端 you_see（10.42.0.221，旧 pid 已终止）
[ui] Restart you_see OK | 稍等约 3 秒再点 Search，设备开始广播后即可连接
```

SSH 关闭时点按钮会先自动开启再重启，第 2 行变成：

```
[ui] Restart you_see OK | SSH 未开放，已通过设备 web 接口开启（10.42.0.221）
```

如果 web 接口本身也不可达（设备刚上电、lighttpd 还没起），会明确提示"请稍后再试或重新上电"，不会静默失败。日志的三种终态：

```
[ui] Restart you_see OK     | 已重启设备端 you_see（10.42.0.221，旧 pid 已终止）
[ui] Restart you_see OK     | 设备端 you_see 原本未运行，已启动（10.42.0.221）
[ui] Restart you_see FAILED | 重启 you_see 失败: <设备日志中的真实原因>
```

失败原因会从设备 `/var/log/ys.log` 里带回关键行，例如固件不支持当前 `work_mode`（见 4.2）。

**重启会中断推流约 5~8 秒**（2 秒等句柄释放 + 4 秒拉起校验）。实测设备端 pid 由 `2163` → `2567`，
重新连接后 `Radar Live` 首帧 76800 点、稳定 10 fps。校验通过后再点 **Search**，
等设备重新广播即可连接。

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

### 3.1 主机密钥变更（换雷达或重刷系统）

`REMOTE HOST IDENTIFICATION HAS CHANGED` 在**换设备、重刷固件、恢复出厂**后会稳定复现，因为主机密钥随系统镜像重建。先确认是不是真的换了设备（对比指纹），再清理条目。**这里有两个互相独立的文件**，不要只清一个：

| 用途 | 文件 |
|---|---|
| 你手动 `ssh` 登录 | `~/.ssh/known_hosts` |
| `cloud_viewer` 专用 | `~/.config/cloud_viewer/beta_known_hosts`（可用 `CV_BETA_KNOWN_HOSTS` 覆盖） |

```bash
# [本机] 只删除该 IP 的旧条目，不影响其他主机。
ssh-keygen -R 10.42.0.222
ssh-keygen -R 10.42.0.222 -f ~/.config/cloud_viewer/beta_known_hosts
```

`cloud_viewer` 已内置自动恢复：连接时若命中该告警，会删除**该 IP 一条**旧记录，用 `StrictHostKeyChecking=accept-new` 重试一次，并在日志明确提示：

```
[HOST-KEY] 设备主机密钥已变更（换了雷达或设备重刷固件），已更新信任并自动重试
```

所以"点连接失败、报一堆 `@@@@ WARNING`"不再是需要人工处理的问题；但**自动恢复只作用于该 IP**，不会关闭主机密钥校验，也不会掩盖其它设备的异常。若日志反复出现该提示而你没有换过设备，应当按上面的原则查明原因，而不是忽略。

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

lsmod | head -5                      # 驱动是否已加载：point_cloud → yousee_lidar
mount | grep mtd || echo 'mtd 未挂载（多数情况下正常，见 4.1）'
md5sum /root/you_see                 # 与 /mnt/mtdblock2/*/sbin/you_see 对比，判断来自哪个构建包

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
| `work_mode` | 0=直方图、1=原始点云、2=真实点云、4=灰度。**合法取值由 `mode` 文件决定**，见 4.2。 |
| `/mnt/mtdblock2/mode` | 决定加载哪套驱动与库（`histogram` / `point_cloud` / `point_cloud_gray`），必须与 `work_mode` 匹配。 |
| `/mnt/mtdblock2` 内容为空 | 正常现象：开机脚本执行完会 `umount` 该分区，见 4.1。 |
| `/etc/init.d/S99user` | 先看本机设备脚本；有的固件 `stop/restart` 只是空实现，不能据此认为进程已重启。 |

**`work_mode` 不是可以随手改的开关。**固件在编译期固定了数据模式，配置值必须与 `/mnt/mtdblock2/mode` 所选驱动一致，否则 `you_see` 启动即退出。曾观察到“改成 2 就报不匹配”，那是因为当时设备加载的是灰度驱动；同一台设备切换 `mode` 之后结论会变，不能套用。对应关系与实测现象见 4.2。

查看当前设备标定，但不要把内容直接覆盖其他设备的标定：

```sh
# [雷达]
grep -A 6 '^\[standard_encoder\]' /etc/ys_calibration.ini
grep -A 6 '^\[side_angular_offset\]' /etc/ys_calibration.ini
ls -l /etc/ys_encoder_angle.csv /etc/spot_slope_map.csv 2>/dev/null
```

真实点云模式的 xyz 由设备计算；原始点云在 PC 解算时需要**当前雷达对应的标定**。序列号相同也不等于标定相同。

### 4.1 启动链路与分区状态

应用文件放在 jffs2 分区 `/dev/mtdblock2`（`mtd2 "user"`，约 15.9 MB）上。实测的启动链路：

```
/etc/init.d/S99user
  └─ mount /dev/mtdblock2 /mnt/mtdblock2
      ├─ /mnt/mtdblock2/app_startup.sh start
      │    ├─ date -s 202701010000
      │    ├─ insmod /mnt/mtdblock2/$(cat mode)/*.ko
      │    ├─ cp /mnt/mtdblock2/$(cat mode)/liblidar.so /usr/lib/ -f
      │    ├─ cp /mnt/mtdblock2/interfaces /etc/network/ && ifup eth0
      │    └─ for dir in /mnt/mtdblock2/*/; do sh "$dir"/install.sh; done
      │         ├─ you_see_arm/install.sh       → cp config/* /etc/ ; cp sbin/you_see /root/
      │         └─ you_see_arm_gray/install.sh  → 同上，后执行者覆盖前者
      └─ sync && umount /mnt/mtdblock2        ← 脚本结束立即卸载
```

由此得到几条容易误判的现象：

- **登录后 `ls /mnt/mtdblock2` 为空是正常的**，不是分区损坏——开机脚本跑完就卸载了。
- 此时手动 `mount -t jffs2 /dev/mtdblock2 /mnt/mtdblock2` 若报 `Device or resource busy`，说明仍在挂载/占用中，先用 `mount | grep mtd` 确认，不要反复重试。
- 需要看文件时临时挂载，看完卸载：

```sh
# [雷达] 只读查看
mount -t jffs2 /dev/mtdblock2 /mnt/mtdblock2
ls /mnt/mtdblock2
cat /mnt/mtdblock2/mode
umount /mnt/mtdblock2
```

- `app_startup.sh` 第一件事是 `date -s 202701010000`，所以设备时间被固定为 2027-01-01。`ys.log` 的时间戳只能看相对顺序，**不能与本机时间对齐**。
- 驱动是否加载成功可看 `lsmod`：`point_cloud` 对应模块名 `yousee_lidar`。
- **`app_startup.sh` 不会启动 `you_see`**（那一行 `#    chmod +x /root/you_see && /root/you_see &` 是注释状态），所以每次重启后都要由上位机连接时拉起，或按第 6 节手动启动。

### 4.2 `mode` 与 `work_mode` 必须匹配

`/mnt/mtdblock2/mode` 决定加载哪套驱动与库，`work_mode` 必须与之对应，**只改 `/etc/ys_conf.ini` 是改不动驱动层的**：

| `mode` 内容 | 驱动 + 库 | `/root/you_see` | 允许的 `work_mode` |
|---|---|---|---|
| `histogram` | `yousee_lidar_histogram.ko` + `histogram/liblidar.so` | `you_see_arm` | `0`（原始直方图） |
| `point_cloud`（默认） | `yousee_lidar.ko` + `point_cloud/liblidar.so` | `you_see_arm` | `1` / `2` |
| `point_cloud_gray` | `point_cloud_gray/*.ko` | `you_see_arm_gray` | `4`（灰度，协议不同） |

实测：在 `mode=point_cloud` 下把 `work_mode` 改成 `0`，`you_see` 启动即退出，`/var/log/ys.log` 中：

```
ERROR ys_conf.c:407: 工作模式 0 与当前固件数据模式不匹配，请使用同一构建包中的配置
FATAL main.c:78: load config fail
```

固件在编译期固定了数据模式，**软件侧无法绕过**。要跑直方图必须切驱动层并整机重启：

```sh
# [雷达] 改 mode 后需要重启，才会重新 insmod 并拷贝对应的 liblidar.so
echo histogram > /mnt/mtdblock2/mode
```

> 切换前先挂载分区、备份 `mode` 原值，并向厂商确认三个 `mode` 与构建包的配套关系。`install.sh` 的覆盖顺序决定 `/root/you_see` 最终是哪一份，切换后仍需实测。

`cloud_viewer` 侧已按 `mode` 限制可选模式：连接时会读取并回传，日志形如 `DEVICE_MODE=point_cloud`；对不支持的组合直接跳过，不再反复重启设备：

```
[ui] Mode FAILED | 设备 mode=point_cloud 不支持 work_mode=0，已跳过
```

> 为什么改 `mode` 会改变合法 `work_mode`：这三个值对应芯片 DSP 的不同输出模式
> （`0` = RAW 全直方图，`1`/`2` = RANGING 峰值点云），驱动层决定了芯片往哪条路走。
> 芯片层细节见 [`sl100_chip_notes.md`](./sl100_chip_notes.md) 第 3 节。

### 4.3 `/etc/ys_conf.ini` 每次开机都会被覆盖

`install.sh` 执行 `cp -rf $APP_PATH/config/* /etc/`，即**每次开机都用构建包里的配置覆盖 `/etc/ys_conf.ini`**。实测证据：`/etc/ys_calibration.ini` 的修改时间恰好等于开机时刻，而 `/etc/ys_conf.ini` 只有被上位机改过之后才会变。

出厂配套值（`/mnt/mtdblock2/you_see_arm/config/ys_conf.ini`）常见为：

```
work_mode        = "2"
center_ip        = "10.42.0.145"     ← 上一台上位机
center_data_port = "50037"
```

所以**设备重启后会重新指向旧电脑**，表现为"重启后又收不到点云、但 SSH 能连"。`cloud_viewer` 每次连接都会核对并自动纠正，日志会打印 `CENTER_UPDATED=旧值-新值`；若希望掉电重启后依然指向本机，需要改持久化副本：

```sh
# [雷达] 先挂载，备份后再改；IP/端口替换为实际值
mount -t jffs2 /dev/mtdblock2 /mnt/mtdblock2
cp -p /mnt/mtdblock2/you_see_arm/config/ys_conf.ini /root/ys_conf.persist.bak

sed -i 's|^center_ip[[:space:]]*=.*|center_ip                      = "10.42.0.229" ;上位机IP|' \
  /mnt/mtdblock2/you_see_arm/config/ys_conf.ini
sed -i 's|^center_data_port[[:space:]]*=.*|center_data_port               = "50007" ;上位机的数据端口|' \
  /mnt/mtdblock2/you_see_arm/config/ys_conf.ini

grep -E '^[[:space:]]*(center_ip|center_data_port)' /mnt/mtdblock2/you_see_arm/config/ys_conf.ini
sync && umount /mnt/mtdblock2
```

> 这等于修改出厂配置。先确认当前设备确实由 `you_see_arm` 提供程序（对比 `/root/you_see` 与该目录下 `sbin/you_see` 的 md5），再动手；灰度包对应的是 `you_see_arm_gray`。

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

> 如果只是设备重启后 `you_see` 没在运行、配置本身是正确的（`center_ip` 已指向本机），**不需要改配置**，直接执行本节最后的启动命令即可。

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

配置是否热加载取决于固件；此处按现场已验证的 `/root/you_see` 启动方式重启。**设备刚重启时 `pidof you_see` 为空是正常状态**（见 4.1），下面的命令在无进程时会直接启动。先用 `pidof` 精确获取目标进程，避免用 `ps | grep` 把 SSH 自身误当目标。`kill -9` 是强制终止，只在明确需要重启且确认目标后使用。

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
./build/cloud_viewer
```

在界面点击 **Search Device**，或使用 **Connect Beta IP** 输入雷达 IP。也可从终端直接指定目标并启用统计：

```bash
# [本机] DEVICE、DATA_PORT 需沿用第 1 节实际值。
CV_BETA_IP="$DEVICE" CV_BETA_PORT="$DATA_PORT" CV_BETA_DEBUG=1 ./build/cloud_viewer
```

检查 `Beta Target`、`Radar Live` 以及 `[beta] recv/parsed/timeouts/frames`。仅有截图文件或 `pidof` 不等于收到有效点云；要同时确认设备模式、来源 IP、包格式和有效点数。

`Beta Target` 行会带上 `DEVICE_MODE=<mode>`，可与 4.2 的表格对照；若出现 `CENTER_UPDATED=旧值-新值`，表示设备推流目标被自动纠正（原因见 4.3）。

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

若之前按 4.3 改过**持久化配置**（`/mnt/mtdblock2/*/config/ys_conf.ini`），回滚时必须一并恢复，否则下次开机又会覆盖 `/etc/ys_conf.ini`：

```sh
# [雷达]
mount -t jffs2 /dev/mtdblock2 /mnt/mtdblock2
cp -p /root/ys_conf.persist.bak /mnt/mtdblock2/you_see_arm/config/ys_conf.ini
sync && umount /mnt/mtdblock2
```

## 9. 常见问题速查

| 现象 | 首先核对 |
|---|---|
| **设备重启后突然搜不到，但 ping 通** | `you_see` 未运行（开机不自启）。Search 会自动扫描兜底，选中设备后点 **Restart you_see** 即可（进程没在跑时会直接拉起）；手工判别见 2.1 与 4.1。 |
| ping 通，Search 无结果 | UDP 55000 抓包、`you_see` 是否运行、广播端口与网络过滤；已知 IP 可走手动入口。 |
| 重启后点云又推到旧电脑 | `/etc/ys_conf.ini` 被 `install.sh` 覆盖回出厂值，见 4.3。 |
| 改 `work_mode` 后进程立刻退出 | 与 `mode` 所选驱动不匹配，见 4.2。 |
| 连接日志出现 `[HOST-KEY] ...` | 换了设备或重刷过系统；软件已自动更新该 IP 的信任，但 `~/.ssh/known_hosts` 需你自行 `ssh-keygen -R`，见 3.1。 |
| SSH `Connection refused` | SSH 是否启用、端口是否为 22；Web 可用不代表 SSH 开启。 |
| SSH 提示主机密钥改变 | 重新核实设备身份与指纹，再更新对应条目；不要忽略告警，见 3.1。 |
| Search 找到设备但无点 | `center_ip`、数据端口、`work_mode`、固件兼容性、本机抓包和查看器接收统计。 |
| 配置写入后进程退出 | 立刻看日志，核对固件和模式；按备份回滚并确认旧进程恢复。 |
| ON/OFF 距离无预期变化 | 确认处于原始模式、加载的是该设备标定、开关作用于当前帧；真实点云 xyz 由设备提供，PC 原始标定开关不应被当作设备侧标定开关。 |
