# 激光雷达自动寻址与独占调试系统 · 系统说明

版本 v1.1 ｜ 2026-11-03 ｜ 对应代码 `lidar_ll/`（下位机 `lidar_agent` + 上位机 PowerShell 脚本）

> v1.1 更新：Link-Local 与 mDNS 补充完整原理；新增「命令手册（PC / Ubuntu / 下位机）」与「调试验证步骤」两章。

---

## 30 秒速览

> 多台雷达接在同一交换机上，IP 会撞、工程师会互相抢设备。
> 这套系统让每台雷达**开机自动拿到唯一地址**（没有 DHCP 也行），
> 用**名字**而不是 IP 来找它，并保证**同一时刻只有一个人能操作**。

| 层 | 解决什么 | 关键机制 | 对应章节 |
|---|---|---|---|
| 地址层 | 地址从哪来、会不会撞 | SN 派生 link-local + ACD 冲突检测 + DHCP 双地址 | 四、六 |
| 发现层 | 怎么找到它 | mDNS 名字 + UDP 广播心跳 | 五、六 |
| 会话层 | 谁在用 | 租约锁（8 秒自动释放） | 六 |

---

## 一、问题背景

```
   工程师 PC-1 ─┐
   工程师 PC-2 ─┼── 交换机 ──┬─ 雷达 1
   工程师 PC-3 ─┘            ├─ 雷达 2
                             └─ 雷达 N
```

四个具体痛点：

1. **地址冲突**：出厂同一个固定 IP，接同一网段必然打架，ARP 互相覆盖，表现为"时通时不通"。
2. **地址不可预测**：靠 DHCP 每次重启地址都变，没法贴标签、没法写工单。
3. **认不出是哪台**：知道有个 `10.42.0.31`，但货架上八台机器，哪台是它？
4. **互相抢设备**：两人同时改一台雷达的参数，改着改着数据不对，其实是对方动了。

目标：插上网线就能用，不需要现场配 IP、不需要交换机管理权限、不需要装额外软件。

---

## 二、总体结构

```
┌──────────────── 下位机（每台雷达，Buildroot 精简系统）────────────────┐
│  配置层   SETIP：上位机指定末段地址，设备先探测后应用     TCP 8080     │
│  会话层   租约锁 LOCK / RENEW / UNLOCK / STATUS           TCP 8080     │
│  发现层   mDNS（A/PTR/SRV/TXT） + UDP 广播心跳            UDP 5353/5354│
│  地址层   SN→link-local、ACD 探测/宣告/防御、DHCP 双地址   AF_PACKET    │
└───────────────────────────────────────────────────────────────────────┘
                                │ 二层广播域
┌──────────────── 上位机（Windows，PowerShell 脚本）──────────────────┐
│  lidar_scan.ps1   监听广播，实时设备列表（SN / IP / 状态 / 备用地址）  │
│  lidar_ctl.ps1    发命令：status / lock / hold / setip / identify     │
└───────────────────────────────────────────────────────────────────────┘
```

原则：**下层只负责"能被找到"，上层只负责"谁能用"**。任何一层出问题都能独立定位。

---

## 三、调度模型：为什么是单线程 poll

下位机是**一个进程、一个线程、一个 `poll()` 循环**，同时处理四类事件：

```
poll(arp_fd, mdns_fd, listen_fd, timeout=200ms)
 ├─ ARP 帧到达   → 是否有人抢我们的地址 → 是则 defend
 ├─ mDNS 查询    → 名字匹配则应答
 ├─ TCP 连接     → 读一行命令 → 回一行结果 → 关闭
 └─ 200ms 到期   → 租约到期？广播到期？待应用地址？ALIVE 日志？
```

选单线程而不是多线程的理由：

- 状态只有一份，不需要任何锁，**没有竞态**
- 代码量小，一屏能看完全部逻辑，便于评审
- 对 PREEMPT_RT 内核上的点云任务零干扰（不抢实时线程）

代价：一次只能处理一个 TCP 连接，且任何一条处理路径阻塞都会拖死全部。这条代价我们踩过一次很惨的坑（见 6.6）。

---

## 四、原理篇 A：Link-Local（RFC 3927）与 ACD（RFC 5227）

### 4.1 为什么存在 169.254/16

DHCP 需要一台服务器。当网络里**没有** DHCP（直连、隔离交换机、服务器挂了），设备既不能凭空造一个地址，也不能让用户手工配——手工配必然撞车。

RFC 3927 的办法是划出一段**专门用于链路本地、永不路由到公网**的地址：

```
169.254.0.0/16
  ├─ 169.254.0.0/24     保留，不可用
  ├─ 169.254.255.0/24   保留，不可用（广播/保留）
  └─ 169.254.1.1 ~ 169.254.254.254   可用
```

两个关键性质：

1. **这些地址不会被路由器转发**，所以撞了也不会污染外网
2. **不需要任何中心机构分配**，每台设备自己决定

代价是：**没人保证唯一**。所以必须自己解决冲突——这就是 ACD。

### 4.2 ACD 要解决的核心问题

假设设备 A 选定了 169.254.10.20，它怎么知道这个地址有没有人用？

直觉做法是"发一个 ARP 问一下"。但这里有个陷阱：

> 如果 A 用 169.254.10.20 作为发送方地址去发 ARP 请求，网络里所有主机都会把这个 IP 和 A 的 MAC 记进 ARP 缓存。
> 万一探测发现冲突、A 放弃了这个地址，**别人的缓存里就留下了一条错误记录**，而且会存活几分钟。

所以 RFC 5227 规定：**探测时发送方地址必须填 0.0.0.0**。

```
普通 ARP 请求                          ARP Probe
┌────────┬────────┬────────┐          ┌────────┬────────┬────────┐
│ SPA    │ TPA    │ 含义    │          │ SPA    │ TPA    │ 含义    │
├────────┼────────┼────────┤          ├────────┼────────┼────────┤
│ 我的IP │ 目标IP │ 我在问  │          │ 0.0.0.0│ 候选IP │ 我还没│
│        │        │ 谁是目标│          │        │        │ 定，先问│
└────────┴────────┴────────┘          └────────┴────────┴────────┘
   会污染对方缓存                        不会污染任何缓存
```

这是整个 ACD 最容易被忽略、也最关键的一点。

### 4.3 ARP 帧长什么样

下位机是自己拼以太网帧发的（`AF_PACKET` 原始套接字），因为此时接口还没有 IP，走不了内核的 ARP 模块。

```
┌──────────────────┬──────────────────┬────────┐
│ 目的 MAC (6B)    │ 源 MAC (6B)      │ 0x0806 │  以太网头 14B
├──────┬──────┬─────┴──────┬───────────┴────────┤
│ hrd  │ pro  │ hln │ pln  │ op                 │  ARP 头 8B
│ 0x01 │0x0800│  6  │  4   │ 1=请求 2=应答      │
├──────┴──────┴─────┴──────┴────────────────────┤
│ sha(6B) │ spa(4B) │ tha(6B) │ tpa(4B)         │  地址区 20B
│ 发送方MAC│ 发送方IP│ 目标MAC │ 目标IP          │
└────────────────────────────────────────────────┘
                                        合计 42 字节
```

三种包的字段差异：

| 动作 | op | spa | tpa | 目的 MAC |
|---|---|---|---|---|
| **Probe**（探测） | 1 请求 | `0.0.0.0` | 候选地址 | 广播 ff:ff:ff:ff:ff:ff |
| **Announce**（宣告） | 1 请求 | 本地址 | 本地址 | 广播 |
| **Defend**（防御） | 2 应答 | 本地址 | 询问方 | 询问方 MAC |

`Announce` 用"请求"而不是"应答"，是因为 ARP 请求会被所有主机处理并刷新缓存，覆盖范围更广（免费 ARP 的常见写法）。

### 4.4 标准 ACD 时序与我们的改动

RFC 5227 规定的参数：

| 参数 | 值 | 含义 |
|---|---|---|
| PROBE_WAIT | 1s | 启动后先等一会儿，避免所有设备同时开探 |
| PROBE_NUM | 3 | 探测次数 |
| PROBE_MIN / MAX | 1s / 2s | 探测间隔（带随机） |
| ANNOUNCE_WAIT | 2s | 探测成功后等 2s 再宣告 |
| ANNOUNCE_NUM | 2 | 宣告次数 |
| ANNOUNCE_INTERVAL | 2s | 宣告间隔 |
| MAX_CONFLICTS | 10 | 连续冲突上限，超过就限速重试 |
| DEFEND_INTERVAL | 10s | 10 秒内再次被抢则放弃地址 |

按这套参数，从开机到可用大约需要 **1 + 3×1.5 + 2 ≈ 7.5 秒**。

**我们的改动**：地址不是随机猜的，是按 SN 算出来的（见 4.5），冲突概率极低，所以探测只做 **3 次 × 200ms ≈ 0.6 秒**，省掉 PROBE_WAIT 和 ANNOUNCE_WAIT。

| | 标准 RFC | 本方案 |
|---|---|---|
| 地址来源 | 随机 | crc16(SN) 派生 |
| 探测 | 1 + 3×(1~2s) | 3 × 200ms |
| 上电到可用 | 约 7.5 秒 | **0.4~0.6 秒**（实测） |
| 冲突概率 | 由探测兜底 | 1/65000，仍由探测兜底 |

### 4.5 为什么用 SN 派生地址

```
h = crc16_ccitt("CS96000C0126020001") = 0x1B36
地址 = 169.254.(0x1B36 >> 8).(0x1B36 & 0xFF)
     = 169.254.27.54
```

| 维度 | 纯随机 | SN 派生 |
|---|---|---|
| 同一台设备每次上电 | 地址随机变 | **恒定** |
| 能否印在标签上 | 不能 | **能** |
| 能否写进工单/资产表 | 不能 | **能** |
| 故障复盘 | 得靠时间戳对日志 | 看地址就知道是哪台 |

代价是引入了"哈希碰撞"这个理论可能（1/65000），但 ACD 探测仍然保留，碰撞会被检出并自动退让。

### 4.6 冲突后的退避

```
候选 = derived + attempt × 0x0101      attempt = 0..7
  0 → 169.254.27.54
  1 → 169.254.28.55
  2 → 169.254.29.56
  ...
  8 次全失败 → 放弃，打印错误
```

`0x0101` 而不是 `+1`，是为了让 hi/lo 两个字节同时变化，避免落在同一段密集区域。

### 4.7 运行期的防御

绑定地址后，主循环一直在旁听 ARP。只要看到有人声称拥有我们的地址：

```
收到 ARP，spa == 我们的地址（主地址或 eth0:0）且 sha != 我们的 MAC
  → 立即回一个 Announce（defend）
  → 记日志：ACD: x.x.x.x claimed by <mac> (defend)
```

RFC 要求 10 秒内再次被抢就放弃地址。本实现只做了"日志 + 回击"，**没有做自动重选**——这是已知限制（见十三章）。

---

## 五、原理篇 B：mDNS（RFC 6762）与 DNS-SD（RFC 6763）

### 5.1 为什么需要 mDNS

DNS 需要一个服务器。小网络里没有 DNS 服务器，于是有人问：能不能把"域名解析"这件事变成**组播一问、相关者自答**？

mDNS 就是干这个的：

```
组播地址   224.0.0.251（IPv4） / ff02::fb（IPv6）
端口       UDP 5353（发送源端口也必须是 5353）
名字后缀   .local
TTL 字段   255（保证不被路由器转发出去）
```

一台设备宣告 `lidar-CS96000C0126020001.local` 后，同网段任何人查这个名字，设备自己回答"我是 169.254.27.54"。

### 5.2 报文格式（和 DNS 一样）

```
 0               1               2               3
 +-------+-------+-------+-------+-------+-------+-------+-------+
 |        ID（mDNS 里通常填 0）  |QR|Opcode|AA|TC|RD|RA| Z |RCODE |
 +---------------+---------------+---------------+---------------+
 |  QDCOUNT      |  ANCOUNT      |  NSCOUNT      |  ARCOUNT      |
 +---------------+---------------+---------------+---------------+
 |  问题区（Question）：名字 + 类型 + 类                          |
 |  回答区（Answer）：名字 + 类型 + 类 + TTL + 数据长度 + 数据     |
 +---------------------------------------------------------------+
```

mDNS 特有的两个约定：

| 约定 | 作用 |
|---|---|
| **QR=0 查询 / QR=1 响应** | 实现里靠这一位过滤掉"自己的应答"（否则会自问自答死循环） |
| **类字段最高位 = cache-flush** | 置 1 表示"这条记录应当覆盖你缓存里同名的旧记录" |

### 5.3 名字压缩

DNS 报文里名字重复出现很浪费空间，所以用指针压缩：

```
0xC0 0x0C  →  跳到报文偏移 0x0C 处继续读名字
```

实现时要注意：**解析指针后，返回的偏移必须是"跳过这个指针之后"的位置**，否则调用方会从错误的位置继续解析类型/类字段。这是手写 DNS 解析最常见的 bug，第一版就踩了。

### 5.4 四条记录与解析链（DNS-SD）

DNS-SD 用四条记录把"服务"和"地址"串起来：

```
_PTR_  _lidar._tcp.local
         └─> lidar-001A._lidar._tcp.local        （有哪些实例）
_SRV_  lidar-001A._lidar._tcp.local
         └─> 端口 8080，目标 lidar-001A.local    （用哪个端口、哪台主机）
_TXT_  lidar-001A._lidar._tcp.local
         └─> "sn=001A" "state=FREE" "ip=..."     （附带属性）
_A_    lidar-001A.local
         └─> 10.42.0.31 / 169.254.27.54          （最终地址，可多条）
```

客户端按 PTR → SRV → TXT → A 的顺序解析，就能从"我要找雷达"走到"连哪个 IP 哪个端口"。

TXT 里放 `state=FREE/LOCKED` 是个实用设计：**浏览列表阶段就能看出哪台正被占用**，不用连上去再被拒绝。

### 5.5 标准的四个动作

| 动作 | 做什么 | 我们实现了吗 |
|---|---|---|
| **Probe** | 宣告前先查同名，冲突则改名 `-2`、`-3` | ❌ 未实现（已知限制） |
| **Announce** | 绑定后主动广播 2~3 次 | ✅ 实现 |
| **Respond** | 收到查询时按需应答 | ✅ 实现（A/PTR/SRV/TXT） |
| **Goodbye** | 退出或换地址前发 TTL=0，让客户端立刻遗忘 | ✅ 实现 |

TTL 取值：主机名记录（A）建议 120 秒，共享记录（PTR/TXT）更长。我们统一用 120 秒。

### 5.6 为什么还要一套 UDP 广播

mDNS 是组播。组播在交换机上的命运取决于 **IGMP snooping**：

| 交换机状态 | 组播表现 |
|---|---|
| 未开 snooping | 正常泛洪，mDNS 可用 |
| 开了 snooping 且有 querier | 正常 |
| **开了 snooping 但没有 querier** | 成员表老化，**几分钟后突然不通**，插拔又恢复 |

现场**拿不到交换机管理权限**，因此这三种情况都可能遇到，且会随固件版本变化。这类"时好时坏"最难查，因为抓包时往往又是好的。

所以加一层不依赖组播的兜底：

```
每 5 秒，UDP 广播：
  255.255.255.255:5354   ← 受限广播，通用
  169.254.255.255:5354   ← 子网广播，必须有
```

为什么两个都要发：设备在纯 link-local 模式下只有 `169.254.0.0/16` 一条链路路由、**没有默认路由**，此时 `sendto(255.255.255.255)` 会直接返回 `ENETUNREACH`，包根本出不去。这是实测踩出来的坑（现象极具误导性：设备日志一切正常，PC 什么都收不到）。

分工：

| 通道 | 作用 | 失效表现 |
|---|---|---|
| mDNS（组播） | 标准发现、名字解析、跨平台兼容 | 名字解析失败，但列表里 IP 还在 |
| UDP 广播 | 保证"至少能看到设备" | 几乎不失效 |

---

## 六、下位机实现

### 6.1 启动时序

```
t=0.00  网卡 up
t=0.00  SN → crc16 → 169.254.27.54（候选）
t=0.00  ARP Probe ×3（间隔 200ms）        ← 冲突则 +0x0101 重试，最多 8 次
t=0.60  探测通过
t=0.60  读 /etc/lidar_ip.conf
          ├─ 有且空闲 → 用它作主地址
          └─ 无/被占 → 往下走
t=0.60  udhcpc（3 次 × 2 秒，最长 6 秒）
          ├─ 拿到 → 主地址 = 10.x.x.x，link-local 挂 eth0:0
          └─ 失败 → 主地址 = link-local
t=1.80  mDNS 宣告（2 次，间隔 1s）
t=1.80  TCP 8080 监听
t=1.80  广播心跳开始（每 5 秒）
t=1.80  ALIVE 心跳日志（每 30 秒）
```

### 6.2 地址层实现

| 环节 | 函数 | 做什么 |
|---|---|---|
| 哈希 | `crc16_ccitt()` | CRC-16/CCITT，初值 0xFFFF |
| 生成 | `derive_ip()` | 加 attempt×0x0101，避开保留段，拼成字符串 |
| 发帧 | `arp_send()` | 手工拼 42 字节以太网帧，`AF_PACKET` 发出 |
| 探测 | `arp_probe()` | 3 次 Probe，收包判定冲突，可回传冲突方 MAC |
| 宣告 | `arp_announce()` | 连发 2 个免费 ARP |
| 绑定 | `set_ipv4()` / `add_ipv4_alias()` | `SIOCSIFADDR`；有 DHCP 时挂 `eth0:0` |
| 防御 | `is_our_ip()` + 主循环 | 旁听 ARP，被抢则回击 |

### 6.3 双地址模式

```
eth0
 ├── 10.42.0.31/24        udhcpc 拿的（PC 同网段直连，PC 不需要额外配置）
 └── 169.254.27.54/16     SN 派生，常驻（没有 DHCP 时靠它）
```

优先级：**保存的静态地址 > DHCP > link-local**。

`udhcpc` 的配置脚本在运行时生成到 `/tmp/udhcpc.script`：

```sh
#!/bin/sh
case "$1" in
  bound|renew) ifconfig "$interface" "$ip" netmask "$subnet" ;;
esac
```

udhcpc 用 `waitpid(WNOHANG)` 轮询等待，**12 秒兜底 kill**，避免它卡住拖死整个启动。

### 6.4 发现层实现

mDNS 记录：

| 记录 | 内容 |
|---|---|
| A | `lidar-<SN>.local` → 主地址 + link-local（两条） |
| PTR | `_lidar._tcp.local` → 服务实例 |
| SRV | 端口 8080 |
| TXT | `sn=`、`state=FREE/LOCKED`、`ip=`、`proto=1` |

广播负载：

```json
{"sn":"CS96000C0126020001","ip":"10.42.0.31","ip2":"169.254.27.54",
 "mac":"28:b5:e8:e2:60:19","state":"FREE","port":8080,"lease":8000,"remain":0}
```

### 6.5 会话层：租约锁

```
IDLE ──LOCK──> HOLDING(token, owner_ip, expiry)
HOLDING ──RENEW(token)──> HOLDING（刷新有效期）
HOLDING ──8 秒无续约 / UNLOCK / 强制──> IDLE
```

| 参数 | 值 |
|---|---|
| lease | 8000ms |
| 建议续约周期 | 2666ms（lease/3，容忍两次丢包） |
| 超时释放 | 实测 8.13 秒 |

为什么是租约而不是"锁到手动释放"：客户端崩溃、网线被拔、笔记本合盖，锁会永远挂在不存在的人身上。租约的语义是"你不续约，我就当你不在了"。

### 6.6 三个必须记住的实现约束

**① 所有被 drain 的 socket 必须 `O_NONBLOCK`**

否则"读干净队列"的写法会在读完最后一条包后永久阻塞在 `recvfrom`，整个循环停摆。

诊断特征：**TCP 能完成三次握手（内核做的），但应用永不 accept，客户端表现为 `Read` 超时而不是"连接被拒绝"**。这个区别能一眼区分"没监听"和"卡死了"。

**② 广播要发两个目的地址**

见 5.6。

**③ 定时全部用 `CLOCK_MONOTONIC`**

板子没有 RTC 电池，`date` 显示 1970。用墙钟算租约、算超时会立刻错乱。

### 6.7 配置层：SETIP

```
SETIP <a.b.c.d> <token>
  → OK   ip=10.42.0.218 apply_in=300ms
  → BUSY ip=10.42.0.17 mac=1c:83:41:a7:ae:37
  → BADREQ reason=prefix|range|format
  → LOCKED owner=169.254.1.100
```

规则：

1. **只允许改末段**，前三段必须与当前地址一致（防止改到别的网段导致失联）
2. 末段 1~254
3. 必须持锁
4. 设备先做 ACD 探测，被占用回 BUSY + 对方 MAC
5. **先回包，300ms 后再改地址**（否则响应会从新地址发出，客户端收到陌生源，TCP 复位）
6. 改完发 goodbye → announce 新地址 → 立即广播
7. 写入 `/etc/lidar_ip.conf`，下次开机优先使用

---

## 七、上位机实现

### 7.1 发现面板 `lidar_scan.ps1`

监听 UDP 5354，按 SN 维护设备表：

```
Listening UDP 5354 ... Ctrl+C to stop
SN                     IP               STATE
NEW    CS96000C0126020001  10.42.0.31  FREE    alt=169.254.27.54
CHANGE CS96000C0126020001  10.42.0.31  FREE -> LOCKED
LOST   CS96000C0126020002  10.42.0.55
```

每 10 秒打一次全表；超过 30 秒没心跳标记 `LOST`。

### 7.2 控制脚本 `lidar_ctl.ps1`

| Action | 作用 |
|---|---|
| `status` | 查状态（默认） |
| `lock` | 抢锁，返回 token |
| `hold` | 抢锁并每 2.5 秒自动续约（不用手抄 token） |
| `setip` | 改末段地址（交互式，回车=不改） |
| `identify` | 让设备"举手"（串口打印大号 SN） |
| `unlock` / `info` | 释放锁 / 设备信息 |

不指定 `-Ip` 时，先监听广播自动找到该 SN 的地址。

### 7.3 一次完整会话

```
1. 开面板              → 看到哪些空闲、哪些被占
2. ctl -Action hold    → 抢锁并保持
3. 固件开始推点云       → 只有持锁者能收到
4. 调试
5. Ctrl+C / 关窗口      → 停止续约，8 秒后自动释放
```

异常路径：工程师电脑蓝屏 → 8 秒后锁自动释放，别人可接手，不需要跑到设备跟前按按钮。

---

## 八、协议参考

TCP 8080，一行一条命令，`\n` 结尾，明文 ASCII（便于手工调试）。

| 命令 | 响应 | 说明 |
|---|---|---|
| `LOCK` | `OK <token> lease=8000 renew=2666` / `BUSY owner=<ip> remain=<ms>` | 抢锁 |
| `RENEW <token>` | `OK lease=8000` / `GONE` | 续约 |
| `UNLOCK <token>` | `OK` / `GONE` | 释放 |
| `STATUS` | `STATE FREE\|LOCKED OWNER <ip> REMAIN <ms> SN <sn> IP <ip> IP2 <ip2> PORT 8080` | 查状态 |
| `SETIP <ip> <token>` | `OK ip=... apply_in=300ms` / `BUSY ip=... mac=...` / `BADREQ reason=...` / `LOCKED owner=...` | 改地址 |
| `IDENTIFY` | `OK SN=... IP=...` | 让设备举手 |
| `INFO` | `SN <sn> IP <ip> IP2 <ip2> MAC <mac> IF eth0` | 设备信息 |

---

## 九、命令手册

### 9.1 PC 侧（Windows / PowerShell）

**网络状态排查**

| 命令 | 用途 |
|---|---|
| `Get-NetAdapter \| ft Name,InterfaceIndex,Status` | 看有哪些网卡、哪块是 Up |
| `Get-NetIPAddress -InterfaceIndex 8 -AddressFamily IPv4` | 看某块网卡的地址（含辅助地址） |
| `route print 169.254*` | 确认有 `169.254.0.0/16  On-link` 路由 |
| `ping 10.42.0.31` | 通不通 |
| `arp -a \| findstr 10.42` | 看 ARP 表里的 MAC（核对是不是目标设备） |
| `Get-NetConnectionProfile` | 看网络类别（Public/Private），影响防火墙行为 |

**防火墙**

| 命令 | 用途 |
|---|---|
| `New-NetFirewallRule -DisplayName "LiDAR 5354" -Direction Inbound -Protocol UDP -LocalPort 5354 -Action Allow -Profile Any` | 放行广播 |
| `Get-NetFirewallRule -DisplayName "*5354*"` | 确认规则生效 |
| `Get-NetFirewallRule -Direction Inbound -Action Block -Enabled True` | 找出可能拦截入站的 Block 规则（本机就踩过） |

**改 PC 网络配置（仅在纯 link-local 模式下需要）**

| 命令 | 用途 |
|---|---|
| `netsh interface ipv4 add address name="以太网" address=169.254.1.100 mask=255.255.0.0` | 加辅助地址（注意：会把接口切成静态） |
| `netsh interface ipv4 set address name="以太网" static 10.42.0.17 255.255.255.0 10.42.0.1` | 同时保住原主地址 |
| `netsh interface ipv4 delete address name="以太网" address=169.254.27.54` | 删除残留地址（冲突测试后必做） |

**工具脚本**

| 命令 | 用途 |
|---|---|
| `powershell -ExecutionPolicy Bypass -File .\lidar_scan.ps1` | 设备发现面板 |
| `.\lidar_ctl.ps1 -Sn <SN> -Action status` | 查状态（自动发现 IP） |
| `.\lidar_ctl.ps1 -Sn <SN> -Action hold -Duration 30` | 抢锁并保持 30 秒 |
| `.\lidar_ctl.ps1 -Sn <SN> -Action setip` | 交互改末段地址 |
| `.\lidar_ctl.ps1 -Sn <SN> -Action setip -Octet 66` | 非交互改地址 |

**抓包（Windows 自带）**

| 命令 | 用途 |
|---|---|
| `pktmon start --capture --pkt-size 128 -f C:\temp\p.etl` | 开始抓包 |
| `pktmon stop` | 停止 |
| `pktmon format C:\temp\p.etl -o C:\temp\p.txt` | 转文本 |

### 9.2 Ubuntu 侧（编译与传输）

**环境与编译**

| 命令 | 用途 |
|---|---|
| `sudo apt install -y gcc-aarch64-linux-gnu` | 装交叉编译器 |
| `aarch64-linux-gnu-gcc -O2 -Wall -static -o lidar_agent lidar_agent.c` | 编译（静态链接） |
| `file lidar_agent` | 确认 `ARM aarch64 ... statically linked` |

**版本校验（每次必做）**

| 命令 | 用途 |
|---|---|
| `md5sum lidar_agent.c` | 源码是否为目标版本 |
| `grep -c "v5-setip" lidar_agent.c` | 关键标记（应为 1） |
| `grep -a -c "v5-setip" lidar_agent` | 二进制里是否含标记（应为 1） |
| `md5sum lidar_agent` | 与目标机上的对比 |

> `strings | grep` 在共享目录/缓存异常时会给假结果，用 `grep -a` 直接扫二进制更可靠。

**传输**

| 命令 | 用途 |
|---|---|
| `scp -o StrictHostKeyChecking=no lidar_agent root@<IP>:/tmp/` | 常规传输 |
| `cat lidar_agent \| ssh root@<IP> "cat > /tmp/lidar_agent; chmod +x /tmp/lidar_agent"` | 管道传输（绕开 sftp/chroot 问题） |
| `ssh-keygen -R <IP>` | 清掉过期主机指纹（板子 rootfs 在内存盘，每次重启 key 都变） |

**抓包（若在 Ubuntu 侧分析）**

| 命令 | 用途 |
|---|---|
| `sudo tcpdump -i any -n udp port 5354` | 看广播心跳 |
| `sudo tcpdump -i any -n arp` | 看 ACD 探测/宣告 |
| `sudo tcpdump -i any -n port 8080` | 看控制协议 |

### 9.3 下位机侧（BusyBox）

**运行**

| 命令 | 用途 |
|---|---|
| `cd /tmp && chmod +x lidar_agent` | 赋执行权限 |
| `./lidar_agent CS96000C0126020001 eth0` | 启动（必须 root） |
| `Ctrl+C` | 退出并发 mDNS goodbye |

> 启动请用**串口终端**，不要用 ssh——程序会改 eth0 地址，ssh 会断。

**网卡与地址**

| 命令 | 用途 |
|---|---|
| `ifconfig` | 看主地址与 `eth0:0` 附加地址 |
| `ip addr show eth0` | 同上（信息更全） |
| `ip route` | 看路由；纯 link-local 模式下应只有 `169.254.0.0/16` |
| `ping <ip>` | 连通性 |
| `cat /proc/net/arp` | 看邻居表（没有 `arp` 命令时用这个） |
| `cat /proc/net/dev` | 收发包计数 |

**进程与诊断**

| 命令 | 用途 |
|---|---|
| `ps` | 有没有在跑 |
| `cat /proc/net/snmp \| grep -A1 "^Udp:"` | `OutDatagrams` 每 5 秒应 +2（验证广播真的发出去了） |
| `cat /proc/net/tcp` | 看 8080 是否在监听（十六进制 `1F90`） |
| `dmesg` | 驱动/内核层报错 |
| `cat /proc/$(pidof lidar_agent)/status` | 看 VmRSS 等资源占用 |

**文件与启动项**

| 命令 | 用途 |
|---|---|
| `md5sum /tmp/lidar_agent` | 与 PC 侧对比，确认版本 |
| `ls -l /tmp/lidar_agent` | 时间戳 |
| `cat /etc/lidar_ip.conf` | 看保存的静态地址 |
| `ls -l /etc/init.d/` | 看有哪些开机服务（当前 9 个） |
| `busybox --list` | 系统里到底有哪些命令 |
| `grep -rn "ifconfig\|udhcpc" /etc/init.d/` | 查有没有别的脚本在抢网卡 |

---

## 十、调试验证步骤

### 10.1 前置检查

```powershell
# PC：不能残留 169.254.27.54（冲突测试后常见）
Get-NetIPAddress -InterfaceIndex 8 -AddressFamily IPv4 | ft IPAddress,PrefixLength

# PC：确认板子身份，避免连到别的设备
ssh root@<IP> "cat /sys/class/net/eth0/address; hostname"
# 期望 28:b5:e8:e2:60:19 + lidar
```

### 10.2 编译与部署

```bash
# Ubuntu
cd ~/lidar_ll/agent
md5sum lidar_agent.c                        # 与发布值一致
aarch64-linux-gnu-gcc -O2 -Wall -static -o lidar_agent lidar_agent.c
grep -a -c "v5-setip" lidar_agent           # 必须 1
scp -o StrictHostKeyChecking=no lidar_agent root@<IP>:/tmp/
```

```sh
# 下位机
md5sum /tmp/lidar_agent                     # 与 Ubuntu 上一致
cd /tmp && chmod +x lidar_agent
./lidar_agent CS96000C0126020001 eth0
```

期望日志：

```
[ 0.000] lidar_agent v5-setip  SN=CS96000C0126020001  host=lidar-...local
[ 0.6xx] P0: DHCP address 10.42.0.xx
[ 0.6xx] P0: ready  ip=10.42.0.xx  secondary=169.254.27.54
[ 1.8xx] P1: mDNS announced ...
[ 1.8xx] P2: control on tcp/8080
[ 1.8xx] BCAST: udp/5354 every 5000ms on eth0
```

### 10.3 分项验证

| # | 验证项 | 操作 | 期望 |
|---|---|---|---|
| 1 | 地址派生 | 看启动日志 | `169.254.27.54`（SN 固定对应） |
| 2 | 探测耗时 | 看时间戳差 | 0.4~0.6 秒 |
| 3 | 双地址 | `ifconfig` | `eth0` + `eth0:0` |
| 4 | 广播发出 | `cat /proc/net/snmp \| grep -A1 Udp` | `OutDatagrams` 每 5 秒 +2 |
| 5 | 广播接收 | PC 开面板 | 5 秒内 `NEW`，age 稳定 4~5s |
| 6 | 控制协议 | `-Action status` | `STATE FREE ... PORT 8080` |
| 7 | 抢锁 | `-Action lock` | 返回 token；板子日志 `LOCK granted` |
| 8 | 租约释放 | lock 后等 8 秒 | 板子 `LEASE: expired`；面板回 FREE |
| 9 | 续约保活 | `-Action hold -Duration 30` | 12 次 `OK lease=8000`，不掉锁 |
| 10 | 冲突检测 | PC 先占 169.254.27.54，再启动设备 | `ACD: conflict detected` → 换到 `.28.55` |
| 11 | 改地址（空闲） | `-Action setip -Octet 66` | `OK ip=10.42.0.66`，面板 5 秒内更新 |
| 12 | 改地址（占用） | `-Action setip -Octet 17`（PC 自己） | `BUSY ... mac=<PC 的 MAC>` |
| 13 | 不改 | 交互模式直接回车 | `same as current address` |
| 14 | 存活心跳 | 等 35 秒 | `ALIVE ip=... state=...` 出现 |

### 10.4 冲突测试的正确顺序（容易做错）

必须**先让设备彻底放弃地址**，再让 PC 占用：

```sh
# 下位机
Ctrl+C
ip addr flush dev eth0          # 或 ifconfig eth0 0.0.0.0
```

```powershell
# PC
netsh interface ipv4 delete address name="以太网" address=169.254.27.54
netsh interface ipv4 add    address name="以太网" address=169.254.27.54 mask=255.255.0.0
ping 169.254.27.54              # TTL=128 说明是 PC 自己应答
```

```sh
# 下位机重新启动
./lidar_agent CS96000C0126020001 eth0
```

顺序反了会测不出来：Windows 添加静态地址时会先做免费 ARP，被设备抢先应答后会把该地址标记为"重复"并停用，之后设备探测自然无人应答。

### 10.5 故障对照

| 现象 | 原因 | 处理 |
|---|---|---|
| 面板空白但能 ping 通 | PC 侧防火墙拦入站 UDP 5354 | 加放行规则；查有无 Block 规则 |
| 能发现、连不上 | PC 没有目标网段地址 | 用 DHCP 地址，或给 PC 加辅助地址 |
| 设备日志正常但收不到广播 | 设备无默认路由，`255.255.255.255` 发不出 | 已修（同时发 `169.254.255.255`） |
| 跑几秒后完全无响应 | 有阻塞 socket 卡住事件循环 | 已修（全部 `NONBLOCK`）；看 `ALIVE` |
| 改地址后连不上 | 已切换，旧地址失效 | 重新 `-Action status` 自动发现 |
| `scp` 显示 100% 但文件不在 | 传到了同 IP 的另一台设备 | 先核对 MAC；给设备设唯一 hostname |
| 编译没报错但行为是旧的 | 源码/二进制没真正更新 | `md5sum` + `grep -a` 双重确认 |

---

## 十一、资源使用

### 11.1 磁盘

| 项 | 大小 | 说明 |
|---|---|---|
| `lidar_agent` | 约 770 KB | 静态链接，不依赖目标机任何 .so |
| 运行时文件 | < 1 KB | `/tmp/udhcpc.script`、`/etc/lidar_ip.conf` |
| 上位机脚本 | < 15 KB | 两个 .ps1 |

### 11.2 内存与 CPU

- 常驻内存：约 1.5~2 MB（**估算**；实测命令 `grep VmRSS /proc/$(pidof lidar_agent)/status`）
- 空闲 CPU：每 200ms 一次 poll 超时返回，接近 0
- 对 PREEMPT_RT 上的点云任务无可测影响（普通优先级）

### 11.3 网络

| 流量 | 频率 | 带宽 |
|---|---|---|
| 广播心跳 | 5 秒 × 2 个目的地址 | 约 60 B/s |
| mDNS 宣告 | 启动 3 次 + 状态变化 | 可忽略 |
| 控制命令 | 按需 | 每条 < 200 B |

### 11.4 端口

| 端口 | 协议 | 用途 |
|---|---|---|
| 8080 | TCP | 控制与锁 |
| 5353 | UDP | mDNS |
| 5354 | UDP | 广播心跳 |

### 11.5 系统依赖

| 依赖 | 必需 | 缺失后果 |
|---|---|---|
| root 权限 | ✅ | 无法开 AF_PACKET，启动失败 |
| `/tmp` 可写 | ✅ | DHCP 分支失效（退到 link-local） |
| `udhcpc` | 可选 | 拿不到 10.x，只用 link-local |
| `ifconfig` | 可选 | 同上（udhcpc 脚本用它配地址） |
| `/etc` 可写 | 可选 | `setip` 地址无法持久化 |
| systemd / dbus / avahi | ❌ 不需要 | — |

自检：

```sh
for c in udhcpc ifconfig ip; do command -v $c >/dev/null && echo "OK $c" || echo "MISSING $c"; done
touch /tmp/.t && echo "OK /tmp rw" && rm -f /tmp/.t
[ "$(id -u)" = 0 ] && echo "OK root"
```

---

## 十二、如何嵌入你自己的项目

### 12.1 先判断需要哪一层

| 需求 | 需要搬 | 工作量 |
|---|---|---|
| 只解决"多台设备 IP 冲突" | 地址层（约 250 行） | 小 |
| 还想"按名字找到" | 地址层 + 发现层 | 中 |
| 还想"防止两人抢同一台" | 三层全要 | 中 |

### 12.2 方案 A：整进程共存（推荐先做）

`lidar_agent` 独立跑，固件只读一个状态文件：

```
┌─ 固件（点云采集/发送，原有代码）──┐      ┌─ lidar_agent（新）──┐
│ 每 200ms 读 /tmp/lidar_state      │ ←──  │ 维护锁状态           │
│ LOCKED 就 connect 到 owner 推流   │      │ 周期写状态文件        │
└───────────────────────────────────┘      └──────────────────────┘
```

状态文件一行：`<STATE> <owner_ip> <port>`，例如 `LOCKED 10.42.0.17 8080`。

优点：零风险，出问题 kill 掉 agent 即恢复。
缺点：点云发送前多一次文件读，但**没有额外拷贝**。

### 12.3 方案 B：把锁搬进固件

1. 把 `ctrl_handle()` 的状态机搬进一个**非实时线程**
2. 点云发送线程读原子指针（session 快照），持锁时 `connect()` 到 owner
3. 定时器用 `CLOCK_MONOTONIC`，**不要用 `SIGALRM`**（会打断 RT 线程）
4. 200ms 检查一次租约

```
采集线程（RT，不动）
   ↓ 环形缓冲
发送线程（普通）── 读 session 快照 ──> sendto 持锁方
        ▲
控制线程（普通）── 锁状态机 ──> 原子指针切换
```

**key：session 用原子指针切换，读侧不加锁。** RT 系统里最忌在发送路径上加互斥量——控制线程一持锁，点云就抖动，且现场测不出来，只表现为"偶尔丢帧"。

### 12.4 方案 C：只搬地址层

搬这几个函数（约 250 行，无外部依赖）：

```
crc16_ccitt() / derive_ip()
arp_send() / arp_probe() / arp_announce()
set_ipv4() / add_ipv4_alias()
is_our_ip()
```

加一个 200ms 循环做运行期防御，就是完整的 `lladdr` 守护进程。

### 12.5 三个必改点

| 点 | 现状 | 产品里 |
|---|---|---|
| SN 来源 | 命令行参数 | eFuse / EEPROM / 配置文件 |
| 网卡名 | 硬编码 `eth0` | 可能是 `end0` / `eth1` |
| 持久化路径 | `/etc/lidar_ip.conf`（内存盘会丢） | 持久分区 |

### 12.6 不要照搬的地方

- **单线程轮询**：适合验证，不适合几百 Mbps 点云
- **8 秒租约**：无人值守采集要放大到 30~60 秒
- **mDNS 无改名**：同名会打架，产品要按 RFC 6762 加 `-2`
- **identify 只打印**：产品要接实际灯或蜂鸣器

---

## 十三、已知限制

| 限制 | 影响 | 现状 |
|---|---|---|
| 地址被强占不自动重选（只 defend） | 靠日志发现 | 待补 |
| mDNS 无名字冲突改名 | 同 SN 两台会同名 | 待补 |
| `/etc` 在内存盘 | setip 配置重启丢失 | 依赖存储规划 |
| 无认证 | 同网段任何人都能抢锁 | 内网调试可接受 |
| 单线程 | 不适合高并发 | MVP 定位 |

---

## 十四、实测数据（2026-11-03）

| 项 | 结果 |
|---|---|
| SN 派生地址 | `CS96000C0126020001` → 169.254.27.54（crc16 0x1B36） |
| link-local 探测耗时 | 0.4~0.6 秒 |
| 广播发现延迟 | ≤ 5 秒 |
| 租约到期释放 | 8.13 秒（设定 8000ms） |
| 续约保活 | 12 次连续 renew 不掉锁 |
| 冲突检测 | 0.2 秒内检出，自动跳到 169.254.28.55 |
| SETIP 空闲地址 | 立即应用，面板 5 秒内更新 |
| SETIP 占用地址 | 正确拒绝并返回对方 MAC |
| 双地址 | `eth0` = 10.42.0.31，`eth0:0` = 169.254.27.54 |

环境：TL62x-EVM（Buildroot 2023.02.11，内核 6.1.80-rt26，rootfs 在内存盘）+ Windows 10 调试机。
