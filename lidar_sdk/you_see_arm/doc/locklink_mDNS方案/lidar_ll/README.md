# lidar_ll - Link-Local + mDNS + 会话锁 可行性验证包

对应方案文档 `方案设计/LiDAR_LinkLocal_mDNS_方案.md` 的 P0 + P1 + P2 + P5。
**这是验证工具，不是产品代码。**

```
agent/lidar_agent.c     雷达端：地址分配 + mDNS + 锁服务 + 广播心跳（单文件，约 700 行）
agent/Makefile          编译脚本
pc/lidar_scan.ps1       PC 端：设备发现面板（UDP 广播）
pc/lidar_ctl.ps1        PC 端：发命令（lock / renew / unlock / status / identify / info）
```

本机 SN `CS96000C0126020001` 派生出的地址：**169.254.27.54**，mDNS 名 `lidar-CS96000C0126020001.local`。

---

## 一、编译（在 Ubuntu 上）

```bash
# 方式 A：装了 Ubuntu 原生交叉编译器
sudo apt install -y gcc-aarch64-linux-gnu
aarch64-linux-gnu-gcc -O2 -Wall -static -o lidar_agent lidar_agent.c

# 方式 B：用你 Buildroot 输出的工具链（路径按实际改）
<buildroot>/output/host/bin/aarch64-buildroot-linux-gnu-gcc -O2 -Wall -static -o lidar_agent lidar_agent.c
```

**建议加 `-static`**：你板子的 rootfs 只有 30MB，动态链接容易撞 glibc 版本。

---

## 二、传到板子

⚠️ 你的资料目录带中文，`scp` 会因编码问题报 `No such file or directory`。先复制到纯英文路径：

```powershell
mkdir C:\temp\lidar
copy lidar_agent C:\temp\lidar\
scp C:/temp/lidar/lidar_agent root@<板子IP>:/tmp/
```

---

## 三、在板子上运行

⚠️ **用串口终端启动，不要用 ssh** —— 程序会改 `eth0` 的 IP，ssh 会当场断开。

```sh
/tmp/lidar_agent CS96000C0126020001 eth0
```

需要 root（要开 AF_PACKET 原始套接字）。

预期日志（v4-dual，双地址模式）：

```
[    0.000] lidar_agent v4-dual  SN=CS96000C0126020001  host=lidar-...local  if=eth0  mac=28:b5:e8:e2:60:19
[    0.001] P0: link-local candidate 169.254.27.54 (attempt 0)
[    0.601] P0: DHCP address 10.42.0.107          ← 有 DHCP 时
[    0.607] P0: link-local 169.254.27.54 kept as secondary
[    0.607] P0: ready  ip=10.42.0.107  secondary=169.254.27.54
[    1.610] P1: mDNS announced lidar-CS96000C0126020001.local
[    1.611] P2: control on tcp/8080
[    1.611] BCAST: udp/5354 every 5000ms on eth0
```

没有 DHCP 时自动退回单地址：

```
[    7.203] P0: no DHCP, link-local only: 169.254.27.54/16
[    7.203] P0: ready  ip=169.254.27.54  secondary=-
```

两种模式对上层完全透明：控制端口、mDNS、广播都同时报两个地址，PC 端挑能通的用。

Ctrl+C 退出时会发 mDNS goodbye。

### 双地址模式说明

```
eth0
 ├── 10.x.x.x/24        udhcpc 拿的（PC 同网段，直接访问，不需要给 PC 加 169 地址）
 └── 169.254.27.54/16   SN 派生的，常驻（没有 DHCP 时靠它，可预测、可贴标签）
```

- DHCP 超时 3 次 × 2 秒，最长 6 秒
- udhcpc 卡住会被 12 秒兜底 kill，不会拖死启动
- `STATUS` 和广播 JSON 都多一个 `ip2` 字段

---

## 四、PC 侧网络准备（仅 link-local 模式需要）

> v4 起设备会优先拿 DHCP 地址（10.x），PC 与它同网段时**不需要**做这一节的操作。
> 只有设备退化成"只有 169.254 地址"（现场没有 DHCP、直连），才需要给 PC 加辅助地址。

**现象**：`lidar_scan.ps1` 能看到设备（广播不分子网），但 `lidar_ctl.ps1` 连接超时。

**原因**：PC 的网卡只有 `10.42.0.17/24` 这类地址，没有 169.254 的本地地址，Windows 就把
`169.254.27.54` 当成"外网"，丢给默认网关 `10.42.0.1`，网关不知道往哪转 → 断。

**解决**：给这块网卡加一个辅助 IPv4 地址（一个网卡可以同时有多个地址，互不影响）。

```powershell
# 1) 先确认网卡名字
Get-NetAdapter | Select-Object Name, InterfaceDescription, Status

# 2) 加辅助地址（管理员 PowerShell）
netsh interface ipv4 add address name="以太网" address=169.254.1.100 mask=255.255.0.0
# 或
New-NetIPAddress -InterfaceAlias "以太网" -IPAddress 169.254.1.100 -PrefixLength 16
```

**验证**：

```powershell
route print 169.254*      # 应出现 169.254.0.0/16  On-link  该网卡IP
ping 169.254.27.54        # 板子跑起来后应能通
```

> 说明：RFC 3927 要求支持 link-local 的主机在链路上也持有 169.254 地址。Windows 只在"自动获取 IP
> 且 DHCP 失败"时才自动加（APIPA）；你手工/DHCP 拿到 10.42.0.17 后就不会自动加，所以必须手动补。

**还要放行 UDP 5354**（否则面板收不到任何广播，见下）：

```powershell
# 管理员
New-NetFirewallRule -DisplayName "LiDAR Discovery UDP 5354" -Direction Inbound -Protocol UDP -LocalPort 5354 -Action Allow -Profile Any
```

Windows 把该网络识别为"公用"时，默认拦所有入站，广播包会被静默丢弃；
`ping` 能通只是因为 ICMP 被放行，所以"能 ping 通但面板空白"就是这个原因。

检查当前网络类别：

```powershell
Get-NetConnectionProfile | ft Name,InterfaceAlias,NetworkCategory
```

---

## 五、在 PC 上运行

**发现面板**（先启动这个）：

```powershell
powershell -ExecutionPolicy Bypass -File .\lidar_scan.ps1
```

看到：

```
Listening UDP 5354 ... Ctrl+C to stop
NEW    CS96000C0126020001  169.254.27.54  FREE    from 169.254.27.54
```

**发命令**（另开一个窗口；注意扫描面板占用 5354，单实例测试时先关掉扫描，或直接指定 `-Ip`）：

```powershell
.\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action status
.\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action lock
.\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action identify
```

`lock` 会返回 token，但租约只有 8 秒，不续约就自动释放 —— 这是设计如此，用来验证"人走了锁自动掉"。

### 修改设备地址（v5-setip）

只改**第四段**，前三段保持设备当前网段（DHCP 就跟着 DHCP 走，link-local 就跟着 169.254 走）：

```powershell
.\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action setip
```

```
current address : 10.42.0.31
prefix          : 10.42.0.___

new last octet [31]:
```

直接回车 = 保持现状；输入 `77` = 改成 `10.42.0.77`。

流程是 **先占锁 → 设备 ARP 探测 → 空闲才改**，占用会明确拒绝：

```
new last octet [31]: 77
  10.42.0.77 is already in use -> BUSY ip=10.42.0.77 mac=1c:83:41:a7:ae:37
  please choose another one
new last octet [31]:
```

成功后设备约 300ms 后切换地址（先回包、再改），旧连接会断，用广播重新发现即可。

脚本化调用：

```powershell
.\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action setip -Octet 77
```

> ⚠️ 新地址存在 `/etc/lidar_ip.conf`。你板子的 rootfs 在内存盘里，**重启会丢**；
> 只有 `/etc` 挂到持久分区时才真正长期生效。

---

## 六、这条测试能验证什么

| # | 操作 | 期望 |
|---|---|---|
| 1 | 启动 agent | 绑定 169.254.27.54，PC 10 秒内收到广播 |
| 2 | `status` | `STATE FREE` |
| 3 | `lock` | 返回 token；板子日志 `LOCK granted`；扫描面板显示 `CHANGE ... -> LOCKED` |
| 4 | 等 8 秒不续约 | 板子日志 `LEASE: expired, release`；面板回到 `FREE` |
| 5 | 再 `lock`，记下 token | 每 2~3 秒 `renew` 一次，锁能一直保持 |
| 6 | 另一台板子跑不同 SN | 两个不同 IP，互不干扰 |
| 7 | **同一台板子用同一 SN 跑第二个实例** | 第二个实例 ACD 冲突检测生效，地址自动换一个（日志有 `ACD: conflict detected`） |
| 8 | `identify` | 串口打印大号 SN（这块板没 LED 驱动，先用打印代替闪灯） |
| 9 | PC 上 `ping lidar-CS96000C0126020001.local` | Windows 10+ 可能解析，失败也正常（走 mDNS，系统支持不一） |

第 7 条是本方案的核心验证点：**同一 SN 的两台设备不能抢同一个地址**。

---

## 七、已知限制（故意的，先验证骨架）

- 单线程，TCP 一次只处理一个连接；不做并发
- mDNS 只回答 A/PTR/SRV/TXT，**未实现名字冲突探测与改名**（`-2` 后缀那套）
- ACD 检测到运行期冲突只做 defend + 日志，**不自动重选地址**
- 退出后不会自动把 eth0 恢复成原来的 IP（重启板子即可）
- 完全不碰点云：本程序只验证寻址、发现、加锁三件事
- `identify` 只打印，没有驱动 LED（你板子没有 `/sys/class/leds`）

---

## 八、下一步

这套跑通后，把 `lidar_agent` 里三块逻辑搬进固件：

1. `P0` 的地址分配 → 独立守护进程，开机跑（`/etc/init.d/S10lladdr`）
2. `P2` 的锁状态机 → 并入固件的控制线程，点云发送线程只在持锁时 `connect()` 到客户端
3. 广播心跳 → 保留，作为交换机不支持组播时的兜底
