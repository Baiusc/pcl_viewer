# SN 码上位机读写开发指导

适用范围：当前 `dev/fs_sdk` 的 CS96 Beta 固件。本文中的“SN 码”指设备唯一序列号；上位机界面如使用“SM 码”字样，应先统一为 SN。写入发生在整机固件烧录、Linux 启动之后，由产线系统为每台设备提供具体号码。

## 1. 先区分读取与写入能力

| 功能 | 当前可用入口 | 上位机用途 |
| --- | --- | --- |
| 发现设备 | DNS-SD `_lidar._tcp.local`、UDP 5354 广播 | 获取地址、MAC、SN 与服务端口 |
| 在线读取 | `GET http://<设备IP>/cgi-bin/lidar_network.cgi` | 读取状态对象中的 `sn`、`identity_source`、`mac` |
| 诊断读取 | `GET http://<设备IP>/cgi-bin/lidar_network_status.cgi` | 业务管理服务不可用时，直接读取网络服务状态；需镜像提供该 CGI |
| 旧协议读取 | `CMD_SET_SYSTEM / CMD_ID_GET_SYS_SN` | 复用现有 UDP 管理协议的上位机可读取完整 SN |
| 首次写入 | 通过受控 SSH 会话在设备上执行 `/usr/libexec/lidar/lidar_sn_provision <SN>` | 仅供产线/返修授权流程使用 |

**当前没有 HTTP、UDP 或 Avahi 的 SN 写入接口。** `POST /cgi-bin/lidar_network.cgi` 只处理网络配置；`SN_set` 驱动接口已禁用。上位机不能直接写 `/dev/mtd3`，也不能把待写 SN 塞入固件或普通升级包。若量产环境不开放受控 SSH，需要另行开发设备端鉴权写入服务后，才能设计对应的远程写入 API。

## 2. SN 规则与上位机校验

Flash、协议和 MES 使用**无分隔符、19 个大写 ASCII 字符**，例如 `CS96XXXB01026020001`。界面可显示为 `CS96XXX-B01-0-2602-0001`，提交写入器前必须去掉分隔符。

| 字符位置（从 1 开始） | 长度 | 含义 | 固件当前校验 |
| --- | ---: | --- | --- |
| 1～7 | 7 | 型号 | 前两位为 `CS` 或 `MF`；第 3～4 位为数字；其余位为大写字母或数字 |
| 8～10 | 3 | 硬件版本 | 1 个大写字母与 2 个数字 |
| 11 | 1 | 组装地 | `0` 成都，`1` 德州 |
| 12～15 | 4 | 生产年周 | `YYWW`，周数 `01`～`53` |
| 16～19 | 4 | 当周生产序号 | 大写十六进制 `0001`～`FFFF` |

固件只验证上述格式，不判断型号是否与实物一致，也不检查 MES 中的唯一性或 `YYWW` 是否是实际日历周。上位机必须从 MES 取得已分配的 SN，并核对扫描到的机型、硬件版本、组装地、生产周、MAC 与工单；不能按当前时间自行生成序号。建议 MES 对 SN 设置唯一约束，并记录 `SN ↔ MAC ↔ 工单 ↔ 操作员 ↔ 写入时间`。

## 3. 发现与读取

### 3.1 推荐读取流程

1. 浏览 `_lidar._tcp.local`，按返回的服务实例和目标地址连接；也可监听设备每 5 秒发送一次的 UDP 5354 JSON 广播。允许用户手动输入 IP，作为发现不可用时的入口。
2. 对目标 IP 执行 `GET /cgi-bin/lidar_network.cgi`，要求 HTTP 状态与 JSON `code` 均为 `200`，并检查 `schema_version=1`。
3. 读取 `mac`、`sn`、`identity_source`、`identity`、`model`、`mdns_running`。`sn=""` 且 `identity_source="mac"` 表示没有读到有效 SN；它也可能是分区内存在无效数据，不能仅据此认定 Flash 完全空白。不要把 MAC 临时身份当作 SN。
4. 用 MAC、接收网卡和设备地址区分设备。即使发现两个设备声称相同 SN，也要分别显示并报重复号，不应合并为一台。

状态对象示意，仅列出与本功能有关的字段：

```json
{
  "code": 200,
  "schema_version": 1,
  "identity": "CS96XXXB01026020001",
  "identity_source": "sn",
  "sn": "CS96XXXB01026020001",
  "mac": "98:03:8a:7f:32:ee",
  "model": "CS96-beta",
  "mdns_running": true
}
```

DNS-SD 服务的 TXT 记录包含 `identity`、`sn`、`model`、`path` 等字段，服务类型为 `_lidar._tcp`。写码前主机名使用 `lidar-mac-<12位小写MAC>.local`；写码并重启网络服务后通常使用 `lidar-<SN>.local`。Avahi 遇到重名可改名，上位机必须使用发现结果中的实际目标名和 TXT 数据，不能仅拼接主机名。`mdns_running=true` 只表示 Avahi 子进程存在，仍要从上位机实际解析和读取服务。

采集 UDP v1 广播有 18 字节和 19 字节两种身份字段。19 字节格式可以携带完整 SN。设备发出 `MAC-` 临时身份时，节点详情显示未读到有效 SN。旧管理协议 `CMD_ID_GET_SYS_SN` 的应答数据区为 1 字节执行状态加 19 字节 SN；失败时不得解析后面的 SN 字节。该命令沿用现有协议封包和应答头，上位机应复用已有管理协议实现，不应仅凭这 20 字节自行拼报文。

### 3.2 最小读取示例（Python 标准库）

```python
import ipaddress
import json
from urllib.request import urlopen


def read_sn(ip):
    address = str(ipaddress.IPv4Address(ip))
    url = f"http://{address}/cgi-bin/lidar_network.cgi"
    with urlopen(url, timeout=5) as response:
        status = json.load(response)
    if status.get("code") != 200 or status.get("schema_version") != 1:
        raise RuntimeError("设备网络状态不可用")
    return {
        "sn": status.get("sn", ""),
        "mac": status.get("mac", ""),
        "identity_source": status.get("identity_source", ""),
        "mdns_running": status.get("mdns_running", False),
    }
```

## 4. 产线写入流程

写入入口要求设备运行 Linux、系统镜像包含 `lidar_sn_provision`、目标为 Beta 当前 `/dev/mtd3` 且分区标签为 `sn`。写入器要求 root 权限，并核对 MTD 字符设备和“分区恰好一个擦除块”的布局。其他机型应先确认分区布局并适配写入器。

1. 从 MES 领取 SN 和目标工单，检查格式与唯一性。通过扫描设备标签或产线工装确认目标 MAC，记录待写设备的 IP 与 MAC。
2. 在线读取当前 SN。若已等于 MES 目标 SN，可按重复执行处理；若已是其他有效 SN，停止流程并转人工返修核对。不要自动擦除或覆盖。
3. 使用受控 SSH 连接目标设备，校验设备 SSH 主机密钥和目标 MAC 后执行写入器。不要将 root 凭据嵌入通用上位机软件。
4. 只有写入器退出码为 `0` 时才继续。它会进行写后回读；退出码 `2` 表示参数或格式错误，`1` 表示权限、分区、已有其他内容、擦写或回读失败。
5. 执行 `/usr/libexec/lidar/lidar-network-service restart`，等待网络服务恢复，再通过 HTTP 状态核对 `sn`、`identity_source="sn"`、MAC 与工单一致。
6. 从另一台上位机或同一上位机的实际 DNS-SD 客户端解析 `_lidar._tcp.local`，核对 TXT `sn` 与目标值。完成 MES 结果回传和审计记录后放行。

设备端等效命令：

```sh
/usr/libexec/lidar/lidar_sn_provision CS96XXXB01026020001
/usr/libexec/lidar/lidar-network-service restart
```

写入器只接受空分区、与目标完全相同的 SN，或同一目标 SN 在写入中断后留下的前缀；不会覆盖其他内容。它会擦除整个独立 SN 分区再写入。命令失败、网络中断或回读不一致时不能宣布写入成功；应再次读取并保持设备不放行。固件升级不会主动擦除该分区。

### 4.1 上位机调用示意

示意代码仅覆盖远程调用；正式产线程序还需接入 MES、已登记的 SSH 主机密钥、操作员权限和审计记录。

```python
import re
import ipaddress
import subprocess


def write_sn_over_ssh(ip, sn):
    if not re.fullmatch(r"(?:CS|MF)[0-9]{2}[A-Z0-9]{3}[A-Z][0-9]{2}[01][0-9]{2}(?:0[1-9]|[1-4][0-9]|5[0-3])[0-9A-F]{4}", sn):
        raise ValueError("SN 格式错误")
    if sn[-4:] == "0000":
        raise ValueError("批号不能为 0000")
    address = str(ipaddress.IPv4Address(ip))
    common = ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", f"root@{address}"]
    subprocess.run(common + [f"/usr/libexec/lidar/lidar_sn_provision {sn}"], check=True, timeout=60)
    subprocess.run(common + ["/usr/libexec/lidar/lidar-network-service restart"], check=True, timeout=60)
    # 随后轮询 HTTP 状态，并同时核对 SN、MAC 和 identity_source。
```

OpenSSH 在远端通过 shell 执行命令；上例先把 SN 限制为固定的大写字母和数字。正式实现还应限制 SSH 目标、避免共享 root 凭据，并将主机密钥与目标设备绑定。`StrictHostKeyChecking=yes` 要求产线事先维护可信 `known_hosts`；不能在首次连接时静默接受未知密钥。

## 5. 界面状态与异常处理

| 情况 | 上位机处理 |
| --- | --- |
| `sn` 为空 | 显示“未读到有效 SN”；写入器仍需检查分区是否真正空白 |
| 当前 SN 等于 MES 目标 | 显示“已写入”；核对 MAC、工单和发现记录，不重复擦写 |
| 当前 SN 与目标不同 | 锁定写入，要求人工返修流程 |
| SSH 失败或写入器退出非零 | 保留设备与工单的失败状态，重新读取设备；不自动改写为其他 SN |
| 写入器成功但 HTTP 未恢复 | 视为“写入结果待核验”，保留设备；通过本地控制口或维护流程排查 |
| HTTP SN 正确但 DNS-SD 不可用 | 保留网络状态截图与日志，排查 Avahi；不能将发现失败解释为写码失败 |
| 同一 SN 出现在多个 MAC | 标记重复号，阻止自动合并和放行 |

建议至少覆盖空白设备首次写入、同号重复执行、异号拒绝、非法格式、无 root 权限、网络中断、写后服务重启、DNS-SD 重名和重复 SN 两台设备。板端 Flash 擦写与 Avahi 互通需要在专用设备上验收；模拟网络测试不能替代这些检查。

## 6. 当前实现依据

- SN 校验与只读：`network/device_identity.c`。
- 受控写入与回读：`network/lidar_sn_provision.c`。
- 运行状态、身份来源和广播：`network/lidar_netd.c`、`network/network_discovery.c`。
- HTTP 查询入口：`network/lidar_network_http.c`。
- 旧管理协议读取：`ys_business/ys_business_sys.c`、`ys_business/ys_msg_distribute.c`。
- RootFS 集成和写入顺序：`doc/Jenkins-SN码RootFS集成指导.md`。
