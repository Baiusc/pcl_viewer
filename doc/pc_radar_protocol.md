# CS96-beta 激光雷达 ↔ PC 上位机 通讯协议

> 面向做 PC 上位机（点云显示 / 标定）的开发者。
> 协议来源：设备端实测抓包 + `you_see` 驱动行为对照 + 厂内 `cloud_viewer` 解析实现。
> 适用固件：旧版（`you_see` 独立进程）、新版（2026-11 起 `lidar_mgrd` 托管，固件版本 `8011(beta)`）。
> 最后核对：2026-11-25，设备 SN `CS96000B01026470003`，96 线 / 800 slot。

---

## 1. 概览

上位机与雷达之间有三条独立通道，互不阻塞：

| 通道 | 方向 | 传输 | 用途 | 是否必需 |
|---|---|---|---|---|
| 设备发现 | 雷达 → 广播 | UDP 55000 | 上报 IP / SN / 线数 / 当前模式 | 可选（可手工填 IP） |
| 点云数据 | 雷达 → 上位机 | UDP（端口由设备配置决定） | 点云/原始回波/直方图 | 必需 |
| 设备管理 | 上位机 → 雷达 | HTTP 80（新固件）/ SSH 22 | 查状态、切模式、改推流地址、重启 | 可选 |

关键点：**数据是雷达主动推给上位机的**，上位机只负责监听，不需要也不应该向数据端口发包。

---

## 2. 网络与端口

| 项 | 默认值 | 说明 |
|---|---|---|
| 雷达 IP | `169.254.7.30`（本机实测） | 也可为 `192.168.1.x` 等，以广播或现场配置为准 |
| 上位机 IP | 由设备配置 `center_ip` 指定 | 必须是上位机与雷达同网段网卡上的地址 |
| 数据端口 | `center_data_port`，默认 `50007` | 上位机在本机该端口监听 |
| HTTP | 80 | 新固件固定开启 |
| SSH | 22 | 两代固件都有，账号 `root` |

实测要点：

- **数据包源端口是随机的**（见过 39826 / 51615 / 54359），不能按源端口过滤，只能按源 IP + 目的端口。
- 雷达与 PC 建议直连或同一交换机；跨网段需要 PC 有到 `169.254.0.0/16` 的路由。

---

## 3. 设备发现（UDP 55000 广播）

雷达上电后周期性广播（约 1 s 一次），源端口 65000，目的端口 55000。上位机被动监听即可，**不要回包**。

### 3.1 包结构

| 偏移 | 类型 | 含义 |
|---|---|---|
| `[0]` | uint8 | magic = `0xAA` |
| `[1]` | uint8 | 协议版本 = `0x01` |
| `[2..3]` | uint16 LE | `data_len` |
| `[4]` | uint8 | 消息类型 = `0x02`（设备信息） |
| `[5..10]` | — | 保留 |
| `[11..]` | ASCII | 序列号，如 `CS96000B01026470003` |
| `[..]` | uint8 | 当前 `work_mode` |
| `[len-8..len-7]` | uint16 LE | 垂直线数（CS96 = 96，CS120 = 120） |
| `[len-6..len-5]` | uint16 LE | 每帧 slot 数（= 800） |
| `[len-4..len-1]` | uint32 | CRC |

`len` 是收到的 UDP 载荷总长。**线数与 slot 数必须从包尾取**，不要写死 96×800：CS120 Gamma 是 120×800，包长也不同（见 4.2）。

### 3.2 发现失败时的替代方案

新固件在部分网络状态下广播可能被丢弃。实测有效的兜底顺序：

1. 已知 IP 直连：`TCP 80 /cgi-bin/lidar_status.cgi` 返回 JSON 即在线（新固件）。
2. `TCP 22` 可连接 → 老固件基本可判定在线。
3. 同网段 `TCP 80` 扫描（254 个地址、0.4 s 超时、并发 128，约 1 s 完成）。

---

## 4. 点云数据通道

### 4.0 推流目标

设备把数据发往 `center_ip:center_data_port`，这两个值在设备的 `ys_conf.ini` 里。

**新固件注意**：该文件有两个副本，改错位置等于没改。

| 文件 | 角色 |
|---|---|
| `/run/lidar-data/lidar/config/ys_conf.ini` | 基准配置，**改这个** |
| `/run/lidar/manager/ys_conf.ini` | 运行期配置，`mgrd` 每次拉起 `you_see` 时用基准配置重新生成 |

改完基准配置必须重启服务（见 6.2），然后**校验运行期配置已经跟随**，否则数据仍发往旧地址。

> **20261125 实测补充：`/run` 是 tmpfs，配置不持久化。**
> 设备的 `/etc/lidar/service.conf` 用 `LIDAR_STORAGE_MODE=boot-copy` 把 jffs2 分区
> （`/dev/mtdblock2`）只读复制到 `/run/lidar-data`，`lidar-storage` 里没有回写路径。
> 结果：**每次断电重启后 `center_ip` / `center_data_port` 都会回落到镜像默认值**
> （本机为 `192.168.110.15:50027`）。上位机必须在每次连接时重新核对并写入，
> 不能"配一次用一辈子"。另外 `lidar-service` 反复重启会进入 `phase=FAULT`，
> 那时改配置不生效，只能断电重启。

### 4.1 work_mode 与包长

| work_mode | 名称 | 包长（96 线） | 包长（120 线） | 内容 |
|---|---|---|---|---|
| `0` | 直方图 histogram | 4105 B | 4105 B | 4 像素 × 512 bin × uint16 |
| `1` | 原始点云 origin | 1561 B | 1945 B | 双回波 ft/pk + 转镜角度 |
| `2` | 真实点云 cartesian | 1266 B | 1578 B | 设备算好的 float xyz |

通式：

```text
mode 2:  18 + lines × 13
mode 1:  9 + 16 + lines × 16
mode 0:  9 + 4096
```

新固件发布包实际允许的模式以 `lidar_status.cgi` 的 `validated_work_modes` 为准（本机为 `[1,2]`）；`supported_modes` 里的 `histogram/gray/spot` 是平台能力名，不等于可用的 work_mode。

**按包长自动识别模式**是可靠做法：三种模式包长互不相同，且与线数一一对应。

---

### 4.2 mode 2：真实点云（1266 B，96 线）

设备端已完成坐标解算，上位机拿到就是直角坐标。

#### 包结构

| 偏移 | 类型 | 含义 |
|---|---|---|
| `[0..9]` | — | 包序号/时间戳等头部，含义未公开，**不要依赖** |
| `[10..11]` | uint16 LE | `frame_id`，取值 `0..3`，对应转镜四个面 |
| `[12..13]` | uint16 LE | `total_slots`，固定 800 |
| `[14..15]` | uint16 LE | `slot_index`，`0..799` |
| `[16..17]` | — | 保留 |
| `[18..]` | 96 × 13 B | 点位数组，按 `line 0 → line 95` 顺序 |

#### 点位（13 B）

| 相对偏移 | 类型 | 含义 |
|---|---|---|
| `+0..3` | float32 LE | x（米） |
| `+4..7` | float32 LE | y（米） |
| `+8..11` | float32 LE | z（米） |
| `+12` | uint8 | 反射率/灰度 `albedo`，0~255 |

#### 实测样本

```text
64 c7 0b 00 10 02 00 00 00 00 | 00 00 | 20 03 | 01 00 | 00 00
                                  ↑frame ↑800   ↑slot1
18 字节后第一点：
90 6a ac 40 = 5.3877   fa b9 61 40 = 3.5270   e6 09 59 3f = 0.8478   b6 = 182
```

**单位与判据**：xyz 单位米；`|x|,|y|,|z| < 1000` 且三元组有限（非 NaN/Inf）视为有效点。无效点通常是 `(0,0,0)` 或 NaN，不要画进点云。

**一帧 = 一个面**：`frame_id` 即面号（0~3），与 `slot_index` 无关。`slot_index` 递增方向对应的水平角方向与直觉相反，但因为 xyz 已算好，直接使用即可。

---

### 4.3 mode 1：原始点云（1561 B，96 线）

标定软件主要用这个模式：拿到的是**回波原始计时值**，角度/距离由上位机按标定表解算。

#### 包头（9 B）

| 偏移 | 类型 | 含义 |
|---|---|---|
| `[0]` | uint8 | magic = `0xAA` |
| `[1..2]` | uint16 LE | `frame_id`，帧内不变，逐帧递增 |
| `[3..4]` | uint16 LE | `slot_index`，`0..799` |
| `[5..6]` | — | 保留（实测与模式/包类型相关） |
| `[7..8]` | uint16 LE | `data_len` = `16 + lines × 16`（96 线为 **1552**） |

校验：`data[0] == 0xAA && data_len == 16 + lines*16 && slot_index < 800`，否则整包丢弃。

#### slot 头（16 B，紧跟包头，偏移 9）

| 相对偏移 | 类型 | 含义 |
|---|---|---|
| `+0..3` | uint32 LE | `word0`，**高 20 位是转镜 PA 角度** |
| `+4..15` | — | 保留/时间戳，含义未公开 |

角度拆解（与设备端 `you_see_arm` 一致）：

```c
uint32_t mirrorAngle = (word0 >> 12) & 0xFFFFF;   // 20 位
int scale     = (mirrorAngle >> 12) & 0xFF;       // 粗刻度 0~240
int realAngle =  mirrorAngle        & 0xFFF;      // 12 位细分 0~4095
```

`scale` 是整圈 240 等分的码盘刻度（每格 1.5°），`realAngle` 是格内细分。

> 实测提醒：`slot 0` 是同步包，PA 恒为 0，**不要用它判面**。

#### 像素数组（`lines × 16 B`，紧接 slot 头，偏移 25）

**注意顺序是倒的**：数组第 0 个元素是 `line = lines-1`，第 `i` 个对应 `line = lines-1-i`。

| 相对偏移 | 类型 | 含义 |
|---|---|---|
| `+0..2` | uint24 LE | `ft0`：回波 1 前沿（ps tick） |
| `+3..5` | — | 保留（`et0`） |
| `+6..7` | uint16 LE | `pk0`：回波 1 峰值 |
| `+8..10` | uint24 LE | `ft1`：回波 2 前沿 |
| `+11..13` | — | 保留（`et1`） |
| `+14..15` | uint16 LE | `pk1`：回波 2 峰值 |

#### 回波选择

设备配置为「单回波 / 最强」，但包里始终带两路。选择规则（`cloud_viewer` 实现，与设备端一致）：

```c
bool ok0 = ft0 > 16 && pk0 > 16;
bool ok1 = ft1 > 16 && pk1 > 16;
// 两路都有效时取 ft 较大者（更远的目标），否则取有效的那路
```

#### 距离解算

```text
r = 299792458e-12 * (ft >> 1) * a[line] + d_delta[line]     // 米
```

- `kLightSpeed = 2.99792458e-4` m/ps（即 299792458 × 1e-12）
- `a[line]`：距离系数，正常为 1.0
- `d_delta[line]`：零点修正，单位**米**
- 有效判据：`ft > 16`（约 0.5 m 盲区外）且 `r < dist_max`（默认 200 m，可配置到 500 m）

#### 俯仰角

```text
theta(line) = (i - u) * (theta_max - theta_min) / lines_v + theta_tilt
其中 i = (lines - 1) - line      // 像素物理顺序倒排
```

参数来自标定 `ys_calibration.ini`：`theta_min = -10.0`、`theta_max = 10.0`、`theta_tilt = -2.5`、`u = 47`。

#### 水平角

```text
hc    = encoder_angle[scale] + 0.0009 * realAngle - 本面起点角
omega = wrap180(hc) * 2 + omega0 + omega_delta[line]
```

- `encoder_angle[scale]`：`ys_encoder_angle.csv` 的 241 行表（`scale 0..240` → 机械角 `0..360`）
- `0.0009 * realAngle`：格内细分折算。`realAngle` 是 12 位（0~4095），设备端取 **0.0009°/LSB**；
  实测一格的 1.5° 只用到约 1700 个 LSB（1700 × 0.0009 ≈ 1.53°），高 12 位是余量
- `本面起点角 = angle_face[face] / 240 * 360 + side_angular_offset[face] / 256 * 1.5`
- `omega0 = -60.0`（水平零位偏移）
- `omega_delta[line]`：逐线微调，本机为 `-1.3`
- 单面视场约 120°，`omega ∈ [-60, 60]`，**不含面间 90° 偏移**；画四个面时需自行叠加 `face × 90°` 或直接用绝对角

#### 面判定

按帧起始（`slot_index` 较小、PA 有效）锁定的 PA 角度落到哪个面段，整帧沿用：

```text
F1 = [315°,360°) ∪ [0°,45°)      F2 = [45°,135°)
F3 = [135°,225°)                 F4 = [225°,315°)
```

帧中途接入（前 10% slot 丢失）时不要猜，等下一帧。

#### 直角坐标合成

```text
x = r * cos(theta) * cos(omega)
y = r * cos(theta) * sin(omega)
z = r * sin(theta)
```

`cloud_viewer` 在合成时对 `omega` 取负，使 `slot 0` 在左、`slot 799` 在右（顺时针扫描）。**如果你的软件与现有标定工具对齐，沿用同一约定**，否则四个面拼起来会镜像。

---

### 4.4 mode 0：直方图（4105 B）

> ⚠️ 本机发布包的 `validated_work_modes` 只有 `[1,2]`，mode 0 **未能在真机验证**。
> 下表来自解析实现，字段名以设备端后续确认为准。

| 偏移 | 类型 | 含义 |
|---|---|---|
| `[0]` | uint8 | 包头首字节（实现未校验） |
| `[1..2]` | uint16 LE | `frame_id` |
| `[3..4]` | — | 保留 |
| `[5..6]` | uint16 LE | 包序号 |
| `[7..8]` | uint16 LE | `data_len` = 4096 |
| `[9..4104]` | 4 × 512 × uint16 LE | 4 个像素 × 512 个 bin |

一帧 = `lines / 4` 个包（96 线 = 24 包，120 线 = 30 包）。数值无量纲，只用于回波波形分析，不参与测距。

### 4.5 组帧与丢包

- 一帧 = 800 个 slot 包（mode 1/2）或 24 个包（mode 0）。
- `slot_index` 会乱序、会丢，**按 index 落位，不要按到达顺序**。
- UDP 接收缓冲建议 ≥ 8 MB（`SO_RCVBUF`），否则高帧率下内核丢包。
- 一帧收到 ≥ 90% 的 slot 即可发布；缺的 slot 保留上一帧或标记无效，不要整体丢弃。
- 实测 96×800 时持续丢包率约 2%（`dropSlots=929 / 54 帧`），属于接收端缓冲不足，不是设备问题。

---

## 5. 管理通道 A：HTTP CGI（新固件）

只在 2026-11 起的 `lidar_mgrd` 固件上存在。旧固件访问返回 404 / HTML，可用这一点做固件类型探测。

### 5.1 查状态

```http
GET /cgi-bin/lidar_status.cgi
```

响应（200）：

```json
{
  "code": 200, "message": "状态", "model": "CS96-beta",
  "operation_id": "b064b150c930b1a30ec5690779c00bd1",
  "version": null, "simulation": false,
  "phase": "COMMITTED",
  "confirmed_mode": 2, "actual_mode": 2, "desired_mode": 2,
  "error": "", "pid": 808,
  "initialized": true, "pipeline_verified": true,
  "supported_modes": ["histogram", "pointcloud", "gray", "spot"],
  "validated_work_modes": [1, 2]
}
```

| 字段 | 用法 |
|---|---|
| `phase` | `STOPPING` / `STARTING` / `COMMITTED`；只有 `COMMITTED` 且 `actual_mode == desired_mode` 才算切换完成 |
| `actual_mode` | `-1` 表示正在停采，**不能当成功** |
| `validated_work_modes` | 该发布包真正允许的 work_mode，UI 上只允许切这些 |
| `pid` | 可用于判断是否真的重启过（前后对比） |
| `error` | 非空即失败原因 |

**固件类型判定**：必须同时满足「`code` 是整数」+「含 `phase` / `model` / `actual_mode` / `validated_work_modes` 中至少一个字段」。只看 `code` 会把旧固件的其它 CGI 误判成新固件。

### 5.2 切模式

```http
POST /cgi-bin/lidar_switch.cgi
Content-Type: application/json

{"work_mode": 1}
```

**body 只允许这一个键**，多加字段整包被拒。

受理响应（202）：

```json
{"code":202,"message":"切换已受理","phase":"STOPPING",
 "actual_mode":-1,"desired_mode":1,"confirmed_mode":2,"pid":1985,...}
```

之后轮询状态，直到：

```text
phase == "COMMITTED" && actual_mode == 目标 && confirmed_mode == 目标
```

实测切换耗时 4~10 s，建议轮询上限 30 s、间隔 400 ms。

### 5.3 错误

| 现象 | 原因 |
|---|---|
| `code: 400`，`message` 提到「只包含整数 work_mode」 | body 多了字段或类型不对 |
| `code: 200` 但 `phase` 长期停在 `STOPPING` | 驱动启动失败，读 `error` |
| 连不上 | 设备重启中 / 该固件没有此接口 |

---

## 6. 管理通道 B：SSH

两代固件都保留。**改推流地址只能用 SSH**（HTTP 接口不支持改 `center_ip`）。

登录：`root@<ip>`，密码认证；新固件默认密码与旧固件相同（现场为 `111`，以下用 `<pass>` 占位）。

三条实测注意：

- **sshd 比 web 慢**：上电后 80 先可用，22 要再等十几秒；探测不到就重试，别急着重启设备。
- **`set_ssh.cgi` 在新固件上已停用**（`{"code":501,"msg":"此旧入口已停用，请使用配套管理接口。"}`），
  `reboot.cgi` 同样返回 501，都不再是恢复通道。
- **主机密钥每次重启都变**（机器密钥不持久化），客户端会看到 host key 变化，属正常；
  做 TOFU 的程序要允许"同 IP 密钥变更后确认并重试"，否则每次重启后 SSH 都会失败。

### 6.1 旧固件

| 操作 | 命令 |
|---|---|
| 配置 | `/etc/ys_conf.ini` |
| 重启 | `pids=$(pidof you_see); kill -9 $pids; cd /root && setsid ./you_see >>/tmp/you_stdout.log 2>&1 &` |
| 模式 | 改 ini 的 `work_mode` 后重启 |
| 标定 | `/etc/ys_calibration.ini` + `/etc/ys_encoder_angle.csv` |

### 6.2 新固件

| 操作 | 命令 |
|---|---|
| 基准配置 | `/run/lidar-data/lidar/config/ys_conf.ini` |
| 运行期配置 | `/run/lidar/manager/ys_conf.ini`（只读，mgrd 生成） |
| 重启 | `/usr/libexec/lidar/lidar-service restart`（**不要 kill you_see**，会和守护进程抢） |
| 状态 | `/usr/libexec/lidar/lidar-service status` |
| 直接控制 | `/run/lidar-data/lidar/current/bin/lidar_mgrctl /run/lidar/control.sock status\|switch N` |
| 标定 | `/run/lidar-data/lidar/calibration/`（`/etc/ys_calibration.ini` 是软链） |
| 模式 | 用 HTTP，**不要改 ini**：`work_mode` 每次都被 mgrd 重写 |

### 6.3 改推流地址的标准流程（新固件）

```sh
conf=/run/lidar-data/lidar/config/ys_conf.ini
runtime=/run/lidar/manager/ys_conf.ini

cp -p "$conf" /tmp/backup
sed -i 's|^center_ip.*|center_ip = "<PC_IP>" ;上位机IP|'       "$conf"
sed -i 's|^center_data_port.*|center_data_port = "50007" ;上位机数据端口|' "$conf"

/usr/libexec/lidar/lidar-service restart

# 必须校验运行期配置真的跟上了，否则数据仍发往旧地址
for i in $(seq 20); do
  grep -q "\"<PC_IP>\"" "$runtime" && grep -q '"50007"' "$runtime" && break
  sleep 1
done
```

失败要**回滚**（把 `/tmp/backup` 拷回并再次 restart），否则会留下一个指向错误上位机的雷达。

---

## 7. 标定文件（原始点云解算必需）

### 7.1 文件清单

| 文件 | 作用 | 缺失后果 |
|---|---|---|
| `ys_calibration.ini` | 距离系数 `a[]`、零点 `d_delta[]`、俯仰参数、面起始角、`omega_delta[]` | 无法解算，设备端直接拒绝启动 |
| `ys_encoder_angle.csv` | `scale 0..240` → 机械角 `0..360` | 面内水平角错误 |
| `reflectivity_map.csv` | 反射率查表 | 可缺，退化为峰值归一化 |
| `spot_slope_map.csv` / `angle_calib_YYYYMMDD.csv` | FPGA 旋转角曲线 | 可缺，细角度精度下降 |
| `time_intervals.txt` | 每格实测耗时（us） | 可缺，用名义值 |

`ys_calibration.ini` 关键字段（本机实测值）：

```ini
[parameter]
lines_v = 96
lines_h = 800
omega0 = -60.0
theta_min = -10.0
theta_max = 10.0
theta_tilt = -2.5
u = 47
sat_peak_max = 1612
sat_peak_min = 0
dist_max = 200
fov_max = 120.0
k = 1.0

[standard_encoder]
angle_face_1 = 3000
angle_face_2 = 19384
angle_face_3 = 35768
angle_face_4 = 52152

[side_angular_offset]
side0_angular_offset = 0
...
```

### 7.2 ⚠️ `standard_encoder` 有两代量纲

这是新旧固件最容易踩的差异：

| 版本 | 取值 | 量纲 | 换算 |
|---|---|---|---|
| 旧标定包 | `0 ~ 240` | PA 计数，1 计数 = 1.5° | 直接用 |
| 2026-11 起新包 | `0 ~ 65535` | 20 位原始编码器值，65536 = 360° | `× 240 / 65536` 折算成 PA 计数 |

本机新包的 `3000/19384/35768/52152` 折算后是 `10.986/70.977/130.981/190.986` PA 计数，
乘 1.5° 得 `16.48/106.47/196.47/286.48`，四个面正好相差 90° —— 可作为量纲判据。

**实现建议**：读入后先判断 `max(angle_face_*) > 240`，是则整体折算，再走统一的几何公式。
不要按文件路径或固件版本分支，容易和混装的标定包冲突。

### 7.3 编码器表有两种布局

`ys_encoder_angle.csv` 的 `angle` 列可能：

1. **绝对表**（旧）：单调递增 `0 → 360`，本机当前就是这种。
2. **每面锚定表**（标定端 `anchor_per_face=true`）：表值含负数或 > 360，四个面段互不连续，
   每面从锚点（默认 -30° 机械）重新起算。

判据：表里出现 `< -0.5` 或 `> 360.5` 的值即为锚定表。两种布局下 `hc` 的物理含义相同，
但锚定表需要额外记录每面的 `dialBase`（绝对角 = 表值 + dialBase）才能还原 PA 判面。

---

## 8. 典型时序

### 8.1 首次接入

```text
1. 监听 UDP 55000 广播（约 2 s）→ 得到 IP / SN / line_count / slots / work_mode
2. GET /cgi-bin/lidar_status.cgi
   ├─ 成功 → 新固件：用 actual_mode 初始化，validated_work_modes 限定可切集合
   └─ 失败 → 旧固件：按广播的 work_mode + 22 端口探测
3. 校验 center_ip / center_data_port 是否指向本机（不匹配则按 6.3 改）
4. 在 center_data_port 上监听 → 按首包长度判定模式
5. 加载标定（mode 1 必需）→ 解算显示
```

### 8.2 切换模式

```text
POST lidar_switch.cgi {"work_mode":N}   → 202
轮询 lidar_status.cgi 直到 COMMITTED && actual_mode==N      （4~10 s）
清空接收缓冲（旧模式的残留包会污染新帧）
继续收包
```

注意：切模式会让 `mgrd` 重新拉起驱动，**推流地址会回落到基准配置的值**。
如果基准配置指向别处，切完模式就收不到数据 —— 这是现场最容易误判为「切模式把设备搞坏了」的原因。

### 8.3 改推流地址

```text
SSH: 改基准配置 → lidar-service restart → 校验运行期配置（最多 20 s）
失败：回滚 + 再 restart
成功：本机在新端口重新监听
```

---

## 9. 已知差异与坑

1. **`/etc/ys_conf.ini` 在新固件上不存在**，硬编码这个路径的老程序会直接失败。
2. **`work_mode` 写 ini 无效**（新固件），必须走 HTTP。
3. **不要 kill `you_see`**（新固件），会被 mgrd 重拉并可能与你的 PID 校验打架。
4. **`slot 0` 的 PA 恒为 0**，不能用于判面。
5. **像素数组倒序**：数组第 0 个是 `line = lines-1`。
6. **两路回波都有效时取 `ft` 大的那路**，不是取峰值大的（旧版 `cloud_viewer` 曾按峰值，实测会选错）。
7. **数据源端口随机**，别用它过滤。
8. **切模式后运行期配置回落**（见 8.2）。
9. **`standard_encoder` 量纲两代**（见 7.2），这是标定软件必须处理的。
10. **`dist_max` 可配置**（200 → 500 米），解算时不要写死。
11. **HTTP 管理接口的「冷链路第一个请求必丢」**（实测 PC 有线直连 CS96 beta）：
    链路空闲约 10 s 后，**第一个** TCP 请求会被丢弃（connect 超时或收到 0 字节），
    紧接着再发一次只要十几毫秒就成功。成因在链路层（ARP 邻居表重新解析 / 设备 PHY
    省电唤醒），**跟设备是否在线、CGI 是否正常无关**。
    - 现象：状态查询随机失败、切模式报「管理接口不可达」，用户点第二次就好了。
    - 处理：所有 HTTP 调用（status / switch）必须自带 2~3 次重试，间隔 250~300 ms；
      POST 重试前先查一次 status，若 `desired_mode` 已是目标值说明上次其实已受理
      （只是响应丢了），直接进入轮询，避免一次点击触发两轮切换。
    - 不要用「第一次失败」当作旧固件的判据 —— 旧固件的特征是返回 HTML/404，
      不是超时。

---

## 10. 附录

### 10.1 抓包样本（mode 2，1266 B）

```text
64 c7 0b 00 10 02 00 00 00 00  00 00  20 03  01 00  00 00
                               frame=0 slots=800 slot=1
点 0: 90 6a ac 40 | fa b9 61 40 | e6 09 59 3f | b6
       x=5.3877      y=3.5270      z=0.8478    albedo=182
```

### 10.2 抓包样本（mode 1，1561 B）

```text
aa 56 01 b9 01 01 00 10 06
   frame=342 slot=441      data_len=1552
slot 头: b9 01 38 1e ...   → word0=0x1e3801b9 → PA=(0x1e3801b9>>12)&0xFFFFF=0x1e380
        scale=(0x1e380>>12)&0xFF=0x1E=30   realAngle=0x380=896
像素 0（对应 line 95）: 23 9c 00 | c9 c6 00 | 92 04 | be a0 00 | fb c6 00 | 92 04
        ft0=0x009c23=39971  pk0=0x0492=1170
        ft1=0x00a0be=41150  pk1=0x0492=1170
        r0 = 2.99792458e-4 * (39971>>1) ≈ 5.99 m
        r1 = 2.99792458e-4 * (41150>>1) ≈ 6.17 m
```

### 10.3 参考实现

同仓库 `src/cloud_viewer/stream/BetaUdpStream.cpp`：

- `parseCartesian` / `parseOrigin` / `parseHistogram`：三种模式的完整解析
- `packetSize` 常量：包长与线数的关系
- `src/cloud_viewer/preprocess/Cs96BetaCalibration.cpp`：标定加载与解算（含两代 `standard_encoder` 归一化）
- `src/cloud_viewer/platform/LidarServiceClient.cpp`：HTTP 状态/切换客户端（无第三方依赖，Windows/Linux 通用）

### 10.4 调试命令

```bash
# 抓一包看长度和头部（Linux）
python3 - <<'PY'
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
s.bind(("0.0.0.0", 50007))
s.settimeout(15)
d, a = s.recvfrom(9000)
print(a, len(d)); print(d[:48].hex(" "))
PY

# 设备端状态
curl -s http://<ip>/cgi-bin/lidar_status.cgi
/usr/libexec/lidar/lidar-service status
```
