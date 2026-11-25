# CS96-gamma 点云数据组帧协议（UDP）

> 适用固件：CS96-gamma（SL100 SPAD，每 SLOT 120 像素）
> 传输方式：UDP，单播到上位机数据端口（默认 50027）
> 本文面向**上位机解析实现**，给出可直接编码的字节级定义

本文描述 `work_mode=2` 的解算后点云。核对 FPGA 原始数据请使用 `work_mode=1`，参见 [CS96-gamma 原始点云数据组帧协议](CS96-gamma-原始点云-数据组帧协议.md)。

## 1. 概述

板端把每个 SLOT 解析成点云后，**一个 SLOT 打成一个 UDP 包**直接发送，**不附加任何额外封装**（UDP payload 就是下文结构）。

一帧（转镜一个面）发送 `total_slots` 个包，包内 `slot_number` 从 0 递增。

## 2. 按包长区分机型（重要）

上位机现有的「按 UDP 包长判类型」机制对 gamma 依然适用，**CS96-gamma 的包长与 beta 不同**：

| 机型 | 像素数/SLOT | 回波模式 | 单包长度 |
|---|---|---|---|
| CS96-alpha | 96 | 单回波 | 1266 |
| CS96-beta | 96 | 单回波 | 1266 |
| CS96-beta | 96 | 双回波 | 2514 |
| **CS96-gamma** | **120** | **单回波** | **1578** |
| **CS96-gamma** | **120** | **双回波** | **3138** |

统一公式：

```
包长 = 18 + 像素数 × 回波数 × 13
      (协议头)  (120)   (1 或 2)  (每点 13 字节)
```

判别要点：

- **1578 / 3138 → CS96-gamma**
- 1266 / 2514 → CS96-alpha 或 beta（两者包长相同，需靠 SN 或广播包区分，见 §7）
- 直角坐标与球坐标系**包长相同**（每点都是 13 字节），需读 `coordinate_mode` 字段区分（见 §5）

## 3. UDP Payload 结构

```
+---------------------------+
| 协议头 18 字节             |  point_cloud_head_t
+---------------------------+
| 点数据 13 字节 × N         |  cartesian_point_data_t / spherical_point_data_t
| N = 像素数 × 回波数        |  gamma 单回波 N = 120，双回波 N = 240
+---------------------------+
```

结构体按 **1 字节对齐（packed，无填充）**，多字节整数与 float 均为**小端序**。

### 3.1 协议头（18 字节）

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---|---|---|---|---|
| 0 | 10 | uint8[10] | `delay` | 时间戳，**gamma 只有低 3 字节有效**，见 §4 |
| 10 | 2 | uint16 | `frame_index` | 面号，取值 0~3（4 面转镜） |
| 12 | 2 | uint16 | `total_slots` | 本帧（一个面）的 SLOT 总数：10Hz 时 800，20Hz 时 400 |
| 14 | 2 | uint16 | `slot_number` | 本包 SLOT 序号，0 ~ total_slots-1 |
| 16 | 1 | uint8 | `echo_mode` | 0 = 单回波，1 = 双回波 |
| 17 | 1 | uint8 | `coordinate_mode` | 0 = 直角坐标系，1 = 球坐标系 |

### 3.2 点数据（13 字节/点）

**`coordinate_mode == 0`（直角坐标系）**

| 相对偏移 | 长度 | 类型 | 字段 | 单位 |
|---|---|---|---|---|
| 0 | 4 | float | `x` | 米 |
| 4 | 4 | float | `y` | 米 |
| 8 | 4 | float | `z` | 米 |
| 12 | 1 | uint8 | `albedo` | 反射率 0~255 |

**`coordinate_mode == 1`（球坐标系）**

| 相对偏移 | 长度 | 类型 | 字段 | 单位 |
|---|---|---|---|---|
| 0 | 4 | float | `r` | 米 |
| 4 | 4 | float | `theta` | 弧度（俯仰角） |
| 8 | 4 | float | `omega` | 弧度（水平转角） |
| 12 | 1 | uint8 | `albedo` | 反射率 0~255 |

**点的顺序**：第 i 个点对应 SLOT 内第 i 个垂直像素，i 从 0 递增。直角坐标已含位置信息，通常无需关心顺序；若需按行组织点云，i 与垂直角度的对应关系由标定表决定（板端 `g_theta[]`）。

**双回波模式**：`echo_mode == 1` 时，同一个像素会连续输出 2 个点（回波 1、回波 2），因此点数为 `像素数 × 2`，包长为单回波的两倍。

## 4. `delay` 字段（CS96-gamma 与 beta 的差异点）

**gamma（SL100）**：硬件只提供 **24 位相对时间，单位微秒**。板端按小端填入 `delay[0..2]`，`delay[3..9]` 恒为 0。

解析方式（两者等价）：

```c
uint32_t rel_us = delay[0] | (delay[1] << 8) | (delay[2] << 16);
```

```python
rel_us = int.from_bytes(pkt[0:3], 'little')
```

示例：`delay = 33 83 03 00 00 00 00 00 00 00` → `rel_us = 0x033833 = 230259`

**beta**：`delay` 是硬件给出的 10 字节时间戳，格式与 gamma 不同（上位机已有 beta 的处理，不要复用）。

> ⚠️ 该字段的语义为「SLOT 采集时刻的相对时间（µs）」，上位机如需绝对时间应结合首包时间做基线对齐。

## 5. 分辨率与总量

| 参数 | 值 | 来源 |
|---|---|---|
| 垂直像素数 | 120 | 标定文件 `lines_v` |
| 每面 SLOT 数 | 800（10Hz）/ 400（20Hz） | 协议头 `total_slots` 字段 |
| 每包点数 | 120（单回波）/ 240（双回波） | 由包长与 `echo_mode` 推导 |
| 面数 | 4 | `frame_index` 0~3 |

一帧（一个面）的完整点云 = `total_slots` 个 UDP 包，按 `slot_number` 排序即可。

## 6. 解析示例

### 6.1 Python

```python
import struct

HEAD_LEN   = 18
POINT_LEN  = 13

def parse_packet(pkt: bytes):
    """解析一个 CS96-gamma 点云 UDP 包"""
    length = len(pkt)

    # 1) 按包长判定机型/回波模式
    if length == 1578:
        echo_mode = 0; n_points = 120
    elif length == 3138:
        echo_mode = 1; n_points = 240
    else:
        raise ValueError(f"非 CS96-gamma 包长: {length}")

    # 2) 协议头（小端）
    rel_us       = int.from_bytes(pkt[0:3], 'little')   # gamma: 低 3 字节有效
    frame_index  = struct.unpack_from('<H', pkt, 10)[0]
    total_slots  = struct.unpack_from('<H', pkt, 12)[0]
    slot_number  = struct.unpack_from('<H', pkt, 14)[0]
    echo_mode_f  = pkt[16]
    coord_mode   = pkt[17]

    assert echo_mode_f == echo_mode, "包长与 echo_mode 不一致"
    assert n_points == (length - HEAD_LEN) // POINT_LEN, "包长与点数不一致"

    # 3) 点数组
    points = []
    off = HEAD_LEN
    for _ in range(n_points):
        if coord_mode == 0:
            x, y, z = struct.unpack_from('<fff', pkt, off)
            albedo  = pkt[off + 12]
            points.append((x, y, z, albedo))
        else:
            r, theta, omega = struct.unpack_from('<fff', pkt, off)
            albedo = pkt[off + 12]
            points.append((r, theta, omega, albedo))
        off += POINT_LEN

    return {
        'rel_time_us': rel_us,
        'frame_index': frame_index,
        'total_slots': total_slots,
        'slot_number': slot_number,
        'echo_mode':   echo_mode,
        'coord_mode':  coord_mode,
        'points':      points,
    }
```

> 说明：`(length - HEAD_LEN) // POINT_LEN` 在单回波时得 120、双回波时得 240，与 `n_points` 恒等，可用作包完整性的自检。

### 6.2 C

```c
#include <stdint.h>
#include <string.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t  delay[10];
    uint16_t frame_index;
    uint16_t total_slots;
    uint16_t slot_number;
    uint8_t  echo_mode;
    uint8_t  coordinate_mode;
} point_cloud_head_t;               /* 18 字节 */

typedef struct {
    float   x;        /* 或 r     */
    float   y;        /* 或 theta */
    float   z;        /* 或 omega */
    uint8_t albedo;
} point_data_t;                     /* 13 字节 */
#pragma pack(pop)

/* 按包长判定 */
int is_cs96_gamma_packet(int len, int *echo_mode, int *n_points)
{
    if (len == 18 + 120 * 13) { *echo_mode = 0; *n_points = 120; return 1; }
    if (len == 18 + 240 * 13) { *echo_mode = 1; *n_points = 240; return 1; }
    return 0;
}

void parse(const uint8_t *pkt, int len)
{
    int echo_mode, n_points;
    if (!is_cs96_gamma_packet(len, &echo_mode, &n_points))
        return;                       /* 非 gamma 包 */

    const point_cloud_head_t *h = (const point_cloud_head_t *)pkt;
    uint32_t rel_us = (uint32_t)h->delay[0]
                    | ((uint32_t)h->delay[1] << 8)
                    | ((uint32_t)h->delay[2] << 16);

    const point_data_t *pts = (const point_data_t *)(pkt + 18);
    for (int i = 0; i < n_points; i++) {
        /* h->coordinate_mode == 0：pts[i].x/y/z 为米 */
        /* h->coordinate_mode == 1：pts[i].x/y/z 依次为 r/theta/omega */
        (void)pts[i];
    }
}
```

## 7. 机型识别（可选，比包长更可靠）

若上位机需要精确区分机型，可通过广播包中的 `sn` 字段（握手/广播报文里）：

| 机型 | SN 前缀 |
|---|---|
| CS96-alpha | `CS96000A…` |
| CS96-beta | `CS96000B…` |
| CS96-gamma | `CS96000C…` |

> gamma 的正式 SN 编码待产品最终确认，若后续调整以广播包实际值为准。

## 8. 验证程度与接入自检

### 8.1 已验证

- **包长 1578 为实机实测**：gamma 实机（10Hz、单回波、直角坐标）运行统计
  ```
  [UDP-ST] 40201 pkts/5.0s: len1578=40201(100.00%) other=0(0.00%)
  ```
  40201 包 / 5s ≈ 8000 包/s = 800 SLOT × 10Hz，与「每 SLOT 一个包」一致；无异常包长。
- **结构定义直接取自板端 C 结构体**（`point_cloud_head_t` / `cartesian_point_data_t`，`#pragma pack(1)`）。
- 点数关系自洽：`(1578 - 18) / 13 = 120`，与标定文件 `lines_v = 120` 一致。

### 8.2 尚未逐字节核对的部分

板端只有在与上位机**握手成功后**才会真正外发数据（否则处于广播模式），因此抓包核对需要在**上位机侧**完成。以下两项建议在上位机首次接入时打印原始 hex 确认：

1. `delay[0..2]` 的实际数值是否连续递增（验证 §4 的小端 24 位 µs 解释）
2. 点数据的 `x/y/z` 数值范围是否合理（如距离在米级、量程内）

### 8.3 接入自检清单

```
1. 收到包长是否为 1578（或双回波 3138）？        → 否则不是 gamma 包
2. 偏移 16 的 echo_mode 是否与包长自洽？
   （1578 → 0，3138 → 1）
3. 偏移 12 的 total_slots 是否为 800（10Hz）/ 400（20Hz）？
4. slot_number 是否在 0 ~ total_slots-1 范围内递增？
5. frame_index 是否在 0~3 之间？
6. 取第 1 个点，x/y/z 是否在合理量程内（如 |x|,|y| < 200 米）？
```

任一项不符时，请反馈实际的 `hexdump -C` 前 32 字节，便于定位。

## 9. 排错提示

| 现象 | 可能原因 |
|---|---|
| 收到 1266 字节包 | 板端跑的是 beta 固件/配置，不是 gamma |
| 包长正确但点全为 0 | 板端读到空数据（历史上曾因内核驱动 SLOT 步进不匹配导致，需 `mode:pcl_sl100`） |
| `slot_number` 不连续或有跳变 | 丢包（UDP 无序/丢包属正常），上位机应按 `slot_number` 重组 |
| `coordinate_mode` 与预期不符 | 板端 `ys_conf.ini` 的 `coordinate_mode` 配置项 |
