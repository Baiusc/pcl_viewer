# CS96-gamma 灰度图数据组帧协议（UDP）

> 版本：v1.0
> 日期：2026-10-12
> 适用固件为 CS96-gamma（SL100 SPAD，每 SLOT 120 像素），板端配置 `system:work_mode = 3`。
> 本文面向**上位机解析实现**，给出可直接编码的字节级定义

## 1. 概述

板端把驱动 `MODE_GRY_SL100` 通道上传的每个 SLOT **原样透传**给上位机，每个 SLOT 打成一个 UDP 包。

- 传输方式：UDP，单播到上位机数据端口（默认 `50027`）
- 单个 UDP 数据报长度：**固定 977 B** = 9 B 协议头 + 968 B 数据区
- 字节序：小端（ARM64 板端原生小端）
- 帧组织：**1 帧 = 20 个 SLOT**，每个 DMA 批次包含 10 个 SLOT，`slot_number` 从 0 递增到 19

> 使用 `gamma` 构建和统一 AM6204 内核驱动。应用按 `work_mode=3` 选择 `MODE_GRY_SL100`，每次读取 9680 字节。FPGA 须输出对应的灰度格式。

## 2. 单个 UDP 数据报格式

```
┌──────────┬───────────┬─────────────┬──────────┬───────────┬────────────────────┐
│  sof     │  frame_id │ slot_number │ seq_num  │ data_len  │  value             │
│  1 B     │  2 B      │  2 B        │  2 B     │  2 B      │  968 B             │
│  0xAA    │  小端     │  0~19       │  批次索引│  968      │  1 个原始 SLOT     │
└──────────┴───────────┴─────────────┴──────────┴───────────┴────────────────────┘
偏移: 0          1           3           5          7          9                977
```

### 2.1 协议头（9 B，1 字节对齐，`raw_protocal_data_packet_t`）

| 偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `sof` | 起始字节，固定 `0xAA` |
| 1 | 2 | `frame_id` | 帧号，`0~65535` 循环递增；**每 20 个 SLOT 自增一次**（`slot_number` 回到 0 时） |
| 3 | 2 | `slot_number` | SLOT 序号 `0~19`，一帧内唯一，用于帧内排序/丢包检测 |
| 5 | 2 | `seq_num` | 从驱动批次继承的序号（`0~9`）；**灰度图一个 SLOT 即一个完整包，上位机可直接忽略此字段**，用 `slot_number` 排序即可 |
| 7 | 2 | `data_len` | 数据区长度，固定 `968` |

### 2.2 数据区（968 B，1 个原始 SLOT）

```
偏移 0        8                                    968
     ├─ 8B 头 ─┼─ 8B(pixel0) ─ 8B(pixel1) ─ … ─ 8B(pixel119) ─┤
```

| 相对偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 0 | 8 | SLOT 头 | `[4:0]` = SLOT 序号（与协议头 `slot_number` 同值）；`[63:5]` 保留 |
| 8 | 8 | pixel0 | 第 0 个垂直像素的两个回波灰度值 |
| 8 + i×8 | 8 | pixel i | 第 i 个垂直像素（i = 0~119） |
| 960 | 8 | pixel119 | 第 119 个垂直像素 |

### 2.3 像素编码（8 B）

每个像素 8 字节，按小端 64 位整数解析：

| 位域 | 字段 | 说明 |
|---|---|---|
| `[19:0]` | ECHO1 灰度值 | 20 bit，0 ~ 1048575 |
| `[39:20]` | ECHO2 灰度值 | 20 bit，0 ~ 1048575 |
| `[63:40]` | 保留 | 忽略 |

提取方式（两种等价）：

```c
uint64_t v = pixel_data;                 /* 小端 64 位读入 */
uint32_t echo1 = (uint32_t)(v & 0xFFFFF);
uint32_t echo2 = (uint32_t)((v >> 20) & 0xFFFFF);
```

```python
v = int.from_bytes(pkt[off:off+8], 'little')
echo1 = v & 0xFFFFF
echo2 = (v >> 20) & 0xFFFFF
```

## 3. 帧结构

| 项 | 值 | 说明 |
|---|---|---|
| 每帧 SLOT 数 | 20 | 与驱动 DDR 缓冲一致 |
| 每 SLOT 像素数 | 120 | 标定文件 `lines_v` |
| 每帧数据量 | 20 × 968 = 19360 B | 即 20 × 120 = 2400 个像素 |
| 每帧 UDP 包数 | **20** | 一个 SLOT 一个包 |
| 帧边界 | `slot_number` 回到 0 | 同时 `frame_id` 自增 |

上位机按 `frame_id` 分组，每组 20 个包，按 `slot_number`（0~19）排序后拼成 **20 × 120** 的灰度矩阵（行 = SLOT，列 = 垂直像素）。

> ⚠️ **`frame_id` 存在已知抖动（实机实测）**：驱动 DDR 环形缓冲在极少数时刻会出现「块相位漂移」——应用层按固定地址步进读取时，会读到相邻块的内容（实测约每秒 0.2 次，板端日志表现为 `[Gray] slot0 错位` 告警）。**槽号本身在每个块内始终连续**，因此：
> - **上位机应以 `slot_number`（0~19）为准重组**，不要假设「同一 `frame_id` 必然恰好 20 个包」；
> - 某个 `frame_id` 可能出现包数偏多/偏少，按 §4 的「未收齐处理」容错即可。

## 4. 机型 / 模式识别

板端在指令通道（广播端口默认 `55000`）周期性广播，广播包携带：

- `sn`（18 B）：CS96-gamma 为 `CS96000C0126020001`（待产品最终确认）
- `work_mode`：**`3` = 灰度图模式**；点云模式为 1/2

判别要点：

- **`work_mode == 3` + 包长 977 → CS96-gamma 灰度图包**
- 点云包长为 1578 / 3138（gamma）、1266 / 2514（alpha/beta），与 977 不冲突，不会误判

## 5. 解析示例

### 5.1 Python

```python
import struct

HEAD_LEN  = 9
SLOT_LEN  = 968
PIXELS    = 120
SLOT_HEAD = 8

def parse_packet(pkt: bytes):
    """解析一个 CS96-gamma 灰度图 UDP 包"""
    if len(pkt) != HEAD_LEN + SLOT_LEN:
        raise ValueError(f"非灰度图包长: {len(pkt)}")
    if pkt[0] != 0xAA:
        raise ValueError("sof 非 0xAA")

    frame_id     = struct.unpack_from('<H', pkt, 1)[0]
    slot_number  = struct.unpack_from('<H', pkt, 3)[0]
    seq_num      = struct.unpack_from('<H', pkt, 5)[0]
    data_len     = struct.unpack_from('<H', pkt, 7)[0]
    assert data_len == SLOT_LEN, "data_len 与包长不一致"

    slot = pkt[HEAD_LEN:HEAD_LEN + SLOT_LEN]
    slot_no_in_payload = slot[0] & 0x1F           # SLOT 头内的槽号，应与 slot_number 相同

    echo1 = [0] * PIXELS
    echo2 = [0] * PIXELS
    for i in range(PIXELS):
        off = SLOT_HEAD + i * 8
        v = int.from_bytes(slot[off:off + 8], 'little')
        echo1[i] = v & 0xFFFFF
        echo2[i] = (v >> 20) & 0xFFFFF

    return {
        'frame_id':    frame_id,
        'slot_number': slot_number,
        'seq_num':     seq_num,
        'echo1':       echo1,   # 120 个 ECHO1 灰度值
        'echo2':       echo2,   # 120 个 ECHO2 灰度值
    }
```

### 5.2 C

```c
#include <stdint.h>
#include <string.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t  sof;          /* 0xAA */
    uint16_t frame_id;
    uint16_t slot_number;  /* 0~19 */
    uint16_t seq_num;
    uint16_t data_len;     /* 968 */
    uint8_t  value[968];   /* 1 个原始 SLOT */
} gray_protocal_data_packet_t;  /* 977 字节 */

typedef struct {
    uint64_t data;         /* bit[19:0]=ECHO1, bit[39:20]=ECHO2 */
} gray_pixel_data_t;        /* 8 字节 */
#pragma pack(pop)

static inline uint16_t gray_slot_number(const uint8_t *value)
{
    return (uint16_t)(value[0] & 0x1F);
}

static inline uint32_t gray_pixel_echo1(const gray_pixel_data_t *p)
{
    return (uint32_t)(p->data & 0xFFFFF);
}

static inline uint32_t gray_pixel_echo2(const gray_pixel_data_t *p)
{
    return (uint32_t)((p->data >> 20) & 0xFFFFF);
}

void parse(const uint8_t *pkt, int len)
{
    if (len != (int)sizeof(gray_protocal_data_packet_t))
        return;                                   /* 非灰度图包 */

    const gray_protocal_data_packet_t *p = (const gray_protocal_data_packet_t *)pkt;
    if (p->sof != 0xAA || p->data_len != 968)
        return;

    uint16_t slot_number = gray_slot_number(p->value);
    const gray_pixel_data_t *pixels = (const gray_pixel_data_t *)(p->value + 8);
    for (int i = 0; i < 120; i++) {
        uint32_t e1 = gray_pixel_echo1(&pixels[i]);
        uint32_t e2 = gray_pixel_echo2(&pixels[i]);
        (void)e1; (void)e2;
    }
}
```

## 6. 常量速查

| 项 | 值 | 来源 |
|---|---|---|
| 协议头长度 | 9 B | `raw_protocal_data_packet_t` |
| SLOT 头长度 | 8 B | 驱动 SLOT 头（槽号 `[4:0]`） |
| 每像素字节 | 8 B | ECHO1 `[19:0]` + ECHO2 `[39:20]` + 保留 `[63:40]` |
| 每 SLOT 像素数 | 120 | 标定文件 `lines_v` |
| 单 SLOT 长度 | 968 B | 8 + 120 × 8 |
| UDP 数据报长度 | **977 B** | 9 + 968 |
| 每帧 SLOT 数 | 20 | 驱动 DDR 缓冲 |
| 每帧包数 | 20 | 一个 SLOT 一个包 |

## 7. 验证程度与接入自检

### 7.1 已验证

- **板端回归实测**（`LOCAL_TEST=1` 回放，合成灰度数据）：
  ```
  org gray slot size = 968
  get in gray mode
  [UDP-ST] 31559 pkts/5.0s: len977=31559(100.00%) other=0(0.00%)
  ```
  包长分布 100% 为 977，链路（生产者→内存池→消费者→UDP）全程贯通。
- **统一驱动主机模拟**覆盖 `MODE_GRY_SL100` 的 9680 字节 DMA 批次和双缓冲回绕。
- **结构定义取自板端**：协议头为 `raw_protocal_data_packet_t`（`#pragma pack(1)`），SLOT/像素布局取自驱动 `gray_sl100` 通道定义。

### 7.2 尚未逐字节核对的项（建议实机首次接入时确认）

1. SLOT 头内 `[4:0]` 槽号是否与协议头 `slot_number` 一致
2. 实际帧率与 `frame_id` 自增节奏是否与「20 SLOT/帧」一致
3. ECHO1/ECHO2 灰度值的实际量级（0 ~ 1048575 的 20 bit 满量程）

### 7.3 接入自检清单

```
1. 收到包长是否为 977？                          → 否则不是灰度图包
2. 偏移 0 是否为 0xAA？偏移 7 的 data_len 是否为 968？
3. 偏移 3 的 slot_number 是否在 0 ~ 19 内递增？
4. 同一 frame_id 内是否恰好收到 20 个 slot_number(0~19)？
5. SLOT 头 byte0 的低 5 位是否等于 slot_number？
6. 随机取一个像素，echo1 是否在 0 ~ 1048575 内？
```

任一项不符时，请反馈实际的 `hexdump -C` 前 32 字节，便于定位。

## 8. 排错提示

| 现象 | 可能原因 |
|---|---|
| 收到 1578 / 3138 字节包 | 检查生效的 `system:work_mode` 是否仍为 2 |
| 包长 977 但灰度值是错乱的规整数据 | 检查 DMA 模式与 FPGA 输出的灰度格式是否匹配 |
| 包长 977 但灰度值全为 0 | 板端读到空数据（FPGA 未在灰度格式下工作） |
| `slot_number` 不连续或有跳变 | 丢包（UDP 无序/丢包属正常），上位机应按 `slot_number` 重组 |
| 一直收不到数据 | 未与板端握手（握手前板端处于广播模式，不外发数据） |
| 广播包里 `work_mode` 不是 3 | `ys_conf.ini` 的 `system:work_mode` 未配置为 3 |
| 板端日志偶发 `[Gray] slot0 错位` | 驱动 DDR 缓冲的块相位漂移（见 §3 说明），非应用层错误；上位机按 `slot_number` 重组即可 |

## 9. 修订记录

| 版本 | 日期 | 说明 |
|---|---|---|
| v1.0 | 2026-10-12 | 首版；一个 SLOT 一个 977 B UDP 包，20 SLOT/帧，灰度值 ECHO1/ECHO2 各 20 bit |
