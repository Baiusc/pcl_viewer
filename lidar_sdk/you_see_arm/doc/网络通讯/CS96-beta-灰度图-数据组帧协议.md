# CS96-beta 灰度图数据组帧协议（UDP）

> 版本：v1.0
> 日期：2026-10-13
> 适用固件为 CS96-beta（每 SLOT 576 像素），板端配置 `system:work_mode = 3`。
> 本文面向**上位机解析实现**，给出可直接编码的字节级定义

## 1. 概述

板端把驱动 `MODE_GRY` 通道上传的每个 SLOT **原样透传**给上位机，每个 SLOT 拆成两个 UDP 包。

- 传输方式：UDP，单播到上位机数据端口（默认 `50027`）
- 单个 UDP 数据报长度：**固定 1165 B** = 9 B 协议头 + 1156 B 数据区（SLOT 的一半）
- 字节序：小端（ARM64 板端原生小端）
- SLOT 总长：**2312 B** = 8 B 头 + 576 × 4 B 像素，拆成 2 片（`seq_num` 0/1），上位机按片拼接还原
- 帧组织：**1 帧 = 168 个 SLOT**，`slot_number` 从 0 递增到 167，回到 0 时 `frame_id` 自增
- 帧率：由 driver/FPGA 数据产生速率决定（预计约 1 帧/秒或更低），**上位机不得假设固定帧率**，以 `frame_id` 变化作为换帧边界

> 使用 `beta` 构建和统一 AM6204 内核驱动。应用按 `work_mode=3` 选择 `MODE_GRY`，每次读取 2312 字节。FPGA 须输出对应的灰度格式。

## 2. 单个 UDP 数据报格式

```
┌──────────┬───────────┬─────────────┬──────────┬───────────┬────────────────────┐
│  sof     │  frame_id │ slot_number │ seq_num  │ data_len  │  value             │
│  1 B     │  2 B      │  2 B        │  2 B     │  2 B      │  1156 B            │
│  0xAA    │  小端     │  0~167      │  0~1     │  1156     │  半个原始 SLOT     │
└──────────┴───────────┴─────────────┴──────────┴───────────┴────────────────────┘
偏移: 0          1           3           5          7          9                1165
```

### 2.1 协议头（9 B，1 字节对齐，`raw_protocal_data_packet_t`）

| 偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `sof` | 起始字节，固定 `0xAA` |
| 1 | 2 | `frame_id` | 帧号，`0~65535` 循环递增；每 168 个 SLOT 自增一次（`slot_number` 回到 0 时） |
| 3 | 2 | `slot_number` | SLOT 序号 `0~167`，一帧内唯一，用于帧内排序/丢包检测 |
| 5 | 2 | `seq_num` | **片号 `0~1`**：0 = SLOT 前半片，1 = SLOT 后半片 |
| 7 | 2 | `data_len` | 数据区长度，固定 `1156` |

### 2.2 数据区（1156 B，半个原始 SLOT）

- `seq_num = 0` 的包携带 `SLOT[0 .. 1156)`，即 **8 B SLOT 头 + pixel 0~286**（287 个像素，287×4 = 1148 B）
- `seq_num = 1` 的包携带 `SLOT[1156 .. 2312)`，即 **pixel 287~575**（289 个像素，289×4 = 1156 B）

> 分片边界天然落在像素边界上（2312 ÷ 2 = 1156 = 8 + 287×4），上位机也可先拼成 2312 B 的完整 SLOT 再统一解析，不必关心片内像素分布。

## 3. SLOT 结构（2312 B，两片拼接后）

```
偏移 0        2                        8                                    2312
     ├─ 8B 头 ─┼── 4B(pixel0) ─ 4B(pixel1) ─ … ─ 4B(pixel575) ─┤
     │ slot_num │                                                  │
     │ [15:0]   │                                                  │
```

| 相对偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 0 | 2 | `slot_num` | SLOT 序号 `0~167`（与协议头 `slot_number` 同值） |
| 2 | 6 | 保留 | `[63:16]` 保留 |
| 8 | 4 | pixel0 | 第 0 个垂直像素 |
| 8 + i×4 | 4 | pixel i | 第 i 个垂直像素（i = 0~575） |
| 2308 | 4 | pixel575 | 第 575 个垂直像素 |

### 3.1 像素编码（4 B）

每个像素 4 字节，按小端 32 位整数解析：

| 位域 | 字段 | 说明 |
|---|---|---|
| `[21:0]` | ECHO 灰度值 | 22 bit（环境光），0 ~ 4194303 |
| `[31:22]` | 保留 | 无效，忽略 |

提取方式（两种等价）：

```c
uint32_t v = pixel_data;                 /* 小端 32 位读入 */
uint32_t gray = v & 0x3FFFFF;
```

```python
v = int.from_bytes(pkt[off:off+4], 'little')
gray = v & 0x3FFFFF
```

## 4. 帧结构

| 项 | 值 | 说明 |
|---|---|---|
| 每帧 SLOT 数 | 168 | 与驱动 DDR 缓冲一致 |
| 每 SLOT 像素数 | 576 | 硬件固定值（非点云 `lines_v`） |
| 每帧像素数 | 168 × 576 = 96,768 | 即 576 × 168 的图像 |
| 每 SLOT 数据量 | 2312 B | 8 + 576 × 4 |
| 每帧数据量 | 168 × 2312 = 388,416 B | 不含协议头 |
| 每帧 UDP 包数 | **336** | 168 SLOT × 2 片 |
| 帧边界 | `slot_number` 回到 0 | 同时 `frame_id` 自增 |

上位机按 `frame_id` 分组，每组 168 个 SLOT（`slot_number` 0~167），每个 SLOT 由 `seq_num` 0/1 两片拼成，得到 **168 × 576** 的灰度矩阵（行 = SLOT，列 = 垂直像素；实际显示方向由上位机决定）。

## 5. 与 CS96-gamma 灰度图协议的区别（机型识别）

CS96-beta 与 CS96-gamma 的灰度图协议**结构同构但格式不同**，上位机须按机型（而非包长猜测）选择解析策略：

| 项 | CS96-gamma | CS96-beta |
|---|---|---|
| 驱动接收模式 | 4（gray_sl100） | 5（gray） |
| UDP 数据报长度 | **977 B**（9 + 968，单包完整 SLOT） | **1165 B**（9 + 1156，每 SLOT 两片） |
| 每 SLOT 像素数 | 120 | 576 |
| 每像素字节 | 8 B（ECHO1 20bit + ECHO2 20bit） | 4 B（灰度 22bit） |
| 每 SLOT 长度 | 968 B | 2312 B |
| 每帧 SLOT 数 | 20 | 168 |

板端在指令通道（广播端口默认 `55000`）周期性广播，广播包携带：

- `sn`（18 B）：CS96-beta 为 `CS96000B0126020001`
- `work_mode`：**`3` = 灰度图模式**（gamma 与 beta 相同）

判别要点：

- **SN 以 `CS96000B` 开头 + 包长 1165 → CS96-beta 灰度图包**
- 包长维度不与其它链路冲突：gamma 灰度 977、gamma 点云 1578 / 3138、alpha/beta 点云 1266 / 2514、beta 直方图 4105

## 6. 解析示例

### 6.1 Python

```python
import struct

HEAD_LEN  = 9
SLICE_LEN = 1156
SLOT_LEN  = 2312
PIXELS    = 576
SLICES    = 2

def parse_packet(pkt: bytes):
    """解析一个 CS96-beta 灰度图 UDP 包"""
    if len(pkt) != HEAD_LEN + SLICE_LEN:
        raise ValueError(f"非 beta 灰度图包长: {len(pkt)}")
    if pkt[0] != 0xAA:
        raise ValueError("sof 非 0xAA")

    frame_id     = struct.unpack_from('<H', pkt, 1)[0]
    slot_number  = struct.unpack_from('<H', pkt, 3)[0]
    seq_num      = struct.unpack_from('<H', pkt, 5)[0]
    data_len     = struct.unpack_from('<H', pkt, 7)[0]
    assert data_len == SLICE_LEN, "data_len 与包长不一致"

    return {
        'frame_id':    frame_id,
        'slot_number': slot_number,
        'seq_num':     seq_num,
        'slice':       pkt[HEAD_LEN:HEAD_LEN + SLICE_LEN],  # 半个 SLOT
    }

def assemble_slot(slices: dict):
    """两个片拼成一个完整 SLOT(dict[seq] -> bytes)"""
    assert set(slices.keys()) == {0, 1}, "两片未收齐"
    slot = slices[0] + slices[1]
    assert len(slot) == SLOT_LEN

    slot_no_in_payload = struct.unpack_from('<H', slot, 0)[0]  # 应与协议头 slot_number 相同

    gray = [0] * PIXELS
    for i in range(PIXELS):
        off = 8 + i * 4
        v = int.from_bytes(slot[off:off + 4], 'little')
        gray[i] = v & 0x3FFFFF

    return slot_no_in_payload, gray   # 576 个灰度值
```

### 6.2 C

```c
#include <stdint.h>
#include <string.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t  sof;          /* 0xAA */
    uint16_t frame_id;
    uint16_t slot_number;  /* 0~167 */
    uint16_t seq_num;      /* 0~1 片号 */
    uint16_t data_len;     /* 1156 */
    uint8_t  value[1156];  /* 半个 SLOT */
} gray_protocal_data_packet_t;  /* 1165 字节 */

typedef struct {
    uint32_t data;         /* bit[21:0] = 灰度值 */
} gray_pixel_data_t;        /* 4 字节 */

typedef struct {
    uint8_t header[8];     /* [0:2] = slot_num [15:0] */
    uint8_t value[1];      /* 576 × gray_pixel_data_t */
} org_gray_packet_t;        /* 2312 字节 */
#pragma pack(pop)

static inline uint16_t gray_slot_number(const uint8_t *value)
{
    return (uint16_t)((uint16_t)value[1] << 8 | value[0]);
}

static inline uint32_t gray_pixel_value(const gray_pixel_data_t *p)
{
    return (uint32_t)(p->data & 0x3FFFFF);
}

void on_slice(const uint8_t *pkt, int len, uint8_t slot_buf[2312])
{
    if (len != (int)sizeof(gray_protocal_data_packet_t))
        return;                                   /* 非 beta 灰度图包 */

    const gray_protocal_data_packet_t *p = (const gray_protocal_data_packet_t *)pkt;
    if (p->sof != 0xAA || p->data_len != 1156 || p->seq_num > 1)
        return;

    /* 写入对应半边(两片都到达后 slot_buf 即为完整 SLOT) */
    memcpy(slot_buf + p->seq_num * 1156, p->value, 1156);

    if (p->seq_num == 1) {
        uint16_t slot_number = gray_slot_number(slot_buf);  /* 数据区头内的槽号 */
        const gray_pixel_data_t *pixels =
            (const gray_pixel_data_t *)(slot_buf + 8);
        for (int i = 0; i < 576; i++) {
            uint32_t g = gray_pixel_value(&pixels[i]);
            (void)g;
        }
    }
}
```

## 7. 常量速查

| 项 | 值 | 来源 |
|---|---|---|
| 协议头长度 | 9 B | `raw_protocal_data_packet_t` |
| SLOT 头长度 | 8 B | 驱动 SLOT 头（槽号 `[15:0]`） |
| 每像素字节 | 4 B | 灰度值 `[21:0]` + 保留 `[31:22]` |
| 每 SLOT 像素数 | 576 | 硬件固定值 |
| 单 SLOT 长度 | 2312 B | 8 + 576 × 4 |
| 每 SLOT 片数 | 2 | 2312 B 超过以太网 MTU |
| 每片数据长度 | 1156 B | 2312 ÷ 2（片0 = 8B 头 + 287 像素；片1 = 289 像素） |
| UDP 数据报长度 | **1165 B** | 9 + 1156 |
| 每帧 SLOT 数 | 168 | 驱动 DDR 缓冲 |
| 每帧包数 | 336 | 168 × 2 |
| 每帧数据量 | 388,416 B | 168 × 2312（不含协议头） |

## 8. 验证程度与接入自检

### 8.1 已验证

- **板端回归实测**（`LOCAL_TEST=1` 回放，合成灰度数据，2026-10-13）：
  ```
  org gray slot size = 2312, raw gray full size = 1165
  get in gray mode
  [UDP-ST] 1959 pkts/5.0s: len1165=1959(100.00%) other=0(0.00%)
  ```
  上位机侧（Python 参考实现）共收 4562 包，包长 100% 为 1165，零异常；同一 `frame_id` 收到 168 个 SLOT 且两片全部齐全（slot 0~167）；拼接后 SLOT 头内槽号与协议头 `slot_number` 一致，像素灰度值与回放图案逐点吻合、保留位为 0。链路（生产者→内存池→消费者→UDP）全程贯通。
- **统一驱动主机模拟**覆盖 `MODE_GRY` 的 2312 字节 DMA 批次和双缓冲回绕。
- **结构定义取自板端**：协议头为 `raw_protocal_data_packet_t`（`#pragma pack(1)`），SLOT/像素布局取自驱动 `gray` 通道定义（`core/ys_core_def.h` 的 CS96_BETA 分支）。

### 8.2 尚未逐字节核对的项（建议实机首次接入时确认）

1. SLOT 头内 `[15:0]` 槽号是否与协议头 `slot_number` 一致
2. 实际帧率与 `frame_id` 自增节奏是否与「168 SLOT/帧」一致（预计 ≤1 帧/秒）
3. 灰度值的实际量级（0 ~ 4194303 的 22 bit 满量程）

### 8.3 接入自检清单

```
1. 收到包长是否为 1165？                          → 否则不是 beta 灰度图包
2. 偏移 0 是否为 0xAA？偏移 7 的 data_len 是否为 1156？
3. 偏移 5 的 seq_num 是否为 0 或 1？
4. 偏移 3 的 slot_number 是否在 0 ~ 167 内递增？
5. 同一 (frame_id, slot_number) 是否收到 seq_num=0 和 1 两个包？
6. 两片拼接后，byte[0:2] 的槽号是否等于协议头 slot_number？
7. 随机取一个像素，22 bit 灰度值是否在 0 ~ 4194303 内？
```

任一项不符时，请反馈实际的 `hexdump -C` 前 32 字节，便于定位。

## 9. 排错提示

| 现象 | 可能原因 |
|---|---|
| 收到 977 字节包 | 上位机收到的是 CS96-gamma 灰度图包（按 SN 区分机型） |
| 收到 4105 字节包 | 检查生效的 `system:work_mode` 是否仍为 0 |
| 包长 1165 但灰度值是错乱的规整数据 | 检查 DMA 模式与 FPGA 输出的灰度格式是否匹配 |
| 包长 1165 但灰度值全为 0 | 板端读到空数据（FPGA 未在灰度格式下工作） |
| 只有 seq_num=0 或只有 seq_num=1 到达 | 丢包（UDP 无序/丢包属正常），上位机按 (frame_id, slot_number) 分组、两片到齐才成 SLOT |
| `slot_number` 不连续或有跳变 | 丢包；灰度图数据量大，建议上位机 SO_RCVBUF 调大 |
| 一直收不到数据 | 未与板端握手，或板端 `default_state` 不是扫描状态（握手前板端处于广播模式，不外发数据） |
| 广播包里 `work_mode` 不是 3 | `ys_conf.ini` 的 `system:work_mode` 未配置为 3 |
| 板端日志偶发 `[Gray] slot0 错位` | 驱动 DDR 缓冲的块相位漂移，非应用层错误；上位机按 slot_number 重组即可 |

## 10. 修订记录

| 版本 | 日期 | 说明 |
|---|---|---|
| v1.0 | 2026-10-13 | 首版；一个 SLOT 拆两个 1165 B UDP 包（seq_num 0/1），168 SLOT/帧，灰度值 22 bit |
