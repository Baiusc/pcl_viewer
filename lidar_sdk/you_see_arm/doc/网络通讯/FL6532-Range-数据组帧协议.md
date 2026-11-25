# FL6532 Range 模式数据组帧协议

> 版本: v1.0
> 日期: 2026-09-03
> 适用: MF150-fl6532 机型, 板端 `system:work_mode = 1`(fs_sdk RANGE 模式, 数据 `data_type=0x2E`)
> 面向对象: 上位机软件开发者(按本文档实现 UDP 收包 → 组帧 → 逐像素解析)

---

## 1. 传输概要

| 项 | 值 | 说明 |
|---|---|---|
| 传输 | UDP | 无连接, 不保证有序 |
| 目标地址 | 板端配置文件 `network:center_ip` / `center_data_port` | 默认 `192.168.110.15:50027`, 可在运行时被握手指令改写 |
| UDP 负载上限 | 1472 B | `FS_PROTOCOL_UDP_MTU_SIZE` |
| 帧头 | 每包固定 `6 + 34 = 40 B` 头 | 见 §2 |
| 数据性质 | 芯片 Range 原始测距结果 | 未经过板端点云算法, 由上位机做二次算法 |
| 帧率/丢帧 | 板端 fs_sdk 内置帧级节流(约 3/4), 帧号会跳变 | 见 §4 |

板上每收到一帧都会发一个"完整帧", 帧之间用 **START / END 标记 + 帧号 + 序号** 界定, 包内携带原始像素字节。上位机无需关心"一帧由哪些 ROI / 组构成", 只需把 START 到 END 之间所有包按序号拼成一段连续字节流, 再按 §5 逐像素解析即可。

---

## 2. 单包(UDP 负载)格式

```
┌──────────────────────────┬──────────────────────────┬─────────────────────────┐
│  fs_protocol_head_t      │  fs_lidar_pcl_echo_data_t │  pixel 数据             │
│  固定 6 B                │  固定 34 B                │  dot_num × pixel_len B  │
└──────────────────────────┴──────────────────────────┴─────────────────────────┘
```

### 2.1 协议头 `fs_protocol_head_t` (6 B, 1 字节对齐)

| 偏移 | 大小 | 字段 | 值/说明 |
|---|---|---|---|
| 0 | 2 | `head` | 固定 `0xA55A`(小端) |
| 2 | 1 | `protocol_ver_major` | `2` |
| 3 | 1 | `data_type` | **`0x2E`**(DATA_TYPE_CICL_DATA, Range 原始数据) |
| 4 | 1 | `lidar_module` | 0 |
| 5 | 1 | `reserved` | 0 |

校验: `head==0xA55A && protocol_ver_major<=2 && data_type==0x2E`, 不满足即非法包。

### 2.2 数据头 `fs_lidar_pcl_echo_data_t` (34 B, 1 字节对齐)

| 偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `protocol_ver_minor` | 2 |
| 1 | 2 | `data_sequence` | **帧内包序号, 每帧从 0 连续递增**, 用于丢包检测/排序 |
| 3 | 2 | `frame_cnt` | 雷达帧号(与 data_sequence 无关, 帧号可能跳变) |
| 5 | 2 | `dot_num` | 本包携带的像素个数 |
| 7 | 1 | `type` | `0x05`(FS_CICL_DATA) |
| 8 | 1 | `peak_type_2_1` | bit[3:0] 第 1 回波类型; bit[7:4] 第 2 回波类型(见 §6.4) |
| 9 | 1 | `frame_flag` | **0x00=帧首(START), 0x01=中间, 0x02=帧尾(END)** |
| 10 | 2 | `width` | 本包像素所属 ROI 的宽(列数) |
| 12 | 2 | `height` | 本包像素所属 ROI 的高(行数) |
| 14 | 2 | `max_bin_cnt` | 帧内最大 bin 长 |
| 16 | 1 | `ab_frame_idx` | bit0: A/B 帧标记; bit[7:1]: `pcl_unit` 编码 |
| 17 | 1 | `max_sub_frame_cnt` | bit[3:0]: 最大子帧数(本帧所有组中), 当前为 4 |
| 18 | 1 | `roi_index` | 本包数据所属 ROI 0~3 |
| 19 | 2 | `pixel_len` | **1 个像素的字节数(解析像素的关键, 当前实测 278)** |
| 21 | 2 | `data_offset` | Range 通路恒 0 |
| 23 | 2 | `data_len` | Range 通路恒 0 |
| 25 | 1 | `peak_type_4_3` | bit[3:0] 第 3 回波; bit[7:4] 第 4 回波(Range 模式为 0) |
| 26 | 8 | `timestamp_base` | **帧基准时间戳(us)。注意: Range 模式下板端恒填 0** |

> 注意 `ab_frame_idx` 里压缩了 `pcl_unit` 与 A/B 帧位; `pcl_unit` 表示该 ROI 距离数值的单位, 需要时按
> `pcl_unit = (ab_frame_idx >> 1) & 0x7F` 解出后, 用以下换算还原为 cm:
> 编码: `bit[6:5]` 整数部分, `bit[4:0]` 小数部分(单位 cm), 即 `unit = 整数 + 小数×0.03125 cm`。
> 对当前 ROI01-80 配置: ROI0 的 `range_max≈1920cm`, ROI1 的 `range_max≈7680cm`(仅作量程参考, 具体以 `pcl_unit` 解出为准)。

### 2.3 像素数据段

每个包的数据段 = `dot_num × pixel_len` 字节的连续像素。**像素内部没有包边界概念**, 分片边界只可能落在像素与像素之间(板端按整像素切包), 不会把一个像素切进两个包。

满包大小: `40 + 5×278 = 1430 B`(实测满包即 1430); 最后一个残包可能小于该值(实测 `dot=2 len=596`)。

### 2.4 包长分布(当前预设的实测规律)

板端按每包最多 5 个像素分片(`max_dot_num = 1432 / pixel_len`), 因此一个 UDP 包的长度只取决于 `dot_num`:

| 包长(B) | dot_num | 出现场合 |
|---|---|---|
| 1430 | 5 | 满包 |
| 874 | 3 | ROI-0 单元残包: 单次 push 768 像素, `768 % 5 = 3` |
| 596 | 2 | ROI-1 单元残包: 单次 push 192 像素, `192 % 5 = 2` |

一帧内三种包的数量是确定的(可作为帧完整性自检依据):

| 包长(B) | 每帧数量 | 来源 |
|---|---|---|
| 1430 | 21408 | 128×(768/5) + 48×(192/5) 的满包部分 |
| 874 | 128 | ROI-0: 32 组 × 4 个滚动 phase |
| 596 | 48 | ROI-1: 12 组 × 4 个子帧 |

合计 21584 包/帧。注意: 这些数量与 `pixel_len`、ROI 配置强相关, **上位机不应硬编码**, 应以 `echo.dot_num × echo.pixel_len + 40 == UDP长度` 做一致性校验。

---

## 3. 帧与组帧规则(上位机核心逻辑)

### 3.1 帧边界标记

- **帧首 START**(`frame_flag=0x00`): 每帧第 1 个包。
- **帧尾 END**(`frame_flag=0x02`): 每帧最后 1 个包。
- 其余为 NORMAL(`0x01`)。
- START 优先级高于 END(极端单包帧只标 START)。

### 3.2 序号与完整性

- `data_sequence` 每帧从 0 起连续递增到帧尾(一帧可达 2 万包以上)。
- `frame_cnt` 为雷达帧号。**板端 fs_sdk 会按约 3/4 比例整帧丢弃(帧级节流), 帧号会跳变, 属正常现象**, 上位机不要假设帧号连续。

### 3.3 重组流程(伪代码)

```c
typedef struct {
    bool      in_frame;        // 是否正在收帧
    uint16_t  frame_cnt;       // 当前帧号
    uint32_t  seq_expected;    // 期望的下一个 data_sequence
    bool      got_end;         // 是否已收到 END
    uint64_t  bytes;           // 已缓冲字节数
    uint8_t  *buf;             // 重组缓冲(按需扩容)
    uint32_t  buf_cap;
    uint8_t   roi_index;       // 记录帧内出现过的 ROI 集合(按位)
    uint16_t  roi_width[4], roi_height[4];  // 各 ROI 尺寸(从包头记录)
    uint16_t  pixel_len;
} range_frame_ctx_t;

void on_udp_packet(uint8_t *pkt, int len) {
    fs_protocol_head_t *head  = (fs_protocol_head_t *)pkt;
    fs_lidar_pcl_echo_data_t *echo = (fs_lidar_pcl_echo_data_t *)(pkt + 6);

    /* 0. 校验(§2.1) */
    if (head->head != 0xA55A || head->data_type != 0x2E) return;

    int hdr = 6 + 34;
    if (len < hdr) return;
    uint16_t payload = echo->dot_num * echo->pixel_len;   // = len - hdr

    /* 1. 帧首: 丢弃不完整旧帧, 开新帧 */
    if (echo->frame_flag == 0x00) {
        if (ctx.in_frame) discard_frame(ctx);             // 上一帧未收完(丢包), 放弃
        reset_frame(ctx);
        ctx.in_frame   = true;
        ctx.frame_cnt  = echo->frame_cnt;
        ctx.seq_expected = 0;
        ctx.pixel_len  = echo->pixel_len;
        append(ctx, echo->data, payload);
        return;
    }

    /* 2. 帧尾 */
    if (echo->frame_flag == 0x02) {
        if (!ctx.in_frame || echo->frame_cnt != ctx.frame_cnt) { return; }  // 异常包丢弃
        if (echo->data_sequence != ctx.seq_expected) { discard_frame(ctx); return; } // 中间丢包
        append(ctx, echo->data, payload);
        ctx.in_frame = false;
        on_complete_frame(ctx);                            // 完整一帧交给解析/算法层
        return;
    }

    /* 3. 中间包 */
    if (!ctx.in_frame || echo->frame_cnt != ctx.frame_cnt) return;
    if (echo->data_sequence != ctx.seq_expected) {
        /* 序号空洞 = 网络丢包, 该帧已不完整, 直接丢弃等下一帧 */
        discard_frame(ctx);
        return;
    }
    append(ctx, echo->data, payload);
}
```

重组完成后, `ctx.buf` 中即是按像素顺序排列的整帧字节流, 交给 §5 的解析器。

> 简化提示: 若网络环境可控(同网段有线), 可只依赖 `frame_flag` START/END 组帧, 用 `data_sequence` 校验完整性; 若要容忍乱序, 可按 `data_sequence` 索引把每包写进预留槽位而不是追加。

---

## 4. 帧内容示例(板端真实日志)

```
INFO [FL6532-RANGE] pkt: frm=9168 seq=21583 flag=2 dot=2  len=596     ← 帧尾残包
INFO [FL6532-RANGE] frame stat: frm=9168 pkt_total=21584 dot_total=107520
INFO [FL6532-RANGE] pkt: frm=9173 seq=0     flag=0 dot=5 len=1430     ← 下一帧帧首(注意帧号已跳变)
```

- `frm=9168 → 9173` 跳变: 板端节流丢帧, 正常。
- `dot_total=107520` = 一帧像素总数, `pkt_total≈21584`(每包 5 像素为主, 末尾残包)。
- 单帧约 `21584 × 1430B ≈ 30MB`(此值为当前预设/实测参考, 上位机重组缓冲建议 ≥ 40MB 或按实际帧率核算)。

---

## 5. 像素(pixel)数据结构

### 5.1 总览

一个像素 = `pixel_len` 字节(`echo.pixel_len`, 当前实测 **278**, 由板端 ROI 配置决定, **不要硬编码**)。布局:

```
┌───────────────────────┬──────────────────────┬───────────────────────┐
│ cicl_pix_info_t 头    │ algo_result_unpack_t │ echo bin 区(可选)     │
│ 60 B                  │ 每峰 19 B(当前 2 峰) │ 每峰 (L+R+1)×2 B      │
└───────────────────────┴──────────────────────┴───────────────────────┘
```

**所有字段 1 字节对齐(`#pragma pack(1)`)**, 解析器必须按字节偏移取字段。

### 5.2 像素头 `cicl_pix_info_t`(60 B)

| 偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 0 | 18 | `pkt_info` | 包信息, 见 5.3 |
| 18 | 2 | `pixel_id` | 本像素在 slot 内的编号(从 0) |
| 20 | 4 | `noise_amb` | 寻峰噪声阈值 |
| 24 | 2 | `amb_num` | 环境光总计数 |
| 26 | 4 | `det_sum` | 直方图总计数 |
| 30 | 2 | `real_tx_num` | 实际 TX 次数 |
| 32 | 2 | `lpwr_real_tx_num` | 低功率实际 TX 次数(未启用高低功率时为 0) |
| 34 | 1 | `en_algo_echo_bin` | 是否携带 echo bin(true=1) |
| 35 | 4 | `peak_num` | **枚举(4B)**。`RX_PEAK_NUM_1=0` 表示 1 峰, `RX_PEAK_NUM_2=1` 表示 2 峰; 当前固件=2 峰 |
| 39 | 4 | `echo_sample_l[4]` | 峰 0~3 左采样 bin 数 |
| 43 | 4 | `echo_sample_r[4]` | 峰 0~3 右采样 bin 数 |
| 47 | 4 | `binning_type` | 枚举(4B)。`RX_BINNING_TYPE_3X3=0`(当前), 2X6=1, 2X2=2, 6X6=3 |
| 51 | 5 | `pixel_decode` | 像素通道分解信息(一般无需用, `row/col` 已解好) |
| 56 | 2 | `row` | **物理行坐标(binning 后), 已由板端解码, 直接用于铺图** |
| 58 | 2 | `col` | **物理列坐标(binning 后)** |

`echo_sample_l[j]` / `echo_sample_r[j]` 是峰 j 实际回波采样的左右 bin 数(≤22), 用于定位 §5.4 的 echo bin 区域长度。

### 5.3 像素内 `pkt_info`(18 B)

| 偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 0 | 2 | `packet_type` | 包类型 |
| 2 | 2 | `frame_index` | 雷达帧号(从 1 起, 与本包 echo.frame_cnt 一致) |
| 4 | 2 | `packet_id` | MIPI 帧内包编号(从 1) |
| 6 | 2 | `group_index` | 组索引 |
| 8 | 2 | `slot_index` | slot 索引 |
| 10 | 4 | `timestamp` | 时间戳: 本 slot 首次 TX 距帧同步的时间, 单位 **40ns** |
| 14 | 1 | `ab_frame_sel` | A/B 帧标记 |
| 15 | 1 | `sub_frame_idx` | **子帧编号** |
| 16 | 1 | `piece_roll_mode` | 子帧模式: 最大子帧数 = 2^n(n=0/1/2) |
| 17 | 1 | `tdc_reso_mode` | TDC 分辨率模式 |

> Range 模式下板端不给 `echo.timestamp_base` 填值(恒 0), 像素内的 `timestamp`(40ns)也只是 slot 相对帧同步的偏移, 没有绝对基准。若上位机需要多帧时间对齐, 建议以本机收到帧首(或 CSI 帧)的时间自行打戳。

### 5.4 每峰测距结果 `algo_result_unpack_t`(19 B × peak 数)

紧跟 60 B 头之后, 峰 0、峰 1…… 顺序排列(共 `peak_num 实际值+1` 个; 当前 2 峰 = 38 B)。

| 偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 0 | 2 | `th_peak_addr` | 寻峰峰值 bin 地址(12 bit) |
| 2 | 1 | `crosstalk` | 是否串扰峰 |
| 3 | 4 | `dist_rp` | **上升沿距离**: 16bit 整数 + 8bit 小数的定点数, **cm = 值 / 256** |
| 7 | 2 | `fwhm` | 峰宽(单位 bin) |
| 9 | 4 | `conv_intensity` | 卷积峰高(强度) |
| 13 | 4 | `dist_cent` | **质心距离**: 同上定点, cm = 值 / 256 |
| 17 | 2 | `hist_peak_max` | 信号峰原始 bin 最大值(信号峰高, 12 bit) |

物理换算参考:
- 距离: `dist(cm) = 原始值 / 256.0`; 例如 `dist_cent=115200 → 450.0cm`。
- bin→距离: 当前 ROI01-80 配置 `bin_cnt_ns=4ns`, 每 bin ≈ 3.75 cm(与 `range_max = max_bin_cnt × 15 / bin_cnt_ns` 自洽)。
- 峰上"强/次强"等类型见 §6.4。

### 5.5 echo bin 区(直方图)

峰结果之后是每峰的 echo 采样(仅当 `en_algo_echo_bin==true`, 当前为 true):

```
对每个峰 j(0..peak):
    bin 个数 n_j = echo_sample_l[j] + echo_sample_r[j] + 1
    数据 n_j × uint16_t(小端, 原始 bin 计数)
```

板端组包时每个峰的左右采样 bin 数会被 **截断到 22**(`echo_cfg.bin_cnt_left/right=28` 时实际只发 22)。`echo_sample_l/r` 字段即实际截断后的值, 解析必须用字段值而不是预设值。

### 5.6 解析伪代码

```c
/* pbuf 指向帧缓冲, frame_len 为其字节数 */
int parse_frame(uint8_t *pbuf, int frame_len, uint16_t pixel_len) {
    int off = 0;
    while (off + pixel_len <= frame_len) {
        cicl_pix_info_t  *px = (cicl_pix_info_t *)(pbuf + off);   // 60 B 头
        int peak_cnt = (int)px->peak_num + 1;                     // RX_PEAK_NUM_1=0 -> 1 峰

        /* 逐峰解析(定长段: 每峰 19 B, 总在 echo bin 之前) */
        int pos = off + sizeof(cicl_pix_info_t);                  // 60
        for (int j = 0; j < peak_cnt; j++) {
            algo_result_unpack_t *r = (algo_result_unpack_t *)(pbuf + pos);
            float dist_rp   = r->dist_rp   / 256.0f;              // cm
            float dist_cent = r->dist_cent / 256.0f;              // cm
            pos += sizeof(algo_result_unpack_t);
        }

        /* echo bin 区(可选): 每峰实际长度取像素头里的 echo_sample */
        if (px->en_algo_echo_bin) {
            for (int j = 0; j < peak_cnt; j++) {
                int n = px->echo_sample_l[j] + px->echo_sample_r[j] + 1;
                uint16_t *bin = (uint16_t *)(pbuf + pos);         // n 个 bin 计数
                pos += n * 2;
            }
        }

        /* (row, col) 已在像素头内, 可直接用于铺点 */
        on_pixel(px->row, px->col, peak_cnt, /* 峰结果/echo 数据 */...);

        off += pixel_len;                                          // 固定步长!
    }
    return off;   /* 应等于 frame_len */
}
```

> **重要**: 帧内像素按固定 `pixel_len` 步长遍历。像素头里 `peak_num`/`echo_sample_*` 是"实际有效量",
> 单峰像素的"第 2 峰区域"与未写满的 echo 尾区是陈旧数据, 解析时不要越过 `peak_num`/`echo_sample` 指示的长度即可。

---

## 6. 数据语义与后续重建建议

### 6.1 像素坐标与多 ROI

- 像素头的 `row/col` 是板端 SDK 从像素通道/组/子帧解码出的**物理坐标**(binning 后), 上位机可直接以 `(row, col)` 为图坐标。
- 一帧内不同包可能属于不同 ROI(`echo.roi_index` 0~3)。当前板端预设为双 ROI(ROI-0: 宽视野 80°/近中距; ROI-1: 中央 30°/长距 4 子帧), ROI-1 是嵌套在 ROI-0 中央的**长距增强区**, 与 ROI-0 存在**空间重叠**: 重叠位置会出现分别属于两个 ROI 的两次测距(ROI-0 一次 + ROI-1 一次), 由 `roi_index` 区分。
- 同一 ROI 的网格尺寸 = 该 ROI 包中 `echo.width × echo.height`(每包都带)。
- 板端 Range 模式**不做**遮挡合并/点云算法(PCL 通路才做), 重叠区如何处理属于上位机算法的决策空间(例如: 优先取 ROI-1 的长距结果)。

### 6.2 子帧(sub_frame)

- ROI-1 每组数据会按 4 个子帧(4 次累积曝光)分次发出; 同一组同一像素在不同 `sub_frame_idx` 下可能有多次测距。像素头 `pkt_info.sub_frame_idx` 标明它属于第几个子帧。
- 对"每个像素只保留一份数据"的应用, 上位机可按 `(roi, row, col)` 覆盖或按需聚合(如取 4 子帧的统计/平均)。

### 6.3 时间

- `echo.timestamp_base` 恒 0; 像素内 `pkt_info.timestamp` 为 40ns 步进、相对帧同步的偏移。
- 帧到达率受板端节流与网络带宽影响不恒定, 上位机建议自行打接收时间戳。

### 6.4 回波类型枚举(`peak_type_2_1`, 来自包头的诊断信息)

`0x0` 无回波, `0x1` 最强, `0x2` 次强, `0x3/0x4` 第 3/4 强, `0x5~0x8` 最近~第 4 近, `0x9~0xC` 最远~第 4 远。Range 像素内的峰顺序按配置为"第 1 峰/第 2 峰", 对应关系以上位机实测为准。

---

## 7. 建议

1. 结构体定义建议直接复用板端 SDK 的头文件(保证布局一致):
   - `third_party/fs_sdk/sdk/net_server/inc/fs_net_protocol.h`(`fs_protocol_head_t`、`fs_lidar_pcl_echo_data_t`、帧标记/类型枚举)
   - `third_party/fs_sdk/sdk/algo/inc/algo_cicl.h`(`cicl_pix_info_t`、`pkt_info_unpack_t`)
   - `third_party/fs_sdk/sdk/algo/inc/algo.h`(`algo_result_unpack_t`、`pixel_decode_t`、`rx_peak_num_e`)
   - 结构体均为 `#pragma pack(1)`, 请保持一致后再用 `sizeof`。
2. 重组缓冲与内存: 一帧可达约 2.2 万包 / 约 30MB(当前预设实测), 建议按两帧缓冲规划内存与带宽预算。
3. 首包自检: 收到帧首后可用 `echo.pixel_len` 是否在合理范围(几十~几百)做数据合法性检查; `echo.dot_num*pixel_len+40` 应与 UDP 长度一致。
4. 已知注意点: 帧号跳变正常(板端节流); 序号空洞 = 网络丢包, 重组侧应丢帧等待下一帧首。
