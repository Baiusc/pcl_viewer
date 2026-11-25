# FL6532 Range LZ4 压缩传输协议 (上位机实现规格)

> 版本: v1.0
> 日期: 2026-09-16
> 适用: MF150-fl6532, 板端 `work_mode=1`(fs_sdk RANGE), 压缩发送
> 面向: 上位机开发者(接收 → 组帧 → 解压 → 逆转置 → 拼图)
> 板端实现参考: `ys_business/producer/range/ys_producer_range_mf150_fl6532.c`
> 参考接收实现: `scripts/range_rx_verify.py`(Python, 已验证通过)

---

## 1. 链路概述

```
板端每帧: 26880 像素 × 278 B = 7,472,640 B 原始数据
  → 字节转置(每 push 独立: N条278B记录 → 278个平面)
  → LZ4 Frame 压缩(level 0, 256KB 块, linked, 带内容校验)
  → 压缩流 ≈ 2.63 MB/帧, 按节点容量分片 ≈ 1748 包/帧
  → UDP 发往 center_ip:center_data_port (默认配置见板端 /etc/ys_conf.ini)
```

| 指标 | 值 |
|---|---|
| UDP 负载 | **固定 1400 B/包**(小于 MTU 1472, 无 IP 分片; 末包不足补 0) |
| 压缩率 | ≈ 2.7~2.9:1(随场景内容波动) |
| 帧率 | 峰值 10.9 fps, 长期 ≈ 8 fps(板端节流每 16 帧丢 4 帧) |
| 峰值带宽 | ≈ 28 MB/s(224 Mbps) |
| 单帧压缩流 | ≈ 2.6~2.8 MB |
| 单帧包数 | ≈ 2018~2020 |

**关键概念**
- 一个 **laser frame**(`frame_id`)= 一个 **LZ4 Frame 压缩流** = 一帧 UDP 包集合
- 每帧固定 **44 个 push**(数据单元): ROI0 32 个 + ROI1 12 个
- 转置是**按 push 独立**做的, 上位机必须按相同边界逆转置

---

## 2. 单包格式

### 2.1 基础头 `mf150_fl6532_range_lz4_hdr_t` (14 B, 1 字节对齐)

| 偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | `sof` | 固定 `0xAA` |
| 1 | 1 | `version` | 协议版本, 当前 `1` |
| 2 | 2 | `frame_id` | 雷达帧号(小端), 节流时跳变 |
| 4 | 2 | `seq` | **帧内包序号**, 每帧从 0 连续递增 |
| 6 | 1 | `frame_flag` | `0`=START, `1`=NORMAL, `2`=END |
| 7 | 1 | `sub_frame_idx` | 子帧相位 0~3(ROI1 用) |
| 8 | 2 | `data_len` | 本包总长, **恒为 1400** |
| 10 | 2 | `payload_len` | **本包有效压缩负载长度**(末包可能小于容量) |
| 12 | 2 | `reserved` | 保留 |

### 2.2 帧元信息 `mf150_fl6532_range_lz4_meta_t` (72 B, 仅 START 包)

紧跟基础头(偏移 14), 描述原始像素流布局, **解压/逆转置/拼图必需**。

| 偏移 | 大小 | 字段 | 典型值 | 说明 |
|---|---|---|---|---|
| 0 | 2 | `pixel_len` | 278 | 1 个像素字节数 |
| 2 | 4 | `total_pixels` | 26880 | 本帧像素总数 |
| 6 | 1 | `push_cnt` | 44 | 本帧 push 数 |
| 7 | 1 | `group_cnt` | 44 | 虚拟 group 总数 |
| 8 | 1 | `roi_cnt` | 2 | ROI 数 |
| 9 | 1 | `max_sub_frame_cnt` | 4 | 最大子帧数 |
| 10 | 1 | `peak_num` | 2 | 峰数 |
| 11 | 1 | `group_row_unit` | 12 | group 行单位(spad 数) |
| 12 | 2 | `max_bin_cnt` | 2048 | 最大 bin 长 |
| 14 | 2 | `reserved` | - | 保留 |
| 16 | 14×4 | `roi[4]` | - | ROI 描述数组(见下) |

**`roi[i]` 结构 (14 B)**:

| 相对偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 0 | 2 | `width` | ROI 宽(列数) |
| 2 | 2 | `height` | ROI 高(行数) |
| 4 | 2 | `push_pixels` | **本 ROI 每个 group 单次 push 的像素数**(逆转置边界) |
| 6 | 1 | `group_cnt` | 本 ROI 的 group 数 |
| 7 | 1 | `group_start` | 本 ROI 首个物理 group 索引 |
| 8 | 1 | `sub_frame_cnt` | 本 ROI 子帧数 |
| 9 | 1 | `pcl_unit` | 点云距离单位编码 |
| 10 | 4 | `range_max` | 量程(cm), float |

**当前配置实测值**:

| ROI | width | height | push_pixels | group_cnt | group_start | sub_frame_cnt | range_max | pcl_unit |
|---|---|---|---|---|---|---|---|---|
| 0 | 192 | 128 | 768 | 32 | 0 | 1 | 1920 | 2 |
| 1 | 192 | 48 | 192 | 12 | 4 | 4 | 7680 | 8 |

### 2.3 包布局(固定总长 1400 B)

```
START 包 (frame_flag=0):
┌────────────┬─────────────────┬──────────────────────┐
│ 基础头 14B │ 元信息 72B       │ 压缩负载 ≤ 1314B      │
└────────────┴─────────────────┴──────────────────────┘
  偏移 0        偏移 14            偏移 86          总长 1400

NORMAL/END 包:
┌────────────┬──────────────────────┐
│ 基础头 14B │ 压缩负载 ≤ 1386B      │
└────────────┴──────────────────────┘
  偏移 0        偏移 14            总长 1400
```

校验: `sof==0xAA && version==1 && data_len==1400`。
提取有效负载: `payload = pkt[头长 : 头长 + payload_len]`(末包尾部为 0 填充, 必须按 `payload_len` 截取)。

---

## 3. 接收与组帧流程

```
收到 UDP 包
  │
  ├─ 校验 sof/version, 不合法丢弃
  │
  ├─ 按 frame_id 维护帧上下文 (packets[seq] = payload)
  │
  ├─ frame_flag == 0 (START):
  │    ├─ 若同 frame_id 已有未完成帧 → 丢弃旧帧(残缺)
  │    ├─ 解析 meta, 新建帧上下文
  │    └─ 存入 payload
  │
  ├─ frame_flag == 1 (NORMAL):
  │    └─ 存入 payload
  │
  └─ frame_flag == 2 (END):
       ├─ 检查 seq 覆盖 [0, max_seq] 无空洞
       │    有空洞 → 整帧丢弃(UDP 丢包, LZ4F 无法部分解压)
       ├─ 按 seq 升序拼接 payload → 完整 LZ4F 压缩流
       ├─ 校验压缩流长度 == 所有 payload_len 之和
       └─ 交给解压步骤
```

**伪代码**:

```python
frames = {}  # frame_id -> {'meta':..., 'pkts': {seq: payload}}

def on_packet(data):
    sof, ver, fid, seq, flag, sub, dlen, plen, _ = unpack('<BBHHBBHHH', data[:14])
    if sof != 0xAA or ver != 1:
        return
    hdr_len = 14 + (72 if flag == 0 else 0)
    payload = data[hdr_len:hdr_len + plen]

    if flag == 0:                       # START: 新帧
        frames[fid] = {'meta': parse_meta(data[14:14+72]), 'pkts': {}}
    fr = frames.get(fid)
    if fr is None:
        return                          # 没有 START 的孤儿包, 丢弃
    fr['pkts'][seq] = payload

    if flag == 2:                       # END: 结算
        del frames[fid]
        expect = max(fr['pkts'].keys()) + 1
        if len(fr['pkts']) != expect:
            return                      # 序号空洞, 整帧丢弃
        comp = b''.join(fr['pkts'][i] for i in range(expect))
        on_complete_frame(fid, fr['meta'], comp)
```

**鲁棒性建议**:
- 帧上下文加超时(如 500ms 未收齐则丢弃), 防止内存泄漏
- 只允许一个"正在接收"的帧(板端串行发送, 帧不会交错); 收到新 START 时直接丢弃旧帧
- 建议接收 socket 缓冲 ≥ 16 MB(单帧突发 ≈ 2.6 MB)

---

## 4. 解压

压缩流是**标准 LZ4 Frame 格式**(LZ4F), 任何 lz4 库均可解压:

- Python: `lz4.frame.decompress(comp_data)`
- C: `LZ4F_decompress()` 循环
- C++: 任意 lz4 封装

解压输出 = **转置流**, 长度应等于 `meta.total_pixels × meta.pixel_len`(校验用)。

> 板端 LZ4F 配置: level 0, 256KB 块, blockLinked, 带内容校验(帧尾 xxhash),
> 解压库会自动校验内容完整性。

---

## 5. 逆转置 (关键步骤)

板端为提升压缩率, 对**每个 push** 独立做了字节转置:

```
原始布局 (count 条记录, 每条 pixel_len 字节):
  [px0: b0 b1 ... b277][px1: b0 b1 ... b277]...[pxN-1]

转置后 (pixel_len 个平面, 每平面 count 字节):
  [所有px的b0][所有px的b1]...[所有px的b277]

公式: trans[j * count + i] = orig[i * pixel_len + j]
       (i = 像素索引, j = 字节位置)
```

**还原公式**: `orig[i * pixel_len + j] = trans[j * count + i]`

**按 push 切分**: 解压流按 push 顺序排列, push 边界由 meta 决定:

```
push 顺序: ROI0 的 group_cnt[0] 个 push (每个 push_pixels[0] 像素)
         + ROI1 的 group_cnt[1] 个 push (每个 push_pixels[1] 像素)
         + ...(依次类推)

当前配置: [32 × (768像素 × 278B)] + [12 × (192像素 × 278B)]
         = 32 × 213,504B + 12 × 53,376B
         = 6,832,128B + 640,512B = 7,472,640B ✓
```

**Python 实现**:

```python
import numpy as np

out = bytearray(len(raw))
off = 0
for r in range(meta['roi_cnt']):
    roi = meta['roi'][r]
    count = roi['push_pixels']
    n = count * meta['pixel_len']
    for g in range(roi['group_cnt']):
        t = np.frombuffer(raw[off:off+n], dtype=np.uint8).reshape(pixel_len, count)
        out[off:off+n] = t.T.tobytes()      # 逆转置
        off += n
# out 即原始像素流: [px0 278B][px1 278B]...
```

**C 实现**:

```c
/* trans[j * count + i] -> orig[i * stride + j] */
for (uint32_t i = 0; i < count; i++)
    for (uint32_t j = 0; j < stride; j++)
        orig[i * stride + j] = trans[j * count + i];
```

> 注意: 每个 push 独立逆转置, 不能把整帧当一个矩阵。

---

## 6. 像素结构 (278 B)

`#pragma pack(1)` 逐字节解析。布局:

```
┌──────────────────────────────┬──────────────────────┬─────────────────────┐
│ cicl_pix_info_t 头 (60 B)    │ 峰0 结果 (19 B)      │ 峰0 echo 波形 (90 B)│
│                              │ 峰1 结果 (19 B)      │ 峰1 echo 波形 (90 B)│
└──────────────────────────────┴──────────────────────┴─────────────────────┘
  偏移 0                         偏移 60                 偏移 98
```

### 6.1 像素头关键字段 (拼图必需)

| 偏移 | 大小 | 字段 | 说明 |
|---|---|---|---|
| 15 | 1 | `sub_frame_idx` | **子帧号(ROI1: 0~3)** |
| 18 | 2 | `pixel_id` | slot 内编号 |
| 56 | 2 | `row` | **物理行坐标**(binning 后) |
| 58 | 2 | `col` | **物理列坐标**(binning 后) |

### 6.2 像素头完整布局 (60 B)

| 偏移 | 大小 | 字段 |
|---|---|---|
| 0 | 2 | `packet_type` |
| 2 | 2 | `frame_index` |
| 4 | 2 | `packet_id` |
| 6 | 2 | `group_index` |
| 8 | 2 | `slot_index` |
| 10 | 4 | `timestamp` (40ns 单位, slot 相对帧同步) |
| 14 | 1 | `ab_frame_sel` |
| 15 | 1 | `sub_frame_idx` |
| 16 | 1 | `piece_roll_mode` |
| 17 | 1 | `tdc_reso_mode` |
| 18 | 2 | `pixel_id` |
| 20 | 4 | `noise_amb` |
| 24 | 2 | `amb_num` |
| 26 | 4 | `det_sum` |
| 30 | 2 | `real_tx_num` |
| 32 | 2 | `lpwr_real_tx_num` |
| 34 | 1 | `en_algo_echo_bin` |
| 35 | 4 | `peak_num` (枚举, 0=1峰, 1=2峰) |
| 39 | 4 | `echo_sample_l[4]` |
| 43 | 4 | `echo_sample_r[4]` |
| 47 | 4 | `binning_type` |
| 51 | 5 | `pixel_decode` |
| 56 | 2 | `row` |
| 58 | 2 | `col` |

### 6.3 每峰算法结果 `algo_result_unpack_t` (19 B × peak_num)

| 相对偏移 | 大小 | 字段 | 换算 |
|---|---|---|---|
| 0 | 2 | `th_peak_addr` | 峰地址(12bit) |
| 2 | 1 | `crosstalk` | 串扰标记 |
| 3 | 4 | `dist_rp` | 上升沿距离, **cm = 值/256.0** |
| 7 | 2 | `fwhm` | 峰宽(bin) |
| 9 | 4 | `conv_intensity` | 卷积峰高 |
| 13 | 4 | `dist_cent` | 质心距离, **cm = 值/256.0** |
| 17 | 2 | `hist_peak_max` | 原始峰值 |

### 6.4 echo 原始波形

紧跟在峰结果之后, 每峰 `(echo_sample_l[j] + echo_sample_r[j] + 1) × 2 B` (uint16 小端)。
`echo_sample_l/r` 从像素头读取(板端已截断到 ≤22, 实际 22+22+1=45 个 bin)。

---

## 7. 拼图与子帧规则

### 7.1 空间布局

```
       列 0                                   列 191
        ┌────────────────────────────────────────┐
 行 0   │                                        │
        │           ROI0 (192×128)               │
 行 15  │      ┌──────────────────────────┐      │
        │      │   ROI1 (192×48)           │      │
 行 16  │      │   长距增强区, 行 16~63     │      │
 行 63  │      └──────────────────────────┘      │
        │                                        │
 行 127 └────────────────────────────────────────┘
```

- ROI0 和 ROI1 的 `(row, col)` 是**物理坐标**, 直接铺图
- 行 16~63 是重叠区, 同一位置有两份测量(靠 `roi_index` 区分)
- ROI0 量程 1920cm, ROI1 量程 7680cm; 重叠区处理策略由上位机算法决定(如优先 ROI1)

### 7.2 子帧规则(重要)

**`sub_frame_idx` 是跨帧递增的**: 帧 N → 0, N+1 → 1, N+2 → 2, N+3 → 3, N+4 → 0...

| ROI | sub_frame_cnt | 每帧数据量 | 拼图方式 |
|---|---|---|---|
| ROI0 | 1 | 完整(每帧 24576 像素) | 每帧独立完整, 直接铺图 |
| ROI1 | 4 | **1/4**(每帧 2304 像素) | 需连续 4 帧(sub 0~3)聚合 |

ROI1 的 4 个子帧是**空间互补**的(已实测: 4 帧像素并集 = 768/768, 无重叠):

```
帧1(sub=0) 帧2(sub=1) 帧3(sub=2) 帧4(sub=3)
  └──────────┴──────────┴──────────┘
     拼成完整 ROI1 (192×48)
```

**建议**: 对每个 ROI1 像素按 `(row, col)` 覆盖写入一张 192×48 的图; 收到 4 个不同 sub_frame_idx 后即完整。
帧首包的 `sub_frame_idx` 字段标明当前帧相位, 也可从任意像素头偏移 15 读取(两者一致)。

### 7.3 多帧时序

```
frm=N    sub=0  → ROI0 完整 + ROI1 的 1/4
frm=N+1  sub=1  → ROI0 完整 + ROI1 的 1/4
frm=N+2  sub=2  → ROI0 完整 + ROI1 的 1/4
frm=N+3  sub=3  → ROI0 完整 + ROI1 的 1/4(此时 ROI1 拼齐)
```

板端节流以 4 帧整周期为单位丢弃, 因此收到的 sub 序列永远是完整的 `0,1,2,3` 循环。

---

## 8. 边界情况与处理

| 现象 | 原因 | 处理 |
|---|---|---|
| `frame_id` 跳变(如 8→13) | 板端节流整帧丢弃(正常) | 忽略, 继续等下一帧 |
| 帧只有 ~40 包就收到 END | 节流切换发生在帧中途 | seq 空洞 → **整帧丢弃** |
| seq 有空洞 | UDP 丢包 | 整帧丢弃(LZ4F 无法部分解压) |
| 没有 START 的包 | START 包丢失 | 丢弃孤儿包 |
| 新 START 到达时旧帧未完成 | 上一帧残缺 | 丢弃旧帧, 开新帧 |
| 解压失败 | 压缩流损坏 | 丢弃该帧, 记录日志 |

> 实测数据: 完整帧 ≈ 2018~2020 包 / ≈2.6~2.8 MB 压缩流 / 7,472,640 B 解压数据。
> 可用这三个数字做完整性自检。

---

## 9. 参考实现

`scripts/range_rx_verify.py` 是本协议的完整 Python 参考实现(已在板端实测验证), 包含:
- UDP 收包与缓冲设置
- 帧上下文管理与完整性检查
- LZ4F 解压
- 按 push 逆转置(numpy)
- 像素 (row, col) 解析与校验

可直接作为上位机的实现基础。运行方式:

```bash
pip install lz4 numpy
python3 range_rx_verify.py [监听秒数]
```

输出示例(实测):

```
[frm=1] OK pkts=1745 sub=0 comp=2629908 raw=7472640 ratio=2.84:1 pixel_len=278
    roi0: 32x768px rows 0..127 cols 0..191 (192x128)
    roi1: 12x192px rows 16..63 cols 0..191 (192x48)
```

---

## 10. 快速自检清单

- [ ] UDP 收到包, `sof=0xAA, version=1, data_len=1400`(所有包长度统一)
- [ ] START 包解析出 `pixel_len=278, push_cnt=44, roi_cnt=2`
- [ ] 每帧收包数 ≈ 2018~2020, seq 无空洞
- [ ] 解压输出 7,472,640 B
- [ ] 逆转置后 ROI0 像素行 0..127、列 0..191; ROI1 行 16..63
- [ ] ROI1 连续 4 帧(sub 0~3)的像素并集 = 9216 个位置
- [ ] 单帧压缩流 ≈ 2.6~2.8 MB(压缩率 ≈ 2.7~2.9:1)
