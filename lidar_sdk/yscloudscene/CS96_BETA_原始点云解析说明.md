# CS96-Beta 雷达原始点云解析说明

本文档面向基于 CS96-Beta 雷达原始点云数据做应用的开发者，说明如何从 PC 端收到的 UDP 数据包中解析出完整的原始点云帧。

## 1. 代码库位置与分支

- 代码库地址：`git@10.42.0.10:ys_window/yscloudscene.git`
- 分支：`develop/mf150_raw_point_cloud`
- 关键代码文件：
  - `business/origin_point_parser.cpp` — UDP 接收、拆包、组帧主流程
  - `business/origin_point_parser.h` — 相关类声明
  - `business/cs96_raw_save.cpp` — 原始帧解析落盘示例（`SaveCS96RawFrameSync`）
  - `you_see_sdk/include/ys_lidar_validate_type.h` — 常量与结构体定义（`CS96BetaRawParser` 命名空间、`OriginPointData` 等）

> 注：CS96-Beta 原始点云的字节级定义以《原始点云协议PL-PS_v2.2.docx》为准，本代码库中的实现与该文档保持一致。

## 2. 总体数据流

```
UDP接收线程 (OriginPointReceive2)
    │  收到 1561B/包 的 UDP 数据（被 PC 与 ARM 通信协议包装）
    ▼
拆包：按 1561B 对齐切出 OriginPointData 单元（9B 头 + 1552B 数据）
    │  data_length == 1552，为 CS96-Beta 格式
    ▼
缓存：按 frame_id 组装 800 个 slot → 完整帧 1,241,600B
    │  完整帧进入优先队列 complete_origin_frames_
    ▼
解析线程 (OriginPointParse2，Start() 中启动)
    │  按 frame_id 顺序取出完整帧，交给下游应用处理
    ▼
完整原始点云帧（1,241,600B）
```

## 3. UDP 包格式（1561 Byte）

PC 端收到的每个 UDP 包大小为 **1561 字节**。该包是 PC 与 ARM 之间通信协议包装后的数据，需要先解析出真正的原始点云数据。

每个 1561B 的包内部结构为：

| 偏移 | 长度 | 内容 |
| ---- | ---- | ---- |
| 0 | 9 | `OriginPointData` 头 |
| 9 | 1552 | slot 原始点云数据（16B slot 头 + 96 像素 × 16B） |

接收函数 `OriginPointParser::OriginPointReceive2` 通过包长整除判断格式（`business/origin_point_parser.cpp:79`）：

```cpp
if (length % CS96BetaRawParser::udp_packet_size == 0) {  // 1561
    cs96beta_detected_ = true;
    slot_size = CS96BetaRawParser::udp_packet_size;
}
```

一个 1561B 的 UDP 包恰好对应 **一个 slot**。

## 4. OriginPointData 头（9 字节）

每个 1561B 包以 `OriginPointData` 结构开头（`ys_lidar_validate_type.h:255`，小端字节序）：

```cpp
using OriginPointData = struct {
    uint8_t header;        // SOF 帧头
    uint16_t frame_id;     // 上传的帧 ID
    uint16_t slot_number;  // 当前帧的当前 slot 号 (0~799)
    uint16_t fragment;     // 分片序号
    uint16_t data_length;  // 原始点云数据长度 = 1552
    uint8_t data[1];       // 原始点云数据（本包从偏移 9 开始，长度为 data_length）
};
```

拆包示意（`business/origin_point_parser.cpp:91`）：

```cpp
OriginPointData* origin_point = reinterpret_cast<OriginPointData*>(src_local + i * slot_size);
// 有效载荷: origin_point->data，长度 origin_point->data_length（CS96-Beta 下为 1552）
```

## 5. Slot 数据结构（1552 字节）

`origin_point->data` 指向的 1552B 为一个 slot 的原始数据：

| 偏移 | 长度 | 内容 |
| ---- | ---- | ---- |
| 0 | 4 | word0：含 mirror_angle（20bit，`(w0 >> 12) & 0xFFFFF`） |
| 4 | 4 | word1：含 slot_number（低 12bit，`w1 & 0x0FFF`） |
| 8 | 8 | reserved 保留 |
| 16 | 1536 | 96 个像素 × 16B/像素 |

> 像素**倒序**存放：数据区第 1 个像素是 pixel[95]，最后 1 个是 pixel[0]。

## 6. 像素格式（16 字节，两重回波）

每个像素 16 字节，包含两重回波，每回波 8 字节：

```
偏移  长度  字段
0     3    echo0 前沿时间值 (单位: ps, 小端)
3     3    echo0 后沿时间值 (单位: ps, 小端)
6     2    echo0 峰值       (单位: bin, 最大值 1638, 小端)
8     3    echo1 前沿时间值 (单位: ps, 小端)
11    3    echo1 后沿时间值 (单位: ps, 小端)
14    2    echo1 峰值       (单位: bin, 最大值 1638, 小端)
```

即每回波 8B = 前沿时间(3B) + 后沿时间(3B) + 峰值(2B)，两回波共 16B。

解析参考（`business/cs96_raw_save.cpp:86`）：

```cpp
// 第1重回波: 前沿时间(3B) + 后沿时间(3B) + 峰值(2B)
uint32_t front0  = pixel_data[0] | (pixel_data[1] << 8) | (pixel_data[2] << 16);
uint32_t trail0  = pixel_data[3] | (pixel_data[4] << 8) | (pixel_data[5] << 16);
uint16_t peak0   = pixel_data[6] | (pixel_data[7] << 8);
// 第2重回波位于偏移 8~15，格式相同
```

**注意**：
- 当前 CS96-Beta 原始点云只发送**两重回波**，第三回波是**假数据**（前沿时间/后沿时间/峰值均填 `16`，见 `cs96_raw_save.cpp:124`）。
- 无效点：前沿时间值 `== 0` 或 `== 16`（`kFpgaInvalidPixelValue`）视为无效（`origin_point_parser_imp.cpp:1189`）。
- 两回波均有有效数据时，可取峰值更大的回波作为该像素的测量（`origin_point_parser_imp.cpp:1181`）。

## 7. 完整帧组装

- 每帧 **800 个 slot**（`slot_counts = 800`）；
- 单帧原始数据大小：`800 × 1552 = 1,241,600 字节`（`frame_data_size`）；
- 接收端以 `frame_id` 为 key 缓存不完整帧，收到 800 个 slot 后判定帧完整（`origin_point_parser.cpp:118`），进入有序队列等待解析线程处理；
- 解析线程 `OriginPointParse2` 按 frame_id 顺序取出完整帧，交给下游处理（`business/origin_point_parser.cpp:203`）。

关键常量（`ys_lidar_validate_type.h:92`，命名空间 `CS96BetaRawParser`）：

```cpp
static constexpr size_t udp_packet_size = 1561;        // UDP包总大小 (1561B/slot)
static constexpr size_t origin_header_size = 9;         // OriginPointData头
static constexpr size_t slot_header_size = 16;          // slot头 (word0 4B + word1 4B + reserved 8B)
static constexpr size_t pixel_byte_size = 16;           // 单像素 2回波×(3+3+2)
static constexpr size_t pixels_per_slot = 96;           // 每slot像素数 (0~95, 倒序)
static constexpr size_t slot_data_size = 1552;          // 16 + 96×16
static constexpr size_t slot_counts = 800;              // 每帧slot数
static constexpr size_t echoes = 2;                     // 两重回波
static constexpr size_t frame_data_size = 1'241'600;    // 800 × 1552
static constexpr uint32_t kFpgaInvalidPixelValue = 16;  // FPGA无效点标识值
```

## 8. 参考实现与落盘

- 原始帧解析 + 文本落盘示例：`SaveCS96RawFrameSync`（`business/cs96_raw_save.cpp:49`），输出每 slot 的镜面角度与每像素三回波的前沿时间/后沿时间/峰值；
- 接收入口：`OriginPointParser::OriginPointReceive2`（`business/origin_point_parser.cpp:67`）；
- 解析线程入口：`OriginPointParser::OriginPointParse2`（`business/origin_point_parser.cpp:203`）。

## 9. 快速上手建议

1. 每收到一个 UDP 包，校验 `length % 1561 == 0`；
2. 解析 `OriginPointData` 头，取 `frame_id` / `slot_number` / `data_length`；
3. 把 `data[0..1552)` 按 `frame_id` 拼入对应帧缓存；
4. 一个 `frame_id` 收满 800 个 slot 即得到完整帧（1,241,600B）；
5. 遍历 800 个 slot：解析 16B slot 头，再逐像素解析两回波的前沿时间/后沿时间/峰值；
6. 过滤无效像素（前沿时间 `== 0` 或 `== 16`），即得到该帧完整的原始点云数据。
