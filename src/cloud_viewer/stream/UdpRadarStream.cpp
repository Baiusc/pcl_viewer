#include "cloud_viewer/stream/UdpRadarStream.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <cerrno>
using socket_t = int;
#endif

namespace cloud_viewer
{
namespace
{

#pragma pack(push, 1)
struct SocHead
{
    std::uint16_t head;         // 0xA55A
    std::uint8_t versionMajor;  // 1
    std::uint8_t dataType;      // 0x01 点云, 0x04 IMU
    std::uint16_t reserved;
};

struct SocProtocol
{
    std::uint8_t versionMinor;
    std::uint16_t dataSequence;
    std::uint16_t frameCnt;
    std::uint8_t timeType;
    std::uint16_t col;
    std::uint16_t row;
    std::uint16_t dotNum;
    std::uint8_t type;
    std::uint8_t echoNum;
    std::uint8_t frameFlag; // 0 start, 1 normal, 2 end
    std::uint8_t reserved[11];
    std::uint8_t crc8;
    std::uint32_t crc32;
    std::uint64_t timestamp; // us
};

struct SocDisData
{
    std::uint16_t distance; // cm
    std::uint8_t reflect;
    std::uint8_t label;
};

struct SocPtData
{
    std::int16_t x;
    std::int16_t y;
    std::int16_t z;
    std::uint8_t reflect;
    std::uint8_t label;
};

struct SocDisPtData
{
    std::uint16_t distance; // cm
    std::int16_t x;
    std::int16_t y;
    std::int16_t z;
    std::uint8_t reflect;
    std::uint8_t label;
};
#pragma pack(pop)

// ---- FL6532 (data_type=0x21)，定义来自 yscloudscene/business/fl6532_point_parser.h ----
#pragma pack(push, 1)
struct Fl6532Head
{
    std::uint16_t head;         // 0xA55A
    std::uint8_t verMajor;      // 2
    std::uint8_t dataType;      // 0x21
    std::uint8_t lidarModule;
    std::uint8_t reserved;
};

struct Fl6532Echo
{
    std::uint8_t verMinor;      // 2
    std::uint16_t seq;
    std::uint16_t frameCnt;
    std::uint16_t dotNum;
    std::uint8_t type;          // 0x02 DIS_XYZ
    std::uint8_t peakType21;
    std::uint8_t frameFlag;     // 0 start, 1 normal, 2 end
    std::uint16_t width;
    std::uint16_t height;
    std::uint16_t maxBinCnt;
    std::uint8_t abFrameIdx;
    std::uint8_t maxSubFrameCnt;
    std::uint8_t roiIndex;
    std::uint16_t pixelLen;     // 18
    std::uint16_t dataOffset;   // 40
    std::uint16_t dataLen;
    std::uint8_t peakType43;
    std::uint64_t timestampBase; // us
};

struct Fl6532PointRaw
{
    std::uint32_t timestampOffset;
    std::uint16_t col;
    std::uint16_t row;
    std::uint16_t distance;
    std::int16_t x;
    std::int16_t y;
    std::int16_t z;
    std::uint8_t reflectivity;
    std::uint8_t label;
};
#pragma pack(pop)

static_assert(sizeof(Fl6532Head) == 6, "Fl6532Head layout");
static_assert(sizeof(Fl6532Echo) == 34, "Fl6532Echo layout");
static_assert(sizeof(Fl6532PointRaw) == 18, "Fl6532PointRaw layout");

constexpr std::size_t kFl6532HeaderSize = 40;
constexpr std::size_t kFl6532PointSize = 18;
constexpr std::uint8_t kFl6532DataType = 0x21;
constexpr std::uint8_t kFl6532PointType = 0x02;
constexpr std::uint16_t kFl6532PixelLen = 18;
constexpr std::uint16_t kFl6532MaxPointsPerPacket = 79;

float fl6532UnitScale(std::uint8_t abFrameIdx)
{
    const std::uint8_t pclUnit = static_cast<std::uint8_t>((abFrameIdx >> 1) & 0x7F);
    const float n = static_cast<float>((pclUnit >> 5) & 0x03) +
                    static_cast<float>(pclUnit & 0x1F) * 0.03125f;
    return n > 0.0f ? n : 1.0f;
}

constexpr std::uint16_t kHeadMagic = 0xA55A;
constexpr std::uint8_t kDataTypeCloud = 0x01;
constexpr std::uint8_t kPointTypeDistance = 0;
constexpr std::uint8_t kPointTypeXyz = 1;
constexpr std::uint8_t kPointTypeDistanceXyz = 2;

// SDK 中 pt3d 为 cm：raw -> (raw/10)*2 cm -> /100 得到米。
constexpr float kCoordScale = 0.002f;
constexpr float kDistanceScale = 0.01f;

std::size_t pointSizeForType(std::uint8_t type)
{
    switch (type)
    {
    case kPointTypeDistance:
        return sizeof(SocDisData);
    case kPointTypeXyz:
        return sizeof(SocPtData);
    case kPointTypeDistanceXyz:
        return sizeof(SocDisPtData);
    default:
        return 0;
    }
}

std::uint64_t nowMs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

} // namespace

UdpRadarStream::~UdpRadarStream()
{
    stop();
}

bool UdpRadarStream::open(const UdpRadarConfig &config, FrameSink sink, std::string &error)
{
    stop();

    config_ = config;
    sink_ = std::move(sink);
    rangeDecoder_ = std::make_unique<RangeStreamDecoder>([this](range6532::PointCloudFrame&& frame) { receiveRangeFrame(std::move(frame)); });

#ifdef _WIN32
    WSADATA data;
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
    {
        error = "WSAStartup 失败";
        return false;
    }
    socket_ = static_cast<int>(::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
#else
    socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#endif
    if (socket_ < 0)
    {
        error = "创建 UDP socket 失败";
        return false;
    }

    int reuse = 1;
    setsockopt(static_cast<socket_t>(socket_), SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char *>(&reuse), sizeof(reuse));

    // 降低丢包：尽量增大接收缓冲。
    int bufferSize = 8 * 1024 * 1024;
    setsockopt(static_cast<socket_t>(socket_), SOL_SOCKET, SO_RCVBUF,
               reinterpret_cast<const char *>(&bufferSize), sizeof(bufferSize));

#ifdef _WIN32
    DWORD timeoutMs = 200;
    setsockopt(static_cast<socket_t>(socket_), SOL_SOCKET, SO_RCVTIMEO,
               reinterpret_cast<const char *>(&timeoutMs), sizeof(timeoutMs));
#else
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200 * 1000;
    setsockopt(static_cast<socket_t>(socket_), SOL_SOCKET, SO_RCVTIMEO,
               reinterpret_cast<const char *>(&timeout), sizeof(timeout));
#endif

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(config_.localPort);
    address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(static_cast<socket_t>(socket_), reinterpret_cast<sockaddr *>(&address),
               sizeof(address)) < 0)
    {
        error = std::string("绑定 UDP 端口失败: ") + std::strerror(errno);
        stop();
        return false;
    }

    sockaddr_in bound{};
#ifdef _WIN32
    int boundLength = sizeof(bound);
#else
    socklen_t boundLength = sizeof(bound);
#endif
    if (::getsockname(static_cast<socket_t>(socket_), reinterpret_cast<sockaddr*>(&bound), &boundLength) == 0)
        config_.localPort = ntohs(bound.sin_port);
    running_.store(true);
    hasFrameStart_ = false;
    framePackets_.clear();
    fpsWindowStartMs_ = nowMs();
    fpsWindowFrames_ = 0;

    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        stats_ = RadarStreamStats{};
    }

    thread_ = std::thread(&UdpRadarStream::receiveLoop, this);
    return true;
}

void UdpRadarStream::stop()
{
    running_.store(false);
    if (thread_.joinable())
    {
        thread_.join();
    }
    if (socket_ >= 0)
    {
#ifdef _WIN32
        closesocket(static_cast<socket_t>(socket_));
        WSACleanup();
#else
        ::close(socket_);
#endif
        socket_ = -1;
    }
    framePackets_.clear();
    fl6532Frames_.clear();
    rangeDecoder_.reset();
    hasFrameStart_ = false;
}

RadarStreamStats UdpRadarStream::stats() const
{
    std::lock_guard<std::mutex> lock(statsMutex_);
    return stats_;
}

void UdpRadarStream::receiveLoop()
{
    std::vector<std::uint8_t> buffer(65536);

    while (running_.load())
    {
        sockaddr_in from{};
#ifdef _WIN32
        int fromLength = sizeof(from);
#else
        socklen_t fromLength = sizeof(from);
#endif
        const int received = static_cast<int>(
            ::recvfrom(static_cast<socket_t>(socket_), reinterpret_cast<char *>(buffer.data()),
                       static_cast<int>(buffer.size()), 0,
                       reinterpret_cast<sockaddr *>(&from), &fromLength));
        if (received <= 0)
        {
            continue; // 超时或错误，继续等待
        }

        char ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
        const std::uint16_t port = ntohs(from.sin_port);

        if (std::getenv("CV_UDP_DEBUG") != nullptr)
        {
            static std::atomic<int> debugCount{0};
            if (debugCount.fetch_add(1) < 5)
            {
                std::fprintf(stderr, "[udp] recv %d bytes from %s:%u (expect %s:%u)\n",
                             received, ip, port, config_.radarIp.c_str(),
                             static_cast<unsigned>(config_.radarPort));
            }
        }

        if (config_.radarIp != ip)
        {
            continue;
        }
        if (port != config_.radarPort && port != 6301)
        {
            continue;
        }

        processDatagram(buffer.data(), static_cast<std::size_t>(received));
    }
}

void UdpRadarStream::processDatagram(const std::uint8_t *data, std::size_t length)
{
    if (config_.mode == RadarOutputMode::Range) {
        if (!((length >= 2 && data[0] == 0xaa && data[1] == 1) ||
              (length >= 4 && data[0] == 0x5a && data[1] == 0xa5 && data[3] == 0x2e))) return;
        std::string error;
        const bool ok = rangeDecoder_ && rangeDecoder_->ingest(data, length, error);
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.packets;
        if (!ok) { ++stats_.invalidPackets; stats_.lastError = error; }
        return;
    }
    if (length < sizeof(SocHead) + sizeof(SocProtocol))
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.invalidPackets;
        return;
    }

    const auto *head = reinterpret_cast<const SocHead *>(data);
    if (head->head != kHeadMagic)
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.invalidPackets;
        return;
    }

    if (head->dataType == kFl6532DataType)
    {
        processFl6532(data, length);
        return;
    }

    if (head->dataType != kDataTypeCloud)
    {
        return; // IMU / 其它类型暂不处理
    }

    const auto *protocol = reinterpret_cast<const SocProtocol *>(data + sizeof(SocHead));
    if (protocol->versionMinor != 0)
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.invalidPackets;
        return;
    }

    const std::size_t pixelSize = pointSizeForType(protocol->type);
    if (pixelSize == 0)
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.invalidPackets;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.packets;
    }

    const std::size_t headerSize = sizeof(SocHead) + sizeof(SocProtocol);
    if (length <= headerSize)
    {
        return;
    }

    if (protocol->frameFlag == 0)
    {
        framePackets_.clear();
        hasFrameStart_ = true;
    }
    else if (!hasFrameStart_)
    {
        return; // 丢弃没有帧首的残余包
    }

    PacketRef packet;
    packet.payload.assign(data + headerSize, data + length);
    packet.startRow = protocol->row;
    packet.startCol = protocol->col;
    packet.dotNum = protocol->dotNum;
    packet.pointType = protocol->type;
    packet.timestampUs = protocol->timestamp;
    framePackets_.push_back(std::move(packet));

    if (protocol->frameFlag == 2)
    {
        flushFrame();
    }
}

void UdpRadarStream::flushFrame()
{
    if (framePackets_.empty())
    {
        hasFrameStart_ = false;
        return;
    }

    // 先确定本帧矩阵尺寸（不同固件行列数可能不同）。
    int maxRow = -1;
    int maxCol = -1;
    for (const auto &packet : framePackets_)
    {
        const std::size_t pixelSize = pointSizeForType(packet.pointType);
        if (pixelSize == 0 || packet.payload.size() < pixelSize)
        {
            continue;
        }
        const std::size_t count = packet.payload.size() / pixelSize;
        if (count == 0)
        {
            continue;
        }
        maxRow = std::max(maxRow, static_cast<int>(packet.startRow) +
                                      static_cast<int>((count - 1) / 4));
        maxCol = std::max(maxCol, static_cast<int>(packet.startCol) + 3);
    }

    if (maxRow < 0 || maxCol < 0)
    {
        framePackets_.clear();
        hasFrameStart_ = false;
        return;
    }

    const int rows = maxRow + 1;
    const int cols = maxCol + 1;
    algo_lidar::PointMatrix matrix(static_cast<std::size_t>(rows),
                                   std::vector<algo_lidar::PointData>(
                                       static_cast<std::size_t>(cols)));

    for (const auto &packet : framePackets_)
    {
        const std::size_t pixelSize = pointSizeForType(packet.pointType);
        if (pixelSize == 0)
        {
            continue;
        }
        const std::size_t count = packet.payload.size() / pixelSize;

        for (std::size_t j = 0; j < count; ++j)
        {
            const int row = static_cast<int>(packet.startRow) + static_cast<int>(j / 4);
            const int col = static_cast<int>(packet.startCol) + static_cast<int>(j % 4);
            if (row < 0 || row >= rows || col < 0 || col >= cols)
            {
                continue;
            }

            const std::uint8_t *pixel = packet.payload.data() + j * pixelSize;
            algo_lidar::PointData point{};
            point.pixel_idx = static_cast<std::uint16_t>(row);
            point.slot_idx = static_cast<std::uint16_t>(col);
            point.timestamp = static_cast<std::uint32_t>(packet.timestampUs);

            float distanceCm = 0.0f;
            float rawX = 0.0f;
            float rawY = 0.0f;
            float rawZ = 0.0f;
            std::uint8_t reflect = 0;

            if (packet.pointType == kPointTypeDistance)
            {
                const auto *value = reinterpret_cast<const SocDisData *>(pixel);
                distanceCm = static_cast<float>(value->distance);
                reflect = value->reflect;
            }
            else if (packet.pointType == kPointTypeXyz)
            {
                const auto *value = reinterpret_cast<const SocPtData *>(pixel);
                rawX = static_cast<float>(value->x);
                rawY = static_cast<float>(value->y);
                rawZ = static_cast<float>(value->z);
                reflect = value->reflect;
            }
            else
            {
                const auto *value = reinterpret_cast<const SocDisPtData *>(pixel);
                distanceCm = static_cast<float>(value->distance);
                rawX = static_cast<float>(value->x);
                rawY = static_cast<float>(value->y);
                rawZ = static_cast<float>(value->z);
                reflect = value->reflect;
            }

            point.intensity = reflect;
            point.reflect = reflect / 255.0f;
            point.peak = reflect;

            if (packet.pointType == kPointTypeXyz || packet.pointType == kPointTypeDistanceXyz)
            {
                point.x = rawX * kCoordScale;
                point.y = rawY * kCoordScale;
                point.z = rawZ * kCoordScale;
            }

            if (distanceCm > 0.0f)
            {
                point.r = distanceCm * kDistanceScale;
            }
            else
            {
                point.r = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            }

            if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f && point.r > 0.0f)
            {
                // 仅有距离时用行列角近似还原坐标（具体视场角需按型号标定）。
                const float theta = (static_cast<float>(row) - rows * 0.5f) /
                                    static_cast<float>(rows) * 50.0f;
                const float slot = (static_cast<float>(col) - cols * 0.5f) /
                                   static_cast<float>(cols) * 120.0f;
                constexpr float kPi = 3.14159265358979323846f;
                point.theta = theta;
                point.theta_rad = theta * kPi / 180.0f;
                point.slot_ang = slot;
                point.omega_rad = slot * kPi / 180.0f;
                point.x = point.r * std::cos(point.theta_rad) * std::cos(point.omega_rad);
                point.y = point.r * std::cos(point.theta_rad) * std::sin(point.omega_rad);
                point.z = point.r * std::sin(point.theta_rad);
            }
            else if (point.r > 1e-6f)
            {
                constexpr float kPi = 3.14159265358979323846f;
                point.theta = std::asin(std::clamp(point.z / point.r, -1.0f, 1.0f)) *
                              180.0f / kPi;
                point.theta_rad = point.theta * kPi / 180.0f;
                point.slot_ang = std::atan2(point.y, point.x) * 180.0f / kPi;
                point.omega_rad = point.slot_ang * kPi / 180.0f;
            }

            point.is_valid = (distanceCm > 0.0f) ||
                             (point.x != 0.0f || point.y != 0.0f || point.z != 0.0f);
            point.select_idx = 0;

            matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
                std::move(point);
        }
    }

    const std::uint64_t timestamp = framePackets_.back().timestampUs;
    framePackets_.clear();
    hasFrameStart_ = false;

    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.frames;
        ++fpsWindowFrames_;

        const std::uint64_t now = nowMs();
        if (now - fpsWindowStartMs_ >= 1000)
        {
            stats_.framesPerSecond = static_cast<double>(fpsWindowFrames_) * 1000.0 /
                                     static_cast<double>(now - fpsWindowStartMs_);
            fpsWindowFrames_ = 0;
            fpsWindowStartMs_ = now;
        }
    }

    if (sink_)
    {
        sink_(std::move(matrix), timestamp, -1);
    }
}

void UdpRadarStream::processFl6532(const std::uint8_t *data, std::size_t length)
{
    if (length < kFl6532HeaderSize)
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.invalidPackets;
        return;
    }

    const auto *head = reinterpret_cast<const Fl6532Head *>(data);
    const auto *echo = reinterpret_cast<const Fl6532Echo *>(data + sizeof(Fl6532Head));
    if (head->head != kHeadMagic || head->verMajor != 2 ||
        head->dataType != kFl6532DataType ||
        echo->verMinor != 2 || echo->type != kFl6532PointType ||
        echo->pixelLen != kFl6532PixelLen ||
        echo->dotNum == 0 || echo->dotNum > kFl6532MaxPointsPerPacket ||
        // 该固件 dataOffset 字段实际填 0（代码用固定 40），不能作为强校验。
        (echo->dataOffset != 0 && echo->dataOffset != kFl6532HeaderSize) ||
        echo->roiIndex > 1 ||
        echo->width != 192 || (echo->height != 128 && echo->height != 48))
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.invalidPackets;
        return;
    }

    const std::size_t need = kFl6532HeaderSize +
                             static_cast<std::size_t>(echo->dotNum) * kFl6532PointSize;
    if (length < need)
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.invalidPackets;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.packets;
    }

    if (echo->roiIndex != config_.roi) return;
    const float unitScale = fl6532UnitScale(echo->abFrameIdx);
    const std::uint32_t key = (std::uint32_t(echo->roiIndex) << 16) | echo->frameCnt;
    if (!fl6532Frames_.count(key) && fl6532Frames_.size() >= 8) {
        const auto old = std::min_element(fl6532Frames_.begin(), fl6532Frames_.end(),
            [](const auto& a, const auto& b) { return a.second.touch < b.second.touch; });
        fl6532Frames_.erase(old);
    }
    Fl6532Frame &frame = fl6532Frames_[key];
    frame.touch = ++cloudTick_;
    if (frame.received == 0)
    {
        frame.frameCnt = echo->frameCnt;
        frame.roi = echo->roiIndex;
        frame.seen.resize(static_cast<std::size_t>(echo->width)*echo->height);
        frame.width = echo->width;
        frame.height = echo->height;
        frame.timestampBase = echo->timestampBase;
    }

    if (frame.width != echo->width || frame.height != echo->height || frame.timestampBase != echo->timestampBase) {
        fl6532Frames_.erase(key); return;
    }
    if (frame.packets.count(echo->seq) != 0)
    {
        return; // 重复包
    }

    std::vector<Fl6532PointOut> points;
    points.reserve(echo->dotNum);
    for (std::uint16_t i = 0; i < echo->dotNum; ++i)
    {
        const auto *raw = reinterpret_cast<const Fl6532PointRaw *>(
            data + kFl6532HeaderSize + static_cast<std::size_t>(i) * kFl6532PointSize);

        Fl6532PointOut out;
        out.timestampUs = echo->timestampBase + raw->timestampOffset;
        out.row = raw->row;
        out.col = raw->col;
        const int localRow = static_cast<int>(out.row) - (frame.roi == 1 ? 16 : 0);
        if (localRow < 0 || localRow >= frame.height || out.col >= frame.width) continue;
        const auto index = static_cast<std::size_t>(localRow)*frame.width+out.col;
        if (frame.seen[index]) continue;
        frame.seen[index] = true;
        ++frame.received;
        out.distanceCm = static_cast<float>(raw->distance) / unitScale;
        out.x = static_cast<float>(raw->x) / unitScale / 100.0f;
        out.y = static_cast<float>(raw->y) / unitScale / 100.0f;
        out.z = static_cast<float>(raw->z) / unitScale / 100.0f;
        out.reflectivity = raw->reflectivity;
        out.label = raw->label;
        points.push_back(out);
    }

    if (!points.empty()) frame.packets.emplace(echo->seq, std::move(points));

    const std::uint32_t totalPixels = static_cast<std::uint32_t>(frame.width) * frame.height;
    if (frame.received == totalPixels)
    {
        flushFl6532(key);
    }
}

void UdpRadarStream::flushFl6532(std::uint32_t key)
{
    const auto it = fl6532Frames_.find(key);
    if (it == fl6532Frames_.end())
    {
        return;
    }

    Fl6532Frame frame = std::move(it->second);
    fl6532Frames_.erase(it);

    const int rows = static_cast<int>(frame.height);
    const int cols = static_cast<int>(frame.width);
    if (rows <= 0 || cols <= 0)
    {
        return;
    }

    algo_lidar::PointMatrix matrix(static_cast<std::size_t>(rows),
                                   std::vector<algo_lidar::PointData>(
                                       static_cast<std::size_t>(cols)));

    for (const auto &entry : frame.packets)
    {
        for (const auto &p : entry.second)
        {
            const int localRow = static_cast<int>(p.row) - (frame.roi == 1 ? 16 : 0);
            if (localRow < 0 || localRow >= rows || p.col >= cols)
            {
                continue;
            }

            algo_lidar::PointData &point = matrix[localRow][p.col];
            point.pixel_idx = p.row;
            point.slot_idx = p.col;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            point.r = p.distanceCm / 100.0f;
            if (point.r <= 0.0f)
            {
                point.r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            }
            point.theta_rad = std::atan2(p.z, std::hypot(p.x, p.y));
            point.omega_rad = std::atan2(p.y, p.x);
            point.theta = point.theta_rad * 180.0f / 3.14159265358979323846f;
            point.slot_ang = point.omega_rad * 180.0f / 3.14159265358979323846f;
            point.intensity = p.reflectivity;
            point.reflect = static_cast<float>(p.reflectivity) / 255.0f;
            point.peak = p.reflectivity;
            point.timestamp = static_cast<std::uint32_t>(p.timestampUs);
            point.is_valid = !(std::abs(p.x) < 1e-6f && std::abs(p.y) < 1e-6f &&
                               std::abs(p.z) < 1e-6f);
            point.select_idx = 0;
        }
    }

    emitMatrix(std::move(matrix), frame.timestampBase, frame.roi);
}

void UdpRadarStream::receiveRangeFrame(range6532::PointCloudFrame&& frame)
{
    if (frame.roiIndex != config_.roi) return;
    algo_lidar::PointMatrix matrix(frame.rows, std::vector<algo_lidar::PointData>(frame.cols));
    std::uint64_t timestamp = 0;
    for (const auto& p : frame.points) {
        if (p.row < frame.rowStart || p.row-frame.rowStart >= frame.rows || p.col >= frame.cols) continue;
        auto& out = matrix[p.row-frame.rowStart][p.col];
        out.pixel_idx=p.row; out.slot_idx=p.col; out.r=p.distanceMetres;
        out.x=p.x; out.y=p.y; out.z=p.z;
        out.theta=p.verticalDeg; out.slot_ang=p.horizontalDeg;
        out.theta_rad=p.verticalDeg*3.14159265358979323846f/180.0f;
        out.omega_rad=p.horizontalDeg*3.14159265358979323846f/180.0f;
        out.timestamp=p.timestamp; timestamp=std::max(timestamp,std::uint64_t(p.timestamp));
        out.is_valid=p.valid; out.select_idx=p.selectedEcho < 0 ? 0 : static_cast<std::uint8_t>(p.selectedEcho);
        for(const auto& echo:p.echoes) {
            const float metres=static_cast<float>(echo.distanceRisingRaw)/25600.0f;
            out.list_pulse.emplace_back(metres*2.0f/3.0e8f*1.0e12f,
                                       static_cast<float>(echo.fwhmRaw)/256.0f,
                                       static_cast<float>(echo.convolutionIntensity));
        }
        if(p.valid) {
            const auto& echo=p.echoes[out.select_idx];
            out.peak=static_cast<std::uint16_t>(std::min<std::uint32_t>(echo.convolutionIntensity,65535));
            out.intensity=echo.peakAux; out.reflect=p.intensity;
        }
    }
    emitMatrix(std::move(matrix), timestamp, frame.roiIndex);
}

void UdpRadarStream::emitMatrix(algo_lidar::PointMatrix &&matrix, std::uint64_t timestampUs, int roi)
{
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.frames;
        ++fpsWindowFrames_;

        const std::uint64_t now = nowMs();
        if (now - fpsWindowStartMs_ >= 1000)
        {
            stats_.framesPerSecond = static_cast<double>(fpsWindowFrames_) * 1000.0 /
                                     static_cast<double>(now - fpsWindowStartMs_);
            fpsWindowFrames_ = 0;
            fpsWindowStartMs_ = now;
        }
    }

    if (sink_)
    {
        sink_(std::move(matrix), timestampUs, roi);
    }
}

} // namespace cloud_viewer
