#include "cloud_viewer/stream/BetaUdpStream.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace cloud_viewer
{
namespace
{

constexpr int kLines = 96;
constexpr int kSlots = 800;
constexpr int kBins = 512;
// 直方图每包 4 个像素：96 线 24 包、120 线 30 包。不能写死。
constexpr int histogramPacketCount(int lines)
{
    return (lines > 0 ? lines : kLines) / 4;
}
// 单包点数上限：96×800 与 120×800 共用同一套解析，数组按最大值固定分配。
constexpr int kMaxLines = 160;

/// 垂直线对应的俯仰角（度）。
/// centerLine 为标定里的 u（SPAD 中心线）；未提供时才退回 (lines-1)/2。
/// Gamma（120 线）的 u 仍是 47，若按 (120-1)/2=59 推导会整体上仰约 2°。
inline float pitchDegrees(int line, int lines, int centerLine = -1)
{
    const int u = centerLine >= 0 ? centerLine : (lines - 1) / 2;
    return static_cast<float>((line - u) * (20.0 / lines) - 2.5);
}

constexpr std::size_t kCartesianHead = 18;
constexpr std::size_t kCartesianPoint = 13;
constexpr std::size_t kOriginHead = 9;
constexpr std::size_t kOriginSlotHead = 16;
constexpr std::size_t kOriginPixel = 16;
constexpr std::size_t kHistogramHead = 9;

// kLightSpeed 单位：米/皮秒；r = kLightSpeed * (ft >> 1)。
constexpr float kLightSpeed = 299792458.0f * 1e-12f;
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

std::uint16_t readLe16(const std::uint8_t *data)
{
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

std::uint32_t readLe24(const std::uint8_t *data)
{
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8) |
           (static_cast<std::uint32_t>(data[2]) << 16);
}

std::uint32_t readLe32(const std::uint8_t *data)
{
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8) |
           (static_cast<std::uint32_t>(data[2]) << 16) |
           (static_cast<std::uint32_t>(data[3]) << 24);
}

float readFloat(const std::uint8_t *data)
{
    float value = 0.0f;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

bool validXyz(float x, float y, float z)
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
        return false;
    }
    constexpr float kLimit = 1000.0f;
    return std::fabs(x) < kLimit && std::fabs(y) < kLimit && std::fabs(z) < kLimit;
}

struct ParsedPacket
{
    int mode = -1;          // 0/1/2
    int face = -1;          // 0~3，直方图为 -1
    std::uint16_t frame = 0;
    std::uint16_t index = 0; // 模式 2/1 为 slot，模式 0 为 seq
    int pointCount = 0;
    algo_lidar::PointData points[kMaxLines];
    std::uint16_t bins[4 * kBins];
};

bool parseCartesian(const std::uint8_t *data, ParsedPacket &packet, int lines, int slots)
{
    packet.frame = readLe16(data + 10);
    const std::uint16_t totalSlots = readLe16(data + 12);
    packet.index = readLe16(data + 14);
    if (packet.frame > 3 || totalSlots != slots || packet.index >= slots)
    {
        return false;
    }

    const double degPerSlot = 360.0 / static_cast<double>(slots);
    for (int line = 0; line < lines; ++line)
    {
        const std::uint8_t *raw = data + kCartesianHead + static_cast<std::size_t>(line) * kCartesianPoint;
        const float x = readFloat(raw);
        const float y = readFloat(raw + 4);
        const float z = readFloat(raw + 8);
        const std::uint8_t albedo = raw[12];

        algo_lidar::PointData &point = packet.points[line];

        point.x = x;
        point.y = y;
        point.z = z;
        point.r = std::sqrt(x * x + y * y + z * z);
        point.theta = static_cast<float>(std::asin(z / std::max(point.r, 1e-6f)) * 180.0 / M_PI);
        // 真实点云的 xyz 由 arm 算好，方位角 和 俯仰角 直接反算，别再按 slot 线性铺。
        // 实测：一个面 120°/800 slot，方向与 index 递增相反。
        point.slot_ang = static_cast<float>(std::atan2(y, x) * 180.0 / M_PI);
        point.intensity = albedo;
        point.reflect = static_cast<float>(albedo);
        point.peak = 0;
        point.select_idx = 0;
        point.timestamp = 0;
        point.pixel_idx = static_cast<std::uint16_t>(line);
        point.slot_idx = packet.index;
        point.is_valid = validXyz(x, y, z);
        point.is_tail_pt = false;
        point.list_pulse.clear();
    }
    packet.pointCount = lines;
    packet.mode = 2;
    // 一帧就是一个面，与 slot 无关；真实 xyz 不做二次旋转，面号按帧 ID 取模。
    packet.face = static_cast<int>(packet.frame % 4);
    return true;
}

/// 标定表完全缺失时的最小兜底：只保证点云可见，不代表设备端任何分支。
/// 设备端缺标定表会直接启动失败，这里仅避免 viewer 无图可看。
void applyFallbackNoTable(int line, std::uint32_t ft, std::uint16_t slot, std::uint16_t peak,
                          int face, int lines, int slots, int centerLine,
                          algo_lidar::PointData &point)
{
    point.pixel_idx = static_cast<std::uint16_t>(line);
    point.slot_idx = slot;
    point.peak = peak;
    point.select_idx = 0;
    point.timestamp = 0;
    point.is_tail_pt = false;
    point.list_pulse.clear();

    const float r = (ft > 16) ? kLightSpeed * static_cast<float>(ft >> 1) : 0.0f;
    const float thetaDeg = pitchDegrees(line, lines, centerLine);
    const float omegaDeg = -60.0f + 120.0f * slot / slots;
    const float theta = thetaDeg * kDegToRad;
    const float omega = omegaDeg * kDegToRad;

    point.r = r;
    point.theta = thetaDeg;
    point.omega_rad = omega;
    point.slot_ang = omegaDeg;
    point.reflect = 0.0f;
    point.intensity = static_cast<std::uint8_t>(peak > 1612 ? 255 : peak * 255u / 1612u);
    point.is_valid = face >= 0 && r > 0.1f && r < 200.0f;
    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    if (point.is_valid)
    {
        const float horizontal = r * std::cos(theta);
        // 统一约定：x+ 前、y+ 右；slot0 在左（y<0），与 Cs96BetaCalibration 一致。
        point.x = horizontal * std::cos(omega);
        point.y = horizontal * std::sin(omega);
        point.z = r * std::sin(theta);
    }
}

bool parseOrigin(const std::uint8_t *data, ParsedPacket &packet,
                 const Cs96BetaCalibration *calibration,
                 const Cs96BetaCalibration::DecodeOptions &options, int lockedFace,
                 int lines, int slots, int pitchCenter, bool faceFromFrameId)
{
    packet.frame = readLe16(data + 1);
    packet.index = readLe16(data + 3);
    const std::uint16_t dataLength = readLe16(data + 7);
    // slot 数据长度 = 16B slot 头 + lines×16B；96 线为 1552，120 线为 1936。
    const int expectedLength = 16 + lines * 16;
    if (data[0] != 0xAA || dataLength != expectedLength || packet.index >= slots)
    {
        return false;
    }

    // slot 头 word0 的高 20 位为 FPGA 转镜角度。
    const std::uint32_t word0 = readLe32(data + kOriginHead);
    const std::uint32_t mirrorAngle = (word0 >> 12) & 0xFFFFFu;

    packet.face = lockedFace;
    // Gamma（SL100）板端是 fpga_frame_id % 4，不做 PA 纠偏；
    // 这里沿用同一策略，避免拿 Beta 的 PA 区间表把面号判错。
    if (packet.face < 0 && faceFromFrameId)
    {
        packet.face = static_cast<int>(packet.frame % 4);
    }
    // slot0 为同步包，实测 PA 恒为 0；只用帧起始窗口内首个有效 PA 判面。
    // 中途接入（前 10% slot 全丢）不猜测该帧，等待下一帧；后续绝不逐 slot 重新判面。
    if (packet.face < 0 && packet.index > 0 && packet.index <= slots / 10)
    {
        // 判面只依赖整圈编码器表（PA 实测的结构参数），不受四项标定开关影响。
        if (calibration != nullptr && calibration->loaded())
        {
            const int scale = static_cast<int>((mirrorAngle >> 12) & 0xFF);
            const int realAngle = static_cast<int>(mirrorAngle & 0xFFF);
            packet.face = calibration->faceByPa(calibration->paAngle(scale, realAngle));
        }
        if (packet.face < 0)
        {
            packet.face = BetaUdpStream::faceFromPa(mirrorAngle);
        }
    }

    const std::uint8_t *pixels = data + kOriginHead + kOriginSlotHead;
    for (int i = 0; i < lines; ++i)
    {
        // 数据区第 1 个像素是 pixel[lines-1]，倒序存放。
        const int line = lines - 1 - i;
        const std::uint8_t *raw = pixels + static_cast<std::size_t>(i) * kOriginPixel;

        const std::uint32_t ft0 = readLe24(raw);
        const std::uint16_t pk0 = readLe16(raw + 6);
        const std::uint32_t ft1 = readLe24(raw + 8);
        const std::uint16_t pk1 = readLe16(raw + 14);

        const bool ok0 = ft0 > 16;
        const bool ok1 = ft1 > 16;
        std::uint32_t ft = 0;
        std::uint16_t peak = 0;
        if (ok0 && ok1)
        {
            if (pk1 > pk0) { ft = ft1; peak = pk1; }
            else { ft = ft0; peak = pk0; }
        }
        else if (ok0)
        {
            ft = ft0;
            peak = pk0;
        }
        else if (ok1)
        {
            ft = ft1;
            peak = pk1;
        }

        algo_lidar::PointData &point = packet.points[line];
        if (calibration != nullptr && calibration->loaded())
        {
            // 结构参数始终生效，四项标定由 options 逐项控制。
            calibration->apply(line, ft, mirrorAngle, packet.index, peak, packet.face, options,
                               point);
            continue;
        }

        // 没有标定表：设备端会直接启动失败，这里仅做最小兜底保证有图可看。
        applyFallbackNoTable(line, ft, packet.index, peak, packet.face, lines, slots,
                             pitchCenter, point);
    }
    packet.pointCount = lines;
    packet.mode = 1;
    return true;
}

bool parseHistogram(const std::uint8_t *data, ParsedPacket &packet, int histogramPackets)
{
    packet.frame = readLe16(data + 1);
    packet.index = readLe16(data + 5);
    const std::uint16_t dataLength = readLe16(data + 7);
    if (dataLength != 4096 || packet.index >= histogramPackets)
    {
        return false;
    }

    const std::uint8_t *payload = data + kHistogramHead;
    for (int i = 0; i < 4 * kBins; ++i)
    {
        packet.bins[i] = readLe16(payload + i * 2);
    }
    packet.pointCount = 0;
    packet.mode = 0;
    return true;
}

bool parseAny(const std::uint8_t *data, std::size_t size, ParsedPacket &packet,
              const Cs96BetaCalibration *calibration,
              const Cs96BetaCalibration::DecodeOptions &options, int lockedFace,
              int lines, int slots, int pitchCenter, bool faceFromFrameId)
{
    if (data == nullptr)
    {
        return false;
    }
    // 不整体重建 ParsedPacket：points 含 vector 成员，逐包重建会拖垮接收线程。
    packet.mode = -1;
    // 包长随线数变化：96 线真实点云 1266B / 原始 1561B，120 线分别为 1578B / 1945B。
    if (size == BetaUdpStream::cartesianPacketSize(lines))
    {
        return parseCartesian(data, packet, lines, slots);
    }
    if (size == BetaUdpStream::originPacketSize(lines))
    {
        return parseOrigin(data, packet, calibration, options, lockedFace, lines, slots,
                           pitchCenter, faceFromFrameId);
    }
    if (size == BetaUdpStream::kHistogramPacketSize)
    {
        return parseHistogram(data, packet, histogramPacketCount(lines));
    }
    return false;
}

} // namespace

struct BetaUdpStream::Impl
{
    Config config;
    FrameSink sink;
    std::shared_ptr<const Cs96BetaCalibration> calibration;
    Cs96BetaCalibration::DecodeOptions options;
    std::uint64_t optionsGeneration = 0;
    std::uint64_t activeGeneration = 0;
    mutable std::mutex mutex;
    std::thread worker;
    bool stopRequested = false;
    bool active = false;
    Stats stats;

    /// 运行期规格（96×800 或 120×800）；非法值退回默认 96×800。
    int lineCount() const { return config.lines > 0 ? config.lines : kLines; }
    int slotCount() const { return config.slots > 0 ? config.slots : kSlots; }

    // 帧缓存（模式 2/1：lines×slots；模式 0：lines×512）。
    int currentMode = -1;
    bool hasFrame = false;
    std::uint16_t currentFrame = 0;
    int currentFace = -1;
    std::vector<std::uint8_t> slotReceived;
    std::vector<std::uint16_t> histBins;
    std::vector<std::uint8_t> histSeqReceived;
    std::atomic<std::uint8_t> faceMask{0x0F};
    std::uint8_t publishedMask = 0x0F;
    // 仅原始点云累积：每个面的最近一张达到 90% 完整度的帧。
    struct FaceFrame
    {
        std::vector<algo_lidar::PointData> points; // slot-major
        std::vector<std::uint8_t> received;
        bool ready = false;
    };
    std::array<FaceFrame, 4> originFaces;
    bool closedFrame = false;
    std::uint16_t lastClosedFrame = 0;
    int lastClosedMode = -1;
    int receivedCount = 0;
    int histCount = 0;
    std::atomic<std::uint64_t> packetCounter{0};
    /// 上一次发布的面号；选面开关变化时重发它，避免只改了掩码却不出图。
    int lastPublishedFace = -1;

    void resetFrame(int mode, std::uint16_t frame)
    {
        if (currentMode != mode)
        {
            for (auto &face : originFaces) face.ready = false;
        }
        currentMode = mode;
        hasFrame = true;
        currentFrame = frame;
        currentFace = -1;
        if (mode == 0)
        {
            histBins.assign(static_cast<std::size_t>(lineCount()) * kBins, 0);
            histSeqReceived.assign(static_cast<std::size_t>(histogramPacketCount(lineCount())), 0);
        }
        else
        {
            slotReceived.assign(static_cast<std::size_t>(slotCount()), 0);
            receivedCount = 0;
            // 仅在首次分配；后续帧复用同一块内存，避免每帧重建整帧点云。
            if (flatFrame.size() != static_cast<std::size_t>(lineCount()) * slotCount())
            {
                flatFrame.resize(static_cast<std::size_t>(lineCount()) * slotCount());
            }
        }
        histCount = 0;
    }

    /// 发布单个面（模式 1）。
    ///
    /// 一帧就是一个面：96×800、FOV 120°（车规级前向主雷达，不是环视雷达）。
    /// 原始点云按面流式输出：某个面凑齐 90% 就立刻发出去，不再等 4 个面齐了
    /// 拼成一个超宽矩阵。设备本身就是一帧一个面（10Hz），拼接只会把 4 个面
    /// 压到同一角度区间（angle 计算是每面相对各自起始角），表现为
    /// 「墙壁变厚 / 同一面墙出现两三次」。单面输出后与真实点云一致：
    /// 一帧一个面，下一帧换下一个面。
    void publishOrigin(int face, std::uint64_t frameNumber)
    {
        if (face < 0 || face >= 4)
        {
            return;
        }
        const auto mask = faceMask.load();
        if ((mask & (1u << face)) == 0)
        {
            // 这个面没被选中：只更新掩码，不发布，避免用空帧冲掉当前画面。
            publishedMask = mask;
            return;
        }
        const auto &cached = originFaces[face];
        if (!cached.ready)
        {
            return;
        }

        const int lines = lineCount();
        const int slots = slotCount();
        algo_lidar::PointMatrix matrix(lines, std::vector<algo_lidar::PointData>(slots));
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (int line = 0; line < lines; ++line)
        {
            for (int slot = 0; slot < slots; ++slot)
            {
                auto &target = matrix[line][slot];
                if (cached.received[static_cast<std::size_t>(slot)])
                {
                    target = cached.points[static_cast<std::size_t>(slot) * lines + line];
                }
                else
                {
                    target.x = target.y = target.z = nan;
                    target.is_valid = false;
                }
            }
        }
        publishedMask = mask;
        lastPublishedFace = face;
        // CV_FACE_TRACE=1 时打印发布的面号，用来确认 4 面确实在轮转。
        if (std::getenv("CV_FACE_TRACE") != nullptr)
        {
            std::fprintf(stderr, "[face] publish face=%d frame=%llu\n", face,
                         static_cast<unsigned long long>(frameNumber));
            std::fflush(stderr);
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            ++stats.frames;
        }
        if (sink)
        {
            sink(std::move(matrix), frameNumber);
        }
    }

    void flushFrame()
    {
        if (!hasFrame)
        {
            return;
        }
        hasFrame = false;
        closedFrame = true;
        lastClosedFrame = currentFrame;
        lastClosedMode = currentMode;
        const int mode = currentMode;
        const std::uint64_t frameNumber = currentFrame;

        if (mode == 0)
        {
            const int received = histCount;
            const int expectedPackets = histogramPacketCount(lineCount());
            if (received < expectedPackets)
            {
                std::lock_guard<std::mutex> lock(mutex);
                stats.droppedSlots += static_cast<std::uint64_t>(expectedPackets - received);
                return;
            }

            // 直方图：bin × 像素 × 强度，构造可观察地形。
            const int lines = lineCount();
            algo_lidar::PointMatrix matrix(lines, std::vector<algo_lidar::PointData>(kBins));
            for (int line = 0; line < lines; ++line)
            {
                for (int bin = 0; bin < kBins; ++bin)
                {
                    const std::uint16_t value =
                        histBins[static_cast<std::size_t>(line) * kBins + bin];
                    algo_lidar::PointData &point = matrix[line][bin];
                    point = algo_lidar::PointData{};
                    point.x = static_cast<float>(bin) * 0.05f;
                    point.y = static_cast<float>(line) * 0.05f;
                    point.z = static_cast<float>(value) / 65535.0f * 10.0f;
                    point.intensity = static_cast<std::uint8_t>(value >> 8);
                    point.reflect = static_cast<float>(value);
                    point.peak = value;
                    point.pixel_idx = static_cast<std::uint16_t>(line);
                    point.slot_idx = static_cast<std::uint16_t>(bin);
                    point.is_valid = value > 0;
                }
            }
            histBins.clear();
            histSeqReceived.clear();

            {
                std::lock_guard<std::mutex> lock(mutex);
                ++stats.frames;
            }
            if (sink)
            {
                sink(std::move(matrix), frameNumber);
            }
            return;
        }

        const int slots = slotCount();
        const int received = receivedCount;
        if (received < slots * 9 / 10)
        {
            std::lock_guard<std::mutex> lock(mutex);
            stats.droppedSlots += static_cast<std::uint64_t>(slots - received);
            return;
        }
        const int face = currentFace;
        if (face < 0 || face >= 4) return;
        const auto mask = faceMask.load();
        if (mode == 1)
        {
            auto &cached = originFaces[face];
            cached.points.swap(flatFrame);
            cached.received = slotReceived;
            cached.ready = true;
            // 流式输出：这个面凑齐就立刻发布，不等其他三个面。
            publishOrigin(face, frameNumber);
            return;
        }
        // 真实点云仍然直接输出设备 xyz，行列数跟随设备规格；只修正整帧选面的语义。
        if ((mask & (1u << face)) == 0) return;
        const int lines = lineCount();
        algo_lidar::PointMatrix matrix(lines, std::vector<algo_lidar::PointData>(slots));
        for (int slot = 0; slot < slots; ++slot)
        {
            for (int line = 0; line < lines; ++line)
            {
                auto &target = matrix[line][slot];
                if (slotReceived[slot])
                    target = flatFrame[static_cast<std::size_t>(slot) * lines + line];
                else
                {
                    target.x = target.y = target.z = std::numeric_limits<float>::quiet_NaN();
                    target.is_valid = false;
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex);
            ++stats.frames;
        }
        if (sink)
        {
            sink(std::move(matrix), frameNumber);
        }
    }

    std::vector<algo_lidar::PointData> flatFrame;

    void loop()
    {
#ifndef _WIN32
        const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
        {
            std::lock_guard<std::mutex> lock(mutex);
            active = false;
            return;
        }

        int reuse = 1;
        ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        // 设备以约 7800 包/秒推流，默认接收缓冲会大量丢包。
        int rcvbuf = 8 * 1024 * 1024;
        ::setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;
        ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(config.localPort);
        if (::bind(sock, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
        {
            ::close(sock);
            std::lock_guard<std::mutex> lock(mutex);
            active = false;
            return;
        }

        in_addr expectedSource{};
        if (inet_pton(AF_INET, config.radarIp.c_str(), &expectedSource) != 1)
        {
            ::close(sock);
            std::lock_guard<std::mutex> lock(mutex);
            active = false;
            return;
        }
        std::vector<std::uint8_t> buffer(kHistogramPacketSize);
        ParsedPacket packet;

        // 每 5 秒输出一次接收统计，便于定位丢包发生在内核还是解析阶段。
        auto lastStat = std::chrono::steady_clock::now();
        std::uint64_t recvCount = 0;
        std::uint64_t parseCount = 0;
        std::uint64_t timeoutCount = 0;
        const bool drainOnly = std::getenv("CV_BETA_DRAIN") != nullptr;
        const bool debugStats = std::getenv("CV_BETA_DEBUG") != nullptr;

        while (true)
        {
            std::shared_ptr<const Cs96BetaCalibration> packetCalibration;
            Cs96BetaCalibration::DecodeOptions packetOptions;
            std::uint64_t packetGeneration;
            {
                std::lock_guard<std::mutex> lock(mutex);
                packetCalibration = calibration;
                packetOptions = options;
                packetGeneration = optionsGeneration;
                if (stopRequested)
                {
                    break;
                }
            }

            if (packetGeneration != activeGeneration)
            {
                // 丢弃旧选项的半帧和四面缓存，避免 ON/OFF 混合造成重影。
                hasFrame = false;
                closedFrame = false;
                for (auto &face : originFaces) face.ready = false;
                activeGeneration = packetGeneration;
            }
            if (currentMode == 1 && faceMask.load() != publishedMask && lastPublishedFace >= 0)
                publishOrigin(lastPublishedFace, currentFrame);

            // 每 5 秒输出一次接收统计（仅调试开关开启时）。
            if (debugStats)
            {
                const auto now = std::chrono::steady_clock::now();
                if (now - lastStat >= std::chrono::seconds(5))
                {
                    const std::uint64_t invalid = [this]
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        return stats.invalidPackets;
                    }();
                    const std::uint64_t frames = [this]
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        return stats.frames;
                    }();
                    std::cout << "[beta] recv=" << recvCount << " parsed=" << parseCount
                              << " invalid(total)=" << invalid << " timeouts=" << timeoutCount
                              << " frames=" << frames << std::endl;
                    recvCount = 0;
                    parseCount = 0;
                    timeoutCount = 0;
                    lastStat = now;
                }
            }

            sockaddr_in from{};
            socklen_t fromLength = sizeof(from);
            const ssize_t received = ::recvfrom(sock, buffer.data(), buffer.size(), 0,
                                                reinterpret_cast<sockaddr *>(&from), &fromLength);
            if (received <= 0)
            {
                ++timeoutCount;
                if (hasFrame)
                {
                    flushFrame();
                }
                continue;
            }

            // 只接收用户选中雷达，防止同端口上的其他设备污染帧。
            if (from.sin_addr.s_addr != expectedSource.s_addr) continue;
            ++recvCount;

            // 诊断开关：只收包不解析，用于区分 socket 限流与解析耗时。
            if (drainOnly)
            {
                if (recvCount % 1000 == 0)
                {
                    std::cout << "[beta] drain recv=" << recvCount << std::endl;
                }
                continue;
            }

            const int lockedFace = hasFrame && currentMode == 1 &&
                static_cast<std::size_t>(received) == BetaUdpStream::originPacketSize(lineCount()) &&
                readLe16(buffer.data() + 1) == currentFrame ? currentFace : -1;
            if (!parseAny(buffer.data(), static_cast<std::size_t>(received), packet,
                          packetCalibration.get(), packetOptions, lockedFace,
                          lineCount(), slotCount(), config.pitchCenter,
                          config.faceFromFrameId))
            {
                std::lock_guard<std::mutex> lock(mutex);
                ++stats.invalidPackets;
                continue;
            }
            ++parseCount;

            ++packetCounter;
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (stats.detectedMode != packet.mode)
                {
                    stats.detectedMode = packet.mode;
                }
            }

            // 满帧或超时后迟到的同帧包不能开启一个伪帧。
            if (!hasFrame && closedFrame && packet.mode == lastClosedMode &&
                packet.frame == lastClosedFrame) continue;
            if (!hasFrame || packet.mode != currentMode || packet.frame != currentFrame)
            {
                flushFrame();
                resetFrame(packet.mode, packet.frame);
            }

            if (packet.face >= 0) currentFace = packet.face;
            if (packet.mode == 0)
            {
                if (packet.index >= histogramPacketCount(lineCount()) ||
                    histSeqReceived[packet.index])
                {
                    continue;
                }
                histSeqReceived[packet.index] = 1;
                for (int pixel = 0; pixel < 4; ++pixel)
                {
                    const int line = static_cast<int>(packet.index) * 4 + pixel;
                    if (line >= lineCount())
                    {
                        break;
                    }
                    std::memcpy(&histBins[static_cast<std::size_t>(line) * kBins],
                                &packet.bins[pixel * kBins], kBins * sizeof(std::uint16_t));
                }
                ++histCount;
                if (histCount >= histogramPacketCount(lineCount()))
                {
                    flushFrame();
                }
                continue;
            }

            if (packet.index >= slotCount() ||
                packet.index >= slotReceived.size() || slotReceived[packet.index])
            {
                continue;
            }
            slotReceived[packet.index] = 1;
            ++receivedCount;
            // 同一 slot 的整列点连续写入，避免跨行随机访问。
            algo_lidar::PointData *destination =
                flatFrame.data() + static_cast<std::size_t>(packet.index) * lineCount();
            for (int line = 0; line < packet.pointCount; ++line)
            {
                destination[line] = packet.points[line];
            }
            if (receivedCount >= slotCount())
            {
                flushFrame();
            }
        }

        ::close(sock);
        std::lock_guard<std::mutex> lock(mutex);
        active = false;
#else
        std::lock_guard<std::mutex> lock(mutex);
        active = false;
#endif
    }
};

BetaUdpStream::BetaUdpStream() : impl_(std::make_unique<Impl>())
{
}

BetaUdpStream::~BetaUdpStream()
{
    stop();
}

bool BetaUdpStream::open(const Config &config, FrameSink sink, std::string &error)
{
    stop();
#ifndef _WIN32
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->config = config;
        impl_->sink = std::move(sink);
        impl_->stopRequested = false;
        impl_->active = true;
        impl_->stats = Stats{};
        impl_->hasFrame = false;
        impl_->currentMode = -1;
        impl_->flatFrame.clear();
        impl_->closedFrame = false;
        impl_->packetCounter = 0;
        impl_->publishedMask = impl_->faceMask.load();
        for (auto &face : impl_->originFaces) face = Impl::FaceFrame{};
    }
    impl_->worker = std::thread([this] { impl_->loop(); });
    return true;
#else
    (void)config;
    (void)sink;
    error = "Windows 暂不支持 CS96 Beta 直连";
    return false;
#endif
}

void BetaUdpStream::stop()
{
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->stopRequested = true;
    }
    if (impl_->worker.joinable())
    {
        impl_->worker.join();
    }
}

bool BetaUdpStream::running() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->active;
}

BetaUdpStream::Config BetaUdpStream::config() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->config;
}

BetaUdpStream::Stats BetaUdpStream::stats() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Stats result = impl_->stats;
    result.packets = impl_->packetCounter.load(std::memory_order_relaxed);
    return result;
}

void BetaUdpStream::setCalibration(std::shared_ptr<const Cs96BetaCalibration> calibration)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->calibration = std::move(calibration);
    ++impl_->optionsGeneration;
}

void BetaUdpStream::setCalibrationEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    // 水平角方案（Alpha/Beta）与四项标定开关相互独立，重置 options 时必须保留。
    const bool alphaAngle = impl_->options.alphaHorizontalAngle;
    impl_->options = enabled ? Cs96BetaCalibration::DecodeOptions{}
                             : Cs96BetaCalibration::DecodeOptions::allOff();
    impl_->options.alphaHorizontalAngle = alphaAngle;
    ++impl_->optionsGeneration;
}

bool BetaUdpStream::calibrationEnabled() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->options.any();
}

void BetaUdpStream::setDecodeOptions(const Cs96BetaCalibration::DecodeOptions &options)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->options = options;
    ++impl_->optionsGeneration;
}

Cs96BetaCalibration::DecodeOptions BetaUdpStream::decodeOptions() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->options;
}

void BetaUdpStream::setFaceMask(std::uint8_t mask)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    // 只改掩码。接收循环会对比 publishedMask 发现变化，立刻重发最近一个面。
    impl_->faceMask = (mask & 0x0F) == 0 ? 0x0F : static_cast<std::uint8_t>(mask & 0x0F);
}

std::uint8_t BetaUdpStream::faceMask() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->faceMask.load();
}

int BetaUdpStream::faceFromPa(std::uint32_t mirrorAngle)
{
    if (mirrorAngle > 0xFFFFFu) return -1;
    const auto scale = (mirrorAngle >> 12) & 0xFFu;
    if (scale > 240) return -1;
    // 协议 PA 粗计数一圈 240，细分项单位 0.0009 度；仅用于机械判面。
    // 精确编码器表/面起始角属于坐标标定，不进入本路径。
    const float angle = scale * (360.0f / 240.0f) + (mirrorAngle & 0xFFFu) * 0.0009f;
    return static_cast<int>(std::floor((angle + 45.0f) / 90.0f)) % 4;
}

bool BetaUdpStream::parsePacket(const std::uint8_t *data, std::size_t size,
                                std::uint16_t &frameIndex, std::uint16_t &slot,
                                algo_lidar::PointData *points, int &pointCount)
{
    return parsePacket(data, size, kLines, kSlots, frameIndex, slot, points, pointCount);
}

bool BetaUdpStream::parsePacket(const std::uint8_t *data, std::size_t size, int lines, int slots,
                                std::uint16_t &frameIndex, std::uint16_t &slot,
                                algo_lidar::PointData *points, int &pointCount)
{
    if (data == nullptr || lines <= 0 || lines > kMaxLines || slots <= 0 ||
        size != cartesianPacketSize(lines))
    {
        return false;
    }
    ParsedPacket packet;
    if (!parseCartesian(data, packet, lines, slots))
    {
        return false;
    }
    frameIndex = packet.frame;
    slot = packet.index;
    pointCount = packet.pointCount;
    if (points != nullptr)
    {
        for (int i = 0; i < pointCount; ++i)
        {
            points[i] = packet.points[i];
        }
    }
    return true;
}

} // namespace cloud_viewer
