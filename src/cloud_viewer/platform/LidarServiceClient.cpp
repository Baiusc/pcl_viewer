#include "cloud_viewer/platform/LidarServiceClient.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace cloud_viewer
{
namespace
{

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;

void ensureSocketLibrary()
{
    static std::once_flag once;
    std::call_once(once, []
                   {
                       WSADATA data{};
                       WSAStartup(MAKEWORD(2, 2), &data);
                   });
}

void closeSocket(SocketHandle socket) { closesocket(socket); }
int socketError() { return WSAGetLastError(); }
bool wouldBlock(int error) { return error == WSAEWOULDBLOCK || error == WSAEINPROGRESS; }
#else
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;

void ensureSocketLibrary() {}
void closeSocket(SocketHandle socket) { ::close(socket); }
int socketError() { return errno; }
bool wouldBlock(int error) { return error == EINPROGRESS || error == EWOULDBLOCK || error == EAGAIN; }
#endif

struct HttpResponse
{
    int status = 0;
    std::string body;
};

/// CV_BETA_DEBUG=1 时输出 HTTP 交互失败点，便于现场定位（正常运行时静默）。
void httpDebug(const std::string &message)
{
    if (std::getenv("CV_BETA_DEBUG") != nullptr)
    {
        std::fprintf(stderr, "[lidar-http] %s\n", message.c_str());
    }
}

void setSocketTimeout(SocketHandle socket, int timeoutMs)
{
#ifdef _WIN32
    const DWORD value = static_cast<DWORD>(timeoutMs);
    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&value), sizeof(value));
    setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char *>(&value), sizeof(value));
#else
    timeval value{};
    value.tv_sec = timeoutMs / 1000;
    value.tv_usec = (timeoutMs % 1000) * 1000;
    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &value, sizeof(value));
    setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, &value, sizeof(value));
#endif
}

/// 连接完成后必须切回阻塞模式：非阻塞 socket 的 recv 会立刻返回 WSAEWOULDBLOCK，
/// SO_RCVTIMEO 对非阻塞 socket 不生效，表现为「刚连上就收到 0 字节」。
void setBlockingMode(SocketHandle socket, bool blocking)
{
#ifdef _WIN32
    u_long mode = blocking ? 0 : 1;
    ioctlsocket(socket, FIONBIO, &mode);
#else
    const int flags = fcntl(socket, F_GETFL, 0);
    fcntl(socket, F_SETFL, blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK));
#endif
}

/// 非阻塞 connect + select，避免设备掉线时卡住 UI 线程。
bool connectWithTimeout(SocketHandle socket, const sockaddr_in &address, int timeoutMs)
{
#ifdef _WIN32
    u_long nonBlocking = 1;
    ioctlsocket(socket, FIONBIO, &nonBlocking);
#else
    const int flags = fcntl(socket, F_GETFL, 0);
    fcntl(socket, F_SETFL, flags | O_NONBLOCK);
#endif
    const int rc = ::connect(socket, reinterpret_cast<const sockaddr *>(&address), sizeof(address));
    if (rc == 0)
    {
        return true;
    }
    const int connectError = socketError();
    if (!wouldBlock(connectError))
    {
        httpDebug("connect rc=" + std::to_string(rc) + " error=" + std::to_string(connectError));
        return false;
    }

    fd_set writeSet;
    FD_ZERO(&writeSet);
    FD_SET(socket, &writeSet);
    timeval timeout{};
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;
#ifdef _WIN32
    const int ready = select(0, nullptr, &writeSet, nullptr, &timeout);
#else
    const int ready = select(socket + 1, nullptr, &writeSet, nullptr, &timeout);
#endif
    if (ready <= 0)
    {
        httpDebug("select ready=" + std::to_string(ready) + " error=" +
                  std::to_string(socketError()));
        return false;
    }
    int error = 0;
#ifdef _WIN32
    int length = sizeof(error);
#else
    socklen_t length = sizeof(error);
#endif
    getsockopt(socket, SOL_SOCKET, SO_ERROR, reinterpret_cast<char *>(&error), &length);
    return error == 0;
}

/// 极简 HTTP/1.1 客户端：只支持 http + IPv4，够用且不引入 curl。
bool httpRequest(const std::string &ip, std::uint16_t port, const std::string &method,
                 const std::string &path, const std::string &body, int timeoutMs,
                 HttpResponse &response)
{
    ensureSocketLibrary();
    const SocketHandle socket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket == kInvalidSocket)
    {
        httpDebug("socket() failed, error=" + std::to_string(socketError()));
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1)
    {
        httpDebug("inet_pton failed: " + ip);
        closeSocket(socket);
        return false;
    }
    if (!connectWithTimeout(socket, address, timeoutMs))
    {
        httpDebug("connect failed: " + ip + ":" + std::to_string(port) +
                  " error=" + std::to_string(socketError()));
        closeSocket(socket);
        return false;
    }
    setBlockingMode(socket, true);
    setSocketTimeout(socket, timeoutMs);

    std::ostringstream header;
    header << method << " " << path << " HTTP/1.1\r\n"
           << "Host: " << ip << "\r\n"
           << "Accept: application/json\r\n"
           << "Connection: close\r\n";
    if (!body.empty())
    {
        header << "Content-Type: application/json\r\n"
               << "Content-Length: " << body.size() << "\r\n";
    }
    header << "\r\n" << body;
    const std::string request = header.str();

    std::size_t sent = 0;
    while (sent < request.size())
    {
        const int chunk = ::send(socket, request.data() + sent,
                                 static_cast<int>(request.size() - sent), 0);
        if (chunk <= 0)
        {
            httpDebug("send failed, error=" + std::to_string(socketError()));
            closeSocket(socket);
            return false;
        }
        sent += static_cast<std::size_t>(chunk);
    }

    std::string raw;
    char buffer[2048];
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeoutMs + 500);
    while (raw.size() < 256 * 1024)
    {
        const int received = ::recv(socket, buffer, static_cast<int>(sizeof(buffer)), 0);
        if (received <= 0)
        {
            break;
        }
        raw.append(buffer, static_cast<std::size_t>(received));
        if (std::chrono::steady_clock::now() > deadline)
        {
            break;
        }
    }
    closeSocket(socket);

    const std::size_t split = raw.find("\r\n\r\n");
    if (split == std::string::npos)
    {
        httpDebug("no header terminator, bytes=" + std::to_string(raw.size()));
        return false;
    }
    const std::string head = raw.substr(0, split);
    std::string payload = raw.substr(split + 4);

    const std::size_t space = head.find(' ');
    if (space != std::string::npos)
    {
        response.status = std::atoi(head.c_str() + space + 1);
    }
    // lighttpd 对 CGI 可能用 chunked；不处理会被 JSON 里的长度前缀干扰解析。
    std::string lowerHead;
    lowerHead.reserve(head.size());
    std::transform(head.begin(), head.end(), std::back_inserter(lowerHead),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (lowerHead.find("transfer-encoding: chunked") != std::string::npos)
    {
        std::string decoded;
        std::size_t cursor = 0;
        while (cursor < payload.size())
        {
            const std::size_t lineEnd = payload.find("\r\n", cursor);
            if (lineEnd == std::string::npos)
            {
                break;
            }
            const long length = std::strtol(payload.c_str() + cursor, nullptr, 16);
            if (length <= 0)
            {
                break;
            }
            cursor = lineEnd + 2;
            if (cursor + static_cast<std::size_t>(length) > payload.size())
            {
                decoded.append(payload, cursor, std::string::npos);
                break;
            }
            decoded.append(payload, cursor, static_cast<std::size_t>(length));
            cursor += static_cast<std::size_t>(length) + 2;
        }
        payload.swap(decoded);
    }
    response.body.swap(payload);
    if (response.status == 0)
    {
        httpDebug("unparsable status line: " + head.substr(0, 60));
    }
    return response.status != 0;
}

/// 定位 `"key"` 后的值起点；找不到返回 nullptr。
const char *jsonValue(const std::string &json, const std::string &key)
{
    const std::string needle = "\"" + key + "\"";
    std::size_t position = json.find(needle);
    while (position != std::string::npos)
    {
        std::size_t cursor = position + needle.size();
        while (cursor < json.size() && std::isspace(static_cast<unsigned char>(json[cursor])))
        {
            ++cursor;
        }
        if (cursor < json.size() && json[cursor] == ':')
        {
            ++cursor;
            while (cursor < json.size() && std::isspace(static_cast<unsigned char>(json[cursor])))
            {
                ++cursor;
            }
            return json.c_str() + cursor;
        }
        position = json.find(needle, position + needle.size());
    }
    return nullptr;
}

bool jsonInt(const std::string &json, const std::string &key, int &out)
{
    const char *value = jsonValue(json, key);
    if (value == nullptr)
    {
        return false;
    }
    if (*value == '-' || std::isdigit(static_cast<unsigned char>(*value)))
    {
        out = std::atoi(value);
        return true;
    }
    return false;
}

std::string jsonString(const std::string &json, const std::string &key)
{
    const char *value = jsonValue(json, key);
    if (value == nullptr || *value != '"')
    {
        return {};
    }
    std::string out;
    for (const char *cursor = value + 1; *cursor != '\0' && *cursor != '"'; ++cursor)
    {
        if (*cursor == '\\' && cursor[1] != '\0')
        {
            ++cursor;
            switch (*cursor)
            {
            case 'n':
                out.push_back('\n');
                break;
            case 't':
                out.push_back('\t');
                break;
            case 'r':
                out.push_back('\r');
                break;
            default:
                out.push_back(*cursor);
                break;
            }
            continue;
        }
        out.push_back(*cursor);
    }
    return out;
}

std::vector<int> jsonIntArray(const std::string &json, const std::string &key)
{
    std::vector<int> out;
    const char *value = jsonValue(json, key);
    if (value == nullptr || *value != '[')
    {
        return out;
    }
    const char *cursor = value + 1;
    while (*cursor != '\0' && *cursor != ']')
    {
        if (*cursor == '-' || std::isdigit(static_cast<unsigned char>(*cursor)))
        {
            char *end = nullptr;
            const long number = std::strtol(cursor, &end, 10);
            if (end != cursor)
            {
                out.push_back(static_cast<int>(number));
                cursor = end;
                continue;
            }
        }
        ++cursor;
    }
    return out;
}

bool parseStatus(const std::string &json, LidarServiceStatus &status)
{
    int code = 0;
    if (!jsonInt(json, "code", code))
    {
        return false;
    }
    // 旧固件的 web 服务器上可能也有其它返回 {"code":N} 的 CGI；只认带 mgrd 特征
    // 字段的响应，避免把旧固件误判成新固件（误判会导致改配置/重启全走错分支）。
    const bool mgrdShape = json.find("\"phase\"") != std::string::npos ||
                           json.find("\"model\"") != std::string::npos ||
                           json.find("\"actual_mode\"") != std::string::npos ||
                           json.find("\"validated_work_modes\"") != std::string::npos;
    if (!mgrdShape)
    {
        return false;
    }
    status.code = code;
    status.model = jsonString(json, "model");
    status.phase = jsonString(json, "phase");
    status.error = jsonString(json, "error");
    jsonInt(json, "confirmed_mode", status.confirmedMode);
    jsonInt(json, "actual_mode", status.actualMode);
    jsonInt(json, "desired_mode", status.desiredMode);
    jsonInt(json, "pid", status.pid);
    status.validatedWorkModes = jsonIntArray(json, "validated_work_modes");
    status.reachable = true;
    return true;
}

/// 轮询到目标状态，或超时。
bool pollUntilCommitted(const std::string &ip, std::uint16_t port, int expectMode,
                        int timeoutMs, LidarServiceStatus &status, std::string &error)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    std::string lastPhase;
    while (std::chrono::steady_clock::now() < deadline)
    {
        LidarServiceStatus current;
        if (queryLidarService(ip, port, current, 1200))
        {
            status = current;
            lastPhase = current.phase;
            if (!current.error.empty())
            {
                error = "设备报告错误: " + current.error;
                return false;
            }
            if (current.committed() && current.actualMode == expectMode &&
                current.confirmedMode == expectMode)
            {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
    error = "等待设备确认超时（最后阶段 " + (lastPhase.empty() ? std::string("unknown") : lastPhase) +
            "）";
    return false;
}

} // namespace

bool LidarServiceStatus::supports(int mode) const
{
    return std::find(validatedWorkModes.begin(), validatedWorkModes.end(), mode) !=
           validatedWorkModes.end();
}

std::string LidarServiceStatus::describe() const
{
    if (!reachable)
    {
        return "未发现 lidar_mgrd 管理接口（旧固件）";
    }
    std::ostringstream text;
    text << (model.empty() ? std::string("lidar") : model) << " / lidar_mgrd，phase="
         << (phase.empty() ? std::string("unknown") : phase) << "，mode=";
    if (actualMode >= 0)
    {
        text << actualMode;
    }
    else
    {
        text << "切换中(" << desiredMode << ")";
    }
    if (!validatedWorkModes.empty())
    {
        text << "，可用 ";
        for (std::size_t i = 0; i < validatedWorkModes.size(); ++i)
        {
            text << (i == 0 ? "" : "/") << validatedWorkModes[i];
        }
    }
    if (!error.empty())
    {
        text << "，error=" << error;
    }
    return text.str();
}

std::uint8_t lidarServiceModeMask(const LidarServiceStatus &status)
{
    std::uint8_t mask = 0;
    for (int mode : status.validatedWorkModes)
    {
        if (mode >= 0 && mode < 8)
        {
            mask = static_cast<std::uint8_t>(mask | (1u << mode));
        }
    }
    // 新固件至少支持原始/真实点云；接口异常时不要把所有模式都禁掉。
    return mask != 0 ? mask : static_cast<std::uint8_t>((1u << 1) | (1u << 2));
}

std::uint16_t lidarServiceHttpPort()
{
    if (const char *configured = std::getenv("CV_BETA_HTTP_PORT"))
    {
        const int value = std::atoi(configured);
        if (value > 0 && value < 65536)
        {
            return static_cast<std::uint16_t>(value);
        }
    }
    return 80;
}

bool queryLidarService(const std::string &ip, std::uint16_t httpPort,
                       LidarServiceStatus &status, int timeoutMs)
{
    status = LidarServiceStatus{};
    // 实测（CS96 beta + Windows 有线直连）：链路空闲十几秒后的第一个 TCP 请求
    // 稳定被丢掉 —— 首次 connect 超时，紧接着再发一次只要十几毫秒就成功。
    // 成因是邻居表/ARP 需要重新解析、或设备 PHY 从省电态唤醒，属于链路层现象，
    // 与设备是否在线无关。因此传输层失败必须自带重试，否则「连接设备」「切换模式」
    // 会随机报「管理接口不可达」，用户重试一次又好了。
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        if (attempt > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        HttpResponse response;
        if (!httpRequest(ip, httpPort, "GET", "/cgi-bin/lidar_status.cgi", {}, timeoutMs, response))
        {
            httpDebug("status attempt " + std::to_string(attempt + 1) + " transport failed");
            continue;
        }
        if (!parseStatus(response.body, status))
        {
            // 旧固件的 web 是另一套接口，返回 HTML 或 404，这里视为「没有管理接口」。
            // 这是设备的明确回答，重试无意义，直接返回。
            httpDebug("status json not recognised: " + response.body.substr(0, 120));
            return false;
        }
        // code != 200 时仍然是新固件，只是当前无法给出状态（比如 mgrd 未起来）。
        return true;
    }
    return false;
}

bool switchLidarServiceMode(const std::string &ip, std::uint16_t httpPort, int mode,
                            LidarServiceStatus &finalStatus, std::string &error,
                            int acceptTimeoutMs, int commitTimeoutMs)
{
    if (mode < 0 || mode > 4)
    {
        error = "work_mode 超出范围（0~4）";
        return false;
    }

    // 接口只接受 {"work_mode":N}（不要额外字段，dev 侧会按严格的 JSON 解析拒绝）。
    const std::string body = "{\"work_mode\":" + std::to_string(mode) + "}";
    HttpResponse response;
    bool accepted = false;
    for (int attempt = 0; attempt < 3 && !accepted; ++attempt)
    {
        if (attempt > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            // 冷链路丢包后重发前，先确认上一次请求是否其实已被受理（只是响应丢了），
            // 避免同一次点击触发两轮切换。
            LidarServiceStatus probe;
            if (queryLidarService(ip, httpPort, probe, 1500) && probe.desiredMode == mode &&
                probe.error.empty())
            {
                finalStatus = probe;
                return pollUntilCommitted(ip, httpPort, mode, commitTimeoutMs, finalStatus, error);
            }
        }
        accepted =
            httpRequest(ip, httpPort, "POST", "/cgi-bin/lidar_switch.cgi", body, acceptTimeoutMs,
                        response);
        if (!accepted)
        {
            httpDebug("switch attempt " + std::to_string(attempt + 1) + " transport failed");
        }
    }
    if (!accepted)
    {
        error = "无法访问设备管理接口（" + ip + "，已重试 3 次）";
        return false;
    }

    LidarServiceStatus status;
    const bool parsed = parseStatus(response.body, status);
    if (parsed && status.code != 200 && status.code != 202)
    {
        error = status.error.empty() ? ("设备拒绝切换（code=" + std::to_string(status.code) + "）")
                                     : status.error;
        return false;
    }
    if (!parsed)
    {
        error = "设备返回无法解析的响应: " + response.body.substr(0, 120);
        return false;
    }

    // 202 = 已受理，mgrd 会停采、改模式、重启 you_see，再进入 COMMITTED。
    return pollUntilCommitted(ip, httpPort, mode, commitTimeoutMs, finalStatus, error);
}

} // namespace cloud_viewer
