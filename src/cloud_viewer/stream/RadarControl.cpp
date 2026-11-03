#include "cloud_viewer/stream/RadarControl.h"
#include <array>
#include <chrono>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#endif
namespace cloud_viewer {
std::string setRadarOutputMode(const std::string& ip, std::uint16_t port, RadarOutputMode mode) {
#ifdef _WIN32
    WSADATA ws{}; if(WSAStartup(MAKEWORD(2,2),&ws)) return "WSAStartup failed";
    const SOCKET fd=socket(AF_INET,SOCK_STREAM,0);
    struct Guard { SOCKET fd; ~Guard(){ closesocket(fd); WSACleanup(); } } guard{fd};
    if(fd==INVALID_SOCKET) return "TCP socket failed";
    u_long nonblocking=1; ioctlsocket(fd,FIONBIO,&nonblocking);
#else
    const int fd=socket(AF_INET,SOCK_STREAM,0);
    struct Guard { int fd; ~Guard(){ if(fd>=0) close(fd); } } guard{fd};
    if(fd<0) return "TCP socket failed";
    fcntl(fd,F_SETFL,fcntl(fd,F_GETFL,0)|O_NONBLOCK);
#endif
    sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_port=htons(port);
    if(inet_pton(AF_INET,ip.c_str(),&addr.sin_addr)!=1) return "invalid radar IP";
    const int connected=connect(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr));
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(6);
    auto wait=[&](bool writing) {
        const auto left=std::chrono::duration_cast<std::chrono::microseconds>(deadline-std::chrono::steady_clock::now()).count();
        if(left<=0) return false;
        fd_set fds; FD_ZERO(&fds); FD_SET(fd,&fds);
        timeval tv{}; tv.tv_sec=static_cast<long>(left/1000000); tv.tv_usec=static_cast<long>(left%1000000);
        return select(static_cast<int>(fd)+1,writing?nullptr:&fds,writing?&fds:nullptr,nullptr,&tv)>0;
    };
    if(connected!=0) {
        if(!wait(true)) return "TCP control connect timeout";
        int error=0;
#ifdef _WIN32
        int len=sizeof(error);
#else
        socklen_t len=sizeof(error);
#endif
        getsockopt(fd,SOL_SOCKET,SO_ERROR,reinterpret_cast<char*>(&error),&len);
        if(error) return "TCP control connection failed";
    }
    // protocol_head_t(6) + control_data_t(10) + uint32 work_status。
    std::array<unsigned char,20> request{{0x88,0x77,0,0,0,0,0xaa,0x99,2,0,0,0,4,0,0,0,0,0,0,0}};
    request[16]=mode==RadarOutputMode::Cloud?0x21:0x2a;
    std::size_t sent=0;
    while(sent<request.size()) {
        if(!wait(true)) return "TCP send timeout";
#ifdef _WIN32
        const int n=send(fd,reinterpret_cast<const char*>(request.data()+sent),static_cast<int>(request.size()-sent),0);
#else
        const auto n=send(fd,request.data()+sent,request.size()-sent,MSG_NOSIGNAL);
#endif
        if(n<=0) return "TCP send failed"; sent+=static_cast<std::size_t>(n);
    }
    std::array<unsigned char,16> ack{};std::size_t received=0;
    while(received<ack.size()) {
        if(!wait(false)) return "TCP ACK timeout";
        const auto n=recv(fd,reinterpret_cast<char*>(ack.data()+received),static_cast<int>(ack.size()-received),0);
        if(n<=0) return "TCP ACK truncated"; received+=static_cast<std::size_t>(n);
    }
    if(ack[0]!=0x88||ack[1]!=0x77||ack[6]!=0xaa||ack[7]!=0x99||ack[8]!=2||ack[9]!=0)
        return "invalid mode ACK";
    if(ack[10]||ack[11]) return "device rejected mode switch";
    if(ack[12]||ack[13]||ack[14]||ack[15]) return "unexpected mode ACK payload";
    return {};
}
}
