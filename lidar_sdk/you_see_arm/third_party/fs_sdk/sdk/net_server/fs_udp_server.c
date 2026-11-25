#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>

#include "fs_err.h"
#include "fs_hw.h"
#include "fs_udp_server.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "UdpServer"

void fs_udp_uninit(int sockfd)
{
    if (sockfd > 0) {
        close(sockfd);
        sockfd = -1;
    }
}

int fs_udp_init(fs_udp_param* udp_param, int local_port)
{
    struct sockaddr_in server_addr;
    int ret = FS_SUCCESS;

    if (udp_param->sockfd > 0) {
        close(udp_param->sockfd);
        udp_param->sockfd = -1;
    }

    if (udp_param->sockfd == -1) {
        udp_param->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    }

    if (local_port == 0) {
        FS_LOGE("localPort is null ");
        close(udp_param->sockfd);
        udp_param->sockfd = -1;
        return FS_ERR_LOCAL_PORT_NULL;
    }

    // 初始化服务器地址结构
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(local_port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    FS_LOGI("bind port : %d", local_port);

    // int option = 1;
    int flags = fcntl(udp_param->sockfd, F_GETFL, 0);
    if (flags < 0) {
        FS_LOGE("Cannot get file flags");
        close(udp_param->sockfd);
        return FS_ERR_BAD_SOCKET;
    }

    // 设置为非阻塞模式
    if (fcntl(udp_param->sockfd, F_SETFL, flags | O_NONBLOCK) < 0) {
        FS_LOGE("Cannot set file flags to non-blocking");
        close(udp_param->sockfd);
        return FS_ERR_BAD_SOCKET;
    }

    int option = 1;
    if (setsockopt(udp_param->sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option)) < 0) {
        FS_LOGE("setsockopt SO_REUSEADDR failed");
    }

    ret = bind(udp_param->sockfd, (struct sockaddr*)(&server_addr), sizeof(struct sockaddr_in));
    if (ret < 0) {
        close(udp_param->sockfd);
        udp_param->sockfd = -1;
        FS_LOGE("bind ret = %d", ret);
        return FS_ERR_BIND_FAILD;
    }

    return FS_SUCCESS;
}

int fs_udp_send_data(fs_udp_param* udp_param, uint8_t* sendbuf, uint16_t buflen, const char* flag)
{
    struct sockaddr_in client_addr;
    ssize_t sent_len = 0;
    static uint32_t retry_count = 0;
    static uint32_t eagain_count = 0;

    (void)(flag); // 目前未使用该参数

    socklen_t addr_len = sizeof(client_addr);

    (void)flag;
    if (udp_param->sockfd < 0) {
        FS_LOGE("FS_ERR_BAD_SOCKET");
        return FS_ERR_BAD_SOCKET;
    }

    if (0) {
        /*todo: 点云是否设置组播模式 */
    } else {
        client_addr.sin_family = AF_INET;
        client_addr.sin_port = htons(udp_param->targetPort);
        client_addr.sin_addr.s_addr = inet_addr(udp_param->strTargetIp);
    }

    // if (buflen > 1000) {
    //     static int last_len;
    //     if (flag != NULL && last_len != buflen && last_len != 0) {
    //         FS_LOGD("%s %d---->>%d", flag, last_len, buflen);
    //     }
    //     last_len = buflen;
    // }
    // FS_LOGI("udp send to %s:%d, len=%d", udp_param->strTargetIp, udp_param->targetPort, buflen);

    // 发送数据，支持 EAGAIN 重试
    for (int attempt = 0; attempt < 2; attempt++) {
        sent_len = sendto(udp_param->sockfd, sendbuf, buflen, 0,
                         (struct sockaddr*)&client_addr, addr_len);

        if (sent_len == buflen) {
            if (retry_count > 0) {
                // FS_LOGD("sendto success after %u retries, buflen=%u", retry_count, buflen);
                retry_count = 0;
            }
            return FS_SUCCESS;
        }

        // 非阻塞模式下 EAGAIN 表示缓冲区满
        if (sent_len < 0 && errno == EAGAIN) {
            eagain_count++;
            retry_count++;
            if (attempt == 0) {
                fs_hw_usleep(100); // 等 100us 后重试
                continue;
            } else {
                if (retry_count % 100 == 0) {
                    FS_LOGW("sendto EAGAIN after retry (count=%u), buflen=%u, total_eagain=%u",
                           retry_count, buflen, eagain_count);
                }
                return FS_ERR_IO;
            }
        }

        break;
    }

    FS_LOGE("sendto failed, sent_len=%zd, expected=%u, errno=%d", sent_len, buflen, errno);
    return FS_ERR_IO;
}
