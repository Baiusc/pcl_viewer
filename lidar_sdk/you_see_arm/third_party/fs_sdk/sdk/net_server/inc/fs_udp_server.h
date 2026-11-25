#ifndef __FS_UDP_SERVER_H__
#define __FS_UDP_SERVER_H__

#define UDP_MULTI_TARGET_IP "224.0.0.200"

#define UDP_DATA_TARGET_IP  "192.168.137.100"

typedef struct {
    int sockfd;
    char strTargetIp[16];
    int targetPort;
} fs_udp_param;

#ifdef __cplusplus
extern "C" {
#endif

int  fs_udp_init(fs_udp_param * udp_param, int local_port);
void fs_udp_uninit(int sockfd);
int fs_udp_send_data(fs_udp_param * udp_param, uint8_t *sendbuf, uint16_t buflen, const char *flag);

#ifdef __cplusplus
}
#endif

#endif