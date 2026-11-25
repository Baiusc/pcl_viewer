/**
 * log.c, REE side implement.
 *
 *
 *
**/

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <syslog.h>
#include <sys/syscall.h>   //Linux system call for thread id
#include "utils/fs_log.h"
#include "utils/fs_err.h"
#include "utils/fs_version.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "fs-version"

fs_version_t g_version = {
    .framework = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .hw_ver = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .kernel_ver = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .boot_fpga = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .app_fpga = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .app_daemon = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .app_ota = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .app_soc = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .app_mcu = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .algo_ver = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .spad_sn = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .fpga_sn = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .soc_sn = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .mcu_sn = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .tx_lo_sn = {
        {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
        {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    },
    .tx_ref = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .rx_ref = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .tec_ref = {'u', 'n', 'k', 'n', 'o', 'w', 'u', '\0'},
    .wafer = 0,
    .txver = 0,
};

int fs_get_version(char *ver, uint32_t len)
{
    int ver_len = 0;
    if (ver != NULL) {
        //FS_LOGD("tec_ref len %d %d", len, strlen(g_version.tec_ref));
        ver_len = snprintf(ver, len,
                           "framework:%s %s %s\n"
                           "hardware_ver:%s\n"
                           "kernel_ver:%s\n"
                           "boot_fpga:%s\n"
                           "app_fpga:%s\n"
                           "app_daemon:%s\n"
                           "app_ota:%s\n"
                           "app_soc:%s\n"
                           "app_mcu:%s\n"
                           "algo_ver:%s\n"
                           "spad_sn:%s\n"
                           "fpga_sn:%s\n"
                           "tx_lo_sn:[%s, %s]\n"
                           "tx_ref:%s\n"
                           "rx_ref:%s\n"
                           "tec_ref:%s\n"
                           /*"soc_sn:%s\n"
                           "mcu_sn:%s\n"*/,
                           g_version.framework,
                           __DATE__, __TIME__,
                           g_version.hw_ver,
                           g_version.kernel_ver,
                           g_version.boot_fpga,
                           g_version.app_fpga,
                           g_version.app_daemon,
                           g_version.app_ota,
                           g_version.app_soc,
                           g_version.app_mcu,
                           g_version.algo_ver,
                           g_version.spad_sn,
                           g_version.fpga_sn,
                           g_version.tx_lo_sn[0],
                           g_version.tx_lo_sn[1],
                           g_version.tx_ref,
                           g_version.rx_ref,
                           g_version.tec_ref
                           /*,
                           g_version.soc_sn,
                           g_version.mcu_sn*/);
        // FS_LOGD("ver_len %d, size_t %d", ver_len, sizeof(fs_version_t));
    }
    // FS_LOGI("\nver_len %d %lu, version:\n %s \n", ver_len, sizeof(fs_version_t), (char *)ver);
    return ver_len;
}

void fs_get_kernel_version(void)
{
    FILE *fd = NULL;
    char buff[256];
    memset(buff, 0, sizeof(buff));
    fd = fopen("/proc/version", "rb");
    if (fd) {
        fread(buff, 1, sizeof(buff), fd);
        fclose(fd);
        buff[sizeof(buff) - 1] = '\0';
        // FS_LOGD("/proc/version: %s", buff);
        char *p = strtok(buff, " ");
        char *kernel_ver = NULL;
        while ((p = strtok(NULL, " ")) != NULL) { // 使用第一个参数为NULL来提取子串
            kernel_ver = p;
        }
        snprintf(g_version.kernel_ver, sizeof(g_version.kernel_ver), "%s", kernel_ver);
        // FS_LOGD("kernel_ver %s", kernel_ver);
    }
    else {
        FS_LOGE("open /proc/version");
    }
}

