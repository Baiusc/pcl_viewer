#ifndef __FS_VERSION_H__
#define __FS_VERSION_H__

#include <stdint.h>

#define MCU_FW_VERSION           "v0.6.0"
#define SOC_APP_VERSION          "v0.6.0"
#define SOC_SDK_VERSION          SOC_APP_VERSION

#ifndef GIT_VERSION
#define GIT_VERSION              "00000000-00000000"
#endif

#ifndef GIT_DATE
#define GIT_DATE                 "20250101-000000"
#endif

#define FS_VERSION_MAX_LEN   32

typedef struct {
    char framework  [FS_VERSION_MAX_LEN];
    char hw_ver     [FS_VERSION_MAX_LEN * 2];
    char kernel_ver [FS_VERSION_MAX_LEN];
    char boot_fpga  [FS_VERSION_MAX_LEN];
    char app_fpga   [FS_VERSION_MAX_LEN];
    char app_daemon [FS_VERSION_MAX_LEN];
    char app_ota    [FS_VERSION_MAX_LEN];
    char app_soc    [FS_VERSION_MAX_LEN];
    char app_mcu    [FS_VERSION_MAX_LEN * 2];
    char algo_ver   [FS_VERSION_MAX_LEN];
    char spad_sn    [FS_VERSION_MAX_LEN * 2];
    char fpga_sn    [FS_VERSION_MAX_LEN];
    char soc_sn     [FS_VERSION_MAX_LEN];
    char mcu_sn     [FS_VERSION_MAX_LEN];
    char tx_lo_sn   [2][FS_VERSION_MAX_LEN * 2];
    char tx_ref     [FS_VERSION_MAX_LEN * 4];
    char rx_ref     [FS_VERSION_MAX_LEN * 4];
    char tec_ref    [FS_VERSION_MAX_LEN * 4];
    uint8_t wafer;
    uint8_t txver;
} __attribute__((__packed__)) fs_version_t;
int fs_get_version(char *ver, uint32_t len);
void fs_get_kernel_version(void);

extern fs_version_t g_version;

#ifndef __FS_GIT_COMMIT
#define __FS_GIT_COMMIT "unknown"
#endif


#endif /* __FS_VERSION_H__ */
