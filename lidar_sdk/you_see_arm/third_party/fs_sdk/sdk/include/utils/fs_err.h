#ifndef __FS_ERR_H__
#define __FS_ERR_H__

#include "fs_log.h"

#define RETURN_ON_ERR(expr)                           \
    do {                                              \
        int __err = (expr);                           \
        if (__err) {                                  \
            FS_LOGE("'%s'.", fs_err_strerror(__err)); \
            return (__err);                           \
        }                                             \
    }                                                 \
    while (0)
    /* end of RETURN_ON_ERR */

#define BREAK_ON_ERR(expr)                            \
    {                                                 \
        int __err = (expr);                           \
        if (__err) {                                  \
            FS_LOGE("'%s'.", fs_err_strerror(__err)); \
            break;                                    \
        }                                             \
    }                                                 \
    /* end of BREAK_ON_ERR */

#define CONTINUE_ON_ERR(expr)                         \
    {                                                 \
        int __err = (expr);                           \
        if (__err) {                                  \
            FS_LOGE("'%s'.", fs_err_strerror(__err)); \
            continue;                                 \
        }                                             \
    }
    /* end of CONTINUE_ON_ERR */

#define CHECK_ERR(expr)                               \
    do {                                              \
        int __err = (expr);                           \
        if (__err) {                                  \
            FS_LOGE("'%s'.", fs_err_strerror(__err)); \
            return (__err);                           \
        }                                             \
    }                                                 \
    while (0)

    /* end of CHECK_ERR */

#define CHECK_PTR(ptr)                        \
    do {                                      \
        if ((ptr) == NULL) {                  \
            FS_LOGE("ptr(%s) is NULL", #ptr); \
            return FS_ERR_NULL_PTR;           \
        }                                     \
    }                                         \
    while (0)
/* end of CHECK_PTR */


typedef enum {
    FS_SUCCESS        = 0,
    /* Generic error. */
    FS_ERR_INTERNAL   = -1,
    /* Base on unix errno. */
    FS_ERR_NOT_FOUND  = -2,
    FS_ERR_IO         = -5,
    FS_ERR_AGAIN      = -11,
    FS_ERR_NOMEM      = -12,
    FS_ERR_BUSY       = -16,
    FS_ERR_BAD_PARAMS = -22,
    FS_ERR_TIME_OUT   = -62,
    /* Common error. */
    FS_ERR_NULL_PTR     = -100,
    FS_ERR_BUF_OVERFLOW = -101,
    FS_ERR_BAD_PROTOCOL = -102,
    FS_ERR_SPI_READ     = -103,
    FS_ERR_SPI_WRITE    = -104,
    FS_ERR_NO_DEVICE    = -105,
    FS_ERR_SPI_CRC      = -106,
    FS_ERR_I2C_READ     = -107,
    FS_ERR_I2C_WRITE    = -108,
    FS_ERR_I2C_CRC      = -109,
    FS_ERR_UNSUPPORT    = -110,
    FS_ERR_CHECK_CRC    = -111,
    FS_ERR_SLOT_CHANGE  = -112,    // slot 边界检测，调用方应取走数据
    FS_ERR_GROUP_CHANGE = -113,    // group 边界检测，调用方应取走数据
    FS_ERR_FRAME_CHANGE = -114,    // frame 边界检测，调用方应处理完当前 frame
    FS_ERR_LOCAL_PORT_NULL    = -200,
    FS_ERR_TARGET_IPPORT_NULL = -202,
    FS_ERR_BIND_FAILD         = -203,
    FS_ERR_SHAKE_HAND_FAILD   = -204,
    FS_ERR_BAD_SOCKET         = -205,
    FS_ERR_RECVFROM_FAIL      = -206,
    FS_ERR_SENDTO_FAIL        = -207,
    FS_ERR_LISTEN_FAIL        = -208,
    FS_ERR_PROTOCOL_CRC8   = -300,
    FS_ERR_PROTOCOL_HEAD   = -301,
    FS_ERR_PROTOCOL_NO_CMD = -302,
    FS_ERR_PROTOCOL_DO_CMD = -303,
    FS_ERR_JSON_PARSE      = -304,    // JSON 解析错误
    FS_ERR_JSON_FORMAT     = -305,    // JSON 格式错误
    FS_ERR_INPUT = -404,
    FS_ERR_FPGA_TEMP   = -500,
    FS_ERR_CHECK_INIT  = -501,
    FS_ERR_FPGA_REG_WR = -502,
    FS_ERR_MCU_PROT_ERR     = -600,
    FS_ERR_MCU_PROT_READ    = -601,
    FS_ERR_MCU_PROT_WRITE   = -602,
    FS_ERR_MCU_PROT_CRC     = -603,
    FS_ERR_MCU_PROT_TIMEOUT = -604,
} fs_err_t;

#ifdef __cplusplus
extern "C" {
#endif

const char *fs_err_strerror(int err);

#ifdef __cplusplus
}
#endif

#endif /* __FS_ERR_H__ */
