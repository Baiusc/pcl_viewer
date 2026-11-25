// #include <pthread.h>
// #include <stdlib.h>
// #include <string.h>
#include <stdio.h>
// #include <stdint.h>
// #include <stdbool.h>
// #include <fcntl.h>
#include <unistd.h>
// #include <errno.h>
// #include <sys/ioctl.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// // #include <termios.h>
// #include <sys/types.h>
// #include <sys/stat.h>
#include <time.h>
#include <sys/time.h>

#include "utils/fs_util.h"
#include "utils/fs_err.h"
#include "utils/fs_log.h"


// #include <arpa/inet.h>

// #include <ifaddrs.h>

// #include <sys/socket.h>
// #include <netinet/in.h>

// #include <net/if.h>
// #include <netinet/ip.h>
// #include <netinet/ether.h>
// #include <net/if_arp.h>

// #include <net/route.h>


#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "com.util"

#define __is_print(ch)                ((unsigned int)((ch) - ' ') < 127u - ' ')
#define HEXDUMP_WIDTH                 16

uint8_t fs_util_crc8(const void *data, uint32_t dlen, uint8_t init)
{
#define CRC8_POLYNOMIAL 0x31
    uint8_t* buf = (uint8_t*)data;
    unsigned char usResult = init;
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned char usVal;

    for (i = 0; i < dlen; i++) {
        usVal = buf[i];

        for (j = 0; j < 8; j++) {
            if (usResult & 0x80) {
                usResult = (usResult << 1) | 0x01;
                usResult ^= 0x30;

                if (usVal & 0x80) {
                    usResult ^= CRC8_POLYNOMIAL;
                }
            } else {
                usResult <<= 1;

                if (usVal & 0x80) {
                    usResult ^= CRC8_POLYNOMIAL;
                }
            }

            usVal <<= 1;
        }
    }

    return usResult;
}

uint8_t fs_util_crc_8(const void *data, uint32_t dlen, uint8_t init)
{
    return (uint8_t)fs_util_crc_16(data, dlen, init);
}

uint16_t fs_util_crc_16(const void *data, uint32_t dlen, uint16_t init)
{
#define CRC_POLYNOMIAL 0x8005
    uint8_t *buf = (uint8_t *)data;
    uint16_t usResult = init;
    uint32_t i, j;

    for (i = 0; i < dlen; i++) {
        usResult ^= buf[i] << 8;

        for (j = 0; j < 8; j++) {
            if (usResult & 0x8000) {
                usResult = (unsigned short)((usResult << 1) ^ CRC_POLYNOMIAL);
            }
            else {
                usResult <<= 1;
            }
        }
    }

    return usResult;
}

int32_t fs_util_u32_swap_endian_buf(const void *in_buf, int32_t in_len, uint32_t *out_buf)
{
    int32_t i;
    int32_t out_cnt = in_len / sizeof(*out_buf);

    if ((in_buf == NULL) || (out_buf == NULL) || (in_len < 0) || (in_len % sizeof(*out_buf) != 0)) {
        return FS_ERR_NULL_PTR;
    }

    for (i = 0; i < out_cnt; i++) {
        out_buf[i] = u32_swap_endian(((uint32_t *)in_buf)[i]);
    }

    return out_cnt;
}

uint64_t fs_util_timestamp_ms(void)
{
#if 1
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
#else
    struct timespec tv;
    clock_gettime(CLOCK_REALTIME, &tv);
    return tv.tv_sec * 1000 + tv.tv_nsec / 1000000;
#endif
}

void fs_dump_hex(const uint8_t *data, uint32_t len, const char *name, uint8_t mode)
{
    uint32_t i, j;
    char buf[11 + HEXDUMP_WIDTH * 4 + 1];
    if ((data == NULL) || (len == 0)) {
        return;
    }
    if (mode == 0) {
        // FS_LOGD(" Offset(h) \033[31m00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F \033[0m\033[32m%s\033[0m", (name != NULL) ? name : "unknown");
        FS_LOGD(" Offset(h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F %s", (name != NULL) ? name : "unknown");
        // FS_LOGD("---------------------------------------------------------------------------");
    }
    else {
        printf(" Offset(h) \033[31m00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F \033[0m\033[32m%s\033[0m\n", (name != NULL) ? name : "unknown");
        // printf("---------------------------------------------------------------------------\n");
    }
    for (i = 0; i < len; i += HEXDUMP_WIDTH) {
        uint32_t n, m;
        n  = 0;
        m  = snprintf(&buf[n], sizeof(buf) - n, "[%08X] ", i);
        if (m > sizeof(buf) - n) {
            m = sizeof(buf) - n;
        }
        n += m;
        /* dump hex */
        for (j = 0; j < HEXDUMP_WIDTH; j++) {
            if (i + j < len) {
                m  = snprintf(&buf[n], sizeof(buf) - n, "%02X ", data[i + j]);
                if (m > sizeof(buf) - n) {
                    m = sizeof(buf) - n;
                }
                n += m;
            }
            else {
                m  = snprintf(&buf[n], sizeof(buf) - n, "   ");
                if (m > sizeof(buf) - n) {
                    m = sizeof(buf) - n;
                }
                n += m;
            }
        }
        /* dump char for hex */
        for (j = 0; j < HEXDUMP_WIDTH; j++) {
            if (i + j < len) {
                m  = snprintf(&buf[n], sizeof(buf) - n, "%c", __is_print(data[i + j]) ? data[i + j] : '.');
                if (m > sizeof(buf) - n) {
                    m = sizeof(buf) - n;
                }
                n += m;
            }
        }
        if (mode == 0) {
            FS_LOGD("%s", buf);
        }
        else {
            printf("%s\n", buf);
        }
    }
    return;
}

void fs_dump_hex32(const uint32_t* data, uint32_t len, const char* name, uint8_t mode)
{
    UNUSED_VAR(name);
    uint32_t count = len / 8;
    len = len % 8;

    if (mode == 0) {
        for (uint32_t i = 0; i < count; i++) {
            FS_LOGD("%08x %08x %08x %08x %08x %08x %08x %08x", fs_swap_32(data[8 * i]), fs_swap_32(data[8 * i + 1]),
                    fs_swap_32(data[8 * i + 2]), fs_swap_32(data[8 * i + 3]), fs_swap_32(data[8 * i + 4]),
                    fs_swap_32(data[8 * i + 5]), fs_swap_32(data[8 * i + 6]), fs_swap_32(data[8 * i + 7]));
        }
        switch (len) {
        case 7:
            FS_LOGD("%08x %08x %08x %08x %08x %08x %08x", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]),
                    fs_swap_32(data[8 * count + 2]), fs_swap_32(data[8 * count + 3]), fs_swap_32(data[8 * count + 4]),
                    fs_swap_32(data[8 * count + 5]), fs_swap_32(data[8 * count + 6]));
            break;
        case 6:
            FS_LOGD("%08x %08x %08x %08x %08x %08x", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]),
                    fs_swap_32(data[8 * count + 2]), fs_swap_32(data[8 * count + 3]), fs_swap_32(data[8 * count + 4]),
                    fs_swap_32(data[8 * count + 5]));
            break;
        case 5:
            FS_LOGD("%08x %08x %08x %08x %08x", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]),
                    fs_swap_32(data[8 * count + 2]), fs_swap_32(data[8 * count + 3]), fs_swap_32(data[8 * count + 4]));
            break;
        case 4:
            FS_LOGD("%08x %08x %08x %08x", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]),
                    fs_swap_32(data[8 * count + 2]), fs_swap_32(data[8 * count + 3]));
            break;
        case 3:
            FS_LOGD("%08x %08x %08x", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]),
                    fs_swap_32(fs_swap_32(data[8 * count + 2])));
            break;
        case 2:
            FS_LOGD("%08x %08x", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]));
            break;
        case 1:
            FS_LOGD("%08x", fs_swap_32(data[8 * count]));
            break;
        default:
            break;
        }
    } else {
        for (uint32_t i = 0; i < count; i++) {
            printf("%08x %08x %08x %08x %08x %08x %08x %08x\n", fs_swap_32(data[8 * i]), fs_swap_32(data[8 * i + 1]),
                   fs_swap_32(data[8 * i + 2]), fs_swap_32(data[8 * i + 3]), fs_swap_32(data[8 * i + 4]),
                   fs_swap_32(data[8 * i + 5]), fs_swap_32(data[8 * i + 6]), fs_swap_32(data[8 * i + 7]));
        }
        switch (len) {
        case 7:
            printf("%08x %08x %08x %08x %08x %08x %08x\n", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]),
                   fs_swap_32(data[8 * count + 2]), fs_swap_32(data[8 * count + 3]), fs_swap_32(data[8 * count + 4]),
                   fs_swap_32(data[8 * count + 5]), fs_swap_32(data[8 * count + 6]));
            break;
        case 6:
            printf("%08x %08x %08x %08x %08x %08x\n", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]),
                   fs_swap_32(data[8 * count + 2]), fs_swap_32(data[8 * count + 3]), fs_swap_32(data[8 * count + 4]),
                   fs_swap_32(data[8 * count + 5]));
            break;
        case 5:
            printf("%08x %08x %08x %08x %08x\n", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]),
                   fs_swap_32(data[8 * count + 2]), fs_swap_32(data[8 * count + 3]), fs_swap_32(data[8 * count + 4]));
            break;
        case 4:
            printf("%08x %08x %08x %08x\n", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]),
                   fs_swap_32(data[8 * count + 2]), fs_swap_32(data[8 * count + 3]));
            break;
        case 3:
            printf("%08x %08x %08x\n", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]),
                   fs_swap_32(data[8 * count + 2]));
            break;
        case 2:
            printf("%08x %08x\n", fs_swap_32(data[8 * count]), fs_swap_32(data[8 * count + 1]));
            break;
        case 1:
            printf("%08x\n", fs_swap_32(data[8 * count]));
            break;
        default:
            break;
        }
    }
}

uint32_t fs_swap_32(uint32_t d)
{
    //(((d>>24)&0xFF)|((d>>8)&0xFF00)|((d<<8)&0xFF0000)|((d<<24)&0xFF000000))
    uint32_t t = ((d >> 24) & 0xFF);
    t |= ((d >> 8) & 0xFF00);
    t |= ((d << 8) & 0xFF0000);
    t |= ((d << 24) & 0xFF000000);
    return t;
}

int fs_timingcost(uint8_t ok)
{
    static int beg = 0;
    int ms;
    uint64_t ts;

    ts = fs_util_timestamp_ms();
    if (!ok) {
        beg = (int)ts;
        ms = beg;
    } else {
        int end = (int)ts;
        ms = (end > beg) ? (end - beg) : 0;
    }
    return ms;
}

uint64_t fs_timestamp_us()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (1000000 * tv.tv_sec + tv.tv_usec);
}

void fs_util_mdelay(uint32_t ms)
{
    usleep(ms * 1000);
}

void fs_util_udelay(uint32_t us)
{
    usleep(us);
}

int16_t fs_math_avg(const int16_t *data, int32_t dlen)
{
    int32_t sum = 0;
    int16_t avg = 0;
    int i = 0;
    for (i = 0; i < dlen; i++) {
        sum += data[i];
    }
    avg = (int16_t)(1.0 * sum / dlen);
    return avg;
}


uint32_t fs_crc32(const void *ptr, int len, uint32_t init)
{
    uint32_t i;
    uint32_t crc;
    uint8_t *buf = (uint8_t *)(ptr);
    init = 0xFFFFFFFF;
    crc  = init;
    while (len--) {
        crc ^= (uint32_t)(*buf++) << 24;
        for (i = 0; i < 8; ++i) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ 0x04C11DB7;
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 计算16位累加和校验(ADD16)
 *
 * 16位累加和校验是一种简单的数据完整性校验方法，将数据按16位为单位分割，
 * 采用大端序组合成16位整数后累加，忽略溢出。
 *
 * @param data 指向待校验数据的指针
 * @param dlen 待校验数据的长度（字节数）
 * @param init 初始值，通常为0
 * @return 16位累加和校验结果
 */
uint16_t fs_util_add_16(const void *data, uint32_t dlen, uint16_t init)
{
    #if 0
    uint8_t *buf = (uint8_t *)data;
    uint16_t usResult = init;
    uint32_t i;

    for (i = 0; i < dlen; i += 2) {
        if (i + 1 < dlen) {
            /* 处理完整的16位数据（大端序） */
            usResult += ((uint16_t)buf[i] << 8) | buf[i + 1];
        } else {
            /* 处理剩余的单个字节，作为高字节 */
            usResult += (uint16_t)buf[i] << 8;
        }
    }

    return usResult;
    #else
    uint8_t *buf = (uint8_t *)data;
    uint16_t usResult = init;
    uint32_t i;

    for (i = 0; i < dlen; i ++) {
        usResult += buf[i];
        // if (usResult >= 0xFFFF) {
        //     usResult = (usResult & 0xFFFF) + 1;
        // }
    }

    return usResult;
    #endif
}