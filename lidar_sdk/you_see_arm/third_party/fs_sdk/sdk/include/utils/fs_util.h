#ifndef __FS_UTIL_H__
#define __FS_UTIL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// 暂时定义的模式，等配置文件读写功能完成后再删除
#define FS_RX_MODE_HIST (0)
#define FS_RX_MODE_ECHO (1)
#define FS_RX_MODE_RANGE (2)
#define FS_RX_MODE_GRAY (3)
#define FS_RX_MODE_RANGE_ECHO (4)
#define FS_RX_MODE_HIST_ECHO (5)
#define FS_RX_MODE   FS_RX_MODE_RANGE

/* Swap 16 bit data. */
#define u16_swap_endian(A) ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))
/* End of u16_swap_endian(A) */

/* Swap 32 bit data. */
#define u32_swap_endian(A) ( \
    (((uint32_t)(A) & 0xff000000) >> 24) | (((uint32_t)(A) & 0x00ff0000) >> 8) | (((uint32_t)(A) & 0x0000ff00) << 8) | (((uint32_t)(A) & 0x000000ff) << 24))
/* End of u32_swap_endian(A) */

/* Swap 64 bit data. */
#define u64_swap_endian(A) ( \
    (((uint64_t)(A) & 0xff00000000000000) >> 56) | (((uint64_t)(A) & 0x00ff000000000000) >> 40) | (((uint64_t)(A) & 0x0000ff0000000000) >> 24) | (((uint64_t)(A) & 0x000000ff00000000) >> 8) | (((uint64_t)(A) & 0x00000000ff000000) << 8) | (((uint64_t)(A) & 0x0000000000ff0000) << 24) | (((uint64_t)(A) & 0x000000000000ff00) << 40) | (((uint64_t)(A) & 0x00000000000000ff) << 56))
/* End of u64_swap_endian(A) */

#define FS_BIT(x)                        ((uint64_t)((uint64_t)0x01 << (x)))

#ifdef __cplusplus
extern "C" {
#endif

uint8_t fs_util_crc8(const void *data, uint32_t dlen, uint8_t init);
uint8_t fs_util_crc_8(const void *data, uint32_t dlen, uint8_t init);
uint16_t fs_util_crc_16(const void *data, uint32_t dlen, uint16_t init);
uint16_t fs_util_add_16(const void *data, uint32_t dlen, uint16_t init);
int32_t  fs_util_u32_swap_endian_buf(const void *in_buf, int32_t in_len, uint32_t *out_buf);
uint64_t fs_util_timestamp_ms(void);

void fs_dump_hex(const uint8_t* data, uint32_t len, const char* name, uint8_t mode);
void fs_dump_hex32(const uint32_t* data, uint32_t len, const char* name, uint8_t mode);
uint32_t fs_swap_32(uint32_t d);
uint64_t fs_timestamp_us();
int fs_timingcost(uint8_t ok);
uint8_t fs_util_crc_8(const void* data, uint32_t dlen, uint8_t init);
// void fs_mdelay(uint32_t ms);   // implemented in fs_hw.c as fs_hw_msleep
// void fs_udelay(uint32_t us);   // implemented in fs_hw.c as fs_hw_usleep
int16_t fs_math_avg(const int16_t* data, int32_t dlen);
uint32_t fs_crc32(const void* ptr, int len, uint32_t init);

/* Unused variables */
#define UNUSED_VAR(v)                    ((void)v)
#define UNUSED_FUNC         __attribute__((__unused__))

// #define FS_BIT(x)                        ((uint64_t)((uint64_t)0x01<<(x)))

#define FS_GENMASK(msb, lsb, bit)        ((uint##bit##_t)((((uint##bit##_t)(~((uint##bit##_t)0))) << (lsb)) & (((uint##bit##_t)(~((uint##bit##_t)0))) >> ((bit) - 1 - (msb)))))
#define FS_SHIFT_VAL(msb, lsb, bit, val) ((uint##bit##_t)((((val) & FS_GENMASK(((msb) - (lsb)), 0, bit)) != (val)) ? FS_GENMASK(msb, lsb, bit) : (FS_GENMASK(msb, lsb, bit) & ((val) << (lsb)))))

#define FS_BITS8(msb, lsb, val)          FS_SHIFT_VAL(msb, lsb,  8, val)
#define FS_BITS16(msb, lsb, val)         FS_SHIFT_VAL(msb, lsb, 16, val)
#define FS_BITS32(msb, lsb, val)         FS_SHIFT_VAL(msb, lsb, 32, val)

/**
 * CONTAINER_OF - 通过成员获取结构体地址
 * @ptr:    the pointer to the member.
 * @type:   the type of the container struct this is embedded in.
 * @member: the name of the member within the struct.
 *
 */
#define CONTAINER_OF(ptr, type, member) ({          \
    const typeof(((type *)0)->member) * __mptr = (ptr); \
    (type *)((char *)__mptr - offsetof(type, member)); })

#ifdef __cplusplus
}
#endif

#endif //__FS_UTIL_H__