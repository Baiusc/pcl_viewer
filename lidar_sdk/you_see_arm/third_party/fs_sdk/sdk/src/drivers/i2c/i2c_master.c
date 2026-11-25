#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __linux__
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

#include <pthread.h>

#include "fs_err.h"
#include "board_cfg.h"
#include "drivers/i2c/i2c_master.h"

#define I2C_BUS_NUM     8

#define I2C_WRITE_FLAG  0x00
#define I2C_READ_FLAG   0x01


#define MCU_I2C_CMD_BUS_USED(bus)   ((bus) == MCU_CMD_I2C_BUS)

typedef struct {
    bool used;              // 此路I2C是否要用
    int fd;                 //
    pthread_mutex_t mutex;
} i2c_master_dev_t;

static i2c_master_dev_t i2c_master_info[I2C_BUS_NUM] = {
    // i2c-0
    {
        .used   = MCU_I2C_CMD_BUS_USED(0),
        .fd     = -1,
        .mutex  = PTHREAD_MUTEX_INITIALIZER,
    },
    // i2c-1
    {
        .used   = MCU_I2C_CMD_BUS_USED(1),
        .fd     = -1,
        .mutex  = PTHREAD_MUTEX_INITIALIZER,
    },
    // i2c-2
    {
        .used   = MCU_I2C_CMD_BUS_USED(2),
        .fd     = -1,
        .mutex  = PTHREAD_MUTEX_INITIALIZER,
    },
    // i2c-3
    {
        .used   = MCU_I2C_CMD_BUS_USED(3),
        .fd     = -1,
        .mutex  = PTHREAD_MUTEX_INITIALIZER,
    },
    // i2c-4
    {
        .used   = MCU_I2C_CMD_BUS_USED(4),
        .fd     = -1,
        .mutex  = PTHREAD_MUTEX_INITIALIZER,
    },
    // i2c-5
    {
        .used   = MCU_I2C_CMD_BUS_USED(5),
        .fd     = -1,
        .mutex  = PTHREAD_MUTEX_INITIALIZER,
    },
    // i2c-6
    {
        .used   = MCU_I2C_CMD_BUS_USED(6),
        .fd     = -1,
        .mutex  = PTHREAD_MUTEX_INITIALIZER,
    },
    // i2c-7
    {
        .used   = MCU_I2C_CMD_BUS_USED(7),
        .fd     = -1,
        .mutex  = PTHREAD_MUTEX_INITIALIZER,
    },
};

static int i2c_master_inited = 0;

/**
 * @brief 初始化I2C主机
 * @param I2C设备节点，如"/dev/i2c-0"
 * @return true 成功，false 失败
 */
int i2c_master_init(void)
{
    char dev_name[16];

    if (i2c_master_inited) {
        return 0;
    }

    i2c_master_inited = 1;

#ifdef __linux__
    for (int i = 0; i < I2C_BUS_NUM; i++) {
        if (i2c_master_info[i].used) {
            memset(dev_name, 0, sizeof(dev_name));
            sprintf(dev_name, "/dev/i2c-%d", i);
            i2c_master_info[i].fd = open(dev_name, O_RDWR);
            if (i2c_master_info[i].fd < 0) {
                printf("Failed to open %s\n", dev_name);
                return -1;
            }
        }
    }

#endif

    printf("I2C master initialized\n");
    return 0;
}

/**
 * @brief 关闭I2C主机
 */
void i2c_master_close(void)
{
    if (i2c_master_inited == 1) {
#ifdef __linux__
        for (int i = 0; i < I2C_BUS_NUM; i++) {
            if (i2c_master_info[i].used && i2c_master_info[i].fd >= 0) {
                i2c_master_info[i].fd = close(i2c_master_info[i].fd);
                i2c_master_info[i].fd = -1;
            }
        }
#endif
        i2c_master_inited = 0;
    }
}

/**
 * @brief 发送写命令到I2C从机
 * @param busno I2C主机总线序号
 * @param slave_addr I2C从机地址(8位)
 * @param reg 命令字节
 * @param reg_len 命令长度(1或2)
 * @param data 数据缓冲区
 * @param data_len 数据长度
 * @return 0 成功，非0 失败
 */
int i2c_master_write(uint8_t busno, uint8_t slave_addr, uint16_t reg, uint8_t reg_len, uint8_t *data, uint16_t data_len)
{
    uint8_t* pBuf;
    int len;
    int ret = 0;
    struct i2c_rdwr_ioctl_data ioctl_data;

    if (busno >= I2C_BUS_NUM) {
        printf("busno %d not supported\n", busno);
        return FS_ERR_BAD_PARAMS;
    }

    if (i2c_master_info[busno].fd < 0) {
        printf("busno %d not inited\n", busno);
        return FS_ERR_NOT_FOUND;
    }

    len = reg_len + data_len;
    do {
        pBuf = (uint8_t*)malloc(len);
        if (!pBuf) {
            printf("i2c_write malloc failed\n");
            ret = FS_ERR_NOMEM;
            break;
        }

        ioctl_data.nmsgs = 1;
        ioctl_data.msgs = (struct i2c_msg*)malloc(sizeof(struct i2c_msg) * ioctl_data.nmsgs);

        ioctl_data.msgs[0].addr = slave_addr >> 1;
        ioctl_data.msgs[0].flags = I2C_WRITE_FLAG;
        ioctl_data.msgs[0].len = len;  // reg+data
        ioctl_data.msgs[0].buf = pBuf;

        for (int i = 0; i < reg_len; i++) {
            ioctl_data.msgs[0].buf[i] = reg >> ((reg_len - 1 - i) * 8) & 0xff;
        }

        memcpy(&(ioctl_data.msgs[0].buf[reg_len]), data, data_len);

        pthread_mutex_lock(&i2c_master_info[busno].mutex);
        // Using ioctl to write data
        ret = ioctl(i2c_master_info[busno].fd, I2C_RDWR, (unsigned long)&ioctl_data);
        pthread_mutex_unlock(&i2c_master_info[busno].mutex);

        if (ioctl_data.msgs) {
            free(ioctl_data.msgs);
            ioctl_data.msgs = NULL;
        }

        free(pBuf);
        pBuf = NULL;
    } while (0);

    return ret == 1 ? 0 : FS_ERR_I2C_WRITE;
}

/**
 * @brief 发送写命令到I2C从机
 * @param busno I2C主机总线序号
 * @param slave_addr I2C从机地址(8位)
 * @param reg 命令字节
 * @param reg_len 命令长度(1或2)
 * @param data 读取的数据缓冲区
 * @param data_len 要读取的数据长度
 * @return 0 成功，非0 失败
 */
int i2c_master_read(uint8_t busno, uint8_t slave_addr, uint16_t reg, uint8_t reg_len, uint8_t *data, uint16_t data_len)
{
    int ret = 0;
    struct i2c_rdwr_ioctl_data ioctl_data;
    unsigned char pBuf[2];

    if (busno >= I2C_BUS_NUM) {
        printf("busno %d not supported\n", busno);
        return FS_ERR_BAD_PARAMS;
    }

    if (i2c_master_info[busno].fd < 0) {
        printf("busno %d not inited\n", busno);
        return FS_ERR_NOT_FOUND;
    }

    ioctl_data.nmsgs = 2;
    ioctl_data.msgs = (struct i2c_msg*)malloc(sizeof(struct i2c_msg) * ioctl_data.nmsgs);

    ioctl_data.msgs[0].addr = slave_addr >> 1;
    ioctl_data.msgs[0].flags = I2C_WRITE_FLAG;
    ioctl_data.msgs[0].len = reg_len;  // address data
    ioctl_data.msgs[0].buf = pBuf;
    for (int i = 0; i < reg_len; i++) {
        ioctl_data.msgs[0].buf[i] = reg >> ((reg_len - 1 - i) * 8) & 0xff;
    }

    ioctl_data.msgs[1].addr = slave_addr >> 1;
    ioctl_data.msgs[1].flags = I2C_READ_FLAG;
    ioctl_data.msgs[1].len = data_len;
    ioctl_data.msgs[1].buf = data;
    ioctl_data.msgs[1].buf[0] = 0x00;

    pthread_mutex_lock(&i2c_master_info[busno].mutex);
    // Using ioctl to write data
    ret = ioctl(i2c_master_info[busno].fd, I2C_RDWR, (unsigned long)&ioctl_data);
    pthread_mutex_unlock(&i2c_master_info[busno].mutex);

    if (ioctl_data.msgs) {
        free(ioctl_data.msgs);
        ioctl_data.msgs = NULL;
    }

    return ret == 2 ? 0 : FS_ERR_I2C_WRITE;
}
