
#include "fs_err.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <getopt.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <assert.h>

#include <linux/types.h>
#include <sys/mman.h>
#include <pthread.h>
#include <poll.h>
#include <semaphore.h>

#include "pinctrl.h"
#include "fs_log.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "Pin"

#define PA_BASE 0
#define PB_BASE 32
#define PC_BASE 64
#define PD_BASE 96
#define PE_BASE 128
#define PF_BASE 160
#define PG_BASE 192
#define PH_BASE 224
#define PI_BASE 256
#define PJ_BASE 288
#define PL_BASE 352
#define PM_BASE 384
#define PN_BASE 416

#define GPIO(bank, pin)  (P ## bank ## _BASE + (pin))

#define STR_OUTPUT_HIGH     "high"
#define STR_OUTPUT_LOW      "low"
#define STR_INPUT           "in"
#define STR_OUTPUT          "out"

// RK1126B GPIO分布（每个bank 32个pin）

static uint16_t pins[] = {
    // output
    // GPIO(B,  13),   // PIN_SYNC_EXT     PB13    -- 帧同步信号
    160,   // PIN_MCU_MODE     PD14    -- MCU_MODE
    175,   // PIN_MCU_HW_RST   PD15    -- MCU_REST
    // input
    // GPIO(I,  0),    // PIN_INPUT0       PI0     --

    // irq
    // GPIO(B,  14),   // PIN_MCU_IRQ      PB14    -- MCU主动上报信息的中断信号
};

static bool g_pin_mcu_mode = true;
static pthread_mutex_t GpioMutex = PTHREAD_MUTEX_INITIALIZER;
static sem_t sem_mcuint;
// static volatile int mcu_irq_run = 0;  // 控制 mcu_irq_handler 线程退出
// static pthread_t mcu_irq_tid = 0;     // mcu_irq_handler 线程 ID

// static void* mcu_irq_handler(void *param)
// {
//     struct pollfd pfd;
//     uint16_t pin;
//     int ret;
//     char file_name[128] = {0};
//     char val;

//     (void)param;

//     pin = pins[PIN_MCU_IRQ];
//     sprintf(file_name, "/sys/class/gpio/gpio%d/value", pin);

//     pfd.fd = open(file_name, O_RDONLY);
//     if (pfd.fd < 0) {
//         FS_LOGE("Open %s failed", file_name);
//         return NULL;
//     }

//     pfd.events = POLLPRI;
//     read(pfd.fd, &val, 1);  // clear status
//     FS_LOGD("Enter mcu_irq_handler");

//     while (mcu_irq_run) {
//         ret = poll(&pfd, 1, 100);  // 100ms 超时，快速响应退出
//         if (ret < 0) {
//             FS_LOGE("poll error");
//             break;
//         } else if (ret == 0) {
//             // 超时，继续检查退出标志
//             continue;
//         } else {
//             // FS_LOGI("poll ok");
//         }

//         if (pfd.revents & POLLPRI) {
//             if (lseek(pfd.fd, 0, SEEK_SET) < 0) {
//                 FS_LOGE("lseek error");
//                 break;
//             }

//             if (read(pfd.fd, &val, 1) < 0) {
//                 FS_LOGE("read error");
//                 break;
//             }
//             // FS_LOGI("mcu_int detected");
//             sem_post(&sem_mcuint);
//         }
//     }
//     close(pfd.fd);
//     FS_LOGD("Exit mcu_irq_handler");

//     return NULL;
// }

int mcu_int_wait_timeout(long timeout_ms)
{
    struct timespec tv;
    long ms;

    clock_gettime(CLOCK_REALTIME, &tv);
    ms = tv.tv_sec * 1000 + tv.tv_nsec / 1000000;
    ms += timeout_ms;

    tv.tv_sec = ms / 1000;
    tv.tv_nsec = (ms % 1000) * 1000000 + (tv.tv_nsec % 1000000);

    return sem_timedwait(&sem_mcuint, &tv);
}

int mcu_int_wait_forever(void)
{
    return sem_wait(&sem_mcuint);
}

int gpio_config(const char *gpio_path, const char *attr, const char *val)
{
    int fd;
    int len;
    int ret;
    char path[128] = {0};

    sprintf(path, "%s/%s", gpio_path, attr);
    fd = open(path, O_RDWR);
    if (fd < 0) {
        FS_LOGE("Open %s failed", path);
        return -1;
    }

    len = strlen(val);
    ret = write(fd, val, len);
    if (ret != len) {
        FS_LOGE("Write %s %s failed", path, val);
        close(fd);
        return -1;
    }

    close(fd);

    return 0;
}

int pinctrl_rp_gpio_init(void)
{
    int res = 0;
    char buf[128];
    int f;

    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "%s", "/proc/rp_gpio/gpio5a0");
    if (access(buf, F_OK) == 0) {
        f = open(buf, O_WRONLY);
        if (f < 0) {
            FS_LOGE("Error:open %s failed", buf);
            return -1;
        }
        res = write(f, "0", 1);
        if (res < 0) {
            FS_LOGE("Error: write %s failed", buf);
            close(f);
            return -1;
        }
        res = 0;
        close(f);
    } else {
        FS_LOGE("Error:open %s failed", buf);
        return -1;
    }

    return res;
}

int pinctrl_init(void)
{
    int res = 0;
    char buf[128];
    int count;
    int f;

    sem_init(&sem_mcuint, 0, 0);

    count = sizeof(pins) / sizeof(uint16_t);
    f = open("/sys/class/gpio/export", O_WRONLY);

    if (f < 0) {
        FS_LOGE("Error:open export failed");
        return -1;
    }

    FS_LOGI("pin count = %d", count);
    for (int i = 0; i < count; i++) {
        uint16_t pin = pins[i];

        memset(buf, 0, sizeof(buf));
        snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d", pin);

        /* Do not unexport MCU reset: releasing the line can create a reset pulse. */
        if (access(buf, F_OK) == 0 && i != PIN_MCU_HW_RST) {
            int f2 = open("/sys/class/gpio/unexport", O_WRONLY);
            if (f2 >= 0) {
                char val[8] = {0};
                memset(val, 0, sizeof(val));
                sprintf(val, "%d", pin);
                write(f2, val, strlen(val));
                close(f2);
            }
        }

        if (access(buf, F_OK) != 0) {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "%d", pin);
            res = write(f, buf, strlen(buf));

            if (res < 0) {
                res = -errno;

                if (res != -EBUSY) {
                    FS_LOGE("Error(%d): export gpio %d", errno, pin);
                    continue;
                }
            }
            res = FS_SUCCESS;
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d", pin);
        }

        if (i >= PIN_OUTPUT_START && i <= PIN_OUTPUT_END) {
            if (i == PIN_MCU_HW_RST) {
                if (gpio_config(buf, "direction", STR_OUTPUT_HIGH) < 0) {
                    continue;
                }
                continue;
            }

            if (gpio_config(buf, "direction", "out") < 0) {
                if(i==PIN_MCU_MODE) {
                    g_pin_mcu_mode = false;
                    FS_LOGW("Failed to set GPIO direction for PIN_MCU_MODE, will try to continue with mode pin always low");
                } else {
                    continue;
                }
            }

            if (i == PIN_MCU_MODE) {
                // mcu 的boot默认低电平是正常工作状态，高电平是进入升级模式，所以默认输出低电平
                if (gpio_config(buf, "value", "0") < 0) {
                    continue;
                }
            }
            else {
                if (gpio_config(buf, "value", "0") < 0) {
                    continue;
                }
            }
        } else {
            if (gpio_config(buf, "direction", "in") < 0) {
                continue;
            }
        }

        // if (i >= PIN_INPUT_START && i <= PIN_IRQ_END) {
        //     if (gpio_config(buf, "active_low", "0") < 0) {
        //         continue;
        //     }

        //     if (gpio_config(buf, "edge", "rising") < 0) {
        //         continue;
        //     }
        // }
    }

    close(f);

    if (!g_pin_mcu_mode) {
        FS_LOGW("GPIO for MCU_MODE is not available, MCU will always be in normal mode");
        res = pinctrl_rp_gpio_init();
    }

    // mcu_irq_run = 1;  // 启动前设置运行标志
    // pthread_create(&mcu_irq_tid, NULL, mcu_irq_handler, NULL);

    return res;
}

void pinctrl_deinit(void)
{
    // mcu_irq_run = 0;  // 通知线程退出
    // if (mcu_irq_tid != 0) {
    //     pthread_join(mcu_irq_tid, NULL);  // 等待线程退出
    //     mcu_irq_tid = 0;
    // }
}
int pinctrl_write(uint16_t pin, uint8_t v)
{
    int res = 0;
    char buf[128] = {0};
    int fd;

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", pin);

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        FS_LOGE("Error:open gpio%d/value", pin);
        return -1;
    }

    res = write(fd, v ? "1" : "0", 1);

    if (res < 0) {
        FS_LOGE("Error: write gpio%d value:%d", pin, v);
    }

    close(fd);
    return res > 0 ? 0 : -1;
}

int pinctrl_read(uint16_t pin)
{
    int res = 0;
    char buf[128];
    int fd;

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", pin);
    fd = open(buf, O_RDONLY);

    if (fd < 0) {
        FS_LOGE("Error:open gpio%d/value", pin);
        return -1;
    }

    res = read(fd, buf, 1);

    if (res < 0) {
        FS_LOGE("Error:read gpio%d value fail", pin);
    }

    close(fd);

    if (res < 0) {
        return -1;
    } else if (buf[0] == '0') {
        return 0;
    } else if (buf[0] == '1') {
        return 1;
    } else {
        return -1;
    }
}

int pinctro_set_rp_gpio(uint16_t io, uint8_t value)
{
    int fd ;
    if (io == PIN_MCU_MODE) {
        fd = open("/proc/rp_gpio/gpio5a0", O_RDWR);
        if (fd < 0) {
            FS_LOGE("Error:open /proc/rp_gpio/gpio5a0 failed");
            return -1;
        }
    } else if (io == PIN_MCU_HW_RST) {
        fd = open("/proc/rp_power/4g_pwrkey", O_RDWR);
        if (fd < 0) {
            FS_LOGE("Error:open /proc/rp_power/4g_pwrkey failed");
            return -1;
        }
    } else {
        FS_LOGE("Error: invalid rp gpio io %d", io);
        return -1;
    }
    if (value) {
        write(fd, "1", 1);
    } else {
        write(fd, "0", 1);
    }
    char buf[32] = {0};
    read(fd, buf, 2);
    FS_LOGI("buf[0] = %c", buf[0]);
    close(fd);
    return 0;
}

int pinctrl_set_gpio(uint16_t io, uint8_t v)
{
    int res = 0;

    pthread_mutex_lock(&GpioMutex);

    if (!g_pin_mcu_mode) {
        if (io == PIN_MCU_MODE) {
            pinctro_set_rp_gpio(io, v);
            res = 0;
            pthread_mutex_unlock(&GpioMutex);
            return res;
        }
        if (io == PIN_MCU_HW_RST) {
            pinctro_set_rp_gpio(io, v);
            res = 0;
            pthread_mutex_unlock(&GpioMutex);
            return res;
        }
    }
    do {
        if (io <= PIN_OUTPUT_END) {
            uint16_t pin = pins[io];
            char buf[128];
            snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", pin);
            int fd = open(buf, O_RDWR);

            if (fd < 0) {
                res = -errno;
                FS_LOGE("Error:open gpio%d/direction %d", pin, res);
                res = -1;
                break;
            }

            if (v) {
                res = write(fd, STR_OUTPUT_HIGH, sizeof(STR_OUTPUT_HIGH));
            } else {
                res = write(fd, STR_OUTPUT_LOW, sizeof(STR_OUTPUT_LOW));
            }

            if (res <= 0) {
                res = -errno;
                FS_LOGE("Error - out gpio%u %u: %d", pin, v, res);
                close(fd);
                break;
            } else {
                res = 0;
            }

            close(fd);
        } else {
            res = pinctrl_write(pins[io], v);
        }
    } while (false);

    pthread_mutex_unlock(&GpioMutex);

    return res;
}

int pinctrl_get_gpio(uint16_t io)
{
    int res = 0;

    if (io > PIN_MAX) {
        FS_LOGE("Error:Io %d", io);
        return -1;
    }

    pthread_mutex_lock(&GpioMutex);

    do {
        if (io >= PIN_INPUT_START && io <= PIN_INPUT_END) {
            uint16_t pin = pins[io];
            char buf[128];
            snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", pin);
            int fd = open(buf, O_RDWR);

            if (fd < 0) {
                res = -errno;
                FS_LOGE("Error:open gpio%d/direction %d", pin, res);
                res = -1;
                break;
            }

            res = write(fd, STR_INPUT, sizeof(STR_INPUT));

            if (res <= 0) {
                res = -errno;
                FS_LOGE("Error - in gpio%u: %d", pin, res);
                close(fd);
                break;
            } else {
                res = 0;
            }

            close(fd);
        }

        res = pinctrl_read(pins[io]);
    } while (false);

    pthread_mutex_unlock(&GpioMutex);
    return res;
}
