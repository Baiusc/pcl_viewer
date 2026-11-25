#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#include "drivers/uart/uart.h"
#include "utils/fs_log.h"
#include "utils/fs_err.h"

#undef  LOG_TAG
#define LOG_TAG "uart"

int32_t fs_uart_open(const char *dev_name)
{
    int32_t fd;

    if (dev_name == NULL) {
        FS_LOGE("dev_name is NULL");
        return FS_ERR_BAD_PARAMS;
    }

    if (access(dev_name, F_OK) != 0) {
        FS_LOGE("serial port %s not found", dev_name);
        return FS_ERR_NO_DEVICE;
    }

    fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        FS_LOGE("Can't open serial port %s: %s", dev_name, strerror(errno));
        return FS_ERR_NO_DEVICE;
    }

    if (fcntl(fd, F_SETFL, 0) < 0) {
        FS_LOGE("fcntl F_SETFL failed: %s", strerror(errno));
        close(fd);
        return FS_ERR_BUSY;
    }

    return fd;
}

int32_t fs_uart_init(int32_t fd, uint32_t speed)
{
    uint32_t speed_arr[] = {B300, B600, B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800};
    uint32_t name_arr[]  = { 300,  600,  1200,  2400,  4800,  9600,  19200,  38400,  57600,  115200,  230400,  460800};
    struct termios options;
    int32_t i;

    if (fd < 0) {
        return FS_ERR_BAD_PARAMS;
    }

    if (tcgetattr(fd, &options) != 0) {
        FS_LOGE("tcgetattr failed: %s", strerror(errno));
        return FS_ERR_INTERNAL;
    }

    for (i = 0; i < (int32_t)(sizeof(speed_arr) / sizeof(speed_arr[0])); i++) {
        if (speed == name_arr[i]) {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
            break;
        }
    }

    if (i >= (int32_t)(sizeof(speed_arr) / sizeof(speed_arr[0]))) {
        FS_LOGE("unsupported baudrate %u", speed);
        return FS_ERR_BAD_PARAMS;
    }

    options.c_cflag |= CLOCAL;
    options.c_cflag |= CREAD;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN]  = 0;

    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        FS_LOGE("tcsetattr failed: %s", strerror(errno));
        return FS_ERR_INTERNAL;
    }

    return FS_SUCCESS;
}

int32_t fs_uart_open_init(const char *dev_name, uint32_t speed)
{
    int32_t fd = fs_uart_open(dev_name);
    if (fd < 0) {
        return fd;
    }

    if (fs_uart_init(fd, speed) != FS_SUCCESS) {
        close(fd);
        return FS_ERR_INTERNAL;
    }

    return fd;
}

int32_t fs_uart_close(int32_t fd)
{
    if (fd < 0) {
        return FS_SUCCESS;
    }

    if (close(fd) < 0) {
        FS_LOGE("close uart fd %d failed: %s", fd, strerror(errno));
        return FS_ERR_INTERNAL;
    }

    return FS_SUCCESS;
}
