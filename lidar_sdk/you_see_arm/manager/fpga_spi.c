#define _GNU_SOURCE // NOLINT(bugprone-reserved-identifier)：需要 Linux 扩展接口。
#include "fpga_spi.h"
#include <fcntl.h>
#include <inttypes.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* 与 AM6204 采集驱动使用相同的 FPGA SPI 读协议和片选，独立于采集模块。 */
#define FPGA_SPI_SPEED 6000000U
#define FPGA_SPI_BITS 16
#define FPGA_SPI_READ_FLAG 0x8000
#define FPGA_VERSION_DATA_REGISTER 0x3000
#define FPGA_VERSION_TIME_REGISTER 0x3004
#define FPGA_VERSION_NUM_REGISTER 0x3008
#define FPGA_HASH_VALUE_REGISTER 0x300c
#define FPGA_MOD_STATE_REGISTER 0x3010

static int fpga_spi_open(void)
{
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = FPGA_SPI_BITS;
	uint32_t speed = FPGA_SPI_SPEED;
	int spi_fd = open("/dev/spidev1.0", O_RDWR | O_CLOEXEC);
	if (spi_fd < 0) {
		return -1;
	}
	if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) || ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) ||
	    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)) {
		goto error;
	}

	int gpio_fd = open("/sys/class/gpio/export", O_WRONLY | O_CLOEXEC);
	if (gpio_fd < 0) {
		goto error;
	}
	/* 已导出的 GPIO 会拒绝重复导出，继续使用对应的方向和值节点。 */
	(void)write(gpio_fd, "382", sizeof("382") - 1);
	close(gpio_fd);
	gpio_fd = open("/sys/class/gpio/gpio382/direction", O_WRONLY | O_CLOEXEC);
	if (gpio_fd < 0) {
		goto error;
	}
	ssize_t written = write(gpio_fd, "high", sizeof("high") - 1);
	close(gpio_fd);
	if (written == sizeof("high") - 1) {
		return spi_fd;
	}
error:
	close(spi_fd);
	return -1;
}

static int fpga_spi_select(const char *value)
{
	int gpio_fd = open("/sys/class/gpio/gpio382/value", O_WRONLY | O_CLOEXEC);
	if (gpio_fd < 0) {
		return -1;
	}
	ssize_t written = write(gpio_fd, value, 1);
	close(gpio_fd);
	return written == 1 ? 0 : -1;
}

static int fpga_spi_read32(int spi_fd, uint16_t address, uint32_t *value)
{
	const uint16_t tx_words[3] = { FPGA_SPI_READ_FLAG | address, 0, 0 };
	uint16_t rx_words[3] = { 0 };
	struct spi_ioc_transfer transfer = { .tx_buf = (uintptr_t)tx_words,
					     .rx_buf = (uintptr_t)rx_words,
					     .len = sizeof(tx_words),
					     .speed_hz = FPGA_SPI_SPEED,
					     .bits_per_word = FPGA_SPI_BITS };
	if (fpga_spi_select("0")) {
		return -1;
	}
	int count = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer);
	if (fpga_spi_select("1") || count != (int)sizeof(tx_words)) {
		return -1;
	}
	*value = ((uint32_t)rx_words[1] << FPGA_SPI_BITS) | rx_words[2];
	return 0;
}

int fpga_read_version_info(lidar_fpga_version_info_t *info)
{
	int spi_fd = fpga_spi_open();
	if (spi_fd < 0)
		return -1;
	int failed = fpga_spi_read32(spi_fd, FPGA_VERSION_DATA_REGISTER, &info->version_data) ||
		     fpga_spi_read32(spi_fd, FPGA_VERSION_TIME_REGISTER, &info->version_time) ||
		     fpga_spi_read32(spi_fd, FPGA_VERSION_NUM_REGISTER, &info->version_num) ||
		     fpga_spi_read32(spi_fd, FPGA_HASH_VALUE_REGISTER, &info->hash_value) ||
		     fpga_spi_read32(spi_fd, FPGA_MOD_STATE_REGISTER, &info->mod_state);
	close(spi_fd);
	return failed ? -1 : 0;
}

int fpga_print_version(void)
{
	uint32_t previous = 0;
	int spi_fd = fpga_spi_open();
	if (spi_fd < 0) {
		fprintf(stderr, "无法初始化 FPGA 版本寄存器的 SPI 读取\n");
		return 1;
	}
	/* 重载后留出就绪时间，要求连续两次读到相同的有效寄存器值。 */
	const int max_attempts = 20;
	const useconds_t interval_us = 100000;
	for (int attempt = 0; attempt < max_attempts; ++attempt) {
		uint32_t version;
		if (!fpga_spi_read32(spi_fd, FPGA_VERSION_NUM_REGISTER, &version) && version && version != UINT32_MAX) {
			if (version == previous) {
				close(spi_fd);
				printf("0x%08" PRIx32 "\n", version);
				return 0;
			}
			previous = version;
		} else {
			previous = 0;
		}
		usleep(interval_us);
	}
	close(spi_fd);
	fprintf(stderr, "FPGA 版本寄存器 0x3008 无法稳定读取\n");
	return 1;
}
