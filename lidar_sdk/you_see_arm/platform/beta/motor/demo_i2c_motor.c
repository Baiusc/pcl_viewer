/* Beta 电机启停；原始实现迁自 dist/demo_i2c_motor.c。 */
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define CONF_BASE_FREQ 100
#define CONF_POLE_PAIRS 7
#define CONF_SPEED_DEF 0x4a40
#define CONF_SPEED_MIN 140
#define CONF_SPEED_MAX 160
#define CONF_POLL_COUNT 10
#define MOTOR_I2C_ADDRESS 0x00

static int i2c_reg_read(int fd, unsigned char reg, unsigned char *value)
{
	struct i2c_msg messages[] = {
		{ .addr = MOTOR_I2C_ADDRESS, .flags = 0, .len = 1, .buf = &reg },
		{ .addr = MOTOR_I2C_ADDRESS, .flags = I2C_M_RD, .len = 1, .buf = value },
	};
	struct i2c_rdwr_ioctl_data data = { .msgs = messages, .nmsgs = 2 };
	int result = ioctl(fd, I2C_RDWR, &data);
	if (result == 2)
		return 0;
	if (result >= 0)
		errno = EIO;
	perror("读取电机寄存器失败");
	return -1;
}

static int i2c_reg_write(int fd, unsigned char reg, unsigned char value)
{
	unsigned char buffer[] = { reg, value };
	struct i2c_msg message = { .addr = MOTOR_I2C_ADDRESS, .flags = 0, .len = 2, .buf = buffer };
	struct i2c_rdwr_ioctl_data data = { .msgs = &message, .nmsgs = 1 };
	int result = ioctl(fd, I2C_RDWR, &data);
	if (result == 1)
		return 0;
	if (result >= 0)
		errno = EIO;
	perror("写入电机寄存器失败");
	return -1;
}

static int sysfs_write(const char *path, const char *value, size_t length, int allow_busy)
{
	int fd = open(path, O_WRONLY);
	if (fd < 0) {
		perror(path);
		return -1;
	}
	ssize_t written = write(fd, value, length);
	int saved_errno = errno;
	int result = close(fd);
	/* 同一轮启动中 GPIO 已导出属于正常情况，其他失败必须上报。 */
	if (written < 0 && allow_busy && saved_errno == EBUSY)
		return result;
	if (written != (ssize_t)length) {
		errno = written < 0 ? saved_errno : EIO;
		perror(path);
		return -1;
	}
	if (result < 0)
		perror("关闭 GPIO 文件失败");
	return result;
}

static int motor_notify(void)
{
	if (sysfs_write("/sys/class/gpio/export", "374", 3, 1) < 0 ||
	    sysfs_write("/sys/class/gpio/gpio374/direction", "out", 3, 0) < 0)
		return -1;

	int result = sysfs_write("/sys/class/gpio/gpio374/value", "1", 1, 0);
	if (result == 0 && usleep(1000) < 0) {
		perror("电机通知脉冲等待失败");
		result = -1;
	}
	/* 即便拉高后的操作失败，也尝试将通知引脚恢复为低电平。 */
	int reset_result = sysfs_write("/sys/class/gpio/gpio374/value", "0", 1, 0);
	return result < 0 ? result : reset_result;
}

static int motor_speed_get(int fd, float *speed)
{
	unsigned char high;
	unsigned char low;
	if (i2c_reg_read(fd, 0x44, &high) < 0 || i2c_reg_read(fd, 0x45, &low) < 0)
		return -1;
	unsigned short omega = ((unsigned short)high << 8) | low;
	*speed = (omega * 60.0F * CONF_BASE_FREQ) / (CONF_POLE_PAIRS * 32768);
	return 0;
}

static int motor_speed_set(int fd, int stop)
{
	/* 启动沿用已有顺序。停转时先清零设定值，避免启用覆盖时使用旧的非零设定。 */
	unsigned short speed = stop ? 0 : CONF_SPEED_DEF;
	if ((!stop && i2c_reg_write(fd, 0x04, 0x03) < 0) || i2c_reg_write(fd, 0x02, speed >> 8) < 0 ||
	    i2c_reg_write(fd, 0x03, speed & 0xff) < 0)
		return -1;
	return stop ? i2c_reg_write(fd, 0x04, 0x03) : 0;
}

int main(int argc, char **argv)
{
	if (argc == 2 && strcmp(argv[1], "--version") == 0) {
		/* 平台脚本在执行停转前从二进制检查此标识，避免旧程序忽略参数并启动电机。 */
		puts("beta-i2c-motor start-stop v1");
		return EXIT_SUCCESS;
	}
	int stop = argc == 2 && strcmp(argv[1], "stop") == 0;
	if (argc != 1 && !stop && (argc != 2 || strcmp(argv[1], "start") != 0)) {
		fputs("用法：demo_i2c_motor [start|stop|--version]\n", stderr);
		return EXIT_FAILURE;
	}

	int fd = open("/dev/i2c-0", O_RDWR);
	if (fd < 0) {
		perror("打开 /dev/i2c-0 失败");
		return EXIT_FAILURE;
	}
	int result = EXIT_FAILURE;
	if (ioctl(fd, I2C_TIMEOUT, 2) < 0 || ioctl(fd, I2C_RETRIES, 2) < 0 ||
	    ioctl(fd, I2C_SLAVE, MOTOR_I2C_ADDRESS) < 0) {
		perror("配置电机 I2C 失败");
		goto cleanup;
	}
	if (motor_speed_set(fd, stop) < 0)
		goto cleanup;

	for (int count = 0; count < CONF_POLL_COUNT; ++count) {
		if (sleep(1) != 0) {
			fputs("电机转速等待被中断\n", stderr);
			goto cleanup;
		}
		float speed;
		if (motor_speed_get(fd, &speed) < 0)
			goto cleanup;
		printf("电机转速：%f\n", (double)speed);
		fflush(stdout);
		if (stop ? speed == 0 : CONF_SPEED_MIN < speed && speed < CONF_SPEED_MAX) {
			if (stop || motor_notify() == 0)
				result = EXIT_SUCCESS;
			goto cleanup;
		}
	}
	fputs(stop ? "电机未在 10 次采样内停转\n" : "电机转速未在 10 次采样内进入目标范围\n", stderr);

cleanup:
	if (close(fd) < 0) {
		perror("关闭电机 I2C 失败");
		result = EXIT_FAILURE;
	}
	return result;
}
