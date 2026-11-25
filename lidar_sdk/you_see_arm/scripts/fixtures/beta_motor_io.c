/* 仅用于主机测试：链接包装硬件调用，不访问真实 I2C 或 GPIO。 */
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

static unsigned int writes;
static unsigned int reads;
static unsigned int polls;
static int gpio_kind;

static int scenario(const char *name)
{
	const char *value = getenv("MOTOR_TEST_CASE");
	return value && strcmp(value, name) == 0;
}

static int fail(int error)
{
	errno = error;
	return -1;
}

int __wrap_open(const char *path, int flags, ...)
{
	if (strcmp(path, "/dev/i2c-0") == 0) {
		if (flags != O_RDWR || scenario("open_fail"))
			return fail(ENOENT);
		/* 验证合法的文件描述符 0 不会被误判。 */
		return 0;
	}
	if (flags != O_WRONLY)
		return fail(EINVAL);
	if (strcmp(path, "/sys/class/gpio/export") == 0)
		gpio_kind = 1;
	else if (strcmp(path, "/sys/class/gpio/gpio374/direction") == 0)
		gpio_kind = 2;
	else if (strcmp(path, "/sys/class/gpio/gpio374/value") == 0)
		gpio_kind = 3;
	else
		return fail(ENOENT);
	return 40;
}

int __wrap_ioctl(int fd, unsigned long request, ...)
{
	if (fd != 0)
		return fail(EBADF);
	va_list args;
	va_start(args, request);
	if (request != I2C_RDWR) {
		int value = va_arg(args, int);
		va_end(args);
		if (scenario("setup_fail"))
			return fail(EIO);
		if ((request == I2C_TIMEOUT || request == I2C_RETRIES) && value == 2)
			return 0;
		if (request == I2C_SLAVE && value == 0)
			return 0;
		return fail(EINVAL);
	}
	struct i2c_rdwr_ioctl_data *data = va_arg(args, struct i2c_rdwr_ioctl_data *);
	va_end(args);
	const char *action = getenv("MOTOR_TEST_ACTION");
	int stop = action && strcmp(action, "stop") == 0;
	if (data->nmsgs == 1) {
		const unsigned char start_writes[][2] = { { 0x04, 0x03 }, { 0x02, 0x4a }, { 0x03, 0x40 } };
		const unsigned char stop_writes[][2] = { { 0x02, 0x00 }, { 0x03, 0x00 }, { 0x04, 0x03 } };
		const unsigned char(*expected)[2] = stop ? stop_writes : start_writes;
		struct i2c_msg *msg = data->msgs;
		if (writes >= 3 || msg->addr != 0 || msg->flags != 0 || msg->len != 2 ||
		    memcmp(msg->buf, expected[writes], 2) != 0)
			return fail(EINVAL);
		fprintf(stderr, "REG_WRITE:%02x=%02x\n", msg->buf[0], msg->buf[1]);
		++writes;
		if (scenario("write_fail"))
			return fail(EIO);
		return scenario("write_partial") ? 0 : 1;
	}
	if (data->nmsgs != 2 || writes != 3)
		return fail(EINVAL);
	struct i2c_msg *msgs = data->msgs;
	if (msgs[0].addr != 0 || msgs[1].addr != 0 || msgs[0].flags != 0 || msgs[1].flags != I2C_M_RD ||
	    msgs[0].len != 1 || msgs[1].len != 1 || msgs[0].buf[0] != 0x44 + reads % 2)
		return fail(EINVAL);
	++reads;
	if (scenario("read_fail"))
		return fail(EIO);
	if (scenario("read_partial"))
		return 1;
	unsigned short omega = 5734; /* 对应约 150 的转速。 */
	int settled = !scenario("timeout") && (!scenario("delayed") || polls >= 7);
	if (stop == settled)
		omega = 0;
	*msgs[1].buf = msgs[0].buf[0] == 0x44 ? omega >> 8 : omega & 0xff;
	return 2;
}

ssize_t __wrap_write(int fd, const void *buffer, size_t length)
{
	if (fd != 40)
		return fail(EBADF);
	const char *value = buffer;
	if (gpio_kind == 1) {
		if (length != 3 || memcmp(value, "374", 3) != 0)
			return fail(EINVAL);
		if (scenario("export_fail"))
			return fail(EPERM);
		if (scenario("export_busy"))
			return fail(EBUSY);
	} else if (gpio_kind == 2) {
		if (length != 3 || memcmp(value, "out", 3) != 0)
			return fail(EINVAL);
		if (scenario("gpio_partial"))
			return 1;
	} else if (gpio_kind == 3) {
		if (length != 1 || (*value != '0' && *value != '1'))
			return fail(EINVAL);
		fprintf(stderr, "GPIO_VALUE:%c\n", *value);
		if (scenario("gpio_fail") && *value == '1')
			return fail(EIO);
	} else {
		return fail(EINVAL);
	}
	return (ssize_t)length;
}

int __wrap_close(int fd)
{
	if (fd == 0) {
		fputs("I2C_CLOSED\n", stderr);
		return scenario("close_fail") ? fail(EIO) : 0;
	}
	return fd == 40 ? 0 : fail(EBADF);
}

unsigned int __wrap_sleep(unsigned int seconds)
{
	if (seconds != 1)
		abort();
	++polls;
	fputs("POLL\n", stderr);
	return scenario("sleep_fail") ? 1 : 0;
}

int __wrap_usleep(useconds_t microseconds)
{
	if (microseconds != 1000)
		abort();
	return scenario("pulse_fail") ? fail(EINTR) : 0;
}
