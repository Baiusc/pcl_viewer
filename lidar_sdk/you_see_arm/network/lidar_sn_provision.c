#include "network.h"
#include <errno.h>
#include <fcntl.h>
#include <mtd/mtd-user.h>
#include <stdio.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#define SN_DEVICE "/dev/mtd3"
#define SN_SYSFS_NAME "/sys/class/mtd/mtd3/name"
#define SN_PARTITION_NAME "sn"

static int read_exact(int fd, void *buffer, size_t length)
{
	size_t offset = 0;
	while (offset < length) {
		ssize_t count = read(fd, (char *)buffer + offset, length - offset);
		if (count < 0 && errno == EINTR)
			continue;
		if (count == 0) {
			errno = EIO;
			return -1;
		}
		if (count < 0)
			return -1;
		offset += (size_t)count;
	}
	return 0;
}

static int write_exact(int fd, const void *buffer, size_t length)
{
	size_t offset = 0;
	while (offset < length) {
		ssize_t count = write(fd, (const char *)buffer + offset, length - offset);
		if (count < 0 && errno == EINTR)
			continue;
		if (count == 0) {
			errno = EIO;
			return -1;
		}
		if (count < 0)
			return -1;
		offset += (size_t)count;
	}
	return 0;
}

static int sn_partition_is_expected(void)
{
	char name[32] = { 0 };
	int fd = open(SN_SYSFS_NAME, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
	if (fd < 0)
		return 0;
	ssize_t count;
	do {
		count = read(fd, name, sizeof(name) - 1);
	} while (count < 0 && errno == EINTR);
	close(fd);
	if (count <= 0)
		return 0;
	while (count > 0 && (name[count - 1] == '\n' || name[count - 1] == '\r'))
		name[--count] = '\0';
	return strcmp(name, SN_PARTITION_NAME) == 0;
}

static int is_blank(const unsigned char data[NET_SN_LENGTH])
{
	for (size_t i = 0; i < NET_SN_LENGTH; ++i) {
		if (data[i] != 0xff)
			return 0;
	}
	return 1;
}

/* 仅允许空分区或本次 SN 的未完成前缀，便于安全重试断电中断的首次写入。 */
static int can_retry(const unsigned char current[NET_SN_LENGTH], const char target[NET_SN_CAPACITY])
{
	int erased_tail = 0;
	for (size_t i = 0; i < NET_SN_LENGTH; ++i) {
		if (current[i] == 0xff) {
			erased_tail = 1;
			continue;
		}
		if (erased_tail || current[i] != (unsigned char)target[i])
			return 0;
	}
	return erased_tail;
}

static int provision(const char *sn)
{
	if (geteuid() != 0) {
		fprintf(stderr, "写入 SN 需要 root 权限\n");
		return -1;
	}
	if (!sn_partition_is_expected()) {
		fprintf(stderr, "%s 的分区标签不是 %s，拒绝写入\n", SN_DEVICE, SN_PARTITION_NAME);
		return -1;
	}
	int fd = open(SN_DEVICE, O_RDWR | O_CLOEXEC | O_NOFOLLOW);
	if (fd < 0) {
		fprintf(stderr, "打开 %s 失败：%s\n", SN_DEVICE, strerror(errno));
		return -1;
	}
	if (flock(fd, LOCK_EX | LOCK_NB)) {
		fprintf(stderr, "SN 分区正在被另一写码进程占用：%s\n", strerror(errno));
		close(fd);
		return -1;
	}
	struct stat st;
	struct mtd_info_user info;
	if (fstat(fd, &st)) {
		fprintf(stderr, "检查 %s 失败：%s\n", SN_DEVICE, strerror(errno));
		close(fd);
		return -1;
	}
	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s 不是字符设备，拒绝写入\n", SN_DEVICE);
		close(fd);
		return -1;
	}
	if (ioctl(fd, MEMGETINFO, &info)) {
		fprintf(stderr, "%s 不是可识别的 MTD 设备：%s\n", SN_DEVICE, strerror(errno));
		close(fd);
		return -1;
	}
	if (info.erasesize < NET_SN_LENGTH || info.size != info.erasesize) {
		fprintf(stderr, "%s 必须是仅用于 SN 的单擦除块分区，当前大小=%u 擦除块=%u\n", SN_DEVICE, info.size,
			info.erasesize);
		close(fd);
		return -1;
	}

	unsigned char current[NET_SN_LENGTH];
	if (lseek(fd, 0, SEEK_SET) < 0 || read_exact(fd, current, sizeof(current))) {
		fprintf(stderr, "读取当前 SN 分区失败：%s\n", strerror(errno));
		close(fd);
		return -1;
	}
	if (memcmp(current, sn, NET_SN_LENGTH) == 0) {
		printf("SN 已经是 %s，无需重写\n", sn);
		close(fd);
		return 0;
	}
	if (!is_blank(current) && !can_retry(current, sn)) {
		fprintf(stderr, "SN 分区已有其他内容，拒绝覆盖；请先按返修流程核对设备身份\n");
		close(fd);
		return -1;
	}

	erase_info_t erase = { .start = 0, .length = info.erasesize };
	if (ioctl(fd, MEMERASE, &erase)) {
		fprintf(stderr, "擦除 SN 分区失败：%s\n", strerror(errno));
		close(fd);
		return -1;
	}
	if (lseek(fd, 0, SEEK_SET) < 0 || write_exact(fd, sn, NET_SN_LENGTH)) {
		fprintf(stderr, "写入 SN 失败：%s\n", strerror(errno));
		close(fd);
		return -1;
	}
	if (close(fd)) {
		fprintf(stderr, "关闭 SN 分区失败：%s\n", strerror(errno));
		return -1;
	}

	fd = open(SN_DEVICE, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
	if (fd < 0) {
		fprintf(stderr, "重新打开 SN 分区回读失败：%s\n", strerror(errno));
		return -1;
	}
	unsigned char verify[NET_SN_LENGTH];
	int verified = lseek(fd, 0, SEEK_SET) >= 0 && !read_exact(fd, verify, sizeof(verify)) &&
		       memcmp(verify, sn, NET_SN_LENGTH) == 0;
	close(fd);
	if (!verified) {
		fprintf(stderr, "SN 回读不一致，设备不得进入出厂放行\n");
		return -1;
	}
	printf("SN 写入并回读成功：%s\n", sn);
	return 0;
}

int main(int argc, char **argv)
{
	if (argc != 2 || !net_sn_validate(argv[1])) {
		fprintf(stderr, "用法：%s <19位SN>\n", argv[0]);
		fprintf(stderr, "示例：%s CS96XXXB01026020001\n", argv[0]);
		return 2;
	}
	return provision(argv[1]) == 0 ? 0 : 1;
}
