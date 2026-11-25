#include "network.h"
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

static int digit(char value)
{
	return value >= '0' && value <= '9';
}

static int uppercase_alphanumeric(char value)
{
	return digit(value) || (value >= 'A' && value <= 'Z');
}

int net_sn_validate(const char *sn)
{
	if (sn == NULL || strlen(sn) != NET_SN_LENGTH)
		return 0;
	if ((sn[0] != 'C' || sn[1] != 'S') && (sn[0] != 'M' || sn[1] != 'F'))
		return 0;
	if (!digit(sn[2]) || !digit(sn[3]))
		return 0;
	for (size_t i = 2; i < 7; ++i) {
		if (!uppercase_alphanumeric(sn[i]))
			return 0;
	}
	if (sn[7] < 'A' || sn[7] > 'Z' || !digit(sn[8]) || !digit(sn[9]))
		return 0;
	if (sn[10] != '0' && sn[10] != '1')
		return 0;
	for (size_t i = 11; i < 15; ++i) {
		if (!digit(sn[i]))
			return 0;
	}
	unsigned week = (unsigned)(sn[13] - '0') * 10u + (unsigned)(sn[14] - '0');
	if (week == 0 || week > 53)
		return 0;
	unsigned batch = 0;
	for (size_t i = 15; i < NET_SN_LENGTH; ++i) {
		unsigned value;
		if (digit(sn[i]))
			value = (unsigned)(sn[i] - '0');
		else if (sn[i] >= 'A' && sn[i] <= 'F')
			value = (unsigned)(sn[i] - 'A') + 10u;
		else
			return 0;
		batch = batch * 16u + value;
	}
	return batch != 0;
}

int net_sn_read(const char *sn_path, char sn[NET_SN_CAPACITY])
{
	if (sn == NULL || sn_path == NULL) {
		errno = EINVAL;
		return -1;
	}
	memset(sn, 0, NET_SN_CAPACITY);
	int fd = open(sn_path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
	if (fd < 0)
		return -1;
	size_t offset = 0;
	while (offset < NET_SN_LENGTH) {
		ssize_t count = read(fd, sn + offset, NET_SN_LENGTH - offset);
		if (count < 0 && errno == EINTR)
			continue;
		if (count <= 0) {
			close(fd);
			memset(sn, 0, NET_SN_CAPACITY);
			return count == 0 ? 1 : -1;
		}
		offset += (size_t)count;
	}
	close(fd);
	sn[NET_SN_LENGTH] = '\0';
	if (!net_sn_validate(sn)) {
		memset(sn, 0, NET_SN_CAPACITY);
		return 1;
	}
	return 0;
}
