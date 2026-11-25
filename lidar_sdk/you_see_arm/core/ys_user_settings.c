#define _GNU_SOURCE // NOLINT(bugprone-reserved-identifier)：使用文件描述符及原子落盘接口。
#include "ys_user_settings.h"
#include "ys_flash_commit.h"
#include "../third_party/cJSON/cJSON.h"
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int valid(const ys_user_settings_t *s)
{
	return s->echo_mode >= 0 && s->echo_mode <= 1 && s->echo_type >= 0 && s->echo_type <= 2 &&
	       s->coordinate_mode >= 0 && s->coordinate_mode <= 1;
}

static int integer(cJSON *root, const char *key)
{
	cJSON *value = cJSON_GetObjectItemCaseSensitive(root, key);
	return cJSON_IsNumber(value) && value->valuedouble == value->valueint ? value->valueint : -1;
}

int ys_user_settings_load(const char *path, ys_user_settings_t *s)
{
	if (!path || path[0] != '/' || strlen(path) >= PATH_MAX)
		return -1;
	int fd = open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
	if (fd < 0)
		return errno == ENOENT ? 1 : -1;
	struct stat st;
	char text[1025] = { 0 };
	int ret = -1;
	if (fstat(fd, &st) || !S_ISREG(st.st_mode) || st.st_uid != geteuid() || (st.st_mode & 0022) || st.st_size < 2 ||
	    st.st_size > 1024)
		goto end;
	size_t got = 0;
	while (got < (size_t)st.st_size) {
		ssize_t n = read(fd, text + got, (size_t)st.st_size - got);
		if (n <= 0)
			goto end;
		got += (size_t)n;
	}
	cJSON *root = cJSON_ParseWithOpts(text, NULL, 1);
	ys_user_settings_t value = { integer(root, "echo_mode"), integer(root, "echo_type"),
				     integer(root, "coordinate_mode") };
	if (cJSON_IsObject(root) && cJSON_GetArraySize(root) == 4 && integer(root, "schema_version") == 1 &&
	    valid(&value)) {
		*s = value;
		ret = 0;
	}
	cJSON_Delete(root);
end:
	close(fd);
	return ret;
}

int ys_user_settings_store(const char *path, const ys_user_settings_t *s)
{
	if (!path || path[0] != '/' || strlen(path) >= PATH_MAX - 16 || !valid(s))
		return -1;
	ys_user_settings_t old;
	int previous = ys_user_settings_load(path, &old);
	if (previous < 0)
		return -1;
	if (!previous && old.echo_mode == s->echo_mode && old.echo_type == s->echo_type &&
	    old.coordinate_mode == s->coordinate_mode)
		return 0;
	char parent[PATH_MAX], temporary[PATH_MAX], text[160];
	snprintf(parent, sizeof(parent), "%s", path);
	char *slash = strrchr(parent, '/');
	if (slash == parent)
		slash[1] = 0;
	else
		*slash = 0;
	int dirfd = open(parent, O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
	struct stat st;
	if (dirfd < 0)
		return -1;
	if (fstat(dirfd, &st) || st.st_uid != geteuid() || (st.st_mode & 0022)) {
		close(dirfd);
		return -1;
	}
	snprintf(temporary, sizeof(temporary), "%s.tmp.XXXXXX", path);
	int fd = mkstemp(temporary), ret = -1;
	if (fd < 0)
		goto end;
	int length = snprintf(text, sizeof(text),
			      "{\"schema_version\":1,\"echo_mode\":%d,\"echo_type\":%d,\"coordinate_mode\":%d}\n",
			      s->echo_mode, s->echo_type, s->coordinate_mode);
	if (fchmod(fd, 0600) || write(fd, text, (size_t)length) != length || fsync(fd)) {
		close(fd);
		goto end;
	}
	if (close(fd) || ys_flash_commit("user-settings", temporary) || rename(temporary, path) || fsync(dirfd))
		goto end;
	ret = 0;
end:
	if (ret)
		unlink(temporary);
	close(dirfd);
	return ret;
}
