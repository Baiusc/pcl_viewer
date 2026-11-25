#define _GNU_SOURCE
#include "network.h"
#include "../core/ys_flash_commit.h"
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

int64_t net_now(void)
{
	struct timespec now;
	return clock_gettime(CLOCK_MONOTONIC, &now) ? -1 : (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}
int net_random(void *data, size_t length)
{
	size_t done = 0;
	while (done < length) {
		ssize_t n = getrandom((char *)data + done, length - done, 0);
		if (n < 0 && errno == EINTR)
			continue;
		if (n <= 0)
			return -1;
		done += (size_t)n;
	}
	return 0;
}
int net_token(char out[33])
{
	unsigned char token[16];
	if (net_random(token, sizeof(token)))
		return -1;
	for (unsigned i = 0; i < sizeof(token); ++i)
		snprintf(out + i * 2, 3, "%02x", token[i]);
	return 0;
}
cJSON *net_json_read(const char *path, int *missing)
{
	if (missing)
		*missing = 0;
	int fd = open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
	if (fd < 0) {
		if (missing && errno == ENOENT)
			*missing = 1;
		return NULL;
	}
	struct stat st;
	char buffer[NET_MESSAGE_MAX + 1];
	if (fstat(fd, &st) || !S_ISREG(st.st_mode) || st.st_size < 2 || st.st_size > NET_MESSAGE_MAX ||
	    (st.st_uid != 0 && st.st_uid != geteuid()) || (st.st_mode & 0022)) {
		close(fd);
		return NULL;
	}
	size_t length = 0;
	while (length < (size_t)st.st_size) {
		ssize_t n = read(fd, buffer + length, (size_t)st.st_size - length);
		if (n < 0 && errno == EINTR)
			continue;
		if (n <= 0) {
			close(fd);
			return NULL;
		}
		length += (size_t)n;
	}
	close(fd);
	if (memchr(buffer, 0, length))
		return NULL;
	buffer[length] = 0;
	if (strstr(buffer, "\\u0000"))
		return NULL;
	return cJSON_ParseWithLengthOpts(buffer, length + 1, NULL, 1);
}
static int net_json_write(const char *path, const cJSON *value, int save_to_flash)
{
	char parent[PATH_MAX], candidate[PATH_MAX], temporary[64];
	if (!path || path[0] != '/' || strlen(path) >= sizeof(parent))
		return -1;
	strcpy(parent, path);
	char *name = strrchr(parent, '/');
	if (!name || !name[1])
		return -1;
	*name++ = 0;
	int dirfd = open(*parent ? parent : "/", O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
	struct stat st;
	if (dirfd < 0)
		return -1;
	if (fstat(dirfd, &st) || st.st_uid != geteuid() || (st.st_mode & 0022)) {
		close(dirfd);
		return -1;
	}
	snprintf(temporary, sizeof(temporary), ".network.%ld.tmp", (long)getpid());
	if (snprintf(candidate, sizeof(candidate), "%s/%s", parent, temporary) >= (int)sizeof(candidate)) {
		close(dirfd);
		return -1;
	}
	int fd = openat(dirfd, temporary, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
	if (fd < 0 && errno == EEXIST && !unlinkat(dirfd, temporary, 0))
		fd = openat(dirfd, temporary, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
	char *text = cJSON_PrintUnformatted(value);
	int ret = -1;
	if (fd >= 0 && text) {
		size_t done = 0, length = strlen(text);
		while (done < length) {
			ssize_t n = write(fd, text + done, length - done);
			if (n < 0 && errno == EINTR)
				continue;
			if (n <= 0)
				break;
			done += (size_t)n;
		}
		if (done == length && !fsync(fd) && !ys_flash_commit(save_to_flash ? "network" : NULL, candidate) &&
		    !renameat(dirfd, temporary, dirfd, name) && !fsync(dirfd))
			ret = 0;
	}
	if (fd >= 0)
		close(fd);
	if (ret)
		unlinkat(dirfd, temporary, 0);
	free(text);
	close(dirfd);
	return ret;
}
int net_saved_load(const char *path, NetSaved *saved)
{
	int missing;
	cJSON *value = net_json_read(path, &missing);
	if (!value)
		return missing ? 1 : -1;
	char error[160];
	NetSaved parsed = { 0 };
	const cJSON *version = cJSON_GetObjectItemCaseSensitive(value, "schema_version");
	const cJSON *pending = cJSON_GetObjectItemCaseSensitive(value, "pending");
	const char *operation = net_string(value, "operation_id");
	int invalid = !net_unique(value) || !net_integer(version, NET_SCHEMA, NET_SCHEMA) || !cJSON_IsBool(pending) ||
		      !operation || strlen(operation) > 32 ||
		      net_config_parse(cJSON_GetObjectItemCaseSensitive(value, "confirmed_config"), &parsed.confirmed,
				       error) ||
		      net_config_parse(cJSON_GetObjectItemCaseSensitive(value, "candidate_config"), &parsed.candidate,
				       error) ||
		      net_ip(net_string(value, "confirmed_address"), &parsed.confirmed_address);
	if (!invalid) {
		parsed.pending = cJSON_IsTrue(pending);
		if (parsed.pending && (strlen(operation) != 32 || strspn(operation, "0123456789abcdef") != 32))
			invalid = 1;
		if (parsed.confirmed_address && parsed.confirmed.mode != NET_AUTO &&
		    !net_config_contains(&parsed.confirmed, parsed.confirmed_address))
			invalid = 1;
	}
	if (!invalid) {
		strcpy(parsed.operation, operation);
		*saved = parsed;
	}
	cJSON_Delete(value);
	return invalid ? -1 : 0;
}
int net_saved_store(const char *path, const NetSaved *saved, int save_to_flash)
{
	char text[16];
	cJSON *value = cJSON_CreateObject();
	cJSON_AddNumberToObject(value, "schema_version", NET_SCHEMA);
	cJSON_AddBoolToObject(value, "pending", saved->pending);
	cJSON_AddStringToObject(value, "operation_id", saved->operation);
	cJSON_AddStringToObject(value, "confirmed_address", net_ip_text(saved->confirmed_address, text));
	cJSON_AddItemToObject(value, "confirmed_config", net_config_json(&saved->confirmed));
	cJSON_AddItemToObject(value, "candidate_config", net_config_json(&saved->candidate));
	int ret = net_json_write(path, value, save_to_flash);
	cJSON_Delete(value);
	return ret;
}
static int wait_io(int fd, short events, int64_t deadline)
{
	for (;;) {
		int64_t left = deadline - net_now();
		if (left <= 0)
			return -1;
		struct pollfd item = { .fd = fd, .events = events };
		int ret = poll(&item, 1, left > INT_MAX ? INT_MAX : (int)left);
		if (ret < 0 && errno == EINTR)
			continue;
		return ret > 0 && (item.revents & events) ? 0 : -1;
	}
}
cJSON *net_rpc(const char *path, const cJSON *request, int timeout_ms)
{
	if (!path || *path != '/' || strlen(path) >= sizeof(((struct sockaddr_un *)0)->sun_path) || timeout_ms < 1)
		return NULL;
	int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
	if (fd < 0)
		return NULL;
	struct sockaddr_un address = { .sun_family = AF_UNIX };
	strcpy(address.sun_path, path);
	int64_t deadline = net_now() + timeout_ms;
	cJSON *response = NULL;
	char *text = cJSON_PrintUnformatted(request);
	if (!text || strlen(text) >= NET_MESSAGE_MAX)
		goto end;
	if (connect(fd, (struct sockaddr *)&address, sizeof(address))) {
		int error = 0;
		socklen_t size = sizeof(error);
		if ((errno != EINPROGRESS && errno != EAGAIN) || wait_io(fd, POLLOUT, deadline) ||
		    getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &size) || error)
			goto end;
	}
	struct ucred peer;
	socklen_t length = sizeof(peer);
	if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &peer, &length) || peer.uid != geteuid())
		goto end;
	if (wait_io(fd, POLLOUT, deadline) || send(fd, text, strlen(text), MSG_NOSIGNAL) != (ssize_t)strlen(text) ||
	    wait_io(fd, POLLIN, deadline))
		goto end;
	char buffer[NET_MESSAGE_MAX + 1];
	ssize_t n = recv(fd, buffer, NET_MESSAGE_MAX, MSG_TRUNC);
	if (n > 0 && n <= NET_MESSAGE_MAX && !memchr(buffer, 0, (size_t)n)) {
		buffer[n] = 0;
		response = cJSON_ParseWithLengthOpts(buffer, (size_t)n + 1, NULL, 1);
		if (!net_unique(response)) {
			cJSON_Delete(response);
			response = NULL;
		}
	}
end:
	free(text);
	close(fd);
	return response;
}
int net_identity(const char *path, const uint8_t mac[6], char sn[NET_SN_CAPACITY], char identity[40])
{
	int valid = net_sn_read(path, sn) == 0;
	if (valid)
		snprintf(identity, 40, "%s", sn);
	else {
		memset(sn, 0, NET_SN_CAPACITY);
		snprintf(identity, 40, "mac-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	}
	return valid ? 0 : 1;
}
