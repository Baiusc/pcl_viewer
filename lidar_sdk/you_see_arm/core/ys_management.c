#define _POSIX_C_SOURCE 200809L
#include "ys_management.h"
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static volatile sig_atomic_t shutdown_requested;
static int health_fd = -1;
static char token[33];
static unsigned valid_packets;
static int reported;
static void request_shutdown(int signo)
{
	(void)signo;
	shutdown_requested = 1;
}
int ys_management_init(void)
{
	struct sigaction action = { .sa_handler = request_shutdown };
	sigemptyset(&action.sa_mask);
	if (sigaction(SIGTERM, &action, NULL) || sigaction(SIGINT, &action, NULL))
		return -1;
	const char *fd = getenv("LIDAR_MGR_FD"), *nonce = getenv("LIDAR_MGR_TOKEN");
	if (!fd && !nonce)
		return 0;
	if (!fd || !nonce || strlen(nonce) != 32)
		return -1;
	for (int i = 0; i < 32; ++i)
		if (!strchr("0123456789abcdef", nonce[i]))
			return -1;
	char *end;
	long value = strtol(fd, &end, 10);
	if (!*fd || *end || value < 3 || value > 1048576)
		return -1;
	int type;
	socklen_t size = sizeof(type);
	if (getsockopt((int)value, SOL_SOCKET, SO_TYPE, &type, &size) || type != SOCK_SEQPACKET)
		return -1;
	health_fd = (int)value;
	fcntl(health_fd, F_SETFD, FD_CLOEXEC);
	memcpy(token, nonce, sizeof(token));
	return 0;
}
int ys_management_enabled(void)
{
	return health_fd >= 0;
}
int ys_shutdown_requested(void)
{
	return shutdown_requested;
}
static int notify(const char *event, int mode, unsigned count)
{
	char message[256];
	int size = snprintf(message, sizeof(message),
			    "{\"event\":\"%s\",\"token\":\"%s\",\"work_mode\":%d,\"valid_packets\":%u}", event, token,
			    mode, count);
	return send(health_fd, message, (size_t)size, MSG_DONTWAIT | MSG_NOSIGNAL) == size ? 0 : -1;
}
void ys_management_initialized(int work_mode)
{
	if (health_fd >= 0)
		notify("INITIALIZED", work_mode, 0);
}
void ys_management_progress(int work_mode, int valid)
{
	if (health_fd < 0 || reported)
		return;
	if (!valid) {
		valid_packets = 0;
		return;
	}
	if (valid_packets < 10)
		++valid_packets;
	if (valid_packets >= 10 && !notify("PIPELINE_PROGRESS", work_mode, valid_packets))
		reported = 1;
}
void ys_management_close(void)
{
	if (health_fd >= 0)
		close(health_fd);
	health_fd = -1;
}
