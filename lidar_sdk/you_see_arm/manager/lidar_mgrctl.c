#define _GNU_SOURCE
#include "../third_party/cJSON/cJSON.h"
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

int main(int argc, char **argv)
{
	if (argc < 3 || argc > 4 || (strcmp(argv[2], "status") && strcmp(argv[2], "switch"))) {
		fprintf(stderr, "用法：lidar_mgrctl 套接字路径 status | switch 工作模式\n");
		return 2;
	}
	struct sockaddr_un address = { .sun_family = AF_UNIX };
	if (strlen(argv[1]) >= sizeof(address.sun_path))
		return 2;
	strcpy(address.sun_path, argv[1]);
	cJSON *request = cJSON_CreateObject();
	cJSON_AddStringToObject(request, "op", argv[2]);
	if (!strcmp(argv[2], "switch")) {
		if (argc != 4) {
			cJSON_Delete(request);
			return 2;
		}
		char *end;
		errno = 0;
		long mode = strtol(argv[3], &end, 10);
		if (errno || !*argv[3] || *end || mode < 0 || mode > 3) {
			cJSON_Delete(request);
			return 2;
		}
		cJSON_AddNumberToObject(request, "work_mode", mode);
	} else if (argc != 3) {
		cJSON_Delete(request);
		return 2;
	}
	int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
	char *text = cJSON_PrintUnformatted(request);
	cJSON_Delete(request);
	if (fd < 0 || !text || connect(fd, (struct sockaddr *)&address, sizeof(address)) ||
	    send(fd, text, strlen(text), MSG_NOSIGNAL) != (ssize_t)strlen(text)) {
		perror("管理服务连接失败");
		free(text);
		if (fd >= 0)
			close(fd);
		return 1;
	}
	free(text);
	struct pollfd pollfd = { .fd = fd, .events = POLLIN };
	char response[8192];
	ssize_t size = poll(&pollfd, 1, 10000) > 0 ? recv(fd, response, sizeof(response) - 1, MSG_TRUNC) : -1;
	close(fd);
	if (size <= 0 || size >= (ssize_t)sizeof(response)) {
		fprintf(stderr, "管理服务响应超时或过长\n");
		return 1;
	}
	response[size] = 0;
	puts(response);
	cJSON *value = cJSON_Parse(response), *code = cJSON_GetObjectItemCaseSensitive(value, "code");
	int result = !cJSON_IsNumber(code) || code->valueint >= 400;
	cJSON_Delete(value);
	return result;
}
