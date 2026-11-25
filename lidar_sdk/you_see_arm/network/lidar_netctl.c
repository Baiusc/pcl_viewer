#define _GNU_SOURCE
#include "network.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static cJSON *dhcp_event(const char *event)
{
	const char *token = getenv("LIDAR_DHCP_TOKEN");
	const char *interface = getenv("interface");
	if (!token || !interface)
		return NULL;
	cJSON *value = cJSON_CreateObject();
	cJSON_AddStringToObject(value, "op", "dhcp_event");
	cJSON_AddStringToObject(value, "token", token);
	cJSON_AddStringToObject(value, "event", event);
	cJSON_AddStringToObject(value, "interface", interface);
	if (!strcmp(event, "bound") || !strcmp(event, "renew")) {
		const char *ip = getenv("ip"), *mask = getenv("subnet"), *lease = getenv("lease"),
			   *router = getenv("router");
		if (!ip || !mask || !lease) {
			cJSON_Delete(value);
			return NULL;
		}
		char gateway[16] = "";
		if (router)
			sscanf(router, "%15s", gateway);
		char *end;
		long seconds = strtol(lease, &end, 10);
		if (!*lease || *end || seconds < 1 || seconds > 2147483647) {
			cJSON_Delete(value);
			return NULL;
		}
		cJSON_AddStringToObject(value, "address", ip);
		cJSON_AddStringToObject(value, "netmask", mask);
		cJSON_AddStringToObject(value, "gateway", gateway);
		cJSON_AddNumberToObject(value, "lease_seconds", seconds);
	}
	return value;
}
int main(int argc, char **argv)
{
	if (argc != 3) {
		fprintf(stderr, "用法：lidar_netctl 套接字路径 JSON请求｜get｜ready；DHCP 回调使用 dhcp:事件\n");
		return 2;
	}
	cJSON *request;
	if (!strncmp(argv[2], "dhcp:", 5))
		request = dhcp_event(argv[2] + 5);
	else if (!strcmp(argv[2], "get") || !strcmp(argv[2], "ready")) {
		request = cJSON_CreateObject();
		cJSON_AddStringToObject(request, "op", "network_get");
	} else
		request = cJSON_ParseWithOpts(argv[2], NULL, 1);
	if (!net_unique(request)) {
		cJSON_Delete(request);
		return 2;
	}
	const char *operation = net_string(request, "op");
	cJSON *response = net_rpc(argv[1], request, operation && !strcmp(operation, "network_confirm") ? 7000 : 2000);
	cJSON_Delete(request);
	if (!response) {
		fprintf(stderr, "网络服务不可用或响应超时\n");
		return 1;
	}
	cJSON *code = cJSON_GetObjectItemCaseSensitive(response, "code");
	int failed = !net_integer(code, 200, 299);
	if (!strcmp(argv[2], "ready"))
		failed |= !cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(response, "ready"));
	else {
		char *text = cJSON_PrintUnformatted(response);
		if (text)
			puts(text);
		free(text);
	}
	cJSON_Delete(response);
	return failed ? 1 : 0;
}
