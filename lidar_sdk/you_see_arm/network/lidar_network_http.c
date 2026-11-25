#define _GNU_SOURCE
#include "network.h"
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#ifndef LIDAR_NETWORK_DIAGNOSTIC
#define LIDAR_NETWORK_DIAGNOSTIC 0
#endif

static void respond(int code, const char *text)
{
	const char *reason = code == 200 ? "OK" :
			     code == 202 ? "Accepted" :
			     code == 400 ? "Bad Request" :
			     code == 403 ? "Forbidden" :
			     code == 405 ? "Method Not Allowed" :
			     code == 408 ? "Request Timeout" :
			     code == 409 ? "Conflict" :
			     code == 413 ? "Content Too Large" :
			     code == 415 ? "Unsupported Media Type" :
			     code == 422 ? "Unprocessable Content" :
			     code == 500 ? "Internal Server Error" :
			     code == 501 ? "Not Implemented" :
					   "Service Unavailable";
	printf("Status: %d %s\r\nContent-Type: application/json; charset=utf-8\r\nCache-Control: no-store\r\n"
	       "X-Content-Type-Options: nosniff\r\nContent-Length: %zu\r\n\r\n%s",
	       code, reason, strlen(text), text);
}
static void error_response(int code, const char *message)
{
	cJSON *value = cJSON_CreateObject();
	cJSON_AddNumberToObject(value, "code", code);
	cJSON_AddStringToObject(value, "message", message);
	char *text = cJSON_PrintUnformatted(value);
	respond(code, text ? text : "{\"code\":500,\"message\":\"内存不足\"}");
	free(text);
	cJSON_Delete(value);
}
static int allowed_origin(void)
{
	const char *origin = getenv("HTTP_ORIGIN"), *host = getenv("HTTP_HOST"), *https = getenv("HTTPS");
	if (!origin)
		return 1;
	if (!host || !*host)
		return 0;
	char expected[512];
	const char *scheme = https && (!strcasecmp(https, "on") || !strcmp(https, "1")) ? "https" : "http";
	return snprintf(expected, sizeof(expected), "%s://%s", scheme, host) < (int)sizeof(expected) &&
	       !strcasecmp(expected, origin);
}
static cJSON *read_request(int *code)
{
	const char *method = getenv("REQUEST_METHOD"), *query = getenv("QUERY_STRING");
	if (query && *query) {
		*code = 400;
		return NULL;
	}
	if (method && !strcmp(method, "GET")) {
		cJSON *value = cJSON_CreateObject();
		cJSON_AddStringToObject(value, "op", "network_get");
		return value;
	}
	if (!method || strcmp(method, "POST") || LIDAR_NETWORK_DIAGNOSTIC) {
		*code = 405;
		return NULL;
	}
	if (!allowed_origin()) {
		*code = 403;
		return NULL;
	}
	const char *type = getenv("CONTENT_TYPE"), *length = getenv("CONTENT_LENGTH");
	if (!type || strncasecmp(type, "application/json", 16) || (type[16] && type[16] != ';' && type[16] != ' ')) {
		*code = 415;
		return NULL;
	}
	char *end;
	errno = 0;
	unsigned long count = length ? strtoul(length, &end, 10) : 0;
	if (!length || !*length || strspn(length, "0123456789") != strlen(length) || errno || *end || !count ||
	    count > 2048) {
		*code = count > 2048 ? 413 : 400;
		return NULL;
	}
	char body[2049];
	size_t received = 0;
	int64_t deadline = net_now() + 3000;
	while (received < count) {
		struct pollfd event = { .fd = STDIN_FILENO, .events = POLLIN };
		int64_t left = deadline - net_now();
		if (left <= 0 || poll(&event, 1, (int)left) <= 0) {
			*code = 408;
			return NULL;
		}
		ssize_t n = read(STDIN_FILENO, body + received, count - received);
		if (n < 0 && errno == EINTR)
			continue;
		if (n <= 0) {
			*code = 400;
			return NULL;
		}
		received += (size_t)n;
	}
	body[count] = 0;
	cJSON *value = memchr(body, 0, count) || strstr(body, "\\u0000") ?
			       NULL :
			       cJSON_ParseWithLengthOpts(body, count + 1, NULL, 1);
	const char *op = net_string(value, "op");
	int fields = cJSON_GetArraySize(value), valid = net_unique(value) && op, invalid_config = 0;
	if (valid && !strcmp(op, "network_set")) {
		NetConfig configuration;
		char error[160];
		valid = fields == 2 && cJSON_IsObject(cJSON_GetObjectItemCaseSensitive(value, "config"));
		if (valid &&
		    net_config_parse(cJSON_GetObjectItemCaseSensitive(value, "config"), &configuration, error)) {
			invalid_config = 1;
			valid = 0;
		}
	} else if (valid && !strcmp(op, "network_confirm")) {
		const char *operation = net_string(value, "operation_id"), *local = getenv("SERVER_ADDR");
		uint32_t address;
		valid = fields == 2 && operation && strlen(operation) == 32 &&
			strspn(operation, "0123456789abcdef") == 32 && !net_ip(local, &address) && address;
		if (valid)
			cJSON_AddStringToObject(value, "via_address", local);
	} else
		valid = valid && !strcmp(op, "network_reset") && fields == 1;
	if (!valid) {
		cJSON_Delete(value);
		*code = invalid_config ? 422 : 400;
		return NULL;
	}
	return value;
}
int main(void)
{
	int code = 400;
	cJSON *request = read_request(&code);
	if (!request) {
		error_response(code, "网络请求格式、来源或方法不符合接口要求");
		return 0;
	}
	const char *override = getenv("LIDAR_HTTP_CONFIG");
	cJSON *config = NULL;
	const char *path = LIDAR_NETWORK_DIAGNOSTIC ? NET_SOCKET_DEFAULT : "/run/lidar/control.sock";
	if (!LIDAR_NETWORK_DIAGNOSTIC) {
		int missing;
		config = net_json_read(override ? override : "/etc/lidar/http-bridge.json", &missing);
		if ((!config && (!missing || override)) ||
		    (config && (!net_unique(config) || !net_string(config, "socket_path")))) {
			cJSON_Delete(request);
			cJSON_Delete(config);
			error_response(503, "本地管理桥接配置不可用");
			return 0;
		}
		if (config)
			path = net_string(config, "socket_path");
	}
	const char *operation = net_string(request, "op");
	cJSON *response = net_rpc(path, request, operation && !strcmp(operation, "network_confirm") ? 7000 : 2000);
	cJSON_Delete(request);
	cJSON_Delete(config);
	const cJSON *status = cJSON_GetObjectItemCaseSensitive(response, "code");
	if (!response || !net_integer(status, 200, 599) || !net_string(response, "message") ||
	    (status->valueint < 300 &&
	     !net_integer(cJSON_GetObjectItemCaseSensitive(response, "schema_version"), NET_SCHEMA, NET_SCHEMA))) {
		cJSON_Delete(response);
		error_response(503, "网络管理服务不可用或响应无效；请查询任务结果后再重试");
		return 0;
	}
	char *text = cJSON_PrintUnformatted(response);
	if (text)
		respond(status->valueint, text);
	else
		error_response(500, "内存不足");
	free(text);
	cJSON_Delete(response);
	return 0;
}
