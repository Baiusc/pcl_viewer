#include "network.h"
#include <arpa/inet.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int net_unique(const cJSON *value)
{
	if (!cJSON_IsObject(value))
		return 0;
	for (const cJSON *a = value->child; a; a = a->next)
		for (const cJSON *b = a->next; b; b = b->next)
			if (!strcmp(a->string, b->string))
				return 0;
	return 1;
}
int net_integer(const cJSON *value, int minimum, int maximum)
{
	return cJSON_IsNumber(value) && isfinite(value->valuedouble) && value->valuedouble >= minimum &&
	       value->valuedouble <= maximum && value->valuedouble == value->valueint;
}
const char *net_string(const cJSON *value, const char *key)
{
	const cJSON *item = cJSON_GetObjectItemCaseSensitive(value, key);
	return cJSON_IsString(item) ? item->valuestring : NULL;
}
int net_ip(const char *text, uint32_t *address)
{
	struct in_addr ip;
	if (!text || inet_pton(AF_INET, text, &ip) != 1)
		return -1;
	*address = ntohl(ip.s_addr);
	return 0;
}
const char *net_ip_text(uint32_t address, char out[16])
{
	struct in_addr ip = { .s_addr = htonl(address) };
	return inet_ntop(AF_INET, &ip, out, 16);
}
uint32_t net_mask(unsigned prefix)
{
	return prefix && prefix <= 32 ? UINT32_MAX << (32 - prefix) : 0;
}
int net_prefix(const char *text, unsigned *prefix)
{
	uint32_t mask;
	if (net_ip(text, &mask) || !mask || ((~mask + 1) & ~mask))
		return -1;
	*prefix = 0;
	while (mask & 0x80000000u) {
		++*prefix;
		mask <<= 1;
	}
	return 0;
}
static int ordinary(uint32_t ip)
{
	return (ip >> 24) != 0 && (ip >> 24) != 127 && (ip >> 28) < 14 && (ip >> 16) != 0xa9fe;
}
static int host(uint32_t ip, unsigned prefix)
{
	uint32_t mask = net_mask(prefix);
	return ordinary(ip) && (ip & ~mask) != 0 && (ip & ~mask) != ~mask;
}
static int bad(char error[160], const char *message)
{
	snprintf(error, 160, "%s", message);
	return -1;
}
int net_config_parse(const cJSON *value, NetConfig *config, char error[160])
{
	NetConfig parsed = { 0 };
	const char *mode = net_string(value, "mode");
	if (!net_unique(value) || !mode)
		return bad(error, "网络配置必须是键名唯一的对象，且包含 mode");
	if (!strcmp(mode, "auto"))
		parsed.mode = NET_AUTO;
	else if (!strcmp(mode, "static"))
		parsed.mode = NET_STATIC;
	else if (!strcmp(mode, "pool"))
		parsed.mode = NET_POOL;
	else
		return bad(error, "mode 仅支持 auto、static、pool");
	for (const cJSON *item = value->child; item; item = item->next) {
		int allowed = !strcmp(item->string, "mode");
		if (parsed.mode != NET_AUTO)
			allowed |= !strcmp(item->string, "netmask") || !strcmp(item->string, "gateway");
		if (parsed.mode == NET_STATIC)
			allowed |= !strcmp(item->string, "address");
		if (parsed.mode == NET_POOL)
			allowed |= !strcmp(item->string, "start") || !strcmp(item->string, "end");
		if (!allowed)
			return bad(error, "配置包含不适用于当前模式的字段");
	}
	if (parsed.mode == NET_AUTO) {
		*config = parsed;
		return 0;
	}
	if (net_prefix(net_string(value, "netmask"), &parsed.prefix) || parsed.prefix > 30)
		return bad(error, "以太网掩码必须连续，前缀范围为 1～30");
	uint32_t mask = net_mask(parsed.prefix);
	if (parsed.mode == NET_STATIC) {
		if (net_ip(net_string(value, "address"), &parsed.address) || !host(parsed.address, parsed.prefix))
			return bad(error, "固定地址不是有效的普通 IPv4 主机地址");
		parsed.first = parsed.last = parsed.address;
	} else {
		if (net_ip(net_string(value, "start"), &parsed.first) ||
		    net_ip(net_string(value, "end"), &parsed.last) || parsed.first > parsed.last ||
		    !host(parsed.first, parsed.prefix) || !host(parsed.last, parsed.prefix) ||
		    (parsed.first & mask) != (parsed.last & mask) || parsed.last - parsed.first >= 65534u)
			return bad(error, "地址范围须递增、位于同一子网，且最多包含 65534 个主机地址");
		/* 较宽掩码下也不能跨过回环或 Link-Local 保留范围。 */
		if ((parsed.first <= 0x7fffffffu && parsed.last >= 0x7f000000u) ||
		    (parsed.first <= 0xa9feffffu && parsed.last >= 0xa9fe0000u))
			return bad(error, "地址范围包含保留地址");
	}
	const cJSON *gateway = cJSON_GetObjectItemCaseSensitive(value, "gateway");
	if (gateway) {
		if (!cJSON_IsString(gateway))
			return bad(error, "gateway 必须是 IPv4 字符串或空字符串");
		if (*gateway->valuestring && (net_ip(gateway->valuestring, &parsed.gateway) ||
					      (parsed.gateway && (!host(parsed.gateway, parsed.prefix) ||
								  (parsed.gateway & mask) != (parsed.first & mask)))))
			return bad(error, "网关必须是同一子网的有效主机地址");
	}
	if (parsed.gateway && parsed.gateway >= parsed.first && parsed.gateway <= parsed.last &&
	    parsed.first == parsed.last)
		return bad(error, "设备地址不能与网关相同");
	*config = parsed;
	return 0;
}
cJSON *net_config_json(const NetConfig *config)
{
	char text[16];
	cJSON *value = cJSON_CreateObject();
	cJSON_AddStringToObject(value, "mode",
				config->mode == NET_AUTO ? "auto" :
				config->mode == NET_POOL ? "pool" :
							   "static");
	if (config->mode == NET_AUTO)
		return value;
	if (config->mode == NET_STATIC)
		cJSON_AddStringToObject(value, "address", net_ip_text(config->address, text));
	else {
		cJSON_AddStringToObject(value, "start", net_ip_text(config->first, text));
		cJSON_AddStringToObject(value, "end", net_ip_text(config->last, text));
	}
	cJSON_AddStringToObject(value, "netmask", net_ip_text(net_mask(config->prefix), text));
	cJSON_AddStringToObject(value, "gateway", config->gateway ? net_ip_text(config->gateway, text) : "");
	return value;
}
int net_config_contains(const NetConfig *config, uint32_t address)
{
	return config->mode != NET_AUTO && address >= config->first && address <= config->last &&
	       address != config->gateway;
}
uint32_t net_candidate(const NetConfig *config, uint32_t seed, uint32_t attempt, uint32_t preferred)
{
	if (config->mode == NET_AUTO)
		return attempt < 65024u ? 0xa9fe0100u + ((seed + attempt) % 65024u) : 0;
	uint32_t count = config->last - config->first + 1;
	if (attempt >= count)
		return 0;
	uint32_t offset = net_config_contains(config, preferred) ? preferred - config->first : seed % count;
	return config->first + (offset + attempt) % count;
}
