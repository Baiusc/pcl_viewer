#ifndef LIDAR_NETWORK_H
#define LIDAR_NETWORK_H

#include "../third_party/cJSON/cJSON.h"
#include <stddef.h>
#include <stdint.h>

#define NET_SCHEMA 1
#define NET_MESSAGE_MAX 8192
#define NET_SOCKET_DEFAULT "/run/lidar-network/control.sock"
#define NET_SN_LENGTH 19u
#define NET_SN_CAPACITY (NET_SN_LENGTH + 1)

typedef enum { NET_AUTO, NET_STATIC, NET_POOL } NetMode;
/* IPv4 在模块内部统一使用主机字节序，只在系统调用边界转换。 */
typedef struct {
	NetMode mode;
	unsigned prefix;
	uint32_t address, first, last, gateway;
} NetConfig;
typedef struct {
	NetConfig confirmed, candidate;
	uint32_t confirmed_address;
	int pending;
	char operation[33];
} NetSaved;

int64_t net_now(void);
int net_random(void *data, size_t length);
int net_token(char out[33]);
int net_unique(const cJSON *value);
int net_integer(const cJSON *value, int minimum, int maximum);
const char *net_string(const cJSON *value, const char *key);
int net_ip(const char *text, uint32_t *address);
const char *net_ip_text(uint32_t address, char out[16]);
uint32_t net_mask(unsigned prefix);
int net_prefix(const char *mask, unsigned *prefix);
int net_config_parse(const cJSON *value, NetConfig *config, char error[160]);
cJSON *net_config_json(const NetConfig *config);
uint32_t net_candidate(const NetConfig *config, uint32_t seed, uint32_t attempt, uint32_t preferred);
int net_config_contains(const NetConfig *config, uint32_t address);
/* 读取返回 1 表示不存在，-1 表示损坏或不可访问。 */
cJSON *net_json_read(const char *path, int *missing);
int net_saved_load(const char *path, NetSaved *saved);
int net_saved_store(const char *path, const NetSaved *saved, int save_to_flash);
cJSON *net_rpc(const char *socket_path, const cJSON *request, int timeout_ms);
int net_sn_validate(const char *sn);
/* 成功返回 0；内容无效返回 1；无法读取返回 -1。输出始终以 NUL 结尾。 */
int net_sn_read(const char *sn_path, char sn[NET_SN_CAPACITY]);
int net_identity(const char *sn_path, const uint8_t mac[6], char sn[NET_SN_CAPACITY], char identity[40]);

#endif
