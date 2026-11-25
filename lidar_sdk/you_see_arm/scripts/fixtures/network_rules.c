#include "../../network/network_address.h"
#include <arpa/inet.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
	uint8_t mac[6] = { 2, 0, 0, 0, 0, 1 };
	unsigned char packet[28] = { 0, 1, 8, 0, 6, 4, 0, 1, 2, 0, 0, 0, 0, 2 };
	uint32_t address;
	assert(!net_ip("10.42.0.77", &address));
	uint32_t encoded = htonl(address);
	memcpy(packet + 24, &encoded, 4);
	/* 同时探测同一候选必须视为冲突；普通询问不等于已占用。 */
	assert(net_arp_conflict(packet, sizeof(packet), mac, address, 1));
	assert(!net_arp_conflict(packet, sizeof(packet), mac, address, 0));
	packet[14] = 10;
	assert(!net_arp_conflict(packet, sizeof(packet), mac, address, 1));
	memcpy(packet + 14, &encoded, 4);
	assert(net_arp_conflict(packet, sizeof(packet), mac, address, 0));
	packet[7] = 2;
	assert(net_arp_conflict(packet, sizeof(packet), mac, address, 1));
	memcpy(packet + 8, mac, 6);
	assert(!net_arp_conflict(packet, sizeof(packet), mac, address, 1));
	packet[8] = 4;
	packet[4] = 5;
	assert(!net_arp_conflict(packet, sizeof(packet), mac, address, 1));
	assert(!net_arp_conflict(packet, 12, mac, address, 1));

	/* 用离散时钟验证 RFC 探测与宣告时序，不缩短生产参数。 */
	NetProbe probe;
	net_probe_start(&probe, address, 100, 900);
	assert(!net_probe_tick(&probe, 999, 0));
	assert(net_probe_tick(&probe, 1000, 500) == NET_PROBE_SEND);
	assert(!net_probe_tick(&probe, 2499, 0));
	assert(net_probe_tick(&probe, 2500, 1000) == NET_PROBE_SEND);
	assert(net_probe_tick(&probe, 4500, 0) == NET_PROBE_SEND);
	assert(!net_probe_tick(&probe, 6499, 0));
	assert(net_probe_tick(&probe, 6500, 0) == (NET_PROBE_READY | NET_ANNOUNCE_SEND));
	assert(!net_probe_tick(&probe, 8499, 0));
	assert(net_probe_tick(&probe, 8500, 0) == NET_ANNOUNCE_SEND);
	assert(!net_probe_tick(&probe, 100000, 0));

	/* 范围末段 .0 可以是 /23 的合法主机，不套用 /24 的旧规则。 */
	cJSON *input = cJSON_Parse(
		"{\"mode\":\"pool\",\"start\":\"10.42.0.254\",\"end\":\"10.42.1.2\",\"netmask\":\"255.255.254.0\"}");
	NetConfig config;
	char error[160];
	assert(!net_config_parse(input, &config, error));
	uint32_t selected[5];
	for (unsigned i = 0; i < 5; ++i) {
		selected[i] = net_candidate(&config, 91, i, 0);
		assert(net_config_contains(&config, selected[i]));
		for (unsigned j = 0; j < i; ++j)
			assert(selected[i] != selected[j]);
	}
	assert(!net_candidate(&config, 91, 5, 0));
	cJSON_Delete(input);
	NetConfig automatic = { .mode = NET_AUTO };
	for (unsigned i = 0; i < 65024; ++i) {
		uint32_t candidate = net_candidate(&automatic, 65000, i, 0);
		assert(candidate >= 0xa9fe0100u && candidate <= 0xa9fefeffu);
	}
	puts("地址规则、ARP 冲突分类和标准时序检查通过");
	return 0;
}
