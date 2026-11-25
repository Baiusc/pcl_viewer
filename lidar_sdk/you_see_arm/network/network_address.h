#ifndef LIDAR_NETWORK_ADDRESS_H
#define LIDAR_NETWORK_ADDRESS_H
#include "network.h"
#include <net/if.h>

typedef struct {
	char interface[IFNAMSIZ];
	unsigned index;
	uint8_t mac[6];
	int arp_fd, simulation;
	uint32_t address, gateway;
	unsigned prefix;
	unsigned route_metric;
} NetInterface;
typedef struct {
	uint32_t address;
	unsigned probes, announcements;
	int ready;
	int64_t next;
} NetProbe;
enum { NET_PROBE_SEND = 1, NET_PROBE_READY = 2, NET_ANNOUNCE_SEND = 4 };
int net_interface_open(NetInterface *interface, const char *name, int simulation);
int net_interface_link(const NetInterface *interface);
int net_interface_apply(NetInterface *interface, uint32_t address, unsigned prefix, uint32_t gateway);
int net_arp_send(const NetInterface *interface, uint32_t address, int probe);
int net_arp_read(const NetInterface *interface, unsigned char frame[28]);
int net_arp_conflict(const unsigned char *frame, size_t size, const uint8_t mac[6], uint32_t address, int probing);
void net_probe_start(NetProbe *probe, uint32_t address, int64_t now, uint32_t random);
unsigned net_probe_tick(NetProbe *probe, int64_t now, uint32_t random);
/* 返回 0 表示静态配置，2 表示显式自动/手动交给服务，1 表示未找到，-1 表示损坏。 */
int net_import_legacy(const char *path, const char *interface, NetConfig *config);
#endif
