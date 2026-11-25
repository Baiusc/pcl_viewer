#ifndef LIDAR_NETWORK_DISCOVERY_H
#define LIDAR_NETWORK_DISCOVERY_H
#include "network_address.h"
#include <sys/types.h>

typedef struct {
	int socket, enabled;
	pid_t pid;
	unsigned failures;
	int64_t retry_at, broadcast_at;
	char config_path[4096];
} NetDiscovery;
int net_discovery_open(NetDiscovery *discovery, const NetInterface *interface, const char *directory,
		       const char *services_directory, const char *identity, const char *sn, const char *model,
		       unsigned port, int avahi_dbus_supported);
void net_discovery_tick(NetDiscovery *discovery, const NetInterface *interface, const char *binary,
			const cJSON *snapshot, unsigned port, int64_t now);
void net_discovery_close(NetDiscovery *discovery);
#endif
