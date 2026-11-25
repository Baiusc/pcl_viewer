#define _GNU_SOURCE
#include "network_address.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if_arp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

static int route_request(unsigned index, uint32_t address, unsigned prefix, uint32_t gateway, unsigned metric, int add)
{
	struct {
		struct nlmsghdr header;
		union {
			struct ifaddrmsg address;
			struct rtmsg route;
		} body;
		unsigned char attributes[128];
	} message = { 0 };
	int route = gateway != 0;
	message.header.nlmsg_len = NLMSG_LENGTH(route ? sizeof(struct rtmsg) : sizeof(struct ifaddrmsg));
	message.header.nlmsg_type = route ? (add ? RTM_NEWROUTE : RTM_DELROUTE) : (add ? RTM_NEWADDR : RTM_DELADDR);
	message.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | (add ? NLM_F_CREATE | NLM_F_EXCL : 0);
	message.header.nlmsg_seq = 1;
	if (route) {
		message.body.route.rtm_family = AF_INET;
		message.body.route.rtm_table = RT_TABLE_MAIN;
		message.body.route.rtm_protocol = add ? RTPROT_STATIC : RTPROT_UNSPEC;
		message.body.route.rtm_scope = RT_SCOPE_UNIVERSE;
		message.body.route.rtm_type = RTN_UNICAST;
	} else {
		message.body.address.ifa_family = AF_INET;
		message.body.address.ifa_prefixlen = (unsigned char)prefix;
		message.body.address.ifa_scope = (address >> 16) == 0xa9fe ? RT_SCOPE_LINK : RT_SCOPE_UNIVERSE;
		message.body.address.ifa_index = index;
	}
	uint32_t values[] = { htonl(address), htonl(address | ~net_mask(prefix)), htonl(gateway), index, metric };
	unsigned types[] = { IFA_LOCAL, IFA_BROADCAST, RTA_GATEWAY, RTA_OIF, RTA_PRIORITY };
	for (unsigned i = route ? 2 : 0; i < (route ? 5u : 2u); ++i) {
		struct rtattr *attribute = (void *)((char *)&message + NLMSG_ALIGN(message.header.nlmsg_len));
		attribute->rta_type = (unsigned short)types[i];
		attribute->rta_len = RTA_LENGTH(sizeof(uint32_t));
		memcpy(RTA_DATA(attribute), &values[i], sizeof(uint32_t));
		message.header.nlmsg_len = NLMSG_ALIGN(message.header.nlmsg_len) + RTA_LENGTH(sizeof(uint32_t));
	}
	int fd = socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, NETLINK_ROUTE);
	if (fd < 0)
		return -1;
	struct sockaddr_nl kernel = { .nl_family = AF_NETLINK };
	int ret = -1;
	if (sendto(fd, &message, message.header.nlmsg_len, 0, (struct sockaddr *)&kernel, sizeof(kernel)) < 0)
		goto end;
	struct pollfd event = { .fd = fd, .events = POLLIN };
	if (poll(&event, 1, 250) <= 0)
		goto end;
	unsigned char reply[4096];
	ssize_t length = recv(fd, reply, sizeof(reply), 0);
	for (struct nlmsghdr *header = (void *)reply; NLMSG_OK(header, length); header = NLMSG_NEXT(header, length)) {
		if (header->nlmsg_type == NLMSG_ERROR && header->nlmsg_len >= NLMSG_LENGTH(sizeof(struct nlmsgerr))) {
			int error = ((struct nlmsgerr *)NLMSG_DATA(header))->error;
			if (!error || (add && error == -EEXIST) ||
			    (!add && (error == -EADDRNOTAVAIL || error == -ESRCH)))
				ret = 0;
			else
				errno = -error;
			break;
		}
	}
end:
	close(fd);
	return ret;
}
int net_interface_open(NetInterface *interface, const char *name, int simulation)
{
	memset(interface, 0, sizeof(*interface));
	interface->arp_fd = -1;
	if (!name || !*name || strlen(name) >= IFNAMSIZ ||
	    strspn(name, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-") != strlen(name) ||
	    !strcmp(name, "lo"))
		return -1;
	strcpy(interface->interface, name);
	interface->simulation = simulation;
	if (simulation) {
		interface->index = 1;
		interface->mac[0] = 2;
		interface->mac[5] = 1;
		return 0;
	}
	int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
	struct ifreq request = { 0 };
	strcpy(request.ifr_name, name);
	if (fd < 0)
		return -1;
	int bad = ioctl(fd, SIOCGIFHWADDR, &request);
	if (!bad)
		memcpy(interface->mac, request.ifr_hwaddr.sa_data, 6);
	if (!bad)
		bad = ioctl(fd, SIOCGIFFLAGS, &request);
	if (!bad && !(request.ifr_flags & IFF_UP)) {
		request.ifr_flags |= IFF_UP;
		bad = ioctl(fd, SIOCSIFFLAGS, &request);
	}
	close(fd);
	interface->index = if_nametoindex(name);
	if (bad || !interface->index || (interface->mac[0] & 1))
		return -1;
	struct ifaddrs *addresses = NULL;
	if (getifaddrs(&addresses))
		return -1;
	for (struct ifaddrs *item = addresses; item; item = item->ifa_next) {
		if (!item->ifa_addr || item->ifa_addr->sa_family != AF_INET || strcmp(item->ifa_name, name))
			continue;
		/* 不接管无法明确归属的多地址配置。 */
		if (interface->address || !item->ifa_netmask) {
			freeifaddrs(addresses);
			return -1;
		}
		interface->address = ntohl(((struct sockaddr_in *)item->ifa_addr)->sin_addr.s_addr);
		char mask[16];
		inet_ntop(AF_INET, &((struct sockaddr_in *)item->ifa_netmask)->sin_addr, mask, sizeof(mask));
		if (net_prefix(mask, &interface->prefix)) {
			freeifaddrs(addresses);
			return -1;
		}
	}
	freeifaddrs(addresses);
	/* 首次迁移时接管该接口已有的唯一默认路由，避免旧的 metric=0 路由抢先。 */
	FILE *routes = fopen("/proc/net/route", "r");
	if (!routes)
		return -1;
	char line[512], route_name[IFNAMSIZ];
	unsigned long destination, gateway;
	unsigned flags, metric;
	while (fgets(line, sizeof(line), routes)) {
		if (sscanf(line, "%15s %lx %lx %x %*u %*u %u", route_name, &destination, &gateway, &flags, &metric) !=
			    5 ||
		    strcmp(route_name, name) || destination || !(flags & 2) || !(flags & 1))
			continue;
		if (interface->gateway) {
			fclose(routes);
			return -1;
		}
		interface->gateway = ntohl((uint32_t)gateway);
		interface->route_metric = metric;
	}
	fclose(routes);
	interface->arp_fd = socket(AF_PACKET, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, htons(ETH_P_ARP));
	struct sockaddr_ll link = { .sll_family = AF_PACKET,
				    .sll_protocol = htons(ETH_P_ARP),
				    .sll_ifindex = (int)interface->index };
	if (interface->arp_fd < 0 || bind(interface->arp_fd, (struct sockaddr *)&link, sizeof(link)))
		return -1;
	return 0;
}
int net_interface_link(const NetInterface *interface)
{
	if (interface->simulation)
		return 1;
	int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
	struct ifreq request = { 0 };
	strcpy(request.ifr_name, interface->interface);
	if (fd < 0)
		return 0;
	int linked = !ioctl(fd, SIOCGIFFLAGS, &request) && (request.ifr_flags & IFF_RUNNING);
	close(fd);
	return linked;
}
int net_interface_apply(NetInterface *interface, uint32_t address, unsigned prefix, uint32_t gateway)
{
	if (interface->address == address && interface->prefix == prefix && interface->gateway == gateway)
		return 0;
	uint32_t previous = interface->address, previous_gateway = interface->gateway;
	unsigned previous_prefix = interface->prefix;
	unsigned previous_metric = interface->route_metric;
	int address_changed = previous != address || previous_prefix != prefix;
	if (!interface->simulation) {
		if (previous_gateway && route_request(interface->index, 0, 0, previous_gateway, previous_metric, 0))
			return -1;
		if (address_changed && previous && route_request(interface->index, previous, previous_prefix, 0, 0, 0))
			goto restore;
		if (address_changed && address && route_request(interface->index, address, prefix, 0, 0, 1))
			goto restore;
		if (gateway && route_request(interface->index, 0, 0, gateway, 512, 1)) {
			if (address_changed)
				route_request(interface->index, address, prefix, 0, 0, 0);
			goto restore;
		}
	}
	interface->address = address;
	interface->prefix = prefix;
	interface->gateway = gateway;
	interface->route_metric = gateway ? 512 : 0;
	return 0;
restore:
	if (previous)
		route_request(interface->index, previous, previous_prefix, 0, 0, 1);
	if (previous_gateway)
		route_request(interface->index, 0, 0, previous_gateway, previous_metric, 1);
	return -1;
}
int net_arp_send(const NetInterface *interface, uint32_t address, int probe)
{
	if (interface->simulation)
		return 0;
	unsigned char packet[28] = { 0, 1, 8, 0, 6, 4, 0, 1 };
	uint32_t ip = htonl(address);
	memcpy(packet + 8, interface->mac, 6);
	if (!probe)
		memcpy(packet + 14, &ip, 4);
	memcpy(packet + 24, &ip, 4);
	struct sockaddr_ll destination = { .sll_family = AF_PACKET,
					   .sll_protocol = htons(ETH_P_ARP),
					   .sll_ifindex = (int)interface->index,
					   .sll_halen = 6 };
	memset(destination.sll_addr, 255, 6);
	return sendto(interface->arp_fd, packet, sizeof(packet), 0, (struct sockaddr *)&destination,
		      sizeof(destination)) == sizeof(packet) ?
		       0 :
		       -1;
}
int net_arp_read(const NetInterface *interface, unsigned char packet[28])
{
	unsigned char buffer[2048];
	struct sockaddr_ll from;
	socklen_t length = sizeof(from);
	ssize_t size = recvfrom(interface->arp_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &length);
	if (size < 0)
		return -1;
	if (size < 28 || from.sll_ifindex != (int)interface->index || from.sll_pkttype == PACKET_OUTGOING)
		return 0;
	memcpy(packet, buffer, 28);
	return 1;
}
int net_arp_conflict(const unsigned char *packet, size_t size, const uint8_t mac[6], uint32_t address, int probing)
{
	const unsigned char header[6] = { 0, 1, 8, 0, 6, 4 };
	if (!address || size < 28 || memcmp(packet, header, 6) || packet[6] || (packet[7] != 1 && packet[7] != 2) ||
	    !memcmp(packet + 8, mac, 6))
		return 0;
	uint32_t sender, target;
	memcpy(&sender, packet + 14, 4);
	memcpy(&target, packet + 24, 4);
	return ntohl(sender) == address || (probing && !sender && ntohl(target) == address && packet[7] == 1);
}
void net_probe_start(NetProbe *probe, uint32_t address, int64_t now, uint32_t random)
{
	*probe = (NetProbe){ .address = address, .next = now + random % 1001u };
}
unsigned net_probe_tick(NetProbe *probe, int64_t now, uint32_t random)
{
	if (!probe->address || now < probe->next)
		return 0;
	if (probe->probes < 3) {
		++probe->probes;
		probe->next = now + (probe->probes == 3 ? 2000 : 1000 + random % 1001u);
		return NET_PROBE_SEND;
	}
	if (!probe->ready) {
		probe->ready = 1;
		probe->announcements = 1;
		probe->next = now + 2000;
		return NET_PROBE_READY | NET_ANNOUNCE_SEND;
	}
	if (probe->announcements == 1) {
		probe->announcements = 2;
		probe->next = INT64_MAX;
		return NET_ANNOUNCE_SEND;
	}
	return 0;
}
int net_import_legacy(const char *path, const char *interface, NetConfig *config)
{
	FILE *file = fopen(path, "r");
	if (!file)
		return errno == ENOENT ? 1 : -1;
	char line[512], key[64], value[128], name[64], family[32], mode[32];
	int selected = 0, found = 0, invalid = 0, static_found = 0;
	cJSON *object = cJSON_CreateObject();
	cJSON_AddStringToObject(object, "mode", "static");
	while (fgets(line, sizeof(line), file)) {
		if (sscanf(line, " %63s", key) == 1 &&
		    (!strcmp(key, "source") || !strcmp(key, "source-directory") || !strcmp(key, "mapping")))
			invalid = 1;
		if (sscanf(line, "iface %63s %31s %31s", name, family, mode) == 3) {
			selected = !strcmp(name, interface) && !strcmp(family, "inet");
			if (selected) {
				if (found++)
					invalid = 1;
				if (!strcmp(mode, "dhcp") || !strcmp(mode, "manual"))
					selected = 0;
				else if (strcmp(mode, "static"))
					invalid = 1;
				else
					static_found = 1;
			}
			continue;
		}
		if (selected && sscanf(line, " %63s %127s", key, value) == 2 &&
		    (!strcmp(key, "address") || !strcmp(key, "netmask") || !strcmp(key, "gateway")))
			cJSON_AddStringToObject(object, key, value);
	}
	if (ferror(file))
		invalid = 1;
	fclose(file);
	char error[160];
	int has_static = cJSON_HasObjectItem(object, "address");
	if (static_found && !has_static)
		invalid = 1;
	if (!invalid && has_static)
		invalid = net_config_parse(object, config, error) != 0;
	cJSON_Delete(object);
	return invalid ? -1 : has_static ? 0 : found ? 2 : 1;
}
