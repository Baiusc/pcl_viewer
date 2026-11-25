/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-28 23:25:41
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-03-21 03:06:11
 * @FilePath     : /you_see_arm/core/ys_core_network.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#define _GNU_SOURCE

#include "ys_core_network.h"

#include "../common/common.h"

#include <netinet/in.h>
#include <net/if.h>
#include <sys/ioctl.h>

int get_ip_via_interface(const char *iface, uint8_t *dest)
{
	int fd;
	struct ifreq ifr;

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		log_error("Socket creation error");
		return RET_FAIL;
	}

	memset(&ifr, 0, sizeof(struct ifreq));
	ifr.ifr_addr.sa_family = AF_INET;
	strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);

	// 获取 IP 地址
	if (ioctl(fd, SIOCGIFADDR, &ifr) < 0) {
		log_error("ioctl get ip error");
		if (fd) {
			close(fd);
		}
		return RET_FAIL;
	}

	struct sockaddr_in *addr_in = (struct sockaddr_in *)&ifr.ifr_addr;

	dest[0] = (unsigned char)(addr_in->sin_addr.s_addr & 0xFF);
	dest[1] = (unsigned char)((addr_in->sin_addr.s_addr >> 8) & 0xFF);
	dest[2] = (unsigned char)((addr_in->sin_addr.s_addr >> 16) & 0xFF);
	dest[3] = (unsigned char)((addr_in->sin_addr.s_addr >> 24) & 0xFF);

	close(fd);
	return RET_OK;
}

int get_mask_via_interface(const char *iface, uint8_t *dest)
{
	int fd;
	struct ifreq ifr;

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		log_error("Socket creation error");
		return RET_FAIL;
	}

	memset(&ifr, 0, sizeof(struct ifreq));
	ifr.ifr_addr.sa_family = AF_INET;
	strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);

	// 获取子网掩码
	if (ioctl(fd, SIOCGIFNETMASK, &ifr) < 0) {
		log_error("ioctl get mask error");
		if (fd) {
			close(fd);
		}
		return RET_FAIL;
	}

	struct sockaddr_in *addr_in = (struct sockaddr_in *)&ifr.ifr_netmask;

	dest[0] = (unsigned char)(addr_in->sin_addr.s_addr & 0xFF);
	dest[1] = (unsigned char)((addr_in->sin_addr.s_addr >> 8) & 0xFF);
	dest[2] = (unsigned char)((addr_in->sin_addr.s_addr >> 16) & 0xFF);
	dest[3] = (unsigned char)((addr_in->sin_addr.s_addr >> 24) & 0xFF);

	close(fd);
	return RET_OK;
}

int get_gateway_via_interface(const char *iface, uint8_t *dest)
{
	if (!iface || !dest || !if_nametoindex(iface))
		return RET_FAIL;
	memset(dest, 0, 4);
	FILE *file = fopen("/proc/net/route", "r");
	if (!file)
		return RET_FAIL;
	char line[512], name[IFNAMSIZ];
	unsigned long destination, gateway;
	unsigned flags;
	/* 没有默认网关是合法的直连状态，返回 0.0.0.0。 */
	while (fgets(line, sizeof(line), file)) {
		if (sscanf(line, "%15s %lx %lx %x", name, &destination, &gateway, &flags) == 4 &&
		    !strcmp(name, iface) && !destination && (flags & 3) == 3) {
			uint32_t encoded = (uint32_t)gateway;
			memcpy(dest, &encoded, 4);
			break;
		}
	}
	int failed = ferror(file);
	fclose(file);
	return failed ? RET_FAIL : RET_OK;
}

int get_mac_via_interface(const char *iface, uint8_t *dest)
{
	int fd;
	struct ifreq ifr;

	fd = socket(AF_INET, SOCK_DGRAM, 0);

	ifr.ifr_addr.sa_family = AF_INET;
	strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);

	// 获取 MAC 地址
	if (ioctl(fd, SIOCGIFHWADDR, &ifr) < 0) {
		log_error("ioctl get mac error");
		if (fd) {
			close(fd);
		}
		return RET_FAIL;
	}
	log_info("MAC 地址：%02x:%02x:%02x:%02x:%02x:%02x", (unsigned char)ifr.ifr_hwaddr.sa_data[0],
		 (unsigned char)ifr.ifr_hwaddr.sa_data[1], (unsigned char)ifr.ifr_hwaddr.sa_data[2],
		 (unsigned char)ifr.ifr_hwaddr.sa_data[3], (unsigned char)ifr.ifr_hwaddr.sa_data[4],
		 (unsigned char)ifr.ifr_hwaddr.sa_data[5]);

	dest[0] = (unsigned char)ifr.ifr_hwaddr.sa_data[0];
	dest[1] = (unsigned char)ifr.ifr_hwaddr.sa_data[1];
	dest[2] = (unsigned char)ifr.ifr_hwaddr.sa_data[2];
	dest[3] = (unsigned char)ifr.ifr_hwaddr.sa_data[3];
	dest[4] = (unsigned char)ifr.ifr_hwaddr.sa_data[4];
	dest[5] = (unsigned char)ifr.ifr_hwaddr.sa_data[5];

	close(fd);
	return RET_OK;
}
