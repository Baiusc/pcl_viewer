/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-28 23:44:20
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-10-12 20:09:57
 * @FilePath     : /you_see_arm/ys_business/ys_business_network.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#define _GNU_SOURCE

#include "ys_business_network.h"
#include "ys_errno.h"
#include "ys_msg_distribute.h"
#include "ys_ps_config.h"
#include "ys_state_machine.h"

#include "../core/ys_base.h"
#include "../core/ys_core_network.h"
#include "../core/ys_core_system.h"
#include "../core/ys_file.h"
#include "../core/ys_udp.h"
#include "../core/ys_version.h"
#include "../network/network.h"

#include "../common/common.h"
#include "../common/ys_protocal.h"

#include <sys/socket.h>
#include <error.h>
#include <time.h>

#define MMSG_MAX_CNT 16

udp_t udp_cmd_instance;
udp_t udp_data_instance;

/* UDP 数据通路发送长度分布统计(记录实际包长, 用于核对同机型包长是否固定)
 * 最多记录 UDP_LEN_KIND_MAX 种不同长度, 超出部分归入 other */
#define UDP_LEN_KIND_MAX 8
typedef struct {
	uint32_t len; // 包长, 0 表示空槽
	uint64_t cnt; // 该包长出现次数
} udp_len_stat_t;
static udp_len_stat_t g_udp_len_stat[UDP_LEN_KIND_MAX];
static uint64_t g_udp_len_other;
static struct timespec g_udp_stat_ts;

#define UDP_STAT_INTERVAL_S 5 // 统计打印间隔

static int init_socket(udp_t *net, configuration_t *config);

#if USE_SENDMMSG_FUNC == 1
static int storage_cnt; // 用于统计当前缓存中存储的包数量
static struct iovec iovs[MMSG_MAX_CNT];
static struct mmsghdr mmsg[MMSG_MAX_CNT];
#endif

/**********************************************************************************************/
/****************************************网络相关指令接口****************************************/
/**********************************************************************************************/
static struct {
	int active;
	management_id_t message;
	char operation[33];
	int64_t deadline;
} pending_network;
static cJSON *network_control(cJSON *request)
{
	const char *manager = getenv("LIDAR_MANAGER_SOCKET");
	const char *network = getenv("LIDAR_NETWORK_SOCKET");
	const char *operation = net_string(request, "op");
	const char *path = manager ? manager : network;
	int timeout = operation && !strcmp(operation, "network_confirm") ? 7000 : 500;
	return net_rpc(path ? path : NET_SOCKET_DEFAULT, request, timeout);
}
void set_ip_info(void *arg)
{
	if (!arg || !((management_id_t *)arg)->buf)
		return;
	management_id_t *msg = arg;
	if (pending_network.active) {
		uint8_t result = run_fail;
		pack(msg, &result, 1, ACK);
		return;
	}
	cJSON *request = cJSON_CreateObject(), *configuration = cJSON_CreateObject();
	cJSON_AddStringToObject(request, "op", "network_set");
	cJSON_AddBoolToObject(request, "legacy", 1);
	cJSON_AddStringToObject(configuration, "mode", "static");
	const char *keys[] = { "address", "netmask", "gateway" };
	for (unsigned i = 0; i < 3; ++i) {
		char text[16];
		const uint8_t *part = msg->buf + 2 + i * 4;
		snprintf(text, sizeof(text), "%u.%u.%u.%u", part[0], part[1], part[2], part[3]);
		cJSON_AddStringToObject(configuration, keys[i], text);
	}
	cJSON_AddItemToObject(request, "config", configuration);
	cJSON *response = network_control(request);
	cJSON_Delete(request);
	const char *operation = net_string(response, "operation_id");
	char token[33] = "";
	if (operation && strlen(operation) == 32)
		strcpy(token, operation);
	int accepted = net_integer(cJSON_GetObjectItemCaseSensitive(response, "code"), 202, 202);
	cJSON_Delete(response);
	if (!accepted || !*token) {
		uint8_t result = run_fail;
		pack(msg, &result, 1, ACK);
		return;
	}
	/* 只保留响应所需的本地元数据，不持有解码器即将释放的请求缓冲区。 */
	pending_network.message = *msg;
	pending_network.message.buf = NULL;
	strcpy(pending_network.operation, token);
	pending_network.deadline = net_now() + 10000;
	pending_network.active = 1;
}
static void poll_legacy_network(void *arg)
{
	(void)arg;
	if (!pending_network.active)
		return;
	cJSON *request = cJSON_CreateObject();
	cJSON_AddStringToObject(request, "op", "network_get");
	cJSON *response = network_control(request);
	cJSON_Delete(request);
	const char *phase = net_string(response, "transaction_phase"),
		   *operation = net_string(response, "operation_id");
	int prepared = phase && operation && !strcmp(operation, pending_network.operation) &&
		       !strcmp(phase, "PREPARED");
	int waiting = phase && operation && !strcmp(operation, pending_network.operation) && !strcmp(phase, "PROBING");
	cJSON_Delete(response);
	if (waiting && net_now() < pending_network.deadline)
		return;
	uint8_t result = prepared ? run_succ : run_fail;
	pending_network.active = 0;
	/* 探测期间主循环继续处理心跳；确认旧地址回包后才应用新地址。 */
	if (pack(&pending_network.message, &result, 1, ACK) != RET_OK || !prepared)
		return;
	request = cJSON_CreateObject();
	cJSON_AddStringToObject(request, "op", "network_apply");
	cJSON_AddStringToObject(request, "operation_id", pending_network.operation);
	response = network_control(request);
	cJSON_Delete(response);
	cJSON_Delete(request);
}

static void confirm_legacy_network(void)
{
	if (!getenv("LIDAR_NETWORK_SOCKET"))
		return;
	cJSON *request = cJSON_CreateObject();
	cJSON_AddStringToObject(request, "op", "network_get");
	cJSON *response = network_control(request);
	cJSON_Delete(request);
	const char *phase = net_string(response, "transaction_phase");
	const char *operation = net_string(response, "operation_id");
	uint32_t local = udp_cmd_instance.udpc.center.received_local_address;
	/* 只使用本次 recvmsg 的真实目标地址，不能用当前接口地址猜测请求来源。 */
	if (phase && operation && !strcmp(phase, "WAIT_CONFIRM") &&
	    cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(response, "legacy")) && local) {
		char text[16];
		inet_ntop(AF_INET, &local, text, sizeof(text));
		request = cJSON_CreateObject();
		cJSON_AddStringToObject(request, "op", "network_confirm");
		cJSON_AddStringToObject(request, "operation_id", operation);
		cJSON_AddStringToObject(request, "via_address", text);
		cJSON *confirmed = network_control(request);
		cJSON_Delete(confirmed);
		cJSON_Delete(request);
	}
	cJSON_Delete(response);
}
void get_ip_info(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	ip_info_t ack_data = { 0 };
	int ret = RET_OK;

	ret += get_ip_via_interface(PHY_IFACE, ack_data.ip_addr);
	ret += get_mask_via_interface(PHY_IFACE, ack_data.net_mask);
	ret += get_gateway_via_interface(PHY_IFACE, ack_data.gw_addr);

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	ack_data.ret_code = (uint8_t)ret;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

void get_mac_address(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	mac_info_t ack_data = { 0 };
	int ret = get_mac_via_interface(PHY_IFACE, ack_data.mac_addr);

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	ack_data.ret_code = (uint8_t)ret;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

void network_handshake(void *arg)
{
	char str_value[8];

	if (arg == NULL) {
		log_error("parameter is NULL");
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_FAIL;

	if (msg->buf == NULL) {
		log_error("parameter is NULL");
		return;
	}

	network_handshake_t *data = (network_handshake_t *)(msg->buf);

	pthread_mutex_lock(&(config_lock));

	// 合法性检查
	for (int i = 0; i < (sizeof(data->ip) / sizeof(data->ip[0])); i++) {
		if ((data->ip[i] == 0 && (i == 0 || i == 3)) || data->ip[i] == 255) {
			log_error("ip exceeds the value range");
			ret = RET_FAIL;
			goto end;
		}
	}

	// ip 首位不能大于 223
	if (data->ip[0] > 223) {
		log_error("ip exceeds the value range");
		ret = RET_FAIL;
		goto end;
	}

	// 更新 IP
	char ip[] = "255.255.255.255";
	snprintf(ip, sizeof(ip), "%d.%d.%d.%d", data->ip[0], data->ip[1], data->ip[2], data->ip[3]);
	snprintf(udp_cmd_instance.ser_info.ip, sizeof(udp_cmd_instance.ser_info.ip), "%s", ip);
	snprintf(udp_data_instance.ser_info.ip, sizeof(udp_data_instance.ser_info.ip), "%s", ip);
	snprintf(config.center_ip, sizeof(config.center_ip), "%s", ip);
	log_debug("set key %s value %s", "network:center_ip", ip);
	iniparser_set(conf_ini, "network:center_ip", ip);

	// 更新 指令端口
	uint16_t cmd_port = data->cmd_port;
	if (cmd_port > MAX_PORT || cmd_port < MIN_PORT) {
		log_error("cmd_port exceeds the value range: %d", cmd_port);
		ret = RET_FAIL;
		goto end;
	}
	udp_cmd_instance.ser_info.port = cmd_port;
	config.center_cmd_port = cmd_port;
	snprintf(str_value, sizeof(str_value), "%d", cmd_port);
	log_debug("set key %s value %s", "network:center_cmd_port", str_value);
	iniparser_set(conf_ini, "network:center_cmd_port", str_value);

	// 更新 数据端口
	uint16_t data_port = data->cloud_data_port;
	if (data_port == 0x0000) {
		data_port = data->org_data_port;
	}
	if (data_port > MAX_PORT || data_port < MIN_PORT) {
		log_error("data_port exceeds the value range");
		ret = RET_FAIL;
		goto end;
	}
	udp_data_instance.ser_info.port = data_port;
	config.center_data_port = data_port;
	snprintf(str_value, sizeof(str_value), "%d", data_port);
	log_debug("set key %s value %s", "network:center_data_port", str_value);
	iniparser_set(conf_ini, "network:center_data_port", str_value);

	pthread_mutex_unlock(&(config_lock));

	log_info("handshake %s:%d(cmd),%d(data)", ip, cmd_port, data_port);
	confirm_legacy_network();

	// 关闭 udp 广播
	pthread_mutex_lock(&(udp_cmd_instance.udpc.center.lock));
	udp_cmd_instance.udpc.center.is_handshaked = IS_REG;
	pthread_mutex_unlock(&(udp_cmd_instance.udpc.center.lock));
	ret = set_broadcast_mode_off(&(udp_cmd_instance.udpc.center));

	// 更新 udp 实例
	update_udp_config(&udp_cmd_instance);
	update_udp_config(&udp_data_instance);

	// 更新到文件中
	core_update_ini_file(conf_ini, ys_runtime_config_path());

end:
	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	pack(msg, (uint8_t *)&ret, 1, ACK);
	return;
}

void network_heartbeat(void *arg)
{
	if (arg == NULL) {
		log_error("parameter is NULL");
		return;
	}

	management_id_t *msg = (management_id_t *)arg;

	log_debug("heartbeat");
	pthread_mutex_lock(&(udp_cmd_instance.udpc.center.lock));
	udp_cmd_instance.udpc.center.heartbeat_cnt++;
	pthread_mutex_unlock(&(udp_cmd_instance.udpc.center.lock));

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	get_self_state(arg);
#else
	// MF150_FL6031 机型直接回复 ACK
	int ret = run_succ;
	pack(msg, (uint8_t *)&ret, 1, ACK);
#endif
	return;
}

void network_break_handshake(void *arg)
{
	if (arg == NULL) {
		log_error("parameter is NULL");
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_OK;

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	pack(msg, (uint8_t *)&ret, 1, ACK);

	// 开启 udp 广播
	pthread_mutex_lock(&(udp_cmd_instance.udpc.center.lock));
	udp_cmd_instance.udpc.center.is_handshaked = NO_REG;
	pthread_mutex_unlock(&(udp_cmd_instance.udpc.center.lock));
	set_broadcast_mode_on(&(udp_cmd_instance.udpc.center));
	return;
}
/**********************************************************************************************/
/**************************************网络相关指令接口结束**************************************/
/**********************************************************************************************/

/**********************************************************************************************/
/*****************************************业务相关函数*****************************************/
/**********************************************************************************************/
int respond_command(uint8_t *data, int len)
{
	if (len < 0) {
		return RET_FAIL;
	}
	int status = 0;
	uv_buf_t buf = uv_buf_init((char *)data, len);

	if (udp_cmd_instance.udpc.center.is_connected == CONNECTED) {
		hex_to_string("send", data, len);
		status = uv_udp_try_send(&(udp_cmd_instance.udpc.center.uv_handle), &buf, 1,
					 (const struct sockaddr *)&(udp_cmd_instance.udpc.center.addr));
		if (status < 0) {
			log_error("UDP: Fail to send data to server, error: %s", uv_strerror(status));
		}
	}

	return status;
}

static char control[4096];
char buf[1];
struct iovec iov = { .iov_base = buf, .iov_len = sizeof(buf) };
struct msghdr msg = { .msg_control = control, .msg_controllen = sizeof(control), .msg_iov = &iov, .msg_iovlen = 1 };

#if USE_ZERO_COPY_SENDTO == 1
static void clear_zerocopy_notifications(int sock_fd)
{
	// 非阻塞模式处理通知
	// int flags = fcntl(sock_fd, F_GETFL);
	// fcntl(sock_fd, F_SETFL, flags | O_NONBLOCK);

	// 清理所有待处理的完成通知
	while (recvmsg(sock_fd, &msg, MSG_ERRQUEUE) >= 0) {
		// 只要接收了通知，相应的资源就会被释放
	}

	// 恢复原来的标志
	// fcntl(sock_fd, F_SETFL, flags);
}
#endif

/**
 * @description  : send data via socket by udp
 * @param         {udp_h} *net
 * @param         {uint8_t} *data
 * @param         {int} len
 * @return        {*}
 */
#if USE_SOCKET == 1
static int send_data_via_socket_udp(struct udp_h *net, uint8_t *data, int len)
{
#if 0 // 直接发送（已弃用）
	return sendto(net->socket_fd, data, len, MSG_DONTWAIT, (struct sockaddr *)&(net->client_addr), sizeof(net->client_addr));
#elif USE_SENDMMSG_FUNC == 1
	int ret = RET_FAIL;

	switch (storage_cnt) {
	case 1 ... 14:
		// 非常重要，如果没清空会报错 No buffer space available
		memset(&mmsg[storage_cnt], 0, sizeof(mmsg[storage_cnt]));
		memset(&iovs[storage_cnt], 0, sizeof(iovs[storage_cnt]));
		iovs[storage_cnt].iov_base = data;
		iovs[storage_cnt].iov_len = len;
		mmsg[storage_cnt].msg_hdr.msg_name = &(net->client_addr);
		mmsg[storage_cnt].msg_hdr.msg_namelen = sizeof(net->client_addr);
		mmsg[storage_cnt].msg_hdr.msg_iov = &iovs[storage_cnt];
		mmsg[storage_cnt].msg_hdr.msg_iovlen = 1;

		storage_cnt++;
		ret = RET_OK; // 正常的缓存数据
		break;
	case 0:
		// 非常重要，如果没清空会报错 No buffer space available
		memset(&mmsg[storage_cnt], 0, sizeof(mmsg[storage_cnt]));
		memset(&iovs[storage_cnt], 0, sizeof(iovs[storage_cnt]));
		iovs[storage_cnt].iov_base = data;
		iovs[storage_cnt].iov_len = len;
		mmsg[storage_cnt].msg_hdr.msg_name = &(net->client_addr);
		mmsg[storage_cnt].msg_hdr.msg_namelen = sizeof(net->client_addr);
		mmsg[storage_cnt].msg_hdr.msg_iov = &iovs[storage_cnt];
		mmsg[storage_cnt].msg_hdr.msg_iovlen = 1;

		storage_cnt++;
		ret = 1; // 提示要保存
		break;
	case 15:
		ret = sendmmsg(net->socket_fd, mmsg, MMSG_MAX_CNT, 0);
		storage_cnt = 0;
		break;
	}

	return ret;
#else
#ifdef MSG_ZEROCOPY
	// 零拷贝发送
	int send_len = sendto(net->socket_fd, data, len, MSG_ZEROCOPY, (struct sockaddr *)&(net->client_addr),
			      sizeof(net->client_addr));
	if (send_len < 0) {
		if (errno == ENOBUFS) {
			// 当遇到 buffer space 不足时，清理完成通知
			clear_zerocopy_notifications(net->socket_fd);
			// 重试发送
			send_len = sendto(net->socket_fd, data, len, MSG_ZEROCOPY,
					  (struct sockaddr *)&(net->client_addr), sizeof(net->client_addr));
			if (send_len < 0) {
				return RET_FAIL;
			}
		} else {
			return RET_FAIL;
		}
	}

	return send_len;
#else // 分片发送，防止 MTU 丢包
	return sendto(net->socket_fd, data, len, MSG_DONTWAIT, (struct sockaddr *)&(net->client_addr),
		      sizeof(net->client_addr));

	// static struct iovec t_iovs[3];
	// static struct mmsghdr t_mmsg[3];

	// void *head[] = { 00, 01 };

	// memset(&t_mmsg[0], 0, sizeof(t_mmsg[0]));
	// memset(&t_iovs[0], 0, sizeof(t_iovs[0]));
	// t_iovs[0].iov_base = data;
	// t_iovs[0].iov_len = 1450;
	// t_mmsg[0].msg_hdr.msg_name = &(net->client_addr);
	// t_mmsg[0].msg_hdr.msg_namelen = sizeof(net->client_addr);
	// t_mmsg[0].msg_hdr.msg_iov = &t_iovs[0];
	// t_mmsg[0].msg_hdr.msg_iovlen = 1;

	// memset(&t_mmsg[1], 0, sizeof(t_mmsg[1]));
	// memset(&t_iovs[1], 0, sizeof(t_iovs[1]));
	// // memcpy: head + data + 1450
	// void *pkg_2 = data + 1450;
	// t_iovs[1].iov_base = pkg_2;
	// t_iovs[1].iov_len = 1450;
	// t_mmsg[1].msg_hdr.msg_name = &(net->client_addr);
	// t_mmsg[1].msg_hdr.msg_namelen = sizeof(net->client_addr);
	// t_mmsg[1].msg_hdr.msg_iov = &t_iovs[1];
	// t_mmsg[1].msg_hdr.msg_iovlen = 1;

	// memset(&t_mmsg[2], 0, sizeof(t_mmsg[2]));
	// memset(&t_iovs[2], 0, sizeof(t_iovs[2]));
	// // memcpy: head + data + 1450 + 1450
	// void *pkg_3 = data + 1450 + 1450;
	// t_iovs[2].iov_base = pkg_3;
	// t_iovs[2].iov_len = 1109;
	// t_mmsg[2].msg_hdr.msg_name = &(net->client_addr);
	// t_mmsg[2].msg_hdr.msg_namelen = sizeof(net->client_addr);
	// t_mmsg[2].msg_hdr.msg_iov = &t_iovs[2];
	// t_mmsg[2].msg_hdr.msg_iovlen = 1;

	// sendmmsg(net->socket_fd, t_mmsg, 3, 0);
	// return 4009;
#endif
#endif
}

#endif

/**
 * @description  : send data via socket by tcp
 * @param         {udp_h} *net
 * @param         {uint8_t} *data
 * @param         {int} len
 * @return        {*}
 */
#if USE_SOCKET == 1
static int send_data_via_socket_tcp(struct udp_h *net, uint8_t *data, int len)
{
	return send(net->socket_fd, data, len, MSG_DONTWAIT);
}
#endif

/* 记录一个已发送包的长度 */
static void udp_len_stat_record(int len)
{
	for (int i = 0; i < UDP_LEN_KIND_MAX; i++) {
		if (g_udp_len_stat[i].cnt == 0) {
			g_udp_len_stat[i].len = (uint32_t)len;
			g_udp_len_stat[i].cnt = 1;
			return;
		}
		if (g_udp_len_stat[i].len == (uint32_t)len) {
			g_udp_len_stat[i].cnt++;
			return;
		}
	}
	g_udp_len_other++;
}

/* 打印并清零本统计周期内的包长分布 */
static void udp_len_stat_print(uint64_t elapsed_ms)
{
	uint64_t total = g_udp_len_other;
	for (int i = 0; i < UDP_LEN_KIND_MAX; i++) {
		total += g_udp_len_stat[i].cnt;
	}

	if (total > 0) {
		char detail[320] = { '\0' };
		int off = 0;
		for (int i = 0; i < UDP_LEN_KIND_MAX; i++) {
			if (g_udp_len_stat[i].cnt == 0) {
				continue;
			}
			int n = snprintf(detail + off, sizeof(detail) - (size_t)off, " len%u=%llu(%.2f%%)",
					 g_udp_len_stat[i].len, (unsigned long long)g_udp_len_stat[i].cnt,
					 100.0 * g_udp_len_stat[i].cnt / total);
			if (n < 0 || n >= (int)(sizeof(detail) - (size_t)off)) {
				break;
			}
			off += n;
		}
		log_info("[UDP-ST] %llu pkts/%.1fs:%s other=%llu(%.2f%%)", (unsigned long long)total,
			 (double)elapsed_ms / 1000.0, detail, (unsigned long long)g_udp_len_other,
			 100.0 * g_udp_len_other / total);
	}

	memset(g_udp_len_stat, 0, sizeof(g_udp_len_stat));
	g_udp_len_other = 0;
}

int send_data(udp_t *udp, uint8_t *data, int len)
{
	/* 统计数据通路的发送包长分布(周期清零打印, 用于核对同机型包长是否固定) */
	if (udp == &udp_data_instance) {
		udp_len_stat_record(len);

		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		if (g_udp_stat_ts.tv_sec == 0) {
			g_udp_stat_ts = now;
		} else {
			/* 纳秒差可能为负, 必须用有符号计算后再判断 */
			int64_t elapsed_ms = (int64_t)(now.tv_sec - g_udp_stat_ts.tv_sec) * 1000 +
					     (now.tv_nsec - g_udp_stat_ts.tv_nsec) / 1000000;
			if (elapsed_ms >= (int64_t)UDP_STAT_INTERVAL_S * 1000) {
				udp_len_stat_print((uint64_t)elapsed_ms);
				g_udp_stat_ts = now;
			}
		}
	}

	int ret = RET_FAIL;
#if USE_SOCKET == 1
	ret = udp->send_data_via_socket(udp, data, len);
#else
	ret = send_buf_to_center(&(udp->udpc.center), data, len);
#endif
	return ret;
}

void check_network()
{
	uint8_t address[4];
	/* 网卡与路由由系统网络服务管理，采集启动只读取状态。 */
	int ret = get_ip_via_interface(PHY_IFACE, address);

	if (ret == RET_OK) {
		log_info("Network check success");
	} else {
		log_error("Network check fail");
	}
	return;
}

/**
 * @description  : 创建广播包
 * @param         {int} proto_version
 * @return        {int} buf len
 */
static int proto_build_borad(int proto_version, uint8_t **buf)
{
	int len = 0;
	boradcast_info_t boradcast_info;
	uint8_t msg_data[MSG_BUF_MAX_LEN] = { '\0' };
	management_id_t msg;
	memset(&msg, 0, sizeof(msg));

	switch (proto_version) {
	case 0x01:
		msg.code_set = CMD_SET_COMMON;
		msg.code_id = MSG_ID_BROADCAST;
		msg.version = (uint8_t)proto_version;

		char sn[NET_SN_CAPACITY];
		if (net_sn_read("/dev/mtd3", sn) == 0) {
			memcpy(boradcast_info.sn, sn, sizeof(boradcast_info.sn));
		} else {
			uint8_t mac[6] = { 0 };
			get_mac_via_interface(PHY_IFACE, mac);
			snprintf((char *)boradcast_info.sn, sizeof(boradcast_info.sn), "MAC-%02X%02X%02X%02X%02X%02X00",
				 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		}
		boradcast_info.work_mode = config.work_mode;
		boradcast_info.echo_mode = config.echo_mode;
		boradcast_info.echo_type = config.echo_type;
		boradcast_info.coordinate_mode = config.coordinate_mode;
		boradcast_info.line_counts = cal_param.lines_v;
		boradcast_info.slot_counts = cal_param.lines_h;
		len = proto_version_1_build(&msg, (uint8_t *)&boradcast_info, sizeof(boradcast_info_t), msg_data, MSG,
					    0);
		break;
	default:
		*buf = NULL;
		break;
	}

	*buf = (uint8_t *)Malloc(len);
	if (*buf != NULL) {
		memcpy(*buf, msg_data, len);
	} else {
		len = 0;
	}
	return len;
}

/**
 * @description  : transform data to a new node
 * @param         {udp_t} *udp_instance
 * @param         {uint8_t} *data
 * @param         {int} datalen
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int transform_data_to_new_node(udp_t *udp_instance, uint8_t *data, int datalen)
{
	node_t *p_node = NULL;
	p_node = (node_t *)allocate_memory(udp_instance->pool);
	if (p_node == NULL) {
		log_fatal("Allocate memory fail");
		goto fail;
	}

	if (p_node->data == NULL) {
		log_fatal("p_node->data is NULL");
		goto fail;
	}

	if (p_node->data->buf == NULL) {
		log_fatal("p_node->data->buf is NULL");
		goto fail;
	}

	Memcpy(p_node->data->buf, data, datalen);
	atomic_store(&(p_node->write_ok), READABLE);

	return RET_OK;

fail:
	return RET_FAIL;
}

/**
 * @description  : 存储从本地->网络的数据，对于下位机来说是输出，output
 * @param         {void} *arg
 * @param         {uint8_t} *data
 * @param         {int} datalen
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int default_deal_data_to_output_link(void *arg, uint8_t *data, int datalen)
{
	if (arg == NULL) {
		log_error("parameter is NULL");
		return RET_FAIL;
	}

	udp_t *udp_instance = (udp_t *)arg;
	int ret = RET_FAIL;

	ret = transform_data_to_new_node(udp_instance, data, datalen);
	if (ret != RET_OK) {
		log_fatal("transform data to a new node fail");
		return RET_FAIL;
	}

	return RET_OK;
}
/**********************************************************************************************/
/***************************************业务相关函数结束***************************************/
/**********************************************************************************************/

/**********************************************************************************************/
/****************************************构造、析构接口****************************************/
/**********************************************************************************************/
/**
 * @description  : init socket
 * @param         {udp_t} *net
 * @param         {configuration_t} *config
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
#if USE_SOCKET == 1
static int init_socket(udp_t *net, configuration_t *config)
{
	/* 数据通路单帧突发可达数 MB(Range 压缩流), 256KB 缓冲会导致 sendto
	 * 因缓冲满失败(SndbufErrors)并拖低帧率; 需配合 net.core.wmem_max */
	int sendbuf = 4 * 1024 * 1024; // 4MB
	int zero_copy = 1;
	int val = 1;

	net->socket_host = gethostbyname(net->ser_info.ip);
	net->client_addr.sin_family = AF_INET;
	net->client_addr.sin_addr = *((struct in_addr *)(net->socket_host->h_addr_list[0]));
	net->client_addr.sin_port = htons(net->ser_info.port);

	if (config->protocol == UDP) {
		net->socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

		setsockopt(net->socket_fd, SOL_SOCKET, SO_SNDBUF, &sendbuf, sizeof(sendbuf));
#ifdef SO_ZEROCOPY
		setsockopt(net->socket_fd, SOL_SOCKET, SO_ZEROCOPY, &zero_copy, sizeof(zero_copy));
#endif
		setsockopt(net->socket_fd, SOL_IP, IP_RECVERR, &val, sizeof(val));
		net->send_data_via_socket = send_data_via_socket_udp;
	} else if (config->protocol == TCP) {
		net->socket_fd = socket(AF_INET, SOCK_STREAM, 0);
		if (connect(net->socket_fd, (struct sockaddr *)&net->client_addr, sizeof(net->client_addr)) < 0) {
			log_error("Connection failed");
			close(net->socket_fd);
			return RET_FAIL;
		}
		net->send_data_via_socket = send_data_via_socket_tcp;
	} else {
		log_error("unknow protocol");
		return RET_FAIL;
	}

	return RET_OK;
}
#endif

/**
 * @description  : uninit socket
 * @param         {udp_t} *net
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
#if USE_SOCKET == 1
static int uninit_socket(udp_t *net)
{
	if (net->socket_fd > 0) {
		close(net->socket_fd);
	}

	return RET_OK;
}
#endif

/**
 * @description  : initialization mutex lock
 * @param         {udp_t} *udp_instance
 * @return        {*}
 */
static void init_mutex(udp_t *udp_instance)
{
	pthread_mutex_init(&(udp_instance->ser_info.lock), NULL);
	pthread_mutex_init(&(udp_instance->lock), NULL);
	pthread_mutex_init(&(udp_instance->udpc.center.lock), NULL);
	return;
}

/**
 * @description  : initialization udp instance
 * @param         {udp_t} *udp_instance
 * @return        {int} result
 */
static int initialize(udp_t *udp_instance, size_t size)
{
	memset(udp_instance, 0, sizeof(udp_t));

	init_mutex(udp_instance);

	// 初始化内存池
	udp_instance->pool = create_memory_pool(size);
	if (udp_instance->pool == NULL) {
		return RET_FAIL;
	}
	return RET_OK;
}

/**
 * @description  : destroy mutex lock
 * @param         {udp_t} *udp_instance
 * @return        {*}
 */
static void destroy_mutex(udp_t *udp_instance)
{
	pthread_mutex_destroy(&(udp_instance->udpc.center.lock));
	pthread_mutex_destroy(&(udp_instance->ser_info.lock));
	pthread_mutex_destroy(&(udp_instance->lock));
	return;
}

/**
 * @description  : destroy udp instance
 * @param         {udp_t} *udp_instance
 * @return        {*}
 */
static void destroy(udp_t *udp_instance)
{
	Free((void **)&(udp_instance->boradcast_buf));
	destroy_mutex(udp_instance);

#if USE_SOCKET == 1
	uninit_socket(udp_instance);
#endif

	// 最后才释放内存池
	if (udp_instance->pool != NULL) {
		destroy_memory_pool(udp_instance->pool);
	}
	return;
}

int init_udp(calibration_parm_t *cal_param, configuration_t *config)
{
	int memory_pool_cnt = 0; // 内存池的最大节点个数，取决于当前的工作模式

	if (config == NULL) {
		log_error("initialization udp error, unknow config");
		return RET_FAIL;
	}

	if (strlen(config->center_ip) <= 0) {
		log_error("initialization udp error, unknow ip address");
		return RET_FAIL;
	}

	// 初始化内存池大小和内存池节点大小
	switch (config->work_mode) {
	case RAW_MODE:
#if MF150_FL6031 == 1
		set_block_size(MF150_FL6031_HISTOGRAM_PACKET_SIZE);
		memory_pool_cnt = MF150_FL6031_HISTOGRAM_MEMORY_POOL_MAX_CNT;
#elif MF150_FL6532 == 1
		set_block_size(MF150_FL6532_HISTOGRAM_PACKET_SIZE);
		memory_pool_cnt = MF150_FL6532_HISTOGRAM_MEMORY_POOL_MAX_CNT;
#else
		set_block_size(raw_histogram_full_size);
		memory_pool_cnt = ORG_HISTOGRAM_MEMORY_POOL_MAX_CNT;
#endif
		break;
	case TEST_POINT_CLOUD_MODE:
		set_block_size(raw_point_cloud_full_size);
#if MF150_FL6532 == 1
		memory_pool_cnt = MF150_FL6532_POINT_CLOUD_MEMORY_POOL_MAX_CNT;
#else
		memory_pool_cnt = cal_param->lines_h * 10;
#endif
		break;
	case FORMAL_POINT_CLOUD_MODE:
		set_block_size(raw_point_cloud_full_size);
#if MF150_FL6532 == 1
		memory_pool_cnt = MF150_FL6532_POINT_CLOUD_MEMORY_POOL_MAX_CNT;
#else
		memory_pool_cnt = cal_param->lines_h * 10;
#endif
		break;
	case GRAY_MODE:
		// 灰度图: 一个 SLOT 一个包(977B), 透传发送
		set_block_size(raw_gray_full_size);
		memory_pool_cnt = ORG_GRAY_MEMORY_POOL_MAX_CNT;
		break;

	default:
		return RET_FAIL;
	}

	// 初始化 udp 指令实例
	if (initialize(&udp_cmd_instance, 16) != RET_OK) {
		return RET_FAIL;
	}

	snprintf(udp_cmd_instance.ser_info.ip, sizeof(udp_cmd_instance.ser_info.ip), "%s", config->center_ip);
	udp_cmd_instance.ser_info.port = config->center_cmd_port;
	udp_cmd_instance.ser_info.broadcast_port = config->center_broadcast_port;
	udp_cmd_instance.ser_info.local_port = config->local_cmd_port;
	udp_cmd_instance.simplex = DUPLEX;
	udp_cmd_instance.boradcast_buf = NULL;
	/* 协议主版本以编译期常量为准, 不再读取配置文件: 配置值一旦写错, proto_build_borad 会走
	 * default 分支导致广播长度为 0, 广播静默消失、上位机再也发现不了设备 */
	udp_cmd_instance.boradcast_buf_len =
		proto_build_borad((int)ys_version_protocol_major(), &(udp_cmd_instance.boradcast_buf));
	udp_cmd_instance.deal_data_to_input_link = parse_cmd_data;
	udp_cmd_instance.network_maintenance = poll_legacy_network;
	udp_cmd_instance.deal_data_to_output_link = NULL;
	udp_cmd_instance.udpc.center.handshake_timeout_msec = config->handshake_timeout_sec * 1000; // sec -> ms

	// 初始化 udp 数据实例
	if (initialize(&udp_data_instance, memory_pool_cnt) != RET_OK) {
		return RET_FAIL;
	}

	snprintf(udp_data_instance.ser_info.ip, sizeof(udp_data_instance.ser_info.ip), "%s", config->center_ip);
	udp_data_instance.ser_info.port = config->center_data_port;
	udp_data_instance.ser_info.local_port = config->local_data_port;
	udp_data_instance.simplex = SIMPLEX;
	udp_data_instance.boradcast_buf = NULL;
	udp_data_instance.boradcast_buf_len = 0;
	udp_data_instance.deal_data_to_input_link = NULL;
	udp_data_instance.deal_data_to_output_link = default_deal_data_to_output_link;
	udp_data_instance.udpc.center.handshake_timeout_msec = config->handshake_timeout_sec * 1000; // sec -> ms

#if USE_SOCKET == 1
	if (init_socket(&udp_data_instance, config) != RET_OK) {
		return RET_FAIL;
	}
#endif
	return RET_OK;
}

void uninit_udp()
{
	destroy(&udp_cmd_instance);
	destroy(&udp_data_instance);
	return;
}
/**********************************************************************************************/
/***************************************构造、析构接口结束***************************************/
/**********************************************************************************************/
